"""Canny proposals -> dark-band verification -> subpixel lines -> projective center.

Tracking samples only four narrow strips. Searches always finish in the same frame;
ROI failure never consumes the right to attempt a full-frame reacquisition.
"""
import time
import cv2
import numpy as np
from .rect_geometry import (polygon_area as _polygon_area, order_quad as _order_quad,
    side_lengths as _side_lengths, gaussian_solve as _gaussian_solve,
    homography as _homography, rectangle_corners as _rectangle,
    sample_batch as _sample, homography_8x8, project_points, inverse_map_points,
    standard_circle, quad_is_valid, warp_front_view)
from .contour_locator import ContourLocator
from .kalman_filter import ConstantVelocityKalman2D


class RectangleCenterDetector:
    """有状态检测器：跨帧记住锁定框(_track)与平滑中心(_smooth)，稳态只做窄带精修。

    状态字段：
        _track       上一帧验证通过的外框（dict 候选）；None = 未锁定
        _prev_quad   再上一帧的候选四角（运动预测用）；失锁时清空
        _lost        连续未检出帧数（>0 = 刚失锁，禁用预测，避免用旧速度外推）
        _smooth      上一帧输出中心，用于 CENTER_ALPHA 一阶平滑（仅显示口径）
        _locked_th   上次成功的二值化阈值（下次优先复用，减少阈值轮换）
        _roi_retry   失锁后是否还允许一次"ROI 搜索"（用一次即置 False，防反复局部搜）
        _search_jobs 每 ROI 的阈值轮换进度（持续失败时跨帧轮换候选）
    每帧重置的临时量：_search_count / _threshold_count / _proposals / _deadline（时间预算）。
    """

    def __init__(self, config):
        self.cfg = config
        self.last = None
        self._smooth = None
        self._track = None
        self._prev_quad = None                                 # 上上帧的候选四角（运动预测用）
        self._lost = 0                                         # 连续未检出帧数（>0 时不预测）
        self._locked_th = None
        self._frame = 0
        self._kernel = np.ones((3, 3), np.uint8)               # 二值化后闭运算核
        # 四边上均匀分布的参数 t ∈ (0.12, 0.88)：避开四角（角点附近因透视最易模糊）。
        self._t = np.linspace(0.12, 0.88, config.EDGE_SAMPLES, dtype=np.float32)
        self._circle_cache = {}                                # 正视面圆周采样点缓存（按尺存）
        self._search_count = 0
        self._threshold_count = 0
        self._search_jobs = {}
        self._proposals = 0
        self._deadline = float("inf")
        self._roi_retry = True
        self.locator = ContourLocator(config)
        self._prev_timestamp = self._timestamp = self._frame_shape = None
        self._kalman = ConstantVelocityKalman2D(
            getattr(config, "KALMAN_PROCESS_PX", 40.), getattr(config, "KALMAN_MEASURE_PX", .9),
            getattr(config, "KALMAN_RESET_PX", 25.), getattr(config, "KALMAN_MAX_PREDICT_S", .25),
            getattr(config, "KALMAN_VELOCITY_LIMIT_PX_S", 6000.))

    def reset(self):
        self.last = self._smooth = self._track = self._prev_quad = None
        self._prev_timestamp = self._timestamp = self._frame_shape = None
        self._lost = 0
        self._roi_retry = True
        self._kalman.reset()

    def _valid_quad(self, q, shape, margin=1):
        return quad_is_valid(q, shape, margin, getattr(self.cfg, "MIN_EDGE_LENGTH_PX", 12.))

    def _layout(self, q, kind):
        """由四边形朝向推断物理长宽比，并给出该 kind 对应的物理矩形四角。

        horizontal：两条"横向边长之和" > 两条"纵向边长之和" → 长边水平 → w=长边(mm)。
        kind=="inner" 时物理矩形向内缩进一个胶带宽度 BLACK_FRAME_MM（对应内框角点）。
        """
        lengths = _side_lengths(q)
        horizontal = lengths[0] + lengths[2] > lengths[1] + lengths[3]
        w, h = ((self.cfg.TARGET_LONG_MM, self.cfg.TARGET_SHORT_MM) if horizontal
                else (self.cfg.TARGET_SHORT_MM, self.cfg.TARGET_LONG_MM))
        inset = self.cfg.BLACK_FRAME_MM if kind == "inner" else 0.0
        return w, h, _rectangle(w, h, inset)

    def _verify(self, gray, q, kind, diagnostic=False):
        """在正视面选取四边稀疏采样点并投影，不生成整幅 warp 图。

        每条边检查胶带厚度方向的暗带覆盖率与内侧亮区对比，允许局部反光；
        背景可以是黑色，不再强制外侧为白色。普通纸边及实心黑块会失败。
        """
        if diagnostic:
            self.last_diagnostic = {"reason": "invalid_geometry"}
        if not self._valid_quad(q, gray.shape):
            return None
        w, h, physical_quad = self._layout(q, kind)
        tape = float(self.cfg.BLACK_FRAME_MM)
        if tape <= 0 or 2 * tape >= min(w, h):
            raise ValueError("BLACK_FRAME_MM must be between 0 and half the short side")
        try:
            # 求 mm → px 单应：把物理矩形映射到图像候选四边形 q。
            rect_to_image = _homography(physical_quad, q)
            t = self._t
            # 沿每条边的法向采 5 个偏移：0.20/0.40/0.60×胶带宽 = 黑带内部三点（应为暗），
            # 1.22/1.45×胶带宽 = 越过内沿的亮面两点（应为亮）。暗亮对比即边证据。
            offsets = tape * np.array([0.20, 0.40, 0.60, 1.22, 1.45])
            p = np.empty((4, len(t), 5, 2), np.float64)
            # 四条边分别参数化：(边0 上/边1 右/边2 下/边3 左)，每条边沿 t 铺采样点。
            p[0, :, :, 0] = (tape + t * (w - 2 * tape))[:, None]
            p[0, :, :, 1] = offsets
            p[1, :, :, 0] = w - offsets
            p[1, :, :, 1] = (tape + t * (h - 2 * tape))[:, None]
            p[2, :, :, 0] = (tape + (1 - t) * (w - 2 * tape))[:, None]
            p[2, :, :, 1] = h - offsets
            p[3, :, :, 0] = offsets
            p[3, :, :, 1] = (tape + (1 - t) * (h - 2 * tape))[:, None]
            mapped = project_points(rect_to_image, p.reshape(-1, 2)).reshape(p.shape)
        except (ValueError, np.linalg.LinAlgError):
            return None
        gh, gw = gray.shape
        # 任何采样点落到图像外 → 该候选不可信（中心也无法可靠外推）。
        if (mapped[..., 0].min() < 0 or mapped[..., 1].min() < 0 or
                mapped[..., 0].max() >= gw - 1 or mapped[..., 1].max() >= gh - 1):
            return None
        samples = _sample(gray, mapped).astype(np.float32)
        # 每条边、每个 t：暗带取三点最大值（黑带里最亮的那个仍应暗），亮面取两点中位数。
        dark = np.max(samples[:, :, :3], axis=2)
        bright = np.median(samples[:, :, 3:], axis=2)
        delta = bright - dark
        # "该证据成立" = 对比度达标 且 暗带确实比亮面暗（比值约束，排除两侧都亮的纸边）。
        coverage = ((delta >= self.cfg.MIN_EDGE_CONTRAST) &
                    (dark <= bright * self.cfg.MAX_DARK_BRIGHT_RATIO)).mean(axis=1)
        edge_contrast = np.median(delta, axis=1)
        if diagnostic:
            self.last_diagnostic = dict(reason="dark_band_gates", coverage=coverage.tolist(),
                contrast=edge_contrast.tolist(), dark=np.median(dark,axis=1).tolist(),
                inside=np.median(bright,axis=1).tolist())
        # 至少三边的大部分采样成立，第四边也必须有证据，不接受凭亮中心猜框。
        if (np.count_nonzero(coverage >= self.cfg.MIN_EDGE_COVERAGE) < 3 or
                np.median(edge_contrast) < self.cfg.MIN_EDGE_CONTRAST):
            return None
        # Missing-edge exemption needs three strong sides, never a lone bright stripe.
        if coverage.min() < self.cfg.WEAK_EDGE_COVERAGE and np.sort(coverage)[1] < getattr(self.cfg, "STRONG_EDGE_COVERAGE", .85):
            return None
        med_dark, med_bright = np.median(dark, axis=1), np.median(bright, axis=1)
        rows = [dict(dark=float(med_dark[i]), inside=float(med_bright[i]),
                     contrast=float(edge_contrast[i]), coverage=float(coverage[i])) for i in range(4)]
        outer = project_points(rect_to_image, _rectangle(w, h)).astype(np.float32)
        if not self._valid_quad(outer, gray.shape):
            return None
        # 返回候选：外框(图像)、内框(胶带内沿)、mm→px 单应、四边剖面、质量分。
        # 打分 = 中位对比度 + 60×平均覆盖率，用于多候选时择优（分数越高越像真靶纸）。
        return dict(quad=q, outer_quad=outer,
                    inner_quad=project_points(rect_to_image, _rectangle(w, h, tape)).astype(np.float32),
                    kind=kind, rect_to_image=rect_to_image, width_mm=w, height_mm=h,
                    edge_profile=rows, contrast=float(np.median(edge_contrast)),
                    min_contrast=float(edge_contrast.min()), coverage=coverage,
                    score=float(np.median(edge_contrast) + 60 * coverage.mean()),
                    rank=float(np.median(edge_contrast) + 60 * coverage.mean() + 40 * coverage.min()))

    def _refine(self, gray, q, kind, radius):
        """当前帧四条灰度窄带 → 有方向的边缘跳变 → fitLine → 四角交点。

        与 _verify 的区别：_verify 只判"像不像"，_refine 才真正**重新定位**每条边。
        步骤：沿四边已有点 (base) 的法向 ±radius 采样灰度 → 求有向梯度 → 每列取
        "最近预测位置"的跳变（offset 越小越优先，防止误锁到内侧印刷圆或另一侧胶带边）
        → 用 fitLine 拟合四条直线 → 相邻直线两两求交 = 亚像素四角。
        精修结果偏离原 q 超过 radius*2.8 或四角体检不过 → 判失败（返回 None）。
        """
        q = np.asarray(q, np.float32)
        nxt = np.roll(q, -1, axis=0)
        e = nxt - q
        lengths = np.linalg.norm(e, axis=1)
        if lengths.min() < 10:
            return None
        normal = np.column_stack((-e[:, 1], e[:, 0])) / lengths[:, None]   # 各边外/内法向
        base = q[:, None, :] * (1 - self._t[None, :, None]) + nxt[:, None, :] * self._t[None, :, None]
        offsets = np.arange(-radius, radius + 1, dtype=np.float32)
        # 每条边、每个 t、每个法向偏移 → 采样点网格 (4, len(t), 2r+1, 2)。
        points = base[:, :, None, :] + normal[:, None, None, :] * offsets[None, None, :, None]
        gh, gw = gray.shape
        inside = ((points[..., 0] >= 1) & (points[..., 0] < gw - 1) &
                  (points[..., 1] >= 1) & (points[..., 1] < gh - 1)).all(axis=2)
        values = _sample(gray, points).astype(np.float32)
        gradient = values[:, :, 2:] - values[:, :, :-2]                    # 中心差分近似梯度
        if kind == "outer":
            gradient *= -1                                                 # 外框要求暗→亮方向相反
        # 近预测边缘优先，避免捕获靶纸内部印刷圆或另一侧的胶带边缘。
        penalty = np.abs(offsets[1:-1])[None, None, :] * 0.45              # 偏离预测位置的惩罚
        indices = np.argmax(gradient - penalty, axis=2)                    # 每股取最强跳变
        strength = np.take_along_axis(gradient, indices[..., None], axis=2)[..., 0]
        selected = offsets[indices + 1]
        lines = []
        refitted = 0
        for i in range(4):
            good = inside[i] & (strength[i] >= self.cfg.MIN_EDGE_CONTRAST * 0.7)   # 有效采样
            if good.sum() < max(5, len(self._t) * self.cfg.WEAK_EDGE_COVERAGE):
                # One missing/glared edge may retain its input line; three must be refitted.
                lines.append(np.cross(np.r_[q[i], 1.], np.r_[nxt[i], 1.]))
                continue
            shifts = selected[i, good]
            med = float(np.median(shifts))
            # 去掉偏离中位数过大的离群跳变（反光/杂物），再确认仍够点。
            good &= np.abs(selected[i] - med) <= max(2.5, float(np.median(np.abs(shifts - med))) * 3)
            if good.sum() < 5:
                return None
            samples = base[i, good] + selected[i, good, None] * normal[i]
            # 用鲁棒 Huber 回归拟合该边直线；fitLine 返回 (vx, vy, x0, y0)。
            vx, vy, x, y = cv2.fitLine(samples, cv2.DIST_HUBER, 0, 0.01, 0.01).reshape(4)
            # 直线 ax+by+c=0。
            lines.append(np.array([-vy, vx, vy * x - vx * y], np.float64))
            refitted += 1
        if refitted < 3:
            return None
        lines = np.asarray(lines)
        # 相邻两条直线（roll 1 使其错位一位）叉积求交点：交点齐次坐标 = 两直线系数叉积。
        crosses = np.cross(np.roll(lines, 1, axis=0), lines)
        if np.any(np.abs(crosses[:, 2]) < 1e-8):
            return None                                                    # 有平行边 → 退化
        refined = (crosses[:, :2] / crosses[:, 2:]).astype(np.float32)
        if (np.linalg.norm(refined - q, axis=1).max() > radius * 2.8 or
                not self._valid_quad(refined, gray.shape)):
            return None                                                    # 跑太远或形状不合法
        return refined

    def _four_edges_complete(self, candidate):
        return (min(candidate['coverage']) >= self.cfg.MIN_EDGE_COVERAGE and
                candidate['contrast'] >= self.cfg.MIN_EDGE_CONTRAST)

    def _rescue(self, gray, q, kind):
        radius = getattr(self.cfg, 'REFINE_RADIUS_WIDE', 20)
        if radius <= self.cfg.REFINE_RADIUS:
            return None
        q = self._refine(gray, q, kind, radius)
        candidate = self._verify(gray, q, kind) if q is not None else None
        # These two extra gates are ONLY for wide recovery; keep normal contrast sensitivity.
        if candidate is not None and candidate['contrast'] >= getattr(self.cfg, 'RECOVER_MIN_CONTRAST', 80) and max(r['dark'] for r in candidate['edge_profile']) <= getattr(self.cfg, 'RECOVER_MAX_DARK', 100):
            candidate['recovered'] = True
            return candidate
        return None

    def _accept(self, gray, q, kind):
        return self._verify(gray, q, kind) or self._rescue(gray, q, kind)

    def _finalize(self, gray, candidate, iterations):
        for _ in range(iterations):
            q = self._refine(gray, candidate['quad'], candidate['kind'], self.cfg.REFINE_RADIUS)
            precise = self._verify(gray, q, candidate['kind']) if q is not None else None
            if precise is None:
                break
            shift = np.linalg.norm(q-candidate['quad'], axis=1).max()
            candidate = precise
            if shift < .2:
                break
        return candidate

    def _search(self, gray, roi=None):
        self._search_count += 1
        cap = None if getattr(self.cfg, 'SEARCH_EXHAUSTIVE', True) else self.cfg.SEARCH_MAX_PROPOSALS
        # A local cap cannot veto the required full search. Counters still accumulate.
        proposals_at_start = self._proposals
        best = None
        for level in (0, 1):
            candidates = self.locator.locate(gray, roi, level)
            self._mask_count += self.locator.last_stats['masks']
            for q, kind in candidates:
                if cap is not None and self._proposals-proposals_at_start >= cap:
                    break
                self._proposals += 1
                candidate = self._accept(gray, q, kind)
                if candidate is None:
                    continue
                if self._four_edges_complete(candidate) and not candidate.get('recovered'):
                    return self._finalize(gray, candidate, getattr(self.cfg, 'REFINE_ITERATIONS', 2))
                if best is None or candidate['rank'] > best['rank']:
                    best = candidate
        return self._finalize(gray, best, getattr(self.cfg, 'REFINE_ITERATIONS', 2)) if best else None

    def _continuous_time(self, newer, older):
        return older is not None and np.isfinite([newer, older]).all() and 0 < newer-older <= getattr(self.cfg, 'PREDICT_MAX_GAP_S', .25)

    def _motion_delta(self, timestamp=None):
        timestamp = time.perf_counter() if timestamp is None else timestamp
        if not getattr(self.cfg, 'ENABLE_MOTION_PREDICT', True) or self._prev_quad is None or self._track is None or self._lost:
            return None
        if not self._continuous_time(timestamp, self._timestamp) or not self._continuous_time(self._timestamp, self._prev_timestamp):
            return None
        delta = (self._track['quad']-self._prev_quad)*(timestamp-self._timestamp)/(self._timestamp-self._prev_timestamp)
        step, cap = np.linalg.norm(delta, axis=1).max(), getattr(self.cfg, 'PREDICT_MAX_PX', 60.)
        if cap > 0 and step > cap:
            delta *= cap/step
        return delta

    def _filter_center(self, raw, timestamp):
        if getattr(self.cfg, 'KALMAN_ENABLE', True):
            self._smooth = self._kalman.update(raw, timestamp)
        else:
            alpha = self.cfg.CENTER_ALPHA
            if self._smooth is None or alpha <= 0 or np.linalg.norm(raw-self._smooth) > self.cfg.FILTER_RESET_PX or not self._continuous_time(timestamp, self._timestamp):
                self._smooth = raw.copy()
            else:
                self._smooth += alpha*(raw-self._smooth)
        return self._smooth

    @staticmethod
    def _midline_mm(profile, extent_mm):
        profile = np.asarray(profile, float)
        low, high = np.percentile(profile, [10, 65])
        if high-low < 5:
            raise ValueError('front view lacks dark-band contrast')
        threshold = (low+high)/2
        n = len(profile)
        # Only the first/last third can contain the tape inner edges, not printed rings.
        def crossing(indices, rising):
            for i in indices:
                a, b = profile[i:i+2]
                if (a <= threshold < b) if rising else (a >= threshold > b):
                    return i+(threshold-a)/(b-a)
            raise ValueError('front-view edge missing')
        left = crossing(range(n//3), True)
        right = crossing(range(n-2, 2*n//3, -1), False)
        return (left+right)*.5*extent_mm/(n-1)

    def _center(self, gray, candidate):
        w, h, matrix = candidate['width_mm'], candidate['height_mm'], candidate['rect_to_image']
        raw = project_points(matrix, [[w/2, h/2]])[0]
        front = int(getattr(self.cfg, 'FRONT_VIEW_SHORT_PX', 0))
        if front > 0:
            view = warp_front_view(gray, matrix, w, h, front)
            candidate['front_view'] = view
            try:
                center_mm = [self._midline_mm(np.median(view[view.shape[0]//3:2*view.shape[0]//3], axis=0), w),
                             self._midline_mm(np.median(view[:, view.shape[1]//3:2*view.shape[1]//3], axis=1), h)]
                center = project_points(matrix, [center_mm])[0]
                error = float(np.linalg.norm(center-raw))
                candidate.update(front_view_center=center, front_view_error_px=error,
                                 geometry_consistent=error <= getattr(self.cfg, 'FRONT_VIEW_MAX_ERROR_PX', 5.))
            except ValueError:
                candidate.update(front_view_center=None, geometry_consistent=False)
        return raw

    def process(self, gray, timestamp=None):
        started = time.perf_counter()
        timestamp = started if timestamp is None else float(timestamp)
        g = np.asarray(gray)
        if g.ndim == 3 and g.shape[2] == 1:
            g = g[:, :, 0]
        if g.ndim != 2 or g.dtype != np.uint8:
            raise ValueError('process expects a uint8 grayscale image')
        if not np.isfinite(timestamp):
            raise ValueError('non-finite timestamp')
        if self._frame_shape != g.shape:
            self.reset()
            self._frame_shape = g.shape
        self._search_count = self._mask_count = self._proposals = 0
        candidate, stage = None, 'full_search'
        if g.size == 0:
            self.reset()
            return dict(valid=False, stage='empty_frame', masks=0, search_count=0, proposals=0, detector_ms=0.)
        delta = self._motion_delta(timestamp)
        if self._track is not None:
            kind = self._track['kind']
            guesses = [self._track['quad']+delta] if delta is not None else []
            guesses.append(self._track['quad'])
            for guess in guesses:
                q = self._refine(g, guess, kind, self.cfg.TRACK_RADIUS)
                candidate = self._verify(g, q, kind) if q is not None else None
                if candidate is not None:
                    stage = 'track'
                    break
        if candidate is None and self._track is not None and self._roi_retry:
            q = self._track['outer_quad'] + (delta if delta is not None else 0)
            lo, hi = q.min(axis=0), q.max(axis=0)
            pad = np.maximum((hi-lo)*self.cfg.ROI_PADDING, 24)
            lo = np.maximum(0, np.floor(lo-pad)).astype(int)
            hi = np.minimum([g.shape[1], g.shape[0]], np.ceil(hi+pad)).astype(int)
            self._roi_retry = False
            candidate = self._search(g, tuple(map(int, (*lo, *(hi-lo)))))
            stage = 'roi_search'
        # Never use elif: ROI failure must still attempt a complete full-frame search.
        if candidate is None:
            candidate, stage = self._search(g), 'full_search'
        if candidate is None:
            self._smooth = self._prev_quad = self._prev_timestamp = None
            self._kalman.reset()
            self._lost += 1
            result = dict(valid=False, stage='no_black_frame')
        else:
            previous = self._track
            raw = self._center(g, candidate)
            center = self._filter_center(raw, timestamp)
            self._prev_quad = self._prev_timestamp = None
            if previous is not None and not self._lost and previous['kind'] == candidate['kind'] and self._continuous_time(timestamp, self._timestamp):
                q = previous['quad']
                if stage != 'track':
                    q = min((np.roll(q, i, axis=0) for i in range(4)), key=lambda p: np.sum((p-candidate['quad'])**2))
                self._prev_quad, self._prev_timestamp = q.copy(), self._timestamp
            self._track, self._timestamp, self._lost, self._roi_retry = candidate, timestamp, 0, True
            key = (candidate['width_mm'], candidate['height_mm'], self.cfg.CIRCLE_RADIUS_MM, self.cfg.CIRCLE_SAMPLES)
            if key not in self._circle_cache:
                self._circle_cache[key] = standard_circle(*key)
            result = dict(candidate)
            result.update(valid=True, stage=stage, u=float(center[0]), v=float(center[1]),
                          raw_u=float(raw[0]), raw_v=float(raw[1]), quad=candidate['outer_quad'],
                          detected_quad=candidate['quad'], outer_inferred=candidate['kind']=='inner',
                          circle_points=project_points(candidate['rect_to_image'], self._circle_cache[key]))
        result.update(threshold=None, search_count=self._search_count, masks=self._mask_count,
                      thresholds=self._mask_count, proposals=self._proposals,
                      detector_ms=(time.perf_counter()-started)*1000)
        self.last = result
        return result


def draw_result(gray, result, show_width=480, top_offset=0):
    """缩小后再转 BGR 和描边，输出坐标仍为相机原图坐标。

    top_offset：屏幕上方的文字整体下移量（像素）。激光读数面板占用左上角时由
    main.py 传 vision.laser_spot.PANEL_H，避免两处文字叠在一起。

    绘制元素（均按 scale 换算到缩小图坐标，避免在原始分辨率上画）：
        · 绿色外框 quad、青色内框 inner_quad
        · 洋红四角圆点 + "P1..P4 (x,y)" 标签（原图坐标）
        · 蓝色靶心十字 + "(u,v) stage 耗时" 文字
    返回 BGR 画布，交由 _Preview.render 再 letterbox 到屏幕画布。
    """
    g = np.asarray(gray).squeeze()
    scale = min(1.0, float(show_width) / g.shape[1])        # 原图 → 缩小图 的比例
    if scale < 1:
        g = cv2.resize(g, (int(round(g.shape[1] * scale)), int(round(g.shape[0] * scale))))
    canvas = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    if not result.get("valid"):
        # 无效：只提示一行红字，不画任何框（过期结果已在 main._display_result 降级）。
        cv2.putText(canvas, "NO BLACK RECTANGLE", (8, 24 + top_offset), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 255), 1)
        return canvas
    q = np.rint(result["quad"] * scale).astype(np.int32)          # 外框（缩小图坐标）
    iq = np.rint(result["inner_quad"] * scale).astype(np.int32)   # 内框（缩小图坐标）
    cv2.polylines(canvas, [q], True, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.polylines(canvas, [iq], True, (0, 220, 220), 1)
    for i, (x, y) in enumerate(result["quad"]):
        p = tuple(q[i])
        cv2.circle(canvas, p, 4, (255, 0, 255), -1)               # 洋红实心点
        # 标签用**原图坐标**（x,y），便于与日志/回算对照；位置做边界夹取防出画。
        label = "P%d (%d,%d)" % (i + 1, round(float(x)), round(float(y)))
        tw = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .36, 1)[0][0]
        pos = (max(0, min(int(p[0]) + 5, canvas.shape[1] - tw - 1)),
               max(12, min(int(p[1]) - 7, canvas.shape[0] - 5)))
        cv2.putText(canvas, label, pos, cv2.FONT_HERSHEY_SIMPLEX, .36, (255, 255, 0), 1, cv2.LINE_AA)
    c = tuple(np.rint(np.array([result["u"], result["v"]]) * scale).astype(int))
    cv2.drawMarker(canvas, c, (255, 0, 0), cv2.MARKER_CROSS, 14, 2)   # 蓝色靶心十字
    cv2.putText(canvas, "(%d,%d) %s %.1fms" % (round(result["u"]), round(result["v"]),
                result["stage"], result["detector_ms"]), (8, 22 + top_offset), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 220, 0), 1)
    return canvas
