"""黑环/亮面双候选、稀疏透视验证、四边直线跟踪。仅依赖 cv2/numpy。

搜索在缩小图上执行；精修和坐标输出始终使用当前帧的原始分辨率。
已锁定时只采样四条窄带，不运行全图 Canny、轮廓扫描或阈值遍历。

================================================================================
几何原理（为什么靶心能"算"出来）
================================================================================
靶纸四周贴 1.8cm 黑胶带形成一个矩形环，环的两个"内框角点"构成一个**平面四边形**。
相机斜看时该四边形在图像里是透视投影，但两条对角线的交点 = 靶心，这是**射影不变量**
（任意透视变换下，四边对角线的交点都映射到对应四边形的对角线交点）。因此：
    检测四边形四角 → 求对角线交点 = 靶心像素坐标 (u,v)，
无需解单应、也无需知道相机内外参。本模块即按此实现。

================================================================================
单帧处理链（process 的骨架，四段）
================================================================================
    ① 候选生成 _candidate_quads : 阈值化 → 轮廓树 → 取四边形轮廓，按内外框打标
    ② 搜索 _search              : 在缩小图上轮换阈值/候选，时间预算内做有限次验证
    ③ 四边验证 _verify          : 用"胶带厚度方向的暗带 + 内侧亮面"四边覆盖率判真伪
    ④ 亚像素精修 _refine        : 当前帧四条灰度窄带 → 有向边缘跳变 → fitLine → 四角交点
    末：把四角规整为外框，取正视面中心投影回原图 → (u,v)，并按 CENTER_ALPHA 做一阶平滑。

三种进入方式（process 里按上一帧是否锁定决定，越靠前越省算力）：
    track       已锁定 → 只沿四边窄带精修（不跑 Canny/轮廓/阈值遍历，稳态 ~1ms）
    roi_search  刚失锁 → 在上一帧外框外扩 ROI_PADDING 的窗口内搜索（一次）
    full_search 彻底丢失 → 在全图缩小图上多阈值扫描 + Canny 兜底
"""
import time

import cv2
import numpy as np


def _polygon_area(p):
    """带符号面积的 2 倍（>0 = 逆时针 CCW，<0 = 顺时针 CW）。用于把四角规整成统一方向。"""
    return float(cv2.contourArea(np.asarray(p, np.float32).reshape(-1, 1, 2), oriented=True))


def _order_quad(points):
    """把 4 个点规整为统一顺序：以质心为极点按极角排序 → 保证 CCW → 旋转到"左上角"开头。

    统一顺序是后续所有几何运算（边向量、对角线、单应四点对应）的前提。
    返回顺序语义：**[左上, 右上, 右下, 左下]**（逆时针），与 _side_lengths 的边 0/1/2/3 对应。
    """
    p = np.asarray(points, np.float32).reshape(4, 2)
    c = p.mean(axis=0)
    p = p[np.argsort(np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0]))]   # 按极角排序
    if _polygon_area(p) < 0:
        p = p[::-1]                                                  # 反了则翻转成 CCW
    # np.roll 到"x+y 最小"的点（即左上角）打头。
    return np.roll(p, -int(np.argmin(p.sum(axis=1))), axis=0).copy()


def _side_lengths(q):
    """四边长度 [边0, 边1, 边2, 边3]，边 i = 从点 i 指向点 i+1（roll -1 取下一个点）。"""
    return np.linalg.norm(np.roll(q, -1, axis=0) - q, axis=1)


def _gaussian_solve(a, b):
    """带部分主元的高斯消元；消去行向量化，避免 64 次 Python 行循环。

    a 为 n×n、b 为 n，返回解向量。主元过小（矩阵退化，如四点共线）抛 ValueError，
    由调用方捕获后判该候选无效 —— 不做伪解。
    """
    m = np.column_stack((np.asarray(a, np.float64), np.asarray(b, np.float64)))
    for col in range(len(m)):
        pivot = col + int(np.argmax(np.abs(m[col:, col])))    # 选列内绝对值最大的行做主元
        if abs(m[pivot, col]) < 1e-10:
            raise ValueError("degenerate quadrilateral")
        if pivot != col:
            m[[col, pivot]] = m[[pivot, col]]                 # 换行
        m[col] /= m[col, col]                                 # 归一化主元行
        factors = m[:, col].copy()
        factors[col] = 0.0
        m -= factors[:, None] * m[col]                        # 一次消去整列（行向量化）
    return m[:, -1]


def _homography(src, dst):
    """解 4 点对应的 3×3 单应 H（使 H·src ≡ dst）。

    做法：先对 src/dst **归一化**（平移质心到原点、缩放到量级 1）再建 8×8 线性方程，
    求得 H_n 后用 T_dst·H_n·T_src⁻¹ 还原 —— 归一化能显著提升大像素坐标下的数值稳定性。
    退化情形（任一边集合尺度过小/共线）抛 ValueError。
    """
    src, dst = np.asarray(src, np.float64), np.asarray(dst, np.float64)
    sc, dc = src.mean(axis=0), dst.mean(axis=0)
    ss, ds = float(np.max(np.abs(src - sc))), float(np.max(np.abs(dst - dc)))
    if min(ss, ds) < 1e-8:
        raise ValueError("zero-size rectangle")
    xy, uv = (src - sc) / ss, (dst - dc) / ds                 # 归一化坐标
    a = np.zeros((8, 8), np.float64)
    a[0::2, :3] = np.column_stack((xy, np.ones(4)))           # 偶数行：x 分量方程
    a[1::2, 3:6] = a[0::2, :3]                                # 奇数行：y 分量方程
    a[0::2, 6:] = -uv[:, :1] * xy                             # 第 3 列（透视项）
    a[1::2, 6:] = -uv[:, 1:] * xy
    v = _gaussian_solve(a, uv.reshape(-1))
    hn = np.append(v, 1.0).reshape(3, 3)                      # 归一化域下的 H
    ts = np.array([[1 / ss, 0, -sc[0] / ss], [0, 1 / ss, -sc[1] / ss], [0, 0, 1]])
    td_inv = np.array([[ds, 0, dc[0]], [0, ds, dc[1]], [0, 0, 1]])
    h = td_inv @ hn @ ts                                      # 还原到原坐标域
    return h / h[2, 2] if abs(h[2, 2]) > 1e-12 else h


def homography_8x8(src_quad, width_mm, height_mm):
    """把物理矩形 (0,0)-(w,h) 映射到图像四边形 src_quad 的单应（mm → px）。"""
    return _homography(src_quad, [[0, 0], [width_mm, 0],
                                 [width_mm, height_mm], [0, height_mm]])


def project_points(matrix, points):
    """齐次投影：把 (N,2) 点经 3×3 矩阵映射，做透视除法得 (N,2)。落无穷远点抛错。"""
    p = np.asarray(points, np.float64).reshape(-1, 2)
    z = np.column_stack((p, np.ones(len(p)))) @ np.asarray(matrix).T
    if np.any(np.abs(z[:, 2]) < 1e-9):
        raise ValueError("point at projective infinity")
    return z[:, :2] / z[:, 2:]


def inverse_map_points(h, points):
    """单应逆映射（px → mm）；不做透视除法前的越界检查，由调用方负责。"""
    return project_points(np.linalg.inv(h), points)


def standard_circle(width_mm, height_mm, radius_mm, samples=72):
    """正视面坐标下、以矩形中心为圆心、半径 radius_mm 的等分圆采样点（(samples,2)）。

    用途：把该圆逆映射回原图，用于可视化/交叉校验中心；**圆心本身直接取矩形中心投影**，
    不用圆周平均（见 process 内注释）。
    """
    a = np.arange(samples) * (2 * np.pi / samples)
    return np.array([width_mm / 2, height_mm / 2]) + radius_mm * np.column_stack((np.cos(a), np.sin(a)))


def _rectangle(w, h, inset=0.0):
    """物理矩形四角（mm），inset>0 时向内缩进（用于取胶带内沿 = 内框）。顺序 LL→LR→UR→UL。"""
    return np.array([[inset, inset], [w - inset, inset],
                     [w - inset, h - inset], [inset, h - inset]], np.float64)


def _sample(gray, points):
    """一次 OpenCV remap 双线性采样整批点，越界由调用方拒绝。

    相比逐点 Python 循环，remap 一次调用即可采样成百上千个点（四边验证/精修的关键提速）。
    borderMode=REPLICATE 保证边缘点也能返回数值（越界判据在调用方单独做）。
    """
    shape = points.shape[:-1]
    p = points.reshape(-1, 2).astype(np.float32)
    values = cv2.remap(gray, p[:, 0].reshape(1, -1), p[:, 1].reshape(1, -1),
                       cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    return values.reshape(shape)


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
        self._track_timestamp = self._prev_timestamp = None
        self._frame_shape = None
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

    def reset(self):
        """清空图像坐标与运动历史；换分辨率、断流后不能复用旧框。"""
        self.last = self._smooth = self._track = self._prev_quad = None
        self._track_timestamp = self._prev_timestamp = None
        self._frame_shape = None
        self._lost = self._frame = 0
        self._locked_th = None
        self._roi_retry = True
        self._search_jobs.clear()
        return self

    def _continuous_time(self, timestamp):
        if timestamp is None or self._track_timestamp is None:
            return timestamp is None and self._track_timestamp is None
        gap = timestamp - self._track_timestamp
        return 0 < gap <= float(getattr(self.cfg, "PREDICT_MAX_GAP_S", 0.25))

    def _valid_quad(self, q, shape, margin=1):
        """四边形体检：不越界、凸、边长下限、无明显退化尖角。

        允许任意透视（不要求接近直角/矩形），只排除两类坏形状：
          · 贴边/出界：角点落在图像边缘 margin 内 → 采样会越界、中心不可信；
          · 退化尖角：相邻边几乎平行（sin 很小）→ 四角拟合不稳定。
        """
        h, w = shape
        if (q[:, 0].min() < margin or q[:, 1].min() < margin or
                q[:, 0].max() >= w - margin or q[:, 1].max() >= h - margin):
            return False
        if not cv2.isContourConvex(np.asarray(q, np.float32).reshape(-1, 1, 2)):
            return False
        e = np.roll(q, -1, axis=0) - q
        lengths = np.linalg.norm(e, axis=1)
        if lengths.min() < 12:                                 # 最短边至少 12px
            return False
        # 透视图不要求四角接近 90 度，只排除退化尖角和过扁的四边形。
        # 相邻边向量叉积 / (边长积) = sin(夹角)，要求每个角 > ~11.5°。
        sine = np.abs(e[:, 0] * np.roll(e[:, 1], -1) -
                      e[:, 1] * np.roll(e[:, 0], -1)) / (lengths * np.roll(lengths, -1))
        return bool(sine.min() > 0.20)

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

    def _verify(self, gray, q, kind):
        """在正视面选取四边稀疏采样点并投影，不生成整幅 warp 图。

        每条边检查胶带厚度方向的暗带覆盖率与内侧亮区对比，允许局部反光；
        背景可以是黑色，不再强制外侧为白色。普通纸边及实心黑块会失败。
        """
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
        # 至少三边的大部分采样成立，第四边也必须有证据，不接受凭亮中心猜框。
        if (np.count_nonzero(coverage >= self.cfg.MIN_EDGE_COVERAGE) < 3 or
                coverage.min() < self.cfg.WEAK_EDGE_COVERAGE or
                np.median(edge_contrast) < self.cfg.MIN_EDGE_CONTRAST):
            return None
        rows = [dict(dark=float(np.median(dark[i])), inside=float(np.median(bright[i])),
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
                    score=float(np.median(edge_contrast) + 60 * coverage.mean()))

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
        for i in range(4):
            good = inside[i] & (strength[i] >= self.cfg.MIN_EDGE_CONTRAST * 0.7)   # 有效采样
            if good.sum() < max(5, len(self._t) * self.cfg.WEAK_EDGE_COVERAGE):
                return None                                                # 证据太少，整框弃用
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

    def _candidate_quads(self, mask):
        """用轮廓树的孔/黑环关系代替候选两两组合。

        输入是二值掩膜（暗区为前景）。输出 [(quad, kind), ...]：
          · 借 findContours 的 RETR_TREE 层级：黑胶带环 = 一个"孔"（有子级）；
            深度奇偶决定 kind —— 偶数深度=outer(外框)，奇数=inner(内框)。
          · 用**包围框/面积**先廉价预筛（剔过小、过扁），只对最大若干个轮廓做 approxPolyDP，
            控制昂贵的多边形逼近次数。
          · 排除"实心黑块"（outer 却没有亮孔），因为靶纸黑环中心必是亮的。
        返回排序把 inner 排前：亮面孔比与背景粘连的黑环更稳定。
        """
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        if hierarchy is None:
            return []
        hierarchy = hierarchy[0]
        h, w = mask.shape
        candidates = []
        # 先用廉价的包围框/面积筛选，仅拟合最大的少数轮廓。
        eligible = []
        # findContours 本身是原子调用；后续 Python 遍历只看最大的有限个轮廓。
        ids = sorted(range(len(contours)), key=lambda i: abs(cv2.contourArea(contours[i])),
                     reverse=True)[:self.cfg.MAX_CONTOURS]
        for i in ids:
            cnt = contours[i]
            x, y, cw, ch = cv2.boundingRect(cnt)
            if min(cw, ch) < 15 or cw * ch < w * h * self.cfg.MIN_AREA_RATIO:
                continue                                   # 太小/太扁：不是候选框
            area = abs(cv2.contourArea(cnt))
            if w * h * self.cfg.MIN_AREA_RATIO <= area <= w * h * self.cfg.MAX_AREA_RATIO:
                eligible.append((area, i))
        for area, i in sorted(eligible, reverse=True)[:self.cfg.MAX_CANDIDATES]:
            cnt = contours[i]
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, self.cfg.APPROX_EPSILON * peri, True)   # 多边形逼近
            # 不使用凸包修补任意背景：凸包可把桌面阴影拼成错误大四边形。
            if len(approx) != 4 or not cv2.isContourConvex(approx):
                continue                                   # 只接受凸四边形
            q = _order_quad(approx)
            if not self._valid_quad(q, mask.shape):
                continue
            parent, depth = int(hierarchy[i][3]), 0        # 沿父链数深度
            while parent >= 0:
                depth += 1
                parent = int(hierarchy[parent][3])
            kind = "inner" if depth % 2 else "outer"       # 奇偶定内外框
            if kind == "outer" and hierarchy[i][2] < 0:
                continue  # 实心黑块没有亮孔。
            candidates.append((q, kind))
        # 亮面孔比与背景粘连的黑环更稳定。
        return sorted(candidates, key=lambda c: c[1] != "inner")

    def _motion_delta(self, timestamp=None):
        """上两帧四角的**逐角位移**（图像坐标）；无历史或刚失锁则 None。

        逐角位移天然同时包含平移、缩放与旋转，直接加在"上一帧四角"上即为匀速外推。
        只在**连续两帧都成功**（_lost == 0）时可用：失锁后速度已不可信，宁可退回原地精修。
        位移限幅 PREDICT_MAX_PX 防止噪点/误检把预测甩到画面外。
        """
        if not getattr(self.cfg, "ENABLE_MOTION_PREDICT", True):
            return None
        if self._prev_quad is None or self._track is None or self._lost:
            return None
        delta = np.asarray(self._track["quad"], np.float32) - np.asarray(self._prev_quad, np.float32)
        if timestamp is not None:
            if self._prev_timestamp is None or not self._continuous_time(timestamp):
                return None
            interval = self._track_timestamp - self._prev_timestamp
            if interval <= 0:
                return None
            # 最新帧线程会丢弃旧帧；按采图间隔外推，不能把每次回调当作等间隔视频帧。
            delta = delta * ((timestamp - self._track_timestamp) / interval)
        step = float(np.linalg.norm(delta, axis=1).max())
        cap = float(getattr(self.cfg, "PREDICT_MAX_PX", 60.0))
        if cap > 0 and step > cap:
            delta = delta * (cap / step)
        return delta

    def _search(self, gray, roi=None):
        """在缩小图上按序穷尽阈值并验证；**单帧内跑完**（默认），重捕获恒为 1 帧。

        关键设计（性能与鲁棒性的平衡）：
          · 缩小到 SEARCH_WIDTH 宽再阈值化，全图扫描代价大幅下降；坐标末了乘回原分辨率。
          · 阈值列表 = [上次成功阈值] + [Otsu] + DARK_THRESHOLDS，**在同一帧内按序尝试**，
            直到找到为止；全部阈值试完再用 Canny 轮廓兜底（最贵，只在彻底找不到时用）。
          · ★ 默认单帧跑完（v0.5.2，SEARCH_SPLIT_FRAMES=False）：板端搜索远慢于 PC，
            若跨帧拆分，"重捕获"会从 1 帧退化成十几帧（实测即"重捕获很慢"）。故默认不拆。
            仅当 SEARCH_SPLIT_FRAMES=True 时才按 SEARCH_BUDGET_MS 软预算跨帧续扫
            （job["index"] / job["pending"] 留到下帧），防极慢板卡死单帧。
          · ★ 恢复单帧穷尽（v0.5.1）：v0.4.3 曾改成"每帧只轮换 1 个阈值、只验 1 个候选"，
            实测丢锁后要 11+ 帧才重捕获（板端更慢），大角度/移动场景肉眼即"丢框"。
        找到即锁定 _locked_th 并清空该 ROI 的待验证队列，下次直接复用。
        """
        self._search_count += 1
        split = bool(getattr(self.cfg, "SEARCH_SPLIT_FRAMES", False))
        x, y, w, h = roi if roi is not None else (0, 0, gray.shape[1], gray.shape[0])
        crop = gray[y:y + h, x:x + w]
        scale = min(1.0, self.cfg.SEARCH_WIDTH / float(w))
        sw, sh = max(32, int(round(w * scale))), max(24, int(round(h * scale)))
        small = cv2.resize(crop, (sw, sh), interpolation=cv2.INTER_AREA) if scale < 1 else crop
        sx, sy = w / float(sw), h / float(sh)              # 缩小图 → 原图 的坐标比例
        # 阈值候选：优先复用上次成功值，再并入 Otsu 与固定档位（去掉与已有值太接近的）。
        thresholds = [self._locked_th] if self._locked_th is not None else []
        otsu, _ = cv2.threshold(small, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        for th in (int(otsu),) + tuple(self.cfg.DARK_THRESHOLDS):
            if not any(abs(th - t) < 6 for t in thresholds):
                thresholds.append(th)
        # 跨帧进度：只缓存候选几何与下一档下标，不保存旧图像或旧验证结果。
        key = tuple(roi) if roi is not None else "full"
        job = self._search_jobs.setdefault(key, dict(index=0, pending=[], threshold=None))
        if len(self._search_jobs) > 4:
            self._search_jobs = {key: job}                 # 最多只留 4 个 ROI 的进度
        pending = job["pending"]
        while True:
            if not pending:
                # 取下一档阈值；下标超过 len 表示"全部阈值 + Canny 兜底"都已试过 → 本帧无果。
                if job["index"] > len(thresholds):
                    job["index"] = 0
                    return None
                # 软预算（仅 SEARCH_SPLIT_FRAMES=True 生效）：本帧已建过至少一档阈值且已超时
                # → 停手，剩余留给下一帧续扫（至少保证第一档一定跑完，防原子调用稍慢不推进）。
                if split and self._threshold_count and time.perf_counter() >= self._deadline:
                    return None
                index = job["index"]
                job["index"] += 1
                if index < len(thresholds):
                    th = thresholds[index]
                    job["threshold"] = th
                    self._threshold_count += 1
                    _, mask = cv2.threshold(small, th, 255, cv2.THRESH_BINARY_INV)   # 暗区为前景
                    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)     # 闭运算补断线
                    # 候选坐标从缩小图坐标乘回原图（+ROI 偏移）。
                    pending.extend((q * np.array([sx, sy], np.float32) + [x, y], kind)
                                   for q, kind in self._candidate_quads(mask))
                else:
                    job["threshold"] = None
                    # Canny 兜底：边缘图直接找四边形轮廓，内/外框都试。
                    edges = cv2.Canny(small, self.cfg.CANNY_LOW, self.cfg.CANNY_HIGH)
                    contours = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
                    for cnt in sorted(contours, key=cv2.contourArea, reverse=True)[:self.cfg.MAX_CANDIDATES]:
                        if cv2.contourArea(cnt) < sw * sh * self.cfg.MIN_AREA_RATIO:
                            continue
                        ap = cv2.approxPolyDP(cnt, self.cfg.APPROX_EPSILON * cv2.arcLength(cnt, True), True)
                        if len(ap) == 4 and cv2.isContourConvex(ap):
                            q = _order_quad(ap) * [sx, sy] + [x, y]
                            pending.extend(((q, "inner"), (q, "outer")))
                continue
            # 有候选：至少验证一个；之后一旦超出硬上限（或拆分模式下的软预算）就停手。
            if self._proposals and (self._proposals >= self.cfg.SEARCH_MAX_PROPOSALS or
                                    (split and time.perf_counter() >= self._deadline)):
                return None
            q, kind = pending.pop(0)
            self._proposals += 1
            verified = self._verify(gray, q, kind)         # 四边暗带验证
            if verified is None:
                continue
            refined = self._refine(gray, q, kind, self.cfg.REFINE_RADIUS)   # 亚像素精修
            if refined is not None:
                precise = self._verify(gray, refined, kind)                 # 精修后再验证
                if precise is not None:
                    verified = precise                    # 精修通过则用更准的结果
            self._locked_th = job["threshold"]             # 记住本次成功阈值
            job["pending"] = []
            job["index"] = 0
            return verified

    def _legacy_search(self, gray, roi=None):
        """保留可选线段恢复路径使用的完整搜索；常规帧不调用。"""
        x, y, w, h = roi if roi is not None else (0, 0, gray.shape[1], gray.shape[0])
        crop = gray[y:y + h, x:x + w]
        scale = min(1.0, self.cfg.SEARCH_WIDTH / float(w))
        sw, sh = max(32, int(round(w * scale))), max(24, int(round(h * scale)))
        small = cv2.resize(crop, (sw, sh), interpolation=cv2.INTER_AREA) if scale < 1 else crop
        sx, sy = w / float(sw), h / float(sh)
        thresholds = [self._locked_th] if self._locked_th is not None else []
        otsu, _ = cv2.threshold(small, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        for th in (int(otsu),) + tuple(self.cfg.DARK_THRESHOLDS):
            if not any(abs(th - t) < 6 for t in thresholds):
                thresholds.append(th)
        for th in thresholds:
            self._threshold_count += 1
            _, mask = cv2.threshold(small, th, 255, cv2.THRESH_BINARY_INV)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
            for q, kind in self._candidate_quads(mask):
                q = q * np.array([sx, sy], np.float32) + np.array([x, y], np.float32)
                verified = self._verify(gray, q, kind)
                if verified is None:
                    continue
                refined = self._refine(gray, q, kind, self.cfg.REFINE_RADIUS)
                if refined is not None:
                    precise = self._verify(gray, refined, kind)
                    if precise is not None:
                        verified = precise
                self._locked_th = th
                return verified
        # Canny 是失锁后的最后辅助，避免每帧全图计算梯度。
        edges = cv2.Canny(small, self.cfg.CANNY_LOW, self.cfg.CANNY_HIGH)
        contours = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:self.cfg.MAX_CANDIDATES]
        for cnt in contours:
            if cv2.contourArea(cnt) < sw * sh * self.cfg.MIN_AREA_RATIO:
                continue
            ap = cv2.approxPolyDP(cnt, self.cfg.APPROX_EPSILON * cv2.arcLength(cnt, True), True)
            if len(ap) != 4 or not cv2.isContourConvex(ap):
                continue
            q = _order_quad(ap) * [sx, sy] + [x, y]
            for kind in ("inner", "outer"):
                verified = self._verify(gray, q, kind)
                if verified is not None:
                    refined = self._refine(gray, q, kind, self.cfg.REFINE_RADIUS)
                    precise = self._verify(gray, refined, kind) if refined is not None else None
                    return precise or verified
        # 透明膜的反光可能把整条黑环切开，内孔也不再闭合。
        # 仅失锁时从长线段拼四边，并通过相同暗带验证，绝不凭亮中心猜框。
        if not getattr(self.cfg, "ENABLE_LINE_RECOVERY", False):
            return None
        line_edges = cv2.Canny(cv2.GaussianBlur(small, (5, 5), 0),
                               self.cfg.CANNY_LOW, self.cfg.CANNY_HIGH)
        return self._line_search(gray, line_edges, sx, sy, x, y)

    def _line_search(self, gray, edges, sx, sy, x, y):
        h, w = edges.shape
        segments = cv2.HoughLinesP(edges, 1, np.pi / 180, 25,
                                   minLineLength=max(25, int(min(w, h) * .16)), maxLineGap=12)
        if segments is None:
            return None
        p = segments.reshape(-1, 2, 2).astype(np.float64)
        delta = p[:, 1] - p[:, 0]
        lengths = np.linalg.norm(delta, axis=1)
        ids = np.argsort(lengths)[-64:]
        p, delta, lengths = p[ids], delta[ids], lengths[ids]
        lines = np.cross(np.column_stack((p[:, 0], np.ones(len(p)))),
                         np.column_stack((p[:, 1], np.ones(len(p)))))
        angles = np.arctan2(delta[:, 1], delta[:, 0]) % np.pi
        # 以长线段为种子，最多尝试三组主方向，兼容靶紙绕光轴旋转。
        tried, best = [], None
        for seed in np.argsort(lengths)[::-1]:
            angle = angles[seed]
            distance = lambda a, b: np.abs((a - b + np.pi / 2) % np.pi - np.pi / 2)
            if any(distance(angle, old) < np.deg2rad(20) for old in tried):
                continue
            tried.append(angle)
            groups = []
            for direction in (angle, (angle + np.pi / 2) % np.pi):
                mask = distance(angles, direction) < np.deg2rad(28)
                normal = np.array([-np.sin(direction), np.cos(direction)])
                positions = p.mean(axis=1) @ normal
                group = []
                for index in np.argsort(lengths)[::-1]:
                    if not mask[index]:
                        continue
                    if any(abs(positions[index] - positions[old]) < 5 for old in group):
                        continue
                    group.append(index)
                    if len(group) >= 8:
                        break
                pairs = []
                for ii, a in enumerate(group):
                    for b in group[ii + 1:]:
                        if abs(positions[a] - positions[b]) >= min(w, h) * .24:
                            pairs.append((lengths[a] + lengths[b], a, b))
                groups.append(sorted(pairs, reverse=True)[:6])
            for _, a, b in groups[0]:
                for _, c, d in groups[1]:
                    intersections = np.cross(lines[[a, b, b, a]], lines[[c, c, d, d]])
                    if np.any(np.abs(intersections[:, 2]) < 1e-8):
                        continue
                    q = _order_quad(intersections[:, :2] / intersections[:, 2:])
                    q = q * [sx, sy] + [x, y]
                    for kind in ("inner", "outer"):
                        check = self._verify(gray, q, kind)
                        if check is None:
                            continue
                        refined = self._refine(gray, q, kind, self.cfg.REFINE_RADIUS)
                        precise = self._verify(gray, refined, kind) if refined is not None else None
                        check = precise or check
                        if best is None or check["score"] > best["score"]:
                            best = check
            if best is not None or len(tried) >= 3:
                break
        return best

    def process(self, gray, timestamp=None):
        """单帧入口：按"track → roi_search → full_search"三级降级找框，输出中心与结果 dict。"""
        started = time.perf_counter()
        g = np.asarray(gray)
        if g.ndim == 3 and g.shape[2] == 1:
            g = g[:, :, 0]
        if g.ndim != 2 or g.dtype != np.uint8:
            raise ValueError("process expects a uint8 grayscale image")
        if min(g.shape) < 2:
            self.reset()
            self.last = dict(valid=False, stage="empty_frame", search_count=0,
                             thresholds=0, proposals=0, detector_ms=0.0)
            return self.last
        timestamp = None if timestamp is None else float(timestamp)
        if timestamp is not None and not np.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        if self._frame_shape != g.shape:
            self.reset()
            self._frame_shape = g.shape
        if self._track is not None and not self._continuous_time(timestamp):
            self._prev_quad = self._prev_timestamp = self._smooth = None
        self._frame += 1
        # 每帧重置的临时统计与时间预算（预算内没找到就留给下一帧继续）。
        self._search_count = self._threshold_count = 0
        self._proposals = 0
        self._deadline = started + self.cfg.SEARCH_BUDGET_MS / 1000.0
        candidate, stage = None, "search"
        # ---- ① track：上一帧已锁定 → 先用**运动预测位置**、再用上一帧位置做窄带精修 ----
        # 预测位置让窄带跟上快速移动（不必靠加大 TRACK_RADIUS 牺牲慢速稳定性）；
        # 预测失准（突然停止/变向）时自动回落到上一帧位置，两级都试。
        if self._track is not None:
            kind = self._track["kind"]
            delta = self._motion_delta(timestamp)
            guesses = []
            if delta is not None:
                guesses.append(self._track["quad"] + delta)
            guesses.append(self._track["quad"])
            for guess in guesses:
                q = self._refine(g, guess, kind, self.cfg.TRACK_RADIUS)
                if q is None:
                    continue
                verified = self._verify(g, q, kind)
                if verified is not None:
                    candidate, stage = verified, "track"
                    break
        # ---- ② roi_search：刚失锁 → 在上一帧外框（含预测位移）外扩的窗口内搜一次 ----
        if candidate is None and self._track is not None and self._roi_retry:
            q = np.asarray(self._track["outer_quad"], np.float32)
            delta = self._motion_delta(timestamp)
            if delta is not None:
                q = q + delta                                  # 窗口中心跟上预测位移
            lo, hi = q.min(axis=0), q.max(axis=0)
            pad = np.maximum((hi - lo) * self.cfg.ROI_PADDING, 24)         # 外扩量（至少 24px）
            size = np.array([g.shape[1], g.shape[0]])
            lo = np.clip(np.floor(lo - pad), 0, size).astype(int)
            hi = np.clip(np.ceil(hi + pad), 0, size).astype(int)
            if np.all(hi - lo >= 2):
                candidate = self._search(g, (int(lo[0]), int(lo[1]), int(hi[0] - lo[0]), int(hi[1] - lo[1])))
            stage = "roi_search"
            self._roi_retry = False                                        # 只允许一次 ROI 搜索
        if (candidate is None and
                (self._search_count == 0 or
                 (not getattr(self.cfg, "SEARCH_SPLIT_FRAMES", False) and
                  self._proposals < self.cfg.SEARCH_MAX_PROPOSALS))):
            # ---- ③ full_search：彻底丢失 → 全图多阈值 + Canny 兜底 ----
            candidate = self._search(g)
            stage = "full_search"
        if candidate is None:
            # 三路都没找到：清空平滑中心与预测历史，返回无效结果（不沿用旧框，避免"粘住"）。
            self._smooth = None
            self._lost += 1
            self._prev_quad = None
            self._prev_timestamp = None
            self.last = dict(valid=False, stage="no_black_frame", search_count=self._search_count,
                             thresholds=self._threshold_count, proposals=self._proposals,
                             detector_ms=(time.perf_counter() - started) * 1000)
            return self.last
        # ---- 找到：锁定、清进度、算中心 ----
        # 预测历史只在"连续两帧都成功"时可用：上上帧 = 旧 _track，失锁后清空避免用旧速度外推。
        self._prev_quad = None
        self._prev_timestamp = None
        if (self._track is not None and not self._lost and
                self._track["kind"] == candidate["kind"] and self._continuous_time(timestamp)):
            # 重搜会重新选“左上角”，旋转跨过该分界时编号可能循环移位。
            # 只匹配同一种内/外框的物理角，避免把胶带厚度或编号切换当作速度。
            previous = np.asarray(self._track["quad"], np.float32)
            if stage == "track":
                self._prev_quad = previous                 # 窄带精修保留编号，稳态无需匹配
            else:
                matches = np.stack([np.roll(previous, shift, axis=0) for shift in range(4)])
                errors = np.sum((matches - candidate["quad"]) ** 2, axis=(1, 2))
                self._prev_quad = matches[int(np.argmin(errors))]
            self._prev_timestamp = self._track_timestamp
        self._track = candidate
        self._track_timestamp = timestamp
        self._lost = 0
        self._roi_retry = True
        self._search_jobs.clear()
        w, h = candidate["width_mm"], candidate["height_mm"]
        matrix = candidate["rect_to_image"]
        # 直接映射正视面圆心才是正确中心，圆周采样不用于平均中心。
        # （中心 = 矩形中心的对角线交点；由 mm→px 单应把 (w/2,h/2) 投回原图即得。）
        raw = project_points(matrix, [[w / 2, h / 2]])[0]
        alpha = float(self.cfg.CENTER_ALPHA)
        # 一阶低通：平滑值只在显示口径使用；raw 供闭环（见 config.AIM_USE_RAW）。
        # 跳变过大（FILTER_RESET_PX）直接重置，避免平滑把真运动拖成滞后。
        if (self._smooth is None or alpha <= 0 or
                np.linalg.norm(raw - self._smooth) > self.cfg.FILTER_RESET_PX):
            center = raw
        else:
            center = self._smooth + alpha * (raw - self._smooth)
        self._smooth = center
        # 圆周采样点（仅可视化/交叉校验用）按尺寸缓存，避免每帧重算。
        key = (w, h, self.cfg.CIRCLE_RADIUS_MM, self.cfg.CIRCLE_SAMPLES)
        if key not in self._circle_cache:
            self._circle_cache[key] = standard_circle(*key)
        result = dict(candidate)
        result.update(valid=True, u=float(center[0]), v=float(center[1]), raw_u=float(raw[0]), raw_v=float(raw[1]),
                      quad=candidate["outer_quad"], detected_quad=candidate["quad"],
                      outer_inferred=candidate["kind"] == "inner",
                      circle_points=project_points(matrix, self._circle_cache[key]),
                      stage=stage, threshold=self._locked_th, search_count=self._search_count,
                      thresholds=self._threshold_count, proposals=self._proposals,
                      detector_ms=(time.perf_counter() - started) * 1000)
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
