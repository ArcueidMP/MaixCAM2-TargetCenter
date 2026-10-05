"""黑环/亮面双候选、稀疏透视验证、四边直线跟踪。仅依赖 cv2/numpy。

搜索在缩小图上执行；精修和坐标输出始终使用当前帧的原始分辨率。
已锁定时只采样四条窄带，不运行全图 Canny、轮廓扫描或阈值遍历。
"""
import time

import cv2
import numpy as np


def _polygon_area(p):
    return float(cv2.contourArea(np.asarray(p, np.float32).reshape(-1, 1, 2), oriented=True))


def _order_quad(points):
    p = np.asarray(points, np.float32).reshape(4, 2)
    c = p.mean(axis=0)
    p = p[np.argsort(np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0]))]
    if _polygon_area(p) < 0:
        p = p[::-1]
    return np.roll(p, -int(np.argmin(p.sum(axis=1))), axis=0).copy()


def _side_lengths(q):
    return np.linalg.norm(np.roll(q, -1, axis=0) - q, axis=1)


def _gaussian_solve(a, b):
    """带部分主元的高斯消元；消去行向量化，避免 64 次 Python 行循环。"""
    m = np.column_stack((np.asarray(a, np.float64), np.asarray(b, np.float64)))
    for col in range(len(m)):
        pivot = col + int(np.argmax(np.abs(m[col:, col])))
        if abs(m[pivot, col]) < 1e-10:
            raise ValueError("degenerate quadrilateral")
        if pivot != col:
            m[[col, pivot]] = m[[pivot, col]]
        m[col] /= m[col, col]
        factors = m[:, col].copy()
        factors[col] = 0.0
        m -= factors[:, None] * m[col]
    return m[:, -1]


def _homography(src, dst):
    """显式构建 8×8 方程；先归一化坐标以提高大像素坐标下的稳定性。"""
    src, dst = np.asarray(src, np.float64), np.asarray(dst, np.float64)
    sc, dc = src.mean(axis=0), dst.mean(axis=0)
    ss, ds = float(np.max(np.abs(src - sc))), float(np.max(np.abs(dst - dc)))
    if min(ss, ds) < 1e-8:
        raise ValueError("zero-size rectangle")
    xy, uv = (src - sc) / ss, (dst - dc) / ds
    a = np.zeros((8, 8), np.float64)
    a[0::2, :3] = np.column_stack((xy, np.ones(4)))
    a[1::2, 3:6] = a[0::2, :3]
    a[0::2, 6:] = -uv[:, :1] * xy
    a[1::2, 6:] = -uv[:, 1:] * xy
    v = _gaussian_solve(a, uv.reshape(-1))
    hn = np.append(v, 1.0).reshape(3, 3)
    ts = np.array([[1 / ss, 0, -sc[0] / ss], [0, 1 / ss, -sc[1] / ss], [0, 0, 1]])
    td_inv = np.array([[ds, 0, dc[0]], [0, ds, dc[1]], [0, 0, 1]])
    h = td_inv @ hn @ ts
    return h / h[2, 2] if abs(h[2, 2]) > 1e-12 else h


def homography_8x8(src_quad, width_mm, height_mm):
    return _homography(src_quad, [[0, 0], [width_mm, 0],
                                 [width_mm, height_mm], [0, height_mm]])


def project_points(matrix, points):
    p = np.asarray(points, np.float64).reshape(-1, 2)
    z = np.column_stack((p, np.ones(len(p)))) @ np.asarray(matrix).T
    if np.any(np.abs(z[:, 2]) < 1e-9):
        raise ValueError("point at projective infinity")
    return z[:, :2] / z[:, 2:]


def inverse_map_points(h, points):
    return project_points(np.linalg.inv(h), points)


def standard_circle(width_mm, height_mm, radius_mm, samples=72):
    a = np.arange(samples) * (2 * np.pi / samples)
    return np.array([width_mm / 2, height_mm / 2]) + radius_mm * np.column_stack((np.cos(a), np.sin(a)))


def _rectangle(w, h, inset=0.0):
    return np.array([[inset, inset], [w - inset, inset],
                     [w - inset, h - inset], [inset, h - inset]], np.float64)


def _sample(gray, points):
    """一次 OpenCV remap 双线性采样整批点，越界由调用方拒绝。"""
    shape = points.shape[:-1]
    p = points.reshape(-1, 2).astype(np.float32)
    values = cv2.remap(gray, p[:, 0].reshape(1, -1), p[:, 1].reshape(1, -1),
                       cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    return values.reshape(shape)


class RectangleCenterDetector:
    def __init__(self, config):
        self.cfg = config
        self.last = None
        self._smooth = None
        self._track = None
        self._locked_th = None
        self._frame = 0
        self._kernel = np.ones((3, 3), np.uint8)
        self._t = np.linspace(0.12, 0.88, config.EDGE_SAMPLES, dtype=np.float32)
        self._circle_cache = {}
        self._search_count = 0
        self._threshold_count = 0

    def _valid_quad(self, q, shape, margin=1):
        h, w = shape
        if (q[:, 0].min() < margin or q[:, 1].min() < margin or
                q[:, 0].max() >= w - margin or q[:, 1].max() >= h - margin):
            return False
        if not cv2.isContourConvex(np.asarray(q, np.float32).reshape(-1, 1, 2)):
            return False
        e = np.roll(q, -1, axis=0) - q
        lengths = np.linalg.norm(e, axis=1)
        if lengths.min() < 12:
            return False
        # 透视图不要求四角接近 90 度，只排除退化尖角和过扁的四边形。
        sine = np.abs(e[:, 0] * np.roll(e[:, 1], -1) -
                      e[:, 1] * np.roll(e[:, 0], -1)) / (lengths * np.roll(lengths, -1))
        return bool(sine.min() > 0.20)

    def _layout(self, q, kind):
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
            rect_to_image = _homography(physical_quad, q)
            t = self._t
            # 三个点覆盖黑带的中部，再在内边缘后采两个亮面点。
            offsets = tape * np.array([0.25, 0.50, 0.75, 1.22, 1.45])
            p = np.empty((4, len(t), 5, 2), np.float64)
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
        if (mapped[..., 0].min() < 0 or mapped[..., 1].min() < 0 or
                mapped[..., 0].max() >= gw - 1 or mapped[..., 1].max() >= gh - 1):
            return None
        samples = _sample(gray, mapped).astype(np.float32)
        dark = np.median(samples[:, :, :3], axis=2)
        bright = np.median(samples[:, :, 3:], axis=2)
        delta = bright - dark
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
        return dict(quad=q, outer_quad=outer,
                    inner_quad=project_points(rect_to_image, _rectangle(w, h, tape)).astype(np.float32),
                    kind=kind, rect_to_image=rect_to_image, width_mm=w, height_mm=h,
                    edge_profile=rows, contrast=float(np.median(edge_contrast)),
                    min_contrast=float(edge_contrast.min()), coverage=coverage,
                    score=float(np.median(edge_contrast) + 60 * coverage.mean()))

    def _refine(self, gray, q, kind, radius):
        """当前帧四条灰度窄带 -> 有方向的边缘跳变 -> fitLine -> 四角交点。"""
        q = np.asarray(q, np.float32)
        nxt = np.roll(q, -1, axis=0)
        e = nxt - q
        lengths = np.linalg.norm(e, axis=1)
        if lengths.min() < 10:
            return None
        normal = np.column_stack((-e[:, 1], e[:, 0])) / lengths[:, None]
        base = q[:, None, :] * (1 - self._t[None, :, None]) + nxt[:, None, :] * self._t[None, :, None]
        offsets = np.arange(-radius, radius + 1, dtype=np.float32)
        points = base[:, :, None, :] + normal[:, None, None, :] * offsets[None, None, :, None]
        gh, gw = gray.shape
        inside = ((points[..., 0] >= 1) & (points[..., 0] < gw - 1) &
                  (points[..., 1] >= 1) & (points[..., 1] < gh - 1)).all(axis=2)
        values = _sample(gray, points).astype(np.float32)
        gradient = values[:, :, 2:] - values[:, :, :-2]
        if kind == "outer":
            gradient *= -1
        # 近预测边缘优先，避免捕获靶纸内部印刷圆或另一侧的胶带边缘。
        penalty = np.abs(offsets[1:-1])[None, None, :] * 0.45
        indices = np.argmax(gradient - penalty, axis=2)
        strength = np.take_along_axis(gradient, indices[..., None], axis=2)[..., 0]
        selected = offsets[indices + 1]
        lines = []
        for i in range(4):
            good = inside[i] & (strength[i] >= self.cfg.MIN_EDGE_CONTRAST * 0.7)
            if good.sum() < max(5, len(self._t) * self.cfg.WEAK_EDGE_COVERAGE):
                return None
            shifts = selected[i, good]
            med = float(np.median(shifts))
            good &= np.abs(selected[i] - med) <= max(2.5, float(np.median(np.abs(shifts - med))) * 3)
            if good.sum() < 5:
                return None
            samples = base[i, good] + selected[i, good, None] * normal[i]
            vx, vy, x, y = cv2.fitLine(samples, cv2.DIST_HUBER, 0, 0.01, 0.01).reshape(4)
            # 直线 ax+by+c=0。
            lines.append(np.array([-vy, vx, vy * x - vx * y], np.float64))
        lines = np.asarray(lines)
        crosses = np.cross(np.roll(lines, 1, axis=0), lines)
        if np.any(np.abs(crosses[:, 2]) < 1e-8):
            return None
        refined = (crosses[:, :2] / crosses[:, 2:]).astype(np.float32)
        if (np.linalg.norm(refined - q, axis=1).max() > radius * 2.8 or
                not self._valid_quad(refined, gray.shape)):
            return None
        return refined

    def _candidate_quads(self, mask):
        """用轮廓树的孔/黑环关系代替候选两两组合。"""
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        if hierarchy is None:
            return []
        hierarchy = hierarchy[0]
        h, w = mask.shape
        candidates = []
        # 先用廉价的包围框/面积筛选，仅拟合最大的少数轮廓。
        eligible = []
        for i, cnt in enumerate(contours):
            x, y, cw, ch = cv2.boundingRect(cnt)
            if min(cw, ch) < 15 or cw * ch < w * h * self.cfg.MIN_AREA_RATIO:
                continue
            area = abs(cv2.contourArea(cnt))
            if w * h * self.cfg.MIN_AREA_RATIO <= area <= w * h * self.cfg.MAX_AREA_RATIO:
                eligible.append((area, i))
        for area, i in sorted(eligible, reverse=True)[:self.cfg.MAX_CANDIDATES]:
            cnt = contours[i]
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, self.cfg.APPROX_EPSILON * peri, True)
            # 不使用凸包修补任意背景：凸包可把桌面阴影拼成错误大四边形。
            if len(approx) != 4 or not cv2.isContourConvex(approx):
                continue
            q = _order_quad(approx)
            if not self._valid_quad(q, mask.shape):
                continue
            parent, depth = int(hierarchy[i][3]), 0
            while parent >= 0:
                depth += 1
                parent = int(hierarchy[parent][3])
            kind = "inner" if depth % 2 else "outer"
            if kind == "outer" and hierarchy[i][2] < 0:
                continue  # 实心黑块没有亮孔。
            candidates.append((q, kind))
        # 亮面孔比与背景粘连的黑环更稳定。
        return sorted(candidates, key=lambda c: c[1] != "inner")

    def _search(self, gray, roi=None):
        self._search_count += 1
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
        started = time.perf_counter()
        g = np.asarray(gray)
        if g.ndim == 3 and g.shape[2] == 1:
            g = g[:, :, 0]
        if g.ndim != 2 or g.dtype != np.uint8:
            raise ValueError("process expects a uint8 grayscale image")
        self._frame += 1
        self._search_count = self._threshold_count = 0
        candidate, stage = None, "search"
        if self._track is not None:
            q, kind = self._track["quad"], self._track["kind"]
            q = self._refine(g, q, kind, self.cfg.TRACK_RADIUS)
            if q is not None:
                candidate = self._verify(g, q, kind)
                stage = "track"
        if candidate is None and self._track is not None:
            q = self._track["outer_quad"]
            lo, hi = q.min(axis=0), q.max(axis=0)
            pad = np.maximum((hi - lo) * self.cfg.ROI_PADDING, 24)
            lo = np.maximum(0, np.floor(lo - pad)).astype(int)
            hi = np.minimum([g.shape[1], g.shape[0]], np.ceil(hi + pad)).astype(int)
            candidate = self._search(g, (int(lo[0]), int(lo[1]), int(hi[0] - lo[0]), int(hi[1] - lo[1])))
            stage = "roi_search"
        if candidate is None:
            candidate = self._search(g)
            stage = "full_search"
        if candidate is None:
            self._track = self._smooth = None
            self._locked_th = None
            self.last = dict(valid=False, stage="no_black_frame", search_count=self._search_count,
                             thresholds=self._threshold_count, detector_ms=(time.perf_counter() - started) * 1000)
            return self.last
        self._track = candidate
        w, h = candidate["width_mm"], candidate["height_mm"]
        matrix = candidate["rect_to_image"]
        # 直接映射正视面圆心才是正确中心，圆周采样不用于平均中心。
        raw = project_points(matrix, [[w / 2, h / 2]])[0]
        alpha = float(self.cfg.CENTER_ALPHA)
        if (self._smooth is None or alpha <= 0 or
                np.linalg.norm(raw - self._smooth) > self.cfg.FILTER_RESET_PX):
            center = raw
        else:
            center = self._smooth + alpha * (raw - self._smooth)
        self._smooth = center
        key = (w, h, self.cfg.CIRCLE_RADIUS_MM, self.cfg.CIRCLE_SAMPLES)
        if key not in self._circle_cache:
            self._circle_cache[key] = standard_circle(*key)
        result = dict(candidate)
        result.update(valid=True, u=float(center[0]), v=float(center[1]), raw_u=float(raw[0]), raw_v=float(raw[1]),
                      quad=candidate["outer_quad"], detected_quad=candidate["quad"],
                      outer_inferred=candidate["kind"] == "inner",
                      circle_points=project_points(matrix, self._circle_cache[key]),
                      stage=stage, threshold=self._locked_th, search_count=self._search_count,
                      thresholds=self._threshold_count, detector_ms=(time.perf_counter() - started) * 1000)
        self.last = result
        return result


def draw_result(gray, result, show_width=480):
    """缩小后再转 BGR 和描边，输出坐标仍为相机原图坐标。"""
    g = np.asarray(gray).squeeze()
    scale = min(1.0, float(show_width) / g.shape[1])
    if scale < 1:
        g = cv2.resize(g, (int(round(g.shape[1] * scale)), int(round(g.shape[0] * scale))))
    canvas = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    if not result.get("valid"):
        cv2.putText(canvas, "NO BLACK RECTANGLE", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 255), 1)
        return canvas
    q = np.rint(result["quad"] * scale).astype(np.int32)
    iq = np.rint(result["inner_quad"] * scale).astype(np.int32)
    cv2.polylines(canvas, [q], True, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.polylines(canvas, [iq], True, (0, 220, 220), 1)
    for i, (x, y) in enumerate(result["quad"]):
        p = tuple(q[i])
        cv2.circle(canvas, p, 4, (255, 0, 255), -1)
        label = "P%d (%d,%d)" % (i + 1, round(float(x)), round(float(y)))
        tw = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .36, 1)[0][0]
        pos = (max(0, min(int(p[0]) + 5, canvas.shape[1] - tw - 1)),
               max(12, min(int(p[1]) - 7, canvas.shape[0] - 5)))
        cv2.putText(canvas, label, pos, cv2.FONT_HERSHEY_SIMPLEX, .36, (255, 255, 0), 1, cv2.LINE_AA)
    c = tuple(np.rint(np.array([result["u"], result["v"]]) * scale).astype(int))
    cv2.drawMarker(canvas, c, (255, 0, 0), cv2.MARKER_CROSS, 14, 2)
    cv2.putText(canvas, "(%d,%d) %s %.1fms" % (round(result["u"]), round(result["v"]),
                result["stage"], result["detector_ms"]), (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 220, 0), 1)
    return canvas
