"""Perspective robust black rectangle center detector for MaixCAM2.

Pipeline:
    grayscale -> Canny -> polygon approximation -> nested outer/inner quads
    -> edge gray-profile verification -> 8x8 homography solve -> 72-point
    standard-circle inverse mapping.

The implementation intentionally keeps the homography solve in NumPy/Python.
This makes the eight equations visible and avoids depending on a desktop-only
OpenCV transform helper. OpenCV is still used for image and contour operations.
"""

import math

import cv2
import numpy as np


def _order_quad(points):
    """Return [top-left, top-right, bottom-right, bottom-left]."""
    p = np.asarray(points, dtype=np.float64).reshape(4, 2)
    c = p.mean(axis=0)
    a = np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0])
    p = p[np.argsort(a)]
    if _polygon_area(p) < 0:
        p = p[::-1]
    # The first point is the one nearest the upper-left image corner.
    start = int(np.argmin(p[:, 0] + p[:, 1]))
    p = np.roll(p, -start, axis=0)
    return p.astype(np.float32)


def _polygon_area(points):
    p = np.asarray(points, dtype=np.float64)
    return 0.5 * float(np.sum(p[:, 0] * np.roll(p[:, 1], -1) -
                              p[:, 1] * np.roll(p[:, 0], -1)))


def _side_lengths(q):
    return np.linalg.norm(np.roll(q, -1, axis=0) - q, axis=1)


def _angles_are_rectangular(q, min_angle):
    for i in range(4):
        v1 = q[(i - 1) % 4] - q[i]
        v2 = q[(i + 1) % 4] - q[i]
        d = np.linalg.norm(v1) * np.linalg.norm(v2)
        if d < 1e-6:
            return False
        c = float(np.clip(np.dot(v1, v2) / d, -1.0, 1.0))
        angle = math.degrees(math.acos(c))
        if angle < min_angle or angle > 180.0 - min_angle:
            return False
    return True


def _gaussian_solve(a, b):
    """Solve an 8x8 system with partial-pivot Gaussian elimination."""
    m = np.concatenate((np.asarray(a, dtype=np.float64),
                        np.asarray(b, dtype=np.float64).reshape(-1, 1)), axis=1)
    n = m.shape[0]
    for col in range(n):
        pivot = col + int(np.argmax(np.abs(m[col:, col])))
        if abs(m[pivot, col]) < 1e-10:
            raise ValueError("singular homography system")
        if pivot != col:
            m[[col, pivot]] = m[[pivot, col]]
        m[col] /= m[col, col]
        for row in range(n):
            if row == col:
                continue
            factor = m[row, col]
            if abs(factor) > 1e-12:
                m[row] -= factor * m[col]
    return m[:, -1]


def homography_8x8(src_quad, width_mm, height_mm):
    """Map image quad to a 1 px = 1 mm rectangle using eight equations."""
    src = np.asarray(src_quad, dtype=np.float64).reshape(4, 2)
    dst = np.array([[0.0, 0.0], [width_mm, 0.0],
                    [width_mm, height_mm], [0.0, height_mm]], dtype=np.float64)
    a = []
    b = []
    for (x, y), (u, v) in zip(src, dst):
        a.append([x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y])
        b.append(u)
        a.append([0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y])
        b.append(v)
    h = _gaussian_solve(a, b)
    return np.array([[h[0], h[1], h[2]],
                     [h[3], h[4], h[5]],
                     [h[6], h[7], 1.0]], dtype=np.float64)


def project_points(matrix, points):
    p = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    q = np.concatenate((p, np.ones((len(p), 1))), axis=1)
    z = q.dot(np.asarray(matrix, dtype=np.float64).T)
    good = np.abs(z[:, 2]) > 1e-10
    out = np.zeros((len(p), 2), dtype=np.float64)
    out[good] = z[good, :2] / z[good, 2:3]
    return out


def inverse_map_points(h, points):
    return project_points(np.linalg.inv(h), points)


def standard_circle(width_mm, height_mm, radius_mm, samples=72):
    """Generate a 72-point circle in the rectified 1 mm coordinate system."""
    c = np.array([width_mm * 0.5, height_mm * 0.5], dtype=np.float64)
    angles = np.arange(int(samples), dtype=np.float64) * (2.0 * math.pi / samples)
    return c + float(radius_mm) * np.column_stack((np.cos(angles), np.sin(angles)))


def _sample_gray(gray, x, y):
    h, w = gray.shape[:2]
    xi = int(round(max(0.0, min(float(w - 1), x))))
    yi = int(round(max(0.0, min(float(h - 1), y))))
    return float(gray[yi, xi])


def edge_gray_profile(gray, outer, inner, samples=14):
    """Measure white/black/white values across each of the four black bands."""
    outer = np.asarray(outer, dtype=np.float64)
    inner = np.asarray(inner, dtype=np.float64)
    edge_rows = []
    for i in range(4):
        values = []
        for t in np.linspace(0.12, 0.88, int(samples)):
            po = outer[i] * (1.0 - t) + outer[(i + 1) % 4] * t
            pi = inner[i] * (1.0 - t) + inner[(i + 1) % 4] * t
            band = pi - po
            # 0 = just outside the outer edge, 1 = just inside inner edge.
            positions = (-0.18, 0.50, 1.18)
            values.append([_sample_gray(gray, *(po + band * k)) for k in positions])
        row = np.asarray(values, dtype=np.float64)
        outside, dark, inside = np.median(row, axis=0)
        contrast = min(outside, inside) - dark
        edge_rows.append(dict(outside=float(outside), dark=float(dark),
                              inside=float(inside), contrast=float(contrast)))
    contrasts = [r["contrast"] for r in edge_rows]
    return edge_rows, float(np.median(contrasts)), float(min(contrasts))


class RectangleCenterDetector:
    """Detect a white target surrounded by a black rectangular frame."""

    def __init__(self, config):
        self.cfg = config
        self.last = None
        self._smooth = None

    def _extract_quads(self, binary, w, h, source):
        """Extract relaxed quadrilateral candidates from one binary image.

        The original implementation only used Canny contours. On the target
        board, glare and the printed target rings often split one black tape
        edge into several short Canny contours. A dark-pixel mask is much more
        stable for this scene, so it is used as a complementary candidate
        source and both lists are deduplicated later.
        """
        image_area = float(w * h)
        contours = cv2.findContours(binary, cv2.RETR_LIST,
                                     cv2.CHAIN_APPROX_SIMPLE)[-2]
        out = []
        for cnt in contours:
            area = abs(float(cv2.contourArea(cnt)))
            if area < image_area * self.cfg.MIN_AREA_RATIO or area > image_area * self.cfg.MAX_AREA_RATIO:
                continue
            perimeter = cv2.arcLength(cnt, True)
            if perimeter <= 1.0:
                continue
            # Threshold masks can have a small notch at a specular highlight;
            # the larger epsilon is intentional and still keeps four corners.
            eps = float(self.cfg.APPROX_EPSILON) * perimeter
            approx = cv2.approxPolyDP(cnt, eps, True)
            if len(approx) != 4 or not cv2.isContourConvex(approx):
                hull = cv2.convexHull(cnt)
                approx = cv2.approxPolyDP(hull, eps * 1.25, True)
            if len(approx) != 4 or not cv2.isContourConvex(approx):
                continue
            q = _order_quad(approx.reshape(4, 2))
            if not _angles_are_rectangular(q, self.cfg.MIN_ANGLE_DEG):
                continue
            sides = _side_lengths(q)
            if min(sides) < 15.0:
                continue
            border = np.any((q[:, 0] < 2) | (q[:, 1] < 2) |
                            (q[:, 0] > w - 3) | (q[:, 1] > h - 3))
            out.append(dict(quad=q, area=area, sides=sides,
                            center=q.mean(axis=0), border=bool(border),
                            source=source))
        return out

    @staticmethod
    def _dedupe_quads(candidates):
        """Keep one candidate when Canny and threshold masks found the same quad."""
        kept = []
        for c in sorted(candidates, key=lambda x: x["area"], reverse=True):
            duplicate = False
            for k in kept:
                if abs(c["area"] - k["area"]) / max(c["area"], k["area"], 1.0) > 0.12:
                    continue
                if np.mean(np.linalg.norm(c["quad"] - k["quad"], axis=1)) < 10.0:
                    duplicate = True
                    break
            if not duplicate:
                kept.append(c)
        return kept

    def _quad_candidates(self, gray):
        h, w = gray.shape[:2]
        edges = cv2.Canny(gray, int(self.cfg.CANNY_LOW), int(self.cfg.CANNY_HIGH), L2gradient=True)
        all_candidates = self._extract_quads(edges, w, h, "canny")

        # Otsu is useful with the low-contrast/underexposed frames visible in
        # the screenshot. Also try a few fixed thresholds because a bright
        # reflection can move Otsu to the wrong side of the tape histogram.
        otsu, _ = cv2.threshold(gray, 0, 255,
                                cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        thresholds = [int(otsu)]
        for th in getattr(self.cfg, "DARK_THRESHOLDS", (35, 55, 75, 95, 115)):
            if abs(int(th) - int(otsu)) > 8:
                thresholds.append(int(th))
        for th in thresholds:
            _, dark = cv2.threshold(gray, th, 255, cv2.THRESH_BINARY_INV)
            kernel = np.ones((3, 3), np.uint8)
            dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, kernel, iterations=1)
            all_candidates.extend(self._extract_quads(dark, w, h, "dark_%d" % th))

        all_candidates = self._dedupe_quads(all_candidates)
        all_candidates.sort(key=lambda x: x["area"], reverse=True)
        return edges, all_candidates[:64]

    def _pair_score(self, gray, outer, inner):
        oq, iq = outer["quad"], inner["quad"]
        ratio = inner["area"] / max(outer["area"], 1.0)
        if not (self.cfg.MIN_INNER_RATIO <= ratio <= self.cfg.MAX_INNER_RATIO):
            return None
        if any(cv2.pointPolygonTest(oq.reshape(-1, 1, 2), tuple(p), False) <= 0 for p in iq):
            return None
        outer_ratio = max(outer["sides"]) / max(min(outer["sides"]), 1e-6)
        inner_ratio = max(inner["sides"]) / max(min(inner["sides"]), 1e-6)
        expected = self.cfg.TARGET_LONG_MM / self.cfg.TARGET_SHORT_MM
        shape_score = max(0.0, 1.0 - abs(outer_ratio - expected) / expected)
        shape_score = max(shape_score, max(0.0, 1.0 - abs(outer_ratio - 1.0 / expected) * expected))
        if abs(outer_ratio - inner_ratio) > 0.35 * max(outer_ratio, 1.0):
            return None
        rows, contrast, min_contrast = edge_gray_profile(gray, oq, iq)
        if min_contrast < self.cfg.MIN_EDGE_CONTRAST:
            return None
        # A4-style tape has a predictable inner/outer area ratio. Keep this
        # as a soft score so non-A4 rectangular clutter is less likely to win,
        # while still allowing a different frame width after field tuning.
        frame_mm = max(0.0, float(getattr(self.cfg, "BLACK_FRAME_MM", 0.0)))
        short_mm = float(self.cfg.TARGET_SHORT_MM)
        long_mm = float(self.cfg.TARGET_LONG_MM)
        expected_ratio = ((short_mm - 2.0 * frame_mm) *
                          (long_mm - 2.0 * frame_mm) /
                          max(short_mm * long_mm, 1.0))
        ratio_score = max(0.0, 1.0 - abs(ratio - expected_ratio) / 0.30)
        # Prefer a strong, consistently dark band and a candidate away from a crop edge.
        consistency = max(0.0, min_contrast / max(contrast, 1.0))
        score = (contrast * 2.0 + min_contrast + 45.0 * shape_score +
                 20.0 * consistency + 8.0 * (1.0 - ratio) + 18.0 * ratio_score)
        if outer["border"]:
            score -= 35.0
        return score, rows, contrast, min_contrast

    def _select_pair(self, gray, candidates):
        best = None
        for i, outer in enumerate(candidates):
            for inner in candidates[i + 1:]:
                if inner["area"] >= outer["area"]:
                    continue
                result = self._pair_score(gray, outer, inner)
                if result is None:
                    continue
                score, rows, contrast, min_contrast = result
                if best is None or score > best["score"]:
                    best = dict(score=score, outer=outer, inner=inner,
                                rows=rows, contrast=contrast,
                                min_contrast=min_contrast, fallback=False)
        return best

    def _fallback_quad(self, gray, candidates):
        # If only one edge is available, synthesize the other edge around the
        # quad center. Try both directions: a contour may be the outer tape
        # edge, or it may be the bright paper's inner edge.
        for c in candidates:
            q = c["quad"].astype(np.float64)
            center = q.mean(axis=0)
            best = None
            for scale in (0.68, 0.76, 0.84, 0.90, 0.94, 1.08, 1.18, 1.30, 1.42):
                other = center + scale * (q - center)
                if scale < 1.0:
                    outer_q, inner_q = q, other
                else:
                    outer_q, inner_q = other, q
                rows, contrast, min_contrast = edge_gray_profile(gray, outer_q, inner_q)
                if best is None or min_contrast > best[0]:
                    best = (min_contrast, contrast, rows, outer_q, inner_q)
            min_contrast, contrast, rows, outer_q, inner_q = best
            if min_contrast >= self.cfg.MIN_EDGE_CONTRAST:
                outer = dict(c)
                outer["quad"] = outer_q.astype(np.float32)
                inner = dict(quad=inner_q.astype(np.float32), area=c["area"] * 0.70,
                             sides=_side_lengths(inner_q), center=inner_q.mean(axis=0), border=False)
                return dict(score=contrast, outer=outer, inner=inner, rows=rows,
                            contrast=contrast, min_contrast=min_contrast,
                            fallback=True)
            # A dark-mask contour is already evidence that the candidate is
            # made from black pixels. If glare breaks the white/black/white
            # profile, accept it when the enclosed center is demonstrably
            # bright. This is the important recovery path for the uploaded
            # frame: the black tape is visible, but its Canny profile is not
            # monotonic because of specular reflections.
            if str(c.get("source", "")).startswith("dark_") and not c.get("border"):
                cc = c["center"]
                radius = max(4, int(0.10 * min(c["sides"])))
                x, y = int(round(cc[0])), int(round(cc[1]))
                patch = gray[max(0, y - radius):y + radius + 1,
                             max(0, x - radius):x + radius + 1]
                center_mean = float(patch.mean()) if patch.size else 0.0
                dark_limit = float(getattr(self.cfg, "DARK_CENTER_MIN", 90.0))
                if center_mean >= dark_limit:
                    scale = math.sqrt(max(0.30, min(0.94,
                        ((self.cfg.TARGET_SHORT_MM - 2.0 * self.cfg.BLACK_FRAME_MM) *
                         (self.cfg.TARGET_LONG_MM - 2.0 * self.cfg.BLACK_FRAME_MM) /
                         (self.cfg.TARGET_SHORT_MM * self.cfg.TARGET_LONG_MM)))))
                    iq = center + scale * (q - center)
                    inner = dict(quad=iq.astype(np.float32),
                                 area=c["area"] * scale * scale,
                                 sides=_side_lengths(iq), center=iq.mean(axis=0),
                                 border=False)
                    # A conservative diagnostic score; it is only used for
                    # choosing between fallback candidates.
                    return dict(score=center_mean, outer=c, inner=inner,
                                rows=[], contrast=center_mean,
                                min_contrast=center_mean - dark_limit,
                                fallback=True)
        return None

    def process(self, gray, timestamp=None):
        g = np.asarray(gray)
        if g.ndim == 3:
            g = g[:, :, 0]
        if g.dtype != np.uint8:
            g = np.clip(g, 0, 255).astype(np.uint8)
        edges, candidates = self._quad_candidates(g)
        pair = self._select_pair(g, candidates)
        if pair is None:
            pair = self._fallback_quad(g, candidates)
        if pair is None:
            self.last = dict(valid=False, edges=edges, candidates=len(candidates),
                             stage="no_black_frame")
            return self.last

        oq = _order_quad(pair["outer"]["quad"])
        side0, side1 = _side_lengths(oq)[0], _side_lengths(oq)[1]
        short_mm, long_mm = self.cfg.TARGET_SHORT_MM, self.cfg.TARGET_LONG_MM
        if side0 > side1:
            width_mm, height_mm = long_mm, short_mm
        else:
            width_mm, height_mm = short_mm, long_mm
        try:
            h_img_to_rect = homography_8x8(oq, width_mm, height_mm)
            circle = standard_circle(width_mm, height_mm,
                                     self.cfg.CIRCLE_RADIUS_MM,
                                     self.cfg.CIRCLE_SAMPLES)
            mapped = inverse_map_points(h_img_to_rect, circle)
            # The mean of points after a projective inverse map is not the
            # projective image of the circle center. Under strong perspective
            # it can shift by many pixels. Keep the requested 72-point map for
            # calibration/visualization, but use the mapped rectified center
            # as the actual target center.
            rect_center = np.array([[width_mm * 0.5, height_mm * 0.5]], dtype=np.float64)
            raw_center = inverse_map_points(h_img_to_rect, rect_center)[0]
        except (ValueError, np.linalg.LinAlgError):
            self.last = dict(valid=False, edges=edges, candidates=len(candidates),
                             stage="singular_homography")
            return self.last
        if self._smooth is None or self.cfg.CENTER_ALPHA <= 0.0:
            center = raw_center
        else:
            a = float(self.cfg.CENTER_ALPHA)
            center = self._smooth * (1.0 - a) + raw_center * a
        self._smooth = center
        result = dict(valid=True, u=float(center[0]), v=float(center[1]),
                      raw_u=float(raw_center[0]), raw_v=float(raw_center[1]),
                      quad=oq, inner_quad=_order_quad(pair["inner"]["quad"]),
                      homography=h_img_to_rect, circle_points=mapped,
                      edge_profile=pair["rows"], contrast=pair["contrast"],
                      min_contrast=pair["min_contrast"], score=pair["score"],
                      fallback=pair["fallback"], candidates=len(candidates),
                      width_mm=width_mm, height_mm=height_mm, edges=edges,
                      stage="ok")
        self.last = result
        return result


def draw_result(gray, result):
    """Create a BGR debug image without requiring Maix-specific APIs."""
    g = np.asarray(gray)
    if g.ndim == 3:
        g = g[:, :, 0]
    canvas = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    if not result.get("valid"):
        cv2.putText(canvas, "NO BLACK RECTANGLE", (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return canvas
    q = np.asarray(result["quad"], dtype=np.int32).reshape(-1, 1, 2)
    iq = np.asarray(result["inner_quad"], dtype=np.int32).reshape(-1, 1, 2)
    # Green is the confirmed rectangle outline requested for on-device tuning;
    # the inner edge is yellow so the tape width can be inspected separately.
    cv2.polylines(canvas, [q], True, (0, 255, 0), 3)
    cv2.polylines(canvas, [iq], True, (0, 220, 220), 1)
    for i, (x, y) in enumerate(np.asarray(result["quad"], dtype=np.float32)):
        p = (int(round(x)), int(round(y)))
        cv2.circle(canvas, p, 6, (255, 0, 255), -1)
        cv2.circle(canvas, p, 8, (255, 255, 255), 1)
        cv2.putText(canvas, "P%d (%d,%d)" % (i + 1, p[0], p[1]),
                    (p[0] + 6, p[1] - 8), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (255, 255, 0), 1, cv2.LINE_AA)
    c = (int(round(result["u"])), int(round(result["v"])))
    cv2.circle(canvas, c, 5, (255, 0, 0), -1)
    cv2.putText(canvas, "center=(%d,%d) contrast=%.0f" %
                (c[0], c[1], result["min_contrast"]), (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 220, 0), 2)
    return canvas
