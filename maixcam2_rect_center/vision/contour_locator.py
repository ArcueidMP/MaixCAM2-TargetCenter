"""Bounded low-resolution proposals: one Canny mask, one Otsu fallback."""
import time
import cv2
import numpy as np
from .rect_geometry import order_quad, quad_from_contour, quad_is_valid

LEVEL_EDGE, LEVEL_MASK = 0, 1


class ContourLocator:
    LEVEL_EDGE, LEVEL_MASK = LEVEL_EDGE, LEVEL_MASK

    def __init__(self, cfg):
        self.cfg = cfg
        self.last_stats = {}
        size = max(1, int(getattr(cfg, "EDGE_CLOSE_PX", 3)))
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))

    def _quads_from_mask(self, mask):
        contours = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
        self.last_stats["contours"] = len(contours)
        h, w = mask.shape
        cfg, quads = self.cfg, []
        for cnt in sorted(contours, key=lambda c: abs(cv2.contourArea(c)), reverse=True)[:getattr(cfg, "SEARCH_MAX_CONTOURS", 16)]:
            _, _, cw, ch = cv2.boundingRect(cnt)
            if min(cw, ch) < max(8, .04*min(w, h)):
                continue
            area = abs(cv2.contourArea(cnt))
            if not w*h*cfg.MIN_AREA_RATIO <= area <= w*h*cfg.MAX_AREA_RATIO:
                continue
            q = quad_from_contour(cnt, cfg.APPROX_EPSILON, getattr(cfg, "HULL_FILL_RATIO", .6))
            if q is None or not quad_is_valid(q, mask.shape, min_edge=6):
                continue
            if any(np.max(np.linalg.norm(q-old, axis=1)) < 2 for old in quads):
                continue
            quads.append(q)
            if len(quads) >= cfg.MAX_CANDIDATES:
                break  # Bound expensive polygon fitting, not accepted-candidate stability.
        return quads

    def locate(self, gray, roi=None, level=LEVEL_EDGE):
        started = time.perf_counter()
        x, y, w, h = roi or (0, 0, gray.shape[1], gray.shape[0])
        self.last_stats = dict(masks=0, contours=0, quads=0, ms=0., level=level)
        if w <= 0 or h <= 0:
            return []
        scale = min(1., self.cfg.SEARCH_WIDTH / float(w))
        sw, sh = max(1, round(w*scale)), max(1, round(h*scale))
        crop = gray[y:y+h, x:x+w]
        small = cv2.resize(crop, (sw, sh), interpolation=cv2.INTER_AREA) if scale < 1 else crop
        if level == LEVEL_EDGE:
            mask = cv2.Canny(small, self.cfg.CANNY_LOW, self.cfg.CANNY_HIGH)
        elif level == LEVEL_MASK:
            mask = cv2.threshold(small, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
        else:
            raise ValueError("unknown proposal level")
        if self.kernel.shape[0] > 1:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        self.last_stats["masks"] = 1
        quads = self._quads_from_mask(mask)
        self.last_stats.update(quads=len(quads), ms=(time.perf_counter()-started)*1000)
        mapped = [(q*np.array([w/sw, h/sh], np.float32)+[x, y]).astype(np.float32) for q in quads]
        return [(q, kind) for q in mapped for kind in ("inner", "outer")]
