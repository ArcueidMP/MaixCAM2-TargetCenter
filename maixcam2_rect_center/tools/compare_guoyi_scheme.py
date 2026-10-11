# -*- coding: utf-8 -*-
"""国一方案（2025 电赛 E 题 ShiJiang-hub/2025-E- 树莓派视觉）对照工具。

用途：把上游国一方案**忠实移植**到本工作区，在 fixture 集与自建倾角/旋转序列上
与 B 轨现行 `vision.rectangle_center.RectangleCenterDetector` 同图对拍，回答
「换国一那套视觉能不能做到多角度 / 稳定 / 高帧率 / 逆透视解形变」。

纯 PC、只读，不依赖 maix / 相机；不改动任何应用源码。

国一方案关键步骤（见 reference/upstream_2025E/树莓派视觉/c1.py）：
    灰度 -> bilateralFilter -> Canny(50,150) -> findContours(RETR_TREE)
    -> 面积[2000,80000] + 四角>=70度 + 长宽比 1.52±0.4 三重筛选
    -> 取面积最小者(内侧) -> _order_points -> getPerspectiveTransform 到 171x260
    -> 正视图中心(85,130) 经逆变换 perspectiveTransform 回原图 = 靶心

用法：
    python -B tools/compare_guoyi_scheme.py            # 打印 A–E 五节对拍报告
    python -B tools/compare_guoyi_scheme.py --figures   # 另存两张可视证据图到 tools/e2e_report/

结论摘要（2026-10-12 实测）：
    · 逆透视中心：倾角 0–78° 误差恒 0.000px（质心/外接框中心最大 10.02px）——
      与 B 轨 mm→px 单应投 (w/2,h/2) 数学等价，即国一最有价值的一环 B 轨已具备。
    · 国一前端原样仅检出 11/30（B 轨 29/30）；45°/135° 面内旋转因 _order_points
      用 sum/diff 极值法撞点而错锁 157px。
    · 消融：面积界改相对比例 -> 23/30；角点排序改极角法 -> 45°/135° 降到 0.505px。
    · 国一无跟踪，每帧恒定全分辨率 Canny ≈3.3ms(PC)；B 轨稳态窄带精修 ≈0.86ms。
"""
import argparse
import importlib.util
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from config import Config
from vision.rectangle_center import RectangleCenterDetector


def _load(path, name):
    """按路径加载工具脚本（tools/ 非包，用 importlib 更稳）。"""
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


perf = _load(ROOT / "tools" / "test_performance.py", "perf")


# ------------------------------------------------------------------ 国一方案移植
class GuoyiA4Detector:
    """忠实移植：Canny -> 轮廓 -> 四边形三重筛选 -> 取最小内框 -> 透视中心逆映射。

    area_mode  fixed     = 国一原样固定面积界 [2000, 80000]（与赛场器材过拟合）
               relative  = 按图像面积比例（用于消融，隔离"尺度标定"与"算法"）
    order_mode sumdiff   = 国一原样四角排序（45° 面内旋转会撞点退化）
               polar     = 极角排序（修该缺陷）
    downscale   None     = 原分辨率（国一原设计）；整数 = 先缩到该宽度再算
    """

    def __init__(self, min_area=2000, max_area=80000, angle=70.0,
                 aspect_tol=0.4, tw=171, th=260, downscale=None,
                 area_mode="fixed", order_mode="sumdiff"):
        self.min_area, self.max_area = min_area, max_area
        self.angle_threshold = angle
        self.aspect = 26.0 / 17.1
        self.aspect_tol = aspect_tol
        self.tw, self.th = tw, th
        self.downscale = downscale
        self.area_mode = area_mode
        self.order_mode = order_mode

    def _order_points(self, pts):
        if self.order_mode == "polar":
            c = pts.mean(axis=0)
            p = pts[np.argsort(np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0]))]
            if cv2.contourArea(p.reshape(-1, 1, 2).astype(np.float32), oriented=True) < 0:
                p = p[::-1]
            return np.roll(p, -int(np.argmin(p.sum(axis=1))), axis=0).astype("float32")
        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)                       # 国一原样：x+y 与 y-x 极值
        return np.array([pts[np.argmin(s)], pts[np.argmax(diff)],
                         pts[np.argmax(s)], pts[np.argmin(diff)]], dtype="float32")

    def _is_rectangle(self, approx):
        """四角 >= angle_threshold 度（国一原样，用点积求夹角）。"""
        if len(approx) != 4:
            return False
        corners = approx.reshape(4, 2).astype(np.float32)
        vectors = [corners[(i + 1) % 4] - corners[i] for i in range(4)]
        for i in range(4):
            v1, v2 = vectors[i], vectors[(i + 1) % 4]
            n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
            if n1 == 0 or n2 == 0:
                return False
            cos = np.clip(abs(np.dot(v1, v2) / (n1 * n2)), 0.0, 1.0)
            if np.arccos(cos) * 180 / np.pi < self.angle_threshold:
                return False
        return True

    def _check_aspect(self, corners):
        """相邻边长比接近 A4 比例 1.52±0.4（国一原样）。"""
        s1 = np.linalg.norm(corners[0] - corners[1])
        s2 = np.linalg.norm(corners[1] - corners[2])
        ratio = max(s1, s2) / max(1e-9, min(s1, s2))
        return self.aspect * (1 - self.aspect_tol) <= ratio <= self.aspect * (1 + self.aspect_tol)

    def detect(self, gray, low=50, high=150):
        """返回 (center(u,v) or None, quad or None, ms)。"""
        t0 = time.perf_counter()
        g = np.asarray(gray)
        if g.ndim == 3:
            g = g[:, :, 0]
        scale = 1.0
        if self.downscale and g.shape[1] > self.downscale:
            scale = self.downscale / float(g.shape[1])
            g = cv2.resize(g, (self.downscale, max(24, int(round(g.shape[0] * scale)))),
                           interpolation=cv2.INTER_AREA)
        blur = cv2.bilateralFilter(g, 9, 75, 75)
        edges = cv2.Canny(blur, low, high)
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        total = float(g.shape[1] * g.shape[0] / (scale * scale))
        lo_a, hi_a = ((0.012 * total, 0.62 * total) if self.area_mode == "relative"
                      else (self.min_area, self.max_area))
        best_area, best_corners = hi_a, None
        for cnt in contours:
            area = cv2.contourArea(cnt) / (scale * scale)          # 折算回原图面积口径
            if area < lo_a or area > hi_a:
                continue
            approx = cv2.approxPolyDP(cnt, 0.1 * cv2.arcLength(cnt, True), True)
            if not self._is_rectangle(approx):
                continue
            corners = approx.reshape(4, 2).astype(np.float32)
            if not self._check_aspect(corners):
                continue
            if area < best_area:                                    # 国一：取面积最小（内侧）
                best_area, best_corners = area, self._order_points(corners)
        ms = (time.perf_counter() - t0) * 1000
        if best_corners is None:
            return None, None, ms
        src = best_corners
        dst = np.array([[0, 0], [self.tw - 1, 0],
                        [self.tw - 1, self.th - 1], [0, self.th - 1]], "float32")
        m_inv = cv2.getPerspectiveTransform(dst, src)               # 正视图 -> 原图
        c = cv2.perspectiveTransform(
            np.array([[[self.tw // 2, self.th // 2]]], "float32"), m_inv)[0, 0]
        return (float(c[0] / scale), float(c[1] / scale)), best_corners / scale, ms


# ------------------------------------------------------------------ 场景生成
def tilted_scene(theta_deg, phi_deg=0.0, d=560.0, f=560.0, cx=320.0, cy=240.0,
                 paper=220, tape=20, background=175):
    """210x297mm 靶纸绕竖直轴转 theta、绕水平轴转 phi 后针孔投影到 640x480。

    靶纸中心在光轴上，故真值恒为 (cx,cy)。返回 img, truth, quad（外框四角图像坐标）。
    """
    w2, h2 = 105.0, 148.5
    plane = np.array([[-w2, -h2, 0], [w2, -h2, 0], [w2, h2, 0], [-w2, h2, 0]], float)
    ry, rx = np.deg2rad(theta_deg), np.deg2rad(phi_deg)
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]])
    pts = (Ry @ Rx @ plane.T).T + np.array([0, 0, d])
    uv = np.column_stack((f * pts[:, 0] / pts[:, 2] + cx, f * pts[:, 1] / pts[:, 2] + cy))
    m = cv2.getPerspectiveTransform(
        np.float32([[0, 0], [210, 0], [210, 297], [0, 297]]), np.float32(uv))
    target = np.full((298, 211), tape, np.uint8)
    target[18:280, 18:193] = paper
    cv2.circle(target, (105, 148), 38, int(paper * .72), 1)
    return cv2.warpPerspective(target, m, (640, 480), borderValue=background), np.array([cx, cy], float), uv


def rotated_scene(angle_deg, paper=220, tape=20, background=175):
    """绕光轴面内旋转靶纸，真值 = 靶纸中心投影。"""
    a = np.deg2rad(angle_deg)
    rot = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    q = np.array([[-105, -148.5], [105, -148.5], [105, 148.5], [-105, 148.5]]) @ rot.T + [320, 240]
    h = cv2.getPerspectiveTransform(
        np.float32([[0, 0], [210, 0], [210, 297], [0, 297]]), np.float32(q))
    target = np.full((298, 211), tape, np.uint8)
    target[18:280, 18:193] = paper
    cv2.circle(target, (105, 148), 38, int(paper * .72), 1)
    img = cv2.warpPerspective(target, h, (640, 480), borderValue=background)
    return img, cv2.perspectiveTransform(np.float32([[[105, 148.5]]]), h).reshape(2)


def btrack_center(img, limit=120):
    """B 轨现行：冷启动跑到首次锁定，返回 (center or None, 累计 ms)。"""
    det = RectangleCenterDetector(Config)
    t0 = time.perf_counter()
    r = None
    for _ in range(limit):
        r = det.process(img)
        if r["valid"]:
            break
    ms = (time.perf_counter() - t0) * 1000
    return ((r["raw_u"], r["raw_v"]) if r and r.get("valid") else None), ms


# ------------------------------------------------------------------ 报告
def section_a(gy_full, gy_ds):
    print("=" * 96)
    print("【A】fixture 集（24 随机透视 + 6 面内旋转）——检出率 / 中心误差 / 单帧耗时")
    print("=" * 96)
    stat = {k: dict(hit=0, errs=[], ms=[]) for k in ("gy", "gyds", "bt")}
    for i, (img, truth) in enumerate(perf.fixtures()):
        cells = []
        for key, fn in (("gy", gy_full.detect), ("gyds", gy_ds.detect)):
            c, _q, ms = fn(img)
            stat[key]["ms"].append(ms)
            if c is None:
                cells.append("MISS -  %.1f" % ms)
            else:
                e = float(np.hypot(c[0] - truth[0], c[1] - truth[1]))
                stat[key]["hit"] += 1
                stat[key]["errs"].append(e)
                cells.append("HIT %6.3f %.1f" % (e, ms))
        cb, msb = btrack_center(img)
        stat["bt"]["ms"].append(msb)
        if cb is None:
            cells.append("MISS -  %.1f" % msb)
        else:
            e = float(np.hypot(cb[0] - truth[0], cb[1] - truth[1]))
            stat["bt"]["hit"] += 1
            stat["bt"]["errs"].append(e)
            cells.append("HIT %6.3f %.1f" % (e, msb))
        print("fixture%02d | %-22s | %-22s | %-22s" % (i, *cells))
    print("-" * 96)
    for key, name in (("gy", "国一(全分辨率)"), ("gyds", "国一(缩小320)"), ("bt", "B轨现行")):
        s = stat[key]
        print("%-14s 检出 %2d/30  最大误差 %8s  中位耗时 %.2fms" %
              (name, s["hit"], ("%.3f" % max(s["errs"])) if s["errs"] else "-", float(np.median(s["ms"]))))


def section_b(gy):
    print("\n" + "=" * 96)
    print("【B】面内旋转（绕光轴）—— 专测角点排序在 45° 附近的退化")
    print("=" * 96)
    for ang in (0, 15, 30, 42, 45, 60, 90, 135, 180):
        img, truth = rotated_scene(ang)
        cg, _q, msg = gy.detect(img)
        cb, msb = btrack_center(img)
        eg = "%.3f" % np.hypot(cg[0] - truth[0], cg[1] - truth[1]) if cg else "MISS"
        eb = "%.3f" % np.hypot(cb[0] - truth[0], cb[1] - truth[1]) if cb else "MISS"
        print("rot %3d°  国一 err=%-8s (%4.1fms)   B轨 err=%-8s (%4.1fms)" % (ang, eg, msg, eb, msb))


def section_c(gy):
    print("\n" + "=" * 96)
    print("【C】离面倾角扫描 —— 中心方案对比：逆透视 vs 质心 vs 外接框")
    print("=" * 96)
    print("%-6s | %-9s %-9s %-9s | %-9s | %-9s %-8s" %
          ("倾斜", "逆透视", "质心", "外接框", "国一检出", "B轨", "国一ms"))
    for th in (0, 15, 30, 45, 60, 70, 78):
        img, truth, quad = tilted_scene(th)
        h = cv2.getPerspectiveTransform(
            np.float32([[0, 0], [210, 0], [210, 297], [0, 297]]), np.float32(quad))
        proj = cv2.perspectiveTransform(np.float32([[[105, 148.5]]]), h).reshape(2)
        cen = quad.mean(axis=0)
        rcx, rcy = cv2.minAreaRect(np.float32(quad).reshape(-1, 1, 2))[0]
        e_p = np.hypot(proj[0] - truth[0], proj[1] - truth[1])
        e_c = np.hypot(cen[0] - truth[0], cen[1] - truth[1])
        e_b = np.hypot(rcx - truth[0], rcy - truth[1])
        cg, _q, msg = gy.detect(img)
        cb, msb = btrack_center(img)
        eg = "%.2f" % np.hypot(cg[0] - truth[0], cg[1] - truth[1]) if cg else "MISS"
        eb = "%.2f" % np.hypot(cb[0] - truth[0], cb[1] - truth[1]) if cb else "MISS"
        print("%4d°  | %8.3f %8.3f %8.3f | %9s | %8s %8.1f" % (th, e_p, e_c, e_b, eg, eb, msg))


def section_d(gy):
    print("\n" + "=" * 96)
    print("【D】端到端场景（e2e 用例：强倾斜/近距离/低对比/反光/噪声/负样本）")
    print("=" * 96)
    e2e = _load(ROOT / "tools" / "test_images_e2e.py", "e2e")
    for c in e2e.build_cases():
        img = c["img"]
        res = e2e.detect_black_frame(img)
        cg, _q, msg = gy.detect(img)
        if c["truth"] is not None:
            eb = "%.2f" % np.hypot(res.raw_u - c["truth"][0], res.raw_v - c["truth"][1]) if res else "MISS"
            eg = "%.2f" % np.hypot(cg[0] - c["truth"][0], cg[1] - c["truth"][1]) if cg else "MISS"
        else:
            eb = "FALSE+" if res else "reject"
            eg = "FALSE+" if cg else "reject"
        print("%-18s expect=%-5s | 国一 %-8s (%4.1fms) | B轨 %-8s (%4.1fms)" %
              (c["name"], "YES" if c["expect"] else "NO", eg, msg, eb, res.get("ms", 0.0)))


def section_e():
    print("\n" + "=" * 96)
    print("【E】消融：国一的两处可修点各自贡献")
    print("=" * 96)
    fit = perf.fixtures()
    for name, kw in (("国一原样(fixed+sumdiff)", {}),
                     ("面积改相对(relative)", dict(area_mode="relative")),
                     ("角点改极角(polar)", dict(order_mode="polar")),
                     ("两处都改", dict(area_mode="relative", order_mode="polar"))):
        d = GuoyiA4Detector(**kw)
        hit, errs, ms = 0, [], []
        for img, truth in fit:
            c, _q, t = d.detect(img)
            ms.append(t)
            if c is not None:
                hit += 1
                errs.append(float(np.hypot(c[0] - truth[0], c[1] - truth[1])))
        rot = []
        for ang in (45, 135):
            img, truth = rotated_scene(ang)
            c, _q, _t = d.detect(img)
            rot.append(float(np.hypot(c[0] - truth[0], c[1] - truth[1])) if c else float("inf"))
        print("%-24s 检出 %2d/30  最大误差 %8s  中位 %.2fms | 45/135°误差 %s" %
              (name, hit, ("%.3f" % max(errs)) if errs else "-", float(np.median(ms)),
               " / ".join(("%.3f" % e) if np.isfinite(e) else "MISS" for e in rot)))


# ------------------------------------------------------------------ 可视证据图
def figures():
    out = ROOT / "tools" / "e2e_report"
    out.mkdir(parents=True, exist_ok=True)
    e2e = _load(ROOT / "tools" / "test_images_e2e.py", "e2e")

    def to_bgr(img):
        g = np.asarray(img)
        return g.copy() if g.ndim == 3 else cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)

    def banner(tile, name, line2, ok):
        h, w = tile.shape[:2]
        b = np.zeros((40, w, 3), np.uint8)
        color = (0, 200, 0) if ok else (0, 0, 235)
        cv2.rectangle(b, (0, 0), (w, 3), color, -1)
        cv2.putText(b, name, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, .46, (235, 235, 235), 1)
        cv2.putText(b, line2, (6, 34), cv2.FONT_HERSHEY_SIMPLEX, .40, color, 1)
        return np.vstack([b, tile])

    # 图1 倾角演示
    tiles = []
    for th in (0, 30, 60, 78):
        img, truth, quad = tilted_scene(th)
        h = cv2.getPerspectiveTransform(
            np.float32([[0, 0], [210, 0], [210, 297], [0, 297]]), np.float32(quad))
        proj = cv2.perspectiveTransform(np.float32([[[105, 148.5]]]), h).reshape(2)
        cen = quad.mean(axis=0)
        v = to_bgr(img)
        cv2.polylines(v, [np.rint(quad).astype(np.int32)], True, (0, 255, 255), 2)
        for p in quad:
            cv2.circle(v, tuple(np.rint(p).astype(int)), 3, (255, 0, 255), -1)
        cv2.drawMarker(v, tuple(np.rint(truth).astype(int)), (0, 220, 0), cv2.MARKER_TILTED_CROSS, 16, 2)
        cv2.circle(v, tuple(np.rint(proj).astype(int)), 5, (255, 90, 0), -1)
        cv2.circle(v, tuple(np.rint(cen).astype(int)), 5, (0, 0, 235), -1)
        e_p = np.hypot(proj[0] - truth[0], proj[1] - truth[1])
        e_c = np.hypot(cen[0] - truth[0], cen[1] - truth[1])
        tiles.append(banner(cv2.resize(v, (330, 248), interpolation=cv2.INTER_AREA),
                            "tilt %d deg" % th,
                            "inverse-persp %.3f  centroid %.2f" % (e_p, e_c), e_c < 1.0))
    strip = cv2.copyMakeBorder(np.hstack(tiles), 6, 6, 6, 6, cv2.BORDER_CONSTANT, value=(32, 32, 32))
    cv2.imencode(".png", strip)[1].tofile(str(out / "guoyi_tilt_demo.png"))

    # 图2 失效案例（国一 与 B轨 并排）
    cases = [("rot45", *rotated_scene(45)), ("rot135", *rotated_scene(135))]
    cases.append(("strong_tilt", *e2e.scene(np.float32([[120, 40], [520, 60], [540, 430], [90, 410]]))))
    cases.append(("near_large", *e2e.scene(np.float32([[40, 25], [600, 30], [595, 450], [35, 445]]))))
    cases.append(("low_contrast", *e2e.scene(np.float32(e2e.BASE), low=True)))
    gy = GuoyiA4Detector()
    tiles = []
    for name, img, truth in cases:
        cg, quad, _ms = gy.detect(img)
        v = to_bgr(img)
        t = tuple(np.rint(truth).astype(int))
        cv2.drawMarker(v, t, (0, 220, 0), cv2.MARKER_TILTED_CROSS, 16, 2)
        if quad is not None:
            cv2.polylines(v, [np.rint(quad).astype(np.int32)], True, (0, 0, 235), 2)
        if cg is not None:
            cv2.circle(v, tuple(np.rint(cg).astype(int)), 6, (0, 0, 235), -1)
            e = float(np.hypot(cg[0] - truth[0], cg[1] - truth[1]))
            line2, ok = ("guoyi WRONG %.1fpx" % e, e <= 3)
        else:
            line2, ok = "guoyi MISS", False
        tiles.append(banner(cv2.resize(v, (330, 248), interpolation=cv2.INTER_AREA), name, line2, ok))
        cb, _msb = btrack_center(img)
        vb = to_bgr(img)
        cv2.drawMarker(vb, t, (0, 220, 0), cv2.MARKER_TILTED_CROSS, 16, 2)
        if cb is not None:
            cv2.circle(vb, tuple(np.rint(cb).astype(int)), 6, (255, 90, 0), -1)
            eb = float(np.hypot(cb[0] - truth[0], cb[1] - truth[1]))
            line2, ok = ("B-track %.3f px OK" % eb, eb < 3)
        else:
            line2, ok = "B-track MISS", False
        tiles.append(banner(cv2.resize(vb, (330, 248), interpolation=cv2.INTER_AREA),
                            name + " [B-track]", line2, ok))
    while len(tiles) % 4:
        tiles.append(np.zeros_like(tiles[0]))
    rows = [np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)]
    grid = cv2.copyMakeBorder(np.vstack(rows), 6, 6, 6, 6, cv2.BORDER_CONSTANT, value=(32, 32, 32))
    cv2.imencode(".png", grid)[1].tofile(str(out / "guoyi_failures.png"))
    print("figures -> %s" % out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--figures", action="store_true", help="另存两张可视证据图")
    args = ap.parse_args()
    cv2.setNumThreads(Config.OPENCV_THREADS)
    print("国一方案 vs B 轨现行 —— PC 只读对拍（D:/Python311 运行）\n")
    gy_full = GuoyiA4Detector()
    gy_ds = GuoyiA4Detector(downscale=320)
    section_a(gy_full, gy_ds)
    section_b(gy_full)
    section_c(gy_full)
    section_d(gy_full)
    section_e()
    if args.figures:
        figures()


if __name__ == "__main__":
    main()
