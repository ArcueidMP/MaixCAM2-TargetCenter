#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/profile_hotspots.py · 视觉链路逐阶段耗时剖析
================================================================================
目的：把 "0.1 fps" 拆开看清每一段到底花在哪，而不是猜。

做法：复用 test_pc.make_scene 合成一张真值已知的靶纸图，然后
  ① 端到端：连续跑 detector.process()，报告平均/中位耗时与 fps；
  ② 阶段级：用 timeit 单独测 二值化 / 形态学 / findContours / 连接域分析 /
     候选打分 / 角点提取 / 亚像素精修 / resize / ROI 精算 各占多少 ms；
  ③ 最坏情况：把 detector 的阈值锁清掉，强制它跑一遍"全扫"（THRESH_SWEEP x 形态档），
     测出真正的峰值耗时 —— 这才是上板后偶尔掉到 0.1fps 的来源。

运行：  "D:/Python311/python.exe" tools/profile_hotspots.py
参数：  --scene {easy,hard,far}   --iters N   --scale S
"""

import argparse
import os
import sys
import time

import numpy as np
import cv2

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from config import Config                                          # noqa: E402
from vision.detector import (TargetCenterDetector, contour_features,  # noqa: E402
                             select_frame_contour, refine_corners_subpixel,
                             boundary_estimate, _top_contours)
from vision import geometry as G                                   # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_pc import make_scene                                     # noqa: E402


SCENES = {
    # 名字: (dst, noise, blur)
    "easy": ([[430, 140], [850, 140], [850, 590], [430, 590]], 1.0, 0.6),
    "hard": ([[420, 110], [880, 150], [915, 610], [405, 655]], 6.0, 1.2),
    "far":  ([[600, 330], [690, 335], [690, 425], [600, 420]], 4.0, 1.0),
}


def _t(fn, n=20):
    """测 fn 的平均耗时（ms）。"""
    fn()                                   # 预热
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    return (time.perf_counter() - t0) * 1000.0 / n


def bench_stages(gray, cfg, label):
    H, W = gray.shape[:2]
    print("\n%s  图像 %dx%d" % (label, W, H))
    print("  %-42s %9s" % ("阶段", "ms"))

    det = TargetCenterDetector(cfg)
    th = int(cfg.BLACK_THRESH)
    ok_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                     (int(cfg.MORPH_OPEN_K),) * 2)
    ck_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                     (int(cfg.MORPH_CLOSE_K),) * 2)
    _, bw_raw = cv2.threshold(gray, th, 255, cv2.THRESH_BINARY_INV)

    def f_thresh():
        cv2.threshold(gray, th, 255, cv2.THRESH_BINARY_INV)

    def f_otsu():
        cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    def f_open():
        cv2.morphologyEx(bw_raw, cv2.MORPH_OPEN, ok_k)

    def f_close():
        cv2.morphologyEx(bw_raw, cv2.MORPH_CLOSE, ck_k)

    bw = det._morph(bw_raw)

    def f_contours_none():
        cv2.findContours(bw, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

    def f_contours_simple():
        cv2.findContours(bw, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    def f_resize():
        cv2.resize(gray, (W // 2, H // 2), interpolation=cv2.INTER_AREA)

    p = cv2.findContours(bw, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    cnts = p[-2]
    big = max(cnts, key=cv2.contourArea) if cnts else None

    def f_features():
        contour_features(bw, gray, cfg, W, H)

    def f_select():
        select_frame_contour(bw, gray, cfg, W, H)

    rows = [
        ("threshold (固定阈值, 全图)", f_thresh),
        ("threshold (Otsu, 全图)", f_otsu),
        ("morphology 开运算 (全图)", f_open),
        ("morphology 闭运算 (全图)", f_close),
        ("findContours NONE (全图)", f_contours_none),
        ("findContours SIMPLE (全图)", f_contours_simple),
        ("resize 全图 -> 0.5", f_resize),
        ("contour_features（打分全流程）", f_features),
        ("select_frame_contour", f_select),
    ]
    for name, fn in rows:
        print("  %-42s %9.2f" % (name, _t(fn)))

    n_pts = len(big) if big is not None else 0
    print("  %-42s %9d  (最大轮廓点数)" % ("findContours 结果规模", n_pts))

    # 后端：角点提取 / 亚像素精修（依赖一次成功的检测）
    r = TargetCenterDetector(cfg).process(gray)
    if r["valid"]:
        q = r["quad"]
        pts = None
        feats = contour_features(bw, gray, cfg, W, H)
        if feats:
            pts = feats[0]["pts"]
        else:
            sl = select_frame_contour(bw, gray, cfg, W, H)
            pts = sl[0] if sl else None
        if pts is not None:
            print("  %-42s %9.2f" % ("extract_ordered_corners", _t(
                lambda: G.extract_ordered_corners(pts))))
            print("  %-42s %9.2f" % ("refine_corners_subpixel", _t(
                lambda: refine_corners_subpixel(pts, q, cfg))))
            print("  %-42s %9d" % ("轮廓点数", len(pts)))


def bench_end_to_end(gray, cfg, iters):
    print("\n端到端（detector.process，含帧间 ROI 跟踪）")
    det = TargetCenterDetector(cfg)
    det.process(gray)
    ts, valid, stages, masks = [], 0, {}, []
    for i in range(iters):
        t0 = time.perf_counter()
        r = det.process(gray, timestamp=i / 30.0)
        ts.append((time.perf_counter() - t0) * 1000.0)
        valid += 1 if r["valid"] else 0
        if r["valid"]:
            stages[r["stage"].split("+")[0]] = stages.get(r["stage"].split("+")[0], 0) + 1
        masks.append(r.get("masks", 0))
    ts = np.array(ts)
    print("  %-42s %9.2f ms  (中位 %.2f, 最大 %.2f)" % (
        "稳态（阈值已锁定 + ROI 跟踪）", ts.mean(), np.median(ts), ts.max()))
    print("  %-42s %9s" % ("检出率", "%d/%d" % (valid, iters)))
    print("  %-42s %9.2f fps" % ("对应帧率(仅算法)", 1000.0 / max(1e-6, np.median(ts))))
    print("  %-42s %9s" % ("解算路径分布", stages or "-"))
    print("  %-42s %9.1f (最大 %d)" % ("每帧掩膜数 平均", float(np.mean(masks)),
                                       int(np.max(masks))))
    return ts.mean()


def bench_no_track(gray, cfg, iters):
    """关掉 ROI 跟踪，量出它到底省了多少。"""
    old = getattr(cfg, "ROI_TRACK", True)
    cfg.ROI_TRACK = False
    try:
        det = TargetCenterDetector(cfg)
        det.process(gray)
        ts = []
        for i in range(iters):
            t0 = time.perf_counter()
            det.process(gray, timestamp=i / 30.0)
            ts.append((time.perf_counter() - t0) * 1000.0)
        return float(np.median(ts))
    finally:
        cfg.ROI_TRACK = old


def bench_worst_case(gray, cfg):
    """强制解锁 -> 每帧都做满扫描，暴露峰值耗时。"""
    print("\n最坏情况（强制全扫：THRESH_SWEEP x 形态档）")
    det = TargetCenterDetector(cfg)
    specs, n_pri = det._mask_specs(gray, force=True)
    n_spec = len(specs)
    ts = []
    for _ in range(3):
        det.reset()
        t0 = time.perf_counter()
        det._th_lock = None
        det._detect_on(gray, "worst", force_sweep=True)
        ts.append((time.perf_counter() - t0) * 1000.0)
    print("  %-42s %9d  (其中必试 %d)" % ("一次全扫要过的 (阈值,形态) 组合数",
                                          n_spec, n_pri))
    print("  %-42s %9.2f ms" % ("全分辨率一次全扫（不限预算）", float(np.mean(ts))))
    print("  %-42s %9.2f fps" % ("若每帧都全扫", 1000.0 / max(1e-6, float(np.mean(ts)))))
    # 带每帧掩膜预算（板上实际用的配置）
    ts = []
    for _ in range(3):
        det.reset()
        t0 = time.perf_counter()
        det._th_lock = None
        det._detect_on(gray, "worst", force_sweep=True,
                       budget=int(getattr(cfg, "MAX_MASKS_FULL", 8)))
        ts.append((time.perf_counter() - t0) * 1000.0)
    print("  %-42s %9.2f ms" % ("全帧预算=%d 时" % int(getattr(cfg, "MAX_MASKS_FULL", 8)),
                                float(np.mean(ts))))
    return float(np.mean(ts))


def bench_pathological(cfg):
    """
    病态掩膜回归：真实相机的噪声/纹理二值图。

    这是本项目"0.1fps"的真凶所在 —— 纹理/噪点掩膜上一张就有 2 万条轮廓，
    旧实现对每条做 contourArea + minAreaRect（内部要算凸包），实测 260ms（PC）。
    修复后靠"按轮廓点数先截断"把这段压到有上界。**这个测试必须一直保持绿色**。

    注意 lvl=1（精细形态档）**不做开运算**，噪点原样保留，是最容易爆的一档 ——
    而 MORPH_SEARCH=True 时它是每轮扫描的一部分，所以这条路径一定会被走到。
    """
    W, H = 1280, 720
    rng = np.random.default_rng(0)

    def _morph(bw, lvl):
        if lvl == 0:
            bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (3, 3)))
            return cv2.morphologyEx(bw, cv2.MORPH_CLOSE, cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (9, 9)))
        return cv2.morphologyEx(bw, cv2.MORPH_CLOSE, cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (3, 3)))

    def _texture(scale, seed):
        r = np.random.default_rng(seed)
        small = r.normal(128, 48, (H // scale + 1, W // scale + 1)).clip(0, 255)
        return cv2.resize(small.astype(np.uint8), (W, H), interpolation=cv2.INTER_LINEAR)

    cases = {
        "白噪声": (rng.normal(140, 55, (H, W)).clip(0, 255)).astype(np.uint8),
        "纹理/4px": _texture(4, 5),
        "暗背景": (rng.normal(40, 12, (H, W)).clip(0, 255)).astype(np.uint8),
    }

    print("\n" + "=" * 92)
    print(" 病态掩膜回归（真实相机噪声/纹理；旧实现在这类掩膜上单张最慢 293ms / PC）")
    print("=" * 92)
    print("  %-12s %5s %4s %9s %10s %10s %10s %9s" %
          ("场景", "阈值", "形态", "轮廓数", "总点数", "边界估计", "送入cv2", "features"))
    worst = 0.0
    for name, g in cases.items():
        for th in (95, 150, 205):
            for lvl in (0, 1):
                _, bw = cv2.threshold(g, th, 255, cv2.THRESH_BINARY_INV)
                bw = _morph(bw, lvl)
                if cv2.countNonZero(bw) == 0:
                    continue
                cnts = cv2.findContours(bw, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)[-2]
                pts = sum(len(c) for c in cnts)
                est = boundary_estimate(bw)
                n_scan = len(list(_top_contours(cnts, cfg.MAX_SCAN_CONTOURS)))
                t_feat = _t(lambda: contour_features(bw, g, cfg, W, H), n=2)
                worst = max(worst, t_feat)
                route = "预筛" if est > cfg.FRAG_BOUNDARY_EST else "常规"
                print("  %-12s %5d %4d %9d %10d %10d %10d %7.1fms %s"
                      % (name, th, lvl, len(cnts), pts, est, n_scan, t_feat, route))
    print("-" * 92)
    print("  单张掩膜最慢 %.1fms（旧实现同场景最慢 293ms / PC；板上要再乘十几倍）" % worst)
    print("=" * 92)
    return worst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="hard", choices=list(SCENES))
    ap.add_argument("--iters", type=int, default=30)
    ap.add_argument("--scale", type=float, default=None)
    args = ap.parse_args()

    dst, noise, blur = SCENES[args.scene]
    _, gray, truth, _ = make_scene(dst=dst, seed=1, noise=noise, blur=blur)

    ws = args.scale if args.scale is not None else Config.profile()[6]
    Config.WORK_SCALE = ws

    print("=" * 78)
    print(" 视觉链路性能剖析   scene=%s  WORK_SCALE=%.2f" % (args.scene, ws))
    print("=" * 78)

    bench_stages(gray, Config, "【全分辨率 1280x720】")
    med_track = bench_end_to_end(gray, Config, args.iters)
    med_notrack = bench_no_track(gray, Config, args.iters)
    print("  %-42s %9.2f ms  (ROI 跟踪省掉 %.0f%%)"
          % ("关掉 ROI_TRACK 的稳态中位", med_notrack,
             100.0 * (med_notrack - med_track) / max(1e-9, med_notrack)))
    bench_worst_case(gray, Config)
    bench_pathological(Config)

    # 顺带看看 coarse 那一遍的规模
    sw, sh = int(gray.shape[1] * ws), int(gray.shape[0] * ws)
    small = cv2.resize(gray, (sw, sh), interpolation=cv2.INTER_AREA)
    print("\n【缩小图 %dx%d = 0.5 倍】" % (sw, sh))
    det = TargetCenterDetector(Config)
    _, bw_s = cv2.threshold(small, Config.BLACK_THRESH, 255, cv2.THRESH_BINARY_INV)
    bw_s = det._morph(bw_s)
    print("  %-42s %9.2f" % ("coarse: threshold+morph+findContours+打分",
                             _t(lambda: select_frame_contour(bw_s, small, Config, sw, sh))))
    print("=" * 78)


if __name__ == "__main__":
    main()
