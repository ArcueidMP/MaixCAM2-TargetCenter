#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/test_jitter.py · 抖动（小车行进/手持晃动）下的锁框稳定性回归
================================================================================
真机症状：帧率正常、也能检出，但**抖动时锁框中心会明显偏移**：
    u=488.9 v=482.2  22.47 px/cm  候选=ring
    u=539.1 v=417.3  19.37 px/cm  候选=ring
    u=539.3 v=384.5  12.51 px/cm  候选=ring     ← px/cm 一夜之间掉 1.8 倍
    u=398.0 v=475.4  13.31 px/cm  候选=paper    ← 候选族还在 ring/paper 之间跳
中心在几十~上百像素之间来回蹦，云台照着这个值打就废了。

本脚本用**合成场景**复现它，并给出可量化的对照（真值已知）：
  · 相机抖动  ：每帧对整幅做「绕中心小角度旋转 + 平移」，振幅按正弦 + 随机游走，
                 并额外插入"小车起步/过坎"式的大幅冲击（impact 帧）
  · 运动模糊  ：沿本帧晃动方向做定向模糊（抖动的主要光学后果；曝光 6ms 时很明显）
  · 干扰物    ：可选周期性出现的同尺寸深色矩形，制造"偶发误锁"

逐帧统计（对比 `TRACK_ENABLE` 开/关）：
    中心误差   |检测中心 - 真值|  的 均值 / 最大      ← 反映"偏移"
    帧间跳变   |本帧 - 上帧|      的 均值 / 最大      ← 反映"抖"
    px/cm 波动 尺度相对中位数的最大偏离           ← 反映"尺度乱跳"
    检出率

运行：
    "D:/Python311/python.exe" tools/test_jitter.py            # 开/关跟踪对照
    "D:/Python311/python.exe" tools/test_jitter.py --frames 160 --impact 0.35
输出：tools/jitter_montage.jpg（若干帧的检出叠加图，肉眼看偏移）
"""

import argparse
import os
import sys

import numpy as np
import cv2

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from config import Config                                       # noqa: E402
from vision.detector import TargetCenterDetector                # noqa: E402
from vision import geometry as G                                 # noqa: E402

A4W, A4H = 21.0, 29.7
W, H = 1280, 720
S = 40.0                                    # cm 画布分辨率 px/cm
OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# 靶纸基础姿态（占屏中等偏大：和真机日志里 px/cm 12~22 的量级一致）
DST_DEF = [[420, 110], [880, 150], [915, 610], [405, 655]]


def make_target(seed=1, tape_refl=0.07, paper_refl=0.85, wall=0.62, illum=300.0):
    """
    生成一张"靶纸在画面里"的 BGR 场景（不含抖动）。
    返回 (scene_bgr, truth_center_xy, dst)。
    反射率模型与 test_hardscene 一致：黑胶带 0.07 / 纸面 0.85 / 墙 0.62。
    """
    rng = np.random.default_rng(seed)
    cw, ch = int(A4W * S), int(A4H * S)
    refl = np.full((ch, cw), paper_refl, np.float32)
    t = int(1.8 * S)
    refl[:t, :] = tape_refl
    refl[-t:, :] = tape_refl
    refl[:, :t] = tape_refl
    refl[:, -t:] = tape_refl

    dst = np.float32(DST_DEF)
    Ms = cv2.getPerspectiveTransform(
        np.float32([[0, 0], [cw, 0], [cw, ch], [0, ch]]), dst)
    wr = cv2.warpPerspective(refl, Ms, (W, H), flags=cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_CONSTANT, borderValue=0.0)
    m = np.clip(cv2.warpPerspective(np.ones((ch, cw), np.float32), Ms, (W, H),
                                    flags=cv2.INTER_LINEAR,
                                    borderMode=cv2.BORDER_CONSTANT,
                                    borderValue=0.0), 0.0, 1.0)

    scene = (wall * (1.0 - m) + wr * m) * illum
    scene = np.clip(scene, 0, 255).astype(np.uint8)
    scene = cv2.GaussianBlur(scene, (0, 0), 0.8)
    scene = np.clip(scene.astype(np.float32) + rng.normal(0, 2.0, scene.shape),
                    0, 255).astype(np.uint8)
    scene = cv2.cvtColor(scene, cv2.COLOR_GRAY2BGR)

    truth = project(Ms, (cw - 1) / 2.0, (ch - 1) / 2.0)
    return scene, truth, dst


def project(Hm, x, y):
    v = Hm @ np.array([x, y, 1.0], dtype=np.float64)
    return np.array([v[0] / v[2], v[1] / v[2]])


def shake(i, rng, amp, impact=0.0):
    """
    第 i 帧的"相机抖动"参数：(dx, dy, dtheta_deg, blur_len, blur_ang_deg)

    模型：低频正弦（小车行进时的车身晃动）+ 随机游走（高频手抖）
          + 偶发冲击（过坎/起步，`impact` 是冲击帧的比例）。
    """
    t = i / 30.0
    dx = amp * (9.0 * np.sin(2 * np.pi * 1.7 * t) + 5.0 * np.sin(2 * np.pi * 4.3 * t + 1.1))
    dy = amp * (7.0 * np.sin(2 * np.pi * 2.3 * t + 0.6) + 4.0 * np.sin(2 * np.pi * 5.1 * t))
    th = amp * (0.55 * np.sin(2 * np.pi * 1.9 * t + 0.3))
    dx += rng.normal(0, 1.6 * amp)
    dy += rng.normal(0, 1.6 * amp)
    th += rng.normal(0, 0.10 * amp)

    if impact > 0 and rng.random() < impact:       # 冲击帧：大幅位移 + 强模糊
        k = rng.uniform(1.6, 2.6)
        dx *= k
        dy *= k
        th *= k

    blur = min(15.0, 1.2 + 0.10 * float(np.hypot(dx, dy)) + abs(th) * 2.2)
    ang = float(np.degrees(np.arctan2(dy, dx)))
    return float(dx), float(dy), float(th), float(blur), ang


def warp_frame(scene, dx, dy, dtheta_deg, blur, ang, rng, noise=3.0, distractor=None):
    """把静态场景按抖动参数变换 + 定向模糊 + 加噪；返回 (gray, 附加变换矩阵)。"""
    M = cv2.getRotationMatrix2D((W / 2.0, H / 2.0), dtheta_deg, 1.0)
    M[0, 2] += dx
    M[1, 2] += dy
    out = cv2.warpAffine(scene, M, (W, H), flags=cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_REPLICATE)
    if distractor is not None:
        cv2.rectangle(out, distractor[0], distractor[1], (18, 18, 18), -1)
    if blur >= 2.0:                                # 定向运动模糊
        k = int(round(blur)) | 1
        ker = np.zeros((k, k), np.float32)
        ker[k // 2, :] = 1.0 / k
        Mr = cv2.getRotationMatrix2D((k / 2.0 - 0.5, k / 2.0 - 0.5), ang, 1.0)
        ker = cv2.warpAffine(ker, Mr, (k, k))
        s = ker.sum()
        if s > 1e-6:
            ker /= s
        out = cv2.filter2D(out, -1, ker)
    g = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
    return np.clip(g.astype(np.float32) + rng.normal(0, noise, g.shape),
                   0, 255).astype(np.uint8), M


def run_seq(track_on, frames=140, amp=1.0, impact=0.06, seed=11,
            distractor_period=0, noise=3.0):
    """跑一整段抖动序列，返回逐帧统计。"""
    rng = np.random.default_rng(seed)
    scene, truth0, _ = make_target(seed=1)
    old = getattr(Config, "TRACK_ENABLE", True)
    Config.TRACK_ENABLE = track_on
    Config.WORK_SCALE = Config.profile()[6]
    det = TargetCenterDetector(Config)
    rec = []
    try:
        for i in range(frames):
            dx, dy, th, blur, ang = shake(i, rng, amp, impact)
            dis = None
            if distractor_period and (i % distractor_period) == 0 and i > 0:
                dis = ((560, 240), (860, 540))      # 与靶纸同量级的深色矩形
            g, M = warp_frame(scene, dx, dy, th, blur, ang, rng, noise, dis)
            # 真值也随抖动一起变换（相机动了，靶心的像也动）
            v = M @ np.array([truth0[0], truth0[1], 1.0])
            truth = np.array([v[0], v[1]])
            r = det.process(g, timestamp=i / 30.0)
            if not r["valid"]:
                rec.append(dict(ok=False))
                continue
            ru = r["raw_u"] if r.get("raw_u") is not None else r["u"]
            rv = r["raw_v"] if r.get("raw_v") is not None else r["v"]
            rec.append(dict(
                ok=True,
                raw=np.array([ru, rv]),
                raw_err=float(np.hypot(ru - truth[0], rv - truth[1])),
                out=np.array([r["u"], r["v"]]),
                err=float(np.hypot(r["u"] - truth[0], r["v"] - truth[1])),
                scale=float(r["scale"]), held=bool(r.get("held")),
                blur=blur, stage=r["stage"]))
    finally:
        Config.TRACK_ENABLE = old
    return rec


def stats(rec):
    ok = [r for r in rec if r["ok"]]
    if len(ok) < 3:
        return None
    raw_err = np.array([r["raw_err"] for r in ok])
    f_err = np.array([r["err"] for r in ok])
    raw_out = np.array([r["raw"] for r in ok])
    f_out = np.array([r["out"] for r in ok])
    sc = np.array([r["scale"] for r in ok])
    med = float(np.median(sc)) or 1.0
    jump = lambda a: np.hypot(np.diff(a[:, 0]), np.diff(a[:, 1]))
    return dict(
        n=len(rec), hit=len(ok),
        raw_err_mean=float(raw_err.mean()), raw_err_max=float(raw_err.max()),
        err_mean=float(f_err.mean()), err_max=float(f_err.max()),
        raw_jump_mean=float(jump(raw_out).mean()), raw_jump_max=float(jump(raw_out).max()),
        jump_mean=float(jump(f_out).mean()), jump_max=float(jump(f_out).max()),
        scale_dev=float(np.max(np.abs(sc - med)) / med),
        held=int(sum(1 for r in ok if r["held"])))


def report(track_on, **kw):
    rec = run_seq(track_on, **kw)
    st = stats(rec)
    tag = "开(TRACK_ENABLE=True)" if track_on else "关(TRACK_ENABLE=False)"
    if st is None:
        print("  %-24s 检出太少，无法统计" % tag)
        return st, rec
    print("  %-24s 检出%3d/%3d | 中心误差 均值%7.2f 最大%8.2f px | "
          "帧间跳变 均值%6.2f 最大%7.2f px | 尺度最大偏离%6.1f%% | 保持%d帧"
          % (tag, st["hit"], st["n"], st["err_mean"], st["err_max"],
             st["jump_mean"], st["jump_max"], st["scale_dev"] * 100, st["held"]))
    print("  %-24s                  （同期**原始测量**：中心误差 均值%7.2f 最大%8.2f px | "
          "帧间跳变 均值%6.2f 最大%7.2f px）"
          % ("", st["raw_err_mean"], st["raw_err_max"],
             st["raw_jump_mean"], st["raw_jump_max"]))
    return st, rec


def tracker_unit(N=240, n_out=14, out_px=(60.0, 160.0), seed=5, fps=30.0):
    """
    跟踪器**单元测试**（不经过图像，结论完全确定）。

    喂一条"已知的平滑运动轨迹 + 偶发大幅离群测量"，直接检验跟踪器的两条契约：
        ① 平常跟得准（无滞后）—— 输出误差应与"原始测量"同量级，不能因为滤波而滞后
        ② 离群被挡住（不跳）  —— 离群帧的输出不能跟着跳，误差应显著小于原始测量
    同时给出 One Euro(static) 的对照 —— 那就是优化前真机在用的滤波器，
    "车一晃中心就偏" 就是它造成的。
    """
    from vision.tracker import TargetTracker
    rng = np.random.default_rng(seed)
    t = np.arange(N) / fps
    truth = np.stack([500.0 + 60.0 * np.sin(2 * np.pi * 1.2 * t)
                      + 20.0 * np.sin(2 * np.pi * 3.7 * t),
                      360.0 + 45.0 * np.sin(2 * np.pi * 0.9 * t + 0.7)
                      + 15.0 * np.sin(2 * np.pi * 4.1 * t)], axis=1)
    meas = truth + rng.normal(0, 0.5, truth.shape)
    out_idx = rng.choice(np.arange(3, N - 3), size=n_out, replace=False)
    bad = np.zeros(N, bool)
    for i in out_idx:
        ang = rng.uniform(0, 2 * np.pi)
        d = rng.uniform(*out_px)
        meas[i] = truth[i] + np.array([d * np.cos(ang), d * np.sin(ang)])
        bad[i] = True

    quad = np.array([[0.0, 0.0], [470.0, 0.0], [470.0, 620.0], [0.0, 620.0]])

    tk = TargetTracker(Config)
    one = G.OneEuroFilter(freq=fps, **Config.FILTER_PARAMS["static"])
    o1 = G.OneEuroFilter(freq=fps, **Config.FILTER_PARAMS["static"])
    o2 = G.OneEuroFilter(freq=fps, **Config.FILTER_PARAMS["static"])
    rows = {"原始测量": [], "本跟踪器": [], "OneEuro(旧)": []}
    for i in range(N):
        ts = i / fps
        r = tk.update(meas[i, 0], meas[i, 1], 20.0, quad, ts, sharp=100.0, quality=1.0)
        rows["原始测量"].append(meas[i] - truth[i])
        rows["本跟踪器"].append(np.array([r["u"], r["v"]]) - truth[i])
        rows["OneEuro(旧)"].append(np.array([o1(meas[i, 0], ts), o2(meas[i, 1], ts)])
                                   - truth[i])
    print("=" * 118)
    print(" 跟踪器单元测试（%d 帧 @%.0fHz，其中 %d 帧注入 %.0f~%.0fpx 离群测量；"
          "目标框 470x620px）" % (N, fps, n_out, out_px[0], out_px[1]))
    print("=" * 118)
    print("  %-14s %12s %12s %14s %14s" %
          ("方案", "平常误差均值", "平常误差最大", "离群帧误差均值", "离群帧误差最大"))
    keep = ~bad
    for name, d in rows.items():
        d = np.array(d)
        e = np.hypot(d[:, 0], d[:, 1])
        print("  %-14s %11.2f %12.2f %14.2f %14.2f" %
              (name, e[keep].mean(), e[keep].max(), e[bad].mean(), e[bad].max()))
    print("-" * 118)
    d = np.array(rows["本跟踪器"])
    e = np.hypot(d[:, 0], d[:, 1])
    print("  结论：平常（跟手）%.2fpx —— 与原始测量同量级即说明**没有滞后**；"
          "离群帧 %.2fpx —— 远小于原始测量的 %.2fpx 即说明**门限在起作用**"
          % (e[keep].mean(), e[bad].mean(),
             np.hypot(np.array(rows["原始测量"])[bad, 0],
                      np.array(rows["原始测量"])[bad, 1]).mean()))
    print("=" * 118)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=140)
    ap.add_argument("--amp", type=float, default=1.0)
    ap.add_argument("--impact", type=float, default=0.06)
    ap.add_argument("--distractor", type=int, default=0,
                    help="每 N 帧插入一个更大更强的深色干扰物（0=不插）")
    ap.add_argument("--noise", type=float, default=3.0, help="帧噪声 sigma")
    ap.add_argument("--unit", action="store_true", help="只跑跟踪器单元测试")
    args = ap.parse_args()

    if args.unit:
        tracker_unit()
        return 0

    kw = dict(frames=args.frames, amp=args.amp, impact=args.impact,
              distractor_period=args.distractor, noise=args.noise)
    print("=" * 118)
    print(" 抖动稳定性回归  （%d 帧，抖动幅度 x%.2f，冲击帧占比 %.0f%%，噪声 sigma=%.1f，%s）"
          % (args.frames, args.amp, args.impact * 100, args.noise,
             "含干扰物" if args.distractor else "无干扰物"))
    print("=" * 118)
    print("  条目说明：中心误差 = |检测-真值|（偏移）；帧间跳变 = 相邻帧输出之差（抖）")
    print("-" * 118)
    s_off, rec_off = report(False, **kw)
    s_on, rec_on = report(True, **kw)
    print("-" * 118)
    if s_off and s_on:
        print("  开跟踪相对关跟踪：中心误差最大 %.2f -> %.2f px（降 %.0f%%）  "
              "帧间跳变最大 %.2f -> %.2f px（降 %.0f%%）  尺度偏离 %.1f%% -> %.1f%%"
              % (s_off["err_max"], s_on["err_max"],
                 100.0 * (s_off["err_max"] - s_on["err_max"]) / max(1e-9, s_off["err_max"]),
                 s_off["jump_max"], s_on["jump_max"],
                 100.0 * (s_off["jump_max"] - s_on["jump_max"]) / max(1e-9, s_off["jump_max"]),
                 s_off["scale_dev"] * 100, s_on["scale_dev"] * 100))
    print("=" * 118)
    return 0


if __name__ == "__main__":
    sys.exit(main())
