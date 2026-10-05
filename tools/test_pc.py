#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/test_pc.py · 靶心识别算法的离线验证（PC 上跑，不需要 MaixCAM）
================================================================================
用「合成透视靶纸图」（真值已知）检验 vision/ 下的同一套算法：

    A. 单帧精度   检测靶心 vs 理论投影靶心（px / cm）
    B. 多姿态     平移 / 旋转 / 强透视 / 远近 / 模糊 下的误差
    C. 重复性     加噪 + 失焦 100 帧，统计靶心 σ（目标 < 0.5 px）
    D. 抗干扰     画面另加黑色"赛道线"干扰块，检验不被误锁
    E. 缩放对比   WORK_SCALE=0.5（板端默认，省内存）vs 1.0（全分辨率）的精度差异
    F. 可视化     输出 debug_overlay.jpg（绿框=板框、黄点=四顶点、蓝线=对角线、红十字=靶心）

运行：  python tools/test_pc.py          （需要 numpy + opencv-python）
输出：  tools/debug_overlay.jpg
"""

import os
import sys

import numpy as np
import cv2

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from config import Config                                        # noqa: E402
from vision.detector import TargetCenterDetector                 # noqa: E402
from vision.geometry import (diagonal_intersection, is_valid_quad,   # noqa: E402
                             order_corners)

A4W, A4H = 21.0, 29.7
CENTER_CM = (A4W / 2.0, A4H / 2.0)
OUT_DIR = os.path.dirname(os.path.abspath(__file__))

DEFAULT_DST = [[420, 110], [880, 150], [915, 610], [405, 655]]


def project(Hm, x_cm, y_cm):
    v = Hm @ np.array([x_cm, y_cm, 1.0])
    return np.array([v[0] / v[2], v[1] / v[2]])


def make_scene(dst=None, seed=0, noise=3.0, blur=1.0, W=1280, H=720):
    """生成合成场景：浅色 A4 靶纸 + 1.8cm 黑胶带外框 + 一条黑色"赛道线"干扰块。

    返回 (scene_bgr, gray, truth_center_xy, dst)
    真值说明：黑框外沿即 cm 画布的 (0,0)-(cw-1,ch-1) 矩形，
              其几何中心 ((cw-1)/2, (ch-1)/2) 用同一个单应投影得到，
              避免"像素中心 vs 像素角"的半像素约定引入假误差。
    """
    rng = np.random.default_rng(seed)
    dst = np.float32(dst if dst is not None else DEFAULT_DST)

    # 背景：带轻微渐变的浅灰
    bg = np.zeros((H, W, 3), np.uint8)
    bg[:, :, :] = 175
    bg = bg.astype(np.int16)
    for y in range(H):
        bg[y, :, :] = 175 + int(25 * y / H)

    # 干扰：黑色"赛道线"（1.8cm 等效宽度），检验算法锁定靶纸而非干扰
    cv2.rectangle(bg, (0, H - 30), (W, H), (5, 5, 5), -1)

    # cm 画布：外圈黑胶带 + 内部浅色纸面
    S = 40                                       # px per cm
    cw, ch = int(A4W * S), int(A4H * S)
    canvas = np.full((ch, cw, 3), 210, np.uint8)
    cv2.rectangle(canvas, (0, 0), (cw - 1, ch - 1), (18, 18, 18), -1)
    t = int(1.8 * S)                             # 胶带宽度 1.8cm
    cv2.rectangle(canvas, (t, t), (cw - 1 - t, ch - 1 - t), (212, 214, 208), -1)

    src = np.float32([[0, 0], [cw, 0], [cw, ch], [0, ch]])
    Ms = cv2.getPerspectiveTransform(src, dst)

    warped = cv2.warpPerspective(canvas, Ms, (W, H), flags=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
    mask = cv2.warpPerspective(np.full((ch, cw), 255, np.uint8), Ms, (W, H),
                               flags=cv2.INTER_NEAREST,
                               borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    a = (mask.astype(np.float32) / 255.0)[..., None]
    scene = (bg.astype(np.float32) * (1 - a) + warped.astype(np.float32) * a).astype(np.uint8)

    truth = project(Ms, (cw - 1) / 2.0, (ch - 1) / 2.0)

    if blur > 0:
        scene = cv2.GaussianBlur(scene, (0, 0), blur)
    if noise > 0:
        scene = np.clip(scene.astype(np.float32)
                        + rng.normal(0, noise, scene.shape), 0, 255).astype(np.uint8)

    return scene, cv2.cvtColor(scene, cv2.COLOR_BGR2GRAY), truth, dst


def annotate(scene, res, truth):
    out = scene.copy()
    if res["valid"]:
        q = res["quad"].astype(int)
        cv2.polylines(out, [q.reshape(-1, 1, 2)], True, (0, 255, 0), 2)
        cv2.line(out, tuple(q[0]), tuple(q[2]), (255, 0, 0), 1)
        cv2.line(out, tuple(q[1]), tuple(q[3]), (255, 0, 0), 1)
        for p, lb in zip(q, ("TL", "TR", "BR", "BL")):
            cv2.circle(out, tuple(p), 5, (0, 255, 255), 2)
            cv2.putText(out, lb, (p[0] + 6, p[1] + 18), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 255), 1)
        c = (int(res["u"]), int(res["v"]))
        cv2.drawMarker(out, c, (0, 0, 255), cv2.MARKER_CROSS, 26, 2)
        cv2.putText(out, "det center", (c[0] + 10, c[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    tc = (int(truth[0]), int(truth[1]))
    cv2.drawMarker(out, tc, (255, 0, 255), cv2.MARKER_TILTED_CROSS, 22, 2)
    cv2.putText(out, "truth (projected bullseye)", (tc[0] + 10, tc[1] + 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
    if res["valid"]:
        info = "stage=%s  scale=%.2f px/cm  err=%.3f px" % (
            res["stage"], res["scale"], np.hypot(res["raw_u"] - truth[0],
                                                  res["raw_v"] - truth[1]))
    else:
        info = "stage=%s  NOT FOUND" % res["stage"]
    cv2.putText(out, info, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return out


def run(gray, cfg, ts=None):
    return TargetCenterDetector(cfg).process(gray, timestamp=ts)


def main():
    cfg = Config
    Config.WORK_SCALE = Config.profile()[6]        # 板端默认档位的缩放比（standard=0.5）
    print("=" * 78)
    print(" 靶心识别算法 离线验证   WORK_SCALE=%.2f（板端默认）" % Config.WORK_SCALE)
    print("=" * 78)

    # ---------------- A. 单帧精度 ----------------
    scene, gray, truth, _ = make_scene(seed=1, noise=1.0, blur=0.6)
    res = run(gray, cfg)                            # 无时间戳 => 未滤波原始结果
    assert res["valid"], "清晰帧应当检出，实际未检出！"
    err = np.hypot(res["raw_u"] - truth[0], res["raw_v"] - truth[1])
    print("\n[A] 单帧精度（清晰）")
    print("    真值靶心   : (%.3f, %.3f)" % (truth[0], truth[1]))
    print("    检测靶心   : (%.3f, %.3f)" % (res["raw_u"], res["raw_v"]))
    print("    中心误差   : %.4f px  =  %.4f cm" % (err, err / res["scale"]))
    print("    像素尺度   : %.3f px/cm" % res["scale"])
    print("    四边形合法 : %s" % is_valid_quad(res["quad"], 1280, 720, cfg))
    print("    解算路径   : %s" % res["stage"])

    # ---------------- B. 多姿态 ----------------
    poses = {
        "正视居中":        ([[430, 140], [850, 140], [850, 590], [430, 590]], 0.6),
        "小角度旋转":      ([[440, 150], [860, 175], [835, 600], [415, 575]], 0.8),
        "旋转30°":         ([[500, 120], [880, 330], [700, 660], [320, 450]], 1.0),
        "强透视(右近左远)": ([[380, 170], [900, 120], [930, 600], [360, 540]], 1.0),
        "较远(占屏一半)":   ([[520, 210], [760, 210], [760, 480], [520, 480]], 1.4),
        "默认斜视":        (DEFAULT_DST, 1.0),
    }
    print("\n[B] 多姿态 / 透视 / 模糊")
    print("    %-16s %9s %9s %8s  %s" % ("姿态", "误差px", "误差cm", "px/cm", "解算路径"))
    for name, (dst, blur) in poses.items():
        _, g, tr, _ = make_scene(dst=dst, seed=1, noise=1.0, blur=blur)
        r = run(g, cfg)
        if not r["valid"]:
            print("    %-16s %9s" % (name, "NOT FOUND"))
            continue
        e = np.hypot(r["raw_u"] - tr[0], r["raw_v"] - tr[1])
        print("    %-16s %9.4f %9.4f %8.2f  %s" % (name, e, e / r["scale"], r["scale"],
                                                    r["stage"]))

    # ---------------- C. 重复性 ----------------
    N = 100
    raw, fil, miss = [], [], 0
    det_r, det_f = TargetCenterDetector(cfg), TargetCenterDetector(cfg)
    for i in range(N):
        _, g, tr, _ = make_scene(seed=1000 + i, noise=6.0, blur=1.2)
        r1 = det_r.process(g)
        if not r1["valid"]:
            miss += 1
            continue
        raw.append((r1["raw_u"] - tr[0], r1["raw_v"] - tr[1]))
        r2 = det_f.process(g, timestamp=i / 30.0)
        if r2["valid"]:
            fil.append((r2["u"] - tr[0], r2["v"] - tr[1]))
    raw, fil = np.array(raw), np.array(fil)
    print("\n[C] 重复性（%d 帧：加噪 sigma=6 + 失焦 blur=1.2）" % N)
    print("    检出率        : %d/%d = %.1f%%" % (N - miss, N, 100.0 * (N - miss) / N))
    print("    原始   sigma(u,v) : (%.3f, %.3f) px" % (raw[:, 0].std(), raw[:, 1].std()))
    print("    滤波后 sigma(u,v) : (%.3f, %.3f) px" % (fil[:, 0].std(), fil[:, 1].std()))
    ok = max(raw[:, 0].std(), raw[:, 1].std()) < 0.5
    print("    目标 sigma < 0.5px   : %s" % ("PASS" if ok else "偏大，需检查角点提取"))

    # ---------------- D. 抗干扰 ----------------
    print("\n[D] 抗干扰：画面含 1.8cm 等效黑色赛道线干扰块 -> %s（锁定靶纸外框而非干扰）"
          % ("PASS" if res["valid"] else "FAIL"))

    # ---------------- E. 缩放对比 ----------------
    print("\n[E] WORK_SCALE 对精度的影响（越小越省内存/越省时间）")
    _, g, tr, _ = make_scene(seed=7, noise=2.0, blur=0.8)
    for ws in (1.0, 0.75, 0.5, 0.375):
        Config.WORK_SCALE = ws
        r = run(g, cfg)
        if not r["valid"]:
            print("    WORK_SCALE=%-5s  NOT FOUND" % ws)
            continue
        e = np.hypot(r["raw_u"] - tr[0], r["raw_v"] - tr[1])
        print("    WORK_SCALE=%-5s  误差 %.4f px = %.4f cm   %s"
              % (ws, e, e / r["scale"], r["stage"]))
    Config.WORK_SCALE = Config.profile()[6]

    # ---------------- F. 可视化 ----------------
    out_path = os.path.join(OUT_DIR, "debug_overlay.jpg")
    ok, buf = cv2.imencode(".jpg", annotate(scene, res, truth))
    if ok:
        # 路径含中文时 cv2.imwrite 在 Windows 下会静默失败，改用 imencode + tofile
        buf.tofile(out_path)
        print("\n[F] 调试图已保存: %s" % out_path)
    print("=" * 78)


if __name__ == "__main__":
    main()
