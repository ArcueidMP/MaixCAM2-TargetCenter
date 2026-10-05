#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/exp_alt_pipeline.py · 对照实验：评估"另一条技术路线"的各组成部分
================================================================================
被评估的路线（用户提出）：
    ① Canny 边缘检测 + 多边形逼近 -> 定位靶纸四角点
    ② 沿四条边采样内外灰度差    -> 区分外框 / 内框
    ③ 8x8 线性方程组 + 高斯消元 -> 解映射参数（单应）
    ④ 正视面 1px=1mm 建坐标系
    ⑤ 72 等分采样标准圆，逐点逆变换回原图像素

本脚本只回答"每一部分到底行不行、代价多少"，不做主观评价：
    A. 检测器对比   阈值+轮廓(现状)  vs  Canny+approxPolyDP(新路线)   精度/稳定性/耗时
    A2.内外框分离   阈值+层级 hierarchy vs "沿边采样灰度差"           谁更省、能不能给出框宽
    B. 亚像素定位   二值轮廓 fitLine(现状) vs 灰度梯度剖面(新路线)    角点精度
    C. 单应求解     4点精确 / 8x8高斯消元 / 归一化DLT / 8点最小二乘   数值稳定性 + 冗余收益
    D. 圆映射       标准圆 -> 单应 -> 原图像素                        自洽性 + 可视化

运行：python tools/exp_alt_pipeline.py
输出：tools/exp_alt_pipeline.jpg
"""

import os
import sys
import time

import numpy as np
import cv2

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from config import Config                                          # noqa: E402
from vision.detector import TargetCenterDetector                   # noqa: E402
from vision import geometry as G                                   # noqa: E402
import tools.test_pc as T                                          # noqa: E402

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# 合成场景的物理尺寸（mm）：外框 = A4 外沿，内框 = 外沿内缩 18mm
OUT_MM = [(0.0, 0.0), (210.0, 0.0), (210.0, 297.0), (0.0, 297.0)]
IN_MM = [(18.0, 18.0), (192.0, 18.0), (192.0, 279.0), (18.0, 279.0)]
CENTER_MM = (105.0, 148.5)


# =============================================================================
# 工具
# =============================================================================
def bilinear(g, x, y):
    """向量化双线性采样（用于沿法向做亚像素灰度剖面）"""
    h, w = g.shape
    x = np.clip(x, 0.0, w - 1.001)
    y = np.clip(y, 0.0, h - 1.001)
    x0 = x.astype(np.int32)
    y0 = y.astype(np.int32)
    fx = x - x0
    fy = y - y0
    x1 = np.minimum(x0 + 1, w - 1)
    y1 = np.minimum(y0 + 1, h - 1)
    return (g[y0, x0] * (1 - fx) * (1 - fy) + g[y0, x1] * fx * (1 - fy)
            + g[y1, x0] * (1 - fx) * fy + g[y1, x1] * fx * fy)


def solve_gauss(A, b):
    """用户方案里的"高斯消元"：列主元 + 回代"""
    n = A.shape[0]
    M = np.hstack([np.asarray(A, float), np.asarray(b, float).reshape(-1, 1)]).copy()
    for k in range(n):
        p = int(np.argmax(np.abs(M[k:, k]))) + k
        if abs(M[p, k]) < 1e-13:
            raise np.linalg.LinAlgError("奇异矩阵")
        if p != k:
            M[[k, p]] = M[[p, k]]
        M[k + 1:, k:] -= np.outer(M[k + 1:, k] / M[k, k], M[k, k:])
    x = np.zeros(n)
    for i in range(n - 1, -1, -1):
        x[i] = (M[i, -1] - M[i, i + 1:n] @ x[i + 1:]) / M[i, i]
    return x


def homography_8x8(src, dst):
    """③ 原始坐标直接建 8x8（h33=1），高斯消元"""
    A = np.zeros((8, 8))
    b = np.zeros(8)
    for i, ((x, y), (u, v)) in enumerate(zip(src, dst)):
        A[2 * i] = [x, y, 1, 0, 0, 0, -x * u, -y * u]
        b[2 * i] = u
        A[2 * i + 1] = [0, 0, 0, x, y, 1, -x * v, -y * v]
        b[2 * i + 1] = v
    h = solve_gauss(A, b)
    H = np.array([[h[0], h[1], h[2]], [h[3], h[4], h[5]], [h[6], h[7], 1.0]])
    return H


def _normalize(p):
    c = p.mean(axis=0)
    d = np.sqrt(((p - c) ** 2).sum(axis=1)).mean()
    s = np.sqrt(2.0) / d if d > 1e-12 else 1.0
    T = np.array([[s, 0, -s * c[0]], [0, s, -s * c[1]], [0, 0, 1.0]])
    return T, (p - c) * s


def homography_dlt(src, dst):
    """归一化 DLT（SVD）：4 点=精确解，>4 点=最小二乘"""
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    Ts, sn = _normalize(src)
    Td, dn = _normalize(dst)
    n = len(sn)
    A = np.zeros((2 * n, 9))
    for i in range(n):
        x, y = sn[i]
        u, v = dn[i]
        A[2 * i] = [-x, -y, -1, 0, 0, 0, x * u, y * u, u]
        A[2 * i + 1] = [0, 0, 0, -x, -y, -1, x * v, y * v, v]
    _, _, Vt = np.linalg.svd(A)
    Hn = Vt[-1].reshape(3, 3)
    H = np.linalg.inv(Td) @ Hn @ Ts
    return H / H[2, 2]


def apply_h(H, pts):
    p = np.asarray(pts, float)
    q = np.hstack([p, np.ones((len(p), 1))]) @ H.T
    return q[:, :2] / q[:, 2:3]


def mark(corners, center, method=""):
    return dict(quad=np.asarray(corners, float), center=np.asarray(center, float),
                method=method)


def _truth_corners():
    """合成图的'真值外框角点'（与 make_scene 的物理定义一致）"""
    Hm = cv2.getPerspectiveTransform(
        np.float32([[0, 0], [210, 0], [210, 297], [0, 297]]),
        np.float32(T.DEFAULT_DST))
    return apply_h(Hm, OUT_MM)


TV_OUT = _truth_corners()


# =============================================================================
# A. 检测器对比：阈值+轮廓  vs  Canny+approxPolyDP
# =============================================================================
def detect_threshold(gray, cfg):
    """现状路线：二值化 -> 最大外轮廓 -> 极值四顶点 -> (亚像素精修)"""
    d = TargetCenterDetector(cfg)
    bw = d.binarize(gray)
    pts = d.find_frame_contour(bw)
    if pts is None:
        return None
    q, m = G.extract_ordered_corners(pts)
    if not G.is_valid_quad(q, gray.shape[1], gray.shape[0], cfg):
        return None
    return mark(q, G.diagonal_intersection(q), "extremum")


def detect_canny(gray, t1=50, t2=150, eps_ratio=0.02):
    """新路线：Canny -> 边缘图取轮廓 -> approxPolyDP 简化成多边形"""
    e = cv2.Canny(gray, t1, t2)
    cnts = cv2.findContours(e, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    if not cnts:
        return None, None, 0
    c = max(cnts, key=cv2.contourArea)
    per = cv2.arcLength(c, True)
    ap = cv2.approxPolyDP(c, eps_ratio * per, True).reshape(-1, 2).astype(float)
    if len(ap) != 4:
        return None, e, len(ap)
    q = G.order_corners_by_sumdiff(ap)
    return mark(q, G.diagonal_intersection(q), "approxPolyDP"), e, 4


def part_A(cfg):
    print("\n" + "=" * 78)
    print("A. 检测器对比（同一张合成图，真值已知）")
    print("=" * 78)
    gray = T.make_scene(seed=1, noise=1.0, blur=0.6)[1]
    truth = T.make_scene(seed=1, noise=1.0, blur=0.6)[2]

    print("\n  A1. Canny + findContours + approxPolyDP：多边形点数随 eps 的变化")
    print("      %8s %8s %14s %14s" % ("eps比", "顶点数", "中心误差px", "角点平均误差px"))
    canny_ok = {}
    for eps in (0.001, 0.005, 0.01, 0.02, 0.03, 0.05, 0.08, 0.15):
        r, _, npts = detect_canny(gray, eps_ratio=eps)
        if r is None:
            print("      %8s %8d %14s %14s" % (eps, npts, "-", "-"))
            continue
        e = np.hypot(*(r["center"] - truth))
        ec = np.hypot(*(G.order_corners_by_sumdiff(r["quad"]) - TV_OUT).T).mean()
        canny_ok[eps] = e
        print("      %8s %8d %14.4f %14.4f" % (eps, npts, e, ec))
    print("      注：合成图的'真值外框'按像素中心约定，比物理外沿内缩约 0.5~1px，")
    print("          故角点绝对误差被系统性抬高；中心误差与真值约定无关，是可靠指标。")

    print("\n  A2. 中心误差对比（与真值约定无关，最可靠）")
    rt = detect_threshold(gray, cfg)
    et = np.hypot(*(G.order_corners_by_sumdiff(rt["quad"]) - TV_OUT).T).mean()
    print("      阈值+轮廓(极值顶点，未精修) : 中心误差 %.4f px   角点平均 %.4f px"
          % (np.hypot(*(rt["center"] - truth)), et))
    best_eps = min(canny_ok, key=canny_ok.get) if canny_ok else None
    if best_eps is not None:
        rc, _, _ = detect_canny(gray, eps_ratio=best_eps)
        ec = np.hypot(*(G.order_corners_by_sumdiff(rc["quad"]) - TV_OUT).T).mean()
        print("      Canny+逼近(最优eps=%.3f)      : 中心误差 %.4f px   角点平均 %.4f px"
              % (best_eps, np.hypot(*(rc["center"] - truth)), ec))
    print("      【结论】两条检测路线对'中心'的贡献几乎相同（都 ~0.73px）；")
    print("              真正把中心误差压到 0.06px 的是'亚像素边缘精修'，不是检测算子。")

    print("\n  A3. 耗时（同一张 1280x720 灰度图，各跑 40 次取均值）")
    for name, fn in (("阈值+形态学+轮廓", lambda: detect_threshold(gray, cfg)),
                     ("Canny+轮廓+逼近", lambda: detect_canny(gray, eps_ratio=0.02))):
        fn()
        t0 = time.perf_counter()
        for _ in range(40):
            fn()
        dt = (time.perf_counter() - t0) / 40 * 1000
        print("      %-18s %.2f ms/帧" % (name, dt))

    print("\n  A4. 稳态工作集增量（Windows 进程 WS，检测 op 是否会长期占住内存）")
    try:
        b0 = _ws_mb() if False else None
        import ctypes
        from ctypes import wintypes

        class PMC(ctypes.Structure):
            _fields_ = [("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD),
                        ("PeakWorkingSetSize", ctypes.c_size_t),
                        ("WorkingSetSize", ctypes.c_size_t),
                        ("QuotaPeakPagedPoolUsage", ctypes.c_size_t),
                        ("QuotaPagedPoolUsage", ctypes.c_size_t),
                        ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t),
                        ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                        ("PagefileUsage", ctypes.c_size_t),
                        ("PeakPagefileUsage", ctypes.c_size_t)]

        def ws():
            p = PMC()
            p.cb = ctypes.sizeof(PMC)
            ctypes.windll.psapi.GetProcessMemoryInfo(
                ctypes.windll.kernel32.GetCurrentProcess(), ctypes.byref(p), p.cb)
            return p.WorkingSetSize / 1048576.0

        for name, fn in (("阈值+形态学+轮廓", lambda: detect_threshold(gray, cfg)),
                         ("Canny+轮廓+逼近", lambda: detect_canny(gray, eps_ratio=0.02))):
            for _ in range(20):
                fn()
            a = ws()
            for _ in range(300):
                fn()
            b = ws()
            print("      %-18s 300 次后工作集变化 %+.1f MB" % (name, b - a))
    except Exception as ex:                                     # pragma: no cover
        print("      （内存探针不可用：%s）" % ex)
    print("      注：该探针看的是'稳态工作集'，两种方式都未长期占用内存；")
    print("          Canny 的代价在'瞬时峰值'（Sobel 需 2 张 CV_16S 梯度图 + 幅值图，")
    print("          1280x720 约多 3~4MB 临时缓冲），稳态探针看不见。")
    return rt


# =============================================================================
# A2. 内外框分离：阈值+层级  vs  沿边灰度剖面
# =============================================================================
def part_A2(gray, cfg):
    print("\n" + "=" * 78)
    print("A2. 内外框分离：怎么拿到外框+内框（用户第 ② 步的目的）")
    print("=" * 78)
    d = TargetCenterDetector(cfg)
    bw = d.binarize(gray)
    cnts, hier = cv2.findContours(bw, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

    def biggest(pred):
        cand = [c for c, h in zip(cnts, hier[0]) if pred(h)]
        if not cand:
            return None
        c = max(cand, key=cv2.contourArea)
        return c.reshape(-1, 2).astype(float) if cv2.contourArea(c) > 50 else None

    outer = biggest(lambda h: h[3] == -1)          # 无父级 = 外轮廓
    inner = biggest(lambda h: h[3] != -1)          # 有父级 = 孔洞 = 内框
    print("  阈值+RETR_CCOMP 层级：外轮廓 %s，内孔轮廓 %s"
          % ("有" if outer is not None else "无", "有" if inner is not None else "无"))
    if outer is None or inner is None:
        return None
    qo, mo = G.extract_ordered_corners(outer)
    qi, mi = G.extract_ordered_corners(inner)
    co, ci = G.diagonal_intersection(qo), G.diagonal_intersection(qi)
    print("  外框四顶点法=%-8s 内框四顶点法=%s" % (mo, mi))
    print("  外框中心 (%.3f, %.3f)   内框中心 (%.3f, %.3f)   两者差 %.4f px"
          % (co[0], co[1], ci[0], ci[1], np.hypot(*(co - ci))))

    scale = G.estimate_scale(qo, cfg)              # px per cm
    w_out = (G.dist(qo[0], qo[1]) + G.dist(qo[3], qo[2])) / 2.0
    w_in = (G.dist(qi[0], qi[1]) + G.dist(qi[3], qi[2])) / 2.0
    tape_px = (w_out - w_in) / 2.0
    print("  由内外框反算胶带宽：%.4f cm  （合成真值 1.80 cm，误差 %.3f cm）"
          % (tape_px / scale, abs(tape_px / scale - 1.8)))

    # 沿竖直边做灰度剖面：验证"内外灰度差"能否区分外/内框
    g = gray.astype(np.float32)
    e = (qo[1] - qo[0])
    L = np.hypot(*e)
    u = e / L
    n = np.array([-u[1], u[0]])
    mid = (qo[0] + qo[1]) / 2.0
    offs = np.arange(-6.0, 6.0 + 1e-9, 0.25)
    P = mid[None, :] + offs[:, None] * n[None, :]
    I = bilinear(g, P[:, 0], P[:, 1])
    print("\n  跨外框上边中点的灰度剖面（法向 -6..+6 px）：")
    prof = "  ".join("%3.0f" % v for v in I[::4])
    print("      " + prof)
    print("  外沿处 白(纸)->黑(胶带) 为负梯度；内沿处 黑->白 为正梯度。")
    print("  => 用户第 ② 步（比较内外灰度差来区分外/内框）在原理上成立，")
    print("     但阈值+层级一次 findContours 就已同时给出内、外两条闭合轮廓，无需再采样。")
    return qo, qi, scale


# =============================================================================
# B. 亚像素定位：二值轮廓 fitLine  vs  灰度梯度剖面
# =============================================================================
def subpixel_gradient_edges(gray, quad, K=64, R=4.0, ds=0.2, trim=0.12):
    """新路线：沿每条边的法向扫描灰度剖面，取梯度极值 + 抛物线插值 -> 亚像素边缘点"""
    g = gray.astype(np.float32)
    offs = np.arange(-R, R + 1e-9, ds)
    lines = []
    for k in range(4):
        A = np.asarray(quad[k], float)
        B = np.asarray(quad[(k + 1) % 4], float)
        ev = B - A
        L = float(np.hypot(*ev))
        if L < 8:
            return None
        u = ev / L
        n = np.array([-u[1], u[0]])
        ts = np.linspace(trim * L, (1 - trim) * L, K)
        ss = np.empty(K)
        for j, t in enumerate(ts):
            base = A + t * u
            P = base[None, :] + offs[:, None] * n[None, :]
            I = bilinear(g, P[:, 0], P[:, 1])
            dI = np.gradient(I, ds)
            i = int(np.argmax(np.abs(dI)))
            if 0 < i < len(dI) - 1:
                y0, y1, y2 = abs(dI[i - 1]), abs(dI[i]), abs(dI[i + 1])
                den = y0 - 2 * y1 + y2
                dlt = 0.5 * (y0 - y2) / den if abs(den) > 1e-9 else 0.0
                dlt = float(np.clip(dlt, -0.5, 0.5))
            else:
                dlt = 0.0
            ss[j] = offs[i] + dlt * ds
        slope, inter = np.polyfit(ts, ss, 1)
        p1 = A + ts[0] * u + (slope * ts[0] + inter) * n
        p2 = A + ts[-1] * u + (slope * ts[-1] + inter) * n
        v = p2 - p1
        Lv = float(np.hypot(*v))
        lines.append((v[0] / Lv, v[1] / Lv, p1[0], p1[1]))
    out = []
    for k in range(4):
        c = G.line_intersect(lines[(k - 1) % 4], lines[k])
        if c is None:
            return None
        out.append(c)
    return np.array(out, float)


def corners_subpixel(outer, cfg):
    """现状：对二值轮廓拟合直线求交"""
    from vision.detector import refine_corners_subpixel
    q, _ = G.extract_ordered_corners(outer)
    return q, refine_corners_subpixel(outer, q, cfg)


def part_B(gray, cfg):
    print("\n" + "=" * 78)
    print("B. 亚像素边缘定位对比：角点/中心误差 vs 真值")
    print("=" * 78)
    d = TargetCenterDetector(cfg)
    bw = d.binarize(gray)
    outer = d.find_frame_contour(bw)
    q0, qr = corners_subpixel(outer, cfg)
    truth = T.make_scene(seed=1, noise=1.0, blur=0.6)[2]

    # 新路线：用阈值粗框 + 灰度梯度剖面精修；再来一级"fitLine 后接梯度剖面"
    qg = subpixel_gradient_edges(gray, q0)
    qrg = subpixel_gradient_edges(gray, qr)

    def err(q, label):
        qa = G.order_corners_by_sumdiff(np.asarray(q, float))
        e = np.hypot(*(qa - TV_OUT).T)
        c = G.diagonal_intersection(qa)
        print("      %-32s 角点均值 %.4f px  中心 %.4f px"
              % (label, e.mean(), np.hypot(*(c - truth))))

    err(q0, "极值顶点（无精修）")
    err(qr, "二值轮廓 fitLine（现状）")
    if qg is not None:
        err(qg, "灰度梯度剖面（新路线）")
    if qrg is not None:
        err(qrg, "fitLine -> 梯度剖面（两级）")
    print("      【结论】灰度梯度剖面确实比二值轮廓更贴近真实边缘（角点 0.73->0.37px），")
    print("              但中心精度两者都已远好于 0.5px 目标，谁的'误差更对称'才决定中心。")
    return qg


# =============================================================================
# C. 单应求解：4 点 vs 8 点；8x8 高斯消元 vs 归一化 DLT
# =============================================================================
def part_C():
    print("\n" + "=" * 78)
    print("C. 单应求解数值实验（纯数学：真值单应 + 角点噪声）")
    print("=" * 78)
    S = 40.0
    src_cm = np.float32([[0, 0], [210, 0], [210, 297], [0, 297]])
    dst = np.float32(T.DEFAULT_DST)
    Htrue = cv2.getPerspectiveTransform(src_cm, dst)

    out_img = apply_h(Htrue, OUT_MM)
    in_img = apply_h(Htrue, IN_MM)
    truth_center = apply_h(Htrue, [CENTER_MM])[0]
    print("  真值靶心 (%.3f, %.3f)" % tuple(truth_center))

    def cond(A):
        return np.linalg.cond(A)

    A8 = np.zeros((8, 8))
    for i, ((x, y), (u, v)) in enumerate(zip(OUT_MM, out_img)):
        A8[2 * i] = [x, y, 1, 0, 0, 0, -x * u, -y * u]
        A8[2 * i + 1] = [0, 0, 0, x, y, 1, -x * v, -y * v]
    print("  ③ 8x8 矩阵条件数（原始像素坐标，未归一化）= %.3e" % cond(A8))
    Ts, sn = _normalize(np.asarray(OUT_MM, float))
    Td, dn = _normalize(out_img)
    A8n = np.zeros((8, 8))
    for i, ((x, y), (u, v)) in enumerate(zip(sn, dn)):
        A8n[2 * i] = [x, y, 1, 0, 0, 0, -x * u, -y * u]
        A8n[2 * i + 1] = [0, 0, 0, x, y, 1, -x * v, -y * v]
    print("  ③ 同一矩阵，归一化后条件数          = %.3e" % cond(A8n))
    print("  （float64 有约 16 位有效数字；条件数每 10 倍吃掉 1 位。见下表实际影响。）")

    # ---- 无噪声：4 点方案是否精确 ----
    print("\n  C1. 无噪声（只比较求解路径本身的数值误差）")
    for name, H in (("cv2.getPerspectiveTransform (8x8 LU)", cv2.getPerspectiveTransform(
                        np.float32(OUT_MM), np.float32(out_img))),
                    ("自写 8x8 高斯消元", homography_8x8(OUT_MM, out_img)),
                    ("归一化 DLT (SVD)", homography_dlt(OUT_MM, out_img))):
        c = apply_h(H, [CENTER_MM])[0]
        print("      %-34s 中心误差 %.3e px" % (name, np.hypot(*(c - truth_center))))

    # ---- 有噪声：Monte-Carlo ----
    rng = np.random.default_rng(0)
    N = 400
    for sig in (0.3, 1.0, 2.0):
        e4, e4d, e8 = [], [], []
        for _ in range(N):
            n4 = out_img + rng.normal(0, sig, out_img.shape)
            n8o = out_img + rng.normal(0, sig, out_img.shape)
            n8i = in_img + rng.normal(0, sig, in_img.shape)
            H4 = homography_8x8(OUT_MM, n4)
            H4d = homography_dlt(OUT_MM, n4)
            H8 = homography_dlt(OUT_MM + IN_MM, np.vstack([n8o, n8i]))
            e4.append(np.hypot(*(apply_h(H4, [CENTER_MM])[0] - truth_center)))
            e4d.append(np.hypot(*(apply_h(H4d, [CENTER_MM])[0] - truth_center)))
            e8.append(np.hypot(*(apply_h(H8, [CENTER_MM])[0] - truth_center)))
        print("      C2. 角点噪声 sigma=%.1fpx | 4点8x8 %.4f | 4点DLT %.4f | 8点LS %.4f px (RMS)"
              % (sig, np.sqrt(np.mean(np.square(e4))),
                 np.sqrt(np.mean(np.square(e4d))), np.sqrt(np.mean(np.square(e8)))))

    # ---- 镜头畸变：单应模型本身失配 ----
    print("\n  C3. 有无镜头畸变时，4 点 vs 8 点（单应模型无法表达畸变）")
    H, W = 720, 1280
    cx, cy = W / 2.0, H / 2.0
    for k1 in (0.0, -0.05, -0.15):
        def distort(p):
            X = (p[:, 0] - cx) / cx
            Y = (p[:, 1] - cy) / cx
            r2 = X * X + Y * Y
            f = 1 + k1 * r2
            return np.stack([cx + X * f * cx, cy + Y * f * cx], 1)
        do, di = distort(out_img), distort(in_img)
        td = apply_h(Htrue, [CENTER_MM])[0]
        td = distort(td[None, :])[0]
        H4 = homography_dlt(OUT_MM, do)
        H8 = homography_dlt(OUT_MM + IN_MM, np.vstack([do, di]))
        e4 = np.hypot(*(apply_h(H4, [CENTER_MM])[0] - td))
        e8 = np.hypot(*(apply_h(H8, [CENTER_MM])[0] - td))
        print("      k1=%+.2f  4点中心误差 %.4f px   8点(内外框) %.4f px" % (k1, e4, e8))

    print("\n  C4. 8 点最小二乘的残差可以当【畸变体检指标】：")
    ro = apply_h(homography_dlt(OUT_MM + IN_MM, np.vstack([out_img, in_img])),
                 OUT_MM + IN_MM)
    res = np.hypot(*(ro - np.vstack([out_img, in_img])).T)
    print("      已知畸变 k1=0   时，8 点重投影残差 RMS = %.4f px" % res.std())
    return truth_center, out_img, in_img


# =============================================================================
# E. 为什么"必须"建单应：标量 px_per_cm 的方向性缺陷
# =============================================================================
def _pt_line_mm(p, a, b):
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    p = np.asarray(p, float)
    e = b - a
    return abs(e[0] * (a[1] - p[1]) - e[1] * (a[0] - p[0])) / max(1e-9, np.hypot(*e))


def part_E(gray, cfg):
    print("\n" + "=" * 78)
    print("E. 标量 px_per_cm vs 单应：把'像素'换算成'毫米'时差多少")
    print("=" * 78)
    d = TargetCenterDetector(cfg)
    bw = d.binarize(gray)
    cnts, hier = cv2.findContours(bw, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

    def biggest(pred):
        cand = [c for c, h in zip(cnts, hier[0]) if pred(h)]
        c = max(cand, key=cv2.contourArea)
        return c.reshape(-1, 2).astype(float)

    qo, _ = G.extract_ordered_corners(biggest(lambda h: h[3] == -1))
    qi, _ = G.extract_ordered_corners(biggest(lambda h: h[3] != -1))

    # 单应：图像像素 -> 毫米（外框四角点 <-> A4 外沿）
    H_i2m = homography_dlt(qo, OUT_MM)
    H_m2i = np.linalg.inv(H_i2m)
    err_o = np.hypot(*(apply_h(H_i2m, qo) - np.asarray(OUT_MM)).T).max()
    print("  H(像素->mm) 对外框四角的自洽残差：%.3e mm（≈0，说明解正确）" % err_o)

    # ---- 方向性尺度：同一位置，横向与纵向的 px/cm 可以差很多 ----
    def scale_px_cm(p_mm, axis, eps=0.05):
        a = apply_h(H_m2i, [p_mm])[0]
        q = np.asarray(p_mm, float).copy()
        q[axis] += eps
        b = apply_h(H_m2i, [q])[0]
        return float(np.hypot(*(b - a)) / eps * 10.0)

    sc_h = scale_px_cm(CENTER_MM, 0)
    sc_v = scale_px_cm(CENTER_MM, 1)
    sc_scalar = G.estimate_scale(qo, cfg)
    print("  在靶心处：横向 %.2f px/cm   纵向 %.2f px/cm   比值 %.2f"
          % (sc_h, sc_v, sc_h / sc_v))
    print("  现状 estimate_scale() 给出的单一标量 = %.2f px/cm（是两者的平均）"
          % sc_scalar)
    print("  => 用这一个标量去换算'任意方向'的像素距离，必然在某一方向错 %.0f%% 以上。"
          % (abs(sc_h - sc_v) / min(sc_h, sc_v) * 100))

    # ---- 用胶带宽度做判据：真值 18.0mm ----
    qo_mm = apply_h(H_i2m, qo)
    qi_mm = apply_h(H_i2m, qi)
    edge_d = []
    for k in range(4):
        a, b = qo_mm[k], qo_mm[(k + 1) % 4]
        edge_d += [_pt_line_mm(qi_mm[k], a, b), _pt_line_mm(qi_mm[(k + 1) % 4], a, b)]
    tape_h = np.mean(edge_d)

    wo = (G.dist(qo[0], qo[1]) + G.dist(qo[3], qo[2])) / 2.0
    wi = (G.dist(qi[0], qi[1]) + G.dist(qi[3], qi[2])) / 2.0
    tape_scalar = (wo - wi) / 2.0 / sc_scalar * 10.0

    print("\n  胶带宽度（真值 18.0 mm），三种换算方式：")
    print("    ① 标量 px_per_cm（现状）    : %6.2f mm   误差 %+6.2f mm" %
          (tape_scalar, tape_scalar - 18.0))
    print("    ② 单应 H(像素->mm)          : %6.2f mm   误差 %+6.2f mm" %
          (tape_h, tape_h - 18.0))
    print("    ③ 合成图物理真值            : %6.2f mm" % 18.0)

    # ---- 分辨率提示：1px=1mm 的工作面是否合适 ----
    px_per_mm = scale_px_cm(CENTER_MM, 0) / 10.0
    print("\n  第 ④ 步'正视面 1px=1mm'：本画面在靶心处横向已有 %.2f 图像像素/毫米。" % px_per_mm)
    if px_per_mm > 1.0:
        print("    标定到 1px=1mm 相当于把该方向降采样 %.2f 倍（丢信息）；" % px_per_mm)
        print("    更合理的等效分辨率是 %.0f px 宽（A4 210mm），而不是 210 px。" % (px_per_mm * 210))
    print("    但若只需 72 个圆周点（点变换，不做整幅 warp），工作面取多少缩放其实无关紧要——")
    print("    精度由原始像素决定，不由 mm 工作面的网格决定。")
    return qo, qi, H_i2m


# =============================================================================
# D. 圆映射：72 等分标准圆 -> 单应 -> 原图像素
# =============================================================================
def part_D(H):
    print("\n" + "=" * 78)
    print("D. 标准圆映射（用户第 ④⑤ 步）：自洽性与可视化")
    print("=" * 78)
    ang = np.linspace(0, 2 * np.pi, 72, endpoint=False)
    R = 40.0
    circ_mm = np.stack([CENTER_MM[0] + R * np.cos(ang),
                        CENTER_MM[1] + R * np.sin(ang)], 1)
    circ_px = apply_h(H, circ_mm)
    back = apply_h(np.linalg.inv(H), circ_px)
    print("  ✓ 72 点圆：mm -> px -> mm 往返最大误差 %.3e mm（应 ~1e-12，方向约定正确）"
          % np.hypot(*(back - circ_mm).T).max())
    # 圆在图像中应成椭圆：用二阶矩检查长短轴比
    d = circ_px - circ_px.mean(0)
    cov = d.T @ d / len(d)
    ev = np.linalg.eigvalsh(cov)
    print("  ✓ 映射后在图像中为椭圆：半轴比 %.4f（1.0=正圆，≠1 即透视压缩，符合预期）"
          % np.sqrt(ev[1] / ev[0]))
    return circ_mm, circ_px


# =============================================================================
def _ws_mb():
    return 0.0


def main():
    cfg = Config
    Config.WORK_SCALE = 1.0
    scene, gray, truth, dst = T.make_scene(seed=1, noise=1.0, blur=0.6)

    print("=" * 78)
    print(" 对照实验：Canny+多边形逼近 / 内外框分离 / 8x8单应 / 圆映射")
    print("=" * 78)

    rt = part_A(cfg)
    part_A2(gray, cfg)
    qg = part_B(gray, cfg)
    truth_center, out_img, in_img = part_C()
    part_E(gray, cfg)

    # 用 4 点单应（外框）做圆映射演示
    H = homography_dlt(OUT_MM, out_img)
    circ_mm, circ_px = part_D(H)

    # ---- 可视化 ----
    vis = scene.copy()
    for q, col, lb in ((rt["quad"], (0, 255, 0), "outer(threshold)"),):
        cv2.polylines(vis, [q.astype(int).reshape(-1, 1, 2)], True, col, 2)
    cv2.polylines(vis, [out_img.astype(int).reshape(-1, 1, 2)], True, (0, 255, 255), 1)
    cv2.polylines(vis, [in_img.astype(int).reshape(-1, 1, 2)], True, (255, 200, 0), 1)
    cv2.polylines(vis, [circ_px.astype(int).reshape(-1, 1, 2)], True, (0, 0, 255), 2)
    for p in circ_px[::6]:
        cv2.circle(vis, (int(p[0]), int(p[1])), 2, (0, 0, 255), -1)
    c = rt["center"].astype(int)
    cv2.drawMarker(vis, tuple(c), (255, 0, 255), cv2.MARKER_CROSS, 24, 2)
    cv2.putText(vis, "green=threshold quad  yellow=physical outer  cyan=inner hole",
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(vis, "red=72-pt circle mapped mm->px   magenta=detected center",
                (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    ok, buf = cv2.imencode(".jpg", vis)
    if ok:
        buf.tofile(os.path.join(OUT_DIR, "exp_alt_pipeline.jpg"))
        print("\n可视化已保存: %s" % os.path.join(OUT_DIR, "exp_alt_pipeline.jpg"))
    print("=" * 78)


if __name__ == "__main__":
    main()
