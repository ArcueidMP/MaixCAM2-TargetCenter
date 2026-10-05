#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/exp_perspective.py · 「视角不一致 / 透视变形」鲁棒性专项实验（PC 离线）
================================================================================
回答一个问题：现有代码能不能在**视角不一致**时依然给对靶心？

以往的工具都是手挑几个四边形（[[430,140],...]），看不出规律。本脚本改用
**真实针孔相机模型**合成靶纸图：靶纸先绕自身横轴俯仰 θ、再绕竖轴偏转 ψ，
然后放到相机前方 dist 处。θ 连续扫描，观察：

    A. 中心误差（像素 & 靶纸平面上的 mm）
    B. 各向异性 sx/sy —— 标量 px_per_cm 在透视下究竟错多少
    C. 几何合法性检查的实际容忍上限（什么时候开始拒检，而不是给错值）
    D. 透视校正能力：用 4 角点单应把"内框角点"逆映射回 mm 平面，
       看能否恢复标准 A4 的 18mm 胶带矩形（用的点不在建单应的点集里）

运行： python tools/exp_perspective.py
输出： tools/exp_perspective.jpg（误差-角度曲线 + 各向异性-角度曲线）
"""

import os
import sys

import numpy as np
import cv2

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)

from config import Config                                    # noqa: E402
from vision.detector import TargetCenterDetector             # noqa: E402
from vision import geometry as G                             # noqa: E402
import test_pc as T                                          # noqa: E402

A4W, A4H = 21.0, 29.7            # cm
S = 40.0                         # 画布分辨率 px/cm（与 test_pc 一致）
CW, CH = int(A4W * S), int(A4H * S)
TAPE_CM = 1.8                    # 黑胶带宽 cm
IMG_W, IMG_H = 1280, 720
OUT_JPG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "exp_perspective.jpg")


# =============================================================================
# 真实针孔相机模型
# =============================================================================
def make_proj(pitch_deg, yaw_deg=0.0, dist=50.0, fl=842.0,
              cx=IMG_W / 2.0, cy=IMG_H / 2.0):
    """
    靶纸平面上的点 (a, b)（cm；a 横向 0~21，b 纵向 0~29.7）-> 图像像素 (u, v)。

    模型：靶纸局部坐标以中心为原点 -> 绕 x 轴俯仰 pitch -> 绕 y 轴偏转 yaw
          -> 平移到 (0, 0, dist) -> 针孔投影。
    pitch=0 时靶纸正对相机（无透视）；pitch 越大越斜。
    """
    p, y = np.radians(pitch_deg), np.radians(yaw_deg)
    cp, sp, cyw, syw = np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    ac, bc = A4W / 2.0, A4H / 2.0

    def proj(a, b):
        x0, y0 = a - ac, b - bc
        X, Y, Z = x0, y0 * cp, y0 * sp          # 绕 x 轴俯仰
        Xr = X * cyw + Z * syw                  # 绕 y 轴偏转
        Zr = -X * syw + Z * cyw
        Zf = Zr + dist
        return (cx + fl * Xr / Zf, cy + fl * Y / Zf)

    return proj


def make_scene_pose(pitch_deg, yaw_deg=0.0, seed=1, noise=1.0, blur=0.8):
    """按指定姿态合成靶纸图，返回 (scene, gray, truth_px, dst4, H_cm2px, proj)。"""
    proj = make_proj(pitch_deg, yaw_deg)
    dst = np.float32([proj(0, 0), proj(A4W, 0), proj(A4W, A4H), proj(0, A4H)])
    rng = np.random.default_rng(seed)

    bg = np.zeros((IMG_H, IMG_W, 3), np.int16)
    for yy in range(IMG_H):
        bg[yy, :, :] = 175 + int(25 * yy / IMG_H)
    cv2.rectangle(bg, (0, IMG_H - 30), (IMG_W, IMG_H), (5, 5, 5), -1)   # 干扰块

    canvas = np.full((CH, CW, 3), 210, np.uint8)
    cv2.rectangle(canvas, (0, 0), (CW - 1, CH - 1), (18, 18, 18), -1)
    t = int(TAPE_CM * S)
    cv2.rectangle(canvas, (t, t), (CW - 1 - t, CH - 1 - t), (212, 214, 208), -1)

    src = np.float32([[0, 0], [CW, 0], [CW, CH], [0, CH]])
    Hm = cv2.getPerspectiveTransform(src, dst)

    warped = cv2.warpPerspective(canvas, Hm, (IMG_W, IMG_H), flags=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
    mask = cv2.warpPerspective(np.full((CH, CW), 255, np.uint8), Hm, (IMG_W, IMG_H),
                               flags=cv2.INTER_NEAREST,
                               borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    a = (mask.astype(np.float32) / 255.0)[..., None]
    scene = (bg.astype(np.float32) * (1 - a) + warped.astype(np.float32) * a).astype(np.uint8)
    if blur > 0:
        scene = cv2.GaussianBlur(scene, (0, 0), blur)
    if noise > 0:
        scene = np.clip(scene.astype(np.float32)
                        + rng.normal(0, noise, scene.shape), 0, 255).astype(np.uint8)

    gray = cv2.cvtColor(scene, cv2.COLOR_BGR2GRAY)
    truth_px = np.array(proj((CW - 1) / 2.0 / S, (CH - 1) / 2.0 / S), dtype=np.float64)
    return scene, gray, truth_px, dst, Hm, proj


# =============================================================================
# 指标
# =============================================================================
def edge_asymmetry(dst):
    """远/近边长度比（>1 表示有透视压缩）。dst = [TL,TR,BR,BL]。"""
    top = float(np.hypot(*(dst[1] - dst[0])))
    bot = float(np.hypot(*(dst[2] - dst[3])))
    lft = float(np.hypot(*(dst[3] - dst[0])))
    rgt = float(np.hypot(*(dst[2] - dst[1])))
    return max(top, bot) / max(1e-6, min(top, bot)), \
        max(lft, rgt) / max(1e-6, min(lft, rgt))


def local_scales(proj, a0=A4W / 2, b0=A4H / 2, d=0.05):
    """靶心处的横向/纵向局部尺度（px per cm），数值微分求得。"""
    u1, u0 = proj(a0 + d, b0), proj(a0 - d, b0)
    v1, v0 = proj(a0, b0 + d), proj(a0, b0 - d)
    sx = float(np.hypot(u1[0] - u0[0], u1[1] - u0[1]) / (2 * d))
    sy = float(np.hypot(v1[0] - v0[0], v1[1] - v0[1]) / (2 * d))
    return sx, sy


def to_mm(pt_px, H_px2cm):
    """图像像素 -> 靶纸平面 mm（用真值单应做逆映射，与误差方向无关）。"""
    v = H_px2cm @ np.array([pt_px[0], pt_px[1], 1.0])
    return np.array([v[0] / v[2], v[1] / v[2]]) / S * 10.0


def nearest_err(det4, tru4):
    """检测四角 vs 真值四角，每个检测点配最近真值点（误差小时贪婪足够）。"""
    det4 = np.asarray(det4, float)
    tru4 = np.asarray(tru4, float)
    d = np.linalg.norm(det4[:, None, :] - tru4[None, :, :], axis=2)
    return float(d.min(axis=1).mean())


# =============================================================================
# 主实验
# =============================================================================
def main():
    Config.WORK_SCALE = Config.profile()[6]          # 板端默认 0.5
    angles = list(range(0, 81, 5))

    print("=" * 100)
    print(" 透视鲁棒性扫描（针孔模型：靶纸俯仰 theta，真实 1.8cm 黑胶带，WORK_SCALE=%.2f）"
          % Config.WORK_SCALE)
    print("=" * 100)
    print(" %4s %7s %10s %9s %9s %7s %-10s %8s %8s %7s" %
          ("θ°", "透视比", "误差px", "误差mm", "角点px", "检出", "method",
           "横px/cm", "纵px/cm", "各向异性"))
    print("-" * 100)

    SEEDS = (1, 2, 3)          # 每角度多帧平均，避免单帧噪声掩盖趋势
    rows = []
    for th in angles:
        # 几何量（与随机噪声无关）：用第 1 张场景算
        _s0, _g0, _t0, dst0, Hm0, proj0 = make_scene_pose(th, seed=1)
        ratio, _ = edge_asymmetry(dst0)
        sx, sy = local_scales(proj0)

        eps_px, eps_mm, ces, scales, stages = [], [], [], [], []
        for sd in SEEDS:
            scene, gray, truth, dst, Hm, proj = make_scene_pose(th, seed=sd)
            _, Hp = _inv(Hm)
            r = TargetCenterDetector(Config).process(gray)
            if not r["valid"]:
                continue
            eps_px.append(float(np.hypot(r["raw_u"] - truth[0], r["raw_v"] - truth[1])))
            eps_mm.append(float(np.hypot(*(to_mm((r["raw_u"], r["raw_v"]), Hp)
                                           - to_mm(truth, Hp)))))
            ces.append(nearest_err(r["quad"], dst))
            scales.append(float(r["scale"]))
            stages.append(r["stage"].split("+")[-1])

        if not eps_px:
            print(" %4d %7.2f %10s %9s %9s %7s %-10s %8.2f %8.2f %7.2f"
                  % (th, ratio, "NOT FOUND", "-", "-", "否", "-", sx, sy, sx / sy))
            rows.append(dict(th=th, ratio=ratio, err_px=np.nan, err_mm=np.nan,
                             corner=np.nan, ok=False, method="-", sx=sx, sy=sy,
                             scale=None))
            continue

        okstr = "是" if len(eps_px) == len(SEEDS) else "%d/%d" % (len(eps_px), len(SEEDS))
        print(" %4d %7.2f %10.4f %9.4f %9.4f %7s %-10s %8.2f %8.2f %7.2f"
              % (th, ratio, float(np.mean(eps_px)), float(np.mean(eps_mm)),
                 float(np.mean(ces)), okstr, max(set(stages), key=stages.count),
                 sx, sy, sx / sy))
        rows.append(dict(th=th, ratio=ratio, err_px=float(np.mean(eps_px)),
                         err_mm=float(np.mean(eps_mm)), corner=float(np.mean(ces)),
                         ok=True, method=max(set(stages), key=stages.count),
                         sx=sx, sy=sy, scale=float(np.mean(scales))))

    # ---------------- 汇总 ----------------
    ok_rows = [x for x in rows if x["ok"]]
    bad = [x for x in rows if not x["ok"]]
    print("-" * 100)
    if ok_rows:
        print(" 检出 %d/%d 个角度。成功区间内：误差px 最大 %.4f，误差mm 最大 %.4f"
              % (len(ok_rows), len(rows),
                 max(x["err_px"] for x in ok_rows), max(x["err_mm"] for x in ok_rows)))
    if bad:
        print(" 拒检角度（几何合法性检查拦下，不给错值）: %s"
              % ", ".join("%d°" % x["th"] for x in bad))
        print(" 拒检原因诊断：")
        for x in bad:
            print("   %3d° : %s" % (x["th"], diagnose(x["th"], Config)))

    # ---------------- 两条取顶点路径的对比 ----------------
    print("\n" + "=" * 100)
    print(" 「极值顶点法」（指定主路径）vs 「x+y/x−y 极值兜底」：命中情况与结果差异")
    print("=" * 100)
    print(" %4s %-10s %8s %10s %14s %14s" %
          ("θ°", "实际走哪条", "去重后点数", "极值法可用", "两法中心差px", "两法角点差px"))
    for th in (0, 10, 20, 30, 45, 60, 65):
        scene, gray, truth, dst, Hm, proj = make_scene_pose(th, seed=1)
        d = TargetCenterDetector(Config)
        pts = d.find_frame_contour(d.binarize(gray))
        ext = G.extreme_corners(pts)
        uniq = []
        for p in ext:
            if all(G.dist(p, q) > 3.0 for q in uniq):
                uniq.append(p)
        qe = G.order_corners(np.array(uniq)) if len(uniq) == 4 else None
        if qe is not None and not (G.is_convex(qe) and G.quad_area(qe) > 1.0):
            qe = None
        qs = G.order_corners_by_sumdiff(pts)
        _qx, meth = G.extract_ordered_corners(pts)
        if qe is not None:
            dc = float(np.hypot(*(G.diagonal_intersection(qe)
                                  - G.diagonal_intersection(qs))))
            # 两法角点配对（按最近邻）
            dq = float(np.linalg.norm(qe[:, None] - qs[None], axis=2).min(axis=1).mean())
            print(" %4d %-10s %8d %10s %14.4f %14.4f"
                  % (th, meth, len(uniq), "是", dc, dq))
        else:
            print(" %4d %-10s %8d %10s %14s %14s"
                  % (th, meth, len(uniq), "否(退化)", "-", "-"))

    # ---------------- 让「极值顶点法」真正生效的场景：绕光轴旋转 ----------------
    print("\n" + "=" * 100)
    print(" 何时「极值顶点法」才给出 4 个互异顶点：四条边都明显偏离水平/竖直（绕光轴 roll）")
    print("=" * 100)
    print(" %6s %-9s %6s %13s %13s %13s" %
          ("roll°", "走哪条", "点数", "极值法误差px", "兜底法误差px", "最终(精修)误差px"))
    _s, _g, _t, dst0, _h, _p = make_scene_pose(30, seed=1)
    ccx, ccy = dst0.mean(axis=0)
    for roll in (0, 10, 20, 30, 45):
        a = np.radians(roll)
        R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        dst = ((np.asarray(dst0, float) - [ccx, ccy]) @ R.T + [ccx, ccy]).astype(np.float32)
        scene, gray, truth, _ = T.make_scene(dst=dst, seed=1, noise=1.0, blur=0.8)
        d = TargetCenterDetector(Config)
        pts = d.find_frame_contour(d.binarize(gray))
        ext = G.extreme_corners(pts)
        uniq = []
        for p in ext:
            if all(G.dist(p, q) > 3.0 for q in uniq):
                uniq.append(p)
        qe = G.order_corners(np.array(uniq)) if len(uniq) == 4 else None
        if qe is not None and not (G.is_convex(qe) and G.quad_area(qe) > 1.0):
            qe = None
        qs = G.order_corners_by_sumdiff(pts)
        _qx, meth = G.extract_ordered_corners(pts)
        r = TargetCenterDetector(Config).process(gray)
        e_fin = (float(np.hypot(r["raw_u"] - truth[0], r["raw_v"] - truth[1]))
                 if r["valid"] else float("nan"))
        e_ext = (float(np.hypot(*(G.diagonal_intersection(qe) - truth)))
                 if qe is not None else float("nan"))
        e_sum = float(np.hypot(*(G.diagonal_intersection(qs) - truth)))
        print(" %6d %-9s %6d %13.4f %13.4f %13.4f"
              % (roll, meth, len(uniq), e_ext, e_sum, e_fin))

    # ---------------- 标量尺度到底错多少 ----------------
    print("\n" + "=" * 100)
    print(" 标量 px_per_cm 的方向性误差（estimate_scale 返回单一标量，横向/纵向其实不同）")
    print("=" * 100)
    print(" %4s %10s %10s %10s %12s %12s" %
          ("θ°", "横px/cm", "纵px/cm", "各向异性", "标量返回值", "对横向相对误差"))
    for x in rows:
        if not x["ok"] or x["scale"] is None:
            continue
        rel = abs(x["scale"] - x["sx"]) / x["sx"]
        flag = "  <-- 超 5%" if rel > 0.05 else ""
        print(" %4d %10.2f %10.2f %10.2f %12.2f %11.1f%%%s"
              % (x["th"], x["sx"], x["sy"], x["sx"] / x["sy"], x["scale"],
                 rel * 100, flag))

    # ---------------- 透视校正能力（单应）----------------
    print("\n" + "=" * 100)
    print(" 透视校正验证：用「外框 4 角」建单应 H(mm->px)，把「内框 4 角」逆映射回 mm 平面")
    print(" 期望落点：18mm 胶带 => (18,18)-(192,279) 的标准 A4 内框（用点不在建单应的点集内）")
    print("=" * 100)
    print(" %4s %12s %12s %13s %13s %10s" %
          ("θ°", "内框TL mm", "内框BR mm", "宽mm(应174)", "高mm(应261)", "最大偏差mm"))
    for th in (0, 10, 20, 30, 40, 50):
        scene, gray, truth, dst, Hm, proj = make_scene_pose(th, seed=1)
        det = TargetCenterDetector(Config)
        r = det.process(gray)
        if not r["valid"]:
            print(" %4d  NOT FOUND" % th)
            continue
        inner = _inner_corners(gray)
        if inner is None:
            print(" %4d  内框未检出" % th)
            continue
        # 用「检测到的外框」建 单应 mm->px
        Hdet = cv2.getPerspectiveTransform(
            np.float32([[0, 0], [A4W * 10, 0], [A4W * 10, A4H * 10], [0, A4H * 10]]),
            np.float32(r["quad"]))
        Hi = np.linalg.inv(Hdet)              # px -> mm
        inn_mm = np.array([_apply(Hi, p) for p in inner])
        exp = np.array([[TAPE_CM * 10, TAPE_CM * 10],
                        [A4W * 10 - TAPE_CM * 10, TAPE_CM * 10],
                        [A4W * 10 - TAPE_CM * 10, A4H * 10 - TAPE_CM * 10],
                        [TAPE_CM * 10, A4H * 10 - TAPE_CM * 10]])
        dev = float(np.hypot(*(inn_mm - exp).T).max())
        tl, br = inn_mm[0], inn_mm[2]
        w = abs(inn_mm[1][0] - inn_mm[0][0])
        h = abs(inn_mm[3][1] - inn_mm[0][1])
        print(" %4d (%6.1f,%6.1f) (%6.1f,%6.1f) %12.2f %12.2f %10.2f"
              % (th, tl[0], tl[1], br[0], br[1], w, h, dev))

    # ---------------- 组合姿态 ----------------
    print("\n" + "=" * 100)
    print(" 组合姿态（俯仰 + 偏转同时存在）")
    print("=" * 100)
    print(" %6s %6s %8s %10s %9s %7s" % ("pitch", "yaw", "透视比", "误差px", "误差mm", "检出"))
    for (p, yawv) in ((20, 20), (30, 30), (40, 20), (15, 45), (50, 10)):
        scene, gray, truth, dst, Hm, proj = make_scene_pose(p, yawv, seed=1)
        ratio, _ = edge_asymmetry(dst)
        resid, Hpx2cm = _inv(Hm)
        r = TargetCenterDetector(Config).process(gray)
        if not r["valid"]:
            print(" %6d %6d %8.2f %10s %9s %7s" % (p, yawv, ratio, "NOT FOUND", "-", "否"))
            continue
        e_px = float(np.hypot(r["raw_u"] - truth[0], r["raw_v"] - truth[1]))
        e_mm = float(np.hypot(*(to_mm((r["raw_u"], r["raw_v"]), Hpx2cm)
                                - to_mm(truth, Hpx2cm))))
        print(" %6d %6d %8.2f %10.4f %9.4f %7s" % (p, yawv, ratio, e_px, e_mm, "是"))

    # ---------------- 画图 ----------------
    _draw(rows, OUT_JPG)
    print("\n曲线图已保存: %s" % OUT_JPG)


def diagnose(th, cfg):
    """复刻 is_valid_quad 的逐项检查，定位到底卡在哪一项。"""
    scene, gray, truth, dst, Hm, proj = make_scene_pose(th, seed=1)
    H, W = gray.shape[:2]
    d = TargetCenterDetector(cfg)
    pts = d.find_frame_contour(d.binarize(gray))
    if pts is None:
        return "无外轮廓"
    q, _m = G.extract_ordered_corners(pts)
    msgs = []
    area = float(G.quad_area(q))
    if area < cfg.MIN_AREA_RATIO * W * H:
        msgs.append("面积 %.0f < %.0f (%.0f%%画面)"
                    % (area, cfg.MIN_AREA_RATIO * W * H, cfg.MIN_AREA_RATIO * 100))
    if not G.is_convex(q):
        msgs.append("非凸")
    w1, w2 = G.dist(q[0], q[1]), G.dist(q[3], q[2])
    h1, h2 = G.dist(q[1], q[2]), G.dist(q[0], q[3])
    for nm, (a, b) in (("上/下边", (w1, w2)), ("左/右边", (h1, h2))):
        rr = max(a, b) / max(1e-3, min(a, b))
        if rr > cfg.OPP_RATIO_MAX:
            msgs.append("%s比 %.2f > 上限 %.2f" % (nm, rr, cfg.OPP_RATIO_MAX))
    ar = ((w1 + w2) / 2.0) / max(1e-3, (h1 + h2) / 2.0)
    if not (cfg.ASPECT_MIN <= ar <= cfg.ASPECT_MAX):
        msgs.append("长宽比 %.2f 超出 [%.2f, %.2f]"
                    % (ar, cfg.ASPECT_MIN, cfg.ASPECT_MAX))
    return "；".join(msgs) if msgs else "通过（失败在后续步骤）"


def _inv(Hm):
    """Hm: cm 画布像素 -> 图像；返回 (残差占位, 图像像素 -> cm 画布像素 的逆)。"""
    return 0.0, np.linalg.inv(Hm)


def _apply(H, p):
    v = H @ np.array([p[0], p[1], 1.0])
    return np.array([v[0] / v[2], v[1] / v[2]])


def _inner_corners(gray):
    """
    取黑胶带**内沿**轮廓的 4 角。
    注意不能用"面积最小的轮廓"——画面里还有底部干扰块，会挑错。
    必须靠 RETR_CCOMP 的父子关系：靶纸外轮廓(prarent=-1) 下挂的那个孔才是内沿。
    """
    d = TargetCenterDetector(Config)
    bw = d.binarize(gray)
    cnts, hier = cv2.findContours(bw, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if not cnts or hier is None:
        return None
    hier = hier[0]
    outers = [i for i in range(len(cnts)) if hier[i][3] == -1]
    if not outers:
        return None
    oi = max(outers, key=lambda i: cv2.contourArea(cnts[i]))
    holes = [i for i in range(len(cnts)) if hier[i][3] == oi]
    if not holes:
        return None
    ii = max(holes, key=lambda i: cv2.contourArea(cnts[i]))
    pts = cnts[ii].reshape(-1, 2).astype(np.float64)
    s = pts.sum(1)
    dd = pts[:, 0] - pts[:, 1]
    return np.array([pts[np.argmin(s)], pts[np.argmax(dd)],
                     pts[np.argmax(s)], pts[np.argmin(dd)]])


def _draw(rows, path):
    """左图：中心误差(mm, 对数轴) vs 俯仰角；右图：各向异性 sx/sy vs 俯仰角。"""
    W, H = 980, 430
    img = np.full((H, W, 3), 255, np.uint8)
    okr = [x for x in rows if x["ok"]]

    def axes(x0, y0, w, h, title, ylab, ymin, ymax, logy=False):
        cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), (0, 0, 0), 1)
        cv2.putText(img, title, (x0, y0 - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 0), 1)
        for i in range(6):
            yy = y0 + int(h * i / 5)
            cv2.line(img, (x0, yy), (x0 + w, yy), (225, 225, 225), 1)
            if logy:
                v = ymin * (ymax / ymin) ** (1 - i / 5)
                cv2.putText(img, "%.2g" % v, (x0 - 46, yy + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.36, (60, 60, 60), 1)
            else:
                v = ymin + (ymax - ymin) * (1 - i / 5)
                cv2.putText(img, "%.2f" % v, (x0 - 46, yy + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.36, (60, 60, 60), 1)
        cv2.putText(img, ylab, (x0 - 6, y0 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (0, 0, 0), 1)
        for a in range(0, 81, 20):
            xx = x0 + int(w * a / 80.0)
            cv2.putText(img, "%d" % a, (xx - 8, y0 + h + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (60, 60, 60), 1)
        cv2.putText(img, "pitch (deg)", (x0 + w // 2 - 34, y0 + h + 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)

    # --- 左：误差 mm（对数轴）---
    x0, y0, w, h = 70, 60, 370, 300
    ymin, ymax = 0.005, 20.0
    axes(x0, y0, w, h, "A. center error vs tilt (log)", "mm", ymin, ymax, True)
    prev = None
    for x in okr:
        xx = x0 + int(w * x["th"] / 80.0)
        v = max(ymin, min(ymax, x["err_mm"]))
        yy = y0 + int(h * (1 - np.log(v / ymin) / np.log(ymax / ymin)))
        if prev is not None:
            cv2.line(img, prev, (xx, yy), (0, 90, 200), 2)
        cv2.circle(img, (xx, yy), 4, (0, 0, 255), -1)
        prev = (xx, yy)
    cv2.line(img, (x0, y0 + h - 1), (x0 + w, y0 + h - 1), (150, 150, 150), 1)

    # --- 右：各向异性 ---
    x0b = 560
    axes(x0b, y0, w, h, "B. anisotropy sx/sy vs tilt", "ratio", 1.0, 4.0, False)
    prev = None
    for x in okr:
        xx = x0b + int(w * x["th"] / 80.0)
        v = max(1.0, min(4.0, x["sx"] / x["sy"]))
        yy = y0 + int(h * (1 - (v - 1.0) / 3.0))
        if prev is not None:
            cv2.line(img, prev, (xx, yy), (30, 140, 30), 2)
        cv2.circle(img, (xx, yy), 4, (0, 0, 255), -1)
        prev = (xx, yy)
    cv2.putText(img, "sx/sy > 1 => scalar px_per_cm cannot be right",
                (x0b, y0 + h + 58), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (140, 60, 0), 1)

    cv2.putText(img, "25E bullseye - perspective robustness", (20, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    ok, buf = cv2.imencode(".jpg", img)
    if ok:
        buf.tofile(path)


if __name__ == "__main__":
    main()
