#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
detector.py · 一帧灰度图 -> 靶心像素坐标（OpenCV + NumPy）
================================================================================
把需求指定的流程逐步落地，每一步都可单独调用 / 验证：

    ① 二值化锁定板框  灰度 -> 黑阈值二值化 -> 形态学开/闭净化
    ② 描  边          外轮廓提取 findContours
    ③ 找四个顶点      轮廓上 x 最小 / x 最大 / y 最小 / y 最大 四个点
    ④ 连  线          排序成 [左上 右上 右下 左下]
    ⑤ 求  交          两条对角线交点 = 靶心
    ⑥ 尺度估计        A4 边长反算 px_per_cm
    ⑦ 时序滤波        One Euro Filter
    （⑧ 绘制见 overlay.py；串口打包见 main.py）

②′ 为什么要"按结构打分"而不是"取面积最大的轮廓"
--------------------------------------------------------------------------------
    合成图上"取最大黑轮廓"够用，但**真机上极易锁错**，因为只要出现下面任一情况，
    最大轮廓就不是黑胶带框：
        · 深色背景/桌面/机架  —— 面积比胶带框更大，甚至与框连成一片；
        · 画面里有比靶纸更大的深色干扰物（显示器、鼠标垫、阴影）；
        · 黑框局部贴到画面边缘  —— 轮廓被画面边界切断，算出来的"四角"是画面角；
        · 曝光没标定好        —— 固定阈值下黑胶带根本进不了二值图（或整幅全黑）。
    而黑胶带框有一个**与光照无关的结构特征**：一块深色细环，把一块亮面围在里面。
    所以改成：对每个够大的候选轮廓算结构特征 -> 打分 -> 取最高分。
    结构特征全部是**旋转/透视不变**的（最小外接矩形、面积比），不依赖角点排序。

③″ 阈值自动标定
--------------------------------------------------------------------------------
    固定阈值稳定的大前提是"曝光锁定且标定过"。为防现场没标定好，
    引入两级机制：
        · 候选阈值：fixed / otsu 都试，用同一套结构分择优（THRESH_CANDIDATES）；
        · 锁定 + 重标定：一旦选中某个阈值就锁住，后续帧只算这一个（不增加开销）；
          连续 THRESH_RELOCK_MISS 帧失败才重新扫描（THRESH_SWEEP）。

内存策略（针对开发板"帧缓冲内存不足"的报错）
--------------------------------------------------------------------------------
    · 相机直接输出 GRAYSCALE（1 B/px），不做彩色转换，零拷贝拿到 numpy 视图；
    · 两级解算：先在缩小图（WORK_SCALE，默认 0.5）上快速**粗定位**板框，
      再在粗定位得到的外接框（只比靶纸大一圈）内用**全分辨率**精算一遍。
      这样中间缓冲的内存只与"目标在画面里的大小"相关，而不是整幅画面；
    · 缩小图未能锁定板框时，才回退到整幅全分辨率重算；
    · cv2 的中间缓冲通过 dst= 复用，避免每帧重复分配。
"""

import math

import numpy as np
import cv2

from . import geometry as G


def _empty():
    return dict(valid=False, u=None, v=None, scale=0.0, quad=None, corners=None,
                stage="none", raw_u=None, raw_v=None, th=None, lvl=0, cand=None)


def _dbg_rec(th, lvl=0, n=0, score=0.0, kind="-", fg=0.0, fill=0.0, rim=0.0,
             ring=0.0, aspect=0.0, contrast=0.0, ok=False, skip=""):
    """
    诊断记录：**必须把每个字段都填上数值默认值**。

    踩过的坑：早先在不同分支里手写 dict(...)，有的分支只填了一半字段
    （比如"前景占比越界"那条就没有 fill/rim/ring/aspect/contrast），
    结果板端诊断打印那一行时 `"%.2f" % None` 直接抛
    `TypeError: must be real number, not NoneType` —— 整条诊断信息永远打不出来，
    真机上就只能看到"锁不住"+一行报错，等于完全瞎调。
    统一用这个工厂函数构造，就不会再漏字段。
    """
    return dict(th=th, lvl=lvl, n=n, score=round(float(score), 3), kind=kind,
                fg=fg, fill=round(float(fill), 3), rim=round(float(rim), 3),
                ring=round(float(ring), 3), aspect=round(float(aspect), 3),
                contrast=round(float(contrast), 1), ok=bool(ok), skip=skip)


# =============================================================================
# ②′ 候选轮廓的结构特征 + 打分
# =============================================================================
def _soft(x, lo, hi, soft=0.4):
    """区间 [lo,hi] 内得满分 1；越界按 soft*(hi-lo) 的宽度线性衰减到 0。"""
    if lo <= x <= hi:
        return 1.0
    span = max(1e-9, float(hi) - float(lo))
    d = (float(lo) - x) if x < lo else (x - float(hi))
    return max(0.0, 1.0 - d / max(1e-9, soft * span))


def _rise(x, lo, hi):
    """
    "越大越好"的量：<=lo 得 0，>=hi 得 1，中间线性。
    对比度这类量必须用它，**不能**用 _soft（_soft 的区间外会衰减到 0，
    会把"对比度远高于上限"这种最好的情况判成 0 —— 这是踩过的坑）。
    """
    if x >= hi:
        return 1.0
    if x <= lo:
        return 0.0
    return float(x - lo) / max(1e-9, float(hi) - float(lo))


def contour_features(bw, gray, cfg, W, H):
    """
    列出二值图上所有"够大"的候选轮廓，并算出判断"这是不是靶纸框"的结构特征。

    **两族候选（互补，必须都要）**
        kind="ring"  深色环：黑胶带外沿。内孔 = 纸面。中心 = 环的两条对角线交点。
                     优点：外沿最接近纸边，中心最准（胶带贴得不均匀也不影响）。
                     缺点：**胶带一旦与任何深色区域相邻（暗背景/桌面/干扰物/阴影），
                     两者在二值图里连成一片，环的外沿就不再是闭合轮廓** —— 取不到。
        kind="paper" 亮面矩形：纸面（= 胶带内沿围出的区域）。中心与环同心。
                     优点：只要"纸面亮、周围深"就始终成立，抗上述所有破坏。
                     缺点：胶带贴得不均匀时，内沿中心会相对纸边中心偏一点点。

    所以策略是：能拿到可靠的环就用环（更准），环被破坏时用纸面（更稳）。

    字段（都可给板端诊断打印用）：
        kind       "ring" / "paper"
        area/fill  面积、轮廓 / 最小外接矩形 面积比（矩形外沿应≈1）
        aspect     最小外接矩形长宽比（旋转不变）
        rim        内孔/外框 面积比（环应≈0.6~0.8）
        ring       环厚/外框等效边长（1.8cm 胶带 / 21cm ≈ 0.07）
        hole_fill  内孔 / 内孔外接矩形（内孔也应是矩形）
        contrast   亮面灰度 - 深色带灰度（框与纸面的实际对比度，判阈值是否合理）
        border     是否贴到画面边缘（靶纸永远不贴边）
        score      结构分（见 score_candidate）

    两趟走，为了远距离也不拖慢帧率：
        第一趟只算"便宜"的特征（面积/外接框/最小外接矩形）先过硬门槛，把候选砍到
        MAX_CANDIDATES 个；第二趟才对幸存者算灰度探针（要读图）并打分。
        MIN_AREA_RATIO 从 0.02 调小后，暗背景下可能出现成百上千个小轮廓，
        没有这趟预筛 + 上限，帧率会被直接拖垮。
    """
    p = cv2.findContours(bw, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    cnts, hier = p[-2], p[-1]
    if not cnts:
        return []
    if hier is not None and getattr(hier, "ndim", 0) == 3:
        hier = hier[0]

    img_a = float(W * H)
    amin = float(cfg.MIN_AREA_RATIO) * img_a
    bpx = int(getattr(cfg, "SCORE_BORDER_PX", 2))
    max_c = max(1, int(getattr(cfg, "MAX_CANDIDATES", 60)))
    allow_border = bool(getattr(cfg, "SCORE_ALLOW_BORDER", False))

    # ---- 第一趟：便宜特征 + 硬门槛 ----
    cheap = []
    for i, c in enumerate(cnts):
        area = cv2.contourArea(c)
        if area < amin:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if not allow_border and (x <= bpx or y <= bpx
                                 or x + w >= W - bpx or y + h >= H - bpx):
            continue                                    # 贴边：靶纸永远不贴边，直接丢
        (_c, (rw, rh), _a) = cv2.minAreaRect(c)
        if rw < 1.0 or rh < 1.0:
            continue
        fill = float(area / (rw * rh))
        if fill < cfg.SCORE_MIN_FILL:
            continue
        aspect = float(max(rw, rh) / min(rw, rh))
        if not (cfg.SCORE_ASPECT_LO <= aspect <= cfg.SCORE_ASPECT_HI):
            continue
        cheap.append((area, i, c, w, h, fill, aspect))
    if not cheap:
        return []
    if len(cheap) > max_c:                              # 按面积取前 N，保证耗时可控
        cheap.sort(key=lambda t: -t[0])
        cheap = cheap[:max_c]

    # ---- 第二趟：灰度探针 + 打分 ----
    out = []
    for (area, i, c, w, h, fill, aspect) in cheap:
        f = dict(area=float(area), fill=float(fill), aspect=float(aspect),
                 border=False, parent_area=0.0)
        pp = c.reshape(-1, 2).astype(np.float64)
        sp = pp[np.linspace(0, len(pp) - 1, 16).astype(int)]     # 16 个等距采样点
        pid = int(hier[i][3]) if hier is not None else -1

        # ---------------------------------------------------------------
        # 族一：深色环（黑胶带外沿）—— 层级里的"外层"轮廓
        # ---------------------------------------------------------------
        if pid == -1:
            f["kind"] = "ring"
            hole, hole_fill, hb, hc = 0.0, 0.0, None, None
            ci = hier[i][2] if hier is not None else -1
            while ci != -1:
                ca = cv2.contourArea(cnts[ci])
                if ca > hole:
                    hole = ca
                    hx, hy, hw, hh = cv2.boundingRect(cnts[ci])
                    hole_fill = ca / max(1.0, float(hw * hh))
                    hb = (hw, hh)
                    M = cv2.moments(cnts[ci])
                    if M["m00"] > 1e-9:
                        hc = (M["m10"] / M["m00"], M["m01"] / M["m00"])
                ci = hier[ci][0]
            f["rim"] = float(hole / area) if area > 0 else 0.0
            f["hole_fill"] = float(hole_fill)
            f["ring"] = (float((1.0 - math.sqrt(max(0.0, f["rim"]))) / 2.0)
                         if hole > 0 else 0.5)
            if hc is None or hb is None:
                f.update(contrast=-1.0, hole_gray=0.0, ring_gray=0.0)
                f["score"] = 0.0
                continue
            # 孔内亮面：孔心小片均值
            k = max(1, int(0.20 * min(hb)))
            g_in = _patch_mean(gray, hc[0], hc[1], k, W, H)
            # 环上：从轮廓点朝孔心方向内移，落在胶带上。
            # 步长按**该候选自己的尺度**取（≈0.35 倍胶带宽），小目标就移小一点 ——
            # 固定 2px 在远距离会直接跨过 1px 宽的胶带落到背景上，对比度就测错了。
            tape_px = ring_thickness_px(area, hole)
            step = min(6.0, max(1.0, 0.45 * tape_px))
            g_ring = _band_gray(gray, sp, hc, step, W, H, inward=True)

        # ---------------------------------------------------------------
        # 族二：亮面矩形（纸面 = 胶带内沿）—— 层级里的"内孔"轮廓
        # ---------------------------------------------------------------
        else:
            f["kind"] = "paper"
            pa = cv2.contourArea(cnts[pid]) if pid >= 0 else 0.0
            f["parent_area"] = float(pa)
            f["rim"], f["ring"], f["hole_fill"] = 0.0, 0.0, 1.0
            M = cv2.moments(c)
            hc = ((M["m10"] / M["m00"], M["m01"] / M["m00"])
                  if M["m00"] > 1e-9 else None)
            if hc is None:
                f.update(contrast=-1.0, hole_gray=0.0, ring_gray=0.0)
                f["score"] = 0.0
                continue
            k = max(1, int(0.20 * min((w, h))))
            g_in = _patch_mean(gray, hc[0], hc[1], k, W, H)
            # 外圈：从轮廓点**向外**移，落在深色胶带上。
            # 胶带宽 = (外沿半径 - 内沿半径)，直接由"父轮廓(环)面积 vs 本轮廓(纸面)面积"估出，
            # 所以这个探针在任何距离都落在胶带中间，不会跨到背景上去。
            tape_px = ring_thickness_px(pa, area)
            step = min(8.0, max(1.0, 0.50 * tape_px))
            g_ring = _band_gray(gray, sp, hc, step, W, H, inward=False)

        f["hole_gray"], f["ring_gray"] = g_in, g_ring
        f["contrast"] = float(g_in - g_ring)
        f["score"] = score_candidate(f, cfg)
        if f["score"] > 0:
            f["pts"] = pp
            out.append(f)

    out.sort(key=lambda d: d["score"], reverse=True)
    return out


def ring_thickness_px(area_outer, area_inner):
    """
    由"外轮廓面积"与"内孔面积"估算环的厚度（px）。

        矩形环：外边长 √A，内孔边长 √a  =>  厚度 = (√A − √a)/2

    为什么要它：给"灰度探针该往外/往里移多少像素"提供一个**随目标尺度自适应**的长度。
    固定步长（比如恒为 2px）在远距离目标上会直接跨过 1~2px 宽的胶带落到背景里，
    测出来的对比度就是背景的对比度 —— 于是"纸面/胶带"的判据全部失准。
    两边为0或数据异常时返回一个保守的小值。
    """
    ao, ai = max(0.0, float(area_outer)), max(0.0, float(area_inner))
    if ao <= 0.0:
        return 1.0
    return max(0.5, (math.sqrt(ao) - math.sqrt(min(ai, ao))) / 2.0)


def _patch_mean(gray, cx, cy, k, W, H):
    """(cx,cy) 处边长 2k+1 的小片灰度均值"""
    x, y = int(round(cx)), int(round(cy))
    patch = gray[max(0, y - k):y + k + 1, max(0, x - k):x + k + 1]
    return float(patch.mean()) if patch.size else 0.0


def _band_gray(gray, sp, hc, step, W, H, inward=True):
    """
    在"框带"上取灰度中位数：把 16 个轮廓采样点沿**指向/背离**孔心的方向平移 step 像素。
    inward=True  -> 从外轮廓往里走，落在深色胶带上（环族）
    inward=False -> 从亮面轮廓往外走，落在深色胶带上（亮面族）
    """
    v = np.array(hc, dtype=np.float64) - sp
    n = np.hypot(v[:, 0], v[:, 1])
    n[n < 1e-6] = 1.0
    u = (v / n[:, None]) * (1.0 if inward else -1.0)
    q = sp + u * float(step)
    qx = np.clip(np.round(q[:, 0]).astype(int), 0, W - 1)
    qy = np.clip(np.round(q[:, 1]).astype(int), 0, H - 1)
    return float(np.median(gray[qy, qx]))


def score_candidate(f, cfg):
    """
    候选"是不是靶纸框"的结构分（0~1；<=0 表示直接否决）。

    两族的判据不同（因为可用的结构信息不同）：
      ring  —— 先过硬性条件（外沿近矩形 / 内孔占比像环 / 环厚像 1.8cm 胶带 / 长宽比像 A4），
               再加权：fill 0.30 + rim 0.30 + aspect 0.20 + ring 0.20
      paper —— 外沿近矩形 + 长宽比像 A4 + **外面确实是一圈深色**（对比度）+ 外围深色区
               比亮面大（parent_area > area）
    最后乘一个"实际灰度对比度"因子 —— 对比度太低说明阈值没选对。
    """
    if f["border"] and not getattr(cfg, "SCORE_ALLOW_BORDER", False):
        return 0.0
    if f["fill"] < cfg.SCORE_MIN_FILL:
        return 0.0
    if not (cfg.SCORE_ASPECT_LO <= f["aspect"] <= cfg.SCORE_ASPECT_HI):
        return 0.0

    if f.get("kind") == "paper":
        if f["parent_area"] < 1.02 * f["area"]:
            return 0.0
        # 硬性：外围那圈必须是**真深色**。不然"亮块贴在亮墙上、只是把阈值拉到比墙还高"
        # 这种假框也会通过（实测对比度只有 69，而黑胶带围出来的纸面对比度 >180）。
        if f["contrast"] < getattr(cfg, "PAPER_MIN_CONTRAST", 85.0):
            return 0.0
        s = (0.35 * _soft(f["fill"], 0.80, 1.00, 1.0)
             + 0.30 * _soft(f["aspect"], 1.05, 1.85, 0.7)
             + 0.35 * _rise(f["contrast"],
                            getattr(cfg, "PAPER_MIN_CONTRAST", 85.0),
                            getattr(cfg, "PAPER_GOOD_CONTRAST", 180.0)))
    else:
        if not (cfg.SCORE_MIN_HOLE <= f["rim"] <= cfg.SCORE_MAX_HOLE):
            return 0.0
        if f["hole_fill"] < cfg.SCORE_MIN_HOLEFILL:
            return 0.0
        if not (cfg.SCORE_RING_MIN <= f["ring"] <= cfg.SCORE_RING_MAX):
            return 0.0
        c = _rise(f["contrast"], getattr(cfg, "SCORE_MIN_CONTRAST", 12.0),
                  getattr(cfg, "SCORE_GOOD_CONTRAST", 60.0))
        s = (0.30 * _soft(f["fill"], 0.80, 1.00, 1.0)
             + 0.30 * _soft(f["rim"], 0.60, 0.86, 0.6)
             + 0.20 * _soft(f["aspect"], 1.15, 1.75, 0.7)
             + 0.20 * _soft(f["ring"], 0.035, 0.16, 0.8)) * (0.55 + 0.45 * c)
        s += float(getattr(cfg, "RING_BONUS", 0.0))
    return float(s)


def largest_contour(bw):
    """旧行为：面积最大的外轮廓（RETR_EXTERNAL）。仅用于对照 / 打分全失败时的兜底。"""
    contours = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < 10:
        return None
    return c.reshape(-1, 2).astype(np.float64)


def select_frame_contour(bw, gray, cfg, W, H):
    """
    ② 描边：从二值图里挑出"靶纸框"那条轮廓（两族候选取结构分最高者）。

    返回 (pts, info) 或 None。info 里带 kind（ring/paper）与全部结构特征，供诊断。
    SELECT_MODE="largest" 时退化成旧行为（取最大外轮廓），用于 A/B 对照。
    """
    if getattr(cfg, "SELECT_MODE", "score") == "largest":
        pts = largest_contour(bw)
        if pts is None:
            return None
        return pts, dict(score=1.0, mode="largest", kind="ring", border=False,
                         fill=1.0, aspect=1.414, rim=0.73, ring=0.07, hole_fill=1.0,
                         contrast=0.0, area=float(len(pts)), n_cand=1, parent_area=0.0)

    feats = contour_features(bw, gray, cfg, W, H)
    if feats:
        f = feats[0]
        info = dict(f)
        info["mode"] = "score"
        info["n_cand"] = len(feats)
        return f["pts"], info

    # 打分全不通过：退回"最大"轮廓（几何合法性检查仍会再拦一道）
    if getattr(cfg, "SCORE_FALLBACK_LARGEST", True):
        pts = largest_contour(bw)
        if pts is not None:
            return pts, dict(score=0.0, mode="fallback", kind="ring", border=False,
                             fill=0.0, aspect=0.0, rim=0.0, ring=0.0, hole_fill=0.0,
                             contrast=0.0, parent_area=0.0, n_cand=0,
                             area=float(cv2.contourArea(
                                 pts.astype(np.float32).reshape(-1, 1, 2))))
    return None


# =============================================================================
# 角点亚像素精修
# =============================================================================
def refine_corners_subpixel(pts, corners, cfg, min_pts=8):
    """
    角点亚像素精修（默认开启）：
    对板框 4 条边分别取"靠近该边且避开角点"的轮廓点，用 cv2.fitLine 拟合直线，
    再把相邻两条拟合直线的交点作为新角点 —— 把 ±1~2px 的角点量化抖动压到亚像素。
    拟合失败时原样返回输入角点。
    """
    p = np.asarray(pts, dtype=np.float64)
    lines = []
    for k in range(4):
        A = corners[k]
        B = corners[(k + 1) % 4]
        e = B - A
        L = float(np.hypot(e[0], e[1]))
        if L < 1e-3:
            return corners
        u = e / L
        d = p - A
        t = d @ u                                        # 沿边投影
        perp = np.abs(d[:, 0] * u[1] - d[:, 1] * u[0])    # 到边的垂直距离
        m = (t > cfg.REFINE_TRIM * L) & (t < (1.0 - cfg.REFINE_TRIM) * L) \
            & (perp < cfg.REFINE_D_RATIO * L + 2.0)
        seg = p[m]
        if seg.shape[0] < min_pts:
            return corners
        vx, vy, x0, y0 = cv2.fitLine(seg.astype(np.float32),
                                     cv2.DIST_L2, 0, 0.01, 0.01).flatten()
        lines.append((float(vx), float(vy), float(x0), float(y0)))

    out = []
    for k in range(4):
        c = G.line_intersect(lines[(k - 1) % 4], lines[k])   # 角点 k = 边(k-1) ∩ 边 k
        if c is None:
            return corners
        out.append(c)
    return np.array(out, dtype=np.float64)


class TargetCenterDetector:
    """无状态几何 + 有状态滤波/阈值锁定；cfg 见 config.Config。"""

    def __init__(self, cfg):
        self.cfg = cfg
        fp = cfg.FILTER_PARAMS[cfg.FILTER_MODE]
        self.fu = G.OneEuroFilter(freq=30.0, **fp)
        self.fv = G.OneEuroFilter(freq=30.0, **fp)
        self._small = None            # 复用的缩小图缓冲（避免每帧分配）
        self._th_lock = None          # 锁定的二值化阈值（None = 需要扫描标定）
        self._lvl_lock = 0            # 与阈值一起锁定的形态学档位（0 常规 / 1 精细）
        self._th_prev = None          # 上一轮锁定的阈值（迟滞用，避免阈值来回跳）
        self._next_sweep = 0          # 下次允许扫描的帧号（防止每帧都全扫）
        self._frame_i = 0
        self._miss = 0                # 连续失败帧数（触发重新标定）
        self._dbg_pass = ""           # last_debug 来自哪一趟（coarse/full/roi）
        self.last_debug = []          # 最近一帧各阈值的候选情况（供板端打印诊断）

    # =====================================================================
    # ① 二值化
    # =====================================================================
    def _morph(self, bw, lvl=0):
        """
        形态学净化。lvl=0 用配置的常规核；lvl=1 用"精细核"（保住细环）。

        为什么要分档：远距离时胶带在缩小图上只有 1~2px 宽，MORPH_OPEN_K=3 的
        开运算会把它整个腐蚀掉。环一没，纸面就不再是"内孔"，**两族候选同时失效**
        —— 真机上表现为"近距离锁得住、远距离怎么调阈值都锁不住"。
        """
        cfg = self.cfg
        if lvl == 1:
            ok = int(getattr(cfg, "MORPH_OPEN_K_FINE", 1))
            ck = int(getattr(cfg, "MORPH_CLOSE_K_FINE", 3))
        else:
            ok = int(getattr(cfg, "MORPH_OPEN_K", 3))
            ck = int(getattr(cfg, "MORPH_CLOSE_K", 9))
        if ok >= 2:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ok,) * 2)
            bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, k)
        if ck >= 2:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ck,) * 2)
            bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, k)
        return bw

    def binarize(self, gray):
        """单一阈值二值化（固定或 Otsu）+ 形态学净化。外部脚本/诊断可直接调用。"""
        cfg = self.cfg
        if cfg.USE_OTSU:
            _, bw = cv2.threshold(gray, 0, 255,
                                  cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        else:
            _, bw = cv2.threshold(gray, cfg.BLACK_THRESH, 255, cv2.THRESH_BINARY_INV)
        return self._morph(bw)

    def _mask_specs(self, g, force=False):
        """
        本帧要试的 (阈值, 形态学档位) 列表（按优先级）。锁定后只有一个 —— 稳态零额外开销。

        排序策略：**先把所有阈值用常规形态学跑一遍，再整体换成精细形态学跑一遍**。
        这样近处（常规够用）在第一轮就命中并提前退出，不会为"远距离"付额外代价；
        只有真的什么都找不到时，才会花第二轮去救细环。

        force=True：无视已有的锁定、强制全扫。用于"缩小图这一遍没找到 → 全分辨率升级这一遍"，
        否则上一遍留下的（可能是错的）锁会把升级这一遍一起锁死，永远救不回来。
        """
        cfg = self.cfg
        if self._th_lock is not None and not force:
            return [(int(self._th_lock), int(self._lvl_lock))]

        if getattr(cfg, "SELECT_MODE", "score") == "largest":
            mode = "otsu" if cfg.USE_OTSU else "fixed"     # 完全复现旧行为
        else:
            mode = getattr(cfg, "THRESH_CANDIDATES", "auto")
            if cfg.USE_OTSU and mode == "fixed":
                mode = "otsu"

        vals = []
        if mode in ("fixed", "auto"):
            vals.append(int(cfg.BLACK_THRESH))
        if mode in ("otsu", "auto"):
            t, _ = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            vals.append(int(round(float(t))))
        if (mode != "fixed" and getattr(cfg, "THRESH_RELOCK", False)
                and (force or self._frame_i >= self._next_sweep)):
            vals += [int(v) for v in getattr(cfg, "THRESH_SWEEP", ())]

        seen, ths = set(), []
        for v in vals:
            v = max(1, min(254, int(v)))
            if v not in seen:
                seen.add(v)
                ths.append(v)

        lvls = [0]
        if getattr(cfg, "MORPH_SEARCH", False):
            lvls.append(1)
        return [(t, lvl) for lvl in lvls for t in ths]

    def _masks(self, g, force=False):
        """产出 (阈值, 形态学档位, 二值图) 列表；阈值 -1 表示自适应阈值（无固定阈值）。"""
        outs = []
        for th, lvl in self._mask_specs(g, force):
            _, bw = cv2.threshold(g, th, 255, cv2.THRESH_BINARY_INV)
            outs.append((th, lvl, self._morph(bw, lvl)))
        if (getattr(self.cfg, "USE_ADAPTIVE", False)
                and self._th_lock is None and self._frame_i >= self._next_sweep):
            blk = int(self.cfg.ADAPTIVE_BLOCK) | 1
            bw = cv2.adaptiveThreshold(g, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                       cv2.THRESH_BINARY_INV, max(3, blk),
                                       float(self.cfg.ADAPTIVE_C))
            outs.append((-1, 0, self._morph(bw, 0)))
        return outs

    # =====================================================================
    # ② 描边：取面积最大的闭合轮廓（旧接口，保留给诊断脚本）
    # =====================================================================
    @staticmethod
    def find_frame_contour(bw):
        return largest_contour(bw)

    # =====================================================================
    # ③④⑤ 由一条轮廓点序列解出四边形 + 靶心
    # =====================================================================
    def _quad_from_pts(self, pts, w, h):
        cfg = self.cfg
        q, method = G.extract_ordered_corners(pts)
        if not G.is_valid_quad(q, w, h, cfg):
            # 兜底：极值法失效。典型场景是靶纸在画面里**绕光轴旋转约 45°**时，
            # x+y 与 x−y 两个极值方向会指向同一个角，4 个候选点里"缺一个角、
            # 另一个角被取重"，四边形退化。这时改用凸包 + 多边形逼近取四角。
            alt = self._polygon_quad(pts, w, h)
            if alt is None:
                return None
            q, method = alt
        raw_q = q.copy()

        if cfg.REFINE_CORNERS:
            qr = refine_corners_subpixel(pts, q, cfg)
            if G.is_valid_quad(qr, w, h, cfg):
                q = qr
            else:
                q = raw_q

        c = G.diagonal_intersection(q)
        if c is None or not np.isfinite(c).all():
            return None
        # 合理性：靶心必须落在板框外接矩形内
        xmin, ymin = q.min(axis=0)
        xmax, ymax = q.max(axis=0)
        if not (xmin - 5 <= c[0] <= xmax + 5 and ymin - 5 <= c[1] <= ymax + 5):
            return None
        return dict(quad=q, center=c, method=method)

    def _detect_on(self, g, tag="", force_sweep=False):
        """
        在一张（可能已缩小的）图上完成 ①~⑤。

        会遍历本帧允许尝试的 (阈值, 形态学档位)，用**结构分**在"mask x 候选轮廓"里
        选最优；首次（或重新标定）会顺带把胜出的组合锁下来。
        tag 只用于诊断：标出这批 dbg 来自哪一趟（coarse / full / roi）；
        force_sweep=True 用于"缩小图失败 → 全分辨率升级"这一趟（见 _mask_specs）。
        """
        cfg = self.cfg
        h, w = g.shape[:2]
        sweeping = force_sweep or (self._th_lock is None)
        bonus = float(getattr(cfg, "THRESH_LOCK_BONUS", 0.0))
        best, dbg = None, []

        for th, lvl, bw in self._masks(g, force_sweep):
            fg = float((bw > 0).mean())
            lo = float(getattr(cfg, "MASK_FG_MIN", 0.002))
            hi = float(getattr(cfg, "MASK_FG_MAX", 0.93))
            if not (lo <= fg <= hi):
                # 阈值不合理：太低(什么都切不出)或太高(把背景都当深色)，直接跳过
                dbg.append(_dbg_rec(th, lvl, n=0, score=0.0, kind="-",
                                    fg=round(fg, 3), ok=False, skip="前景占比越界"))
                continue
            sel = select_frame_contour(bw, g, cfg, w, h)
            if sel is None:
                dbg.append(_dbg_rec(th, lvl, n=0, score=0.0, kind="-",
                                    fg=round(fg, 3), ok=False))
                continue
            pts, info = sel
            d = self._quad_from_pts(pts, w, h)
            dbg.append(_dbg_rec(th, lvl, n=info.get("n_cand", 1),
                                score=info.get("score", 0.0),
                                kind=info.get("kind", "?"), fg=round(fg, 3),
                                fill=info.get("fill", 0.0), rim=info.get("rim", 0.0),
                                ring=info.get("ring", 0.0),
                                aspect=info.get("aspect", 0.0),
                                contrast=info.get("contrast", 0.0),
                                ok=d is not None))
            if d is None:
                continue
            d["info"], d["th"], d["lvl"] = info, th, lvl
            # 迟滞：给"上一轮锁定过的那个阈值"一点加分，避免两个阈值分数接近时来回跳
            # （阈值一跳，二值图和 px_per_cm 都会跟着跳，观感就是"锁定不稳"）
            d["rank"] = float(info.get("score", 0.0)) + (bonus if th == self._th_prev else 0.0)
            if best is None or d["rank"] > best["rank"]:
                best = d
            if float(info.get("score", 0.0)) >= cfg.SCORE_EARLY_STOP:
                break

        self.last_debug = dbg
        self._dbg_pass = tag
        if sweeping:
            if best is not None:
                self._th_lock = int(best["th"])      # 锁定胜出组合：后续帧零额外开销
                self._lvl_lock = int(best["lvl"])
                self._th_prev = int(best["th"])
            else:
                # ⚠ 全扫也失败时**绝不能**把阈值锁成"配置值"。
                #   踩过的坑：这里原来写 self._th_lock = cfg.BLACK_THRESH，
                #   于是"缩小图这一遍没找到"这个错误结论被固定下来，
                #   紧接着的"全分辨率升级那一遍"就只试这一个阈值 —— 继续失败。
                #   真机上表现为"远距离/强环境光怎么都锁不住"，而且日志里
                #   每帧只有一条 th=95 的诊断行（就是这个原因）。
                #   正确做法：保持解锁，只把"下次允许全扫"的时刻往后推，靠 _next_sweep 限流。
                self._th_lock = None
                self._lvl_lock = 0
                self._next_sweep = self._frame_i + int(
                    getattr(cfg, "THRESH_RELOCK_EVERY", 15))
        return best

    def _polygon_quad(self, pts, w, h):
        """
        凸包 + 多边形逼近取四角（极值法失效时的兜底路径）。

        为什么需要它：`extreme_corners`（x/y 极值）在靶纸某条边接近水平/竖直时退化；
        `order_corners_by_sumdiff`（x+y / x−y 极值）在靶纸绕光轴旋转约 45° 时，
        两个极值方向指向同一个角，导致 4 个候选点里缺角。这一路径对任意旋转角都成立：
        矩形（透视后仍是凸四边形）的凸包按弧长比例逼近，恰好还原 4 个顶点。

        返回 (corners4x2, method_str)；找不到合法四边形时返回 None。
        """
        cfg = self.cfg
        try:
            hull = cv2.convexHull(np.asarray(pts, np.float32).reshape(-1, 1, 2))
            peri = cv2.arcLength(hull, True)
            if peri < 1.0:
                return None
            for k in (0.01, 0.015, 0.02, 0.03, 0.04, 0.06, 0.08, 0.12):
                ap = cv2.approxPolyDP(hull, k * peri, True).reshape(-1, 2)
                if ap.shape[0] != 4:
                    continue
                q = G.order_corners(ap.astype(np.float64))
                if G.is_valid_quad(q, w, h, cfg):
                    return q, "hull%.3f" % k
        except Exception:
            return None
        return None

    # =====================================================================
    # 对外主入口
    # =====================================================================
    def process(self, gray, timestamp=None):
        """
        输入：灰度 ndarray（HxW uint8，通常是相机缓冲的零拷贝视图）
              timestamp：秒；给定时启用 One Euro 滤波（连续帧）
        输出：dict(valid, u, v, scale, quad, corners, stage, raw_u, raw_v, th, cand)
              u,v   = 滤波后靶心（未滤波时等于 raw_u/raw_v）
              scale = px_per_cm（局部尺度）
              th    = 本帧采用的二值化阈值
              cand  = 选中候选的结构特征（调参/诊断用）
        """
        if gray is None:
            return _empty()
        if gray.ndim == 3:
            gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
        H, W = gray.shape[:2]

        # 可选 ROI：在 ROI 内解算后再平移回全图坐标
        roi = self.cfg.ROI
        if roi and len(roi) == 4:
            x, y, w, h = [int(v) for v in roi]
            x, y = max(0, x), max(0, y)
            w, h = max(8, min(w, W - x)), max(8, min(h, H - y))
            r = self._solve(gray[y:y + h, x:x + w], timestamp)
            if r["valid"]:
                off = np.array([x, y], dtype=np.float64)
                r["u"] += x
                r["v"] += y
                r["raw_u"] += x
                r["raw_v"] += y
                for k in ("quad", "corners"):
                    if r[k] is not None:
                        r[k] = r[k] + off
            return r
        return self._solve(gray, timestamp)

    def _solve(self, gray, timestamp):
        H, W = gray.shape[:2]
        # WORK_SCALE 默认 None（表示"由 main.py 按相机档位注入"），单独用本类时必须兜底
        _ws = getattr(self.cfg, "WORK_SCALE", None)
        ws = 1.0 if _ws is None else float(_ws)
        ws = max(0.15, min(1.0, ws))

        out, stage = None, "none"

        # ---- 第一遍：缩小图上"粗定位"板框（省内存 / 省时间）----
        if ws < 0.999:
            sw, sh = max(64, int(round(W * ws))), max(64, int(round(H * ws)))
            self._small = cv2.resize(gray, (sw, sh), interpolation=cv2.INTER_AREA,
                                     dst=self._small)
            d = self._detect_on(self._small, "coarse")
            if d is not None:
                sx, sy = W / float(sw), H / float(sh)
                out = dict(quad=d["quad"] * np.array([sx, sy], dtype=np.float64),
                           center=np.array([d["center"][0] * sx, d["center"][1] * sy],
                                           dtype=np.float64),
                           method=d["method"], info=d["info"], th=d["th"],
                           lvl=d.get("lvl", 0))
                stage = "coarse%.2f" % ws

        # ---- 缩小图失败：整幅全分辨率直接解算 ----
        # force_sweep=True：这一趟是"升级"，必须无视上一趟留下的（可能是错的）锁定全扫一遍。
        if out is None:
            d = self._detect_on(gray, "full", force_sweep=True)
            if d is None:
                self._book_keep(False)
                return _empty()
            out, stage = d, "full"

        # ---- 第二遍：在粗定位外接框内用全分辨率精算 ----
        # 缩小图会把边缘量化到小的像素栅格上（约 ±1 个原始像素的偏置）；
        # 这一遍只处理"靶纸附近"的一小块，既拿回全分辨率精度，
        # 内存占用又只与目标在画面中的大小相关。
        if ws < 0.999:
            refined = self._refine_in_roi(gray, out)
            if refined is not None:
                out = refined
                stage += "+roi"

        q = out["quad"]
        c = out["center"]
        # 用"深色环外沿(纸边 21x29.7)"还是"亮面矩形(胶带内沿 17.4x26.1)"换算物理尺度
        info = out.get("info") or {}
        inner = (info.get("kind") == "paper")
        scale = G.estimate_scale(q, self.cfg, inner=inner)

        # ⑦ 时序滤波
        if timestamp is None:
            u_f, v_f = float(c[0]), float(c[1])
        else:
            u_f = self.fu(float(c[0]), timestamp)
            v_f = self.fv(float(c[1]), timestamp)

        self._book_keep(True)
        return dict(valid=True, u=float(u_f), v=float(v_f), scale=float(scale),
                    quad=q, corners=q,
                    stage="%s+%s%s" % (stage, out["method"],
                                       "/paper" if inner else ""),
                    raw_u=float(c[0]), raw_v=float(c[1]),
                    th=out.get("th"), lvl=out.get("lvl", 0), cand=out.get("info"))

    def _book_keep(self, ok):
        """记录连续失败帧数：够了就把阈值解锁，下一帧重新扫描标定。"""
        self._frame_i += 1
        if ok:
            self._miss = 0
            return
        self._miss += 1
        if (getattr(self.cfg, "THRESH_RELOCK", False)
                and self._miss >= int(getattr(self.cfg, "THRESH_RELOCK_MISS", 3))):
            self._th_lock = None
            self._lvl_lock = 0
            self._miss = 0

    def _refine_in_roi(self, gray, coarse):
        """
        用粗定位的结果裁出一小块（靶纸外接框 + 8% 余量），在里面以全分辨率重解。
        返回裁剪坐标系已平移回全图的字典；不满足条件时返回 None（沿用粗定位结果）。
        """
        H, W = gray.shape[:2]
        q = coarse["quad"]
        xmin, ymin = q.min(axis=0)
        xmax, ymax = q.max(axis=0)
        bw, bh = xmax - xmin, ymax - ymin
        if bw < 8 or bh < 8:
            return None
        mx, my = 0.08 * bw + 6.0, 0.08 * bh + 6.0
        x0 = int(max(0, np.floor(xmin - mx)))
        y0 = int(max(0, np.floor(ymin - my)))
        x1 = int(min(W, np.ceil(xmax + mx)))
        y1 = int(min(H, np.ceil(ymax + my)))
        cw, ch = x1 - x0, y1 - y0
        if cw < 48 or ch < 48:
            return None
        if cw * ch > 0.85 * W * H:          # 框几乎等于整幅，不值得再来一遍
            return None

        d = self._detect_on(gray[y0:y1, x0:x1], "roi")
        if d is None:
            return None
        sq = d["quad"]
        # 轮廓贴到裁剪边界 => 可能被裁断，结果不可信，弃用
        if (sq[:, 0].min() <= 1.0 or sq[:, 1].min() <= 1.0
                or sq[:, 0].max() >= cw - 2.0 or sq[:, 1].max() >= ch - 2.0):
            return None
        off = np.array([x0, y0], dtype=np.float64)
        return dict(quad=sq + off, center=d["center"] + off, method=d["method"],
                    info=d["info"], th=d["th"], lvl=d.get("lvl", 0))

    def reset(self):
        self.fu.reset()
        self.fv.reset()
        self._small = None
        self._th_lock = None
        self._lvl_lock = 0
        self._th_prev = None
        self._next_sweep = 0
        self._frame_i = 0
        self._miss = 0
        self._dbg_pass = ""
        self.last_debug = []
