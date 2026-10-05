#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
geometry.py · 纯 NumPy 几何运算（不依赖 OpenCV / maix，PC 端可直接测试）
================================================================================
对应需求流程的第 ③④⑤⑥ 步：
    ③ 找四个顶点  取轮廓上 像素 x 最小 / x 最大 / y 最小 / y 最大 的四个点
    ④ 连    线    四顶点按 [左上 右上 右下 左下] 排序
    ⑤ 求    交    两条对角线 (TL-BR) 与 (TR-BL) 的交点 = 靶心
    ⑥ 尺度        用 A4 已知边长反算 px_per_cm

取顶点是**三级**的，逐级兜底（细节见各函数 docstring）：
    1) extreme_corners          —— 需求指定的 x/y 极值四顶点（轴对齐时会退化成 3 个点）
    2) order_corners_by_sumdiff —— 多方向极值候选 + 最大面积四边形（透视/旋转下稳妥）
    3) detector._polygon_quad   —— 凸包 + approxPolyDP（前两级全失效时用，实现在 detector.py）
注意：**排序必须用质心极角排序**。若用 x+y / x−y 排序，靶纸绕光轴旋转约 45° 时 s 与 d
会出现并列极值，同一个角被排进两个位置、四边形退化 → 直接漏检（已实测复现并修复）。

为什么"对角线交点"就是靶心（关键结论，可省去单应矩阵）
--------------------------------------------------------------------------------
    靶纸黑框是一个矩形。透视成像是单应变换 H，它把"直线映射为直线、交点映射为交点"，
    所以矩形两条对角线在图像中的交点，恰好是矩形中心(=靶心)的投影。
    => 只要四角点找得准，对角线交点给出的靶心是**精确解**，无需再求解 H。
       （这也是"取外沿还是内沿"不影响中心、只影响尺度的原因：两者同心。）
"""

import math
from itertools import combinations as _combinations

import numpy as np


# =============================================================================
# 基础量
# =============================================================================
def dist(a, b):
    return float(math.hypot(a[0] - b[0], a[1] - b[1]))


def cross(o, a, b):
    """叉积（图像坐标 y 轴向下）"""
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def quad_area(q):
    """
    四边形面积（鞋带公式），q 为 4x2。

    ⚠ 这里原来是 `abs(sum(cross(q[i], q[i+1], q[i+2]) for i in range(4))) / 2`，
      其中 `cross(o,a,b) = (a−o)×(b−o)` —— 用的是 `(q[i+1]−q[i]) × (q[i+2]−q[i])`，
      单位正方形实算得 2，**结果是真实面积的 2 倍**（恒等式要求的是相邻边向量
      `(q[i+2]−q[i+1])`，不是 `(q[i+2]−q[i])`）。因为下游几乎都只用它做**比值/比较**
      （`quad_area(q) > 1.0`、`_best_quad` 里比大小），这个 2 倍一直没暴露出来；
      但 `is_valid_quad` 的"面积下限"是按它写的，于是那个门槛实际只有标称的一半。
      已修正为真实面积，并把 `MIN_QUAD_AREA_RATIO` 一起按真实面积取值。
    """
    q = np.asarray(q, dtype=np.float64).reshape(4, 2)
    return abs(float(np.dot(q[:, 0], np.roll(q[:, 1], -1))
                      - np.dot(q[:, 1], np.roll(q[:, 0], -1)))) / 2.0


def is_convex(q):
    """顺序正确的凸四边形：4 个叉积同号"""
    sg = [cross(q[i], q[(i + 1) % 4], q[(i + 2) % 4]) for i in range(4)]
    return all(v > 0 for v in sg) or all(v < 0 for v in sg)


# =============================================================================
# ③ 四个顶点
# =============================================================================
def extreme_corners(pts):
    """
    ③ 找四个顶点：像素 x 最小 / x 最大 / y 最小 / y 最大 的四个点。
    对凸四边形（矩形）而言，这 4 个极值点正是它的 4 个顶点。
    返回 4x2 数组（未排序）。
    """
    p = np.asarray(pts, dtype=np.float64)
    xs, ys = p[:, 0], p[:, 1]
    return np.array([p[int(np.argmin(xs))], p[int(np.argmax(xs))],
                     p[int(np.argmin(ys))], p[int(np.argmax(ys))]], dtype=np.float64)


# =============================================================================
# ④ 排序 [TL, TR, BR, BL]
# =============================================================================
def order_corners(pts4):
    """
    把 4 个点排成 [左上, 右上, 右下, 左下]。

    实现：以 4 点质心为原点做**极角排序**得到环序（凸四边形的顶点按极角排序
    必然还原正确的相邻关系），再选一个起点。对任意旋转角都成立。

    为什么不能用 x+y / x−y 排序：那张做法在靶纸相对图像**旋转约 45°** 时会退化 ——
    x+y 与 x−y 的两个极值落到同一个角上，排出来的 4 个位置里出现重复点、
    并漏掉一个真的角，四边形直接不合法。（这正是"相机装歪/靶纸在画面里斜着"时
    会漏检的原因。）

    起点不影响结果：环序为 [P0,P1,P2,P3] 时两条对角线是 P0-P2、P1-P3；
    起点做循环移位后对角线**集合不变**，故靶心（对角线交点）与起点选择无关。
    """
    p = np.asarray(pts4, dtype=np.float64).reshape(4, 2)
    c = p.mean(axis=0)
    ang = np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0])
    q = p[np.argsort(ang)]
    k = int(np.argmin(q.sum(axis=1)))      # 起点取 x+y 最小者（仅为让 TL/TR/BR/BL 标签可读）
    return np.roll(q, -k, axis=0)


def _best_quad(points):
    """
    在候选点里挑出**凸包面积最大**的 4 个，排成 [TL,TR,BR,BL]。
    候选点都是轮廓上的角点邻域点（通常 ≤8 个），穷举组合即可（C(8,4)=70）。
    返回 None 表示无合法凸四边形。
    """
    n = len(points)
    if n < 4:
        return None
    best_q, best_a = None, -1.0
    for comb in _combinations(range(n), 4):
        q = order_corners(np.array([points[i] for i in comb], dtype=np.float64))
        if not is_convex(q):
            continue
        a = quad_area(q)
        if a > best_a:
            best_a, best_q = a, q
    return best_q


def order_corners_by_sumdiff(contour_pts):
    """
    直接在整条轮廓上取角点 -> [TL,TR,BR,BL]（透视/旋转下的稳妥兜底）。

    候选点由**多组方向**极值合并而成：x / y 极值 + x±y 极值，去重后再极角排序。
    只取单一 x±y 方向时，靶纸旋转约 45° 会把同一个角取重、并漏掉另一个角；
    x / y 极值在轴对齐时又会把两个极值指向同一角 —— 合并多组才能覆盖所有姿态。
    去重后多于 4 个时，取其中凸包面积最大的 4 个（角点的特征就是"围得最大"）。
    """
    p = np.asarray(contour_pts, dtype=np.float64)
    s = p.sum(axis=1)
    d = p[:, 0] - p[:, 1]
    cand = [p[int(np.argmin(p[:, 0]))], p[int(np.argmax(p[:, 0]))],
            p[int(np.argmin(p[:, 1]))], p[int(np.argmax(p[:, 1]))],
            p[int(np.argmin(s))], p[int(np.argmax(s))],
            p[int(np.argmin(d))], p[int(np.argmax(d))]]
    diag = float(np.hypot(np.ptp(p[:, 0]), np.ptp(p[:, 1])))
    tol = max(3.0, 0.015 * diag)
    uniq = []
    for c in cand:
        if all(dist(c, u) > tol for u in uniq):
            uniq.append(c)
    if len(uniq) == 4:
        return order_corners(np.array(uniq))
    if len(uniq) > 4:
        q = _best_quad(uniq)
        if q is not None:
            return q
    # 候选不足 4 个：退回纯 x±y 四极值（原行为，由上层 hull 兜底路径接住）
    return np.array([p[int(np.argmin(s))], p[int(np.argmax(d))],
                     p[int(np.argmax(s))], p[int(np.argmin(d))]], dtype=np.float64)


def _too_thin(q, ratio_max=4.0):
    """
    四边形是否已退化成"细长条 / 三角形"。
    x/y 极值法的典型退化：两个极值落在**同一个角的两侧边缘**上（相距几像素，
    躲过去重），而另一个真正的角整个缺失 —— 此时 shape 上表现为某一组对边
    长度相差一个数量级。这种"四边形"虽然凸，但不能当真。
    """
    w1, w2 = dist(q[0], q[1]), dist(q[3], q[2])
    h1, h2 = dist(q[1], q[2]), dist(q[0], q[3])
    if min(w1, w2, h1, h2) < 1e-6:
        return True
    return (max(w1, w2) / min(w1, w2) > ratio_max
            or max(h1, h2) / min(h1, h2) > ratio_max)


def quad_fit_error(pts, q, n=64):
    """
    轮廓点（等距采样 n 个）到**四边形边界**的平均距离（px）。

    这是判断"这个四边形到底有没有框住轮廓"最直接的量：
      · 真正贴着轮廓取出来的四边形 —— 轮廓点基本都落在四条边上，均值 ~0.5px；
      · **极值法在小目标上取错角**的假四边形：它的某条边会从轮廓中间横切过去，
        轮廓上有一段点离边界十几像素（真机上实测 14px），均值直接上到几像素。

    为什么需要它：这种假四边形是**凸的、面积也正常、长宽比也正常**，
    凸性/面积/长宽比那几条检查全都拦不住它，中心却能偏 10~20px。
    用它做"两个候选取角方案谁更好"的裁判，就可以在极值法失手时自动改用多方向极值法。
    """
    p = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
    if len(p) > n:
        p = p[np.linspace(0, len(p) - 1, n).astype(int)]
    q = np.asarray(q, dtype=np.float64).reshape(4, 2)
    d = np.full(len(p), np.inf)
    for i in range(4):
        a, b = q[i], q[(i + 1) % 4]
        e = b - a
        L2 = float(e[0] * e[0] + e[1] * e[1])
        if L2 < 1e-12:
            continue
        t = np.clip(((p[:, 0] - a[0]) * e[0] + (p[:, 1] - a[1]) * e[1]) / L2, 0.0, 1.0)
        dx = p[:, 0] - (a[0] + t * e[0])
        dy = p[:, 1] - (a[1] + t * e[1])
        d = np.minimum(d, np.hypot(dx, dy))
    return float(d.mean()) if len(d) else float("inf")


def extract_ordered_corners(pts, min_sep=3.0):
    """
    ③+④ 组合：先取 x/y 极值四顶点并去重排序为 [TL,TR,BR,BL]；
    若极值点退化（不足 4 个互异点 / 退化成细长条）或非法，则回退到多方向极值法。

    注意：「x 最小」和「y 最小」在靶纸**正视（轴对齐）**时会落到同一个角点；
    且当靶纸在画面中倾斜时，某个极值可能落在"另一个角的两侧边缘"上导致缺角 ——
    这两类退化都靠 `order_corners_by_sumdiff` 兜住。

    ★ 再加一层裁判：极值法对**小目标 + 接近轴对齐**还会犯另一种错 ——
      矩形上边接近水平时，"y 最小的点"是**整条上边**，argmin 可能取到边的中段
      而不是右上角；x 最大同理。这样得到的 4 个点互异、排出来还凸、面积也正常，
      **但根本没有框住轮廓**（真机实测中心偏 14~20px，只在 40~60px 的小目标上出现）。
      所以这里同时算出两套候选，用 `quad_fit_error`（轮廓点到四边形边界的平均距离）
      当裁判取更贴合的那个 —— 极值法真失手时会自动让位给多方向极值法。

    返回 (corners4x2, method_str)
    """
    ext = extreme_corners(pts)
    uniq = []
    for p in ext:
        if all(dist(p, q) > min_sep for q in uniq):
            uniq.append(p)
    q_ext = None
    if len(uniq) == 4:
        q = order_corners(np.array(uniq))
        if is_convex(q) and quad_area(q) > 1.0 and not _too_thin(q):
            q_ext = q
    q_sd = order_corners_by_sumdiff(pts)
    if q_ext is None:
        return q_sd, "sumdiff"
    try:
        e_ext = quad_fit_error(pts, q_ext)
        e_sd = quad_fit_error(pts, q_sd)
    except Exception:
        return q_ext, "extreme"
    if e_sd < e_ext - 0.2:                           # 多方向极值法明显更贴合 -> 用它的
        return q_sd, "sumdiff"
    return q_ext, "extreme"


# =============================================================================
# ⑤ 连线求交 -> 靶心
# =============================================================================
def line_intersect(l1, l2):
    """两条参数直线 (vx, vy, x0, y0) 求交；平行返回 None"""
    vx1, vy1, x1, y1 = l1
    vx2, vy2, x2, y2 = l2
    den = vx1 * vy2 - vy1 * vx2
    if abs(den) < 1e-9:
        return None
    dx, dy = x2 - x1, y2 - y1
    t = (dx * vy2 - dy * vx2) / den
    return np.array([x1 + t * vx1, y1 + t * vy1], dtype=np.float64)


def seg_intersect(p, p2, q, q2):
    """线段所在直线的交点"""
    return line_intersect((p2[0] - p[0], p2[1] - p[1], p[0], p[1]),
                          (q2[0] - q[0], q2[1] - q[1], q[0], q[1]))


def diagonal_intersection(q):
    """
    ⑤ 板框两条对角线 (TL-BR 与 TR-BL) 的交点 = 靶心。
    q = [TL, TR, BR, BL]。数学上精确，见模块开头说明。
    """
    return seg_intersect(q[0], q[2], q[1], q[3])


# =============================================================================
# 合法性与尺度
# =============================================================================
def opp_side_convergence(q):
    """
    投影矩形的一条**尺度无关自洽性**判据。

    事实：矩形的对边本来平行；一旦投影带有透视，对边就会收敛（相交于消失点），
    而"收敛"必然同时表现为**对边长度不相等**。两者是同一件事的两种表现，
    所以可以互相校验：

        实测夹角 meas  ≈  由长度差推算的 exp = atan( |L1 − L2| / D )，D = 另一组边的平均长

    返回 (meas, exp)，单位度（max 取两组对边里较大的那组）。

    为什么需要它（真机踩到的坑）：
      远距离时靶纸只有 40~60px，**x/y 极值取角**会因为 1px 量化而取错 ——
      "x 最大的那个点"到底是右上角还是右下角，只差 1px 就会翻转。
      结果得到 4 个"互异"点、排出来还是**凸**的、面积也正常，
      于是骗过了 is_quad_valid 的凸性/面积/长宽比检查，中心却偏了 20px。
      这种假四边形的特征很明确：对边夹角 74°，而对边长度差只推得出 43° —— 根本对不上。
      （对照：合法的强透视姿态 θ=60~70°，测量夹角最大 40~51°，但推算值也有 34~43°，
        差值始终 <= ~10°，所以这条判据分得开。）
    """
    q = np.asarray(q, dtype=np.float64).reshape(4, 2)
    e = [q[(i + 1) % 4] - q[i] for i in range(4)]
    L = [float(np.hypot(v[0], v[1])) for v in e]
    if min(L) < 1e-6:
        return 999.0, 0.0
    meas, exp = 0.0, 0.0
    for a, b in ((0, 2), (1, 3)):                       # 两组对边
        u, v = e[a], e[b]
        c = abs(float(u[0] * v[0] + u[1] * v[1]) / (L[a] * L[b]))
        ang = math.degrees(math.acos(max(-1.0, min(1.0, c))))
        d = 0.5 * (L[(a + 1) % 4] + L[(b + 1) % 4])     # 另一组边的平均长度
        ee = math.degrees(math.atan2(abs(L[a] - L[b]), max(1e-6, d)))
        meas, exp = max(meas, ang), max(exp, ee)
    return meas, exp


def is_valid_quad(q, img_w, img_h, cfg):
    """
    四边形合法性：面积下限 + 凸性 + 对边比 + 长宽比 + **对边收敛自洽**。

    面积下限用的是 `MIN_QUAD_AREA_RATIO`（比候选轮廓用的 `MIN_AREA_RATIO` 更小）。
    这两个值**都直接决定"最远能识别多远"**：曾经 MIN_AREA_RATIO=0.02 时，
    1280x720 下 2% = 18432px 要求靶纸宽 >=114px，165cm 处才 141px 宽
    => 超过约 2.0m 就一个候选都产生不出来（真机表现：近了能锁、远了完全锁不住）。

    最后一条 `opp_side_convergence` 是后来补的：**只有凸性 + 面积 + 长宽比挡不住
    "小目标上极值取错角"**（那种假四边形是凸的、面积也正常），必须再加这条
    尺度无关的自洽性判据，否则远距离会给出偏 20px 的错误中心。见该函数 docstring。
    """
    ratio = float(getattr(cfg, "MIN_QUAD_AREA_RATIO", cfg.MIN_AREA_RATIO))
    if quad_area(q) < ratio * img_w * img_h:
        return False
    if not is_convex(q):
        return False
    w1, w2 = dist(q[0], q[1]), dist(q[3], q[2])
    h1, h2 = dist(q[1], q[2]), dist(q[0], q[3])
    if min(w1, w2, h1, h2) < float(getattr(cfg, "MIN_QUAD_SIDE_PX", 1.0)):
        return False
    for a, b in ((w1, w2), (h1, h2)):
        if min(a, b) < 1e-3 or max(a, b) / min(a, b) > cfg.OPP_RATIO_MAX:
            return False
    ar = ((w1 + w2) / 2.0) / max(1e-3, (h1 + h2) / 2.0)
    if not (cfg.ASPECT_MIN <= ar <= cfg.ASPECT_MAX):
        return False
    meas, exp = opp_side_convergence(q)
    return meas <= exp + float(getattr(cfg, "OPP_ANGLE_SLACK", 18.0))


def estimate_scale(q, cfg, inner=False):
    """
    ⑥ 用 A4 已知边长反算 px_per_cm（局部尺度，用于把像素误差换算成 cm）

    inner=False：q 是**黑胶带外沿**（= 纸边），对应 21.0 x 29.7 cm；
    inner=True ：q 是**胶带内沿**（= 纸面可见区），对应 21-2w x 29.7-2w cm。

    为什么需要两种：外沿与内沿可能因为"胶带没贴均匀"而不同心，
    所以求靶心优先用外沿（更接近纸边、更准）；外沿被深色背景/干扰吃掉时，
    才退回用内沿求中心（同心度略差，但至少能锁住框）。
    """
    w_cm = cfg.A4_IN_W_CM if inner else cfg.A4_W_CM
    h_cm = cfg.A4_IN_H_CM if inner else cfg.A4_H_CM
    w = (dist(q[0], q[1]) + dist(q[3], q[2])) / 2.0     # 上、下边均值
    h = (dist(q[0], q[3]) + dist(q[1], q[2])) / 2.0     # 左、右边均值
    return 0.5 * (w / w_cm + h / h_cm)


def scale_quad(q, s):
    """四边形整体缩放（坐标乘 s）"""
    return np.asarray(q, dtype=np.float64) * float(s)


# =============================================================================
# ⑦ One Euro 滤波器（低延迟强抑抖：动得越快滤波越弱）
# =============================================================================
class OneEuroFilter:
    """u、v 各用一个实例。目标：静止时抖动最小，快速移动时不拖影。"""

    def __init__(self, freq=30.0, min_cutoff=1.0, beta=0.007, d_cutoff=1.0):
        self.freq = freq
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None

    @staticmethod
    def _alpha(cutoff, freq):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau * freq)

    def reset(self):
        self.x_prev, self.dx_prev, self.t_prev = None, 0.0, None

    def __call__(self, x, t):
        if self.t_prev is not None and t > self.t_prev:
            self.freq = 1.0 / max(1e-3, t - self.t_prev)
        if self.x_prev is None:
            self.x_prev, self.t_prev = x, t
            return x
        dx = (x - self.x_prev) * self.freq
        a_d = self._alpha(self.d_cutoff, self.freq)
        dx_hat = a_d * dx + (1 - a_d) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self._alpha(cutoff, self.freq)
        x_hat = a * x + (1 - a) * self.x_prev
        self.x_prev, self.dx_prev, self.t_prev = x_hat, dx_hat, t
        return x_hat
