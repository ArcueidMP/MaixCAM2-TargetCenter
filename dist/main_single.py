#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main_single.py · 25E 靶心识别 —— 单文件自包含版
================================================================================
★ 本文件由 tools/build_single.py 自动生成，请勿手改！
    改参数 -> config.py ；改算法 -> vision/*.py ；改流程 -> main.py
    改完在工程根目录重新执行：  python tools/build_single.py

用途：在 MaixVision 里点「运行当前文件」即可运行（不必用「运行项目」）。
      行为与模块化工程完全一致，输出也就完全一致。

生成时间：2026-10-05 15:43:29
    config.py            sha1=42091430c1c7
    main.py              sha1=c0df766f7db2
    vision/detector.py   sha1=b3d11a0e4aea
    vision/geometry.py   sha1=aeb94b7f8c58
    vision/overlay.py    sha1=ce3ff7f48092
"""

import sys as _sys
import types as _types

_SRC_SHA1 = {'vision/geometry.py': 'aeb94b7f8c588dbe789efcf59d92c814b8105ffe', 'vision/detector.py': 'b3d11a0e4aeae9ebd328b13a4dac1815b8e36874', 'vision/overlay.py': 'ce3ff7f4809228e2fcba0b754b22e2a8725eb1b4', 'config.py': '42091430c1c779e6970295139b209c82d7b328f9', 'main.py': 'c0df766f7db26feaf677e48d79daaf9a29a58a0c'}

# --------------------------------------------------------------------------
# ① 各模块源码原样嵌入，运行时装进 sys.modules（保持原 import 语义）
# --------------------------------------------------------------------------
_MOD_SRC = {}

# ===== vision.geometry  (vision/geometry.py) =====
_MOD_SRC['vision.geometry'] = r'''
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
'''

# ===== vision.detector  (vision/detector.py) =====
_MOD_SRC['vision.detector'] = r'''
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
import time

import numpy as np
import cv2

from . import geometry as G


def _empty():
    return dict(valid=False, u=None, v=None, scale=0.0, quad=None, corners=None,
                stage="none", raw_u=None, raw_v=None, th=None, lvl=0, cand=None,
                ms={}, masks=0)


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


def _top_contours(cnts, limit):
    """
    轮廓太多时，只保留"点数最多的前 limit 条"的下标，其余直接不看。

    ★ 为什么需要它（实测数据说话）
    --------------------------------------------------------------------------
    噪声/纹理二值图上一张掩膜能有 **2 万条**轮廓。旧实现对每条都做
    `contourArea` + `boundingRect` + `minAreaRect`（后者内部要算凸包），
    实测单张掩膜 **260ms**（PC/x86），板上还要乘以十几倍 —— 这就是"每帧几秒"。
    而 `MAX_CANDIDATES` 只在循环**之后**才截断，救不了这一段。

    为什么用"点数"来排序，而不是直接算面积：
        `len(contour)` 是**纯 Python** 就能拿到的量，不需要任何 cv2 调用；
        而"算面积"本身正是我们要躲开的那 2 万次 cv2 调用之一。
        轮廓的点数与它的尺度正相关（周长越大点越多），用它做排序足够选出大轮廓；
        真实靶纸框的轮廓点数（上百）远高于噪点碎片（几个），稳居前列。

    什么时候不截断：`len(cnts) <= limit`（干净场景，本来就没几条轮廓）时
    原样返回 —— 所以常规场景**零额外开销、零行为变化**，只给病态掩膜一个上界。

    返回一个下标序列（list/range，可直接迭代）。
    """
    n = len(cnts)
    lim = max(1, int(limit or 0))
    if n <= lim:
        return range(n)
    sizes = np.fromiter(map(len, cnts), dtype=np.int64, count=n)
    # argpartition 是 O(n)，先粗选出前 lim 个，再只对这小批做精确排序
    sel = np.argpartition(-sizes, lim - 1)[:lim]
    return sel[np.argsort(-sizes[sel], kind="stable")]


def boundary_estimate(bw):
    """
    廉价估计掩膜的"总边界长度"（隔点采样，成本约 0.3ms）。

    做法：按自适应步长抽稀后，统计水平/垂直方向上 0<->255 的跳变个数。
    跳变数正比于总边界长度，而 `findContours` 的开销（要创建的轮廓对象数、
    总点数）也完全由边界长度决定 —— 所以它能**很好地预测 findContours 有多贵**：
    实测（1280x720）| 估计值 | findContours
        20 ~ 600     |  0.7 ~ 3ms     （干净掩膜：一条胶带环）
        9.7k         |  17ms
        13.7k ~ 18.9k|  19 ~ 48ms
        72k          |  190ms         （白噪声 + 精细形态档：2.3 万条轮廓）

    步长随分辨率自适应（约每 160 像素取一个），所以同一个场景在
    1280x720 / 640x480 下估出来的量级一致，阈值不用跟着分辨率改。

    这个量**不参与任何判定结果**，只用来决定"要不要走连通域预筛这条贵路径"。
    """
    h, w = bw.shape[:2]
    step = max(1, int(round(max(w, h) / 160.0)))
    if step <= 1:
        return (int(np.count_nonzero(bw[:, 1:] != bw[:, :-1]))
                + int(np.count_nonzero(bw[1:, :] != bw[:-1, :])))
    a = bw[::step, :]
    c = bw[:, ::step]
    return (int(np.count_nonzero(a[:, 1:] != a[:, :-1]))
            + int(np.count_nonzero(c[1:, :] != c[:-1, :])))


def _survivor_contours(bw, cfg, W, H):
    """
    "掩膜碎成渣"时才走的贵路径：先用连通域统计把碎块整体滤掉，再对幸存者描边。

    为什么需要它：`findContours` 在 2 万条轮廓的掩膜上要 **190ms**（PC/x86），
    因为光是**创建 2 万个小轮廓对象**就占了绝大部分时间 —— 无论后面怎么截断
    （`_top_contours` 只能省掉循环，省不掉这里的对象创建）都已经晚了。
    所以必须先"不生成"这些轮廓：连通域统计一次拿到每块的前景面积与外接框，
    用 numpy 向量化筛出够大的那些，再只在"幸存掩膜"上描边。

    ⚠ 两条"想当然就会静默丢候选"的等价性约束：
      · 判"够大"用 **像素面积 ≥ amin 或 外接框面积 ≥ amin**（并集，保守）。
        不能只看像素面积：环的像素面积 = 外框 − 内孔，比内孔还小；
        亮面族候选（内孔轮廓）的面积可能**大于**它所属连通域的像素面积。
        只用像素面积会把远距离的亮面候选整族砍掉。
        取并集后本预筛是**超集**：正常路径能通过的轮廓，这里一定还在。
      · **这里绝不做"贴边"过滤。** 贴边判断必须留给后面的逐轮廓检查：
        暗背景场景里整幅深色背景是一条**贴边的巨大连通域**，而真正的靶纸
        "纸面"（它的内孔）好好地待在内部、并不贴边。在连通域层面按贴边丢弃，
        会把"暗背景 + 亮面族"这条救命路径一起丢掉。

    因为这条路径只可能在"阈值明显不合理"的碎渣掩膜上触发，这里用
    `CHAIN_APPROX_SIMPLE` 描边（点数更少）；真正承载靶纸的干净掩膜走的是
    正常路径、仍然是 `CHAIN_APPROX_NONE`（亚像素精修需要密点）。

    返回 (cnts, hier)；没有够大的连通域时返回 None。
    """
    if bw is None or bw.size == 0:
        return None
    n, labels, stats, _ = cv2.connectedComponentsWithStats(bw, 8, cv2.CV_32S)
    if n <= 1:                                  # 只有背景
        return None
    st = stats[1:]                              # 去掉 label=0（背景）
    # stats 列序：LEFT=0, TOP=1, WIDTH=2, HEIGHT=3, AREA=4（用整数下标，跨版本稳）
    a = st[:, 4].astype(np.int64)
    box = st[:, 2].astype(np.int64) * st[:, 3].astype(np.int64)
    amin = float(cfg.MIN_AREA_RATIO) * float(W * H)
    idx = np.flatnonzero((a >= amin) | (box >= amin))
    if idx.size == 0:
        return None
    cap = int(getattr(cfg, "MAX_SCAN_COMPONENTS", 256) or 0)
    if cap > 0 and idx.size > cap:
        rank = np.maximum(a, box)[idx]
        idx = idx[np.argsort(-rank, kind="stable")[:cap]]
    lut = np.zeros(n, np.uint8)
    lut[idx + 1] = 255
    surv = lut[labels]                          # 一次 C 级查找，整幅一次搞定
    p = cv2.findContours(surv, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    cnts, hier = p[-2], p[-1]
    if hier is not None and getattr(hier, "ndim", 0) == 3:
        hier = hier[0]
    return cnts, hier


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

    三趟走，为了远距离也不拖慢帧率（顺序很重要，别调换）：
        第 0 趟 `boundary_estimate` + `_survivor_contours`：先用约 0.3ms 的
                 "边界长度估计"判断这张掩膜是不是"碎成渣"（2 万条轮廓那种）。
                 是 -> 走连通域预筛，只对少数幸存连通域描边（避开 findContours
                 创建 2 万个轮廓对象的 190ms）；不是 -> 正常 findContours 描边，
                 零额外开销。**干净掩膜永远走正常路径，行为与优化前完全一致。**
        第 0′ 趟 `_top_contours`：轮廓仍然很多时，按点数只留前 N 条 ——
                 让后面那批"逐条 cv2 特征计算"的耗时有上界。
        第一趟只算"便宜"的特征（面积/外接框/最小外接矩形）先过硬门槛，把候选砍到
                 MAX_CANDIDATES 个；第二趟才对幸存者算灰度探针（要读图）并打分。
        MIN_AREA_RATIO 从 0.02 调小后，暗背景下可能出现成百上千个小轮廓，
        没有这三趟预筛 + 上限，帧率会被直接拖垮。
    """
    est = boundary_estimate(bw)
    if est > int(getattr(cfg, "FRAG_BOUNDARY_EST", 8000)):
        # 碎渣掩膜：先滤碎块，避免 findContours 为 2 万条轮廓创建对象
        got = _survivor_contours(bw, cfg, W, H)
        if got is None:
            return []
        cnts, hier = got
    else:
        p = cv2.findContours(bw, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        cnts, hier = p[-2], p[-1]
        if not cnts:
            return []
        if hier is not None and getattr(hier, "ndim", 0) == 3:
            hier = hier[0]
    scan_order = _top_contours(cnts, getattr(cfg, "MAX_SCAN_CONTOURS", 512))

    img_a = float(W * H)
    amin = float(cfg.MIN_AREA_RATIO) * img_a
    bpx = int(getattr(cfg, "SCORE_BORDER_PX", 2))
    max_c = max(1, int(getattr(cfg, "MAX_CANDIDATES", 60)))
    allow_border = bool(getattr(cfg, "SCORE_ALLOW_BORDER", False))

    # ---- 第一趟：便宜特征 + 硬门槛 ----
    cheap = []
    for i in scan_order:
        c = cnts[i]
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
        # 只取视图、不拷贝：整条轮廓可能上万点，逐候选 float64 拷贝是纯浪费。
        # 16 个等距采样点才需要被后续数学用到；`pts` 交给下游按需转换。
        pp = c.reshape(-1, 2)
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
        self._dbg_pass = ""           # last_debug 来自哪一趟（coarse/full/roi/track）
        self.last_debug = []          # 最近一帧各阈值的候选情况（供板端打印诊断）
        self._sweep_cursor = 0        # 扫描表的轮转游标（每帧掩膜预算用，见 _masks）
        self.last_masks = 0           # 上一帧实际消费的掩膜数（性能观测用）
        self._last_quad = None        # 上一帧锁定的四边形（帧间 ROI 跟踪的种子）
        self._track_miss = 0          # 连续多少帧没能靠 ROI 跟住

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
        本帧要试的 (阈值, 形态学档位) 列表（按优先级），外加"必试项个数" n_pri。

        排序策略（与旧实现完全同序，只在其前面插了一个"上一帧锁定值"）：
            ① 上一帧锁定的那个阈值+形态档 —— **放最前面**；
            ② 常规形态学：先优先阈值（固定 / Otsu），再扫描表 THRESH_SWEEP；
            ③ 精细形态学：整块再来一遍（救远距离被腐蚀掉的细环）。
        这样近处（常规够用）在第一轮就命中并提前退出，不会为"远距离"付额外代价；
        只有真的什么都找不到时，才会花第二轮去救细环。

        ★ 为什么要把"上一帧锁定值"提到最前（这是"板上 0.1fps"的直接原因之一）：
            "缩小图没锁住 → 全分辨率升级"那一趟用的是 force_sweep=True
            （理由见 docstring 末尾），于是它**无视已有的锁定**、把 12 个阈值 ×
            2 个形态档 = 24 个掩膜在**全分辨率**上重扫一遍。一旦"缩小图这一遍
            长期失败"（远距离时胶带在 0.5 倍图上只有 1px，很容易失败），
            就变成**每一帧**都在全分辨率上重扫 24 个掩膜 —— 每帧几秒。
            把锁定值排到最前，最常见的"缩小图没看出来、全分辨率一看就中"这种情况
            第 1 个掩膜就命中并 break（生成器让 break 真正省下后面的工作）。

        返回 (specs, n_pri)：n_pri 之前的条目是"每帧必试"的，不参与预算削减。

        force=True：无视已有的锁定、强制重建完整扫描表。用于"缩小图这一遍没找到
        → 全分辨率升级这一遍"，否则上一遍留下的（可能是错的）锁会把升级这一遍
        一起锁死，永远救不回来。
        """
        cfg = self.cfg
        if self._th_lock is not None and not force:
            return [(int(self._th_lock), int(self._lvl_lock))], 1

        if getattr(cfg, "SELECT_MODE", "score") == "largest":
            mode = "otsu" if cfg.USE_OTSU else "fixed"     # 完全复现旧行为
        else:
            mode = getattr(cfg, "THRESH_CANDIDATES", "auto")
            if cfg.USE_OTSU and mode == "fixed":
                mode = "otsu"

        # 优先阈值：固定 / Otsu（Otsu 要对本图算一次直方图，很便宜）
        pri = []
        if mode in ("fixed", "auto"):
            pri.append(int(cfg.BLACK_THRESH))
        if mode in ("otsu", "auto"):
            t, _ = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            pri.append(int(round(float(t))))
        # 扫描表：只有"需要重新标定"的帧才加（限流见 THRESH_RELOCK_EVERY）
        sweep = []
        if (mode != "fixed" and getattr(cfg, "THRESH_RELOCK", False)
                and (force or self._frame_i >= self._next_sweep)):
            sweep = [int(v) for v in getattr(cfg, "THRESH_SWEEP", ())]

        def _norm(v):
            return max(1, min(254, int(v)))

        lvls = [0] + ([1] if getattr(cfg, "MORPH_SEARCH", False) else [])
        specs, seen = [], set()
        if self._th_lock is not None:                  # ① 锁定值（含它自己的形态档）
            s = (_norm(self._th_lock), int(self._lvl_lock))
            specs.append(s)
            seen.add(s)
        for v in pri:                                  # ② 常规形态学下的优先阈值
            s = (_norm(v), 0)
            if s not in seen:
                seen.add(s)
                specs.append(s)
        n_pri = len(specs)                             # ← 到此为止每帧必试
        for v in sweep:                                # ②′ 常规形态学下的扫描表
            s = (_norm(v), 0)
            if s not in seen:
                seen.add(s)
                specs.append(s)
        for lvl in lvls[1:]:                           # ③ 精细形态学整块再来一遍
            for v in pri + sweep:
                s = (_norm(v), lvl)
                if s not in seen:
                    seen.add(s)
                    specs.append(s)
        return specs, n_pri

    def _masks(self, g, force=False, budget=None):
        """
        产出 (阈值, 形态学档位, 二值图)；阈值 -1 表示自适应阈值（无固定阈值）。

        两个关键点，都是"0.1fps"的成因：
          · **生成器**：旧实现一次性把所有掩膜算完再返回，于是 `_detect_on` 里
            的"命中就 break"只省下了描边，**省不掉阈值化 + 形态学** ——
            24 个掩膜照样全做完。改成生成器后 break 才是真的 break。
          · **每帧预算 + 轮转游标**：全分辨率兜底那一趟最坏要过 24 个掩膜，
            单帧耗时没有上界。这里给"必试项"之外的扫描表加预算，超出的部分
            留到下一帧接着扫（游标轮转），既给单帧耗时一个上界，
            又不牺牲"最终一定能把扫描表扫完"。
        """
        specs, n_pri = self._mask_specs(g, force)
        if budget is not None and len(specs) > n_pri:
            budget = max(int(budget), n_pri)
            if len(specs) > budget:
                tail = specs[n_pri:]
                k = min(budget - n_pri, len(tail))
                off = int(self._sweep_cursor) % len(tail)
                win = [tail[(off + i) % len(tail)] for i in range(k)]
                self._sweep_cursor = off + k
                specs = specs[:n_pri] + win
        for th, lvl in specs:
            _, bw = cv2.threshold(g, th, 255, cv2.THRESH_BINARY_INV)
            yield th, lvl, self._morph(bw, lvl)
        if (getattr(self.cfg, "USE_ADAPTIVE", False)
                and self._th_lock is None and self._frame_i >= self._next_sweep):
            blk = int(self.cfg.ADAPTIVE_BLOCK) | 1
            bw = cv2.adaptiveThreshold(g, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                       cv2.THRESH_BINARY_INV, max(3, blk),
                                       float(self.cfg.ADAPTIVE_C))
            yield -1, 0, self._morph(bw, 0)

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

    def _detect_on(self, g, tag="", force_sweep=False, budget=None):
        """
        在一张（可能已缩小的）图上完成 ①~⑤。

        会遍历本帧允许尝试的 (阈值, 形态学档位)，用**结构分**在"mask x 候选轮廓"里
        选最优；首次（或重新标定）会顺带把胜出的组合锁下来。
        tag 只用于诊断：标出这批 dbg 来自哪一趟（coarse / full / roi / track）；
        force_sweep=True 用于"缩小图失败 → 全分辨率升级"这一趟（见 _mask_specs）。
        budget：本帧最多消费几个掩膜（None = 不限）。见 _masks 的说明。
        """
        cfg = self.cfg
        h, w = g.shape[:2]
        sweeping = force_sweep or (self._th_lock is None)
        bonus = float(getattr(cfg, "THRESH_LOCK_BONUS", 0.0))
        best, dbg = None, []
        n_mask = 0

        lo = float(getattr(cfg, "MASK_FG_MIN", 0.002))
        hi = float(getattr(cfg, "MASK_FG_MAX", 0.93))
        px = float(w * h)

        for th, lvl, bw in self._masks(g, force_sweep, budget=budget):
            n_mask += 1
            # 前景占比用 countNonZero（C 级计数）算；旧写法 (bw>0).mean() 会先
            # 分配一整幅 bool 数组再归约，每个掩膜白花一次 O(W·H) 的分配+归约。
            fg = cv2.countNonZero(bw) / px
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
        self.last_masks = n_mask            # 本帧实际过了几个掩膜（[fps] 日志会显示）
        if sweeping:
            if best is not None:
                self._th_lock = int(best["th"])      # 锁定胜出组合：后续帧零额外开销
                self._lvl_lock = int(best["lvl"])
                self._th_prev = int(best["th"])
                self._sweep_cursor = 0               # 锁定变了：扫描预算游标归零
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
        cfg = self.cfg
        H, W = gray.shape[:2]
        # WORK_SCALE 默认 None（表示"由 main.py 按相机档位注入"），单独用本类时必须兜底
        _ws = getattr(cfg, "WORK_SCALE", None)
        ws = 1.0 if _ws is None else float(_ws)
        ws = max(0.15, min(1.0, ws))

        out, stage = None, "none"
        # 分段耗时（板上靠这个判断瓶颈到底消没消；perf_counter 开销可忽略）
        ms = {}
        t0 = time.perf_counter()

        # ---- 第 0 遍（可关）：帧间 ROI 跟踪 ----
        # 靶纸是静止的，帧间位移很小。直接用上一帧的四边形扩一圈当搜索区，
        # 在**全分辨率**上只解这一小块 —— 稳态耗时降到全局搜索的几分之一。
        # 跟丢/跳变超限就返回 None，自动落回下面的全局搜索，所以只会更快、不会更差。
        if (getattr(cfg, "ROI_TRACK", True) and self._last_quad is not None
                and ws < 0.999):
            tr = self._try_track(gray)
            ms["track"] = (time.perf_counter() - t0) * 1000.0
            if tr is not None:
                out, stage = tr, "track"

        # ---- 第一遍：缩小图上"粗定位"板框（省内存 / 省时间）----
        if out is None and ws < 0.999:
            t0 = time.perf_counter()
            sw, sh = max(64, int(round(W * ws))), max(64, int(round(H * ws)))
            self._small = cv2.resize(gray, (sw, sh), interpolation=cv2.INTER_AREA,
                                     dst=self._small)
            d = self._detect_on(self._small, "coarse")
            ms["coarse"] = (time.perf_counter() - t0) * 1000.0
            if d is not None:
                sx, sy = W / float(sw), H / float(sh)
                out = dict(quad=d["quad"] * np.array([sx, sy], dtype=np.float64),
                           center=np.array([d["center"][0] * sx, d["center"][1] * sy],
                                           dtype=np.float64),
                           method=d["method"], info=d["info"], th=d["th"],
                           lvl=d.get("lvl", 0))
                stage = "coarse%.2f" % ws

        # ---- 缩小图（或在禁用缩小时的首帧）失败：整幅全分辨率直接解算 ----
        # force_sweep=True：这一趟是"升级"，必须无视上一趟留下的（可能是错的）锁定。
        # budget：给这一趟一个"每帧最多几个掩膜"的上界 —— 它在全分辨率上跑，
        #         24 个掩膜无上界就是"每帧几秒"。超出的部分下一帧接着扫（游标轮转），
        #         所以既不牺牲"最终扫得完"，也不让单帧爆炸。
        if out is None:
            t0 = time.perf_counter()
            d = self._detect_on(gray, "full", force_sweep=True,
                                budget=int(getattr(cfg, "MAX_MASKS_FULL", 8)))
            ms["full"] = (time.perf_counter() - t0) * 1000.0
            if d is None:
                self._book_keep(False)
                self._last_quad = None
                r = _empty()
                r["ms"], r["masks"] = ms, self.last_masks
                return r
            out, stage = d, "full"

        # ---- 第二遍：在粗定位外接框内用全分辨率精算 ----
        # 缩小图会把边缘量化到小的像素栅格上（约 ±1 个原始像素的偏置）；
        # 这一遍只处理"靶纸附近"的一小块，既拿回全分辨率精度，
        # 内存占用又只与目标在画面中的大小相关。
        if ws < 0.999 and stage != "track":
            t0 = time.perf_counter()
            refined = self._refine_in_roi(gray, out["quad"], tag="roi")
            ms["roi"] = (time.perf_counter() - t0) * 1000.0
            if refined is not None:
                out = refined
                stage += "+roi"

        q = out["quad"]
        c = out["center"]
        # 用"深色环外沿(纸边 21x29.7)"还是"亮面矩形(胶带内沿 17.4x26.1)"换算物理尺度
        info = out.get("info") or {}
        inner = (info.get("kind") == "paper")
        scale = G.estimate_scale(q, cfg, inner=inner)

        # ⑦ 时序滤波
        if timestamp is None:
            u_f, v_f = float(c[0]), float(c[1])
        else:
            u_f = self.fu(float(c[0]), timestamp)
            v_f = self.fv(float(c[1]), timestamp)

        self._book_keep(True)
        self._last_quad = np.asarray(q, dtype=np.float64)   # 给下一帧的 ROI 跟踪当种子
        return dict(valid=True, u=float(u_f), v=float(v_f), scale=float(scale),
                    quad=q, corners=q,
                    stage="%s+%s%s" % (stage, out["method"],
                                       "/paper" if inner else ""),
                    raw_u=float(c[0]), raw_v=float(c[1]),
                    th=out.get("th"), lvl=out.get("lvl", 0), cand=out.get("info"),
                    ms=ms, masks=self.last_masks)

    def _try_track(self, gray):
        """
        帧间 ROI 跟踪：拿上一帧的四边形扩一圈当搜索区，只在这一小块上全分辨率解算。

        返回结果字典，或 None（跟丢 → 调用方落回全局搜索）。

        为什么要"跳变门限"：跟踪最大的风险是**跟着跟着跑到别的干扰物上去了**
        （裁剪区里另一个东西结构分更高）。所以跟到的框必须和上一帧的框基本重合，
        否则宁可丢掉这一帧、回全局搜索重找 —— 全局搜索只是慢一点，绝不会锁错。
        """
        cfg = self.cfg
        q_prev = np.asarray(self._last_quad, dtype=np.float64)
        r = self._refine_in_roi(
            gray, q_prev, tag="track",
            margin=float(getattr(cfg, "ROI_TRACK_MARGIN", 0.25)))
        if r is None:
            self._track_miss += 1
            return None
        c0 = q_prev.mean(axis=0)
        c1 = np.asarray(r["quad"], dtype=np.float64).mean(axis=0)
        span = max(1.0, float(max(np.ptp(q_prev[:, 0]), np.ptp(q_prev[:, 1]))))
        jump = float(np.hypot(c1[0] - c0[0], c1[1] - c0[1]))
        if jump > float(getattr(cfg, "ROI_TRACK_MAX_JUMP", 0.6)) * span:
            self._track_miss += 1
            return None
        self._track_miss = 0
        return r

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
            self._sweep_cursor = 0          # 解锁后从头扫，别接着上一轮的残留游标
            self._miss = 0

    def _refine_in_roi(self, gray, quad, tag="roi", margin=0.08):
        """
        用给定的四边形裁出一小块（外接框 + margin 余量 + 6px），在里面以全分辨率重解。
        返回裁剪坐标系已平移回全图的字典；不满足条件时返回 None（沿用上层结果）。

        margin：0.08 是"粗定位后精算"用的紧余量（缩小图量化偏差很小）；
                帧间跟踪（tag="track"）要放大到 0.25 —— 目标帧间会移动，
                余量太小会让框贴到裁剪边界而被判"可能被裁断"、白白跟丢。
        """
        H, W = gray.shape[:2]
        q = np.asarray(quad, dtype=np.float64)
        if q.shape != (4, 2) or not np.isfinite(q).all():
            return None
        xmin, ymin = q.min(axis=0)
        xmax, ymax = q.max(axis=0)
        bw, bh = xmax - xmin, ymax - ymin
        if bw < 8 or bh < 8:
            return None
        mx, my = float(margin) * bw + 6.0, float(margin) * bh + 6.0
        x0 = int(max(0, np.floor(xmin - mx)))
        y0 = int(max(0, np.floor(ymin - my)))
        x1 = int(min(W, np.ceil(xmax + mx)))
        y1 = int(min(H, np.ceil(ymax + my)))
        cw, ch = x1 - x0, y1 - y0
        if cw < 48 or ch < 48:
            return None
        if cw * ch > 0.85 * W * H:          # 框几乎等于整幅，不值得再来一遍
            return None

        d = self._detect_on(gray[y0:y1, x0:x1], tag)
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
        self._sweep_cursor = 0
        self.last_masks = 0
        self._last_quad = None
        self._track_miss = 0
'''

# ===== vision.overlay  (vision/overlay.py) =====
_MOD_SRC['vision.overlay'] = r'''
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
overlay.py · 结果可视化（OpenCV 绘制，预分配画布，零逐帧内存分配）
================================================================================
为什么不用 MaixPy 原生 draw_* ？
    ① 相机现在输出 GRAYSCALE，在其上画彩色标注行为不确定；
    ② 直接在 1280x720 原图上绘制会一直持有一张大缓冲。
做法：把灰度图缩小到 SHOW_W 宽（默认 640），在**复用的** RGB 画布上画标注，
      再交给 display 显示。画布只在启动时分配一次，之后每帧复用同一块内存。
"""

import numpy as np
import cv2


class Overlay:
    def __init__(self, cfg):
        self.cfg = cfg
        self._gray = None       # HxW uint8，复用
        self._canvas = None     # HxWx3 RGB888，复用

    def _ensure(self, h, w):
        if self._gray is None or self._gray.shape[0] != h or self._gray.shape[1] != w:
            self._gray = np.empty((h, w), np.uint8)
            self._canvas = np.empty((h, w, 3), np.uint8)

    def render(self, gray, res, fps=None, extra=""):
        """
        gray : 全分辨率灰度图（HxW）
        res  : detector.process() 的返回（坐标为全分辨率像素）
        返回 : RGB888 的 numpy 画布（内存复用，下一帧会被覆盖）
        """
        H, W = gray.shape[:2]
        cw = int(self.cfg.SHOW_W)
        ch = max(2, int(round(cw * H / float(max(1, W)))))
        self._ensure(ch, cw)
        cv2.resize(gray, (cw, ch), dst=self._gray, interpolation=cv2.INTER_AREA)
        cv2.cvtColor(self._gray, cv2.COLOR_GRAY2RGB, dst=self._canvas)
        img = self._canvas
        s = cw / float(max(1, W))          # 全分辨率 -> 画布 的缩放

        if res.get("valid"):
            q = np.rint(np.asarray(res["quad"], dtype=np.float64) * s).astype(np.int32)
            # 板框 4 条边（绿）
            cv2.polylines(img, [q.reshape(-1, 1, 2)], True, (0, 255, 0), 2)
            # 求交用的两条对角线（蓝）
            cv2.line(img, tuple(q[0]), tuple(q[2]), (255, 60, 60), 1)
            cv2.line(img, tuple(q[1]), tuple(q[3]), (255, 60, 60), 1)
            # 4 个顶点（黄）
            for p, lb in zip(q, ("TL", "TR", "BR", "BL")):
                cv2.circle(img, (int(p[0]), int(p[1])), 5, (0, 255, 255), 2)
                cv2.putText(img, lb, (int(p[0]) + 7, int(p[1]) + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
            # 靶心（红）
            c = (int(round(res["u"] * s)), int(round(res["v"] * s)))
            cv2.drawMarker(img, c, (255, 0, 0), cv2.MARKER_CROSS, 22, 2)
            cv2.circle(img, c, 2, (255, 0, 0), -1)
            txt = "center=(%.1f, %.1f)  %.2f px/cm  %s" % (
                res["u"], res["v"], res["scale"], res["stage"])
        else:
            cv2.putText(img, "TARGET NOT FOUND", (8, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2, cv2.LINE_AA)
            txt = res.get("stage", "")

        cv2.rectangle(img, (0, 0), (cw - 1, 20), (0, 0, 0), -1)
        cv2.putText(img, txt, (6, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (255, 255, 255), 1, cv2.LINE_AA)
        if fps is not None:
            cv2.putText(img, "%.1f fps" % fps, (cw - 74, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1, cv2.LINE_AA)
        if extra:
            cv2.putText(img, extra, (6, ch - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (200, 200, 200), 1, cv2.LINE_AA)
        return img
'''

# ===== config  (config.py) =====
_MOD_SRC['config'] = r'''
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
config.py · 25E 靶心识别 —— 现场标定 & 运行档位
================================================================================
这是**唯一需要按现场情况修改**的文件。刻意不 import maix / numpy / cv2，
保证 PC 端 tools/test_pc.py 能直接导入它做离线验证。

对应赛题：2025 全国大学生电子设计竞赛本科组 E 题《简易自行瞄准装置》
    靶面 = A4 紫外感光纸 + 四周 1.8cm 黑胶带勾出的矩形；靶心由黑框几何反算。
    本视觉模块输出：靶心像素坐标 (u, v) + 局部尺度 px_per_cm。
"""


class Config:
    # =========================================================================
    # 一、运行档位（分辨率 / 格式 / 内存）
    # =========================================================================
    # 内存是第一优先级：MaixPy 摄像头缓冲按 分辨率 x 每像素字节数 x buff_num
    # 占用「快速内存(CMM/ION)」，RGB888 是 3 B/px，灰度直出(GRAYSCALE)只要 1 B/px。
    # 本工程默认灰度直出 + buff_num=2，同一个 1280x720 相比 RGB888/buff3
    # 省下约 4.6 倍摄像头内存（8.3MB -> 1.8MB），这是解决
    # "算法需要的帧缓冲内存超过开发板可用内存" 的关键。
    #
    # 字段：(档位名, 宽, 高, 相机格式, buff_num, fps, work_scale)
    #   work_scale：算法在「宽 x work_scale」的缩小图上做二值化/描边，
    #               坐标最后换算回全分辨率；缩小图失败才回退到全分辨率重算。
    #               0.5 = 用 1/4 的像素量，内存与耗时都大幅下降，精度不受影响。
    PROFILES = (
        # 档位名        宽    高   相机格式      buff  fps  work_scale
        ("standard", 1280, 720, "GRAYSCALE", 2, 30, 0.50),  # 默认：165cm 处靶纸约 141px
        ("lowmem",    640, 480, "GRAYSCALE", 2, 30, 0.75),  # 快速内存更紧时用
        ("lowmem32",  640, 480, "GRAYSCALE", 1, 30, 0.75),  # 兜底：单缓冲
    )
    PROFILE = "standard"        # 启动用哪个档位；更省内存的档位会自动兜底

    # 算法工作缩放比：由 main.py 按实际启用的档位覆盖（PC 端测试可自己设）。
    # 1.0 = 不缩小（全分辨率解算）；0.5 = 用 1/4 像素量。
    WORK_SCALE = None

    # =========================================================================
    # 二、相机
    # =========================================================================
    LOCK_AE_AWB = True          # 锁定曝光 + 白平衡（固定阈值稳定的前提）
    EXPOSURE = 6000             # TODO 现场标定
    GAIN = 40                   # TODO 现场标定
    # MaixCAM2 官方推荐初值 [R, Gr, Gb, B]；MaixCAM 请改成 [0.134,0.0625,0.0625,0.1239]
    WB_GAIN = [0.0682, 0.0, 0.0, 0.04897]
    H_MIRROR = False            # 画面左右镜像
    V_FLIP = False              # 画面上下翻转
    LENS_CORR = False           # 镜头畸变校正（软件矫正，耗 CPU，边缘直线弯时再开）
    LENS_STRENGTH = 1.8
    SKIP_FRAMES = 20            # 丢弃开头若干帧，避开相机启动不稳定期

    # =========================================================================
    # 三、① 二值化（锁定黑胶带）
    # =========================================================================
    BLACK_THRESH = 95           # 灰度 < 该值判为黑胶带（锁定曝光后固定阈值最稳）
    USE_OTSU = False            # 旧开关，已被下面的 THRESH_CANDIDATES 取代（保留兼容）
    MORPH_OPEN_K = 3            # 开运算核：去孤立噪点
    MORPH_CLOSE_K = 9           # 闭运算核：补断裂的框（在缩小图上做，等效更大）
    # ---- 形态学的"远距离"问题：核太大 = 把细胶带环直接抹掉 ----
    # 远距离时（比如 3m）胶带在缩小图上只有 1~2px 宽，MORPH_OPEN_K=3 会把它腐蚀掉；
    # 一旦环没了，纸面就不再是"内孔"，两族候选同时失效 —— 表现为"远距离完全锁不住"。
    # 所以把形态学也当成一个**搜索维度**（和阈值一样）：先试常规核，全都不行再试"精细核"。
    # 命中后连形态学档位一起锁住，稳态零额外开销。
    MORPH_SEARCH = True         # 扫描阈值时同时扫形态学档位（False = 只用常规核）
    MORPH_OPEN_K_FINE = 1       # 精细档开运算核（1 = 不做开运算，保住细环）
    MORPH_CLOSE_K_FINE = 3      # 精细档闭运算核（补 <=3px 的断口即可）

    # ---- ③′ 候选筛选：真机光照下"取面积最大的轮廓"极易锁错，改为按结构打分 ----
    # 为什么：深色背景/桌面/干扰物都可能比胶带框更大；黑框贴边时轮廓会被画面切断。
    # 黑框稳定的结构特征是 —— 一块**深色细环，把一块亮面围在里面**，且外沿接近矩形。
    # "largest" = 旧行为（仅用于对照/回退）；"score" = 结构打分（默认，推荐）
    SELECT_MODE = "score"
    SCORE_MIN_FILL = 0.62        # 轮廓 / 最小外接矩形 面积比 下限（矩形外沿应≈1）
    # 内孔 / 外框 面积比：胶带 1.8cm 贴在 21x29.7 上 -> (17.4*26.1)/(21*29.7) ≈ 0.73。
    # 下限给 0.45：真实的环都远高于它，而"深色背景与胶带连成一片"的伪环只有 0.34 左右，
    # 正好被挡掉（它虽然形状像矩形，但"环"太厚，不可能是 1.8cm 的胶带）。
    SCORE_MIN_HOLE = 0.45
    SCORE_MAX_HOLE = 0.95
    SCORE_MIN_HOLEFILL = 0.45    # 内孔 / 内孔外接矩形 下限（内孔本身也应是矩形）
    SCORE_RING_MIN = 0.015       # 环厚 / 外框等效边长 下限（1.8cm 胶带实测≈0.07）
    SCORE_RING_MAX = 0.30
    SCORE_ASPECT_LO = 0.85       # 最小外接矩形长宽比区间（宽松，含透视；A4 竖放 1.414）
    SCORE_ASPECT_HI = 2.45
    SCORE_BORDER_PX = 2          # 轮廓进入画面边缘这么多像素内即认为贴边、不可信
    SCORE_ALLOW_BORDER = False   # 靶纸黑框永远不该贴边；确实会贴边时才置 True
    SCORE_EARLY_STOP = 0.90      # 分数够高就不必再试其它阈值（省时间）
    SCORE_MIN_CONTRAST = 12      # 环族：框-纸面对比度 的"无意义"下限
    SCORE_GOOD_CONTRAST = 60     # 环族：达到该对比度即认为阈值合理
    PAPER_MIN_CONTRAST = 85      # 亮面族：亮面-深色带 对比度下限（硬性，低于此直接否决）
                                 # 依据：黑胶带反射率 <0.15、纸面 >0.7，正常曝光下两者
                                 # 灰度差至少 ~100 级；只有 ~70 级说明"深色"其实是背景，
                                 # 不是胶带（实测该门槛挡掉了"亮块贴亮墙"这类误锁）
    PAPER_GOOD_CONTRAST = 180    # 亮面族：达到该对比度即认为阈值合理
    # 打分全不通过时是否退回"最大轮廓"。
    # **默认必须关**：实测开着它，画面里"有矩形外形但不是环"的东西（实心深色块、
    # 黑板、纯墙的渐变区）会被救回来变成"合法四边形"，误锁到完全错误的位置 ——
    # 误检比漏检更糟（云台会照着错位置打）。关掉后这些场景全部正确拒检。
    SCORE_FALLBACK_LARGEST = False

    # ---- ③‴ 互补结构：深色环(胶带外沿) vs 亮面矩形(纸面=胶带内沿) ----
    # 为什么必须要两种：胶带一旦与任何深色区域相邻（暗背景/桌面/干扰物），
    # 两者在二值图里会连成一片，**胶带外沿不再是闭合轮廓**，此时无论怎么调阈值
    # 都取不到环 —— 但"纸面"这块亮矩形始终被深色胶带围着，轮廓完好。
    # 两者同心（内沿=外沿向内收 1.8cm），所以中心都能用；优先外沿（更接近纸边、更准）。
    RING_BONUS = 0.08            # 深色环候选的加分：分数接近时优先用外沿求中心
    TAPE_W_CM = 1.8              # 黑胶带宽度（用于把"纸面"换算成物理尺寸）

    # ---- ③″ 阈值自动标定（现场没标定好曝光时的救命机制）----
    THRESH_CANDIDATES = "auto"   # "fixed" / "otsu" / "auto"（两者都试，按结构分择优）
    THRESH_RELOCK = True         # 一旦锁定阈值就只算这一个；连续失败才重新扫描
    THRESH_RELOCK_MISS = 3       # 连续失败多少帧后解锁重扫
    THRESH_RELOCK_EVERY = 15     # 全扫也失败时，至少隔多少帧才允许再全扫一次
    # 重扫时给"上一帧锁定的那个阈值"一点加分（迟滞），避免两个阈值分数接近时来回跳：
    # 阈值一抖，二值图就变，锁定框和 px_per_cm 都会跟着跳 —— 这正是"锁定效果不佳"的观感来源。
    THRESH_LOCK_BONUS = 0.05
    # 重新标定时扫描的固定阈值（覆盖"过曝/欠曝/强反光"的大范围；结构打分负责挡住误检）
    THRESH_SWEEP = (30, 45, 60, 75, 90, 110, 130, 150, 175, 205)
    USE_ADAPTIVE = False         # 自适应阈值（光照极不均时可试，噪点多时慎用）
    ADAPTIVE_BLOCK = 31
    ADAPTIVE_C = 8
    # 掩膜"前景(深色)占比"的合理区间。越界说明这个阈值根本不合理，直接跳过：
    #   ~0    -> 阈值太低，画面里什么都没有（黑框根本没切出来）
    #   接近 1 -> 阈值高于背景亮度，把整个背景都当成了"深色"
    # ⚠ 这个区间**必须按"靶纸可能很小"来定**，否则远距离会全被拦掉：
    #   · 下限：3.4m 处胶带环只占画面 0.15%、5.2m 处只占 0.06%，
    #     原来写 0.002(=0.2%) 时这些远距离目标**每个阈值都判"前景太少"直接跳过**；
    #   · 上限：暗背景场景里"背景+胶带连成一片"本来就该接近 1
    #     （近处还只到 0.81，远处靶纸一小，这个比例直接冲到 0.996）。
    #   真正防误检的是 SCORE_* 结构打分（"全是深色"压根没有内孔，本来就出不了候选），
    #   不是这两个数，所以这里只留一个"彻底退化成全黑/全亮"的保险。
    MASK_FG_MIN = 0.0003
    MASK_FG_MAX = 0.999

    # =========================================================================
    # 四、③ 几何筛选 & 角点精修
    # =========================================================================
    # ⚠ 这个值**同时**是「候选轮廓面积下限」和「is_valid_quad 的面积下限」，
    #   所以它直接决定"最远能识别到多少距离"。曾经取 0.02，实测后果是：
    #     1280x720 下 2% = 18432px；A4 长宽比 1.414 -> 要求靶纸宽 >=114px；
    #     165cm 处靶纸才 141px 宽（约 6.7px/cm）=> 只要超过约 **2.0m**，
    #     靶纸面积就掉到 2% 以下，**在任何阈值下都一个候选轮廓都产生不出来**
    #     （真机上表现为"近距离能锁、远距离完全锁不住"，日志里连候选都没有）。
    #   改小的依据：真正的噪声/误检是靠 SCORE_* 结构打分挡的，不该靠"面积要够大"。
    #   取 0.002 -> 1280x720 下 1843px -> 靶纸宽约 36px（约 5.6m）仍可检出。
    MIN_AREA_RATIO = 0.002
    # is_valid_quad 用的四边形面积下限（与候选轮廓下限分开，可以更小）
    MIN_QUAD_AREA_RATIO = 0.0006
    MIN_QUAD_SIDE_PX = 5.0           # 四边形最短边下限（远距离小目标也要有个下限）
    # 单帧最多评估多少个候选轮廓（按面积从大到小取）。MIN_AREA_RATIO 调小后
    # 暗背景下可能有成百上千个小轮廓，加个上限保证帧率不至于被拖垮。
    MAX_CANDIDATES = 60
    OPP_RATIO_MAX = 2.5              # 对边长度比上限（透视下给宽容区间）
    # 对边收敛自洽性容差（度）：投影矩形的对边夹角必须与"对边长度差"自洽
    # （见 geometry.opp_side_convergence）。实测合法姿态（含 θ=70° 强透视）的
    # 富余量 <=10.3°，而"小目标极值取错角"造出来的假四边形富余量 30.4°，
    # 取 18 正好分开。它挡的是"凸性/面积/长宽比全都合法、但中心偏 20px"的假四边形。
    OPP_ANGLE_SLACK = 18.0
    # 长宽比区间。A4 竖放 = 1.414；透视下会明显偏离，所以上下都要留足余量：
    # 之前写 (0.45, 1.6) 比结构分里的 SCORE_ASPECT_HI=2.45 还严，
    # 会让"画面里斜着、透视很强"的合法靶纸在最后一道几何校验被拦掉。
    ASPECT_MIN, ASPECT_MAX = 0.40, 2.10
    ROI = []                         # 可选搜索区域 [x, y, w, h]（全图坐标），空=全图
    REFINE_CORNERS = True            # 对 4 条边拟合直线求交 -> 亚像素角点
    REFINE_TRIM = 0.12               # 每条边去掉两端各 12% 的点（避开圆角污染）
    REFINE_D_RATIO = 0.05            # 参与拟合的点到边的最大垂直距离系数

    # =========================================================================
    # 四′ 性能预算（把"最坏一帧"的耗时有上界，别让单帧掉到秒级）
    # =========================================================================
    # 背景：真机实测帧率曾只有 0.1fps，而算法本身在 PC 上端到端只要 3.4ms
    # （≈298fps）。也就是说"慢"不是算法复杂度，而是**某些输入下复杂度爆炸**。
    # 拆开看是两处，本组参数就是给这两处设上界：
    #   ① 噪声/纹理二值图（尤其"精细形态档"lvl=1 不做开运算，噪点原样保留）
    #      一张掩膜能有 2 万条轮廓。旧实现对每条做 contourArea + boundingRect
    #      + minAreaRect（内部要算凸包）—— 单张掩膜实测 260ms（PC/x86），
    #      而**更贵的是 findContours 本身**：为 2 万条轮廓创建对象就要 190ms。
    #      现在分三步设上界（见 vision/detector.contour_features）：
    #        · boundary_estimate（0.3ms）先判断掩膜是不是"碎成渣"；
    #          是 -> 走连通域预筛（_survivor_contours），只对少数幸存块描边；
    #          不是 -> 正常描边，**零额外开销、行为与优化前完全一致**；
    #        · _top_contours 按轮廓点数只取前 MAX_SCAN_CONTOURS 条进 cv2。
    #      ⚠ 实测过另一条路（无条件用连通域统计预筛）**被否决**：它在每张掩膜上
    #        固定加 17~20ms，干净场景反而劣化十几倍。所以必须是"按需触发"。
    #   ② "缩小图没锁住 → 全分辨率升级"那一趟会无视锁定、在**全分辨率**上重扫
    #      12 阈值 × 2 形态档 = 24 个掩膜。一旦缩小图长期失败（远距离胶带在
    #      0.5 倍图上只有 1px，很容易失败），就变成每帧几秒。现在：
    #      锁定值排到最前 + 每帧掩膜预算 MAX_MASKS_FULL（超出的下一帧接着扫）。
    # 这三个都是"只加天花板、不减能力"的改动，精度不参与权衡。
    FRAG_BOUNDARY_EST = 13000    # "掩膜碎成渣"的判据（边界跳变数，分辨率无关）。
                                 # 实测（PC/x86，1280x720）：
                                 #   est 20~600   -> findContours 约 1~3ms（干净掩膜：一条胶带环）
                                 #   est ~9000    -> 约 17ms；~9700 -> 约 20ms
                                 #   est ~13700   -> 约 27ms；~17200 -> 约 58ms
                                 #   est ~72000   -> 约 190ms（白噪声+精细形态档：2.3 万条轮廓）
                                 # 13000 是两条路径的**实测交叉点**：低于它走
                                 # findContours 更便宜，高于它走连通域预筛更便宜。
                                 # （曾取 8000，实测在 est≈9000 处反而劣化 2 倍，
                                 #   因为连通域统计本身有约 11ms 的固定开销。）
    MAX_SCAN_COMPONENTS = 256    # 碎渣掩膜下，最多保留多少个连通域（按面积取前 N）
    MAX_SCAN_CONTOURS = 512      # 一张掩膜最多对多少条轮廓做 cv2 特征计算（按点数取前 N）
                                 # 512 对真机靶纸场景绰绰有余；调小=更快、极端杂乱更易漏
    MAX_MASKS_FULL = 8           # 全分辨率"升级"那一趟每帧最多过几个掩膜
                                 # 必试项（上一帧锁定值 / 固定阈值 / Otsu）不受此限制
    ROI_TRACK = True             # 帧间 ROI 跟踪：稳态优先用上一帧框扩一圈只解一小块
                                 # 跟丢/跳变超限自动落回全局搜索，只更快不更差
    ROI_TRACK_MARGIN = 0.25      # 跟踪裁剪区的余量（按框尺寸的比例）。太小会因目标
                                 # 移动而贴到裁剪边界被判"可能被裁断"、白白跟丢
    ROI_TRACK_MAX_JUMP = 0.6     # 跟踪结果相对上一帧的中心跳变上限（按框尺寸的比例）。
                                 # 超了就丢掉这一帧、回全局搜索 —— 宁可慢一点，绝不跟错目标
    PROFILE_STAGES = True        # 在 [fps] 日志里显示分段耗时与掩膜数（板上定位瓶颈用）

    # =========================================================================
    # 五、⑦ 时序滤波（One Euro）
    # =========================================================================
    FILTER_MODE = "static"           # "static"(要稳) 或 "dynamic"(要快)
    FILTER_PARAMS = {
        "static":  dict(min_cutoff=0.6, beta=0.004),
        "dynamic": dict(min_cutoff=2.5, beta=0.02),
    }

    # =========================================================================
    # 六、靶纸物理尺寸（cm）
    # =========================================================================
    A4_W_CM, A4_H_CM = 21.0, 29.7
    CENTER_CM = (A4_W_CM / 2.0, A4_H_CM / 2.0)   # 靶心 = (10.5, 14.85)
    # 胶带内沿（= 纸面可见区）的物理尺寸：用"亮面矩形"方案时按这个换算 px_per_cm
    A4_IN_W_CM = A4_W_CM - 2.0 * TAPE_W_CM       # 17.4
    A4_IN_H_CM = A4_H_CM - 2.0 * TAPE_W_CM       # 26.1

    # =========================================================================
    # 七、⑧ 显示 / 输出
    # =========================================================================
    SHOW = True                  # 是否显示（关闭可用于纯串口模块模式）
    DEBUG_SHAPE = True           # 启动首帧打印一次 image2cv 的 shape/format
                                 # （不同固件对灰度图的形状约定不同，留着便于定位问题）
    # ---- 锁框失败时的诊断（真机调参全靠它）----
    DEBUG_DIAG_EVERY = 2.0       # 失败时隔多少秒打印一次"灰度分布 + 候选结构"（只打印，很便宜）
    DEBUG_DUMP = True            # 失败时把原始帧/二值图落盘，便于离线分析
    DEBUG_DIR = "/root/target_dbg"   # 落盘目录（只覆盖同名文件，不会写满磁盘）
                                     # 置空字符串即可关闭；板子上可 `ls /root/target_dbg`
                                     # 再用 MaixVision 的文件管理/`scp` 取回电脑
    # ⚠ 落盘是**秒级**开销：板上把 3 张全分辨率图编成 PNG 写 flash，比重算一帧慢得多，
    #   而且会和主循环抢同一个 CPU。原来它和"打印诊断"共用 2 秒的节流、每次失败都写，
    #   一旦检测在失败/成功之间来回跳，帧率会被它压到很低。
    #   现在单独节流 + 限量 + 可降采样：前三张够定位问题了，之后不再写盘。
    DEBUG_DUMP_EVERY = 20.0      # 两次落盘之间的最小间隔（秒）
    DEBUG_DUMP_MAX = 3           # 本次运行最多落盘几次（<=0 表示不限）
    DEBUG_DUMP_SCALE = 1.0       # 落盘图缩放（1.0=原始分辨率；0.5=写盘快约 4 倍）
    DEBUG_DUMP_COMPRESS = 1      # PNG 压缩级别 0~9。1 = 明显更快、体积略大
    SHOW_W = 640                 # 显示画布宽度：先缩到这么宽再用 OpenCV 画标注
                                 # （画布只分配一次并复用，避免每帧新建图像）
    ENABLE_UART = False          # 需要给云台主控发坐标时置 True
    UART_PORT = "/dev/ttyS0"     # TODO 按实际接线改
    UART_BAUD = 115200
    PRINT_FPS = True
    PRINT_EVERY = 2.0            # 串口/终端打印检测结果的间隔（秒）

    # =========================================================================
    # 便捷查询
    # =========================================================================
    @classmethod
    def profile(cls, name=None):
        """
        返回 (name, width, height, fmt, buff_num, fps, work_scale)。
        name=None 或未命中时返回第一个（standard）。
        """
        want = (name or cls.PROFILE or "").strip().lower()
        for p in cls.PROFILES:
            if p[0].lower() == want:
                return p
        return cls.PROFILES[0]

    @classmethod
    def profile_order(cls, name=None):
        """返回档位尝试顺序：先配置档位，再按内存从省到多依次兜底。"""
        first = cls.profile(name)
        rest = sorted(cls.PROFILES, key=lambda p: -p[1])
        out = [first]
        for p in rest:
            if p[0] != first[0] and p[1] <= first[1]:
                out.append(p)
        return out
'''

_MAIN_SRC = r'''
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py · MaixCAM / MaixCAM2 应用入口 —— 靶纸矩形框中心点（靶心）识别
================================================================================
对应赛题：2025 全国大学生电子设计竞赛本科组 E 题《简易自行瞄准装置》

    靶面：A4 紫外感光纸，四周 1.8cm 黑胶带贴出矩形；红色记号笔在中心点靶心
          （≤0.1cm）—— 该点在 165cm 处只有约 0.67px，物理不可见。
    结论：靶心只能由"黑框矩形"几何反算，不能靠找红点。
    输出：靶心像素坐标 (u, v) + 局部尺度 px_per_cm，供云台主控解算 D1。

运行方式（MaixVision）
--------------------------------------------------------------------------------
    · 单文件调试：本文件必须叫 main.py 才能作为应用入口（MaixPy 应用规范）。
    · 项目方式：MaixVision「打开文件夹」选中本工程根目录，点「运行项目」，
      整个文件夹（main.py / config.py / vision/ / app.yaml ...）会一起下发运行。
    · 打包安装：MaixVision 左下角「安装」填写信息后「打包应用 / 安装应用」；
      或命令行 `maixtool release`（产物 dist/<id>_v<ver>.zip），
      再用 `app_store_cli install xxx.zip` 装到板子上，脱离电脑离线运行。

启动时请核对日志里的一行：
        [app] 靶心识别 <BUILD_ID> （根目录 ...）
    版本号与本地 main.py 顶部的 BUILD_ID 一致，才说明板子跑的是最新代码。

内存要点（"帧缓冲内存不足"报错的解法）
--------------------------------------------------------------------------------
    ① 相机直接输出 GRAYSCALE（1 B/px，RGB888 是 3 B/px）；
    ② buff_num 默认从 3 降到 2；
    ③ 算法在 WORK_SCALE（默认 0.5，即 1/4 像素量）的缩小图上做，失败才回退全分辨率；
    ④ 显示画布只分配一次并复用，不做逐帧新建；
    ⑤ 高分辨率 + MaixVision 图像预览同时开容易内存不足，必要时关掉预览。
    以 1280x720 为例：RGB888/buff3 = 8.3MB  ->  GRAYSCALE/buff2 = 1.8MB。
    仍不够时可改 /boot/configs 的 maix_memory_cmm（单位 MiB，MaixCAM2 默认 -1）。

图像形状要点（cv2.error: Bad number of channels ... 'scn' is 1 的解法）
--------------------------------------------------------------------------------
    实测 MaixCAM2 的 image.image2cv() 对 GRAYSCALE 返回的是 **(H, W, 1) 三维数组**
    （带一个哑轴），不是 (H, W)。所以绝不能按 `ndim == 2` 判"是不是灰度图"，
    否则会落到彩色分支、把单通道喂给 cv2.cvtColor(..., COLOR_BGR2GRAY) 而崩。
    本文件的 to_gray() 按**实际 shape** 判断，并兼容 (H,W)/(H,W,1)/(1,H,W)/(H,1,W)
    以及 3/4 通道；非常规打包格式（如 RGB565）交给 MaixPy 自己转灰度兜底。
"""

import os
import sys
import time


# -----------------------------------------------------------------------------
# 找到工程根目录，让"任何启动方式"都能 import 到 config / vision
# -----------------------------------------------------------------------------
# 三种文件落地位置：
#   · MaixVision「运行项目」  -> /tmp/maixpy_run/
#   · 安装成应用后离线运行    -> /root/apps/<app_id>/
#   · 手动 scp / 终端启动     -> 任意目录（甚至 cwd 与脚本不同）
# 优先用 __file__ 所在目录；个别固件里 __file__/__cwd__ 不可靠，再逐个兜底探测。
def _find_project_root():
    cands = []
    try:                                    # __file__ 可能未定义
        cands.append(os.path.dirname(os.path.abspath(__file__)))
    except NameError:
        pass
    for key in ("__cwd__", "__dir__"):      # MaixPy 部分版本提供的路径变量
        try:
            cands.append(os.path.abspath(str(globals()[key])))
        except Exception:
            pass
    try:
        cands.append(os.path.abspath(os.getcwd()))
    except Exception:
        pass
    cands += ["/tmp/maixpy_run", "/root/apps"]      # 设备上固定的两个位置
    for d in cands:
        try:
            if d and os.path.isfile(os.path.join(d, "config.py")):
                return d
        except Exception:
            continue
    return cands[0] if cands else "."


_ROOT = _find_project_root()
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)


def _missing_project_modules():
    """返回根目录下缺失的工程模块名（单文件版已把它们放进 sys.modules，故返回空）。"""
    miss = []
    for name, rel in (("config", "config.py"), ("vision", "vision")):
        if name in sys.modules:
            continue
        if not os.path.exists(os.path.join(_ROOT, rel)):
            miss.append(name)
    return miss


# 启动方式不对时，给出能直接照做的中文提示，而不是丢一个 ModuleNotFoundError
_miss = _missing_project_modules()
if _miss:
    raise SystemExit(
        "\n[致命] 设备上只有 main.py，缺少 %s —— 这次只下发了单个文件。\n"
        "  原因：点了 MaixVision 左下角的「运行当前文件」——它只发当前这一个文件，\n"
        "        不发送工程里的其它 .py（官方文档第七节明确说明）。\n"
        "  解决（任选其一）：\n"
        "    A. 点「打开文件夹」选中工程根目录 25Efork，再点左下角「运行项目」；\n"
        "    B. 改用单文件版 dist/main_single.py（所有模块已合成一个文件），\n"
        "       打开它点「运行当前文件」即可；源码改动后在电脑上跑\n"
        "       python tools/build_single.py 重新生成。\n"
        % ", ".join(_miss)
    )

import cv2                                             # noqa: E402
import numpy as np                                     # noqa: E402

from maix import app, camera, display, image           # noqa: E402

from config import Config                              # noqa: E402
from vision.detector import TargetCenterDetector       # noqa: E402
from vision.overlay import Overlay                     # noqa: E402

# 构建标识 —— 启动时打印，用来一眼确认板子跑的是不是最新代码。
# （本项目出现过"本地 main.py 被旧内容覆盖，设备仍在跑旧逻辑"的情况，
#   日志里对不上这个版本号就说明下发的是旧文件。）
BUILD_ID = "2026-10-05-r7"

# 档位里的格式名 -> MaixPy 枚举
_FMT = {
    "GRAYSCALE": image.Format.FMT_GRAYSCALE,
    "RGB565": image.Format.FMT_RGB565,
    "RGB888": image.Format.FMT_RGB888,
    "BGR888": image.Format.FMT_BGR888,
}


# =============================================================================
# 辅助
# =============================================================================
def log_memory(tag=""):
    """打印板端内存占用，便于判断"帧缓冲内存不足"到底卡在哪。"""
    try:
        from maix import sys as m_sys
        print("[mem]%s %s" % (tag, m_sys.memory_info()))
    except Exception as e:                             # 老版本没有该接口
        print("[mem]%s 查询不可用: %s" % (tag, e))


def open_camera(cfg):
    """按"配置档位 -> 更省内存的档位"顺序尝试打开相机，成功即返回。"""
    for name, w, h, fmt, buff, fps, work in Config.profile_order(cfg.PROFILE):
        try:
            cam = camera.Camera(w, h, _FMT.get(fmt, image.Format.FMT_GRAYSCALE),
                                fps=fps, buff_num=buff)
            print("[cam] 档位 %s : %dx%d %s buff_num=%d fps=%d work_scale=%.2f"
                  % (name, w, h, fmt, buff, fps, work))
            return cam, dict(name=name, w=w, h=h, fmt=fmt, buff=buff,
                             fps=fps, work=work)
        except Exception as e:
            print("[cam] 档位 %s(%dx%d %s buff=%d) 打开失败，尝试更省内存的档位: %s"
                  % (name, w, h, fmt, buff, e))
    raise RuntimeError("所有分辨率档位都无法打开相机；请检查摄像头排线与供电。")


def _as_gray(a, bgr=False):
    """
    numpy 数组 -> 单通道灰度（能零拷贝就零拷贝）；判不出单通道则返回 None。

    **按实际 shape 判断，不按"当初请求了什么格式"猜。** 实测 MaixCAM2 的
    image2cv() 对 GRAYSCALE 返回 (H, W, 1)，而不是 (H, W)；若用 `ndim == 2`
    判灰度就会漏判 → 落到彩色分支 → 把单通道喂给 COLOR_BGR2GRAY，于是抛

        cv2.error: (-15:Bad number of channels) in function 'CvtHelper'
        > 'VScn::contains(scn)' where 'scn' is 1

    这是本项目在板上踩过的坑，预检脚本里有一项"灰度形状矩阵"专门守着它。
    """
    if a is None:
        return None
    if a.ndim == 3:
        c = a.shape[2]
        if c == 1:                                  # (H,W,1) 哑轴 —— 取视图，零拷贝
            return a[:, :, 0]
        if c == 3:                                  # RGB888 / BGR888
            return cv2.cvtColor(a, cv2.COLOR_BGR2GRAY if bgr else cv2.COLOR_RGB2GRAY)
        if c == 4:                                  # RGBA8888 / BGRA8888
            return cv2.cvtColor(a, cv2.COLOR_BGRA2GRAY if bgr else cv2.COLOR_RGBA2GRAY)
    g = np.squeeze(a)                               # (1,H,W) / (H,1,W) 等哑轴
    return g if g.ndim == 2 else None


def to_gray(img, debug=False):
    """
    maix.image.Image -> 灰度 numpy 视图（本帧内用完即弃，勿跨帧持有）。

    优先零拷贝直出（灰度相机）；万一相机给了非常规打包格式（如 RGB565），
    交由 MaixPy 自己转灰度 —— 保证任何情况下都不会把单通道喂给 cvtColor。
    """
    a = image.image2cv(img, ensure_bgr=False, copy=False)
    if debug:
        try:
            print("[gray] format=%s shape=%s dtype=%s ndim=%d"
                  % (img.format(), tuple(a.shape), a.dtype, a.ndim))
        except Exception as e:
            print("[gray] 形状打印失败: %s" % e)
    g = _as_gray(a)
    if g is not None:
        return g
    # 罕见打包格式（如 RGB565）的兜底：让 MaixPy 做一次 BGR 归一化再处理。
    # 关键：真机上 ensure_bgr 对**灰度图是空操作**（灰度没有 BGR 形式，仍返回
    # 单通道），所以这里必须再过一次 _as_gray 按实际通道数判断，
    # 绝不能直接 cvtColor(..., COLOR_BGR2GRAY) —— 那正是板上崩掉的那一行。
    b = image.image2cv(img, ensure_bgr=True, copy=True)
    g = _as_gray(b, bgr=True)
    if g is None:
        raise RuntimeError("[gray] 无法把 shape=%s 的帧转成灰度"
                           % (tuple(getattr(b, "shape", ())),))
    return g


def pack_result(res):
    """⑧ 串口帧：AA 55 | valid(B) u(H) v(H) scale*100(H) | XOR 校验（小端）"""
    if res.get("valid"):
        u = int(max(-32768, min(32767, res["u"])))
        v = int(max(-32768, min(32767, res["v"])))
        s = int(max(0, min(65535, res["scale"] * 100)))
        body = (bytes([0xAA, 0x55, 1])
                + (u & 0xFFFF).to_bytes(2, "little")
                + (v & 0xFFFF).to_bytes(2, "little")
                + s.to_bytes(2, "little"))
    else:
        body = bytes([0xAA, 0x55, 0]) + bytes(6)
    chk = 0
    for b in body:
        chk ^= b
    return body + bytes([chk])


# =============================================================================
# 锁框失败时的诊断（真机调参全靠这个 —— 不然只能靠猜）
# =============================================================================
def _write_png(path, arr):
    # 压缩级别默认 1（不是 OpenCV 默认的 3）：板上写盘是"秒级"操作，
    # 低压缩级别显著更快、体积只大一点，对"看一眼二值图对不对"完全够用。
    lvl = max(0, min(9, int(getattr(Config, "DEBUG_DUMP_COMPRESS", 1))))
    ok, buf = cv2.imencode(".png", np.ascontiguousarray(arr),
                           [int(cv2.IMWRITE_PNG_COMPRESSION), lvl])
    if ok:
        buf.tofile(path)
        return True
    return False


def _maybe_small(arr, cfg):
    """按 DEBUG_DUMP_SCALE 降采样（1.0 = 原样）。写盘耗时随像素数平方级下降。"""
    s = float(getattr(cfg, "DEBUG_DUMP_SCALE", 1.0) or 1.0)
    if not (0.0 < s < 0.999):
        return arr
    h, w = arr.shape[:2]
    return cv2.resize(arr, (max(8, int(round(w * s))), max(8, int(round(h * s)))),
                      interpolation=cv2.INTER_AREA)


def debug_dump(gray, cfg, lock_th, lvl=0):
    """
    把"这一帧到底长什么样"落盘，便于离线分析（把这三个文件发出来即可定位问题）：
        frame_gray.png  相机原始灰度帧（未做任何处理）
        mask.png        当前阈值下的**原始**二值图（白=被判为"深色"的像素）
        mask_morph.png  过完形态学之后的二值图（描边实际看到的就是它）
    只覆盖同名文件，不会把设备磁盘写满。

    mask.png 与 mask_morph.png 的差别很关键：如果 mask.png 里胶带环是完整的、
    但 mask_morph.png 里环断了/消失了，那就是**形态学核太大把细环腐蚀掉了**
    （远距离的典型症状），该调 MORPH_OPEN_K / MORPH_CLOSE_K，而不是调阈值。

    ⚠ 本函数是**秒级**开销（板上编 3 张 PNG + 写 flash），调用方必须限流：
       main.py 用 DEBUG_DUMP_EVERY（间隔）+ DEBUG_DUMP_MAX（总量）双重限制。
       早期版本和"打印诊断"共用 2 秒节流、每次失败都写，在"检测在成败之间
       来回跳"的场景里会把帧率压到很低 —— 这是 0.1fps 的帮凶之一。
    """
    if not (cfg.DEBUG_DUMP and cfg.DEBUG_DIR):
        return
    try:
        os.makedirs(cfg.DEBUG_DIR, exist_ok=True)
        _write_png(os.path.join(cfg.DEBUG_DIR, "frame_gray.png"),
                   _maybe_small(gray, cfg))
        th = lock_th if lock_th is not None else cfg.BLACK_THRESH
        th = int(th)
        if th > 0:
            _, bw = cv2.threshold(gray, th, 255, cv2.THRESH_BINARY_INV)
            _write_png(os.path.join(cfg.DEBUG_DIR, "mask.png"), _maybe_small(bw, cfg))
            ok = int(getattr(cfg, "MORPH_OPEN_K_FINE", 1) if lvl else cfg.MORPH_OPEN_K)
            ck = int(getattr(cfg, "MORPH_CLOSE_K_FINE", 3) if lvl else cfg.MORPH_CLOSE_K)
            mw = bw
            if ok >= 2:
                mw = cv2.morphologyEx(
                    mw, cv2.MORPH_OPEN,
                    cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ok,) * 2))
            if ck >= 2:
                mw = cv2.morphologyEx(
                    mw, cv2.MORPH_CLOSE,
                    cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ck,) * 2))
            _write_png(os.path.join(cfg.DEBUG_DIR, "mask_morph.png"),
                       _maybe_small(mw, cfg))
        print("[dump] 已保存 %s/{frame_gray,mask,mask_morph}.png（阈值=%d 形态档=%d 缩放=%.2f）"
              % (cfg.DEBUG_DIR, th, int(lvl), float(getattr(cfg, "DEBUG_DUMP_SCALE", 1.0))))
    except Exception as e:
        print("[dump] 保存失败(不影响运行): %s" % e)


def diag_line(det, gray, cfg):
    """
    打印"为什么没锁住框"的诊断。看这三件事基本就能定位：

      ① 灰度分布 p1/p5/p50/p95/p99 —— 判断是不是【曝光】问题
         例：p1=120 说明连最暗的胶带都有 120 灰，固定阈值 95 根本切不出黑，必须提高阈值
             或重新标定曝光；p99=60 说明整幅都黑，是欠曝。
      ② 黑占比（该阈值下被判为"深色"的像素比例）与锁定阈值
         0.00 → 框完全没进二值图；0.90+ → 整幅都黑了（背景也被算进来）
      ③ 每个 (阈值, 形态学档位) 上的候选结构（kind/分数/各项特征）—— 判断是不是【形状】问题
         有候选但分数低 → 看哪一项不达标；一个候选都没有 → 连轮廓都没形成

    ⚠ 这里**只能用 e.get(k) or 0.0 兜底**：早先直接 "%.2f" % e.get("fill") 时，
      只要某条诊断记录缺字段就会抛 `must be real number, not NoneType`，
      整行诊断全丢 —— 真机上就只剩"锁不住"，等于完全瞎调。现在记录由
      detector._dbg_rec() 统一构造（字段齐全），这里再加一层兜底，双保险。
    """
    def _n(rec, k, d=0.0):
        v = rec.get(k)
        return d if v is None else v

    try:
        p = np.percentile(gray, [1, 5, 50, 95, 99]).round(0).astype(int)
        th = det._th_lock
        thr = int(th) if th else int(cfg.BLACK_THRESH)
        _, bw = cv2.threshold(gray, thr, 255, cv2.THRESH_BINARY_INV)
        print("[diag] 灰度 p1/p5/p50/p95/p99=%d/%d/%d/%d/%d  锁定阈值=%s(形态档%d)  "
              "黑占比=%.3f  本帧来自=%s"
              % (p[0], p[1], p[2], p[3], p[4], th,
                 int(getattr(det, "_lvl_lock", 0)),
                 cv2.countNonZero(bw) / float(bw.size),
                 getattr(det, "_dbg_pass", "?")))
        rows = det.last_debug or []
        if not rows:
            print("[diag]   （本帧没生成任何候选记录 —— 说明连二值图都没跑出来）")
        for e in rows:
            print("[diag]   阈值%-5s 形态%-2s 候选%-3s 最优[%s 分=%.2f fill=%.2f rim=%.2f "
                  "ring=%.2f 长宽比=%.2f 对比=%.0f 几何合法=%s]%s"
                  % (e.get("th"), e.get("lvl"), e.get("n"), e.get("kind"),
                     _n(e, "score"), _n(e, "fill"), _n(e, "rim"), _n(e, "ring"),
                     _n(e, "aspect"), _n(e, "contrast"), e.get("ok"),
                     ("  " + e["skip"]) if e.get("skip") else ""))
    except Exception as e:
        print("[diag] 诊断输出失败(不影响运行): %s" % e)


# =============================================================================
# 主流程
# =============================================================================
def main():
    cfg = Config
    print("[app] 靶心识别 %s  （根目录 %s）" % (BUILD_ID, _ROOT))
    log_memory(" 启动前 ")

    cam, prof = open_camera(cfg)
    Config.WORK_SCALE = prof["work"]        # 交给 detector 使用
    cam.skip_frames(cfg.SKIP_FRAMES)

    if cfg.LOCK_AE_AWB:                     # 锁定曝光/白平衡：固定阈值稳定的前提
        for fn, arg in (("exp_mode", camera.AeMode.Manual),
                        ("exposure", cfg.EXPOSURE),
                        ("gain", cfg.GAIN)):
            try:
                getattr(cam, fn)(arg)
            except Exception as e:
                print("[cam] %s 设置失败(可忽略): %s" % (fn, e))
        try:
            cam.awb_mode(camera.AwbMode.Manual)
            cam.set_wb_gain(cfg.WB_GAIN)
        except Exception as e:
            print("[cam] 白平衡锁定失败(可忽略): %s" % e)
    for fn, flag in (("hmirror", cfg.H_MIRROR), ("vflip", cfg.V_FLIP)):
        try:
            getattr(cam, fn)(bool(flag))
        except Exception as e:
            print("[cam] %s 设置失败(可忽略): %s" % (fn, e))

    disp = display.Display() if cfg.SHOW else None
    det = TargetCenterDetector(cfg)
    ov = Overlay(cfg)

    uart = None
    if cfg.ENABLE_UART:
        try:
            from maix import uart as _uart
            uart = _uart.UART(cfg.UART_PORT, cfg.UART_BAUD)
        except Exception as e:
            print("[uart] 打开失败: %s" % e)

    log_memory(" 初始化后 ")
    print("[app] 进入主循环；按设备功能键退出。")

    fps, t_last, t_print = 0.0, time.time(), time.time()
    t_diag = 0.0
    t_dump = 0.0                    # 落盘单独计时（落盘是秒级开销，不能跟打印共用节流）
    n_dump = 0
    ms_det = 0.0                    # 算法耗时的指数滑动平均（板上定位瓶颈用）
    first_frame = True

    while not app.need_exit():
        t_now = time.time()
        img = cam.read()
        if img is None:
            continue

        if cfg.LENS_CORR:
            try:
                # 不同版本可能原地改或返回新对象，两种都兼容
                _c = img.lens_corr(strength=cfg.LENS_STRENGTH)
                if _c is not None:
                    img = _c
            except Exception as e:
                print("[cam] lens_corr 失败(可忽略): %s" % e)

        # 首帧打印一次真实形状，换板子/固件时用来确认走对了分支
        gray = to_gray(img, debug=first_frame and cfg.DEBUG_SHAPE)
        first_frame = False
        _t_det = time.time()
        res = det.process(gray, timestamp=t_now)  # ①~⑦
        if cfg.PROFILE_STAGES:
            ms_det = 0.9 * ms_det + 0.1 * (time.time() - _t_det) * 1000.0

        # 锁框失败：打印可定位问题的诊断（便宜），并把原始帧落盘（贵，单独限流+限量）
        if not res["valid"] and t_now - t_diag > cfg.DEBUG_DIAG_EVERY:
            t_diag = t_now
            diag_line(det, gray, cfg)
            dump_max = int(getattr(cfg, "DEBUG_DUMP_MAX", 3) or 0)
            if (t_now - t_dump > float(getattr(cfg, "DEBUG_DUMP_EVERY", 20.0))
                    and (dump_max <= 0 or n_dump < dump_max)):
                t_dump = t_now
                n_dump += 1
                debug_dump(gray, cfg, det._th_lock, getattr(det, "_lvl_lock", 0))

        if disp is not None and cfg.SHOW:
            frame = ov.render(gray, res, fps=fps)
            # copy=True：把画布拷进 maix Image，避免与显示的异步读取抢同一块内存
            disp.show(image.cv2image(frame, bgr=False, copy=True))

        if uart is not None:
            try:
                uart.write(pack_result(res))
            except Exception as e:
                print("[uart] 发送失败: %s" % e)

        dt = t_now - t_last
        t_last = t_now
        if dt > 1e-4:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)
        if cfg.PRINT_FPS and t_now - t_print > cfg.PRINT_EVERY:
            t_print = t_now
            cand = res.get("cand") or {}
            # 分段耗时 / 掩膜数：板上据此直接看出"瓶颈消没消"。
            # 掩膜数是最关键的一项 —— 稳态应该是个位数（锁定后是 1）；
            # 如果它长期等于 12/24，说明帧帧都在全扫，那才是 0.1fps 的真凶。
            extra = ""
            if cfg.PROFILE_STAGES:
                seg = res.get("ms") or {}
                extra = "  算法=%.1fms 掩膜=%s  [%s]" % (
                    ms_det, res.get("masks", "-"),
                    " ".join("%s=%.1f" % (k, v) for k, v in seg.items()) or "-")
            print("[fps] %.1f  valid=%s  u=%s v=%s  %.2f px/cm  阈值=%s/形态%s 候选=%s  %s%s"
                  % (fps, res["valid"],
                     "%.1f" % res["u"] if res["valid"] else "-",
                     "%.1f" % res["v"] if res["valid"] else "-",
                     res["scale"], res.get("th"), res.get("lvl", 0),
                     cand.get("kind", "-"), res["stage"], extra))

    try:
        cam.close()
    except Exception:
        pass
    print("[app] 已退出。")


if __name__ == "__main__":
    main()
'''

_MOD_ORDER = ('vision.geometry', 'vision.detector', 'vision.overlay', 'config')

_pkg = _types.ModuleType("vision")          # 造出虚拟包 vision
_pkg.__path__ = []
_sys.modules["vision"] = _pkg
for _full in _MOD_ORDER:
    if "." in _full:                       # vision.geometry 这类子模块
        _parent, _leaf = _full.split(".", 1)
        _m = _types.ModuleType(_full)
        setattr(_sys.modules[_parent], _leaf, _m)
        _sys.modules[_full] = _m
    else:                                  # config 这类顶层模块
        _sys.modules[_full] = _types.ModuleType(_full)
for _full in _MOD_ORDER:
    _m = _sys.modules[_full]
    _m.__dict__["__name__"] = _full
    exec(compile(_MOD_SRC[_full], _full.replace(".", "/") + ".py", "exec"),
         _m.__dict__)
del _m, _full, _MOD_SRC

# --------------------------------------------------------------------------
# ② 主程序：main.py 原文照搬，exec 进本文件全局命名空间
#    这样 __name__ == "__main__" 判断、traceback 文件名都与原来一致
# --------------------------------------------------------------------------
exec(compile(_MAIN_SRC, 'main.py', "exec"), globals())
