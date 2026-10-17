"""Pure projective geometry; all homographies map physical mm to image pixels."""
import cv2
import numpy as np


def polygon_area(p):
    """带符号面积的 2 倍（>0 = 逆时针 CCW，<0 = 顺时针 CW）。用于把四角规整成统一方向。"""
    p = np.asarray(p)
    return float(p[0,0]*(p[1,1]-p[3,1]) + p[1,0]*(p[2,1]-p[0,1]) +
                 p[2,0]*(p[3,1]-p[1,1]) + p[3,0]*(p[0,1]-p[2,1]))


def order_quad(points):
    """把 4 个点规整为统一顺序：以质心为极点按极角排序 → 保证 CCW → 旋转到"左上角"开头。

    统一顺序是后续所有几何运算（边向量、对角线、单应四点对应）的前提。
    返回顺序语义：**[左上, 右上, 右下, 左下]**（逆时针），与 side_lengths 的边 0/1/2/3 对应。
    """
    p = np.asarray(points, np.float32).reshape(4, 2)
    c = p.mean(axis=0)
    p = p[np.argsort(np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0]))]   # 按极角排序
    if polygon_area(p) < 0:
        p = p[::-1]                                                  # 反了则翻转成 CCW
    # np.roll 到"x+y 最小"的点（即左上角）打头。
    i = int(np.argmin(p.sum(axis=1)))
    return np.concatenate((p[i:], p[:i]))


def side_lengths(q):
    """四边长度 [边0, 边1, 边2, 边3]，边 i = 从点 i 指向点 i+1（roll -1 取下一个点）。"""
    return np.linalg.norm(np.roll(q, -1, axis=0) - q, axis=1)


def gaussian_solve(a, b):
    """带部分主元的高斯消元；消去行向量化，避免 64 次 Python 行循环。

    a 为 n×n、b 为 n，返回解向量。主元过小（矩阵退化，如四点共线）抛 ValueError，
    由调用方捕获后判该候选无效 —— 不做伪解。
    """
    m = np.column_stack((np.asarray(a, np.float64), np.asarray(b, np.float64)))
    for col in range(len(m)):
        pivot = col + int(np.argmax(np.abs(m[col:, col])))    # 选列内绝对值最大的行做主元
        if abs(m[pivot, col]) < 1e-10:
            raise ValueError("degenerate quadrilateral")
        if pivot != col:
            m[[col, pivot]] = m[[pivot, col]]                 # 换行
        m[col] /= m[col, col]                                 # 归一化主元行
        factors = m[:, col].copy()
        factors[col] = 0.0
        m -= factors[:, None] * m[col]                        # 一次消去整列（行向量化）
    return m[:, -1]


def homography(src, dst):
    """解 4 点对应的 3×3 单应 H（使 H·src ≡ dst）。

    做法：先对 src/dst **归一化**（平移质心到原点、缩放到量级 1）再建 8×8 线性方程，
    求得 H_n 后用 T_dst·H_n·T_src⁻¹ 还原 —— 归一化能显著提升大像素坐标下的数值稳定性。
    退化情形（任一边集合尺度过小/共线）抛 ValueError。
    """
    src, dst = np.asarray(src, np.float64), np.asarray(dst, np.float64)
    sc, dc = src.mean(axis=0), dst.mean(axis=0)
    ss, ds = float(np.max(np.abs(src - sc))), float(np.max(np.abs(dst - dc)))
    if min(ss, ds) < 1e-8:
        raise ValueError("zero-size rectangle")
    xy, uv = (src - sc) / ss, (dst - dc) / ds                 # 归一化坐标
    a = np.zeros((8, 8), np.float64)
    a[0::2, :3] = np.column_stack((xy, np.ones(4)))           # 偶数行：x 分量方程
    a[1::2, 3:6] = a[0::2, :3]                                # 奇数行：y 分量方程
    a[0::2, 6:] = -uv[:, :1] * xy                             # 第 3 列（透视项）
    a[1::2, 6:] = -uv[:, 1:] * xy
    v = gaussian_solve(a, uv.reshape(-1))
    hn = np.append(v, 1.0).reshape(3, 3)                      # 归一化域下的 H
    ts = np.array([[1 / ss, 0, -sc[0] / ss], [0, 1 / ss, -sc[1] / ss], [0, 0, 1]])
    td_inv = np.array([[ds, 0, dc[0]], [0, ds, dc[1]], [0, 0, 1]])
    h = td_inv @ hn @ ts                                      # 还原到原坐标域
    return h / h[2, 2] if abs(h[2, 2]) > 1e-12 else h


def homography_8x8(src_quad, width_mm, height_mm):
    """把物理矩形 (0,0)-(w,h) 映射到图像四边形 src_quad 的单应（mm → px）。"""
    return homography(rectangle_corners(width_mm, height_mm), src_quad)


def project_points(matrix, points):
    """齐次投影：把 (N,2) 点经 3×3 矩阵映射，做透视除法得 (N,2)。落无穷远点抛错。"""
    p = np.asarray(points, np.float64).reshape(-1, 2)
    z = np.column_stack((p, np.ones(len(p)))) @ np.asarray(matrix).T
    if np.any(np.abs(z[:, 2]) < 1e-9):
        raise ValueError("point at projective infinity")
    return z[:, :2] / z[:, 2:]


def inverse_map_points(h, points):
    """单应逆映射（px → mm）；不做透视除法前的越界检查，由调用方负责。"""
    return project_points(np.linalg.inv(h), points)


def standard_circle(width_mm, height_mm, radius_mm, samples=72):
    """正视面坐标下、以矩形中心为圆心、半径 radius_mm 的等分圆采样点（(samples,2)）。

    用途：把该圆逆映射回原图，用于可视化/交叉校验中心；**圆心本身直接取矩形中心投影**，
    不用圆周平均（见 process 内注释）。
    """
    a = np.arange(samples) * (2 * np.pi / samples)
    return np.array([width_mm / 2, height_mm / 2]) + radius_mm * np.column_stack((np.cos(a), np.sin(a)))


def rectangle_corners(w, h, inset=0.0):
    """物理矩形四角（mm），inset>0 时向内缩进（用于取胶带内沿 = 内框）。顺序 LL→LR→UR→UL。"""
    return np.array([[inset, inset], [w - inset, inset],
                     [w - inset, h - inset], [inset, h - inset]], np.float64)


def sample_batch(gray, points):
    """一次 OpenCV remap 双线性采样整批点，越界由调用方拒绝。

    相比逐点 Python 循环，remap 一次调用即可采样成百上千个点（四边验证/精修的关键提速）。
    borderMode=REPLICATE 保证边缘点也能返回数值（越界判据在调用方单独做）。
    """
    shape = points.shape[:-1]
    p = points.reshape(-1, 2).astype(np.float32)
    values = cv2.remap(gray, p[:, 0].reshape(1, -1), p[:, 1].reshape(1, -1),
                       cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    return values.reshape(shape)


def quad_is_valid(q, shape, margin=1, min_edge=12., min_sine=.20):
    q = np.asarray(q, np.float32)
    if q.shape != (4, 2) or not np.isfinite(q).all():
        return False
    h, w = shape[:2]
    if q[:,0].min() < margin or q[:,1].min() < margin or q[:,0].max() >= w-margin or q[:,1].max() >= h-margin:
        return False
    if not cv2.isContourConvex(q.reshape(-1, 1, 2)):
        return False
    e = np.concatenate((q[1:], q[:1])) - q
    lengths = np.linalg.norm(e, axis=1)
    if lengths.min() < min_edge:
        return False
    nxt = np.concatenate((e[1:], e[:1]))
    sine = np.abs(e[:,0]*nxt[:,1]-e[:,1]*nxt[:,0]) / (lengths*np.r_[lengths[1:], lengths[:1]])
    return bool(sine.min() >= min_sine)


def quad_from_contour(cnt, eps_ratio, hull_fill_ratio, factors=(1., 1.6, 2.4, 3.5, 5.)):
    def fit(contour):
        perimeter = cv2.arcLength(contour, True)
        for factor in factors:
            ap = cv2.approxPolyDP(contour, eps_ratio*factor*perimeter, True)
            if len(ap) == 4 and cv2.isContourConvex(ap):
                return order_quad(ap)
        return None
    q = fit(cnt)
    if q is not None:
        return q
    hull = cv2.convexHull(cnt)
    # A thin broken ring can be repaired; all hull proposals still need dark-band evidence.
    if abs(cv2.contourArea(cnt)) < hull_fill_ratio*abs(cv2.contourArea(hull)):
        return fit(hull)
    return None


def front_view_size(w, h, short_px):
    scale = float(short_px) / min(w, h)
    return max(2, round(w*scale)), max(2, round(h*scale))


def warp_front_view(gray, rect_to_image, w, h, short_px):
    fw, fh = front_view_size(w, h, short_px)
    mm_to_front = np.diag([(fw-1)/w, (fh-1)/h, 1.])
    # OpenCV takes src -> dst; reversing this matrix silently warps background.
    return cv2.warpPerspective(gray, mm_to_front @ np.linalg.inv(rect_to_image), (fw, fh))
