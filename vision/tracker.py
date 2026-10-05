#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tracker.py · 抗抖跟踪器：常数速度卡尔曼 + 新息门限 + 尺度/清晰度一致性
================================================================================
为什么需要它（合成复现 + 实测数据，见 tools/test_jitter.py）
--------------------------------------------------------------------------------
真机症状：帧率正常、也检得出，但**小车一动/一晃，锁框中心就明显偏移**。

用 `tools/test_jitter.py`（合成靶纸 + 模拟车身晃动 + 定向运动模糊，真值已知）复现后，
结论和"检测算错了"的直觉相反：

    原始解算中心（未滤波）   抖动下中心误差   均值 0.10 px   最大  0.39 px   ← 几乎完美
    One Euro(static) 输出    抖动下中心误差   均值 8.06 px   最大 27.26 px   ← 偏移在这里

**偏移不是检测算错了，是滤波器跟不动。** 抖动让靶心的像在画面里以 1.7~5 Hz 晃动
（幅度几十 px），这是**必须被跟踪的真实运动** —— 云台要靠它瞄准，把它当噪声滤掉
就等于"照着几帧前的位置打"。而 `static` 档 One Euro 的 `min_cutoff=0.6 / beta=0.004`
对这种晃动严重滞后，于是输出被拖出几十像素的**系统性偏移**。

所以"一刀切低通"在这里是错的。正确做法是把两件事**解耦**：

    ① 跟运动 —— 状态用 (位置, 速度)，靠速度项外推。
       这就是常数速度卡尔曼比低通强的地方：**同样的抑噪，滞后小得多**
       （低通是"用滞后换平滑"，卡尔曼是"用模型外推 + 残差修正"）。
       **过程噪声 q 必须自适应**：实测车身晃动要 q≈1e7 px²/s³ 才跟得住，
       而静止靶纸用 q≈1e3 才平滑得干净 —— 固定值不可能两头都满足。
       所以 q 由**新息（innovtion）**在线估计：新息大 = 目标此刻机动强 = 放宽 q；
       新息小 = 目标基本静止 = 收紧 q。全自动，不需要现场标参数。
    ② 防误锁 —— 用**新息门限**（normalized innovation）把离群测量挡在外面：
       测量与预测差得太远（按目标自身尺度衡量）就判为"误锁/被遮挡/糊了"，
       这一帧不采信、继续用预测值输出；同时慢慢**放大不确定度**，
       等偏差持续存在（目标真的动了）时自动重新采信 —— 既不会跳，也不会卡死。

另外三条一致性约束（都来自真机日志的真实现象）：
    ③ 尺度一致性 —— 误锁几乎总伴随框尺寸突变（真机日志里 px/cm 在 22↔12.5 之间跳）。
       框尺寸跳变超限时，整条测量（含中心）一并判为可疑。
    ④ 候选族迟滞 —— 真机日志里 `候选=ring` 与 `候选=paper` 来回切，两者中心/尺度
       并不严格相同。族切换要有代价（见 config.KIND_BONUS），不能逐帧随便换。
    ⑤ 清晰度自适应 —— 运动模糊越重，测量越不可信：按模糊量放大测量噪声 r，
       让滤波器在糊帧上更依赖预测值。

设计原则：**只加"何时不采信"的门限，不改检测本身的判定**；门限全部用
"相对目标自身尺度"的量，所以对远近、大小都自适应，不用按距离重标。
"""

import math

import numpy as np

from . import geometry as G


def quad_diag(quad):
    """四边形外接框的最长边（px）—— 作为"目标在画面里有多大"的尺度参考。"""
    if quad is None:
        return 0.0
    q = np.asarray(quad, dtype=np.float64).reshape(-1, 2)
    if q.shape[0] < 2 or not np.isfinite(q).all():
        return 0.0
    return float(max(np.ptp(q[:, 0]), np.ptp(q[:, 1])))


class _CV1D:
    """
    单轴常数速度卡尔曼：(位置, 速度)。状态协方差手写展开，避免 4x4 矩阵运算开销。

    连续白噪声加速度模型：
        F = [[1, dt], [0, 1]]
        Q = q * [[dt^3/3, dt^2/2], [dt^2/2, dt]]

    q 是"加速度谱密度"（px²/s³）：**越大越跟手、越小越平滑**。
    它是本模块唯一的"跟手程度"旋钮，**不要拿低通的截止频率去类比**
    （低通调高截止只是"少滤一点"，卡尔曼调大 q 是"允许更大的加速度"）。
    """

    __slots__ = ("ok", "p", "v", "P00", "P01", "P11")

    def __init__(self):
        self.reset()

    def reset(self):
        self.ok = False
        self.p = self.v = 0.0
        self.P00 = self.P01 = self.P11 = 0.0

    def seed(self, z, p_var=1.0e4, v_var=1.0e6):
        self.ok = True
        self.p = float(z)
        self.v = 0.0
        self.P00 = float(p_var)
        self.P01 = 0.0
        self.P11 = float(v_var)

    def predict(self, dt, q):
        if not self.ok:
            return 0.0
        self.p += self.v * dt
        P00, P01, P11 = self.P00, self.P01, self.P11
        self.P00 = P00 + dt * (2.0 * P01 + dt * P11) + q * dt ** 3 / 3.0
        self.P01 = P01 + dt * P11 + q * dt ** 2 / 2.0
        self.P11 = P11 + q * dt
        return self.p

    def update(self, z, r, dv_max=None):
        """
        标准卡尔曼更新；返回新息（测量 − 预测）。

        dv_max：**单帧速度修正量的上限**（px/s）。这是防"一个坏测量把速度带飞"的关键：
            K1 = P01/S 在这种"高过程噪声"工况下可以很大（实测 K1≈30），
            于是 v += 30*y —— 一个 60px 的离群点能把速度一次改掉 1800px/s，
            下一帧预测就会飞出去几十上百像素（实测 208px），再触发拒信级联。
            按"由近期运动推断出的可能加速度"给个上限，坏点就只能造成有界的偏差。
        """
        if not self.ok:
            self.seed(z)
            return 0.0
        S = self.P00 + float(r)
        if S <= 1e-12:
            return 0.0
        K0 = self.P00 / S
        K1 = self.P01 / S
        y = float(z) - self.p
        if dv_max is not None and dv_max > 0.0 and abs(y) > 1e-9:
            k1_lim = float(dv_max) / abs(y)
            if abs(K1) > k1_lim:
                K1 = math.copysign(k1_lim, K1)
        self.p += K0 * y
        self.v += K1 * y
        P00, P01, P11 = self.P00, self.P01, self.P11
        self.P00 = max(1e-9, (1.0 - K0) * P00)
        self.P01 = (1.0 - K0) * P01
        self.P11 = max(1e-9, P11 - K1 * P01)
        return y

    def inflate(self, gain):
        """拒信之后放大不确定度：让"目标真动了"这件事能自动被重新接受。"""
        self.P00 *= gain
        self.P01 *= gain
        self.P11 *= gain


class TargetTracker:
    """
    把"每帧独立解算出的 (u, v, scale, quad)"变成"抗抖、不跳变的锁定输出"。

    用法（detector 内部）：
        t = TargetTracker(cfg)
        out = t.update(u, v, scale, quad, ts, sharp=sharp, quality=score)
        out = t.coast(ts)          # 本帧没检出时的短时外推（可选）
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    # ------------------------------------------------------------------
    # 参数与生命周期
    # ------------------------------------------------------------------
    def _p(self, name, default):
        """取参数：优先 TRACK_PRESETS[当前档位]，其次 config.TRACK_XXX，最后默认值。"""
        cfg = self.cfg
        pres = getattr(cfg, "TRACK_PRESETS", {}) or {}
        mode = getattr(cfg, "TRACK_MODE", None) or getattr(cfg, "FILTER_MODE", "static")
        d = pres.get(mode, {}) or {}
        if name in d:
            return d[name]
        return getattr(cfg, "TRACK_" + name.upper(), default)

    def reset(self):
        cfg = self.cfg
        fp = cfg.FILTER_PARAMS[cfg.FILTER_MODE]
        self.fu = _CV1D()
        self.fv = _CV1D()
        # 尺度变化慢、且不需要运动学外推 → 用 One Euro 就够（保留它的正当用途）
        self.fscale = G.OneEuroFilter(freq=30.0, **fp)
        self.t_clock = 0.0          # 内部单调时钟（时间戳缺失/异常时也能正常工作）
        self.q_est = None           # 自适应后的过程噪声（下一帧预测用）
        self.inn2 = None            # 新息平方的滑动估计（px²）—— 门限/速度上限用的"位置实际动多少"
        self.man = None             # 机动倍率（新息²/meas_var − 1）的滑动估计 —— 决定 q
        self.kind = None            # 当前锁定的候选族（ring/paper），供族迟滞
        self.th = None
        self.quad = None
        self.scale = 0.0
        self.diag = 0.0
        self.t_prev = None
        self.n_accept = 0
        self.n_gate = 0             # 当前连续拒信帧数
        self.n_hold = 0             # 最近"保持"帧数（输出预测值而非测量）
        self.n_coast = 0            # 连续"外推"帧数（这一帧干脆没检出）
        self.last_resid = 0.0
        self.last_nis = 0.0
        self.sharp_ema = None
        self.last_sharp = None
        self.gate_reason = ""
        self.jumped = False

    def ready(self):
        return self.fu.ok and self.fv.ok

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------
    def _tick(self, ts):
        """推进内部时钟并返回 dt（秒）。时间戳缺失/乱序时退回标称 1/30s。"""
        if ts is not None and self.t_prev is not None:
            try:
                d = float(ts) - float(self.t_prev)
            except Exception:
                d = 0.0
            if not np.isfinite(d) or d <= 0.0:
                d = 1.0 / 30.0
        else:
            d = 1.0 / 30.0
        d = float(min(max(d, 1.0 / 240.0), 0.5))
        self.t_clock += d
        return d

    def _sharp_ratio(self, sharp):
        """
        清晰度相对基线（自标定，不需要现场标参数）。
        基线用**非对称 EMA**：只让"更清晰"的帧把它快速抬上去，
        变糊的帧只慢慢拉低 —— 这样长时间抖动不会把基线一起拖下去
        （否则连续晃动会让"糊"变成新常态，门限就失效了）。
        """
        if sharp is None or not np.isfinite(sharp) or sharp <= 0:
            return 1.0
        if self.sharp_ema is None:
            self.sharp_ema = float(sharp)
            return 1.0
        up = sharp >= self.sharp_ema
        self.sharp_ema = float(self.sharp_ema
                               + (0.30 if up else 0.02) * (sharp - self.sharp_ema))
        return float(sharp) / max(1e-9, self.sharp_ema)

    def _ema(self, name, val, dt, tau, burnin=False, clip=0.0, clip_floor=0.0):
        """
        指数滑动平均（可选"单帧上限"）。

        单帧上限的意义：一次离群不该改变这把尺子。但上限必须有一个**正的绝对下限**
        （clip_floor）—— 踩过的坑：只用"当前值的 N 倍"做上限时，一旦估计落到 0
        （excess 是可以正好为 0 的），上限也变成 0，估计就**永久锁死在 0**，
        之后再也涨不回来（表现为"跟手能力整段消失"）。
        """
        cur = getattr(self, name)
        if cur is None:
            setattr(self, name, float(val))
            return float(val)
        v = float(val)
        if clip > 1.0:
            v = min(v, max(cur * clip, float(clip_floor)))
        tau_eff = max(1e-3, tau / (4.0 if burnin else 1.0))
        beta = 1.0 - math.exp(-max(1e-6, dt) / tau_eff)
        v = float(cur + beta * (v - cur))
        setattr(self, name, v)
        return v

    def _adapt_q(self, resid, dt, q_min, q_max, tau, burnin=False, r_base=1.0):
        """
        由新息在线估计过程噪声 q —— **本模块最关键的一环**。

        为什么必须自适应（实测）：车身晃动要让 CV 模型跟得住，需要
        q ≈ 1e7 px²/s³；而静止靶纸要平滑干净，q ≈ 1e3 就够。
        差四个数量级，固定值不可能两头兼顾：
            q 取小 → 抖动时滞后几十像素（真机症状："车一晃中心就偏"）
            q 取大 → 静止时等于没滤波，测量噪声原样透出来

        ★ 判据必须**扣掉测量噪声**（这里踩过一个很隐蔽的坑）：
          不能直接拿"新息平方"当机动强度 —— 静止靶纸的新息**就是测量噪声本身**，
          于是 q 被噪声抬起来 → 滤波器转而去追噪声 → 抑噪能力归零
          （实测：无论噪声 σ=1/2/4px，输出抖动只降 1~5%，等于没滤波）。
          正确做法是看**新息²相对"纯噪声水平"的富余倍率**：

              ratio = max(0, 新息² / meas_var − 1)      （无量纲）

          静止时 新息²≈meas_var → ratio≈0 → q 落到下限 → 重平滑；
          晃动时 新息²远大于 meas_var → ratio 很大 → q 升到上限 → 跟手。

        ⚠ 也不要拿"新息² − S"（S 含滤波器自身的预测协方差）：那会形成自反馈 ——
          q 一大 S 就大、于是判据认为"没有机动"、q 又掉下来，估计被自己压住，
          实测跟手能力大幅退化。用 meas_var 这种**与滤波器状态无关**的量做基准，
          估计才是单调、稳定、不会自锁的。

        ratio 经时间常数为 tau 的滑动平均（起捕前若干帧用 4 倍速 burn-in）后，
        再**按对数**映射到 [q_min, q_max] —— 必须是对数，不能线性：
        q 跨 4 个数量级，线性映射下 ratio 只要有一点余量（比如噪声正常波动带来
        的 0.04）就一步跳到 1.6e6，"静止时重平滑"直接失效（实测抑噪只剩 1%）。

        ⚠ 前提：meas_var 要大致符合实际检测抖动 σ²（见 config 说明）。
          它是"多小的新息算噪声"这条分界线，标定偏小会把噪声当运动（不抑噪）。
        """
        r = max(1e-9, float(r_base))
        ratio = max(0.0, float(resid) * float(resid) / r - 1.0)
        m = self._ema("man", ratio, dt, tau, burnin,
                      clip=float(getattr(self.cfg, "TRACK_INN_CLIP", 9.0)),
                      clip_floor=1.0)
        f = m / (m + 1.0)
        qmin = max(1e-6, float(q_min))
        qmax = max(qmin, float(q_max))
        return float(qmin * (qmax / qmin) ** f)

    def _seed(self, u, v, scale, diag, quad, ts):
        self.fu.seed(u)
        self.fv.seed(v)
        if scale > 0:
            self.fscale.reset()
            self.fscale(scale, self.t_clock)
            self.scale = float(scale)
        self.diag = diag if diag > 1.0 else self.diag
        self.quad = quad
        self.t_prev = ts
        self.n_gate = self.n_hold = self.n_coast = 0
        # 刚起捕/刚重捕：还没有新息可用 → 先用"跟手"的 q，等看到新息再自适应收紧，
        # 免得一上来就用重平滑把真实的大位移拖住（宁可跟手，不可滞后）
        if self.q_est is None:
            self.q_est = float(self._p("q_max", 4.0e7))
        # "机动强度"初值给宽一点：起捕时还不知道抖动有多大，门限先松、再随新息收紧。
        # 取目标尺寸的 5% 作为初始"一帧可能移动多少"的量级。
        if self.inn2 is None and diag > 1.0:
            self.inn2 = (0.05 * diag) ** 2

    # ------------------------------------------------------------------
    # 主入口
    # ------------------------------------------------------------------
    def update(self, u, v, scale, quad, ts, sharp=None, quality=1.0):
        """
        采入一帧"原始解算结果"，返回稳定化后的输出。

        返回 dict(valid, u, v, scale, held, resid, nis, rejected, stage, quad)
          held     True 表示这一帧的输出是**预测值**（测量被判为可疑，没采信）
          resid    测量与预测的欧氏距离（px）
          nis      归一化新息平方（越小越"讲得通"）
        """
        cfg = self.cfg
        dt = self._tick(ts)
        r_base = float(self._p("meas_var", 6.0))
        gate = float(self._p("gate", 4.5))
        q_min = float(self._p("q_min", 800.0))
        q_max = float(self._p("q_max", 4.0e7))
        q_tau = float(getattr(cfg, "TRACK_Q_TAU", 0.35))
        # 本帧预测用**上一帧估出来的** q（还没看到本帧测量，不能偷看未来）
        q_used = self.q_est if self.q_est is not None else q_max
        size_tol = float(getattr(cfg, "TRACK_SIZE_TOL", 1.28))
        scale_tol = float(getattr(cfg, "TRACK_SCALE_TOL", 1.22))
        max_gate = max(1, int(getattr(cfg, "TRACK_MAX_GATE_FRAMES", 12)))
        rec_gain = float(getattr(cfg, "TRACK_RECOVER_GAIN", 2.2))
        blur_infl = float(getattr(cfg, "TRACK_BLUR_INFLATE", 3.0))
        sharp_min = float(getattr(cfg, "TRACK_SHARP_RATIO", 0.65))
        warmup = int(getattr(cfg, "TRACK_SHARP_WARMUP", 8))

        u, v = float(u), float(v)
        scale = float(scale) if scale and scale > 0 else 0.0
        diag = quad_diag(quad)

        # ---- 第一帧：直接建立状态（此时没有"历史"可供比较）----
        if not self.ready():
            self._seed(u, v, scale, diag, quad, ts)
            self.kind, self.th = None, None
            self.n_accept = 1
            self.last_resid = self.last_nis = 0.0
            self.last_sharp = sharp
            self._sharp_ratio(sharp)
            self.gate_reason = ""
            self.jumped = False
            return dict(valid=True, u=u, v=v, scale=scale, held=False, resid=0.0,
                        nis=0.0, rejected=False, stage="track-init", quad=quad)

        # ---- 预测 ----
        pu = self.fu.predict(dt, q_used)
        pv = self.fv.predict(dt, q_used)

        shr = self._sharp_ratio(sharp)
        self.last_sharp = sharp
        blurry = (self.n_accept >= warmup and blur_infl > 0.0 and shr < sharp_min)

        # 测量噪声：糊帧放大（滤波器会更依赖预测值）；低分候选也放大
        r = r_base
        if blurry:
            r *= 1.0 + (blur_infl - 1.0) * max(0.0, 1.0 - shr)
        try:
            qv = float(quality)
            if 0.0 < qv < 1.0:
                r *= 1.0 + 1.5 * (1.0 - min(1.0, qv))
        except Exception:
            pass

        resid = float(math.hypot(u - pu, v - pv))
        S = self.fu.P00 + self.fv.P00 + 2.0 * r
        nis = float((resid * resid) / max(1e-9, S))
        # 门限用的尺度取**本帧之前**的估计（不能拿正被检验的这一个样本来定门限）
        inn_prev = self.inn2

        # ---- 一致性判定（任一不满足即判"这一帧不可信"）----
        # ① 硬跳变门限（与协方差无关，**这条是抗误锁的主力**）
        #    为什么不能只靠统计门限（nis）：q 自适应到上限后（跟住抖动必须如此），
        #    过程噪声把新息协方差抬得很大，实测一个 84px 的离群点只算 3.65σ → 直接放行；
        #    放行之后它会**污染速度**，下一帧预测能飞出去 200px。
        #    门限怎么定：位置是连续的，"这一帧该移动多少"由**最近几帧的实际移动量**
        #    决定（即新息尺度 sqrt(inn2)），而不是由目标多大决定 ——
        #    大目标上的一次误锁可能只偏了自身尺寸的 10%，用"相对尺寸"根本拦不住。
        #    所以 gate = max(绝对下限, K x 近期新息尺度)，对远近/快慢都自适应：
        #    靶纸纹丝不动时门限很紧（几十像素的误锁都拦），剧烈晃动时门限自动放宽。
        reason = ""
        scale_inn = math.sqrt(inn_prev) if (inn_prev and inn_prev > 0) else 0.0
        jump_max = max(float(getattr(cfg, "TRACK_JUMP_ABS", 25.0)),
                       float(getattr(cfg, "TRACK_JUMP_K", 5.0)) * scale_inn)
        # ★ 连续拒信时**主动放开门限**：真运动（过坎/起步的突然位移）与误锁在
        #   "单帧跳变"上是不可区分的；如果门限死不动，一次真跳变会被判成误锁，
        #   接着"预测值停留在旧位置 → 后续测量全都显得更离谱 → 继续拒信"形成
        #   级联锁死（实测能连续保持十几帧、误差漂到 200px）。
        #   所以第 1 条门限只在**第一帧**上严格（挡住一次性坏点），异常持续存在就
        #   认为"这是真运动"并放开。放开的自由度只给"中心"，**形状门限不放开** ——
        #   持续误锁与真运动的可靠分野在形状（框尺寸/尺度），不在中心。
        jump_max *= 1.0 + float(getattr(cfg, "TRACK_GATE_OPEN", 1.2)) * self.n_gate
        if resid > jump_max:
            reason = "跳变%.0fpx>%.0f" % (resid, jump_max)
        # ② 框尺寸 / 尺度一致性：误锁几乎总伴随形状突变（真机 px/cm 22↔12.5 就是这类）
        if not reason and self.diag > 1.0 and diag > 1.0:
            ratio = diag / self.diag
            if ratio > size_tol or ratio < 1.0 / size_tol:
                reason = "框尺寸突变%.2fx" % ratio
        if not reason and self.scale > 0 and scale > 0:
            ratio = scale / self.scale
            if ratio > scale_tol or ratio < 1.0 / scale_tol:
                reason = "尺度突变%.2fx" % ratio
        # ③ 统计门限（新息偏离预测几个 sigma）—— 辅助，硬门限漏掉的再补一刀
        if not reason and nis > gate * gate:
            reason = "残差%.0fpx" % resid

        # ---- 可疑：不采信，输出预测值；并逐步放大不确定度 ----
        if reason:
            self.n_gate += 1
            self.n_hold += 1
            self.last_resid, self.last_nis = resid, nis
            self.t_prev = ts
            if self.n_gate >= max_gate:
                # 偏差持续存在 → 不是误锁而是目标真的动了/换了位置：重新起捕，别卡死
                self._seed(u, v, scale, diag, quad, ts)
                self.jumped = True
                self.last_resid = self.last_nis = 0.0
                self.gate_reason = "重捕(%s)" % reason
                return dict(valid=True, u=u, v=v, scale=self.scale, held=False,
                            resid=resid, nis=nis, rejected=False,
                            stage="track-jump", quad=quad)
            self.fu.inflate(rec_gain)
            self.fv.inflate(rec_gain)
            # 保持期间**衰减速度**：靶纸是静止的，拿陈旧速度一路外推会让预测值越跑越偏
            # （实测"被遮挡十几帧"的场景里能漂出十几像素）。缓衰减既压住漂移，
            # 又不像直接清零那样在"目标真在动"时一把丢掉动量。
            damp = float(getattr(cfg, "TRACK_HOLD_VEL_DAMP", 0.6))
            if 0.0 < damp < 1.0:
                self.fu.v *= damp
                self.fv.v *= damp
            self.gate_reason = reason
            self.jumped = False
            return dict(valid=True, u=pu, v=pv, scale=self.scale, held=True,
                        resid=resid, nis=nis, rejected=True,
                        stage="track-hold", quad=self.quad)

        # ---- 采信 ----
        # ① 机动强度（喂"模型解释不了的那部分"，见 _adapt_q）→ 决定过程噪声 q
        #    ★ 只在**采信**时更新：离群点绝不允许喂养这把"尺度尺子"。
        #    踩过的坑：最早连被拒的样本也喂进去（虽然做了 9 倍截顶），
        #    但 beta≈0.09 下单帧仍能让它涨 72%，十四次离群就把门限自己撑松了，
        #    于是后面的离群点全部被放行 —— 门限形同虚设。
        burnin = bool(self.n_accept < int(getattr(cfg, "TRACK_Q_BURNIN", 10)))
        self.q_est = self._adapt_q(resid, dt, q_min, q_max, q_tau,
                                   burnin=burnin, r_base=r_base)
        # ② 新息尺度（供**下一帧**的门限与速度上限用）：这里要的是"位置实际在动多少"，
        #    所以吃**全部**新息（含测量噪声）；同样只由采信帧喂养。
        self._ema("inn2", resid * resid, dt, q_tau, burnin,
                  clip=float(getattr(cfg, "TRACK_INN_CLIP", 9.0)),
                  clip_floor=r_base)
        # ③ 速度修正量上限：由"位置实际在动多少"推断的可能加速度 a≈sqrt(inn2)/dt²，
        #    一帧内速度最多改 a*dt = sqrt(inn2)/dt，再留 K 倍余量。
        dv_max = 0.0
        if self.inn2 and self.inn2 > 0.0:
            dv_max = float(getattr(cfg, "TRACK_DV_K", 1.5)) * math.sqrt(self.inn2) / dt
        self.fu.update(u, r, dv_max)
        self.fv.update(v, r, dv_max)
        self.scale = (float(self.fscale(scale, self.t_clock))
                      if scale > 0 else self.scale)
        self.diag = diag if diag > 1.0 else self.diag
        self.quad = quad
        self.t_prev = ts
        self.n_accept += 1
        self.n_gate = 0
        self.n_coast = 0
        if self.n_hold > 0:
            self.n_hold -= 1            # 缓缓回落，便于日志观察"刚刚保持过"
        self.last_resid, self.last_nis = resid, nis
        self.gate_reason = ""
        self.jumped = False
        return dict(valid=True, u=self.fu.p, v=self.fv.p, scale=self.scale,
                    held=False, resid=resid, nis=nis, rejected=False,
                    stage="track", quad=quad)

    def note_meta(self, kind=None, th=None):
        """记录本帧胜出候选的族与阈值（族迟滞用；采样后调用）。"""
        if kind:
            self.kind = kind
        if th is not None:
            self.th = th

    def coast(self, ts):
        """
        本帧**完全没检出**时的短时外推：用预测值继续输出，避免锁定在抖动中掉一帧。
        上限 TRACK_MAX_COAST 帧（约 0.1~0.2s），超了就放掉锁定（返回 None），
        以免一直照着早就消失的目标输出。
        """
        if not self.ready():
            return None
        cap = int(getattr(self.cfg, "TRACK_MAX_COAST", 3))
        if cap <= 0 or self.n_coast >= cap:
            return None
        dt = self._tick(ts)
        q = self.q_est if self.q_est is not None else float(self._p("q_max", 4.0e7))
        pu = self.fu.predict(dt, q)
        pv = self.fv.predict(dt, q)
        self.fu.inflate(1.6)
        self.fv.inflate(1.6)
        self.t_prev = ts
        self.n_coast += 1
        self.n_hold += 1
        return dict(valid=True, u=pu, v=pv, scale=self.scale, held=True,
                    resid=0.0, nis=0.0, rejected=True, stage="track-coast",
                    quad=self.quad)
