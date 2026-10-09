"""开环标定（一次性）：在图像上实测 PULSE_PER_PX / DIR_SIGN / 交叉项 / 回差。

为什么必须有这一步：闭环能否收敛、方向是否正确，全取决于 ``PULSE_PER_PX_*`` 与
``DIR_SIGN_*``（需求文档 §7.2/§7.3）。理论换算要数齿数、查细分、量安装角，任一错就全错；
本模块直接在图像上实测 —— **"走 N 个脉冲，看靶心在图像里动了多少像素"**，一次性把
齿轮比、细分、焦距、安装角全部吸收掉。

流程（非阻塞状态机，与主循环共用链路、不阻塞视觉）::

    静置取基线 (dx0,dy0)
      -> 正方向走 N 脉冲 -> 静置 -> 测 (dx1,dy1)
      -> 反方向走 N 脉冲回位 -> 静置 -> 测 (dx2,dy2)

输出：
    PULSE_PER_PX = N / |Δ|；DIR_SIGN（走正方向后 |误差| 反而变大 ⇒ 符号反 ⇒ -1）；
    交叉项 C = Δdy/Δdx（判相机横滚）；回差估计 |Δ2−Δ0| / 2（决定 DEADBAND 下限）。

标定期 main 会跳过闭环（见 ``CALIBRATE_AXIS`` 非空）。两种用法：

* **人工**：``CALIBRATE_AXIS = "yaw"``（或 ``"pitch"``）跑一次，把打印出的数值填进
  ``config.py``，再置回 ``""``，重新上电进入跟踪；
* **自动**：``CALIBRATE_AXIS = "auto"`` —— ``AutoCalibrate`` 依次标定两轴，成功后把结果
  写回**运行时** cfg 并解锁闭环，**上电一次即进入跟踪**（无需回填、无需重启）。

保持纯 Python（不 import maix），可在 PC 上用假链路回归。
"""


def _estimate_seconds(pulses, rpm, acc, pulses_per_rev):
    """复用自检的时间建模；单文件打包等 import 不可用场景退化为保守估计（曲线留 1.4 倍余量）。"""
    try:
        from vision.gimbal_self_test import estimate_move_seconds
        return estimate_move_seconds(pulses, rpm, acc, pulses_per_rev)
    except Exception:
        travel = pulses / max(1e-6, pulses_per_rev * max(1, rpm) / 60.0)
        return travel * (1.4 if int(acc) > 0 else 1.0)


class GimbalCalibrator:
    def __init__(self, cfg, link, now=0.0, axis=None):
        self.cfg = cfg
        self.link = link
        # axis 显式传入时优先于 cfg.CALIBRATE_AXIS —— AutoCalibrate 靠它逐轴切换。
        self.axis = (str(axis) if axis is not None
                     else str(getattr(cfg, "CALIBRATE_AXIS", ""))).strip().lower()
        self.pulses = int(getattr(cfg, "CALIBRATE_PULSES", 2000))
        self.need = max(2, int(getattr(cfg, "CALIBRATE_SAMPLES", 5)))
        self.samples = []
        self.base = self.out = self.back = None
        self.report = None
        self.reason = None
        self.started = float(now)
        self.deadline = float(now) + float(getattr(cfg, "CALIBRATE_SETTLE_S", 1.0))
        # ★ 视觉无效快速失败：标定每一步都要 dx/dy，画面里没有靶心时不该干等 CALIBRATE_TIMEOUT_S
        #   （现场"抬头后黑框出视野 -> 卡在等 dv"就是这么来的）。
        self._last_dv = float(now)
        self.visual_timeout_s = float(getattr(cfg, "CALIBRATE_VISUAL_TIMEOUT_S", 0.0) or 0.0)
        self.limit_s = float(getattr(cfg, "CALIBRATE_TIMEOUT_S", 20.0))
        self.state = "BASELINE"
        if self.axis not in ("yaw", "pitch") or self.pulses <= 0:
            self.state, self.reason = "SKIPPED", "CALIBRATE_AXIS unset or invalid"
        else:
            print("[calib] start axis=%s pulses=%d samples=%d rpm=%d" %
                  (self.axis, self.pulses, self.need, int(getattr(cfg, "CALIBRATE_RPM", 300))))

    # ------------------------------------------------------------- 接口
    @property
    def done(self):
        return self.state in ("DONE", "SKIPPED", "FAULT")

    def update(self, now, result=None):
        """每帧调用；到达估算运动时间后推进状态机，不阻塞主循环。"""
        now = float(now)
        if self.done:
            return self.state
        # ★ 视觉无效**快速失败**：不再等到 20s 总超时。只要连续 visual_timeout_s 收不到有效
        #   dx/dy，就立刻判失败、把控制权交还主流程 —— 这是"没靶心 / 抬头后黑框跑出视野
        #   导致校准卡住"的直接解法。
        if self.visual_timeout_s > 0 and now - self._last_dv >= self.visual_timeout_s:
            return self._fault("no_vision(%.1fs 无有效 dx/dy)" % self.visual_timeout_s)
        if now - self.started > self.limit_s:
            return self._fault("timeout_no_valid_aim(%.1fs)" % self.limit_s)
        if now < self.deadline:
            return self.state
        dv = None
        if result and result.get("aim_valid") and result.get("dx") is not None:
            dv = (float(result["dx"]), float(result["dy"]))
            self._last_dv = now          # 有有效视觉 -> 刷新，视觉超时不会误伤正常标定
        if self.state == "BASELINE":
            if dv is None:
                return self.state
            self.samples.append(dv)
            if len(self.samples) < self.need:
                return self.state
            self.base = self._avg()
            print("[calib] baseline  dx=%+.2f dy=%+.2f" % self.base)
            if not self._send(ccw=True):
                return self.state
            self.state = "MOVE_OUT"
            self.deadline = now + self._move_seconds()
        elif self.state == "MOVE_OUT":
            self.samples = []
            self.state = "MEASURE_OUT"
        elif self.state == "MEASURE_OUT":
            if dv is None:
                return self.state
            self.samples.append(dv)
            if len(self.samples) < self.need:
                return self.state
            self.out = self._avg()
            print("[calib] after +%dp dx=%+.2f dy=%+.2f" % (self.pulses, self.out[0], self.out[1]))
            if not self._send(ccw=False):
                return self.state
            self.state = "MOVE_BACK"
            self.deadline = now + self._move_seconds()
        elif self.state == "MOVE_BACK":
            self.samples = []
            self.state = "MEASURE_BACK"
        elif self.state == "MEASURE_BACK":
            if dv is None:
                return self.state
            self.samples.append(dv)
            if len(self.samples) < self.need:
                return self.state
            self.back = self._avg()
            print("[calib] after -%dp dx=%+.2f dy=%+.2f" % (self.pulses, self.back[0], self.back[1]))
            self._finish()
        return self.state

    # ------------------------------------------------------------- 内部
    def _idx(self):
        return 0 if self.axis == "yaw" else 1

    def _avg(self):
        n = float(len(self.samples))
        return (sum(s[0] for s in self.samples) / n,
                sum(s[1] for s in self.samples) / n)

    def _move_seconds(self):
        rpm = max(1, int(getattr(self.cfg, "CALIBRATE_RPM", 60)))
        acc = int(getattr(self.cfg, "CALIBRATE_ACC", getattr(self.cfg, "SELF_TEST_ACC", 0)))
        ppr = max(1.0, float(getattr(self.cfg, "MOTOR_PULSES_PER_REV", 3200)))
        # 与自检同一套建模（含曲线加减速；行程不足自动退化三角曲线），避免早发下一条。
        travel = _estimate_seconds(self.pulses, rpm, acc, ppr)
        settle = float(getattr(self.cfg, "CALIBRATE_SETTLE_S", 1.0))
        return travel + settle

    def _send(self, ccw):
        rpm = int(getattr(self.cfg, "CALIBRATE_RPM", 60))
        acc = int(getattr(self.cfg, "CALIBRATE_ACC", getattr(self.cfg, "SELF_TEST_ACC", 0)))
        ok = self.link.move(self.axis, ccw, self.pulses, rpm, acc)
        if not ok:
            self._fault("uart_write_failed")
        return bool(ok)

    def _fault(self, reason):
        self.state, self.reason = "FAULT", str(reason)
        print("[calib] fault: %s" % self.reason)
        return self.state

    def _finish(self):
        i = self._idx()
        delta = self.out[i] - self.base[i]
        cross = (self.out[1] - self.base[1]) if i == 0 else (self.out[0] - self.base[0])
        if abs(delta) < 0.5:
            return self._fault("no_motion(%.2fpx after %dp)" % (delta, self.pulses))
        ppp = self.pulses / abs(delta)
        sign = 1 if delta < 0 else -1
        c = cross / delta
        residual = self.back[i] - self.base[i]
        backlash = abs(residual) / 2.0
        upper = self.axis.upper()
        self.report = dict(axis=self.axis, pulse_per_px=ppp, dir_sign=sign,
                           cross=c, backlash_px=backlash, residual_px=residual)
        print("[calib] ---------------- 结果（填进 config.py）----------------")
        print("[calib]   PULSE_PER_PX_%s = %.2f" % (upper, ppp))
        print("[calib]   DIR_SIGN_%s = %d" % (upper, sign))
        print("[calib]   交叉项 C=%.3f %s" % (
            c, "→ |C|>0.25，相机横滚没装正，优先机械调整" if abs(c) > 0.25 else "(正常)"))
        print("[calib]   回差估计 ≈ %.2fpx（DEADBAND_PX 建议 ≥ 1.5×该值，下限 1.5px）" % backlash)
        print("[calib]   换轴标定请把 CALIBRATE_AXIS 改成另一轴重跑；标完置回 \"\"")
        self.state = "DONE"
        return self.state


class AutoCalibrate:
    """开机自动逐轴标定（``CALIBRATE_AXIS = "auto"``）：yaw -> pitch，成功后自动解锁闭环。

    与人工标定的差别：不需要看屏幕读数、不需要回填 config 再重启 —— 实测结果直接写回
    **运行时**的 cfg 对象，上层据此解除 ``GimbalAim`` 的未标定锁，上电一次即可从
    "自检" 直接进入 "跟踪"。

    安全口径：**任一轴失败就整体不解锁**（宁可不追，也不用错误参数正反馈甩头）。
    标定前提是画面里能看到靶心，否则 ``dx/dy`` 无效 -> 超时失败。
    """

    def __init__(self, cfg, link, now=0.0, axes=None):
        self.cfg = cfg
        self.link = link
        self.axes = [str(a).lower() for a in
                     (axes or getattr(cfg, "AUTO_CALIBRATE_AXES", ("yaw", "pitch")))]
        self.axis = None
        self.current = None
        self.results = {}
        self.failed = {}
        self.state = "RUNNING"
        self.reason = None
        self._started = float(now)
        self._limit_s = float(getattr(cfg, "AUTO_CALIBRATE_TIMEOUT_S", 150.0))
        self._next(float(now))

    # ------------------------------------------------------------- 接口
    @property
    def done(self):
        return self.state in ("DONE", "FAILED", "SKIPPED")

    @property
    def applied(self):
        """是否已把结果写回 cfg 且**两轴全部成功** —— 只有此时才允许解锁闭环。"""
        return bool(self.results) and not self.failed

    @property
    def deadline(self):
        """当前正在标定的那一轴的截止时刻（供上层推进时钟/测试用）。"""
        return self.current.deadline if self.current is not None else self._started

    def update(self, now, result=None):
        now = float(now)
        if self.done:
            return self.state
        if now - self._started > self._limit_s:
            return self._fail("auto_calibrate_timeout(%.0fs)" % self._limit_s)
        if self.current is None:
            return self.state
        self.current.update(now, result)
        if not self.current.done:
            return self.state
        if self.current.state == "DONE" and self.current.report:
            self.results[self.axis] = dict(self.current.report)
            print("[calib-auto] %s 标定成功" % self.axis)
        else:
            self.failed[self.axis] = self.current.reason or self.current.state
            print("[calib-auto] %s 标定失败: %s" % (self.axis, self.failed[self.axis]))
        self._next(now)
        return self.state

    def apply(self):
        """把实测值写回运行时 cfg（**不改磁盘 config.py**），并打印可粘贴片段。"""
        for axis, rep in self.results.items():
            upper = axis.upper()
            setattr(self.cfg, "PULSE_PER_PX_%s" % upper, round(float(rep["pulse_per_px"]), 2))
            setattr(self.cfg, "DIR_SIGN_%s" % upper, int(rep["dir_sign"]))
        print("[calib-auto] 已写入运行时 config（磁盘文件未改）。要固定下来请复制到 config.py：")
        for axis, rep in self.results.items():
            upper = axis.upper()
            print("[calib-auto]   PULSE_PER_PX_%s = %.2f" % (upper, rep["pulse_per_px"]))
            print("[calib-auto]   DIR_SIGN_%s = %d" % (upper, rep["dir_sign"]))
        return len(self.results)

    # ------------------------------------------------------------- 内部
    def _next(self, now):
        remaining = [a for a in self.axes if a not in self.results and a not in self.failed]
        if not remaining:
            return self._finish()
        self.axis = remaining[0]
        self.current = GimbalCalibrator(self.cfg, self.link, now, axis=self.axis)
        if self.current.done:
            # 该轴不可标（脉冲数非法等）-> 记失败后继续下一轴
            self.failed[self.axis] = self.current.reason or "unavailable"
            print("[calib-auto] %s 无法标定: %s" % (self.axis, self.failed[self.axis]))
            return self._next(now)
        idx = len(self.results) + len(self.failed) + 1
        print("[calib-auto] 开始标定 %s（第 %d/%d 轴，走 %d 脉冲，请确认云台有行程余量）" %
              (self.axis, idx, len(self.axes), self.current.pulses))
        return None

    def _finish(self):
        print("[calib-auto] ================== 自动标定汇总 ==================")
        for axis in self.axes:
            if axis in self.results:
                rep = self.results[axis]
                print("[calib-auto]   %-5s OK    PULSE_PER_PX=%7.2f  DIR_SIGN=%+d  交叉=%+.3f  回差=%.2fpx"
                      % (axis, rep["pulse_per_px"], rep["dir_sign"],
                         rep["cross"], rep["backlash_px"]))
            else:
                print("[calib-auto]   %-5s 失败  %s" % (axis, self.failed.get(axis)))
        if not self.results:
            self.state, self.reason = "FAILED", "no_axis_calibrated"
            print("[calib-auto] 没有任何轴标定成功 -> 闭环保持锁定（不会发电机指令）")
            return self.state
        self.apply()
        if self.failed:
            self.state = "DONE"
            self.reason = "partial:%s" % ",".join(sorted(self.failed))
            print("[calib-auto] 有轴未标定成功(%s) -> 闭环仍**保持锁定**，请修好后重跑"
                  % ",".join(sorted(self.failed)))
            return self.state
        self.state = "DONE"
        return self.state

    def _fail(self, reason):
        self.state, self.reason = "FAILED", str(reason)
        print("[calib-auto] 失败：%s" % self.reason)
        return self.state
