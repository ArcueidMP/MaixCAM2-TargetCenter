"""基于像素误差的两轴增量比例控制器 + 状态机。

对照 `MaixCAM2串口直控云台需求文档.md` §7 实现：

- 闭环输入 = ``result["dx"]/["dy"]``（靶心 − 光斑，见 ``laser_spot.deviation``）；
- 相对位置命令（0xFD、mode=0），运动量低通/增长限速，等待曲线运动估算完成；
- 状态机 ``BOOT / COARSE / FINE / SETTLE / HOLD / SCAN / FAULT``；
- 三重保护：离群拒收（``JUMP_REJECT_PX``）、死区（``DEADBAND_PX``）、指令限频
  （``COMMAND_INTERVAL_S``）、单帧限幅（``MAX_STEP_PULSES``）。

本模块**保持纯 Python（不 import maix）**，可在 PC 上离线仿真与回归。
控制律可整体替换：``controller(axis, error_px, now) -> 带符号脉冲数``，传 None 用内置比例律。
"""

import math

from vision.gimbal_self_test import estimate_move_seconds

# 状态语义（给日志和上层判断用）
BOOT = "BOOT"        # 上电，等第一帧合法误差
COARSE = "COARSE"    # 大误差，大步快速逼近
FINE = "FINE"        # 小误差，小步逼近
SETTLE = "SETTLE"    # 进入死区，停止发指令
HOLD = "HOLD"        # 视觉失效，保持不动
SCAN = "SCAN"        # 失锁后慢扫重捕获（需 ENABLE_SCAN）
FAULT = "FAULT"      # 链路/机械故障，停机不再发指令
UNCALIBRATED = "UNCALIBRATED"  # 未标定 PULSE_PER_PX/DIR_SIGN，闭环锁定、只出视觉结果


class GimbalAim:
    def __init__(self, cfg, controller=None):
        self.cfg = cfg
        self.controller = controller
        self.state = BOOT
        self.fault_reason = None
        self.last_command_time = {"yaw": -1e9, "pitch": -1e9}
        self.last = {"state": self.state, "dx": None, "dy": None,
                     "cmds": {"yaw": None, "pitch": None}}
        self._jump = {"yaw": 0, "pitch": 0}
        self._lost = 0
        self._settle = 0
        self._scan_dir = 1
        self._scan_pos = 0
        # 安全闸：未完成开环标定时禁止发电机指令。PULSE_PER_PX_*/DIR_SIGN_* 若还是占位值，
        # 直接闭环可能正反馈甩头（README §四）。标定完把 GIMBAL_CALIBRATED=True 解锁。
        self.require_calibration = bool(getattr(cfg, "AIM_REQUIRE_CALIBRATION", False))
        self.calibrated = bool(getattr(cfg, "GIMBAL_CALIBRATED", False))
        self.locked = self.require_calibration and not self.calibrated
        if self.locked:
            self.state = UNCALIBRATED
            print("[aim] 未标定：闭环已锁定（只输出视觉结果，不发电机指令）")
            print("[aim] 先跑 CALIBRATE_AXIS 标定 PULSE_PER_PX_*/DIR_SIGN_*，再置 GIMBAL_CALIBRATED=True")
        # ★ 发散看护：一直发指令却始终不收敛（误差反而变大）= 方向反了（正反馈）
        #   -> 自动 FAULT 停机，避免云台一路冲到底。详见 `_guard_check()`。
        self._guard_on = bool(getattr(cfg, "DIVERGE_GUARD", False))
        self._guard_frames = int(getattr(cfg, "DIVERGE_FRAMES", 30))
        self._guard_ratio = float(getattr(cfg, "DIVERGE_RATIO", 1.5))
        self._guard_n = 0
        self._guard_min = None
        self._motion_until = {"yaw": -1e9, "pitch": -1e9}
        self._pending_stops = set()
        self._clear_output()

    # ------------------------------------------------------------- 外部接口
    def set_controller(self, controller):
        """设置后续 PID/前馈控制器；传入 None 恢复内置比例律。"""
        self.controller = controller
        self._clear_output()

    def fault(self, reason):
        """进入 FAULT：不再发任何指令，由上层决定是否复位。"""
        self.state = FAULT
        self.fault_reason = str(reason)
        self._clear_output()
        print("[aim] FAULT: %s" % self.fault_reason)
        return self.state

    def unlock(self, reason="calibrated"):
        """标定完成 -> 解除未标定锁，允许发电机指令。返回是否真的发生了状态变化。

        由 ``AutoCalibrate`` 在两轴全部标定成功后调用。只有在
        ``AIM_REQUIRE_CALIBRATION=True`` 且 ``GIMBAL_CALIBRATED=False`` 时才处于锁定态，
        因此重复调用是幂等的（第二次返回 False）。
        """
        if not self.locked:
            return False
        self.locked = False
        self.calibrated = True
        if self.state == UNCALIBRATED:
            self.state = BOOT
        self.fault_reason = None
        self._lost = self._settle = self._scan_pos = 0
        self._scan_dir = 1
        self._jump = {"yaw": 0, "pitch": 0}
        self.last_command_time = {"yaw": -1e9, "pitch": -1e9}
        self._clear_output()
        print("[aim] 闭环解锁（%s）：PULSE_PER_PX_*/DIR_SIGN_* 已按实测值写入，开始跟踪" % reason)
        return True

    def reset(self):
        self.state = UNCALIBRATED if self.locked else BOOT
        self.fault_reason = None
        self._lost = self._settle = self._scan_pos = 0
        self._scan_dir = 1
        self._jump = {"yaw": 0, "pitch": 0}
        self.last_command_time = {"yaw": -1e9, "pitch": -1e9}
        self._guard_n = 0
        self._guard_min = None
        self._clear_output()
        return self.state

    def update(self, result, now=0.0):
        """输入视觉 result，返回 ``cmds``；无效结果按状态机处理（HOLD / SCAN）。"""
        now = float(now)
        self._pending_stops = set()
        if self.locked:
            valid = bool(result and result.get("aim_valid") and
                         result.get("dx") is not None and result.get("dy") is not None)
            dx = float(result["dx"]) if valid else None
            dy = float(result["dy"]) if valid else None
            self.state = UNCALIBRATED
            return self._emit(dx, dy, {"yaw": None, "pitch": None})
        if self.state == FAULT:
            return self._emit(None, None, {"yaw": None, "pitch": None})
        valid = bool(result and result.get("aim_valid") and
                     result.get("dx") is not None and result.get("dy") is not None)
        if not valid:
            return self._on_lost(now)
        dx, dy = float(result["dx"]), float(result["dy"])
        self._lost = 0
        self._scan_pos = 0
        deadband = self._deadband()
        if abs(dx) <= deadband and abs(dy) <= deadband:
            for axis in ("yaw", "pitch"):
                self._cancel_motion(axis, now)
            self._clear_output()
            self._guard_n, self._guard_min = 0, None
            self._settle += 1
            self.state = SETTLE
            return self._emit(dx, dy, {"yaw": None, "pitch": None})
        self._settle = 0
        # 发散看护：只在"确实要发指令"时检查（死区内已提前 return，不参与判断）。
        if self._guard_check(max(abs(dx), abs(dy))):
            for axis in ("yaw", "pitch"):
                self._cancel_motion(axis, now)
            return self._emit(dx, dy, {"yaw": None, "pitch": None})
        coarse_px = float(getattr(self.cfg, "COARSE_PX", 0.0))
        self.state = COARSE if (coarse_px > 0 and max(abs(dx), abs(dy)) > coarse_px) else FINE
        cmds = {"yaw": self._axis_command("yaw", dx, now),
                "pitch": self._axis_command("pitch", dy, now)}
        return self._emit(dx, dy, cmds)

    def status_line(self):
        def fmt(cmd):
            if not cmd:
                return "-"
            return ("CCW" if cmd[0] else "CW") + "/%dp" % cmd[1]
        return "[aim] state=%s dx=%s dy=%s yaw=%s pit=%s lost=%d" % (
            self.last.get("state", self.state), self.last.get("dx"), self.last.get("dy"),
            fmt(self.last.get("cmds", {}).get("yaw")),
            fmt(self.last.get("cmds", {}).get("pitch")), self._lost)

    # ------------------------------------------------------------- 内部实现
    def _emit(self, dx, dy, cmds):
        out = {"state": self.state, "dx": dx, "dy": dy, "cmds": cmds,
               "stops": sorted(self._pending_stops),
               "filtered_pulses": dict(self._output)}
        self.last = out
        return out

    def _deadband(self):
        return float(getattr(self.cfg, "DEADBAND_PX", 2.0))

    def _clear_output(self, axis=None):
        """清除滤波记忆，但保留尚未结束的机械运动等待时间。"""
        if axis is None:
            self._output = {"yaw": 0.0, "pitch": 0.0}
            self._output_time = {"yaw": None, "pitch": None}
            self._error_sign = {"yaw": 0, "pitch": 0}
        else:
            self._output[axis] = 0.0
            self._output_time[axis] = None
            self._error_sign[axis] = 0

    def _cancel_motion(self, axis, now):
        if now < self._motion_until[axis]:
            self._pending_stops.add(axis)
            # 停止后留出一帧间隔再反向，避免同帧 stop/move 贴在一起。
            self.last_command_time[axis] = now
            self._motion_until[axis] = now

    def _filter_output(self, axis, requested, now):
        """时间低通 + 增长限速；缩小运动量立即生效，避免滤波拖尾越过靶心。"""
        tau = max(0.0, float(getattr(self.cfg, "AIM_OUTPUT_LPF_TAU_S", 0.0)))
        slew = max(0.0, float(getattr(self.cfg, "AIM_OUTPUT_SLEW_PPS", 0.0)))
        previous = self._output[axis]
        last = self._output_time[axis]
        max_dt = max(0.001, float(getattr(self.cfg, "COMMAND_INTERVAL_S", 0.1)))
        if last is None:
            dt = max_dt
        else:
            # 以实际控制周期计算，不再把 0.5s 的间隔压成 0.1s 导致低帧率下永远小步爬行。
            dt = min(max(0.25, max_dt), max(0.0, now - last))
        self._output_time[axis] = now
        if abs(requested) <= abs(previous):
            value = requested
        else:
            alpha = 1.0 - math.exp(-dt / tau) if tau > 0 else 1.0
            magnitude = abs(previous) + alpha * (abs(requested) - abs(previous))
            if slew > 0:
                magnitude = min(magnitude, abs(previous) + slew * dt)
            start = float(getattr(self.cfg, "AIM_START_PULSES", 0.0))
            if last is None and start > 0:
                magnitude = min(magnitude, start)
            value = math.copysign(magnitude, requested)
        self._output[axis] = value
        return value

    def _guard_check(self, err_abs):
        """发散看护：判断"一直在发指令却始终不收敛"，是则置 FAULT。

        判据：观察窗内记录 |误差| 的**最小值**；窗口满之后若当前值涨到 `最小值 × DIVERGE_RATIO`
        以上，说明"追了这么久反而更偏"，与收敛的预期相反 —— 多半是 `DIR_SIGN_*` 反了。
        误差每次创出新低就重新计数，因此正常波动/正常收敛不会被误判。
        """
        if not self._guard_on:
            return False
        if self._guard_min is None or err_abs < self._guard_min:
            self._guard_min = err_abs
            self._guard_n = 0
            return False
        self._guard_n += 1
        if self._guard_n < self._guard_frames:
            return False
        if self._guard_min > 0 and err_abs >= self._guard_min * self._guard_ratio:
            self.fault("diverge(|误差| 从 %.1fpx 涨到 %.1fpx，趋势不收敛 -> 检查 DIR_SIGN_*)"
                       % (self._guard_min, err_abs))
            return True
        self._guard_n = 0
        return False

    def _on_lost(self, now):
        self._clear_output()
        self._guard_n, self._guard_min = 0, None
        self._jump = {"yaw": 0, "pitch": 0}
        for axis in ("yaw", "pitch"):
            self._cancel_motion(axis, now)
        self._lost += 1
        self._settle = 0
        scan_on = bool(getattr(self.cfg, "ENABLE_SCAN", False))
        if scan_on and self._lost >= int(getattr(self.cfg, "LOST_FRAMES", 15)):
            self.state = SCAN
            cmd = self._scan_command("yaw", now)
            return self._emit(None, None, {"yaw": cmd, "pitch": None})
        self.state = HOLD
        return self._emit(None, None, {"yaw": None, "pitch": None})

    def _gain(self, axis, error):
        cfg = self.cfg
        fine = float(getattr(cfg, "GAIN_YAW" if axis == "yaw" else "GAIN_PITCH", 0.5))
        coarse_px = float(getattr(cfg, "COARSE_PX", 0.0))
        if coarse_px > 0 and abs(error) > coarse_px:
            return float(getattr(cfg, "GAIN_COARSE", min(1.0, fine * 1.8)))
        return fine

    def _pulse_per_px(self, axis):
        """像素 -> 脉冲 的换算系数，也是"调节激烈程度"的总闸门。

        取值优先级（现场调参入口见 `config.py` 的「★ 追踪手感」）：
          ① `DEG_PER_PX_x × PULSES_PER_DEG_x` —— **手感宏，推荐**（每像素多少度 × 每度多少脉冲）；
          ② 回退 `PULSE_PER_PX_x` —— CALIBRATE_AXIS 实测写回的值 / 旧口径。

        标定实测出的脉冲/像素可反算回宏：`DEG_PER_PX = PULSE_PER_PX / PULSES_PER_DEG`。
        """
        cfg = self.cfg
        upper = axis.upper()
        deg = float(getattr(cfg, "DEG_PER_PX_%s" % upper, 0.0) or 0.0)
        ppd = float(getattr(cfg, "PULSES_PER_DEG_%s" % upper, 0.0) or 0.0)
        if deg > 0 and ppd > 0:
            return deg * ppd
        return float(getattr(cfg, "PULSE_PER_PX_%s" % upper, 3.67))

    def _rpm(self):
        if self.state == COARSE:
            return int(getattr(self.cfg, "RPM_COARSE", 150))
        return int(getattr(self.cfg, "RPM_FINE", 60))

    def _axis_command(self, axis, error, now):
        cfg = self.cfg
        if abs(error) <= self._deadband():
            self._cancel_motion(axis, now)
            self._clear_output(axis)
            self._jump[axis] = 0
            return None
        direction = 1 if error > 0 else -1
        if self._error_sign[axis] and direction != self._error_sign[axis]:
            self._cancel_motion(axis, now)
            self._clear_output(axis)
        self._error_sign[axis] = direction
        # 离群拒收：单帧超大误差先当误检丢弃，连续若干帧才采信（防甩头）。
        jump = float(getattr(cfg, "JUMP_REJECT_PX", 0.0))
        if jump > 0 and abs(error) > jump:
            self._jump[axis] += 1
            if self._jump[axis] < int(getattr(cfg, "JUMP_CONFIRM_FRAMES", 3)):
                self._clear_output(axis)
                return None
        else:
            self._jump[axis] = 0
        # 等待只比较时刻，不 sleep/循环。只有准备发送追踪命令时才运行低通/控制律。
        interval = float(getattr(cfg, "COMMAND_INTERVAL_S", 0.08))
        if now - self.last_command_time[axis] < interval or now < self._motion_until[axis]:
            return None
        sign = float(getattr(cfg, "DIR_SIGN_YAW" if axis == "yaw" else "DIR_SIGN_PITCH", 1))
        max_pulses = int(getattr(cfg, "MAX_STEP_PULSES", 360))
        if self.controller is not None:
            output = self.controller(axis, float(error), now)
            if output is None or float(output) == 0:
                self._clear_output(axis)
                return None
            output = float(output)
        else:
            ppp = self._pulse_per_px(axis)
            output = error * ppp * self._gain(axis, error)
        output = max(-max_pulses, min(max_pulses, output))
        if self._output[axis] * output < 0:
            self._cancel_motion(axis, now)
            self._clear_output(axis)
        output = self._filter_output(axis, output, now)
        if axis in self._pending_stops:
            return None
        pulses = min(max_pulses, int(round(abs(output))))
        if pulses <= 0:
            return None
        ccw = output * sign > 0
        rpm = self._rpm()
        if bool(getattr(cfg, "AIM_WAIT_MOTION", False)):
            acc = int(getattr(cfg, "ACC_TRACK", getattr(cfg, "ACC_DEFAULT", 0)))
            duration = estimate_move_seconds(pulses, rpm, acc,
                                            getattr(cfg, "MOTOR_PULSES_PER_REV", 3200))
            margin = max(0.0, float(getattr(cfg, "AIM_MOTION_MARGIN_S", 0.03)))
            self._motion_until[axis] = now + duration + margin
        self.last_command_time[axis] = now
        return ccw, pulses, rpm

    def _scan_command(self, axis, now):
        cfg = self.cfg
        if now < self._motion_until[axis]:
            return None
        if now - self.last_command_time[axis] < float(getattr(cfg, "SCAN_STEP_INTERVAL_S", 0.15)):
            return None
        ppd = float(getattr(cfg, "YAW_PULSES_PER_DEG", 0.0))
        if ppd > 0:
            step = max(1, int(round(float(getattr(cfg, "SCAN_STEP_DEG", 1.5)) * ppd)))
            limit = max(step, int(round(float(getattr(cfg, "SCAN_LIMIT_DEG", 15.0)) * ppd)))
        else:
            step = max(1, int(getattr(cfg, "SCAN_STEP_PULSES", 40)))
            limit = max(step, int(getattr(cfg, "SCAN_LIMIT_PULSES", 400)))
        if self._scan_pos >= limit:
            self._scan_dir = -1
        elif self._scan_pos <= -limit:
            self._scan_dir = 1
        sign = float(getattr(cfg, "DIR_SIGN_YAW", 1))
        ccw = (self._scan_dir * sign) > 0
        self._scan_pos += self._scan_dir * step
        self.last_command_time[axis] = now
        return ccw, step, int(getattr(cfg, "SCAN_RPM", 60))
