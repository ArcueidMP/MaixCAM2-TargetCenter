"""基于像素误差的两轴增量控制器 + 状态机。

对照 `MaixCAM2串口直控云台需求文档.md` §7 实现：

- 闭环输入 = ``result["dx"]/["dy"]``（靶心 − 光斑，见 ``laser_spot.deviation``）；
- **相对位置**命令（0xFD、mode=0），单次脉冲限幅；两轴独立渐变增益和速度；
- 可等待加减速运动完成与运动后的新图像，避免重复使用运动前误差；
- 状态机 ``BOOT / COARSE / FINE / SETTLE / HOLD / SCAN / FAULT``；
- 三重保护：离群拒收（``JUMP_REJECT_PX``）、死区（``DEADBAND_PX``）、指令限频
  （``COMMAND_INTERVAL_S``）、单帧限幅（``MAX_STEP_PULSES``）。

本模块**保持纯 Python（不 import maix）**，可在 PC 上离线仿真与回归。
控制律可整体替换：``controller(axis, error_px, now) -> 带符号脉冲数``，传 None 用内置比例律。
"""

import math

from .stepper_link import estimate_move_seconds

# 状态语义（给日志和上层判断用）
BOOT = "BOOT"        # 上电，等第一帧合法误差
COARSE = "COARSE"    # 大误差，大步快速逼近
FINE = "FINE"        # 小误差，小步逼近
SETTLE = "SETTLE"    # 进入死区，停止发指令
HOLD = "HOLD"        # 视觉失效，保持不动
SCAN = "SCAN"        # 失锁后慢扫重捕获（需 ENABLE_SCAN）
FAULT = "FAULT"      # 链路/机械故障，停机不再发指令


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
        self._busy_until = {"yaw": -1e9, "pitch": -1e9}
        self._capture_after = {"yaw": -1e9, "pitch": -1e9}
        self._motion_dir = {"yaw": 0, "pitch": 0}
        self._last_measurement_time = None
        self._stop_axes = []
        self._source = "vision"

    # ------------------------------------------------------------- 外部接口
    @property
    def capture_after(self):
        """最近一次任意轴停止后的采图门槛，供视觉/IMU参考共同使用。"""
        return max(self._capture_after.values())

    def set_controller(self, controller):
        """设置后续 PID/前馈控制器；传入 None 恢复内置比例律。"""
        self.controller = controller

    def fault(self, reason):
        """进入 FAULT：不再发任何指令，由上层决定是否复位。"""
        self.state = FAULT
        self.fault_reason = str(reason)
        print("[aim] FAULT: %s" % self.fault_reason)
        return self.state

    def reset(self):
        self.state = BOOT
        self.fault_reason = None
        self._lost = self._settle = self._scan_pos = 0
        self._scan_dir = 1
        self._jump = {"yaw": 0, "pitch": 0}
        self.last_command_time = {"yaw": -1e9, "pitch": -1e9}
        self.motion_stopped()
        return self.state

    def motion_stopped(self, axis=None, now=None):
        """上层已停机时清除排程和历史；提供停机时间可拒收停机前的图像。"""
        axes = ("yaw", "pitch") if axis is None else (axis,)
        for name in axes:
            if name not in self._busy_until:
                raise ValueError("axis must be yaw or pitch")
            self._busy_until[name] = -1e9
            self._capture_after[name] = float(now) if now is not None and math.isfinite(float(now)) else -1e9
            self._motion_dir[name] = 0
            self._jump[name] = 0
        self._last_measurement_time = None

    def _request_stop(self, axis, now):
        if axis not in self._stop_axes:
            self._stop_axes.append(axis)
        captured_at = self._last_measurement_time
        self.motion_stopped(axis, now=now)
        self._last_measurement_time = captured_at

    def update(self, result, now=0.0):
        """输入视觉 result，返回 ``cmds``；无效结果按状态机处理（HOLD / SCAN）。"""
        now = float(now)
        self._stop_axes = []
        if self.state == FAULT:
            return self._emit(None, None, {"yaw": None, "pitch": None})
        valid = bool(result and result.get("aim_valid") and
                     result.get("dx") is not None and result.get("dy") is not None)
        if not valid or not math.isfinite(now):
            return self._on_lost(now)
        try:
            dx, dy = float(result["dx"]), float(result["dy"])
            captured_at = float(result.get("captured_at", now))
        except (TypeError, ValueError, OverflowError):
            return self._on_lost(now)
        if not all(math.isfinite(v) for v in (dx, dy, captured_at)) or captured_at > now + 1e-6:
            return self._on_lost(now)
        if self._last_measurement_time is not None:
            if captured_at <= self._last_measurement_time:
                # 重复或倒序图像不是新观测，不能再追加相对位置动作。
                return self._emit(dx, dy, {"yaw": None, "pitch": None})
            gap = float(getattr(self.cfg, "AIM_HISTORY_MAX_GAP_S", 0.0))
            if gap > 0 and captured_at - self._last_measurement_time > gap:
                self._jump = {"yaw": 0, "pitch": 0}
        self._last_measurement_time = captured_at
        self._source = "imu" if (result.get("source") == "imu" and
                                    bool(getattr(self.cfg, "ENABLE_IMU_FEEDFORWARD", False))) else "vision"
        self._lost = 0
        self._scan_pos = 0
        deadband = self._deadband()
        if abs(dx) <= deadband and abs(dy) <= deadband:
            self._settle += 1
            self.state = SETTLE
            for axis in ("yaw", "pitch"):
                self._stop_if_needed(axis, 0.0, now, captured_at)
            return self._emit(dx, dy, {"yaw": None, "pitch": None})
        self._settle = 0
        coarse_px = float(getattr(self.cfg, "COARSE_PX", 0.0))
        self.state = COARSE if (coarse_px > 0 and max(abs(dx), abs(dy)) > coarse_px) else FINE
        cmds = {"yaw": self._axis_command("yaw", dx, now, captured_at),
                "pitch": self._axis_command("pitch", dy, now, captured_at)}
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
               "stop_axes": list(self._stop_axes)}
        self.last = out
        return out

    def _deadband(self):
        return float(getattr(self.cfg, "DEADBAND_PX", 2.0))

    def _on_lost(self, now):
        for axis in ("yaw", "pitch"):
            if self._motion_dir[axis] and (not math.isfinite(now) or now < self._busy_until[axis]):
                self._request_stop(axis, now)
        self.motion_stopped(now=now)
        self._last_measurement_time = None
        self._jump = {"yaw": 0, "pitch": 0}
        self._lost += 1
        self._settle = 0
        scan_on = bool(getattr(self.cfg, "ENABLE_SCAN", False))
        if scan_on and math.isfinite(now) and self._lost >= int(getattr(self.cfg, "LOST_FRAMES", 15)):
            self.state = SCAN
            cmd = self._scan_command("yaw", now)
            return self._emit(None, None, {"yaw": cmd, "pitch": None})
        self.state = HOLD
        return self._emit(None, None, {"yaw": None, "pitch": None})

    def _gain(self, axis, error):
        cfg = self.cfg
        if self._source == "imu":
            return max(0.0, min(1.0, float(getattr(cfg, "IMU_GAIN", 0.75))))
        fine = float(getattr(cfg, "GAIN_YAW" if axis == "yaw" else "GAIN_PITCH", 0.5))
        coarse_px = float(getattr(cfg, "COARSE_PX", 0.0))
        if bool(getattr(cfg, "AIM_PROGRESSIVE", False)) and coarse_px > self._deadband():
            coarse = float(getattr(cfg, "GAIN_COARSE", min(1.0, fine * 1.8)))
            mix = max(0.0, min(1.0, (abs(error) - self._deadband()) / (coarse_px - self._deadband())))
            return max(0.0, min(1.0, fine + mix * (coarse - fine)))
        if coarse_px > 0 and abs(error) > coarse_px:
            return float(getattr(cfg, "GAIN_COARSE", min(1.0, fine * 1.8)))
        return fine

    def _rpm(self, error):
        if self._source == "imu" and abs(error) > float(getattr(self.cfg, "COARSE_PX", 24)):
            return max(1, min(5000, int(getattr(self.cfg, "IMU_RPM", 180))))
        coarse_px = float(getattr(self.cfg, "COARSE_PX", 0.0))
        fine = int(getattr(self.cfg, "RPM_FINE", 300))
        coarse = int(getattr(self.cfg, "RPM_COARSE", 1200))
        if bool(getattr(self.cfg, "AIM_PROGRESSIVE", False)) and coarse_px > self._deadband():
            mix = max(0.0, min(1.0, (abs(error) - self._deadband()) / (coarse_px - self._deadband())))
            return max(1, int(round(fine + mix * (coarse - fine))))
        if coarse_px > 0 and abs(error) > coarse_px:
            return int(getattr(self.cfg, "RPM_COARSE", 1200))
        return fine

    def _stop_if_needed(self, axis, error, now, captured_at):
        direction = self._motion_dir[axis]
        if not direction or captured_at <= self.last_command_time[axis]:
            return False
        if now >= self._busy_until[axis]:
            return False
        in_band = abs(error) <= self._deadband()
        reversed_error = error * direction < 0
        if in_band or reversed_error:
            self._request_stop(axis, now)
            return True
        return False

    def _axis_command(self, axis, error, now, captured_at):
        cfg = self.cfg
        # 离群拒收：单帧超大误差先当误检丢弃，连续若干帧才采信（防甩头）。
        jump = float(getattr(cfg, "JUMP_REJECT_PX", 0.0))
        if self._source != "imu" and jump > 0 and abs(error) > jump:
            self._jump[axis] += 1
            if self._jump[axis] < int(getattr(cfg, "JUMP_CONFIRM_FRAMES", 3)):
                return None
        else:
            self._jump[axis] = 0
        if self._stop_if_needed(axis, error, now, captured_at):
            return None
        if abs(error) <= self._deadband():
            return None
        interval = float(getattr(cfg, "COMMAND_INTERVAL_S", 0.08))
        if now - self.last_command_time[axis] < interval:
            return None
        if bool(getattr(cfg, "AIM_WAIT_MOTION", False)):
            if (now < self._busy_until[axis] or captured_at <= self._busy_until[axis]
                    or captured_at <= self._capture_after[axis]):
                return None
        sign = float(getattr(cfg, "DIR_SIGN_YAW" if axis == "yaw" else "DIR_SIGN_PITCH", 1))
        max_pulses = int(getattr(cfg, "MAX_STEP_PULSES", 360))
        if self._source == "imu":
            ppd = float(getattr(cfg, axis.upper() + "_PULSES_PER_DEG", 0.0))
            if ppd <= 0:
                ppd = float(getattr(cfg, "GIMBAL_GEAR_RATIO", 0.0)) * float(
                    getattr(cfg, "MOTOR_PULSES_PER_REV", 3200)) / 360.0
            step_deg = float(getattr(cfg, "IMU_MAX_STEP_DEG", 30.0))
            if not all(math.isfinite(v) for v in (ppd, step_deg)) or ppd <= 0 or not 0 < step_deg <= 45:
                return None
            max_pulses = min(0xFFFFFFFF, int(ppd * step_deg))
        if self.controller is not None:
            output = self.controller(axis, float(error), now)
            if output is None or not math.isfinite(float(output)) or float(output) == 0:
                return None
            pulses = min(max_pulses, max(1, int(round(abs(float(output))))))
            ccw = float(output) * sign > 0
        else:
            ppp = float(getattr(cfg, "PULSE_PER_PX_YAW" if axis == "yaw"
                                else "PULSE_PER_PX_PITCH", 3.67))
            pulses = min(max_pulses, max(1, int(round(abs(error) * ppp * self._gain(axis, error)))))
            ccw = error * sign > 0
        if max_pulses <= 0:
            return None
        rpm = self._rpm(error)
        self.last_command_time[axis] = now
        duration = estimate_move_seconds(pulses, rpm,
                                       getattr(cfg, "IMU_ACC", 80) if self._source == "imu" else
                                       getattr(cfg, "ACC_TRACK", getattr(cfg, "ACC_DEFAULT", 0)),
                                       getattr(cfg, "MOTOR_PULSES_PER_REV", 3200))
        self._busy_until[axis] = now + duration + max(0.0, float(getattr(cfg, "AIM_MOTION_MARGIN_S", 0.0)))
        self._motion_dir[axis] = 1 if (1 if ccw else -1) * sign > 0 else -1
        return ccw, pulses, rpm

    def _scan_command(self, axis, now):
        cfg = self.cfg
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
