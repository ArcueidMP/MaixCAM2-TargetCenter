"""按钮启动的非阻塞双轴寻靶：相对按钮时姿态，各轴覆盖正负 45°。

这里记录的是已请求的相对位置，不是编码器实测角度。只允许使用实测
脉冲/度或已知减速比；途中识别到黑框、取消或故障时立即停车，不猜测
剩余脉冲，也不自动回零。UART 与本状态机均由控制线程独占。
"""
import math

from .stepper_link import estimate_move_seconds


class SearchSweep:
    """每帧/idle 调用 update；构造不发运动，终态后不会再发运动指令。

    调度器启动前应停止其它运动。ACQUIRED/DONE/CANCELLED 的 stopped_at
    是停机帧成功写出的时刻：切回追踪必须等待该时刻之后采集的新画面。
    stop 写失败则为 FAULT，不能将 stop_attempted_at 当成成功停机。
    """

    TERMINAL = ("DONE", "ACQUIRED", "CANCELLED", "UNCALIBRATED", "FAULT")

    def __init__(self, cfg, link, now=0.0):
        self.cfg, self.link = cfg, link
        self.state, self.reason = "RUNNING", None
        self.index, self.sequence = 0, []
        self.active_axis = None
        self.stopped_at = self.stop_attempted_at = None
        self.position_known = True
        self.commanded_pulses = {"yaw": 0, "pitch": 0}
        self.pulses_per_degree = {}
        self.limit_pulses = {}
        self.started_at = self.deadline = self._last_now = float(now)
        self._last_capture = -math.inf
        self._prepare()

    @property
    def done(self):
        return self.state in self.TERMINAL

    @property
    def positions_deg(self):
        """累计请求端点；中途停车后不代表实际姿态（见 position_known）。"""
        return {axis: self.commanded_pulses[axis] / ppd
                for axis, ppd in self.pulses_per_degree.items()}

    def _prepare(self):
        try:
            if not math.isfinite(self.started_at):
                raise ValueError("invalid_timestamp")
            self.rpm = int(getattr(self.cfg, "SEARCH_RPM", 60))
            self.acc = int(getattr(self.cfg, "SEARCH_ACC", 60))
            self.settle_s = float(getattr(self.cfg, "SEARCH_SETTLE_S", 0.10))
            self.timeout_s = float(getattr(self.cfg, "SEARCH_TIMEOUT_S", 60.0))
            self.max_age_s = float(getattr(self.cfg, "RESULT_MAX_AGE_S", 0.20))
            self.per_rev = float(getattr(self.cfg, "MOTOR_PULSES_PER_REV", 3200))
            numbers = (self.settle_s, self.timeout_s, self.max_age_s, self.per_rev)
            if not all(math.isfinite(value) for value in numbers):
                raise ValueError("invalid_config")
            if (not 1 <= self.rpm <= 5000 or not 0 <= self.acc <= 255 or
                    self.settle_s < 0 or self.timeout_s <= 0 or
                    self.max_age_s <= 0 or self.per_rev <= 0):
                raise ValueError("invalid_config")
            ratio = float(getattr(self.cfg, "GIMBAL_GEAR_RATIO", 0.0))
            for axis in ("yaw", "pitch"):
                ppd = float(getattr(self.cfg, axis.upper() + "_PULSES_PER_DEG", 0.0))
                if ppd <= 0 and math.isfinite(ratio) and ratio > 0:
                    ppd = self.per_rev * ratio / 360.0
                if not math.isfinite(ppd) or ppd <= 0:
                    self.state, self.reason = "UNCALIBRATED", "pulse_per_degree_not_calibrated"
                    return
                angle = float(getattr(self.cfg, "SEARCH_" + axis.upper() + "_DEG", 45.0))
                if not math.isfinite(angle) or not 0 < angle <= 45.0:
                    raise ValueError("search_angle_out_of_range")
                # 端点先量化、再作差；不会因逐段取整把累计端点推到 ±45° 之外。
                limit = int(math.floor(angle * ppd))
                if limit <= 0 or 2 * limit > 0xFFFFFFFF:
                    raise ValueError("invalid_pulse_count")
                self.pulses_per_degree[axis] = ppd
                self.limit_pulses[axis] = limit
                self.sequence.extend(((axis, limit), (axis, -limit), (axis, 0)))
            self.timeout_at = self.started_at + self.timeout_s
        except (TypeError, ValueError, OverflowError) as exc:
            self.state, self.reason = "FAULT", str(exc) or "invalid_config"

    def _target_is_fresh(self, result, now):
        if not result or not result.get("valid") or result.get("held") or result.get("predicted"):
            return False
        try:
            captured = float(result["captured_at"])
            coordinates = (float(result["u"]), float(result["v"]))
        except (KeyError, TypeError, ValueError, OverflowError):
            return False
        if (not all(math.isfinite(value) for value in (captured,) + coordinates) or
                captured < self.started_at or captured <= self._last_capture or
                not 0 <= now - captured <= self.max_age_s):
            return False
        self._last_capture = captured
        return True

    def _finish(self, state, reason, now):
        moving = self.active_axis is not None and now < self.deadline
        self.position_known = self.position_known and not moving
        self.stop_attempted_at = now
        try:
            accepted = self.link.stop(now=now)
        except Exception:
            accepted = False
        self.active_axis = None
        if accepted is False:
            self.state, self.reason = "FAULT", "stop_failed:%s" % reason
            self.position_known = False
        else:
            self.state, self.reason, self.stopped_at = state, reason, now
        return self.state

    def cancel(self, now):
        """按钮再次点击取消；禁止按已请求端点倒算回起点。"""
        if self.done:
            return self.state
        try:
            now = float(now)
        except (TypeError, ValueError, OverflowError):
            now = self._last_now
        if not math.isfinite(now):
            now = self._last_now
        now = max(now, self._last_now)
        return self._finish("CANCELLED", "cancelled", now)

    def fail(self, reason, now):
        """控制器发现链路故障时调用；失败后的 update 不会再下发运动。"""
        if self.done:
            return self.state
        try:
            now = float(now)
        except (TypeError, ValueError, OverflowError):
            now = self._last_now
        if not math.isfinite(now):
            now = self._last_now
        now = max(now, self._last_now)
        return self._finish("FAULT", str(reason), now)

    def update(self, now, result=None):
        """最多发一段位置指令；视觉识别在运动未完成时也能立即停车。"""
        if self.done:
            return self.state
        try:
            now = float(now)
        except (TypeError, ValueError, OverflowError):
            return self.fail("invalid_timestamp", self._last_now)
        if not math.isfinite(now) or now < self._last_now:
            return self.fail("invalid_timestamp", self._last_now)
        self._last_now = now
        fault = getattr(self.link, "fault_reason", None)
        if fault:
            return self.fail("link:%s" % fault, now)
        if self._target_is_fresh(result, now):
            return self._finish("ACQUIRED", "black_frame_found", now)
        if now >= self.timeout_at:
            return self._finish("DONE", "timeout", now)
        if now < self.deadline:
            return self.state
        self.active_axis = None
        if self.index >= len(self.sequence):
            return self._finish("DONE", "sweep_complete", now)
        axis, target = self.sequence[self.index]
        delta = target - self.commanded_pulses[axis]
        pulses = abs(delta)
        positive_ccw = bool(getattr(self.cfg, axis.upper() + "_POSITIVE_CCW", True))
        ccw = positive_ccw if delta > 0 else not positive_ccw
        try:
            duration = estimate_move_seconds(pulses, self.rpm, self.acc, self.per_rev)
            accepted = self.link.move(axis, ccw, pulses, self.rpm, self.acc, now=now)
        except Exception:
            accepted = False
        if accepted is False:
            self.position_known = False
            return self.fail("uart_write_failed:%s" % axis, now)
        self.commanded_pulses[axis] = target
        self.active_axis = axis
        self.deadline = now + duration + self.settle_s
        self.index += 1
        return self.state
