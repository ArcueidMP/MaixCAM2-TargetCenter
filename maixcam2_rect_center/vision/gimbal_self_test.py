"""上电云台小幅自检：yaw ±15°，pitch ±30°，最后回到启动姿态。"""


class GimbalSelfTest:
    """非阻塞自检状态机。角度通过实测的电机脉冲/输出轴度数换算。"""

    def __init__(self, cfg, link, now=0.0):
        self.cfg = cfg
        self.link = link
        self.state = "IDLE"
        self.index = 0
        self.deadline = float(now)
        self.sequence = []
        self.reason = None
        self._prepare()

    def _prepare(self):
        if not bool(getattr(self.cfg, "GIMBAL_SELF_TEST", True)):
            self.state, self.reason = "DONE", "disabled"
            return
        yaw_ppd = float(getattr(self.cfg, "YAW_PULSES_PER_DEG", 0.0))
        pitch_ppd = float(getattr(self.cfg, "PITCH_PULSES_PER_DEG", 0.0))
        # 未做"脉冲/度"标定时，允许用已知减速比推导：脉冲/度 = 每圈脉冲 × 减速比 / 360。
        # 减速比未知（0）则保持跳过 —— 不用错误比例去撞限位。
        ratio = float(getattr(self.cfg, "GIMBAL_GEAR_RATIO", 0.0))
        if ratio > 0:
            per_rev = float(getattr(self.cfg, "MOTOR_PULSES_PER_REV", 3200))
            derived = per_rev * ratio / 360.0
            if yaw_ppd <= 0:
                yaw_ppd = derived
            if pitch_ppd <= 0:
                pitch_ppd = derived
        if yaw_ppd <= 0 or pitch_ppd <= 0:
            self.state, self.reason = "SKIPPED", "pulse_per_degree_not_calibrated"
            print("[self-test] skipped: set YAW_PULSES_PER_DEG / PITCH_PULSES_PER_DEG")
            print("[self-test]           或用 GIMBAL_GEAR_RATIO 提供减速比，或先用 CALIBRATE_AXIS 实测")
            return
        yaw_angle = abs(float(getattr(self.cfg, "SELF_TEST_YAW_DEG", 15.0)))
        pitch_angle = abs(float(getattr(self.cfg, "SELF_TEST_PITCH_DEG", 30.0)))
        yaw_ccw_positive = bool(getattr(self.cfg, "YAW_POSITIVE_CCW", False))
        pitch_ccw_positive = bool(getattr(self.cfg, "PITCH_POSITIVE_CCW", False))
        self.sequence = [
            ("yaw", yaw_ccw_positive, yaw_angle * yaw_ppd),
            ("yaw", not yaw_ccw_positive, 2 * yaw_angle * yaw_ppd),
            ("yaw", yaw_ccw_positive, yaw_angle * yaw_ppd),
            ("pitch", pitch_ccw_positive, pitch_angle * pitch_ppd),
            ("pitch", not pitch_ccw_positive, 2 * pitch_angle * pitch_ppd),
            ("pitch", pitch_ccw_positive, pitch_angle * pitch_ppd),
        ]
        self.state = "RUNNING"

    @property
    def done(self):
        return self.state in ("DONE", "SKIPPED", "FAULT")

    def update(self, now):
        """每帧调用；到达估算运动时间后发下一段，不阻塞视觉主循环。"""
        if self.done:
            return self.state
        now = float(now)
        if now < self.deadline:
            return self.state
        if self.index >= len(self.sequence):
            self.state = "DONE"
            print("[self-test] complete; returned to startup pose")
            return self.state
        axis, ccw, raw_pulses = self.sequence[self.index]
        pulses = int(round(raw_pulses))
        if pulses <= 0:
            self.state, self.reason = "FAULT", "invalid_pulse_count"
            self.link.stop(axis)
            return self.state
        rpm = int(getattr(self.cfg, "SELF_TEST_RPM", 900))
        if not self.link.move(axis, ccw, pulses, rpm,
                              int(getattr(self.cfg, "ACC_DEFAULT", 0))):
            self.state, self.reason = "FAULT", "uart_write_failed"
            self.link.stop(axis)
            print("[self-test] fault: %s command failed" % axis)
            return self.state
        pulses_per_rev = float(getattr(self.cfg, "MOTOR_PULSES_PER_REV", 3200))
        move_s = pulses / max(1.0, pulses_per_rev * max(1, rpm) / 60.0)
        settle_s = float(getattr(self.cfg, "SELF_TEST_SETTLE_S", 0.35))
        self.deadline = now + move_s + settle_s
        self.index += 1
        print("[self-test] %s %s %dp rpm=%d step=%d/%d" %
              (axis, "CCW" if ccw else "CW", pulses, rpm,
               self.index, len(self.sequence)))
        return self.state


def probe_imu():
    """尽力实例化 MaixPy IMU 并读取一次样本；不影响主程序运行。"""
    try:
        import maix
        imu = getattr(maix, "imu", None)
        if imu is None:
            import importlib
            imu = importlib.import_module("maix.imu")
        names = [name for name in dir(imu) if not name.startswith("_")]
        device = None
        sample = None
        errors = []
        for class_name in ("IMU", "Imu", "imu"):
            constructor = getattr(imu, class_name, None)
            if not callable(constructor):
                continue
            try:
                device = constructor()
                break
            except Exception as exc:
                errors.append("%s(): %s" % (class_name, exc))
        if device is None:
            return {"available": False, "module": imu, "names": names,
                    "error": "; ".join(errors) or "module has no recognized IMU constructor"}
        for method_name in ("read", "get_data", "get_accel_gyro", "get_gyro"):
            method = getattr(device, method_name, None)
            if callable(method):
                try:
                    sample = method()
                    return {"available": True, "module": imu, "names": names,
                            "device": device, "method": method_name,
                            "sample": sample, "error": None}
                except Exception as exc:
                    errors.append("%s(): %s" % (method_name, exc))
        return {"available": False, "module": imu, "names": names,
                "device": device, "error": "; ".join(errors) or
                "IMU object has no recognized read method"}
    except Exception as exc:
        return {"available": False, "module": None, "names": [], "error": str(exc)}
