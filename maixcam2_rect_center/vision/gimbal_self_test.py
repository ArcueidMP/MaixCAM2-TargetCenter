"""上电云台小幅自检：**pitch 先上转再下转、yaw 先左转再右转**，最后回到启动姿态。

默认走**固定脉冲**（``SELF_TEST_PULSES``）——**不需要看到靶子、也不依赖脉冲/度标定**，
上电即可验证"串口通 + 两轴机械行程"；置 0 才回退到按角度换算的精细自检。

两条设计红线（2026-10-09 修复"U2 只使能、不自校"时定下）：

1. **只对链路真的打开的轴做自检** —— 一条轴不通不该拖累另一条；
2. **单轴写失败只跳过该轴剩余步骤**，另一轴继续走完。旧实现遇到第一条 yaw 指令
   写失败就整机 FAULT，导致 pitch 一步都收不到指令（现场表现就是"只使能、不动"）。

第三条（2026-10-09 现场反馈"自检把电机抖得卡顿"后定下）：**自检运动必须慢 + 带曲线**。
位置模式命令的加速度档位 `acc=0` 表示"不使用曲线加减速、直接以设定速度启停"，
对云台这种小惯量机构就是硬启硬停；`acc≠0` 时按 ZDT 官方公式每 `(256-acc)*50us`
变化 1 RPM，**acc 越小越柔和**。见 `estimate_move_seconds()`。
"""


def estimate_move_seconds(pulses, rpm, acc, pulses_per_rev=3200.0):
    """估算一段位置命令的运动时间（秒），供自检排程使用（纯函数，可单测）。

    模型（ZDT Emm_V5.0 官方说明）：
      · `acc == 0`：不使用曲线加减速，直接以设定速度运行 → 只有匀速段；
      · `acc != 0`：曲线加减速，速度变化率 a = 1 / ((256 - acc) * 50us) [RPM/s]。
    行程不足时自动退化为**三角速度曲线**（实际峰值低于设定速度），此时按
    `2 * V_peak / a` 计算，不会高估成梯形时间。
    """
    pulses = max(0.0, float(pulses))
    rpm = max(1e-6, float(rpm))
    per_rev = max(1.0, float(pulses_per_rev))
    rate = rpm * per_rev / 60.0                     # 脉冲/秒（匀速段）
    cruise_s = pulses / rate
    acc = int(acc)
    if acc <= 0:
        return cruise_s
    # 秒/RPM：每 (256-acc)*50us 变化 1 RPM
    k = (256 - max(0, min(255, acc))) * 50e-6
    t_ramp = rpm * k                                # 从 0 加到设定速度所需时间
    s_ramp = 0.5 * rpm * t_ramp * per_rev / 60.0    # 加速段走过的脉冲（平均速度 = rpm/2）
    if 2.0 * s_ramp <= pulses:                      # 能加到设定速度 -> 梯形
        return 2.0 * t_ramp + (pulses - 2.0 * s_ramp) / rate
    v_peak = (pulses * 60.0 / (k * per_rev)) ** 0.5  # 三角：峰值转速（RPM）
    return 2.0 * v_peak * k


class GimbalSelfTest:
    """非阻塞自检状态机。角度通过实测的电机脉冲/输出轴度数换算。"""

    def __init__(self, cfg, link, now=0.0):
        self.cfg = cfg
        self.link = link
        self.state = "IDLE"
        self.index = 0
        self.sent = 0
        self.total = 0
        self.deadline = float(now) + float(getattr(cfg, "SELF_TEST_START_DELAY_S", 0.2))
        # ★ 硬超时：到点**无条件**结束自检（"上电 3s 内跑完"的兜底），0 = 关闭。
        self._t0 = float(now)
        self.timeout_s = float(getattr(cfg, "SELF_TEST_TIMEOUT_S", 0.0) or 0.0)
        self.sequence = []
        self.reason = None
        self._failed = set()
        self._planned = set()
        # 可用轴：链路报告了就把不通的轴排除在自检之外；没有该接口（PC 假链路）则不筛。
        self.available = None
        probe = getattr(link, "available_axes", None)
        if callable(probe):
            try:
                self.available = set(probe())
            except Exception:
                self.available = None
        self._prepare()

    def _install(self, seq):
        """装入自检序列；剔除链路未打开的轴（一条轴不通不该拖累另一条）。"""
        missing = sorted({a for a, _c, _p in seq
                          if self.available is not None and a not in self.available})
        if missing:
            print("[self-test] 跳过链路未打开的轴: %s（不参与自检，另一轴照常）" % ",".join(missing))
            seq = [s for s in seq if s[0] not in missing]
        info_map = getattr(self.link, "axis_info", None)
        if isinstance(info_map, dict):
            for axis in ("yaw", "pitch"):
                info = info_map.get(axis) or {}
                if not info.get("opened"):
                    print("[self-test] %-5s 链路未就绪: dev=%s err=%s" %
                          (axis, info.get("device"), info.get("error") or "not opened"))
        if not seq:
            self.state, self.reason = "SKIPPED", "no_available_axis"
            print("[self-test] 两轴串口都没打开 -> 自检跳过（不会看到任何电机动作）")
            return
        self.sequence = list(seq)
        self.total = len(self.sequence)
        self._planned = {s[0] for s in self.sequence}
        self.state = "RUNNING"

    def _prepare(self):
        if not bool(getattr(self.cfg, "GIMBAL_SELF_TEST", True)):
            self.state, self.reason = "DONE", "disabled"
            return
        yaw_ccw_positive = bool(getattr(self.cfg, "YAW_POSITIVE_CCW", False))
        pitch_ccw_positive = bool(getattr(self.cfg, "PITCH_POSITIVE_CCW", False))
        # ★ 首选「固定脉冲自检」：**不需要看到靶子（不依赖视觉）、也不依赖脉冲/度标定**，
        #   上电即可验证两轴链路与机械行程。顺序按现场要求：
        #   **pitch 先上转再下转，yaw 先左转再右转**；每轴两段等长，结束时回到起始位姿。
        pulses = int(getattr(self.cfg, "SELF_TEST_PULSES",
                             getattr(self.cfg, "SELF_TEST_FALLBACK_PULSES", 0)))
        if pulses > 0:
            print("[self-test] 固定脉冲自检（无需靶子 / 无需标定）："
                  "pitch 上转 %d → 下转 %d，yaw 左转 %d → 右转 %d"
                  % (pulses, pulses, pulses, pulses))
            self._install([
                ("pitch", pitch_ccw_positive, pulses),        # pitch 上转（仰起）
                ("pitch", not pitch_ccw_positive, pulses),    # pitch 下转（回到起始）
                ("yaw", yaw_ccw_positive, pulses),            # yaw 左转
                ("yaw", not yaw_ccw_positive, pulses),        # yaw 右转（回到起始）
            ])
            return
        # 备选：按"脉冲/度"换算角度的精细自检（需先 CALIBRATE_AXIS 实测，或给 GIMBAL_GEAR_RATIO）。
        yaw_ppd = float(getattr(self.cfg, "YAW_PULSES_PER_DEG", 0.0))
        pitch_ppd = float(getattr(self.cfg, "PITCH_PULSES_PER_DEG", 0.0))
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
            print("[self-test] skipped: 置 SELF_TEST_PULSES>0 走固定脉冲自检（推荐，无需靶子）")
            print("[self-test]           或填 YAW/PITCH_PULSES_PER_DEG / GIMBAL_GEAR_RATIO 走角度自检")
            return
        yaw_angle = abs(float(getattr(self.cfg, "SELF_TEST_YAW_DEG", 15.0)))
        pitch_angle = abs(float(getattr(self.cfg, "SELF_TEST_PITCH_DEG", 30.0)))
        self._install([
            ("yaw", yaw_ccw_positive, yaw_angle * yaw_ppd),
            ("yaw", not yaw_ccw_positive, 2 * yaw_angle * yaw_ppd),
            ("yaw", yaw_ccw_positive, yaw_angle * yaw_ppd),
            ("pitch", pitch_ccw_positive, pitch_angle * pitch_ppd),
            ("pitch", not pitch_ccw_positive, 2 * pitch_angle * pitch_ppd),
            ("pitch", pitch_ccw_positive, pitch_angle * pitch_ppd),
        ])

    @property
    def done(self):
        return self.state in ("DONE", "SKIPPED", "FAULT")

    def update(self, now):
        """每帧调用；到达估算运动时间后发下一段，不阻塞视觉主循环。"""
        if self.done:
            return self.state
        now = float(now)
        # ★ 硬超时优先于一切：无论卡在哪一步（链路抖动、帧率骤降、某步不返回），
        #   到点就结束自检并交给上层继续（接着标定 / 直接追踪）。
        # 只在"**还有步没发完**"时才算卡住并强制结束；4 步都发出后应正常等最后一步走完，
        # 否则最后一段机械动作会被截断、紧接着的追踪指令会与之堆叠。
        if self.timeout_s > 0 and self.sequence and now - self._t0 >= self.timeout_s:
            self.state = "DONE"
            print("[self-test] 硬超时 %.1fs -> 强制结束（已发 %d/%d 步），转入下一阶段"
                  % (self.timeout_s, self.sent, self.total))
            return self.state
        if now < self.deadline:
            return self.state
        if not self.sequence:
            self.state = "DONE"
            print("[self-test] complete; returned to startup pose")
            return self.state
        axis, ccw, raw_pulses = self.sequence.pop(0)
        pulses = int(round(raw_pulses))
        self.sent += 1
        if pulses <= 0:
            print("[self-test] %s 脉冲数非法(%d) -> 跳过该步" % (axis, pulses))
            self.deadline = now + 0.05
            return self.state
        # ★ 慢速 + 曲线加减速：SELF_TEST_RPM 默认 60（旧 900 太快、肉眼像"弹"一下），
        #   SELF_TEST_ACC 默认 60（非 0 才启用 ZDT 曲线加减速；0 = 直接启停 = 卡顿）。
        rpm = int(getattr(self.cfg, "SELF_TEST_RPM", 60))
        acc = int(getattr(self.cfg, "SELF_TEST_ACC", 0))
        if not self.link.move(axis, ccw, pulses, rpm, acc):
            # 关键：单轴写失败只跳过该轴剩余步骤，另一轴继续 —— 旧实现直接整机 FAULT，
            # 结果"yaw 链路不通 -> pitch 一步指令都发不出去"（现场=只使能、不动）。
            self._failed.add(axis)
            self.sequence = [s for s in self.sequence if s[0] not in self._failed]
            print("[self-test] %s 指令发送失败 -> 跳过该轴剩余步骤（另一轴继续）" % axis)
            # 判据是"**所有参与轴都失败**"才算 FAULT，而不是"序列恰好走空" ——
            # 否则"最后一轴才失败"（例如 pitch 先动完、yaw 才写失败）会被误判成整机故障。
            if self._planned and self._failed >= self._planned:
                self.state, self.reason = "FAULT", "uart_write_failed"
                print("[self-test] fault: 所有参与轴的指令都发不出去")
            self.deadline = now + 0.05
            return self.state
        pulses_per_rev = float(getattr(self.cfg, "MOTOR_PULSES_PER_REV", 3200))
        move_s = estimate_move_seconds(pulses, rpm, acc, pulses_per_rev)
        settle_s = float(getattr(self.cfg, "SELF_TEST_SETTLE_S", 0.8))
        self.deadline = now + move_s + settle_s
        print("[self-test] %s %s %dp rpm=%d acc=%d step=%d/%d 预计%.2fs" %
              (axis, "CCW" if ccw else "CW", pulses, rpm, acc,
               self.sent, self.total, move_s + settle_s))
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
