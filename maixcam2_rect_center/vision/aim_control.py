"""闭环调度中枢 + 旧版 ASCII 串口输出（从 main.py 搬迁，v0.4.6 模块化）。

VisionControl 把"识别 → 光斑 → 偏差 → 串口 → 闭环 → 打包给显示层"整条链放进**识别线程**，
因为它独占 UART（同一轴收发必须串行）。主线程要触发自检只置 request_test 事件。
★ 调度优先级顺序不可调换（见 B 轨 AGENTS.md §3.2）：FAULT 必须最前。
★ clock 注入：构造可传 `clock`（默认标准库 time），使假时钟可穿透（见 vision_worker.py 说明）。
"""
import threading
import time
import math

from .gimbal_search import SearchSweep
from .gimbal_self_test import GimbalSelfTest
from .imu_heading import HeadingFeedforward
from .laser_spot import LaserSpotDetector, deviation
from .rectangle_center import RectangleCenterDetector


def send_uart(uart, result):
    """旧版纯视觉 ASCII 输出（ENABLE_GIMBAL=False 的降级路径才用；开云台时 uart 为 None）。"""
    if uart is None:
        return
    if result.get("valid"):
        payload = "@[%d,%d]%%\r\n" % (int(round(result["u"])), int(round(result["v"])))
    else:
        payload = "@[invalid]%%\r\n"
    uart.write(payload.encode("ascii"))


class VisionControl:
    """识别 + 电机状态由**识别线程独占**；主循环只提交图像、按钮事件、读结果。

    独占理由：UART 读写不可并发（两条轴各有独立串口，但同一轴的收发必须串行），
    把"识别 → 偏差 → 闭环下发 → 收 ACK"全放进识别线程，天然避免与主线程抢串口。
    主线程要触发自检只置 `request_test` 事件，由识别线程在自己的上下文里安全地启动。
    """

    def __init__(self, cfg, link, aim, calibrator, uart, clock=None, heading=None):
        self.cfg, self.link, self.aim = cfg, link, aim
        self.calibrator, self.uart = calibrator, uart
        self._clock = clock if clock is not None else time
        self.self_test = None
        self.request_test = threading.Event()   # 主线程置位、识别线程消费（按钮 → 自检）
        self.request_search = threading.Event()
        self.request_imu_bias = threading.Event()
        self.search = None
        self.notice = ""
        self.heading = heading
        if self.heading is None and bool(getattr(cfg, "ENABLE_IMU_FEEDFORWARD", False)):
            self.heading = HeadingFeedforward(cfg, clock=self._clock)
        self.last_visual_reference = -math.inf
        self.last_visual_capture = -math.inf
        self.control_source = "vision"
        self.last_commands = {"yaw": None, "pitch": None}
        self.detector = RectangleCenterDetector(cfg)
        self.laser = LaserSpotDetector(cfg) if cfg.ENABLE_LASER else None
        self.laser_static = self.laser is not None and self.laser.mode == "fixed"
        # fixed 模式：光斑位置是常量，开机取一次即可，之后每帧只做减法（零检测开销）。
        self.laser_state = self.laser.static_state() if self.laser_static else None
        self.last_capture = -1e9
        self.stale_stopped = True

    def process(self, gray, captured_at):
        """识别线程的单帧主流程：识别 → 光斑 → 偏差 → 串口 → 闭环 → 打包给显示层。"""
        started = self._clock.perf_counter()
        result = self.detector.process(gray, timestamp=captured_at)  # ① 按采图时间预测
        result["captured_at"] = float(captured_at)
        detected = self._clock.perf_counter()
        if self.laser is not None and not self.laser_static:
            self.laser_state = self.laser.process(gray, detected)   # ② detect 模式才实测光斑
        dv = deviation(self.laser_state, result, use_raw=self.cfg.AIM_USE_RAW) if self.laser_state else None
        if dv is not None:
            result["laser_u"], result["laser_v"] = self.laser_state["u"], self.laser_state["v"]
            result["dx"], result["dy"] = dv                 # dx=靶心−光斑（闭环用的那个偏差）
        now = self._clock.perf_counter()
        result["aim_valid"] = dv is not None
        return self.accept_result(result, captured_at, self.laser_state,
                                  started=started, detected=detected)

    def accept_result(self, result, captured_at, laser=None, started=None, detected=None):
        """统一 API/相机的控制入口；result 必须是本帧的真实视觉测量。"""
        now = self._clock.perf_counter()
        started = now if started is None else started
        detected = now if detected is None else detected
        result["captured_at"] = float(captured_at)
        result["age_s"] = max(0.0, now - captured_at)
        timestamp_valid = math.isfinite(float(captured_at)) and 0 <= now - captured_at <= self.cfg.RESULT_MAX_AGE_S
        ordered = captured_at > self.last_capture
        fresh = timestamp_valid and ordered
        result["aim_valid"] = bool(result.get("aim_valid") and fresh)
        if laser is not None:
            self.laser_state = laser
        if fresh:
            self.last_capture = captured_at
            if result.get("aim_valid"):
                self.last_visual_capture = captured_at
        # 过期结果不外发：串口与闭环都用降级后的 output，避免拿旧定位驱动电机。
        output = result if fresh else dict(valid=False, aim_valid=False, stage="stale")
        send_uart(self.uart, output)
        # 重复/倒序图像只触发保活，不能覆盖新参考或按旧误差打断正在进行的动作。
        self._control(output if ordered else None, now, new_measurement=ordered)
        # 返回给显示层的快照：识别结果 + 光斑 + 计时 + 状态串。
        return dict(result=result, laser=dict(self.laser_state) if self.laser_state else None,
                    captured_at=captured_at, completed_at=self._clock.perf_counter(),
                    detect_ms=(detected - started) * 1000, status=self.status())

    def idle(self):
        # 摄像头/图传停顿时继续收 ACK 与推进按钮自检，禁止重复发送旧视觉误差。
        self._control(None, self._clock.perf_counter(), new_measurement=False)

    def status(self):
        """给屏幕显示状态：故障优先，其后为自检、标定、搜索和闭环。"""
        if self.aim is not None and self.aim.state == "FAULT":
            return "FAULT"
        if self.self_test is not None and not self.self_test.done:
            return "self-test %d/%d" % (self.self_test.index, len(self.self_test.sequence))
        if self.calibrator is not None and not self.calibrator.done:
            return "calibrate %s" % self.calibrator.axis
        if self.search is not None:
            if not self.search.done:
                return "SEARCH %d/%d" % (self.search.index, len(self.search.sequence))
            if self.search.state in ("UNCALIBRATED", "FAULT"):
                return "SEARCH %s: %s" % (self.search.state, self.search.reason)
        if self.control_source == "imu":
            return "IMU HOLD %s" % self.aim.state
        if self.notice:
            return self.notice
        return self.aim.state if self.aim is not None else "vision"

    def imu_status(self):
        return self.heading.sample(self._clock.perf_counter()) if self.heading is not None else {
            "valid": False, "state": "DISABLED"}

    def stop(self, now=None):
        now = self._clock.perf_counter() if now is None else float(now)
        if self.search is not None and not self.search.done:
            self.search.cancel(now)
            self._finish_search()
            accepted = self.search.state != "FAULT"
        else:
            accepted = True if self.link is None else self._stop_motion(now)
        self.stale_stopped = True
        if self.heading is not None:
            self.heading.reset_reference()
        self.last_visual_reference = -math.inf
        return accepted

    def close(self):
        if self.heading is not None:
            self.heading.close()

    def _finish_search(self):
        if self.search.state == "FAULT":
            self.aim.fault("search:%s" % self.search.reason)
        elif self.search.stopped_at is not None:
            self.aim.motion_stopped(now=self.search.stopped_at)
            self.aim.update({"aim_valid": False}, now=self.search.stopped_at)
        self.stale_stopped = True
        self.last_visual_reference = -math.inf
        if self.heading is not None:
            self.heading.reset_reference()

    def _degree_scales(self):
        result = {}
        for axis in ("yaw", "pitch"):
            ppd = float(getattr(self.cfg, axis.upper() + "_PULSES_PER_DEG", 0.0))
            if ppd <= 0:
                ppd = float(getattr(self.cfg, "GIMBAL_GEAR_RATIO", 0.0)) * float(
                    getattr(self.cfg, "MOTOR_PULSES_PER_REV", 3200)) / 360.0
            ppp = float(getattr(self.cfg, "PULSE_PER_PX_" + axis.upper(), 0.0))
            sign = float(getattr(self.cfg, "DIR_SIGN_" + axis.upper(), 0.0))
            if not all(math.isfinite(v) for v in (ppd, ppp, sign)) or min(ppd, ppp) <= 0 or abs(sign) != 1:
                return None
            physical = 1 if bool(getattr(self.cfg, axis.upper() + "_POSITIVE_CCW", True)) else -1
            result[axis] = (ppd, ppp, sign * physical)
        return result

    def _lock_visual_heading(self, result, now):
        if self.heading is None or not result.get("aim_valid"):
            return
        scales = self._degree_scales()
        if scales is None:
            self.notice = "IMU: pulse/degree calibration required"
            return
        offsets = {axis: float(result[key]) * ppp / ppd * sign
                   for axis, key in (("yaw", "dx"), ("pitch", "dy"))
                   for ppd, ppp, sign in (scales[axis],)}
        captured = float(result["captured_at"])
        if (captured <= getattr(self.aim, "capture_after", -math.inf) or
                captured <= self.last_visual_reference):
            return
        if self.heading.lock_reference(captured, yaw_offset_deg=offsets["yaw"],
                                       pitch_offset_deg=offsets["pitch"]):
            self.last_visual_reference = captured

    def _inertial_result(self, now):
        if self.heading is None or now - self.last_visual_reference > float(
                getattr(self.cfg, "IMU_HOLD_MAX_S", 2.0)):
            return None
        scales = self._degree_scales()
        if scales is None:
            return None
        state = self.heading.correction(now, lead_s=float(getattr(self.cfg, "IMU_LEAD_S", 0.03)))
        if not state.get("valid"):
            return None
        angles = (float(state["yaw_deg"]), float(state["pitch_deg"]))
        if not all(math.isfinite(x) for x in angles) or max(abs(x) for x in angles) > float(
                getattr(self.cfg, "IMU_MAX_CORRECTION_DEG", 100.0)):
            return None
        errors = {axis: angle * ppd / ppp * sign
                  for axis, angle in zip(("yaw", "pitch"), angles)
                  for ppd, ppp, sign in (scales[axis],)}
        return dict(aim_valid=True, valid=False, source="imu", dx=errors["yaw"],
                    dy=errors["pitch"], captured_at=state["sampled_at"])

    def _dispatch(self, result, now):
        output = self.aim.update(result, now=now)
        for axis in output.get("stop_axes", ()):
            if not self._stop_motion(now, axis=axis, reset=False):
                return
        for axis, command in output["cmds"].items():
            if command is not None:
                ccw, pulses, rpm = command
                acc = getattr(self.cfg, "IMU_ACC", 80) if result.get("source") == "imu" else self.cfg.ACC_TRACK
                accepted = self.link.move(axis, ccw, pulses, rpm, acc, now=now)
                if accepted is False:
                    self._stop_motion(now)
                    self.aim.fault("tx:%s" % axis)
                    self.last_commands = {"yaw": None, "pitch": None}
                    return
                self.last_commands[axis] = command

    def _stop_motion(self, now, axis=None, reset=True):
        """停机帧未写成功时保持 FAULT，不能按已停机继续追加动作。"""
        accepted = self.link.stop(axis=axis, now=now)
        if accepted is False:
            if axis is not None:
                self.link.stop(now=now)                    # 尝试停止另一轴，随后阻断所有动作
            self.aim.fault("stop:%s" % (axis or "all"))
            return False
        if reset:
            self.aim.motion_stopped(axis, now=now)
        return True

    def _control(self, result, now, new_measurement):
        """闭环调度中枢：故障检查 → 按钮自检 → 标定 → 自检 → 失锁停机 → 追踪下发。

        优先级顺序**不可随意调换**（见 B 轨 AGENTS.md §3.2）；尤其 FAULT 必须在最前，
        因为链路故障要阻断的不只是追踪，还包括已启动的自检/标定。
        """
        link, aim = self.link, self.aim
        self.last_commands = {"yaw": None, "pitch": None}
        if link is None or aim is None:
            if self.request_search.is_set():
                self.request_search.clear()
                self.notice = "SEARCH unavailable: gimbal disabled"
            return
        feedback = link.poll(now)                            # 非阻塞收两条串口的返回帧
        if feedback and bool(feedback.get("fault")) and aim.state != "FAULT":
            reason = feedback.get("reason")
            self._stop_motion(now)
            aim.fault("link:%s" % reason)                    # 链路故障 → 进入 FAULT
            if self.search is not None and not self.search.done:
                self.search.fail(reason, now)
        imu_calibrating = self.heading is not None and self.heading.sample(now).get("state") == "CALIBRATING"
        if self.request_test.is_set():                       # 屏幕按钮置位的自检请求
            self.request_test.clear()
            if self.self_test is not None and not self.self_test.done:
                print("[ui] 自检进行中，忽略重复点击")
            elif aim.state == "FAULT":
                print("[ui] 链路 FAULT，拒绝自检")
            elif self.calibrator is not None and not self.calibrator.done:
                print("[ui] 标定进行中，拒绝自检")
            elif self.search is not None and not self.search.done:
                print("[ui] 寻靶进行中，先取消 SEARCH 再自检")
            elif imu_calibrating:
                print("[ui] IMU 偏置标定中，拒绝自检")
            elif not self.cfg.GIMBAL_SELF_TEST:
                print("[ui] GIMBAL_SELF_TEST=False，忽略自检")
            else:
                if not self._stop_motion(now):              # 先停当前动作再起自检
                    return
                self.self_test = GimbalSelfTest(self.cfg, link, now)
                print("[ui] 收到点击 -> 启动上电自检（%d 步）" % len(self.self_test.sequence))
        if self.request_search.is_set():
            self.request_search.clear()
            if aim.state == "FAULT":
                self.notice = "SEARCH rejected: FAULT"
            elif self.calibrator is not None and not self.calibrator.done:
                self.notice = "SEARCH rejected: calibrating"
            elif self.self_test is not None and not self.self_test.done:
                self.notice = "SEARCH rejected: self-test"
            elif imu_calibrating:
                self.notice = "SEARCH rejected: IMU calibrating"
            elif self.search is not None and not self.search.done:
                self.search.cancel(now)
                self._finish_search()
                return
            elif new_measurement and result and result.get("valid"):
                self.notice = "SEARCH: target already visible"
            elif self._stop_motion(now):
                self.notice = ""
                self.search = SearchSweep(self.cfg, link, now)
                if self.search.done:
                    self._finish_search()
        if self.request_imu_bias.is_set():
            self.request_imu_bias.clear()
            if (self.heading is not None and aim.state != "FAULT" and
                    not (self.search is not None and not self.search.done) and
                    not (self.self_test is not None and not self.self_test.done) and
                    not (self.calibrator is not None and not self.calibrator.done) and
                    self._stop_motion(now)):
                self.heading.start_bias_calibration(now)
                self.notice = "IMU bias: keep vehicle and gimbal stationary"
        # 链路故障必须阻断已启动的自检/标定，不只阻断普通追踪。
        if aim.state == "FAULT":
            return
        if self.heading is not None and self.heading.sample(now).get("state") == "CALIBRATING":
            return
        if self.calibrator is not None and not self.calibrator.done:
            self.calibrator.update(now, result or {"valid": False, "aim_valid": False})
            if self.calibrator.state == "FAULT":
                self._stop_motion(now)
                aim.fault("calibration:%s" % self.calibrator.reason)
            return
        if self.self_test is not None and not self.self_test.done:
            self.self_test.update(now)
            if self.self_test.state == "FAULT":
                self._stop_motion(now)
                aim.fault("self-test:%s" % self.self_test.reason)
            return
        if self.search is not None and not self.search.done:
            self.control_source = "search"
            self.search.update(now, result if new_measurement else None)
            if self.search.done:
                self._finish_search()
            return
        previous_source = self.control_source
        self.control_source = "vision"
        if new_measurement and result and result.get("aim_valid"):
            self._lock_visual_heading(result, now)
        elif not new_measurement and now - self.last_visual_capture < float(getattr(self.cfg, "IMU_VISUAL_GAP_S", 0.05)):
            return
        else:
            inertial = self._inertial_result(now)
            if inertial is not None:
                self.control_source = "imu"
                self.stale_stopped = False
                self._dispatch(inertial, now)
                return
            if previous_source == "imu":
                if not self.stale_stopped:
                    self._stop_motion(now)
                aim.update({"aim_valid": False}, now=now)
                self.stale_stopped = True
                return
        if not new_measurement:
            # idle 路径：没有新视觉误差。若已超时且尚未停机，则停一次并置 aim 无效（防"漂"）。
            if now - self.last_capture > self.cfg.RESULT_MAX_AGE_S and not self.stale_stopped:
                self._stop_motion(now)
                aim.update({"aim_valid": False}, now=now)
                self.stale_stopped = True
            return
        if not result.get("aim_valid"):
            if not self.stale_stopped:                       # 视觉失效：停一次电机，不回零
                self._stop_motion(now)
            self.stale_stopped = True
        else:
            self.stale_stopped = False
        self._dispatch(result, now)
