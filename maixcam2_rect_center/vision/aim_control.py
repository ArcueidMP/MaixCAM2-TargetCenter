"""闭环调度中枢 + 旧版 ASCII 串口输出（从 main.py 搬迁，v0.4.6 模块化）。

VisionControl 把"识别 → 光斑 → 偏差 → 串口 → 闭环 → 打包给显示层"整条链放进**识别线程**，
因为它独占 UART（同一轴收发必须串行）。主线程要触发自检只置 request_test 事件。
★ 调度优先级顺序不可调换（见 B 轨 AGENTS.md §3.2）：FAULT 必须最前。
★ clock 注入：构造可传 `clock`（默认标准库 time），使假时钟可穿透（见 vision_worker.py 说明）。
"""
import threading
import time

from .gimbal_self_test import GimbalSelfTest
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

    def __init__(self, cfg, link, aim, calibrator, uart, clock=None):
        self.cfg, self.link, self.aim = cfg, link, aim
        self.calibrator, self.uart = calibrator, uart
        self._clock = clock if clock is not None else time
        self.self_test = None
        self.request_test = threading.Event()   # 主线程置位、识别线程消费（按钮 → 自检）
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
        result = self.detector.process(gray)                # ① 黑框识别
        detected = self._clock.perf_counter()
        if self.laser is not None and not self.laser_static:
            self.laser_state = self.laser.process(gray, detected)   # ② detect 模式才实测光斑
        dv = deviation(self.laser_state, result, use_raw=self.cfg.AIM_USE_RAW) if self.laser_state else None
        if dv is not None:
            result["laser_u"], result["laser_v"] = self.laser_state["u"], self.laser_state["v"]
            result["dx"], result["dy"] = dv                 # dx=靶心−光斑（闭环用的那个偏差）
        now = self._clock.perf_counter()
        fresh = now - captured_at <= self.cfg.RESULT_MAX_AGE_S   # 结果是否还"新鲜"
        result["aim_valid"] = bool(dv is not None and fresh)
        self.last_capture = captured_at
        # 过期结果不外发：串口与闭环都用降级后的 output，避免拿旧定位驱动电机。
        output = result if fresh else dict(valid=False, aim_valid=False, stage="stale")
        send_uart(self.uart, output)
        self._control(output, now, new_measurement=True)
        # 返回给显示层的快照：识别结果 + 光斑 + 计时 + 状态串。
        return dict(result=result, laser=dict(self.laser_state) if self.laser_state else None,
                    captured_at=captured_at, completed_at=self._clock.perf_counter(),
                    detect_ms=(detected - started) * 1000, status=self.status())

    def idle(self):
        # 摄像头/图传停顿时继续收 ACK 与推进按钮自检，禁止重复发送旧视觉误差。
        self._control(None, self._clock.perf_counter(), new_measurement=False)

    def status(self):
        """给屏幕左下角显示的状态串：自检 > 标定 > 闭环状态机的优先级。"""
        if self.self_test is not None and not self.self_test.done:
            return "self-test %d/%d" % (self.self_test.index, len(self.self_test.sequence))
        if self.calibrator is not None and not self.calibrator.done:
            return "calibrate %s" % self.calibrator.axis
        return self.aim.state if self.aim is not None else "vision"

    def _control(self, result, now, new_measurement):
        """闭环调度中枢：故障检查 → 按钮自检 → 标定 → 自检 → 失锁停机 → 追踪下发。

        优先级顺序**不可随意调换**（见 B 轨 AGENTS.md §3.2）；尤其 FAULT 必须在最前，
        因为链路故障要阻断的不只是追踪，还包括已启动的自检/标定。
        """
        link, aim = self.link, self.aim
        if link is None or aim is None:
            return
        feedback = link.poll(now)                            # 非阻塞收两条串口的返回帧
        if feedback and bool(feedback.get("fault")) and aim.state != "FAULT":
            reason = feedback.get("reason")
            link.stop(now=now)
            aim.fault("link:%s" % reason)                    # 链路故障 → 进入 FAULT
        if self.request_test.is_set():                       # 屏幕按钮置位的自检请求
            self.request_test.clear()
            if self.self_test is not None and not self.self_test.done:
                print("[ui] 自检进行中，忽略重复点击")
            elif aim.state == "FAULT":
                print("[ui] 链路 FAULT，拒绝自检")
            elif self.calibrator is not None and not self.calibrator.done:
                print("[ui] 标定进行中，拒绝自检")
            elif not self.cfg.GIMBAL_SELF_TEST:
                print("[ui] GIMBAL_SELF_TEST=False，忽略自检")
            else:
                link.stop(now=now)                           # 先停当前动作再起自检
                self.self_test = GimbalSelfTest(self.cfg, link, now)
                print("[ui] 收到点击 -> 启动上电自检（%d 步）" % len(self.self_test.sequence))
        # 链路故障必须阻断已启动的自检/标定，不只阻断普通追踪。
        if aim.state == "FAULT":
            return
        if self.calibrator is not None and not self.calibrator.done:
            self.calibrator.update(now, result or {"valid": False, "aim_valid": False})
            return
        if self.self_test is not None and not self.self_test.done:
            self.self_test.update(now)
            return
        if not new_measurement:
            # idle 路径：没有新视觉误差。若已超时且尚未停机，则停一次并置 aim 无效（防"漂"）。
            if now - self.last_capture > self.cfg.RESULT_MAX_AGE_S and not self.stale_stopped:
                link.stop(now=now)
                aim.update({"aim_valid": False}, now=now)
                self.stale_stopped = True
            return
        if not result.get("aim_valid"):
            if not self.stale_stopped:                       # 视觉失效：停一次电机，不回零
                link.stop(now=now)
            self.stale_stopped = True
        else:
            self.stale_stopped = False
        commands = aim.update(result, now=now)["cmds"]       # 状态机算出的两轴增量指令
        for axis, command in commands.items():
            if command is not None:
                ccw, pulses, rpm = command
                link.move(axis, ccw, pulses, rpm, self.cfg.ACC_DEFAULT, now=now)
