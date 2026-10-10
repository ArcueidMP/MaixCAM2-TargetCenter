"""端到端跑真实 main()：验证「上电先自检（固定脉冲、不等黑框）→ 再标定/追踪」的顺序。

为什么必须有这条用例（2026-10-10）：
`CALIBRATE_AXIS="auto"` 时旧代码在初始化阶段**只创建 AutoCalibrate、从不创建
GimbalSelfTest** —— 于是上电后一条运动指令都不发，必须等画面里识别到黑框（靶心）
才开始自动标定；现场表现就是"上电没动作、得等靶纸"。本用例用**全白画面（无黑框）**
跑真实 main()，锁定三件事：

  1. 上电自检**不依赖视觉**：画面里没有靶心，也照走 pitch 上→下、yaw 左→右；
  2. 自检**先于**自动标定：标定开始的日志晚于自检完成，且此前只有 4 条自检脉冲帧；
  3. 链路 FAULT 时自检与标定**都不发指令**。

顺序 / 每轴帧数的口径见 `GimbalSelfTest._prepare`（固定脉冲：pitch, pitch, yaw, yaw）。
"""
import contextlib
import io
import sys
import types
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import main as entry
from vision.stepper_link import StepperLink

SELF_TEST_PULSES = 400        # = config.SELF_TEST_PULSES（固定脉冲自检幅度）
# 每次 perf_counter() 调用推进的模拟时间。主循环一帧会调用 ~10 次，故约 0.2s/帧，
# 与真实板端 20~30fps 同量级 —— 太大会让自检的 SELF_TEST_TIMEOUT_S 兜底被瞬间触发。
CLOCK_STEP_S = 0.02


def run(fault=False, calibration="auto", iterations=400, tracking_errors=None):
    ports = []

    class UART:
        def __init__(self, dev, baud):
            self.dev, self.sent = dev, []
            ports.append(self)

        def write(self, data):
            self.sent.append(bytes(data))
            return len(data)

        def available(self, timeout):
            return 0

    class Camera:
        def __init__(self, *args, **kwargs):
            pass

        def skip_frames(self, n):
            pass

        def read(self):
            # ★ 纯白画面：永远识别不到黑框 -> 自检若仍完成，即证明它不依赖视觉。
            return np.full((480, 640), 200, np.uint8)

        def close(self):
            pass

    class Clock:
        current = 0.0

        @classmethod
        def perf_counter(cls):
            cls.current += CLOCK_STEP_S
            return cls.current

    class App:
        def __init__(self):
            self.n = 0

        def need_exit(self):
            self.n += 1
            return self.n > iterations

    maix = types.ModuleType("maix")
    maix.app = types.SimpleNamespace(need_exit=App().need_exit)
    maix.uart = types.SimpleNamespace(UART=UART)
    maix.pinmap = types.SimpleNamespace(set_pin_function=lambda *args: 0,
                                        get_pin_function=lambda pin: "?")
    maix.err = types.SimpleNamespace(check_raise=lambda *args: None)
    maix.camera = types.SimpleNamespace(Camera=Camera)
    maix.display = types.SimpleNamespace()
    maix.image = types.SimpleNamespace(
        Format=types.SimpleNamespace(FMT_GRAYSCALE=0),
        image2cv=lambda img, **kwargs: img)
    cfg = type("TestConfig", (entry.Config,), dict(
        SHOW=False, LENS_CORR=False, LOG_INTERVAL=1000, CALIBRATE_AXIS=calibration))
    if tracking_errors is not None:
        cfg.GIMBAL_SELF_TEST = False
        errors = iter(tracking_errors)

        def detection(gray):
            dx = next(errors, None)
            if dx is None:
                return dict(valid=False, stage="no_black_frame")
            u, v = cfg.LASER_U_NOMINAL + dx, cfg.LASER_V_NOMINAL
            return dict(valid=True, stage="track", u=u, v=v, raw_u=u, raw_v=v)
    with contextlib.ExitStack() as stack:
        stack.enter_context(patch.dict(sys.modules, maix=maix))
        stack.enter_context(patch.object(entry, "Config", cfg))
        stack.enter_context(patch.object(entry, "time", Clock))
        stack.enter_context(patch.object(entry, "probe_imu", return_value={
            "available": False, "error": "test"}))
        if tracking_errors is not None:
            stack.enter_context(patch.object(entry.RectangleCenterDetector, "process", side_effect=detection))
        if fault:
            stack.enter_context(patch.object(StepperLink, "poll", return_value={
                "fault": True, "reason": "test_fault", "frames": []}))
        log = stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        entry.main()
    return ports, log.getvalue()


def _motion_frames(port):
    return [p for p in port.sent if p[1] == 0xFD]


def test_self_test_runs_before_calibration():
    """正常/自动标定两种配置下，上电都必须先跑固定脉冲自检（不等黑框）。"""
    for calibration in ("", "auto"):
        ports, log = run(calibration=calibration)
        # 端口顺序 = StepperLink 逐轴打开的顺序：yaw(ttyS2) 在前、pitch(ttyS0) 在后。
        assert [p.dev for p in ports] == ["/dev/ttyS2", "/dev/ttyS0"], ports
        for port, address in zip(ports, (2, 1)):          # yaw=ID2, pitch=ID1
            assert port.sent[0][1] == 0xF3, "上电应先发使能帧"
            moves = _motion_frames(port)
            assert len(moves) == 2, port.sent            # 每轴两段（往返）
            assert [int.from_bytes(p[6:10], "big") for p in moves] == [
                SELF_TEST_PULSES, SELF_TEST_PULSES]
            assert all(p[0] == address for p in moves)
        assert "FAULT" not in log
        assert "[self-test] complete" in log, "上电自检没有跑完"
        assert "硬超时" not in log, "自检未在 SELF_TEST_TIMEOUT_S 内自然跑完（3s 目标没达成）"
        assert "上电自检 4 步" in log
        if calibration == "auto":
            # ★ 关键：自检完成之后才开始自动标定（旧代码是直接标定、根本不自检）。
            assert "上电自检完成 -> 开始自动标定" in log
            assert log.index("[self-test] complete") < log.index("上电自检完成")
    print("PASS: 上电先自检（4 步、不等黑框），auto 模式下自检完成后才开始标定")


def test_fault_blocks_self_test_and_calibration():
    """链路判 FAULT 后，自检与标定都不得再下发运动指令。"""
    for calibration in ("", "auto"):
        ports, log = run(fault=True, calibration=calibration)
        assert all(not _motion_frames(port) for port in ports), "FAULT 后仍在发运动帧"
        assert "link:test_fault" in log
    print("PASS: 链路 FAULT 阻断自检与标定（不发任何 0xFD 运动帧）")


def test_tracking_filter_stops_reach_uart():
    """真实 main() 与 API 都要把控制器取消运动的决定发送为 0xFE，不能只停止生成新指令。"""
    with patch.dict(globals(), CLOCK_STEP_S=0.001):
        for errors in ([100, 1], [100, None], [100, -100]):
            ports, _ = run(calibration="", iterations=2, tracking_errors=errors)
            yaw = ports[0]
            assert len(_motion_frames(yaw)) == 1, yaw.sent
            pulses = int.from_bytes(_motion_frames(yaw)[0][6:10], "big")
            assert pulses <= 4, pulses
            # main 正常退出另外发一条 stop；退出前必须已因死区/失锁/换向发过 stop。
            assert [p[1] for p in yaw.sent].count(0xFE) == 2, yaw.sent
    from target_api import AimLoop
    from vision.gimbal_aim import GimbalAim

    class Link:
        def __init__(self):
            self.sent = []

        def move(self, axis, ccw, pulses, rpm, acc, now=None):
            self.sent.append((axis, 0xFD, pulses, acc))
            return True

        def stop(self, axis=None, now=None):
            self.sent.append((axis, 0xFE))
            return True

        def poll(self, now):
            return {"fault": False}

    loop = AimLoop(enable_motors=False)
    loop.aim, loop.link = GimbalAim(entry.Config), Link()
    results = iter([dict(valid=True, aim_valid=True, dx=100, dy=0),
                    dict(valid=True, aim_valid=True, dx=1, dy=0)])
    loop.tracker = types.SimpleNamespace(track=lambda frame, now: next(results))
    loop.step(None, now=0)
    stopped = loop.step(None, now=0.02)
    assert loop.link.sent == [("yaw", 0xFD, 4, entry.Config.ACC_TRACK), ("yaw", 0xFE)], loop.link.sent
    assert stopped["aim_state"] == "SETTLE"
    print("PASS: main/API 大误差小步起动，死区/失锁/换向取消帧抵达电机链路")


if __name__ == "__main__":
    test_self_test_runs_before_calibration()
    test_fault_blocks_self_test_and_calibration()
    test_tracking_filter_stops_reach_uart()
