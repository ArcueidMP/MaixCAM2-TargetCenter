"""端到端跑真实 main()：验证「上电自检改为屏幕按钮」的新契约（v0.4.1）。

背景（2026-10-11 需求变更）：原来的「上电自检优先级最高、开机自动跑」被取消 ——
自检改成一个**屏幕可点击按钮**，点一下才跑。本用例用 mock 的 maix（含 display/touchscreen）
跑真实 `main()`，锁定三件事：

  1. **上电不自动跑自检**：没有任何点击时，一条运动帧（0xFD）都不发；
  2. **点击按钮才跑**：模拟点击右上角 SELF-TEST 后，自检 6 步（yaw 左→右、pitch 上→下）应发出；
  3. **链路 FAULT 时点击也不发指令**。

另外确认「画面流转到 MaixVision」：`display.Display().show()` 被真正调用（preview）。
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

CLOCK_STEP_S = 0.02          # 每次 perf_counter() 前进的模拟时间（~0.2s/帧，同板端量级）
SCREEN_W, SCREEN_H = 640, 480
TAP = (500, 50)              # 落在右上角 SELF-TEST 按钮内


def run(fault=False, iterations=600, touch_events=(), log_interval=1000):
    ports = []
    shown = []

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
            # 纯白画面：永远识别不到黑框 -> 若仍有运动帧，必来自自检。
            return np.full((480, 640), 200, np.uint8)

        def close(self):
            pass

    class Disp:
        def width(self):
            return SCREEN_W

        def height(self):
            return SCREEN_H

        def show(self, img):
            shown.append(img)

        def close(self):
            pass

    class Touch:
        def __init__(self):
            self.events = list(touch_events)
            self.i = 0

        def read(self):
            if self.i < len(self.events):
                ev = self.events[self.i]
                self.i += 1
                return ev
            return (0, 0, False)

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
    maix.app = types.SimpleNamespace(need_exit=App().need_exit, set_exit_flag=lambda v: None)
    maix.uart = types.SimpleNamespace(UART=UART)
    maix.pinmap = types.SimpleNamespace(set_pin_function=lambda *args: 0,
                                       get_pin_function=lambda pin: "?")
    maix.err = types.SimpleNamespace(check_raise=lambda *args: None)
    maix.camera = types.SimpleNamespace(Camera=Camera)
    maix.display = types.SimpleNamespace(Display=Disp)
    maix.touchscreen = types.SimpleNamespace(TouchScreen=Touch)
    maix.image = types.SimpleNamespace(
        Format=types.SimpleNamespace(FMT_GRAYSCALE=0),
        image2cv=lambda img, **kwargs: img,
        cv2image=lambda img, **kwargs: img)
    cfg = type("TestConfig", (entry.Config,), dict(
        SHOW=True, SHOW_FPS=30, VISION_ASYNC=False, RESULT_MAX_AGE_S=1.0,
        # 本用例用假时钟 (Clock) 逐步推进，只能跑**同步**显示路径 ——
        # 异步显示线程会与假时钟并发抢 perf_counter，判定不可复现（异步由
        # test_runtime_performance.test_slow_show_does_not_block_main_loop 单独覆盖）。
        SHOW_ASYNC=False, LOG_INTERVAL=log_interval, CALIBRATE_AXIS=""))
    with contextlib.ExitStack() as stack:
        stack.enter_context(patch.dict(sys.modules, maix=maix))
        stack.enter_context(patch.object(entry, "Config", cfg))
        stack.enter_context(patch.object(entry, "time", Clock))
        stack.enter_context(patch.object(entry, "probe_imu", return_value={
            "available": False, "error": "test"}))
        if fault:
            stack.enter_context(patch.object(StepperLink, "poll", return_value={
                "fault": True, "reason": "test_fault", "frames": []}))
        log = stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        entry.main()
    return ports, shown, log.getvalue()


def _motion_frames(port):
    return [p for p in port.sent if len(p) >= 2 and p[1] == 0xFD]


def test_boot_does_not_autostart_self_test():
    """上电不自动跑自检：无点击 -> 0 条运动帧；使能帧应已发。"""
    ports, shown, log = run(fault=False, touch_events=())
    assert [p.dev for p in ports] == ["/dev/ttyS2", "/dev/ttyS0"], ports
    for port in ports:
        assert port.sent and port.sent[0][1] == 0xF3, "上电应先发使能帧"
        assert not _motion_frames(port), "上电不应自动发运动帧（自检改为按钮触发）"
    assert "[ui] 上电自检已改为屏幕按钮" in log
    assert "[self-test] complete" not in log
    assert shown, "画面应已流转到显示（display.show 未被调用）"
    print("PASS: 上电不自动跑自检（0 运动帧），且画面已流转到 MaixVision")


def test_tap_button_runs_self_test():
    """点击 SELF-TEST 按钮 -> 自检 6 步（yaw×3、pitch×3）并回到起点。"""
    ports, shown, log = run(fault=False, touch_events=[(TAP[0], TAP[1], True)])
    assert "[ui] 收到点击 -> 启动上电自检" in log, "按钮点击没有被识别"
    yaw, pitch = ports
    assert len(_motion_frames(yaw)) == 3, yaw.sent
    assert len(_motion_frames(pitch)) == 3, pitch.sent
    assert all(p[0] == 2 for p in _motion_frames(yaw)), "yaw 帧地址应为 ID2"
    assert all(p[0] == 1 for p in _motion_frames(pitch)), "pitch 帧地址应为 ID1"
    assert "[self-test] complete" in log, "自检没有跑完"
    print("PASS: 点击按钮后自检 6 步发出并完成（yaw×3@ID2、pitch×3@ID1）")


def test_fault_blocks_self_test():
    """链路判 FAULT 后，点击按钮也不得下发运动指令。"""
    ports, _, log = run(fault=True, touch_events=[(TAP[0], TAP[1], True)])
    assert all(not _motion_frames(port) for port in ports), "FAULT 后仍在发运动帧"
    assert "link:test_fault" in log
    print("PASS: 链路 FAULT 阻断自检（点击也不发任何 0xFD 运动帧）")


def test_perf_log_branch_formatting():
    """[perf] 日志分支必须能跑通、字段齐全。

    这条 print 在板上每 `LOG_INTERVAL`(2s) 执行一次，一旦格式串与实参不匹配就会抛 TypeError
    **直接打死主循环**；而上面两个用例为了日志干净把 LOG_INTERVAL 设成 1000（从不执行该分支），
    等于把这段代码留白。这里把周期压到 0 强制命中（v0.4.3 新增分相字段时补的守卫）。
    """
    _, _, log = run(fault=False, iterations=400, log_interval=0.0)
    perf = [l for l in log.splitlines() if l.startswith("[perf]")]
    assert perf, "没命中 [perf] 日志分支"
    for field in ("capture=", "disp=", "detect_fps=", "read=", "draw=", "loop=",
                  "show avg=", "drop(show)=", "drop(vision)="):
        assert field in perf[0], "缺字段 %s：%s" % (field, perf[0])
    assert "[preview] async=" in log, "启动未打印显示线程模式"
    print("PASS: [perf] 日志分支可执行、字段齐全（%d 行）" % len(perf))


if __name__ == "__main__":
    test_boot_does_not_autostart_self_test()
    test_tap_button_runs_self_test()
    test_fault_blocks_self_test()
    test_perf_log_branch_formatting()
    print("main self-test(按钮式) e2e: PASS (4)")
