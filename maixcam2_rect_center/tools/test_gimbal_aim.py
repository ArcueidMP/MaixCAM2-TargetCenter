"""B 轨云台闭环离线回归：协议帧 / 控制律 / 状态机 / 链路看门狗 / 开环标定。

纯 PC，不依赖 MaixPy；覆盖需求文档 §8.1 要求的"假电机收敛仿真"与
"DIR_SIGN 反了会发散"两个最关键用例。
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vision.stepper_link import (StepperLink, cmd_enable, cmd_position, cmd_speed,
                                 cmd_stop_now, cmd_sync_motion, frame_status)
from vision.gimbal_aim import GimbalAim
from vision.gimbal_self_test import GimbalSelfTest
from vision.gimbal_calibrate import GimbalCalibrator


class Cfg:
    # Isolate the legacy two-level law; new dynamic scheduling is tested separately.
    AIM_PROGRESSIVE = False
    AIM_WAIT_MOTION = False
    AIM_HISTORY_MAX_GAP_S = 10.0
    DEADBAND_PX = 2.0
    COMMAND_INTERVAL_S = 0.0
    PULSE_PER_PX_YAW = 3.67
    PULSE_PER_PX_PITCH = 3.67
    GAIN_YAW = GAIN_PITCH = 0.5
    GAIN_COARSE = 0.9
    COARSE_PX = 24.0
    RPM_FINE = 300
    RPM_COARSE = 1200
    DIR_SIGN_YAW = DIR_SIGN_PITCH = 1
    MAX_STEP_PULSES = 360
    JUMP_REJECT_PX = 400.0
    JUMP_CONFIRM_FRAMES = 3
    ENABLE_SCAN = False
    LOST_FRAMES = 15
    SCAN_STEP_PULSES = 40
    SCAN_LIMIT_PULSES = 400
    SCAN_RPM = 60
    SCAN_STEP_INTERVAL_S = 0.15
    GIMBAL_SELF_TEST = True
    SELF_TEST_YAW_DEG = 15.0
    SELF_TEST_PITCH_DEG = 30.0
    YAW_PULSES_PER_DEG = 10.0
    PITCH_PULSES_PER_DEG = 20.0
    GIMBAL_GEAR_RATIO = 0.0
    YAW_POSITIVE_CCW = False
    PITCH_POSITIVE_CCW = False
    SELF_TEST_RPM = 900
    SELF_TEST_SETTLE_S = 0.35
    MOTOR_PULSES_PER_REV = 3200
    ACC_DEFAULT = 0
    YAW_PINS = {"B0": "UART2_TX", "B1": "UART2_RX"}
    PITCH_PINS = {"A28": "UART0_TX", "A29": "UART0_RX"}
    ADDR_YAW, ADDR_PITCH = 2, 1
    PINMAP_STRICT = False
    FLUSH_RX_ON_START = True
    ENABLE_ACK_WATCHDOG = True
    E2_REENABLE = True
    E2_WINDOW_S = 2.0
    E2_FAULT_COUNT = 10
    RX_TIMEOUT_MS = 300
    RX_TIMEOUT_LIMIT = 3
    CALIBRATE_AXIS = ""
    CALIBRATE_PULSES = 2000
    CALIBRATE_RPM = 300
    CALIBRATE_SAMPLES = 3
    CALIBRATE_SETTLE_S = 0.2
    CALIBRATE_TIMEOUT_S = 60.0


class FakeLink:
    def __init__(self):
        self.moves = []

    def move(self, axis, ccw, pulses, rpm, acc=0, now=None):
        self.moves.append((axis, ccw, pulses, rpm, acc))
        return True

    def stop(self, axis=None, now=None):
        return True

    def enable(self, axis=None, now=None):
        return True


def test_protocol():
    assert cmd_enable(1) == bytes.fromhex("01 F3 AB 01 00 6B")
    assert cmd_speed(1, True, 1500, 10) == bytes.fromhex("01 F6 01 05 DC 0A 00 6B")
    assert cmd_position(1, True, 1500, 32000) == bytes.fromhex(
        "01 FD 01 05 DC 00 00 00 7D 00 00 00 6B")
    assert cmd_stop_now(1) == bytes.fromhex("01 FE 98 00 6B")
    assert cmd_sync_motion() == bytes.fromhex("00 FF 66 6B")
    for frame, want in ((b"\x01\xF3\x02\x6B", 0x02), (b"\x01\xF6\xE2\x6B", 0xE2),
                        (b"\x01\x00\xEE\x6B", 0xEE)):
        assert frame_status(frame) == want
    assert frame_status(b"\x01\xF3") is None


def test_aim_basic():
    aim = GimbalAim(Cfg())
    a = aim.update({"aim_valid": True, "dx": 20, "dy": -10}, now=0)
    assert a["cmds"]["yaw"] == (True, 37, 300)
    assert a["cmds"]["pitch"] == (False, 18, 300)
    b = aim.update({"aim_valid": True, "dx": 1, "dy": 0}, now=1)
    assert b["state"] == "SETTLE" and b["cmds"]["yaw"] is None
    c = aim.update({"aim_valid": False}, now=2)
    assert c["state"] == "HOLD" and c["cmds"]["pitch"] is None
    pid = GimbalAim(Cfg(), controller=lambda axis, error, now: error * 2)
    p = pid.update({"aim_valid": True, "dx": 10, "dy": -5}, now=0)
    assert p["cmds"]["yaw"] == (True, 20, 300)
    assert p["cmds"]["pitch"] == (False, 10, 300)


def test_clamp_and_jump_reject():
    cfg = Cfg()
    cfg.COMMAND_INTERVAL_S = 0.0
    aim = GimbalAim(cfg)
    # 单帧离群：前 2 帧丢弃，第 3 帧才采信（防甩头）。
    assert aim.update({"aim_valid": True, "dx": 2000, "dy": 0}, now=0)["cmds"]["yaw"] is None
    assert aim.update({"aim_valid": True, "dx": 2000, "dy": 0}, now=1)["cmds"]["yaw"] is None
    cmd = aim.update({"aim_valid": True, "dx": 2000, "dy": 0}, now=2)["cmds"]["yaw"]
    assert cmd is not None and cmd[1] == cfg.MAX_STEP_PULSES  # 单帧脉冲限幅生效
    # 恢复正常后离群计数归零
    aim.update({"aim_valid": True, "dx": 20, "dy": 0}, now=3)
    assert aim._jump["yaw"] == 0


def test_coarse_fine_state():
    aim = GimbalAim(Cfg())
    big = aim.update({"aim_valid": True, "dx": 100, "dy": 0}, now=0)
    assert big["state"] == "COARSE" and big["cmds"]["yaw"][2] == Cfg.RPM_COARSE
    small = aim.update({"aim_valid": True, "dx": 10, "dy": 0}, now=1)
    assert small["state"] == "FINE" and small["cmds"]["yaw"][2] == Cfg.RPM_FINE


def _simulate(cfg, dx0, ppp_truth=90.0, frames=80, dt=0.05, sign_truth=1):
    """假电机：正方向脉冲使 dx 减小，每 ppp_truth 个脉冲改变 1px。"""
    aim = GimbalAim(cfg)
    x, t, hist = float(dx0), 0.0, [float(dx0)]
    for _ in range(frames):
        t += dt
        out = aim.update({"aim_valid": True, "dx": x, "dy": 0.0}, now=t)
        cmd = out["cmds"]["yaw"]
        if cmd is not None:
            ccw, pulses, _ = cmd
            x = x - sign_truth * ((1 if ccw else -1) * pulses) / ppp_truth
        hist.append(x)
    return aim, hist


def test_closed_loop_convergence():
    cfg = Cfg()
    cfg.PULSE_PER_PX_YAW = 90.0     # 与真值一致
    cfg.MAX_STEP_PULSES = 100000    # 隔离收敛性本身，限幅另有用例
    aim, hist = _simulate(cfg, 30.0)
    assert abs(hist[-1]) <= cfg.DEADBAND_PX + 1e-9, hist[-3:]
    assert min(hist) >= -1e-9, "无超调（不越过零点）"
    assert aim.state == "SETTLE"
    assert len([h for h in hist if abs(h) <= cfg.DEADBAND_PX + 1e-9]) > 0


def test_reversed_sign_diverges():
    """最危险的失败模式：DIR_SIGN 反了 → 正反馈发散（必须锁住这个行为）。"""
    cfg = Cfg()
    cfg.PULSE_PER_PX_YAW = 90.0
    cfg.DIR_SIGN_YAW = -1                    # 真值恒为 +1
    _, hist = _simulate(cfg, 30.0, frames=20)
    assert abs(hist[-1]) > abs(hist[0]) * 2, "符号反了必须能观察到发散"


def test_ack_watchdog():
    link = StepperLink(Cfg())               # 无 MaixPy -> 无端口，仍可测解析
    reenabled = []
    link.enable = lambda axis=None, now=None: (reenabled.append(axis) or True)
    out = link._ingest([("yaw", bytes.fromhex("01 F3 02 6B"))], 1.0)
    assert out["ok"] == 1 and out["e2"] == 0 and link.stats()["ok"] == 1
    out = link._ingest([("yaw", bytes.fromhex("01 F3 E2 6B"))], 2.0)
    assert out["e2"] == 1 and out["reenabled"] == ["yaw"] and reenabled == ["yaw"]
    # E2 洪水 -> FAULT
    fault = None
    for i in range(Cfg.E2_FAULT_COUNT + 2):
        fault = link._ingest([("yaw", bytes.fromhex("01 F6 E2 6B"))], 3.0 + i * 0.01)
    assert fault["fault"] and "e2_flood" in fault["reason"]


def test_ack_timeout_gated():
    # 从未收到确认帧（Response=None）-> 超时判据自动不生效
    link = StepperLink(Cfg())
    link._ports = {"yaw": object()}
    link._stats["ok"] = 0
    link._last_cmd["yaw"] = 0.5
    link._last_ack["yaw"] = 0.0
    for i in range(10):
        assert link._ingest([], 10.0 + i)["fault"] is False
    # 收到过确认帧之后，链路断 -> 超时判据生效
    link._stats["ok"] = 1
    fault = None
    for i in range(10):
        fault = link._ingest([], 20.0 + i)
    assert fault["fault"] and "ack_timeout" in fault["reason"]


def test_self_test():
    link = FakeLink()
    sweep = GimbalSelfTest(Cfg(), link)
    clock = 0.0
    while not sweep.done:
        sweep.update(clock)
        clock = max(clock, sweep.deadline)
    assert [m[:3] for m in link.moves] == [
        ("yaw", False, 150), ("yaw", True, 300), ("yaw", False, 150),
        ("pitch", False, 600), ("pitch", True, 1200), ("pitch", False, 600)]
    no_cal = type("Uncalibrated", (), dict(Cfg.__dict__))
    no_cal.YAW_PULSES_PER_DEG = 0
    no_cal.PITCH_PULSES_PER_DEG = 0
    assert GimbalSelfTest(no_cal, FakeLink()).state == "SKIPPED"
    # 提供减速比即可在未做脉冲/度标定时推导出角度比例
    geared = type("Geared", (), dict(Cfg.__dict__))
    geared.YAW_PULSES_PER_DEG = 0
    geared.PITCH_PULSES_PER_DEG = 0
    geared.GIMBAL_GEAR_RATIO = 4.0
    link2 = FakeLink()
    sweep2 = GimbalSelfTest(geared, link2)
    clock = 0.0
    while not sweep2.done:
        sweep2.update(clock)
        clock = max(clock, sweep2.deadline)
    assert sweep2.state == "DONE" and len(link2.moves) == 6


def test_calibrator():
    cfg = Cfg()
    cfg.CALIBRATE_AXIS = "yaw"
    link = FakeLink()
    cal = GimbalCalibrator(cfg, link)
    clock = 0.0

    def result():
        # 真值：ppp=90，正方向走 2000 脉冲 -> dx 减 22.22；回程残余 0.5px
        if len(link.moves) == 0:
            dx = 30.0
        elif len(link.moves) == 1:
            dx = 30.0 - 2000.0 / 90.0
        else:
            dx = 30.5
        return {"aim_valid": True, "dx": dx, "dy": 0.0}

    while not cal.done:
        cal.update(clock, result())
        clock = max(clock, cal.deadline) + 0.01
    assert cal.state == "DONE", cal.reason
    assert abs(cal.report["pulse_per_px"] - 90.0) < 0.5
    assert cal.report["dir_sign"] == 1
    assert abs(cal.report["cross"]) < 1e-6
    assert abs(cal.report["backlash_px"] - 0.25) < 1e-6
    assert [m[1] for m in link.moves] == [True, False]      # 正走后回位
    # 轴未配置 -> SKIPPED，不动作
    off = GimbalCalibrator(Cfg(), FakeLink())
    assert off.state == "SKIPPED" and off.done


def test_axis_mapping_config():
    """锁定实测轴映射定案（AGENTS.md §2）——防止再次被静默改回 v3 旧值而张冠李戴。

    v3 旧值（错）：yaw=ttyS1/UART1/A30A31/ID1、pitch=ttyS2/UART2/B0B1/ID2、DIR_SIGN=+1、ACC=0。
    定案（对）：   yaw=ttyS2/UART2/B0B1/ID2、pitch=ttyS0/UART0/A28A29/ID1、DIR_SIGN=-1、ACC≠0。
    """
    from config import Config
    # yaw = UART2 / B0,B1 / ttyS2 / ID2
    assert Config.YAW_DEV == "/dev/ttyS2", Config.YAW_DEV
    assert Config.YAW_PINS == {"B0": "UART2_TX", "B1": "UART2_RX"}, Config.YAW_PINS
    assert Config.ADDR_YAW == 2, Config.ADDR_YAW
    # pitch = UART0 / A28,A29 / ttyS0 / ID1
    assert Config.PITCH_DEV == "/dev/ttyS0", Config.PITCH_DEV
    assert Config.PITCH_PINS == {"A28": "UART0_TX", "A29": "UART0_RX"}, Config.PITCH_PINS
    assert Config.ADDR_PITCH == 1, Config.ADDR_PITCH
    # pinmap 必须显式下发；U0 是日志口，开机清 RX
    assert Config.PINMAP_STRICT is False and Config.FLUSH_RX_ON_START is True
    # 两轴正方向均为 CCW；闭环方向符号按几何推断取 -1（未经标定）
    assert Config.YAW_POSITIVE_CCW is True and Config.PITCH_POSITIVE_CCW is True
    assert Config.DIR_SIGN_YAW == -1 and Config.DIR_SIGN_PITCH == -1
    # acc=0 是卡顿根源，必须给非 0 曲线档位
    assert Config.ACC_DEFAULT != 0, Config.ACC_DEFAULT
    # 未标定的软件畸变校正会引入畸变 -> 默认关闭
    assert Config.LENS_CORR is False


def main():
    tests = [test_protocol, test_aim_basic, test_clamp_and_jump_reject,
             test_coarse_fine_state, test_closed_loop_convergence,
             test_reversed_sign_diverges, test_ack_watchdog,
             test_ack_timeout_gated, test_self_test, test_calibrator,
             test_axis_mapping_config]
    for fn in tests:
        fn()
        print("  OK   %s" % fn.__name__)
    print("gimbal aim/protocol/watchdog/calibrate tests: PASS (%d)" % len(tests))


if __name__ == "__main__":
    main()
