"""B 轨云台闭环离线回归：协议帧 / 控制律 / 状态机 / 链路看门狗 / 开环标定。

纯 PC，不依赖 MaixPy；覆盖需求文档 §8.1 要求的"假电机收敛仿真"与
"DIR_SIGN 反了会发散"两个最关键用例。
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vision.stepper_link import (StepperLink, axis_cfg, cmd_enable, cmd_position,
                                 cmd_speed, cmd_stop_now, cmd_sync_motion, frame_status,
                                 fault_reason)
from vision.gimbal_aim import GimbalAim
from vision.gimbal_self_test import GimbalSelfTest, estimate_move_seconds
from vision.gimbal_calibrate import GimbalCalibrator


class Cfg:
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
    SELF_TEST_RPM = 60
    SELF_TEST_ACC = 60
    SELF_TEST_SETTLE_S = 0.8
    SELF_TEST_START_DELAY_S = 0.5
    SELF_TEST_PULSES = 400
    SELF_TEST_FALLBACK_PULSES = 0
    MOTOR_PULSES_PER_REV = 3200
    ACC_DEFAULT = 0
    YAW_DEV, YAW_BAUD = "/dev/ttyS0", 115200
    YAW_PINS = {"A28": "UART0_TX", "A29": "UART0_RX"}
    PINMAP_STRICT = False
    PITCH_DEV, PITCH_BAUD = "/dev/ttyS2", 115200
    PITCH_PINS = {"B0": "UART2_TX", "B1": "UART2_RX"}
    FLUSH_RX_ON_START = True
    ADDR_YAW = ADDR_PITCH = 1
    ENABLE_ACK_WATCHDOG = True
    E2_REENABLE = True
    E2_WINDOW_S = 2.0
    E2_FAULT_COUNT = 10
    RX_TIMEOUT_MS = 300
    RX_TIMEOUT_LIMIT = 3
    DIVERGE_GUARD = False
    DIVERGE_FRAMES = 30
    DIVERGE_RATIO = 1.5
    CALIBRATE_AXIS = ""
    AUTO_CALIBRATE_AXES = ("yaw", "pitch")
    AUTO_CALIBRATE_TIMEOUT_S = 150.0
    CALIBRATE_PULSES = 2000
    CALIBRATE_RPM = 60
    CALIBRATE_ACC = 60
    CALIBRATE_SAMPLES = 3
    CALIBRATE_SETTLE_S = 0.2
    CALIBRATE_TIMEOUT_S = 60.0
    CALIBRATE_VISUAL_TIMEOUT_S = 0.0


class FakeLink:
    def __init__(self, axes=("yaw", "pitch"), fail=(), info=None, report_axes=True):
        self.moves = []
        self.axes = tuple(axes)
        self.fail = set(fail)
        self.axis_info = info
        self.report_axes = report_axes

    def available_axes(self):
        return list(self.axes)

    def move(self, axis, ccw, pulses, rpm, acc=0, now=None):
        if axis in self.fail:
            return False
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


def test_fault_reason_contract():
    """锁定"无故障"的判定口径 —— 2026-10-09 实机 bug 的回归用例。

    bug 现场：main.py 写的是 `link_fault is not None`，而 ``_ingest()`` 无故障时返回
    ``fault=False``；``False is not None`` 恒为 True -> **开机第 1 帧就把云台判成 FAULT
    并停机**，``self_test.update()`` 再也没被调用过。现象：电机上电有劲、但永不自检、
    永不追踪（与串口模式/接线无关，所以怎么改硬件都没用）。
    """
    link = StepperLink(Cfg())
    # 正常一帧（无回帧、无 E2）-> _ingest 约定 fault=False，但语义是"无故障"
    fb = link._ingest([], 1.0)
    assert fb["fault"] is False
    assert fault_reason(fb) is None            # ★ 修复点：None 才是"无故障"
    assert fault_reason(None) is None
    assert fault_reason({}) is None
    assert fault_reason({"fault": False, "reason": None}) is None
    assert fault_reason({"fault": True, "reason": "ack_timeout:yaw"}) == "ack_timeout:yaw"
    # 没有故障时，自检必须真的把 4 步都发出去（pitch 上/下 + yaw 左/右；假链路 = 两轴都可用）
    fake = FakeLink()
    sweep = GimbalSelfTest(Cfg(), fake)
    assert sweep.state == "RUNNING"
    clock = 0.0
    while not sweep.done:
        assert fault_reason(link._ingest([], clock)) is None
        sweep.update(clock)
        clock = max(clock, sweep.deadline)
    assert sweep.state == "DONE" and len(fake.moves) == 4


def test_self_test():
    """默认走「固定脉冲」自检：pitch 上→下、yaw 左→右，各回到起始位姿（无需靶子与标定）。"""
    link = FakeLink()
    sweep = GimbalSelfTest(Cfg(), link)
    clock = 0.0
    while not sweep.done:
        sweep.update(clock)
        clock = max(clock, sweep.deadline)
    assert sweep.state == "DONE"
    assert [m[:3] for m in link.moves] == [
        ("pitch", False, 400), ("pitch", True, 400),     # 先 pitch：上转 -> 下转
        ("yaw", False, 400), ("yaw", True, 400)]         # 再 yaw：左转 -> 右转
    # 固定脉冲置 0 -> 回退到"按脉冲/度换算角度"的老路径（6 步，先 yaw 后 pitch）
    deg = type("DegMode", (), dict(Cfg.__dict__))
    deg.SELF_TEST_PULSES = 0
    link_deg = FakeLink()
    sweep_deg = GimbalSelfTest(deg, link_deg)
    clock = 0.0
    while not sweep_deg.done:
        sweep_deg.update(clock)
        clock = max(clock, sweep_deg.deadline)
    assert [m[:3] for m in link_deg.moves] == [
        ("yaw", False, 150), ("yaw", True, 300), ("yaw", False, 150),
        ("pitch", False, 600), ("pitch", True, 1200), ("pitch", False, 600)]
    # 角度路径：未标定且无减速比 -> SKIPPED（一条指令都不发）
    no_cal = type("Uncalibrated", (), dict(Cfg.__dict__))
    no_cal.SELF_TEST_PULSES = 0
    no_cal.YAW_PULSES_PER_DEG = 0
    no_cal.PITCH_PULSES_PER_DEG = 0
    quiet = GimbalSelfTest(no_cal, FakeLink())
    assert quiet.state == "SKIPPED" and quiet.reason == "pulse_per_degree_not_calibrated"
    # 角度路径 + 减速比 -> 可推导出比例，走完 6 步
    geared = type("Geared", (), dict(Cfg.__dict__))
    geared.SELF_TEST_PULSES = 0
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


def test_self_test_survives_dead_axis():
    """一条轴写不出去，另一轴必须走完自检。

    旧实现遇到第一条 yaw 指令写失败就整机 FAULT，pitch 一步指令都收不到 ——
    现场表现正是"U2 电机上电只使能、不自校"。这条用例把这个行为锁死。
    """
    link = FakeLink(fail=("yaw",))
    sweep = GimbalSelfTest(Cfg(), link)
    assert sweep.state == "RUNNING"
    clock = 0.0
    while not sweep.done:
        sweep.update(clock)
        clock = max(clock, sweep.deadline)
    assert sweep.state == "DONE", sweep.reason
    # 固定脉冲模式下 yaw 两条都写失败 -> 只剩 pitch 的上转/下转两段
    assert [m[:3] for m in link.moves] == [
        ("pitch", False, 400), ("pitch", True, 400)]


def test_self_test_skips_unavailable_axis():
    """链路报告某轴没打开 -> 该轴不进自检序列，另一轴照常走完。"""
    link = FakeLink(axes=("pitch",))
    sweep = GimbalSelfTest(Cfg(), link)
    assert sweep.state == "RUNNING"
    clock = 0.0
    while not sweep.done:
        sweep.update(clock)
        clock = max(clock, sweep.deadline)
    assert sweep.state == "DONE"
    assert {m[0] for m in link.moves} == {"pitch"} and len(link.moves) == 2
    # 两轴都没打开 -> SKIPPED，一条指令都不发
    dead = FakeLink(axes=())
    quiet = GimbalSelfTest(Cfg(), dead)
    assert quiet.state == "SKIPPED" and quiet.reason == "no_available_axis"
    assert dead.moves == []
    # 两轴都写失败 -> FAULT
    both_bad = FakeLink(fail=("yaw", "pitch"))
    bad = GimbalSelfTest(Cfg(), both_bad)
    clock = 0.0
    while not bad.done:
        bad.update(clock)
        clock = max(clock, bad.deadline)
    assert bad.state == "FAULT" and bad.reason == "uart_write_failed"


def test_estimate_move_seconds():
    """运动时间估算：acc=0 直接启停（快到离谱=卡顿根源）；acc>0 按官方曲线公式显著变慢。"""
    ppr = 3200.0
    # acc=0 -> 纯匀速：400 脉冲 @60RPM = 400/(3200*60/60) = 0.125s
    assert abs(estimate_move_seconds(400, 60, 0, ppr) - 0.125) < 1e-6
    quick = estimate_move_seconds(400, 60, 0, ppr)
    slow = estimate_move_seconds(400, 60, 60, ppr)
    assert slow > 3 * quick, (quick, slow)
    # acc 越小越柔和 -> 时间越长
    assert estimate_move_seconds(400, 60, 30, ppr) > estimate_move_seconds(400, 60, 120, ppr)
    # 行程短到未达设定速度 -> 退化为三角曲线：时间只由行程/加速度决定，与设定速度无关
    assert abs(estimate_move_seconds(400, 30, 60, ppr)
               - estimate_move_seconds(400, 60, 60, ppr)) < 1e-9
    # 行程足够长（能加到设定速度）时，转速越低时间越长
    assert estimate_move_seconds(20000, 30, 60, ppr) > estimate_move_seconds(20000, 60, 60, ppr)
    # 零脉冲不产生时间
    assert estimate_move_seconds(0, 60, 60, ppr) == 0.0


def test_self_test_uses_curve():
    """自检必须以慢速 + 非零加速度档位下发，否则位置模式直接启停会造成明显卡顿。"""
    cfg = Cfg()
    link = FakeLink()
    sweep = GimbalSelfTest(cfg, link)
    clock = 0.0
    while not sweep.done:
        sweep.update(clock)
        clock = max(clock, sweep.deadline)
    assert len(link.moves) == 4
    assert {m[3] for m in link.moves} == {cfg.SELF_TEST_RPM}   # 转速统一
    assert {m[4] for m in link.moves} == {cfg.SELF_TEST_ACC}   # 加速度档位非 0 = 走曲线
    assert cfg.SELF_TEST_ACC > 0 and cfg.SELF_TEST_RPM <= 100


def test_axis_cfg_mapping():
    """yaw 走 UART0(ttyS0)、pitch 走 UART2(ttyS2)，两轴均显式 pinmap；旧命名与空表兼容。"""
    cfg = Cfg()
    assert axis_cfg(cfg, "yaw") == ("/dev/ttyS0", 115200,
                                    {"A28": "UART0_TX", "A29": "UART0_RX"})
    assert axis_cfg(cfg, "pitch") == ("/dev/ttyS2", 115200,
                                      {"B0": "UART2_TX", "B1": "UART2_RX"})
    # pins 显式传空表 -> 该轴不做 pinmap（"默认已复用"的口仍可这样配）
    blank = type("BlankPins", (), dict(Cfg.__dict__))
    blank.YAW_PINS = {}
    assert axis_cfg(blank, "yaw") == ("/dev/ttyS0", 115200, {})
    old = type("OldCfg", (), dict(U1_DEV="/dev/ttyS1", U1_BAUD=115200,
                                  U1_PINS={"A30": "UART1_TX", "A31": "UART1_RX"},
                                  U2_DEV="/dev/ttyS2", U2_BAUD=115200,
                                  U2_PINS={"B0": "UART2_TX", "B1": "UART2_RX"}))
    assert axis_cfg(old, "yaw") == ("/dev/ttyS1", 115200,
                                    {"A30": "UART1_TX", "A31": "UART1_RX"})
    assert axis_cfg(old, "pitch")[0] == "/dev/ttyS2"


def test_uncalibrated_lock():
    """未标定时闭环必须锁定：不发指令（防正反馈甩头），但视觉结果照常透出。"""
    from vision.gimbal_aim import UNCALIBRATED
    cfg = Cfg()
    cfg.AIM_REQUIRE_CALIBRATION = True
    cfg.GIMBAL_CALIBRATED = False
    aim = GimbalAim(cfg)
    out = aim.update({"aim_valid": True, "dx": 50, "dy": -30}, now=0)
    assert out["state"] == UNCALIBRATED
    assert out["cmds"] == {"yaw": None, "pitch": None}
    assert out["dx"] == 50 and out["dy"] == -30
    # 标定完成 -> 解锁，恢复出指令
    cfg2 = Cfg()
    cfg2.AIM_REQUIRE_CALIBRATION = True
    cfg2.GIMBAL_CALIBRATED = True
    aim2 = GimbalAim(cfg2)
    assert aim2.update({"aim_valid": True, "dx": 50, "dy": -30}, now=0)["cmds"]["yaw"] is not None
    # 默认（不要求标定）行为不变
    assert GimbalAim(Cfg()).update({"aim_valid": True, "dx": 50, "dy": 0},
                                   now=0)["cmds"]["yaw"] is not None


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


def test_auto_calibrate_success():
    """auto 模式：两轴依次标定成功 -> 写回运行时 cfg，且 applied=True（允许解锁闭环）。"""
    from vision.gimbal_calibrate import AutoCalibrate
    cfg = Cfg()
    link = FakeLink()
    auto = AutoCalibrate(cfg, link)
    clock = 0.0

    def result():
        # yaw 真值 2000/90 ≈ 22.22px、pitch 真值 2000/50 = 40px；正方向走 -> 误差减小
        n = len(link.moves)
        dx, dy = 30.0, 20.0
        if n == 1:
            dx = 30.0 - 2000.0 / 90.0
        elif n == 3:
            dy = 20.0 - 2000.0 / 50.0
        return {"aim_valid": True, "dx": dx, "dy": dy}

    guard = 0
    while not auto.done and guard < 5000:
        auto.update(clock, result())
        clock = max(clock, auto.deadline) + 0.01
        guard += 1
    assert auto.state == "DONE", (auto.state, auto.reason, auto.failed)
    assert auto.applied and not auto.failed
    assert abs(cfg.PULSE_PER_PX_YAW - 90.0) < 0.5
    assert abs(cfg.PULSE_PER_PX_PITCH - 50.0) < 0.5
    assert cfg.DIR_SIGN_YAW == 1 and cfg.DIR_SIGN_PITCH == 1
    assert [m[0] for m in link.moves] == ["yaw", "yaw", "pitch", "pitch"]


def test_auto_calibrate_failure_keeps_locked():
    """任一轴标不出来（画面没靶心 / 无位移）时绝不能解锁闭环 —— 宁可不追，不用错参数甩头。"""
    from vision.gimbal_calibrate import AutoCalibrate
    cfg = Cfg()
    link = FakeLink()
    auto = AutoCalibrate(cfg, link, axes=("yaw",))
    clock = 0.0
    flat = {"aim_valid": True, "dx": 30.0, "dy": 20.0}   # 走了脉冲但图像不动 -> no_motion
    guard = 0
    while not auto.done and guard < 5000:
        auto.update(clock, flat)
        clock = max(clock, auto.deadline) + 0.01
        guard += 1
    assert auto.state == "FAILED" and auto.reason == "no_axis_calibrated"
    assert not auto.applied and "yaw" in auto.failed
    # 自动标定整体超时同样不解锁
    cfg2 = Cfg()
    cfg2.AUTO_CALIBRATE_TIMEOUT_S = 1.0
    auto2 = AutoCalibrate(cfg2, FakeLink(), axes=("yaw",))
    assert auto2.update(99.0, flat) == "FAILED"
    assert not auto2.applied


def test_unlock_after_calibration():
    """未标定锁 -> unlock() 后恢复发电机指令（自动标定成功后的解锁路径）。"""
    from vision.gimbal_aim import UNCALIBRATED
    cfg = Cfg()
    cfg.AIM_REQUIRE_CALIBRATION = True
    cfg.GIMBAL_CALIBRATED = False
    locked = GimbalAim(cfg)
    assert locked.state == UNCALIBRATED
    assert locked.update({"aim_valid": True, "dx": 50, "dy": -30}, now=0)["cmds"]["yaw"] is None
    assert locked.unlock("auto-calibrate") is True
    out = locked.update({"aim_valid": True, "dx": 50, "dy": -30}, now=1.0)
    assert out["state"] != UNCALIBRATED and out["cmds"]["yaw"] is not None
    assert locked.unlock("again") is False          # 幂等：已解锁后再调不重复生效


def test_track_uses_curve_config():
    """闭环发送点必须用 ACC_TRACK（且非 0）。

    位置模式 acc=0 是"直接以设定速度启停"，追踪时每 80ms 一次硬启停会让激光在停下瞬间
    晃开（现场反馈"急停对不准"）。这条同时是 **main.py 被并发覆盖的探针** ——
    旧版 main.py 用的是 ACC_DEFAULT(=0)。
    """
    root = Path(__file__).resolve().parents[1]
    src = (root / "main.py").read_text(encoding="utf-8")
    assert "Config.ACC_TRACK" in src, "main.py 的闭环发送点未使用 ACC_TRACK（追踪会急停）"
    api = (root / "target_api.py").read_text(encoding="utf-8")
    assert "ACC_TRACK" in api, "target_api 的闭环发送点未使用 ACC_TRACK（追踪会急停）"
    from config import Config
    assert int(getattr(Config, "ACC_TRACK", 0)) > 0, "ACC_TRACK 必须非 0"
    assert int(getattr(Config, "RPM_FINE", 0)) <= 200, "精调转速应偏慢，避免冲击"


def test_deg_per_px_macro():
    """「像素->角度」手感宏优先于旧的 PULSE_PER_PX_*；宏置 0 才回退（现场调参入口）。"""
    cfg = Cfg()
    cfg.DEG_PER_PX_YAW = 0.25
    cfg.PULSES_PER_DEG_YAW = 8.0
    assert abs(GimbalAim(cfg)._pulse_per_px("yaw") - 2.0) < 1e-9      # 0.25 × 8.0
    back = Cfg()
    back.DEG_PER_PX_YAW = 0.0
    back.PULSE_PER_PX_YAW = 3.67
    assert abs(GimbalAim(back)._pulse_per_px("yaw") - 3.67) < 1e-9    # 回退旧口径
    # 真实 config 里宏必须已启用，且 GAIN 已放缓（防"调节过于激烈/超调"回归）
    from config import Config
    assert float(getattr(Config, "DEG_PER_PX_YAW", 0)) > 0
    assert float(getattr(Config, "DEG_PER_PX_PITCH", 0)) > 0
    assert float(getattr(Config, "PULSES_PER_DEG_YAW", 0)) > 0
    for name in ("GAIN_YAW", "GAIN_PITCH", "GAIN_COARSE"):
        assert 0 < float(getattr(Config, name)) <= 0.6, "%s 过大易超调" % name


def test_calibrator_fails_fast_without_vision():
    """画面里没有靶心时，标定必须**尽快失败**，而不是干等到 CALIBRATE_TIMEOUT_S=20s。

    这是"上电自校卡住"的根因修复回归：标定每一步都要 dx/dy，`if dv is None: return`
    会让状态机在"没靶心 / 抬头后黑框出视野"时一直不动。
    """
    from vision.gimbal_calibrate import GimbalCalibrator
    cfg = Cfg()
    cfg.CALIBRATE_AXIS = "yaw"
    cfg.CALIBRATE_TIMEOUT_S = 20.0
    cfg.CALIBRATE_VISUAL_TIMEOUT_S = 3.0
    cal = GimbalCalibrator(cfg, FakeLink())
    blank = {"aim_valid": False, "dx": None, "dy": None}
    clock = 0.0
    while not cal.done and clock < 30.0:
        cal.update(clock, blank)
        clock += 0.1
    assert cal.state == "FAULT", (cal.state, cal.reason)
    assert "no_vision" in (cal.reason or ""), cal.reason
    assert clock <= 5.0, "应在 visual_timeout 附近失败，而不是拖到 20s：clock=%.1f" % clock
    # 视觉持续有效时不得被 visual_timeout 误杀（dx 恒定 -> 只会因 no_motion 失败）
    cfg_ok = Cfg()
    cfg_ok.CALIBRATE_AXIS = "yaw"
    cfg_ok.CALIBRATE_VISUAL_TIMEOUT_S = 3.0
    ok = GimbalCalibrator(cfg_ok, FakeLink())
    clock = 0.0
    while not ok.done and clock < 60.0:
        ok.update(clock, {"aim_valid": True, "dx": 30.0, "dy": 0.0})
        clock += 0.05
    assert "no_vision" not in (ok.reason or ""), ok.reason


def test_diverge_guard():
    """持续发指令却越追越偏 -> 自动 FAULT（防 DIR_SIGN 反了把云台一路冲向限位）。"""
    cfg = Cfg()
    cfg.DIVERGE_GUARD = True
    cfg.DIVERGE_FRAMES = 5
    cfg.DIVERGE_RATIO = 1.5
    cfg.COMMAND_INTERVAL_S = 0.0
    aim = GimbalAim(cfg)
    err = 10.0
    seen = []
    for i in range(20):
        seen.append(aim.update({"aim_valid": True, "dx": err, "dy": 0.0},
                               now=float(i))["state"])
        err += 10.0                      # 误差一路变大 = 不收敛
    assert "FAULT" in seen, seen
    assert "diverge" in (aim.fault_reason or ""), aim.fault_reason
    # 正常收敛（误差单调变小）不得误判
    ok = GimbalAim(cfg)
    err = 100.0
    for i in range(20):
        ok.update({"aim_valid": True, "dx": err, "dy": 0.0}, now=float(i))
        err = max(1.0, err * 0.8)
    assert ok.state != "FAULT", ok.fault_reason
    # 误差在死区内也不参与判断
    quiet = GimbalAim(cfg)
    for i in range(20):
        quiet.update({"aim_valid": True, "dx": 1.0, "dy": 0.0}, now=float(i))
    assert quiet.state != "FAULT"


def _smooth_config(**overrides):
    from config import Config
    return type("SmoothConfig", (Config,), dict(DIVERGE_GUARD=False, **overrides))()


def test_output_filter_spike_and_frame_rate():
    """同一真实时间下不同帧率的低通结果相同；单帧大误差不能变成满幅转动。"""
    cfg = _smooth_config()
    aim = GimbalAim(cfg)
    first = aim.update({"aim_valid": True, "dx": 250, "dy": 0}, 0)
    assert 0 < first["cmds"]["yaw"][1] <= 4, first
    for i in range(1, 61):
        aim.update({"aim_valid": True, "dx": 10, "dy": 0}, i / 30)
    baseline = aim._output["yaw"]
    spike = aim.update({"aim_valid": True, "dx": 250, "dy": 0}, 61 / 30)
    assert spike["filtered_pulses"]["yaw"] - baseline <= cfg.AIM_OUTPUT_SLEW_PPS / 30 + 1e-9
    after = aim.update({"aim_valid": True, "dx": 10, "dy": 0}, 62 / 30)
    assert abs(after["filtered_pulses"]["yaw"]) <= 10 * aim._pulse_per_px("yaw") * cfg.GAIN_YAW + 1e-9
    # 独立验证时间常数：禁用另一道增长限速，起点先建立零状态。
    values = []
    for fps in (20, 40, 80):
        filtered = GimbalAim(_smooth_config(AIM_OUTPUT_SLEW_PPS=0))
        filtered._output_time["yaw"] = 0.0
        for i in range(1, fps + 1):
            filtered._filter_output("yaw", 60, i / fps)
        values.append(filtered._output["yaw"])
    assert max(values) - min(values) < 1e-9, values


def test_filtered_motion_cancels_on_settle_reverse_and_loss():
    for next_result in ({"aim_valid": True, "dx": 1, "dy": 0},
                        {"aim_valid": True, "dx": -100, "dy": 0},
                        {"aim_valid": False}):
        aim = GimbalAim(_smooth_config())
        aim.update({"aim_valid": True, "dx": 100, "dy": 0}, 0)
        assert aim._motion_until["yaw"] > 0.02
        out = aim.update(next_result, 0.02)
        assert out["stops"] == ["yaw"], out
        assert out["cmds"]["yaw"] is None, out
        if next_result.get("dx", 0) < 0:
            reverse = aim.update(next_result, 0.13)
            assert reverse["cmds"]["yaw"][0] is True  # DIR_SIGN=-1，负误差发 CCW
            assert reverse["cmds"]["yaw"][1] <= 20, reverse
        else:
            assert out["filtered_pulses"]["yaw"] == 0
            resumed = aim.update({"aim_valid": True, "dx": 100, "dy": 0}, 10)
            assert resumed["cmds"]["yaw"][1] <= 4, resumed
    # 一轴进入死区也必须清零，不能被另一轴尚在追踪掩盖。
    aim = GimbalAim(_smooth_config())
    aim.update({"aim_valid": True, "dx": 100, "dy": 100}, 0)
    out = aim.update({"aim_valid": True, "dx": 1, "dy": 100}, 0.02)
    assert out["stops"] == ["yaw"] and out["filtered_pulses"]["yaw"] == 0, out


def test_smooth_closed_loop_with_moving_motor():
    """按曲线时间逐帧推进电机，不允许在未完成运动上叠加位置帧，并验证大误差收敛。"""
    cfg = _smooth_config()
    aim = GimbalAim(cfg)
    ppp = cfg.DEG_PER_PX_YAW * cfg.PULSES_PER_DEG_YAW
    x, dt, active, end, commands = 150.0, 0.01, None, 0.0, 0
    last_sent = 0.0
    for i in range(2500):
        now = i * dt
        if active is not None:
            # 同一曲线时间内匀速分摊脉冲，模拟尚未走完的有限行程。
            start, duration, initial, correction = active
            x = initial + correction * min(1.0, (now - start) / duration)
            if now >= end:
                active = None
        out = aim.update({"aim_valid": True, "dx": x, "dy": 0}, now)
        if "yaw" in out["stops"]:
            active = None
        cmd = out["cmds"]["yaw"]
        if cmd is not None:
            assert active is None, (now, end, cmd)
            ccw, pulses, rpm = cmd
            assert pulses <= cfg.MAX_STEP_PULSES
            commands += 1
            last_sent = now
            duration = estimate_move_seconds(pulses, rpm, cfg.ACC_TRACK, cfg.MOTOR_PULSES_PER_REV)
            end = now + duration
            correction = (1 if ccw else -1) * pulses / ppp  # 当前 sign=-1
            active = (now, duration, x, correction)
        assert x >= -cfg.DEADBAND_PX, x
        if out["state"] == "SETTLE":
            break
    assert aim.state == "SETTLE" and abs(x) <= cfg.DEADBAND_PX, (aim.state, x)
    assert commands > 5 and last_sent > 1.0
    print("  smooth finite-motion: commands=%d final=%.2fpx time=%.2fs" % (commands, x, now))


def test_smooth_reversed_direction_still_faults():
    cfg = _smooth_config(DIVERGE_FRAMES=5)
    cfg.DIVERGE_GUARD = True
    aim = GimbalAim(cfg)
    x = 30.0
    for i in range(100):
        out = aim.update({"aim_valid": True, "dx": x, "dy": 0}, i * 0.1)
        cmd = out["cmds"]["yaw"]
        if cmd is not None:
            x += cmd[1] / (cfg.DEG_PER_PX_YAW * cfg.PULSES_PER_DEG_YAW)
        if out["state"] == "FAULT":
            break
    assert out["state"] == "FAULT" and "diverge" in aim.fault_reason, (x, out)


def main():
    tests = [test_protocol, test_aim_basic, test_clamp_and_jump_reject,
             test_coarse_fine_state, test_closed_loop_convergence,
             test_reversed_sign_diverges, test_ack_watchdog,
             test_ack_timeout_gated, test_fault_reason_contract, test_self_test,
             test_self_test_survives_dead_axis, test_self_test_skips_unavailable_axis,
             test_estimate_move_seconds, test_self_test_uses_curve,
             test_axis_cfg_mapping, test_uncalibrated_lock, test_calibrator,
             test_auto_calibrate_success, test_auto_calibrate_failure_keeps_locked,
             test_unlock_after_calibration, test_track_uses_curve_config,
             test_deg_per_px_macro, test_diverge_guard,
             test_calibrator_fails_fast_without_vision,
             test_output_filter_spike_and_frame_rate,
             test_filtered_motion_cancels_on_settle_reverse_and_loss,
             test_smooth_closed_loop_with_moving_motor,
             test_smooth_reversed_direction_still_faults]
    for fn in tests:
        fn()
        print("  OK   %s" % fn.__name__)
    print("gimbal aim/protocol/watchdog/calibrate tests: PASS (%d)" % len(tests))


if __name__ == "__main__":
    main()
