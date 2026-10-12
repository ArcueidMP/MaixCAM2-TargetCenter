"""渐进闭环回归：有运动时长、采图延迟、标定误差的确定性电机模型。

模型按 FIFO 接受相对位置段，每段从静止加速、减速至静止；它用于暴露
尚未走完就重复发相对指令的问题，不代表所有实机驱动器的排队语义。
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vision.gimbal_aim import GimbalAim
from vision.stepper_link import StepperLink, estimate_move_seconds


class Cfg:
    AIM_PROGRESSIVE = True
    AIM_WAIT_MOTION = True
    AIM_MOTION_MARGIN_S = 0.03
    AIM_HISTORY_MAX_GAP_S = 0.30
    DEADBAND_PX = 2.0
    COMMAND_INTERVAL_S = 0.08
    PULSE_PER_PX_YAW = PULSE_PER_PX_PITCH = 3.67
    DIR_SIGN_YAW = DIR_SIGN_PITCH = 1
    GAIN_YAW = GAIN_PITCH = 0.30
    GAIN_COARSE = 0.65
    COARSE_PX = 24.0
    RPM_FINE = 60
    RPM_COARSE = 150
    MAX_STEP_PULSES = 80
    ACC_DEFAULT = ACC_TRACK = 80
    MOTOR_PULSES_PER_REV = 3200
    JUMP_REJECT_PX = 400.0
    JUMP_CONFIRM_FRAMES = 3
    ENABLE_SCAN = False
    ENABLE_ACK_WATCHDOG = True
    E2_REENABLE = False
    RX_TIMEOUT_MS = 300
    RX_TIMEOUT_LIMIT = 3


def observation(dx, dy=0.0, captured_at=0.0, valid=True):
    return dict(aim_valid=valid, dx=dx, dy=dy, captured_at=captured_at)


def test_motion_duration():
    # 协议示例的 acc=80：165p 短行程应低于 1200 RPM，耗时约 330ms。
    assert abs(estimate_move_seconds(165, 1200, 80) - 0.330) < 0.001
    assert abs(estimate_move_seconds(3200, 60, 0) - 1.0) < 1e-12
    # 3200p @60RPM/acc80：1s 匀速距离加一段 60*8.8ms 的加减速余量。
    assert abs(estimate_move_seconds(3200, 60, 80) - 1.528) < 1e-12
    assert estimate_move_seconds(0, 60, 80) == 0.0
    for arguments in ((1, 0, 80), (1, 60, 80, 0), (float("nan"), 60, 80)):
        try:
            estimate_move_seconds(*arguments)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid motion estimate accepted")


def test_progressive_axes():
    values = []
    for error in (3, 5, 12, 24, 80):
        cmd = GimbalAim(Cfg()).update(observation(error), now=0)["cmds"]["yaw"]
        values.append(cmd)
    assert [c[1] for c in values] == sorted(c[1] for c in values)
    assert [c[2] for c in values] == sorted(c[2] for c in values)
    assert values[0][2] < values[-1][2]
    mixed = GimbalAim(Cfg()).update(observation(100, 3), now=0)["cmds"]
    assert mixed["yaw"][2] == Cfg.RPM_COARSE
    assert mixed["pitch"][2] < mixed["yaw"][2]
    assert all(cmd[1] <= Cfg.MAX_STEP_PULSES for cmd in mixed.values())


def test_wait_for_motion_and_capture():
    aim = GimbalAim(Cfg())
    assert aim.update(observation(50), now=0)["cmds"]["yaw"]
    deadline = aim._busy_until["yaw"]
    assert aim.update(observation(50, captured_at=deadline / 2), deadline / 2)["cmds"]["yaw"] is None
    # 已到估算时间，但图像仍在运动完成前拍摄：不得重复追加旧误差。
    assert aim.update(observation(50, captured_at=deadline - 0.01), deadline + 0.10)["cmds"]["yaw"] is None
    fresh = observation(30, captured_at=deadline + 0.01)
    assert aim.update(fresh, deadline + 0.11)["cmds"]["yaw"]
    assert aim.update(fresh, deadline + 1.0)["cmds"]["yaw"] is None
    assert aim.update(observation(30, captured_at=0.01), deadline + 2.0)["cmds"]["yaw"] is None


def test_stop_and_reset():
    aim = GimbalAim(Cfg())
    aim.update(observation(50, 20), now=0)
    in_band = aim.update(observation(0, 20, captured_at=0.03), now=0.03)
    assert in_band["stop_axes"] == ["yaw"]
    assert all(cmd is None for cmd in in_band["cmds"].values())
    assert aim.update(observation(0, 20, captured_at=0.04), now=0.04)["stop_axes"] == []
    lost = aim.update(observation(0, valid=False), now=0.05)
    assert lost["stop_axes"] == ["pitch"]
    assert lost["state"] == "HOLD"
    assert aim.update(observation(0, valid=False), now=0.06)["stop_axes"] == []
    assert aim.update(observation(-10, captured_at=0.10), now=0.10)["cmds"]["yaw"][0] is False
    aim.motion_stopped()
    assert all(value < 0 for value in aim._busy_until.values())
    assert aim._last_measurement_time is None


def test_reversal_stops_before_new_command():
    aim = GimbalAim(Cfg())
    aim.update(observation(50), now=0)
    out = aim.update(observation(-10, captured_at=0.03), now=0.03)
    assert out["stop_axes"] == ["yaw"] and out["cmds"]["yaw"] is None
    assert aim.update(observation(-10, captured_at=0.03), now=0.10)["cmds"]["yaw"] is None
    assert aim.update(observation(-10, captured_at=0.10), now=0.10)["cmds"]["yaw"][0] is False
    # 异步待处理图像虽然比触发停机的测量新，但仍在实际停机之前拍摄。
    aim = GimbalAim(Cfg())
    aim.update(observation(50), now=0)
    assert aim.update(observation(-10, captured_at=0.01), now=0.09)["stop_axes"] == ["yaw"]
    assert aim.update(observation(-10, captured_at=0.03), now=0.10)["cmds"]["yaw"] is None
    assert aim.update(observation(-10, captured_at=0.12), now=0.12)["cmds"]["yaw"] is not None
    aim.motion_stopped(now=0.15)
    assert aim.update(observation(-10, captured_at=0.14), now=0.23)["cmds"]["yaw"] is None
    assert aim.update(observation(-10, captured_at=0.24), now=0.24)["cmds"]["yaw"] is not None


def test_invalid_measurement_does_not_drive():
    for dx, captured_at in ((float("nan"), 0.03), (float("inf"), 0.03), (10, float("nan")), (10, 10)):
        aim = GimbalAim(Cfg())
        aim.update(observation(50), now=0)
        out = aim.update(observation(dx, captured_at=captured_at), now=0.03)
        assert out["state"] == "HOLD" and out["stop_axes"] == ["yaw"]
        assert all(cmd is None for cmd in out["cmds"].values())
    aim = GimbalAim(Cfg(), controller=lambda axis, error, now: float("nan"))
    assert aim.update(observation(10), now=0)["cmds"]["yaw"] is None
    aim = GimbalAim(Cfg())
    aim.update(observation(50), now=0)
    assert aim.update(observation(10), now=float("nan"))["stop_axes"] == ["yaw"]


class MotionPlant:
    """连续加减速轨迹；重复相对命令会排队，采图独立于运动调度。"""

    def __init__(self, initial, pulse_per_px):
        self.base = float(initial)
        self.pulse_per_px = float(pulse_per_px)
        self.moves = []
        self.overlap = 0

    def _travel(self, move, now):
        start, direction, distance, speed, acceleration, ramp, duration = move
        elapsed = max(0.0, min(duration, now - start))
        if elapsed < ramp:
            pulses = 0.5 * acceleration * elapsed ** 2
        elif elapsed <= duration - ramp:
            pulses = 0.5 * acceleration * ramp ** 2 + speed * (elapsed - ramp)
        else:
            pulses = distance - 0.5 * acceleration * (duration - elapsed) ** 2
        return direction * pulses / self.pulse_per_px

    def position(self, now):
        return self.base - sum(self._travel(move, now) for move in self.moves)

    def moving(self, now):
        return bool(self.moves and self.moves[-1][0] + self.moves[-1][-1] > now + 1e-9)

    def stop(self, now):
        self.base = self.position(now)
        self.moves = []

    def command(self, cmd, now, acc=80):
        ccw, distance, rpm = cmd
        start = now
        if self.moving(now):
            self.overlap += 1
            start = self.moves[-1][0] + self.moves[-1][-1]
        acceleration = 3200.0 / (60.0 * (256 - acc) * 50e-6)
        requested_speed = rpm * 3200.0 / 60.0
        # 独立运动学：峰速受总行程可用的制动距离限制。
        speed = min(requested_speed, math.sqrt(distance * acceleration))
        ramp = speed / acceleration
        duration = distance / speed + ramp
        self.moves.append((start, 1 if ccw else -1, distance, speed, acceleration, ramp, duration))


def simulate(aim, initial=50.0, truth_scale=1.0, lag_frames=3, seconds=6.0):
    dt = 1.0 / 30.0
    plant = MotionPlant(initial, Cfg.PULSE_PER_PX_YAW * truth_scale)
    positions, commands = [], []
    settled_at = None
    for i in range(int(seconds / dt) + 1):
        now = i * dt
        positions.append(plant.position(now))
        measured = positions[max(0, i - lag_frames)]
        out = aim.update(observation(measured, captured_at=now - lag_frames * dt), now=now)
        if "yaw" in out.get("stop_axes", []):
            plant.stop(now)
        cmd = out["cmds"]["yaw"]
        if cmd is not None:
            plant.command(cmd, now)
            commands.append((now, cmd))
        if settled_at is None and abs(plant.position(now)) <= Cfg.DEADBAND_PX and not plant.moving(now):
            settled_at = now
    return dict(final=plant.position(seconds), minimum=min(positions), maximum=max(positions),
                overlaps=plant.overlap, commands=commands, settled_at=settled_at)


def test_delayed_finite_motion_convergence():
    cases = []
    for initial in (50.0, -50.0):
        for truth_scale in (0.75, 1.0, 1.25):
            for lag in (0, 1, 3, 5):
                result = simulate(GimbalAim(Cfg()), initial, truth_scale, lag)
                assert abs(result["final"]) <= Cfg.DEADBAND_PX, (initial, truth_scale, lag, result)
                assert result["overlaps"] == 0, result
                assert result["minimum"] >= 0 if initial > 0 else result["maximum"] <= 0, result
                assert result["settled_at"] is not None, result
                cases.append(result)
    print("  model: 24 delayed/scale/direction cases, no command overlap or zero crossing; max settle %.2fs" %
          max(result["settled_at"] for result in cases))


def test_moving_target_then_stop():
    # 靶心以 10px/s 平移 2 秒后停住；检验运动时误差与停下后收敛，不只测固定靶。
    reports = []
    for lag in (0, 3, 5):
        aim, plant = GimbalAim(Cfg()), MotionPlant(0, Cfg.PULSE_PER_PX_YAW)
        errors, settled_at = [], None
        for i in range(181):
            now = i / 30.0
            shift = 10.0 * min(now, 2.0)
            error = plant.position(now) + shift
            errors.append(error)
            measured = errors[max(0, i - lag)]
            out = aim.update(observation(measured, captured_at=now - lag / 30.0), now=now)
            if "yaw" in out["stop_axes"]:
                plant.stop(now)
            cmd = out["cmds"]["yaw"]
            if cmd:
                plant.command(cmd, now)
            if settled_at is None and now >= 2.0 and abs(error) <= Cfg.DEADBAND_PX and not plant.moving(now):
                settled_at = now
        assert abs(errors[-1]) <= Cfg.DEADBAND_PX
        assert max(abs(error) for error in errors) < 10.0
        assert plant.overlap == 0
        assert settled_at is not None and settled_at <= 3.1
        reports.append((round(lag * 1000 / 30), max(abs(error) for error in errors[:61]), settled_at - 2.0))
    print("  moving model: (lag_ms, peak_px, settle_after_stop_s) %s" %
          [(lag, round(peak, 2), round(settle, 2)) for lag, peak, settle in reports])


def test_continuous_commands_do_not_hide_ack_timeout():
    class Port:
        def write(self, payload):
            return len(payload)

    link = StepperLink(Cfg())
    link._ports = {"yaw": Port()}
    link._ingest([("yaw", bytes.fromhex("02 FD 02 6B"))], now=0.01)
    fault = False
    for i in range(1, 31):
        now = i * 0.05
        assert link.move("yaw", True, 20, 60, 80, now=now)
        fault = fault or link._ingest([], now)["fault"]
    assert fault, "new commands indefinitely reset an unanswered command timeout"


def main():
    tests = [test_motion_duration, test_progressive_axes, test_wait_for_motion_and_capture,
             test_stop_and_reset, test_reversal_stops_before_new_command,
             test_invalid_measurement_does_not_drive, test_delayed_finite_motion_convergence,
             test_moving_target_then_stop,
             test_continuous_commands_do_not_hide_ack_timeout]
    for fn in tests:
        fn()
        print("  OK   %s" % fn.__name__)
    print("gimbal smooth/delayed tracking tests: PASS (%d)" % len(tests))


if __name__ == "__main__":
    main()
