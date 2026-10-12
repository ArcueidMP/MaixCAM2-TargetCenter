"""Closed-loop right-angle-turn regression with a continuous physical model.

The actual HeadingFeedforward, VisionControl dispatcher and GimbalAim run with
an independent finite-acceleration motor plant. Gyro signals include actual
camera/gimbal motion, never requested pulse motion. The synthetic vision input
is a calibrated bearing measurement once the target returns inside a specified
FOV; detector image accuracy is tested elsewhere. These assumed gears/FOV and
ideal driver dynamics are not a claim about an uncalibrated physical board.
FOV recovery must take at most four seconds. A separate sixteen-second window
observes the unchanged visual fine controller, whose fixed pulse cap settles
more slowly with the model's higher mechanical reduction.
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from config import Config
from vision.aim_control import VisionControl
from vision.gimbal_aim import GimbalAim
from vision.imu_heading import HeadingFeedforward


class Clock:
    def __init__(self):
        self.now = 0.0

    def perf_counter(self):
        return self.now


class DegreePlant:
    """Position-mode pulse segment: continuous acceleration/cruise/deceleration.

    Angular position/rate are derived from real partial travel. FIFO queuing is
    deliberately supported so an overlapping command would be visible as a
    regression instead of silently cancelling the previous move.
    """

    def __init__(self, pulses_per_degree, pulses_per_rev=3200):
        self.ppd = float(pulses_per_degree)
        self.ppr = float(pulses_per_rev)
        self.base = 0.0
        self.moves = []
        self.overlaps = 0

    @staticmethod
    def _travel(move, now):
        start, direction, pulses, speed, acceleration, ramp, duration = move
        elapsed = max(0.0, min(duration, now - start))
        if elapsed < ramp:
            return direction * .5 * acceleration * elapsed ** 2
        if elapsed <= duration - ramp:
            return direction * (.5 * acceleration * ramp ** 2 + speed * (elapsed - ramp))
        return direction * (pulses - .5 * acceleration * (duration - elapsed) ** 2)

    @staticmethod
    def _velocity(move, now):
        start, direction, pulses, speed, acceleration, ramp, duration = move
        elapsed = now - start
        if elapsed <= 0 or elapsed >= duration:
            return 0.0
        if elapsed < ramp:
            return direction * acceleration * elapsed
        if elapsed <= duration - ramp:
            return direction * speed
        return direction * acceleration * (duration - elapsed)

    def angle(self, now):
        return self.base + sum(self._travel(move, now) for move in self.moves) / self.ppd

    def rate(self, now):
        return sum(self._velocity(move, now) for move in self.moves) / self.ppd

    def moving(self, now):
        return bool(self.moves and self.moves[-1][0] + self.moves[-1][-1] > now + 1e-9)

    def command(self, ccw, pulses, rpm, acc, now):
        assert pulses > 0 and rpm > 0 and 0 < acc < 256
        start = now
        if self.moving(now):
            self.overlaps += 1
            start = self.moves[-1][0] + self.moves[-1][-1]
        acceleration = self.ppr / (60.0 * (256 - acc) * 50e-6)
        requested_speed = rpm * self.ppr / 60.0
        speed = min(requested_speed, math.sqrt(pulses * acceleration))
        ramp = speed / acceleration
        duration = pulses / speed + ramp
        self.moves.append((start, 1 if ccw else -1, pulses, speed, acceleration, ramp, duration))

    def stop(self, now):
        self.base = self.angle(now)
        self.moves = []


class PlantLink:
    def __init__(self, cfg, clock):
        self.clock = clock
        self.plants = {axis: DegreePlant(getattr(cfg, axis.upper() + "_PULSES_PER_DEG"))
                       for axis in ("yaw", "pitch")}
        self.commands = []
        self.stops = []

    def poll(self, now):
        return {"fault": False}

    def move(self, axis, ccw, pulses, rpm, acc, now=None):
        now = self.clock.now if now is None else now
        self.commands.append((now, axis, ccw, pulses, rpm, acc))
        self.plants[axis].command(ccw, pulses, rpm, acc, now)
        return True

    def stop(self, axis=None, now=None):
        now = self.clock.now if now is None else now
        self.stops.append((now, axis))
        for name in self.plants if axis is None else (axis,):
            self.plants[name].stop(now)
        return True


def vehicle_angle(now, direction):
    return direction * 90.0 * max(0.0, min(1.0, now - .20))


def vehicle_rate(now, direction):
    return direction * 90.0 if .20 <= now < 1.20 else 0.0


def make_config(ppd, **overrides):
    values = dict(ENABLE_IMU_FEEDFORWARD=True, IMU_AXES_CALIBRATED=True,
                  IMU_SENSOR_MOUNT="camera", IMU_BIAS_DPS=(0.0, 0.0, 0.0),
                  IMU_CALIBRATE_ON_START=False, IMU_HOLD_MAX_S=4.0,
                  IMU_MAX_STEP_DEG=30.0, IMU_GAIN=.75, IMU_RPM=180, IMU_ACC=80,
                  IMU_MAX_CORRECTION_DEG=100.0,
                  YAW_PULSES_PER_DEG=float(ppd), PITCH_PULSES_PER_DEG=float(ppd),
                  DIR_SIGN_YAW=-1, DIR_SIGN_PITCH=-1,
                  YAW_POSITIVE_CCW=True, PITCH_POSITIVE_CCW=True,
                  GIMBAL_SELF_TEST=False, ENABLE_LASER=False, ENABLE_SCAN=False,
                  PULSE_PER_PX_YAW=3.67, PULSE_PER_PX_PITCH=3.67)
    values.update(overrides)
    return type("TurnConfig", (Config,), values)()


def simulate_turn(ppd=35.0, control_hz=30, direction=1, vision_delay_s=.030,
                  seconds=16.0, **overrides):
    cfg = make_config(ppd, **overrides)
    clock = Clock()
    heading = HeadingFeedforward(cfg, clock=clock, threaded=False)
    link = PlantLink(cfg, clock)
    aim = GimbalAim(cfg)
    control = VisionControl(cfg, link, aim, None, None, clock=clock, heading=heading)
    next_control = 0.0
    reacquired_at, settled_at = None, None
    imu_commands = 0
    camera_errors = []
    # 200Hz gyro sampling remains independent of the slower controller/vision.
    try:
        for tick in range(int(round(seconds * 200)) + 1):
            now = clock.now = tick / 200.0
            camera_rate = vehicle_rate(now, direction) + link.plants["yaw"].rate(now)
            heading.ingest((0.0, 0.0, camera_rate), now)
            actual = vehicle_angle(now, direction) + link.plants["yaw"].angle(now)
            camera_errors.append(actual)
            if now + 1e-9 < next_control:
                continue
            next_control += 1.0 / control_hz
            captured = max(0.0, now - vision_delay_s)
            # Store physical capture history independently: a later motor STOP
            # resets the plant's active trajectory but cannot rewrite old images.
            capture_tick = min(tick, int(round(captured * 200)))
            captured_bearing = camera_errors[capture_tick]
            # Motion blur/occlusion hides the target throughout the body turn.
            # Detection resumes only after the turn and return to a 60deg FOV.
            visible = now < .20 or (captured >= 1.20 and abs(captured_bearing) <= 30.0)
            if visible:
                if now >= 1.20 and reacquired_at is None:
                    reacquired_at = now
                # A positive camera-left angle makes target offset right; the
                # pixel/degree sign is independently derived from calibrated PPP.
                result = dict(valid=True, aim_valid=True,
                              dx=captured_bearing * ppd / cfg.PULSE_PER_PX_YAW,
                              dy=0.0, captured_at=captured)
                control.last_capture = captured
                control._control(result, now, new_measurement=True)
            else:
                control.idle()
            if control.control_source == "imu" and any(control.last_commands.values()):
                imu_commands += 1
            error_px = actual * ppd / cfg.PULSE_PER_PX_YAW
            if (reacquired_at is not None and abs(error_px) <= cfg.DEADBAND_PX and
                    not link.plants["yaw"].moving(now) and settled_at is None):
                settled_at = now
        final = vehicle_angle(seconds, direction) + link.plants["yaw"].angle(seconds)
        return dict(ppd=ppd, hz=control_hz, direction=direction, delay=vision_delay_s,
                    reacquired_at=reacquired_at, settled_at=settled_at,
                    final_deg=final, final_px=final * ppd / cfg.PULSE_PER_PX_YAW,
                    overlaps=sum(plant.overlaps for plant in link.plants.values()),
                    imu_commands=imu_commands, commands=link.commands,
                    maximum_error_deg=max(abs(value) for value in camera_errors),
                    state=aim.state)
    finally:
        control.close()


def test_right_angle_turn_reacquisition_with_finite_motor_motion():
    reports = []
    for ppd in (10.0, 35.0, 100.0):
        for hz in (20, 30, 50):
            for direction in (1, -1):
                report = simulate_turn(ppd, hz, direction)
                diagnostic = {key: value for key, value in report.items() if key != "commands"}
                assert report["reacquired_at"] is not None and report["reacquired_at"] <= 4.0, diagnostic
                assert report["overlaps"] == 0, diagnostic
                assert report["imu_commands"] > 0, diagnostic
                assert report["settled_at"] is not None, diagnostic
                assert abs(report["final_px"]) <= Config.DEADBAND_PX, diagnostic
                assert report["state"] != "FAULT", diagnostic
                assert all(command[3] <= round(30 * ppd) for command in report["commands"]), diagnostic
                reports.append(report)
    print("  18 degree/dispatch/direction cases: max reacquire %.3fs, max settle %.3fs" %
          (max(r["reacquired_at"] for r in reports), max(r["settled_at"] for r in reports)))


def test_reacquisition_under_capture_latency():
    reports = []
    for latency in (0.0, .030, .060):
        for direction in (1, -1):
            report = simulate_turn(35, 30, direction, vision_delay_s=latency)
            diagnostic = {key: value for key, value in report.items() if key != "commands"}
            assert report["reacquired_at"] is not None and report["reacquired_at"] <= 4.0, diagnostic
            assert report["overlaps"] == 0, diagnostic
            assert report["settled_at"] is not None, diagnostic
            assert abs(report["final_px"]) <= Config.DEADBAND_PX, diagnostic
            reports.append(report)
    print("  6 latency/direction cases: max reacquire %.3fs, max settle %.3fs" %
          (max(r["reacquired_at"] for r in reports), max(r["settled_at"] for r in reports)))


def test_pixel_sized_ceiling_cannot_recover_high_reduction_turn():
    # Preserve the earlier 80pulse/150RPM envelope in physical-degree terms.
    # Its lack of overlap does not imply adequate speed for a 90degree turn.
    report = simulate_turn(35, 30, 1, IMU_MAX_STEP_DEG=80.0 / 35.0,
                           IMU_GAIN=.65, IMU_RPM=150)
    diagnostic = {key: value for key, value in report.items() if key != "commands"}
    assert report["reacquired_at"] is None, diagnostic
    assert abs(report["final_deg"]) > 30.0, diagnostic
    assert report["overlaps"] == 0, diagnostic
    assert all(command[3] <= 80 for command in report["commands"]), diagnostic
    print("  prior 80pulse envelope: final %.3fdeg outside FOV after heading-hold timeout" %
          report["final_deg"])


def main():
    tests = [value for name, value in sorted(globals().items()) if name.startswith("test_")]
    for test in tests:
        test()
        print("PASS", test.__name__)
    print("IMU turn tracking PASS", len(tests))


if __name__ == "__main__":
    main()
