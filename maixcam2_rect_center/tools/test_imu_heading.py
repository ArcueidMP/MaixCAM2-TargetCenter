"""Deterministic gyro/mount/time tests plus bounded sensor-worker isolation.

No MaixPy hardware, UART or physical 30fps/overshoot claim is involved. Gyro
signals, bias and tilt trajectories are generated independently of the tracker.
"""
import math
import sys
import threading
import time
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vision.imu_heading import GyroHeading, HeadingFeedforward, MaixIMUSensor
import vision.imu_heading as imu_heading_module


class Config:
    ENABLE_IMU_FEEDFORWARD = True
    IMU_AXES_CALIBRATED = True
    IMU_SENSOR_MOUNT = "camera"
    IMU_YAW_AXIS = (0, 0, 1)
    IMU_PITCH_AXIS = (0, -1, 0)
    IMU_BIAS_DPS = (0, 0, 0)
    IMU_MAX_AGE_S = 0.10
    IMU_MAX_GAP_S = 0.20
    IMU_MAX_PREDICT_S = 0.05
    IMU_REFERENCE_MAX_AGE_S = 0.03
    IMU_HISTORY_S = 1.0
    IMU_HISTORY_MAX_SAMPLES = 256
    IMU_BIAS_CALIBRATE_S = 0.20


def config(**values):
    return type("TestConfig", (Config,), values)()


class Clock:
    def __init__(self, now=0.0):
        self.now = now

    def perf_counter(self):
        return self.now


def turn(rate, duration=1.0, **overrides):
    tracker = GyroHeading(config(**overrides))
    tracker.update((0, 0, rate), 0)
    assert tracker.lock_reference(0)
    for i in range(1, int(round(duration * 200)) + 1):
        tracker.update((0, 0, rate), i / 200)
    return tracker


def close(value, expected, tolerance=1e-7):
    assert abs(value - expected) <= tolerance, (value, expected, tolerance)


def test_right_angle_turns_have_opposite_compensation():
    for rate in (90.0, -90.0):
        correction = turn(rate).correction(1.0)
        assert correction["valid"]
        close(correction["yaw_deg"], -rate)
        close(correction["pitch_deg"], 0.0)


def test_turn_unwrap_crosses_180_without_direction_flip():
    tracker = turn(90.0, duration=3.0)
    close(tracker.correction(3.0)["yaw_deg"], -270.0)
    negative = turn(-90.0, duration=3.0)
    close(negative.correction(3.0)["yaw_deg"], 270.0)


def test_known_bias_and_zero_mean_noise_do_not_become_motion():
    bias = (0.4, -0.3, 0.8)
    tracker = GyroHeading(config(IMU_BIAS_DPS=bias))
    tracker.update(bias, 0.0)
    assert tracker.lock_reference(0.0)
    for i in range(1, 201):
        # Sum of this alternating noise is zero, with a small endpoint residual.
        noise = 0.03 if i % 2 else -0.03
        tracker.update((bias[0], bias[1], bias[2] + noise), i / 200.0)
    correction = tracker.correction(1.0)
    assert correction["valid"]
    close(correction["yaw_deg"], 0.0, 0.001)


def test_dropped_vision_frames_do_not_drop_imu_integration():
    clock = Clock()
    source = HeadingFeedforward(config(), clock=clock, threaded=False)
    source.ingest((0, 0, 90), 0.0)
    assert source.lock_reference(0.0)
    # There is no visual/controller observation in this full 90-degree turn.
    for i in range(1, 201):
        clock.now = i / 200.0
        source.ingest((0, 0, 90), clock.now)
    close(source.correction()["yaw_deg"], -90.0)
    source.close()


def test_camera_own_motion_is_already_measured():
    # Vehicle +90deg and gimbal -60deg => camera measured +30deg.
    tracker = turn(30.0)
    correction = tracker.correction(1.0, gimbal_deg=(-60, 0))
    close(correction["yaw_deg"], -30.0)
    # Supplying a command/position must not remove the own motion twice.
    close(tracker.correction(1.0, gimbal_deg=(-90, 0))["yaw_deg"], -30.0)


def test_base_mount_requires_position_and_uses_remaining_error():
    tracker = GyroHeading(config(IMU_SENSOR_MOUNT="base"))
    tracker.update((0, 0, 90), 0.0)
    assert not tracker.lock_reference(0.0)
    assert tracker.lock_reference(0.0, gimbal_deg={"yaw": 10, "pitch": 2})
    for i in range(1, 201):
        tracker.update((0, 0, 90), i / 200)
    assert not tracker.correction(1.0)["valid"]
    close(tracker.correction(1.0, gimbal_deg=(-50, 2))["yaw_deg"], -30.0)
    close(tracker.correction(1.0, gimbal_deg=(-80, 2))["yaw_deg"], 0.0)


def test_pitch_turn_compensates_independently():
    tracker = GyroHeading(config())
    tracker.update((0, -20, 0), 0.0)
    assert tracker.lock_reference(0.0)
    for i in range(1, 201):
        tracker.update((0, -20, 0), i / 200)
    correction = tracker.correction(1.0)
    close(correction["pitch_deg"], -20.0)
    close(correction["yaw_deg"], 0.0)


def test_tilted_camera_handles_world_vertical_right_angle_turn():
    tracker = GyroHeading(config())
    # First tilt camera up by 20deg from its calibrated home orientation.
    tracker.update((0, -20, 0), 0)
    for i in range(1, 201):
        tracker.update((0, -20, 0), i / 200)
    # Its world-up axis in current camera coordinates is (sin20, 0, cos20).
    tilt = math.radians(20)
    yaw_rates = (90 * math.sin(tilt), 0, 90 * math.cos(tilt))
    tracker.update(yaw_rates, 1.005)
    assert tracker.lock_reference(1.005)
    for i in range(1, 201):
        tracker.update(yaw_rates, 1.005 + i / 200)
    correction = tracker.correction(2.005)
    close(correction["yaw_deg"], -90.0, 0.1)
    close(correction["pitch_deg"], 0.0, 0.1)
    close(correction["rate_yaw_dps"], 90.0, 0.1)


def test_axes_mapping_and_sign_are_explicit():
    tracker = GyroHeading(config(IMU_YAW_AXIS=(0, 1, 0), IMU_PITCH_AXIS=(1, 0, 0)))
    tracker.update((0, 90, 0), 0.0)
    assert tracker.lock_reference(0.0)
    for i in range(1, 201):
        tracker.update((0, 90, 0), i / 200)
    close(tracker.correction(1.0)["yaw_deg"], -90.0)
    try:
        GyroHeading(config(IMU_PITCH_AXIS=(0, 0, 1)))
    except ValueError:
        pass
    else:
        raise AssertionError("parallel mounting axes must be rejected")


def test_uncalibrated_and_disabled_gates():
    for settings, expected in (({"ENABLE_IMU_FEEDFORWARD": False}, "DISABLED"),
                               ({"IMU_AXES_CALIBRATED": False}, "AXES_UNCALIBRATED"),
                               ({"IMU_BIAS_DPS": None}, "BIAS_UNCALIBRATED")):
        tracker = GyroHeading(config(**settings))
        tracker.update((0, 0, 90), 0.0)
        assert tracker.snapshot(0.0)["state"] == expected
        assert not tracker.lock_reference(0.0)
        assert not tracker.correction(0.0)["valid"]


def test_stale_and_future_clock_do_not_drive():
    tracker = turn(90.0)
    assert not tracker.correction(1.11)["valid"]
    assert not tracker.correction(0.99)["valid"]
    close(tracker.correction(1.11)["yaw_deg"], 0.0)


def test_long_gap_and_clock_rollback_invalidate_reference():
    for resumed_at in (1.50, 0.90):
        tracker = turn(90.0)
        tracker.update((0, 0, 90), resumed_at)
        assert not tracker.correction(resumed_at)["valid"]
        assert not tracker.snapshot(resumed_at)["reference_locked"]
        tracker.update((0, 0, 0), resumed_at + 0.01)
        assert tracker.snapshot(resumed_at + 0.01)["state"] == "READY"
        assert not tracker.correction(resumed_at + 0.01)["valid"]


def test_error_and_nonfinite_values_invalidate_reference():
    for raw in ((0, 0, math.nan), (0, 0, 500), (0, 0)):
        tracker = turn(90.0)
        tracker.update(raw, 1.005)
        assert not tracker.correction(1.005)["valid"]
        assert not tracker.snapshot(1.005)["reference_locked"]
        assert tracker.snapshot(1.005)["error"]
    tracker = turn(90.0)
    tracker.fail("sensor_disconnected")
    assert not tracker.correction(1.0)["valid"]


def test_reference_reset_requires_new_lock():
    tracker = turn(90.0)
    tracker.reset_reference()
    assert not tracker.correction(1.0)["valid"]
    assert tracker.lock_reference(1.0)
    close(tracker.correction(1.0)["yaw_deg"], 0.0)


def test_capture_time_lock_does_not_use_processing_time_pose():
    tracker = turn(90.0)
    aligned = tracker.sample_at(0.90)
    assert aligned["valid"] and aligned["sampled_at"] <= .90
    assert tracker.lock_reference(.90)
    # Frame captured at 81deg, processed at 90deg -> remaining turn is 9deg.
    close(tracker.correction(1.0)["yaw_deg"], -9.0)
    assert not tracker.sample_at(-.1)["valid"]
    assert not tracker.lock_reference(-.1)


def test_target_offset_survives_camera_motion():
    tracker = GyroHeading(config())
    tracker.update((0, 0, 10), 0.0)
    assert tracker.lock_reference(0.0, yaw_offset_deg=10.0)
    close(tracker.correction(0.0)["yaw_deg"], 10.0)
    for i in range(1, 201):
        tracker.update((0, 0, 10), i / 200)
    close(tracker.correction(1.0)["yaw_deg"], 0.0)
    tracker.update((0, 0, 0), 1.005)
    assert not tracker.lock_reference(1.005, yaw_offset_deg=math.nan)
    assert tracker.snapshot(1.005)["reference_locked"]


def test_combined_target_offsets_use_common_heading_frame():
    tracker = GyroHeading(config())
    tracker.update((0, 0, 0), 0.0)
    assert tracker.lock_reference(0.0, yaw_offset_deg=10, pitch_offset_deg=5)
    correction = tracker.correction(0.0)
    close(correction["yaw_deg"], 10)
    close(correction["pitch_deg"], 5)


def test_near_vertical_pose_does_not_emit_ambiguous_yaw():
    tracker = GyroHeading(config())
    tracker.update((0, -85, 0), 0.0)
    assert tracker.lock_reference(0.0)
    for i in range(1, 201):
        tracker.update((0, -85, 0), i / 200)
    result = tracker.correction(1.0)
    assert not result["valid"]
    assert result["state"] == "ATTITUDE_OUT_OF_RANGE"
    close(result["yaw_deg"], 0.0)


def test_bounded_history_and_prediction():
    tracker = turn(90.0, duration=3.0)
    assert len(tracker.history) <= 256
    assert not tracker.sample_at(0.0)["valid"]
    correction = tracker.correction(3.01, lead_s=2.0)
    close(correction["prediction_s"], .05)
    close(correction["yaw_deg"], -274.5)
    assert not tracker.sample_at(3.10)["valid"]


def test_explicit_bias_calibration_and_moving_rejection():
    tracker = GyroHeading(config(IMU_BIAS_DPS=None))
    tracker.start_bias_calibration(0.0)
    for i in range(41):
        tracker.update((.4, -.2, .7), i / 200)
    assert tracker.calibration_state == "READY"
    for actual, expected in zip(tracker.bias, (.4, -.2, .7)):
        close(actual, expected)
    tracker.update((.4, -.2, .7), .205)
    assert tracker.lock_reference(.205)
    tracker.start_bias_calibration(.205)
    assert not tracker.snapshot(.205)["reference_locked"]
    for i in range(42):
        tracker.update((0, 0, 90), .205 + i / 200)
    assert tracker.calibration_state == "REJECTED"
    assert tracker.bias is None
    assert not tracker.correction(.41)["valid"]


def test_sensor_fault_aborts_bias_collection():
    tracker = GyroHeading(config(IMU_BIAS_DPS=None))
    tracker.start_bias_calibration(0.0)
    tracker.update((0, 0, .5), .005)
    tracker.fail("sensor_disconnected")
    assert tracker.calibration_state == "REJECTED"
    for i in range(1, 50):
        tracker.update((0, 0, .5), i / 100.0)
    assert tracker.bias is None, "faulted calibration must not resume while motors may move"
    assert not tracker.correction(.50)["valid"]


def test_bias_collection_blocks_motion_before_axis_calibration():
    tracker = GyroHeading(config(IMU_AXES_CALIBRATED=False, IMU_BIAS_DPS=None))
    tracker.start_bias_calibration(0.0)
    assert tracker.snapshot(0.0)["state"] == "CALIBRATING"
    assert not tracker.snapshot(0.0)["valid"]
    for i in range(10):
        tracker.update((.1, .2, .3), i / 200.0)
    assert tracker.snapshot(.05)["state"] == "CALIBRATING"
    assert not tracker.lock_reference(.05)
    assert not tracker.correction(.05)["valid"]


def test_startup_bias_reservation_precedes_slow_sensor_construction():
    for supplied_bias in (None, (0.0, 0.0, 0.0)):
        entered, release = threading.Event(), threading.Event()

        class DelayedSensor:
            def __init__(self):
                entered.set()
                release.wait(2.0)
                self.bias_dps = supplied_bias

            def read(self):
                return (0.0, 0.0, 0.0)

        original_factory = imu_heading_module.MaixIMUSensor
        imu_heading_module.MaixIMUSensor = DelayedSensor
        source = HeadingFeedforward(config(IMU_BIAS_DPS=None, IMU_CALIBRATE_ON_START=True,
                                            IMU_BIAS_CALIBRATE_S=1.0))
        try:
            assert entered.wait(.50)
            status = source.sample()
            assert status["state"] == "CALIBRATING", status
            assert status["calibration"] == "PENDING_SENSOR"
            assert not source.correction()["valid"]
            assert not source.lock_reference()
            release.set()
            deadline = time.perf_counter() + .50
            while source.sample()["calibration"] == "PENDING_SENSOR" and time.perf_counter() < deadline:
                time.sleep(.001)
            status = source.sample()
            if supplied_bias is None:
                assert status["state"] == "CALIBRATING", status
                assert status["calibration"] == "COLLECTING"
            else:
                deadline = time.perf_counter() + .50
                while not source.sample()["valid"] and time.perf_counter() < deadline:
                    time.sleep(.001)
                assert source.sample()["valid"], source.sample()
        finally:
            release.set()
            source.close()
            imu_heading_module.MaixIMUSensor = original_factory


def test_manual_startup_bias_collection_finishes_without_worker():
    clock = Clock()
    source = HeadingFeedforward(config(IMU_BIAS_DPS=None, IMU_CALIBRATE_ON_START=True),
                                clock=clock, threaded=False)
    assert source.sample()["state"] == "CALIBRATING"
    assert source.sample()["calibration"] == "COLLECTING"
    source.start_bias_calibration(0.0)
    for i in range(25):
        clock.now = i / 100.0
        source.ingest((.1, .2, .3), clock.now)
    assert source.sample()["state"] == "READY", source.sample()
    assert source.sample()["valid"]
    source.close()


def test_official_adapter_shape_units_and_not_ready_marker():
    saved = {name: sys.modules.get(name) for name in ("maix", "maix.ext_dev")}
    calls = []

    class Device:
        def __init__(self, *args, **kwargs):
            calls.append((args, kwargs))
            self.raw = [0, 0, 9.80665, 1, 2, 90, 25]

        def calib_gyro_exists(self):
            return True

        def load_calib_gyro(self):
            return types.SimpleNamespace(x=.1, y=.2, z=.3)

        def read(self):
            return self.raw

    imu = types.SimpleNamespace(IMU=Device, Mode=types.SimpleNamespace(DUAL="DUAL"),
                                AccScale=types.SimpleNamespace(ACC_SCALE_2G=2),
                                AccOdr=types.SimpleNamespace(ACC_ODR_208=208),
                                GyroScale=types.SimpleNamespace(GYRO_SCALE_500DPS=500),
                                GyroOdr=types.SimpleNamespace(GYRO_ODR_208=208))
    sys.modules["maix"] = types.ModuleType("maix")
    sys.modules["maix.ext_dev"] = types.SimpleNamespace(imu=imu)
    try:
        sensor = MaixIMUSensor()
        assert calls[0][0] == ("default",)
        assert calls[0][1]["block"] is False
        assert calls[0][1]["gyro_scale"] == 500
        assert sensor.read() == (1.0, 2.0, 90.0)  # No radians conversion or bias applied twice.
        assert sensor.bias_dps == (.1, .2, .3)
        sensor.device.raw = [0] * 7
        assert sensor.read() is None
        sensor.device.raw = []
        assert sensor.read() is None
    finally:
        for name, module in saved.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module


def test_blocked_sensor_does_not_block_controller_or_shutdown():
    entered, release = threading.Event(), threading.Event()

    class BlockedSensor:
        def read(self):
            entered.set()
            release.wait(2.0)
            return (0, 0, 0)

    source = HeadingFeedforward(config(), sensor=BlockedSensor())
    try:
        assert entered.wait(.50)
        started = time.perf_counter()
        for _ in range(100):
            assert not source.sample()["valid"]
            assert not source.correction()["valid"]
        assert time.perf_counter() - started < .10
        started = time.perf_counter()
        source.close()
        assert time.perf_counter() - started < .20
        assert source.sample()["state"] == "CLOSED"
    finally:
        release.set()
        source.close()


def test_saved_bias_sensor_injection_worker_and_closed_gate():
    class Sensor:
        bias_dps = (.4, -.2, .7)

        def read(self):
            return self.bias_dps

    source = HeadingFeedforward(config(IMU_BIAS_DPS=None), sensor=Sensor())
    try:
        deadline = time.perf_counter() + .50
        while not source.sample()["valid"] and time.perf_counter() < deadline:
            time.sleep(.005)
        assert source.sample()["valid"], source.sample()
        assert source.lock_reference()
        assert source.correction()["valid"]
    finally:
        source.close()
    assert not source.correction()["valid"]
    assert not source.lock_reference()


def test_slow_read_completion_cannot_masquerade_as_fresh_sample():
    completed = threading.Event()

    class SlowSensor:
        def read(self):
            time.sleep(.025)
            completed.set()
            return (0, 0, 90)

    source = HeadingFeedforward(config(IMU_READ_MAX_S=.010), sensor=SlowSensor())
    try:
        assert completed.wait(.50)
        deadline = time.perf_counter() + .20
        while source.sample()["error"] is None and time.perf_counter() < deadline:
            time.sleep(.001)
        assert source.sample()["error"] == "imu_read_timeout"
        assert not source.sample()["valid"]
        assert not source.lock_reference()
    finally:
        source.close()


def main():
    tests = [value for name, value in sorted(globals().items()) if name.startswith("test_")]
    for test in tests:
        test()
        print("PASS", test.__name__)
    print("IMU heading PASS", len(tests))


if __name__ == "__main__":
    main()
