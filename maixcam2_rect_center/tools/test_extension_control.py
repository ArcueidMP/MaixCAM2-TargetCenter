"""SEARCH/IMU 与真实控制调度器的确定性回归，无板端 SDK 或串口。

视觉结果与采图时刻独立注入；IMU 输入是实际相机角速度，运动指令仅由
FakeLink 记录，绝不拿已请求脉冲代替相机实测姿态。
"""
import contextlib
import io
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config import Config
from target_api import AimLoop
from test_target_api import scene
from vision.aim_control import VisionControl
from vision.gimbal_aim import GimbalAim
from vision.imu_heading import HeadingFeedforward
from vision.stepper_link import estimate_move_seconds


class Clock:
    def __init__(self, now=0.0):
        self.now = float(now)

    def perf_counter(self):
        return self.now


class FakeLink:
    def __init__(self, cfg):
        self.cfg = cfg
        self.feedback = {"fault": False}
        self.fault_reason = None
        self.moves, self.stops = [], []
        self.busy_until = {"yaw": -math.inf, "pitch": -math.inf}
        self.endpoints = {"yaw": [0], "pitch": [0]}
        self.reject_move = self.reject_stop = False

    def poll(self, now):
        return dict(self.feedback)

    def move(self, axis, ccw, pulses, rpm, acc=0, now=None):
        assert now >= self.busy_until[axis] - 1e-9, "same-axis commands overlap"
        if self.reject_move:
            return False
        self.moves.append((axis, ccw, pulses, rpm, acc, now))
        direction = 1 if ccw == getattr(self.cfg, axis.upper() + "_POSITIVE_CCW") else -1
        self.endpoints[axis].append(self.endpoints[axis][-1] + direction * pulses)
        self.busy_until[axis] = now + estimate_move_seconds(
            pulses, rpm, acc, self.cfg.MOTOR_PULSES_PER_REV)
        return True

    def stop(self, axis=None, now=None):
        self.stops.append((axis, now))
        if self.reject_stop:
            return False
        for name in ("yaw", "pitch") if axis is None else (axis,):
            self.busy_until[name] = now
        return True


def make_control(imu=False, **overrides):
    values = dict(ENABLE_IMU_FEEDFORWARD=imu, IMU_AXES_CALIBRATED=True,
                  IMU_BIAS_DPS=(0.0, 0.0, 0.0), IMU_LEAD_S=0.0,
                  IMU_BIAS_CALIBRATE_S=0.20, YAW_PULSES_PER_DEG=35.0,
                  PITCH_PULSES_PER_DEG=35.0, PULSE_PER_PX_YAW=3.67,
                  PULSE_PER_PX_PITCH=3.67, DIR_SIGN_YAW=-1, DIR_SIGN_PITCH=-1,
                  YAW_POSITIVE_CCW=True, PITCH_POSITIVE_CCW=True,
                  ENABLE_SCAN=False, ENABLE_LASER=False)
    values.update(overrides)
    cfg = type("IntegrationCfg", (Config,), values)()
    clock, link = Clock(), FakeLink(cfg)
    heading = HeadingFeedforward(cfg, clock=clock, threaded=False) if imu else None
    control = VisionControl(cfg, link, GimbalAim(cfg), None, None, clock=clock,
                            heading=heading)
    return control, link, clock, heading


def accept(control, clock, captured_at=None, valid=True, dx=0.0, dy=0.0, aim_valid=None):
    captured_at = clock.now if captured_at is None else captured_at
    result = dict(valid=valid, aim_valid=valid if aim_valid is None else aim_valid,
                  u=320.0 + dx, v=240.0 + dy, dx=dx, dy=dy,
                  raw_u=320.0 + dx, raw_v=240.0 + dy)
    return control.accept_result(result, captured_at)


def rotate_camera(heading, clock, degrees, duration=1.0):
    """零速端点的梯形角速度，独立积分的实际转角恰为 degrees。

    采样间隔 <=10ms，远低于 IMU_MAX_GAP_S。首末各半个间隔加减速，
    中间速率由独立积分反算，避免分段换向的梯形积分误差。
    """
    intervals = max(10, int(round(duration / 0.01)))
    dt = duration / intervals
    start = clock.now
    rate = degrees / (duration - dt)
    for index in range(1, intervals + 1):
        clock.now = start + index * dt
        heading.ingest((0.0, 0.0, rate if index < intervals else 0.0), clock.now)


def initialize_reference(control, clock, heading):
    heading.ingest((0, 0, 0), clock.now)
    accept(control, clock)
    assert heading.sample(clock.now)["reference_locked"]


def test_manual_search_blank_covers_both_axes():
    control, link, clock, _ = make_control()
    control.request_search.set()
    accept(control, clock, valid=False)
    assert control.search.state == "RUNNING" and len(link.moves) == 1
    while not control.search.done:
        assert sum(clock.now < end for end in link.busy_until.values()) <= 1
        clock.now = control.search.deadline
        control.idle()
    assert control.search.reason == "sweep_complete" and len(link.moves) == 6
    for axis in ("yaw", "pitch"):
        assert link.endpoints[axis] == [0, 1575, -1575, 0]
    assert control.aim.state == "HOLD"


def test_search_acquire_interrupts_motion_and_blocks_pre_stop_capture():
    control, link, clock, _ = make_control()
    control.request_search.set()
    accept(control, clock, valid=False)
    clock.now = 0.02
    accept(control, clock, captured_at=0.01, dx=20)
    assert control.search.state == "ACQUIRED" and len(link.moves) == 1
    assert control.search.stopped_at == 0.02 and link.stops[-1] == (None, 0.02)
    clock.now = 0.025
    accept(control, clock, captured_at=0.015, dx=20)
    assert len(link.moves) == 1, "排队的停机前画面不能立即触发追踪动作"
    clock.now = 0.03
    accept(control, clock, dx=20)
    assert len(link.moves) == 2 and link.moves[-1][0] == "yaw"


def test_search_finds_black_frame_without_laser():
    control, link, clock, _ = make_control()
    control.request_search.set()
    control.idle()
    clock.now = 0.02
    accept(control, clock, aim_valid=False)
    assert control.search.state == "ACQUIRED" and len(link.moves) == 1


def test_search_toggle_cancel_does_not_home():
    control, link, clock, _ = make_control()
    control.request_search.set()
    control.idle()
    clock.now = 0.02
    control.request_search.set()
    control.idle()
    assert control.search.state == "CANCELLED" and len(link.moves) == 1
    assert not control.search.position_known
    clock.now = 20.0
    control.idle()
    assert len(link.moves) == 1, "取消不能按请求终点反算回零"


def test_uncalibrated_search_is_visible_and_cannot_move():
    control, link, clock, _ = make_control(YAW_PULSES_PER_DEG=0.0,
                                          PITCH_PULSES_PER_DEG=0.0,
                                          GIMBAL_GEAR_RATIO=0.0)
    control.request_search.set()
    control.idle()
    assert control.search.state == "UNCALIBRATED" and not link.moves
    assert "UNCALIBRATED" in control.status()


def test_pre_button_and_stale_results_do_not_find_target():
    control, link, clock, _ = make_control()
    clock.now = 1.0
    control.request_search.set()
    control.idle()
    clock.now = 1.02
    accept(control, clock, captured_at=0.99)
    assert control.search.state == "RUNNING"
    clock.now = 1.25
    accept(control, clock, captured_at=1.01)
    assert control.search.state == "RUNNING" and len(link.moves) == 1
    accept(control, clock)
    assert control.search.state == "ACQUIRED"


def test_link_fault_blocks_search_selftest_and_imu_motion():
    control, link, clock, heading = make_control(imu=True)
    initialize_reference(control, clock, heading)
    rotate_camera(heading, clock, 20.0, duration=0.10)
    link.feedback = {"fault": True, "reason": "ack_timeout:yaw"}
    control.request_search.set()
    control.request_test.set()
    control.request_imu_bias.set()
    control.idle()
    assert control.aim.state == "FAULT" and not link.moves
    assert control.self_test is None and control.search is None
    assert heading.sample(clock.now)["state"] != "CALIBRATING"
    link.feedback = {"fault": False}
    control.request_search.set()
    control.request_test.set()
    control.idle()
    assert not link.moves


def test_fault_interrupts_active_search_and_stop_failure_is_terminal():
    for fail_stop in (False, True):
        control, link, clock, _ = make_control()
        control.request_search.set()
        control.idle()
        clock.now = 0.02
        link.reject_stop = fail_stop
        link.feedback = {"fault": True, "reason": "e2_flood"}
        control.idle()
        assert control.aim.state == "FAULT" and control.search.state == "FAULT"
        clock.now = 10.0
        control.idle()
        assert len(link.moves) == 1


def test_camera_measures_vehicle_plus_actual_gimbal_once():
    control, link, clock, heading = make_control(imu=True, IMU_HOLD_MAX_S=2.1)
    initialize_reference(control, clock, heading)
    # 小车左转 +90°，云台实际右转 -60°；相机 IMU 最终测得 +30°。
    rotate_camera(heading, clock, 90.0)
    rotate_camera(heading, clock, -60.0)
    correction = heading.correction(clock.now)
    assert abs(correction["yaw_deg"] + 30.0) < 1e-7
    control.idle()
    assert control.control_source == "imu" and len(link.moves) == 1
    assert link.moves[-1][0] == "yaw" and link.moves[-1][1] is False
    # 电机方向继续向右；不能重复减去 -60°，也不能补偿原始 +90°。
    expected_px = 30.0 * 35.0 / 3.67
    assert abs(control.aim.last["dx"] - expected_px) < 1e-7


def test_fresh_visual_has_one_command_and_calibrated_reference_offset():
    control, link, clock, heading = make_control(imu=True)
    initialize_reference(control, clock, heading)
    rotate_camera(heading, clock, 10.0, duration=0.10)
    accept(control, clock, dx=15.0, dy=-7.0)
    assert control.control_source == "vision"
    assert [cmd[0] for cmd in link.moves] == ["yaw", "pitch"]
    assert all(cmd[-1] == clock.now for cmd in link.moves)
    correction = heading.correction(clock.now)
    assert abs(correction["yaw_deg"] - (-15.0 * 3.67 / 35.0)) < 1e-7
    assert abs(correction["pitch_deg"] - (7.0 * 3.67 / 35.0)) < 1e-7


def test_stale_imu_or_expired_reference_stops_motion():
    for stale in (True, False):
        control, link, clock, heading = make_control(imu=True, IMU_HOLD_MAX_S=0.25)
        initialize_reference(control, clock, heading)
        rotate_camera(heading, clock, 10.0, duration=0.10)
        control.idle()
        assert len(link.moves) == 1 and control.control_source == "imu"
        stop_count = len(link.stops)
        if stale:
            clock.now = 0.21
        else:
            for i in range(1, 17):
                clock.now = 0.10 + i * 0.01
                heading.ingest((0, 0, 0), clock.now)
        control.idle()
        assert len(link.moves) == 1 and len(link.stops) > stop_count
        assert control.aim.state == "HOLD"


def test_inertial_motion_requires_sensor_and_motor_calibration():
    for overrides in ({"IMU_AXES_CALIBRATED": False}, {"IMU_BIAS_DPS": None},
                      {"YAW_PULSES_PER_DEG": 0.0, "GIMBAL_GEAR_RATIO": 0.0}):
        control, link, clock, heading = make_control(imu=True, **overrides)
        heading.ingest((0, 0, 0), 0.0)
        accept(control, clock)
        rotate_camera(heading, clock, 10.0, duration=0.10)
        control.idle()
        assert not link.moves and control.control_source != "imu"
        if "YAW_PULSES_PER_DEG" in overrides:
            assert "calibration required" in control.status()


def test_public_api_queues_search_and_real_detection_hands_off():
    prototype, link, clock, _ = make_control()
    loop = AimLoop(prototype.cfg, enable_motors=False, with_laser=False, clock=clock)
    loop.link, loop.aim = link, prototype.aim
    assert loop.request_search() and not link.moves and not link.stops
    blank = np.full((480, 640), 200, np.uint8)
    result = loop.step(blank, now=clock.now)
    assert result["search_state"] == "RUNNING" and len(link.moves) == 1
    assert result["control_source"] == "search"
    clock.now = 0.02
    target, _ = scene(np.array([[230, 110], [410, 110], [410, 370], [230, 370]], np.float32))
    result = loop.step(target, now=clock.now)
    assert result["valid"] and not result["aim_valid"]
    assert result["search_state"] == "ACQUIRED" and len(link.moves) == 1
    assert loop._controller.search.stopped_at == clock.now
    loop.close()


def test_public_api_stop_cancels_without_homing():
    prototype, link, clock, _ = make_control()
    loop = AimLoop(prototype.cfg, enable_motors=False, with_laser=False, clock=clock)
    loop.link, loop.aim = link, prototype.aim
    assert loop.request_search()
    loop.idle()
    clock.now = 0.02
    assert loop.stop()
    assert loop._controller.search.state == "CANCELLED"
    clock.now = 5.0
    loop.idle()
    assert len(link.moves) == 1
    loop.close()


def test_post_search_pre_stop_capture_cannot_relock_imu():
    control, link, clock, heading = make_control(imu=True)
    heading.ingest((0, 0, 0), 0.0)
    control.request_search.set()
    control.idle()
    clock.now = 0.02
    heading.ingest((0, 0, 0), clock.now)
    accept(control, clock, captured_at=0.01, dx=20)
    assert control.search.state == "ACQUIRED"
    clock.now = 0.025
    heading.ingest((0, 0, 0), clock.now)
    accept(control, clock, captured_at=0.015, dx=20)
    assert not heading.sample(clock.now)["reference_locked"], \
        "停机前拍的排队图像不能给 IMU 重新锁参考"
    assert len(link.moves) == 1
    clock.now = 0.03
    heading.ingest((0, 0, 0), clock.now)
    accept(control, clock)
    assert heading.sample(clock.now)["reference_locked"]


def test_out_of_order_visual_frame_cannot_replace_imu_reference():
    control, link, clock, heading = make_control(imu=True)
    initialize_reference(control, clock, heading)
    rotate_camera(heading, clock, 10.0, duration=0.10)
    accept(control, clock, dx=10.0)
    assert control.last_visual_reference == 0.10
    clock.now = 0.11
    heading.ingest((0, 0, 0), clock.now)
    for captured in (0.08, 0.10):
        accept(control, clock, captured_at=captured, dx=40.0)
        assert control.last_visual_reference == 0.10
        assert heading.sample(clock.now)["reference_at"] == 0.10
    assert len(link.moves) == 1


def test_future_capture_cannot_postpone_inertial_updates():
    control, link, clock, heading = make_control(imu=True)
    initialize_reference(control, clock, heading)
    rotate_camera(heading, clock, 10.0, duration=0.10)
    clock.now = 0.11
    heading.ingest((0, 0, 0), clock.now)
    accept(control, clock, captured_at=1000.0, valid=False)
    assert control.last_capture == 0.0, "未来采图不能将视觉间隔推到未来"
    clock.now = 0.12
    heading.ingest((0, 0, 0), clock.now)
    control.idle()
    assert control.control_source == "imu"


def test_bias_calibration_stops_and_blocks_all_motion_requests():
    for request in (None, "request_test", "request_search"):
        control, link, clock, heading = make_control(imu=True)
        heading.ingest((0, 0, 0), 0.0)
        accept(control, clock, dx=20)
        assert len(link.moves) == 1
        clock.now = 0.01
        control.request_imu_bias.set()
        control.idle()
        assert heading.sample(clock.now)["state"] == "CALIBRATING"
        assert link.stops[-1] == (None, 0.01)
        if request:
            getattr(control, request).set()
        clock.now = 0.02
        accept(control, clock, dx=20)
        clock.now = 0.03
        control.idle()
        assert len(link.moves) == 1, "IMU 静置零偏标定期间禁止自检/SEARCH/视觉运动"


def test_uncalibrated_axes_cannot_hide_active_bias_collection():
    control, link, clock, heading = make_control(imu=True, IMU_AXES_CALIBRATED=False)
    heading.ingest((0, 0, 0), 0.0)
    control.request_imu_bias.set()
    control.idle()
    assert heading.sample(clock.now)["calibration"] == "COLLECTING"
    clock.now = 0.01
    control.request_search.set()
    accept(control, clock, valid=False)
    assert not link.moves, "未定安装轴时，COLLECTING 仍是静止偏置标定"


def main():
    tests = [value for name, value in sorted(globals().items()) if name.startswith("test_")]
    for test in tests:
        with contextlib.redirect_stdout(io.StringIO()):
            test()
    print("extension control: PASS (%d)" % len(tests))


if __name__ == "__main__":
    main()
