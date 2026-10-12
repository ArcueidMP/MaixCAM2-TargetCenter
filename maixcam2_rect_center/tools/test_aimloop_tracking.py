"""AimLoop 的视觉到电机集成回归，不需要 MaixPy 或真实串口。

用独立 OpenCV 投影的靶纸驱动真实 CenterTracker / GimbalAim，FakeLink 只记录硬件动作。
正常用例继承实际 Config；停机中断用例仅降低转速，使尚未结束的运动可重复观察。
clock 的处理时刻与 step(now=采图时刻) 独立控制，验证过期和未来图像不得驱动电机。
"""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from config import Config
from target_api import AimLoop, load_config
from vision.aim_control import VisionControl
from vision.gimbal_aim import GimbalAim
from test_target_api import scene


class Clock:
    def __init__(self, now=10.0):
        self.now = float(now)

    def perf_counter(self):
        return self.now


class FakeLink:
    def __init__(self):
        self.feedback = {"fault": False}
        self.reject_move = False
        self.reject_stops = set()
        self.move_result = True
        self.attempts, self.moves, self.stops, self.events = [], [], [], []

    def poll(self, now):
        self.events.append(("poll", now))
        return dict(self.feedback)

    def move(self, axis, ccw, pulses, rpm, acc=0, now=None):
        command = (axis, ccw, pulses, rpm, acc, now)
        self.attempts.append(command)
        self.events.append(("move", command))
        if self.reject_move:
            return False
        self.moves.append(command)
        return self.move_result

    def stop(self, axis=None, now=None):
        self.stops.append((axis, now))
        self.events.append(("stop", axis, now))
        return axis not in self.reject_stops


def target(dx=50.0, dy=40.0):
    """140×210px 靶纸，靶心相对固定光斑的指定误差来自独立投影。"""
    u = Config.LASER_U_NOMINAL + dx
    v = Config.LASER_V_NOMINAL + dy
    q = np.array([[u - 70, v - 105], [u + 70, v - 105],
                  [u + 70, v + 105], [u - 70, v + 105]], np.float32)
    return scene(q)[0]


def make_loop(**overrides):
    cfg = load_config(**overrides)
    clock, link = Clock(), FakeLink()
    # 不创建 StepperLink：仍走真实 AimLoop.step / CenterTracker / GimbalAim。
    loop = AimLoop(cfg, enable_motors=False, clock=clock)
    loop.link, loop.aim = link, GimbalAim(cfg)
    return loop, link, clock


def slow_loop():
    return make_loop(RPM_FINE=6, RPM_COARSE=6, AIM_WAIT_MOTION=True)


def test_fault_feedback_blocks_first_move():
    loop, link, clock = make_loop()
    link.feedback = {"fault": True, "reason": "ack_timeout"}
    res = loop.step(target(), now=clock.now)
    assert res.aim_state == "FAULT", res
    assert not link.attempts, "已存在的链路故障必须在第一条运动指令前被消费"
    assert link.stops, "链路故障应立即停机"
    link.feedback = {"fault": False}
    clock.now += .5
    assert loop.step(target(), now=clock.now).aim_state == "FAULT"
    assert not link.attempts, "反馈恢复后 FAULT 仍须阻断运动，直到显式复位"


def test_lost_target_stops_once_then_reacquires():
    loop, link, clock = slow_loop()
    first = loop.step(target(), now=clock.now)
    assert first.aim_valid and len(link.moves) == 2, first
    count = len(link.moves)
    clock.now += .01
    lost = loop.step(None, now=clock.now)
    assert not lost.aim_valid and lost.aim_state == "HOLD", lost
    assert link.stops == [(None, clock.now)], link.stops
    clock.now += .01
    assert loop.step(None, now=clock.now).aim_state == "HOLD"
    assert len(link.stops) == 1 and len(link.moves) == count
    # 已超过发帧间隔，但原来慢速动作本应仍在执行；停机必须清掉原运动排程。
    clock.now = 10.12
    recovered = loop.step(target(), now=clock.now)
    assert recovered.aim_valid and recovered.stage == "full_search", recovered
    assert len(link.moves) > count, "重捕获不能被失锁前的旧运动排程挡住"


def test_explicit_stop_clears_motion_schedule():
    loop, link, clock = slow_loop()
    loop.step(target(), now=clock.now)
    count = len(link.moves)
    assert count == 2
    loop.stop()
    assert len(link.stops) == 1
    clock.now = 10.12
    res = loop.step(target(), now=clock.now)
    assert res.aim_valid and len(link.moves) > count, \
        "外部 stop() 之后不能继续沿用已经取消的运动等待"


def test_deadband_axis_stops_are_forwarded_once():
    loop, link, clock = slow_loop()
    loop.step(target(), now=clock.now)
    count = len(link.moves)
    assert count == 2
    clock.now += .03
    settled = loop.step(target(0, 0), now=clock.now)
    assert settled.aim_state == "SETTLE" and settled.aim_valid, settled
    assert sorted(axis for axis, _ in link.stops) == ["pitch", "yaw"], link.stops
    assert len(link.moves) == count, "进入死区要停在途动作，不能追加相对位移"
    clock.now += .01
    loop.step(target(0, 0), now=clock.now)
    assert len(link.stops) == 2, "同一已停轴不应每帧重复发送 stop"


def test_rejected_move_faults_without_claiming_success():
    loop, link, clock = make_loop()
    link.reject_move = True
    res = loop.step(target(), now=clock.now)
    assert len(link.attempts) == 1, "任一轴写入失败后，不能继续发送另一轴运动"
    assert not link.moves and not any(res.cmds.values()), res
    assert res.aim_state == "FAULT" and len(link.stops) == 1, res
    link.reject_move = False
    clock.now += .5
    later = loop.step(target(), now=clock.now)
    assert later.aim_state == "FAULT" and len(link.attempts) == 1


def assert_failed_stop_blocks_future_moves(axis):
    loop, link, clock = slow_loop()
    loop.step(target(), now=clock.now)
    count = len(link.attempts)
    assert count == 2
    link.reject_stops.add(axis)
    clock.now += .03
    frame = None if axis is None else target(0, 0)
    failed = loop.step(frame, now=clock.now)
    assert any(stopped_axis == axis for stopped_axis, _ in link.stops)
    assert failed.aim_state == "FAULT" and not any(failed.cmds.values()), failed
    assert len(link.attempts) == count, "停机写入失败时不能把电机当作已停止"
    link.reject_stops.clear()
    for timestamp in (10.12, 12.0):
        clock.now = timestamp
        later = loop.step(target(-50, -40), now=clock.now)
        assert later.aim_state == "FAULT" and len(link.attempts) == count, later


def test_failed_axis_stop_latches_fault():
    assert_failed_stop_blocks_future_moves("yaw")


def test_failed_all_stop_latches_fault():
    assert_failed_stop_blocks_future_moves(None)


def test_runtime_failed_stops_latch_fault():
    """板端 VisionControl 也必须处理写入失败，不能只修 AimLoop 门面。"""
    cfg = load_config(RPM_FINE=6, RPM_COARSE=6, AIM_WAIT_MOTION=True)
    for axis in ("yaw", None):
        clock, link = Clock(), FakeLink()
        aim = GimbalAim(cfg)
        control = VisionControl(cfg, link, aim, None, None, clock=clock)
        first = control.process(target(), clock.now)
        assert first["result"]["aim_valid"] and len(link.attempts) == 2
        count = len(link.attempts)
        link.reject_stops.add(axis)
        clock.now += .03
        frame = np.full((480, 640), 175, np.uint8) if axis is None else target(0, 0)
        failed = control.process(frame, clock.now)
        assert any(stopped_axis == axis for stopped_axis, _ in link.stops), link.stops
        assert aim.state == "FAULT" and failed["status"] == "FAULT", failed
        assert len(link.attempts) == count
        link.reject_stops.clear()
        for timestamp in (10.12, 12.0):
            clock.now = timestamp
            control.process(target(-50, -40), clock.now)
            assert aim.state == "FAULT" and len(link.attempts) == count


def test_pending_capture_before_stop_cannot_reverse_motor():
    loop, link, clock = slow_loop()
    loop.step(target(), now=clock.now)
    count = len(link.moves)
    assert count == 2
    clock.now = 10.08
    assert loop.stop() is True
    clock.now = 10.12
    pending = loop.step(target(-50, -40), now=10.06)
    assert pending.aim_valid, "待处理图像仍在有效期内，不能靠过期检查代替停机时刻屏障"
    assert len(link.moves) == count and not any(pending.cmds.values()), pending
    clock.now = 10.14
    fresh = loop.step(target(-50, -40), now=clock.now)
    assert fresh.aim_valid and len(link.moves) > count, "停机后的新图像才能恢复运动"


def test_legacy_none_move_result_remains_compatible():
    loop, link, clock = make_loop()
    link.move_result = None
    res = loop.step(target(), now=clock.now)
    assert res.aim_valid and res.aim_state != "FAULT", res
    assert len(link.moves) == 2 and all(res.cmds.values()), res
    assert not link.stops


def test_old_capture_time_blocks_motion():
    loop, link, clock = make_loop()
    captured = clock.now - Config.RESULT_MAX_AGE_S - .01
    res = loop.step(target(), now=captured)
    assert res.valid and not res.aim_valid, res
    assert res.captured_at == captured and res.age_s > Config.RESULT_MAX_AGE_S, res
    assert res.aim_state == "HOLD" and not link.moves and not any(res.cmds.values())
    clock.now += .1
    fresh = loop.step(target(), now=clock.now)
    assert fresh.aim_valid and link.moves, "处理时刻的新图像应能正常恢复追踪"


def test_future_capture_time_blocks_motion():
    loop, link, clock = make_loop()
    res = loop.step(target(), now=clock.now + .01)
    assert res.valid and not res.aim_valid, res
    assert res.aim_state == "HOLD" and not link.moves and not any(res.cmds.values())


def main():
    cv2.setNumThreads(Config.OPENCV_THREADS)
    tests = [test_fault_feedback_blocks_first_move,
             test_lost_target_stops_once_then_reacquires,
             test_explicit_stop_clears_motion_schedule,
             test_deadband_axis_stops_are_forwarded_once,
             test_rejected_move_faults_without_claiming_success,
             test_failed_axis_stop_latches_fault,
             test_failed_all_stop_latches_fault,
             test_runtime_failed_stops_latch_fault,
             test_pending_capture_before_stop_cannot_reverse_motor,
             test_legacy_none_move_result_remains_compatible,
             test_old_capture_time_blocks_motion,
             test_future_capture_time_blocks_motion]
    for test in tests:
        test()
        print("PASS", test.__name__)
    print("AimLoop tracking PASS", len(tests))


if __name__ == "__main__":
    main()
