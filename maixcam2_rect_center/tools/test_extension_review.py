"""Independent extension integration regressions found during source review.

No hardware is opened. The control clock, received frames and actual IMU rates
are injected separately so failures cannot be hidden by implementation mirrors.
"""
import contextlib
import io
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from target_api import AimLoop
from test_extension_control import (accept, initialize_reference, make_control,
                                    rotate_camera)


def test_selftest_failed_write_faults_shared_controller():
    control, link, clock, _ = make_control()
    # Legacy self-test calls omit now; preserve the test's injected clock for
    # those calls without altering the real shared scheduler or state machine.
    move, stop = link.move, link.stop
    link.move = lambda axis, ccw, pulses, rpm, acc=0, now=None: move(
        axis, ccw, pulses, rpm, acc, clock.now if now is None else now)
    link.stop = lambda axis=None, now=None: stop(
        axis, clock.now if now is None else now)
    link.reject_move = True
    control.request_test.set()
    control.idle()
    assert control.self_test.state == "FAULT"
    assert control.aim.state == "FAULT", "self-test TX failure must block ordinary tracking"
    link.reject_move = False
    clock.now = .1
    accept(control, clock, dx=20)
    control.request_search.set()
    control.idle()
    assert not link.moves, "recovered transport alone cannot clear an explicit FAULT"


def test_motor_disabled_bias_request_is_not_claimed_as_accepted():
    control, _, clock, heading = make_control(imu=True)
    loop = AimLoop(control.cfg, enable_motors=False, clock=clock, heading=heading)
    try:
        assert loop.calibrate_imu_bias() is False, \
            "without a controller capable of consuming the request, report unavailable"
        loop.idle()
        assert heading.sample(clock.now)["state"] != "CALIBRATING"
    finally:
        loop.close()


def test_selftest_waits_for_acceleration_before_reversing():
    control, link, clock, _ = make_control()
    move, stop = link.move, link.stop
    link.move = lambda axis, ccw, pulses, rpm, acc=0, now=None: move(
        axis, ccw, pulses, rpm, acc, clock.now if now is None else now)
    link.stop = lambda axis=None, now=None: stop(
        axis, clock.now if now is None else now)
    control.request_test.set()
    control.idle()
    while not control.self_test.done:
        # FakeLink independently models curve duration and rejects a segment
        # issued before the preceding segment physically reaches its endpoint.
        clock.now = control.self_test.deadline
        control.idle()
    assert control.self_test.state == "DONE"
    assert len(link.moves) == 6
    assert link.endpoints["yaw"][-1] == link.endpoints["pitch"][-1] == 0


def test_stale_imu_stops_even_after_recent_invalid_camera_frame():
    control, link, clock, heading = make_control(imu=True)
    initialize_reference(control, clock, heading)
    rotate_camera(heading, clock, 10, duration=.1)
    control.idle()
    assert link.moves and control.control_source == "imu"
    # Camera continues delivering frames but sees no black frame. Its capture
    # timestamp must not extend the lifetime of an inertial motor command.
    clock.now = .11
    accept(control, clock, valid=False)
    before = len(link.stops)
    clock.now = .205
    assert heading.sample(clock.now)["state"] == "STALE"
    assert clock.now - control.last_capture < control.cfg.RESULT_MAX_AGE_S
    assert clock.now < link.busy_until["yaw"], "fixture must still be physically moving"
    control.idle()
    assert len(link.stops) > before, "stale inertial feedback must stop an active IMU motion"
    assert control.aim.state == "HOLD"


def main():
    tests = [test_selftest_failed_write_faults_shared_controller,
             test_motor_disabled_bias_request_is_not_claimed_as_accepted,
             test_selftest_waits_for_acceleration_before_reversing,
             test_stale_imu_stops_even_after_recent_invalid_camera_frame]
    for test in tests:
        with contextlib.redirect_stdout(io.StringIO()):
            test()
        print("PASS", test.__name__)
    print("extension review: PASS (%d)" % len(tests))


if __name__ == "__main__":
    main()
