"""无实机确定性寻靶回归：角度边界、串行运动、途中停车与采图屏障。"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vision.gimbal_search import SearchSweep
from vision.stepper_link import estimate_move_seconds


class Cfg:
    YAW_PULSES_PER_DEG = 12.5
    PITCH_PULSES_PER_DEG = 9.25
    GIMBAL_GEAR_RATIO = 0.0
    MOTOR_PULSES_PER_REV = 3200
    YAW_POSITIVE_CCW = True
    PITCH_POSITIVE_CCW = False
    SEARCH_YAW_DEG = SEARCH_PITCH_DEG = 45.0
    SEARCH_RPM = 60
    SEARCH_ACC = 80
    SEARCH_SETTLE_S = 0.10
    SEARCH_TIMEOUT_S = 60.0
    RESULT_MAX_AGE_S = 0.20


def cfg(**overrides):
    values = {name: getattr(Cfg, name) for name in dir(Cfg) if name.isupper()}
    values.update(overrides)
    return type("TestCfg", (), values)()


class FakeLink:
    def __init__(self, config=None, move_ok=True, stop_ok=True, raises=False):
        self.cfg = config or Cfg()
        self.moves, self.stops = [], []
        self.move_ok, self.stop_ok, self.raises = move_ok, stop_ok, raises
        self.fault_reason = None
        self.busy_until = 0.0
        self.positions = {"yaw": 0, "pitch": 0}
        self.endpoints = {"yaw": [0], "pitch": [0]}

    def move(self, axis, ccw, pulses, rpm, acc=0, now=0.0):
        assert now >= self.busy_until - 1e-9, "motion commands overlapped"
        self.moves.append((axis, ccw, pulses, rpm, acc, now))
        if self.raises:
            raise OSError("UART broken")
        if not self.move_ok:
            return False
        sign = 1 if ccw == getattr(self.cfg, axis.upper() + "_POSITIVE_CCW") else -1
        self.positions[axis] += sign * pulses
        self.endpoints[axis].append(self.positions[axis])
        self.busy_until = now + estimate_move_seconds(pulses, rpm, acc,
                                                      self.cfg.MOTOR_PULSES_PER_REV)
        return True

    def stop(self, axis=None, now=0.0):
        self.stops.append((axis, now))
        if self.stop_ok:
            self.busy_until = now
        return self.stop_ok


def frame(captured_at, **overrides):
    result = dict(valid=True, u=320.0, v=240.0, captured_at=captured_at, aim_valid=False)
    result.update(overrides)
    return result


def test_full_bounded_sweep():
    link = FakeLink()
    sweep = SearchSweep(Cfg(), link, now=0.0)
    assert not link.moves, "constructor must not move"
    now = 0.0
    while not sweep.done:
        sweep.update(now)
        now = sweep.deadline
    assert sweep.state == "DONE" and sweep.reason == "sweep_complete"
    assert len(link.moves) == 6 and len(link.stops) == 1
    for axis in ("yaw", "pitch"):
        limit = int(math.floor(45.0 * getattr(Cfg, axis.upper() + "_PULSES_PER_DEG")))
        assert link.endpoints[axis] == [0, limit, -limit, 0]
        assert all(abs(p / getattr(Cfg, axis.upper() + "_PULSES_PER_DEG")) <= 45.0
                   for p in link.endpoints[axis])
    assert sweep.positions_deg == {"yaw": 0.0, "pitch": 0.0}
    assert sweep.position_known and sweep.stopped_at == now
    original_count = len(link.moves)
    sweep.update(now + 1.0, frame(now + 1.0))
    assert len(link.moves) == original_count


def test_live_acquisition_stops_without_home():
    link = FakeLink()
    sweep = SearchSweep(Cfg(), link, now=10.0)
    sweep.update(10.0)
    assert sweep.deadline > 10.02
    assert sweep.update(10.02, frame(10.01)) == "ACQUIRED"
    # 不依赖光斑有效性；停止发生在运动估算时间之前，禁止用端点倒算回零。
    assert len(link.moves) == 1 and link.stops == [(None, 10.02)]
    assert sweep.stopped_at == 10.02 and not sweep.position_known
    sweep.update(100.0)
    sweep.cancel(100.0)
    assert len(link.moves) == 1 and len(link.stops) == 1


def test_target_before_first_motion():
    link = FakeLink()
    sweep = SearchSweep(Cfg(), link, now=5.0)
    assert sweep.update(5.01, frame(5.0)) == "ACQUIRED"
    assert not link.moves and link.stops == [(None, 5.01)]


def test_invalid_and_stale_frames_do_not_acquire():
    link = FakeLink()
    sweep = SearchSweep(Cfg(), link, now=5.0)
    sweep.update(5.0)
    results = [frame(4.99), frame(5.1), frame(5.0, valid=False),
               frame(5.01, u=float("nan")), frame(5.01, v=float("inf")),
               frame(float("nan")), frame(5.01, held=True), frame(5.01, predicted=True),
               {"valid": True, "u": 0, "v": 0}, frame(None)]
    for result in results:
        assert sweep.update(5.02, result) == "RUNNING"
    assert sweep.update(5.25, frame(5.01)) == "RUNNING"
    assert not link.stops and len(link.moves) == 1
    assert sweep.update(5.26, frame(5.25)) == "ACQUIRED"


def test_calibration_required_and_known_ratio():
    for overrides in ({"YAW_PULSES_PER_DEG": 0}, {"PITCH_PULSES_PER_DEG": 0},
                      {"YAW_PULSES_PER_DEG": float("nan")},
                      {"PITCH_PULSES_PER_DEG": float("inf")}):
        config = cfg(**overrides)
        link = FakeLink(config)
        sweep = SearchSweep(config, link)
        assert sweep.state == "UNCALIBRATED" and sweep.done
        sweep.update(0)
        assert not link.moves and not link.stops
    config = cfg(YAW_PULSES_PER_DEG=0, PITCH_PULSES_PER_DEG=0, GIMBAL_GEAR_RATIO=3)
    link = FakeLink(config)
    sweep = SearchSweep(config, link)
    assert sweep.pulses_per_degree == {"yaw": 3200 * 3 / 360, "pitch": 3200 * 3 / 360}
    assert sweep.update(0) == "RUNNING" and len(link.moves) == 1


def test_explicit_cancel_and_global_timeout():
    for end in ("cancel", "timeout"):
        config = cfg(SEARCH_TIMEOUT_S=0.05)
        link = FakeLink(config)
        sweep = SearchSweep(config, link)
        sweep.update(0)
        if end == "cancel":
            assert sweep.cancel(0.02) == "CANCELLED"
        else:
            assert sweep.update(0.05) == "DONE" and sweep.reason == "timeout"
        assert not sweep.position_known
        assert len(link.moves) == 1 and len(link.stops) == 1
        sweep.update(1.0)
        assert len(link.moves) == 1


def test_faults_stop_and_block_moves():
    for fault_source in ("link", "controller", "tx", "exception", "stop"):
        link = FakeLink(move_ok=fault_source != "tx", stop_ok=fault_source != "stop",
                        raises=fault_source == "exception")
        sweep = SearchSweep(Cfg(), link)
        sweep.update(0.0)
        if fault_source == "link":
            link.fault_reason = "ack_timeout:yaw"
            sweep.update(0.02, frame(0.02))
        elif fault_source == "controller":
            sweep.fail("link:e2_flood", 0.02)
        elif fault_source == "stop":
            sweep.update(0.02, frame(0.02))
        assert sweep.state == "FAULT" and len(link.stops) == 1
        move_count = len(link.moves)
        sweep.update(100)
        assert len(link.moves) == move_count
        if fault_source == "stop":
            assert sweep.stopped_at is None and sweep.stop_attempted_at == 0.02


def test_invalid_config_and_time():
    for overrides in ({"SEARCH_YAW_DEG": 46}, {"SEARCH_PITCH_DEG": 0},
                      {"SEARCH_RPM": 0}, {"SEARCH_RPM": 5001},
                      {"SEARCH_ACC": 256}, {"SEARCH_SETTLE_S": -1},
                      {"SEARCH_TIMEOUT_S": float("nan")}, {"MOTOR_PULSES_PER_REV": 0}):
        config = cfg(**overrides)
        link = FakeLink(config)
        sweep = SearchSweep(config, link)
        assert sweep.state == "FAULT"
        sweep.update(0)
        assert not link.moves
    for invalid_now in (float("nan"), float("inf"), -1.0, None):
        link = FakeLink()
        sweep = SearchSweep(Cfg(), link)
        sweep.update(0.0)
        assert sweep.update(invalid_now) == "FAULT"
        assert link.stops == [(None, 0.0)]
    assert SearchSweep(Cfg(), FakeLink(), now=float("nan")).state == "FAULT"


def test_external_stop_time_cannot_move_barrier_backwards():
    for terminal in ("cancel", "fault"):
        link = FakeLink()
        sweep = SearchSweep(Cfg(), link, now=10.0)
        sweep.update(10.0)
        sweep.update(10.1)
        if terminal == "cancel":
            sweep.cancel(9.0)
        else:
            sweep.fail("link:lost", 9.0)
        assert sweep.stopped_at == 10.1 and link.stops == [(None, 10.1)]


def main():
    tests = [value for name, value in sorted(globals().items()) if name.startswith("test_")]
    for test in tests:
        test()
    print("gimbal search: PASS (%d)" % len(tests))


if __name__ == "__main__":
    main()
