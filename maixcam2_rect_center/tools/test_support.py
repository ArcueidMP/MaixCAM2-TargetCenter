"""Shared deterministic clocks and links for offline behavioral regressions."""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from target_api import load_config
from vision.aim_control import VisionControl
from vision.gimbal_aim import GimbalAim
from vision.stepper_link import estimate_move_seconds


class Clock:
    def __init__(self, now=0.):
        self.now = now
    def perf_counter(self):
        return self.now


class Link:
    def __init__(self, cfg=None, clock=None):
        self.cfg, self.clock = cfg or load_config(), clock or Clock()
        self.moves, self.stops = [], []
        self.fail_move = self.fail_stop = False
        self.feedback = {'fault': False}
        self.closed = False
        self.busy = {'yaw': -1e9, 'pitch': -1e9}
    def enable(self, axis=None, now=None):
        return True
    def poll(self, now=None):
        return dict(self.feedback)
    def move(self, axis, ccw, pulses, rpm, acc=0, now=None):
        now = self.clock.now if now is None else now
        self.moves.append((axis, ccw, pulses, rpm, acc, now))
        if self.fail_move:
            return False
        assert now >= self.busy[axis], 'overlapping relative position commands'
        self.busy[axis] = now+estimate_move_seconds(pulses, rpm, acc)
        return True
    def stop(self, axis=None, now=None):
        now = self.clock.now if now is None else now
        self.stops.append((axis, now))
        if self.fail_stop:
            return False
        for name in ('yaw', 'pitch') if axis is None else (axis,):
            self.busy[name] = now
        return True
    def close(self):
        self.closed = True


def control(**overrides):
    cfg = load_config(**overrides)
    clock, link = Clock(), Link(cfg)
    link.clock = clock
    result = VisionControl(cfg, link, GimbalAim(cfg), clock=clock)
    return result, link, clock


def observation(dx=30., dy=0.):
    return dict(valid=True, aim_valid=True, u=294+dx, v=202+dy, raw_u=294+dx, raw_v=202+dy,
                dx=dx, dy=dy, stage='track')
