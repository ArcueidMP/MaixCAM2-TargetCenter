"""Button-triggered nonblocking search. Cancellation stops in place, never homes."""
import math
from .stepper_link import estimate_move_seconds


class SearchSweep:
    def __init__(self, cfg, link, now=0., heading=None):
        self.cfg, self.link, self.heading = cfg, link, heading
        self.started_at = float(now)
        self.state, self.reason = 'SEARCH', None
        self.index, self.deadline = 0, float(now)
        self.sequence = []
        self.segment = None
        self.measurements = []
        for axis in ('yaw', 'pitch'):
            limit = abs(float(getattr(cfg, 'SEARCH_'+axis.upper()+'_DEG', 45.)))
            ppd = float(getattr(cfg, axis.upper()+'_PULSES_PER_DEG', 0.))
            pulses = max(1, round(limit*ppd)) if ppd > 0 else max(1, int(getattr(cfg, 'SEARCH_PULSES', 400)))
            positive = bool(getattr(cfg, axis.upper()+'_POSITIVE_CCW', True))
            for sign, factor in ((1, 1), (-1, 2), (1, 1)):
                self.sequence.append((axis, positive if sign > 0 else not positive, pulses*factor, sign*factor*limit))

    @property
    def done(self):
        return self.state != 'SEARCH'

    def _sample(self, now):
        return self.heading.sample_at(now) if self.heading is not None else None

    def _stop(self, now, state, reason):
        ok = self.link.stop(now=now)
        self.state, self.reason = (state, reason) if ok is not False else ('FAULT', 'stop:all')
        return self.state

    def cancel(self, now):
        return self._stop(now, 'CANCELLED', 'button')

    def _target_is_fresh(self, result, now):
        if not result or not result.get('valid') or result.get('held') or result.get('predicted'):
            return False
        captured = result.get('captured_at', float('-inf'))
        return math.isfinite(captured) and self.started_at <= captured <= now and now-captured <= self.cfg.RESULT_MAX_AGE_S

    def update(self, now, result=None):
        if self.done:
            return self.state
        if self._target_is_fresh(result, now):
            return self._stop(now, 'FOUND', 'visual')
        if now-self.started_at >= getattr(self.cfg, 'SEARCH_TIMEOUT_S', 60.):
            return self._stop(now, 'DONE', 'timeout')
        sample = self._sample(now)
        if self.segment is not None:
            axis, pulses, angle, initial, sent = self.segment
            measured = None
            if sample is not None and initial is not None and sample.get('epoch') == initial.get('epoch'):
                measured = (sample[axis]-initial[axis]+180)%360-180
            early = (getattr(self.cfg, 'SEARCH_USE_IMU', True) and measured is not None and measured*angle > 0 and abs(measured) >= abs(angle)-getattr(self.cfg, 'SEARCH_IMU_MARGIN_DEG', 3.))
            if now < self.deadline and not early:
                return self.state
            if early and self.link.stop(axis, now=now) is False:
                return self._stop(now, 'FAULT', 'stop:'+axis)
            if measured is not None and abs(measured) > 1:
                value = pulses/abs(measured)
                # If stopped early, requested pulses are an upper bound, not measured travel.
                self.measurements.append(dict(axis=axis, pulses_per_deg=value, upper_bound=early))
                print('[search] %s pulses/degree%s=%.3f' % (axis, ' upper-bound' if early else '', value))
            self.segment = None
            self.deadline = now+getattr(self.cfg, 'SEARCH_SETTLE_S', .1)
            return self.state
        if now < self.deadline:
            return self.state
        if self.index >= len(self.sequence):
            self.state, self.reason = 'DONE', 'sequence_complete'
            return self.state
        axis, ccw, pulses, angle = self.sequence[self.index]
        rpm, acc = getattr(self.cfg, 'SEARCH_RPM', 60), getattr(self.cfg, 'SEARCH_ACC', 60)
        if self.link.move(axis, ccw, pulses, rpm, acc, now=now) is False:
            return self._stop(now, 'FAULT', 'tx:'+axis)
        self.segment = axis, pulses, angle, sample, now
        self.deadline = now+estimate_move_seconds(pulses, rpm, acc, self.cfg.MOTOR_PULSES_PER_REV)
        self.index += 1
        return self.state
