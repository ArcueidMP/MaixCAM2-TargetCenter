"""Single owner of UART, priority arbitration, and captured-frame freshness gates."""
import math
import threading
import time
from .gimbal_self_test import GimbalSelfTest
from .gimbal_search import SearchSweep
from .laser_spot import LaserSpotDetector, deviation
from .rectangle_center import RectangleCenterDetector


def send_uart(uart, result):
    if uart is not None:
        payload = '@[%d,%d]%%\r\n' % (round(result['u']), round(result['v'])) if result.get('valid') else '@[invalid]%%\r\n'
        uart.write(payload.encode('ascii'))


class VisionControl:
    def __init__(self, cfg, link, aim, calibrator=None, uart=None, clock=None, heading=None):
        self.cfg, self.link, self.aim, self.calibrator, self.uart = cfg, link, aim, calibrator, uart
        self._clock = clock if clock is not None else time
        self.heading = heading
        self.self_test = self.search = None
        self.request_test = threading.Event()
        self.request_search = threading.Event()
        self.request_bias = threading.Event()
        self.detector = RectangleCenterDetector(cfg)
        self.laser = LaserSpotDetector(cfg) if cfg.ENABLE_LASER else None
        self.laser_static = self.laser is not None and self.laser.mode == 'fixed'
        self.laser_state = self.laser.static_state() if self.laser_static else None
        self.last_capture = -1e9
        self.stale_stopped = True
        self.notice = ''
        self.control_source = 'vision'
        self.commands = {'yaw': None, 'pitch': None}
        self._bias_active = False
        self._auto_axis = None

    def process(self, gray, captured_at):
        started = self._clock.perf_counter()
        result = self.detector.process(gray, timestamp=captured_at)
        detected = self._clock.perf_counter()
        if self.laser is not None and not self.laser_static:
            self.laser_state = self.laser.process(gray, captured_at)
        dv = deviation(self.laser_state, result, use_raw=self.cfg.AIM_USE_RAW) if self.laser_state else None
        if dv is not None:
            result.update(laser_u=self.laser_state['u'], laser_v=self.laser_state['v'], dx=dv[0], dy=dv[1])
        result['aim_valid'] = dv is not None
        output = self.accept_result(result, captured_at)
        return dict(result=output, detector_result=result,
                    laser=dict(self.laser_state) if self.laser_state else None,
                    captured_at=captured_at, completed_at=self._clock.perf_counter(),
                    detect_ms=(detected-started)*1000, status=self.status())

    def accept_result(self, result, captured_at, now=None):
        now = self._clock.perf_counter() if now is None else float(now)
        captured_at = float(captured_at)
        fresh = (math.isfinite(now) and math.isfinite(captured_at) and
                 0 <= now-captured_at <= self.cfg.RESULT_MAX_AGE_S and captured_at > self.last_capture)
        output = dict(result) if fresh else dict(valid=False, aim_valid=False, stage='stale')
        output.update(captured_at=captured_at, age_s=now-captured_at)
        if fresh:
            self.last_capture = captured_at
        send_uart(self.uart, output)
        self._control(output, now, new_measurement=fresh)
        output.update(aim_state=self.aim.state if self.aim else None, cmds=dict(self.commands),
                      control_source=self.control_source, search_state=self.search.state if self.search else None)
        return output

    def idle(self):
        self._control(None, self._clock.perf_counter(), new_measurement=False)

    def status(self):
        if self.aim is not None and self.aim.state == 'FAULT':
            return 'FAULT'
        if self.self_test is not None and not self.self_test.done:
            return 'self-test %d/%d' % (self.self_test.index, len(self.self_test.sequence))
        if self.calibrator is not None and not self.calibrator.done:
            return 'calibrate '+self.calibrator.axis
        if self.search is not None and not self.search.done:
            return 'SEARCH %d/%d' % (self.search.index, len(self.search.sequence))
        if self.control_source == 'imu_hold':
            return 'IMU HOLD '+self.aim.state
        return self.notice or (self.aim.state if self.aim else 'vision')

    def _stop_motion(self, now, axis=None):
        if self.link is None:
            return True
        try:
            ok = self.link.stop(now=now) if axis is None else self.link.stop(axis, now=now)
        except Exception:
            ok = False
        if ok is False:
            if axis is not None:
                try:
                    self.link.stop('pitch' if axis == 'yaw' else 'yaw', now=now)
                except Exception:
                    pass
            self.aim.fault('stop:'+(axis or 'all'))
            return False
        if hasattr(self.aim, 'mark_stopped'):
            self.aim.mark_stopped(now, axis)
        return True

    def _notice(self, text):
        self.notice = text
        print('[ui] '+text)

    def _degree_scales(self):
        if not getattr(self.cfg, 'ENABLE_IMU_FEEDFORWARD', False) or self.heading is None:
            return None
        # A base-mounted sensor needs measured gimbal angles; requested pulses are not feedback.
        if getattr(self.cfg, 'IMU_MOUNT', 'gimbal') != 'gimbal':
            return None
        scales = []
        for axis in ('yaw', 'pitch'):
            ppd = float(getattr(self.cfg, axis.upper()+'_PULSES_PER_DEG', 0))
            ppp = float(getattr(self.cfg, 'PULSE_PER_PX_'+axis.upper(), 0))
            sign = float(getattr(self.cfg, 'DIR_SIGN_'+axis.upper(), 0))
            if not all(map(math.isfinite, (ppd, ppp, sign))) or min(ppd, ppp) <= 0 or abs(sign) != 1:
                return None
            scales.append(ppd/(ppp*sign))
        return scales

    def _with_heading(self, result, now, new_measurement):
        self.control_source = 'vision'
        scales = self._degree_scales()
        if scales is None or (result is not None and result.get('stage') == 'stale'):
            return result
        if new_measurement and result and result.get('aim_valid'):
            captured = result['captured_at']
            at_capture, current = self.heading.sample_at(captured), self.heading.sample_at(now)
            if at_capture is not None and current is not None and at_capture.get('epoch') == current.get('epoch'):
                self.heading.lock_reference(captured, result['dx']/scales[0], result['dy']/scales[1])
                result = dict(result)
                # Add ONLY motion since exposure; adding absolute error double counts rotation.
                result['dx'] -= ((current['yaw']-at_capture['yaw']+180)%360-180)*scales[0]
                result['dy'] -= (current['pitch']-at_capture['pitch'])*scales[1]
                self.control_source = 'vision_imu'
            return result
        if now-self.last_capture <= getattr(self.cfg, 'IMU_HOLD_MAX_S', .25):
            correction, sample = self.heading.correction(now), self.heading.sample_at(now)
            if correction is not None and sample is not None:
                self.control_source = 'imu_hold'
                return dict(aim_valid=True, dx=correction[0]*scales[0], dy=correction[1]*scales[1], captured_at=sample['captured_at'])
        return result

    def _control(self, result, now, new_measurement):
        self.commands = {'yaw': None, 'pitch': None}
        link, aim = self.link, self.aim
        if link is None or aim is None:
            return
        # FAULT before poll: even E2 re-enable is forbidden once motion is faulted.
        if aim.state == 'FAULT':
            self.request_test.clear()
            self.request_search.clear()
            return
        feedback = link.poll(now)
        if feedback and feedback.get('fault'):
            self._stop_motion(now)
            aim.fault('link:%s' % feedback.get('reason'))
            return
        busy_cal = self.calibrator is not None and not self.calibrator.done
        busy_test = self.self_test is not None and not self.self_test.done
        busy_search = self.search is not None and not self.search.done
        if self.request_test.is_set():
            self.request_test.clear()
            if busy_cal or busy_test or busy_search or self._bias_active or not self.cfg.GIMBAL_SELF_TEST:
                self._notice('SELF-TEST rejected: busy or disabled')
            elif self._stop_motion(now):
                self.self_test = GimbalSelfTest(self.cfg, link, now)
                self._notice('收到点击 -> 启动上电自检 SELF-TEST')
                busy_test = not self.self_test.done
        if self.request_search.is_set():
            self.request_search.clear()
            if busy_search:
                self.search.cancel(now)
                self._stop_motion(now)
                if self.search.state == 'FAULT':
                    aim.fault(self.search.reason)
                self._notice('SEARCH cancelled')
                busy_search = False
            elif busy_cal or busy_test or self._bias_active or not getattr(self.cfg, 'ENABLE_SEARCH_BUTTON', True):
                self._notice('SEARCH rejected: busy or disabled')
            elif self._stop_motion(now):
                self.search = SearchSweep(self.cfg, link, now, self.heading)
                self._notice('SEARCH started')
                busy_search = True
        if self.request_bias.is_set():
            self.request_bias.clear()
            if self.heading is None or busy_cal or busy_test or busy_search:
                self._notice('IMU bias rejected: unavailable or busy')
            elif self._stop_motion(now):
                self.heading.start_bias_calibration(now)
                self._bias_active = True
                self._bias_started = now
                self._notice('IMU bias: keep base stationary')
        if aim.state == 'FAULT':
            return
        if self._bias_active:
            sample = self.heading.sample_at(now)
            if sample and sample.get('bias_state') in ('READY', 'REJECTED'):
                self._bias_active = False
                self._notice('IMU bias '+sample['bias_state'])
            elif now-self._bias_started > getattr(self.cfg, 'IMU_BIAS_CALIBRATE_S', 1.)+1:
                self._bias_active = False
                self._notice('IMU bias timeout')
            return
        if busy_cal:
            self.calibrator.update(now, result if new_measurement else None)
            if self.calibrator.done:
                if self.calibrator.state == 'FAULT' and (self.calibrator.reason or '').startswith(('tx:', 'stop:', 'uart_')):
                    self._stop_motion(now)
                    aim.fault(self.calibrator.reason)
                elif self.calibrator.state == 'DONE' and self._auto_axis == 'yaw':
                    from .gimbal_calibrate import GimbalCalibrator
                    cfg = type('PitchCalibration', (self.cfg,), {'CALIBRATE_AXIS': 'pitch'})
                    self.calibrator = GimbalCalibrator(cfg, link, now)
                    self._auto_axis = 'pitch'
            return
        if busy_test:
            self.self_test.update(now)
            if self.self_test.state == 'FAULT':
                self._stop_motion(now)
                aim.fault('self-test:'+str(self.self_test.reason))
            return
        if busy_search:
            self.search.update(now, result if new_measurement else None)
            if self.search.state == 'FAULT':
                aim.fault(self.search.reason)
            elif self.search.done:
                self._stop_motion(now)
                self._notice('SEARCH '+self.search.state)
            return
        if not getattr(self.cfg, 'GIMBAL_CALIBRATED', True):
            if not self.stale_stopped:
                self._stop_motion(now)
            self.stale_stopped = True
            aim.update({'aim_valid': False}, now=now)
            return
        result = self._with_heading(result, now, new_measurement)
        if not new_measurement and self.control_source != 'imu_hold':
            if (result is not None and result.get('stage') == 'stale' or now-self.last_capture > self.cfg.RESULT_MAX_AGE_S) and not self.stale_stopped:
                self._stop_motion(now)
                aim.update({'aim_valid': False}, now=now)
                self.stale_stopped = True
            return
        if not result or not result.get('aim_valid'):
            if not self.stale_stopped:
                self._stop_motion(now)
            self.stale_stopped = True
        else:
            self.stale_stopped = False
        if aim.state == 'FAULT':
            return
        output = aim.update(result or {'aim_valid': False}, now=now)
        if aim.state == 'FAULT':
            self._stop_motion(now)
            return
        for axis in output.get('stops', ()):
            if not self._stop_motion(now, axis):
                return
        for axis, command in output['cmds'].items():
            if command is not None:
                ccw, pulses, rpm = command
                if link.move(axis, ccw, pulses, rpm, self.cfg.ACC_TRACK, now=now) is False:
                    self._stop_motion(now)
                    aim.fault('tx:'+axis)
                    return
                self.commands[axis] = command

    def close(self):
        if self.heading is not None:
            self.heading.close()
