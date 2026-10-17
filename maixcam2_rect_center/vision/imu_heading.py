"""Relative gyro heading, never an absolute compass. Native I/O has one owner."""
import math
import threading
import time
from collections import deque


def _mul(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return (w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v)


def _unit(v):
    n = math.sqrt(sum(x*x for x in v))
    if not math.isfinite(n) or abs(n-1) > 1e-6:
        raise ValueError('IMU axes must be unit vectors')
    return tuple(v)


def open_imu():
    from maix.ext_dev import imu
    return imu.IMU('lsm6dsowtr', mode=imu.Mode.DUAL,
                   gyro_scale=imu.GyroScale.GYRO_SCALE_500DPS, gyro_odr=imu.GyroOdr.GYRO_ODR_208, block=False)


def read_gyro(device):
    raw = device.read()
    if raw is None or len(raw) != 7 or not any(raw):
        return None  # DUAL seven-value all-zero sample means not ready, not zero rotation.
    gyro = tuple(float(v) for v in raw[3:6])
    return gyro if all(map(math.isfinite, gyro)) else None


class GyroHeading:
    def __init__(self, cfg, clock=None):
        self.cfg, self.clock = cfg, clock if clock is not None else time
        self.yaw_axis = _unit(getattr(cfg, 'IMU_YAW_AXIS', (0, 0, 1)))
        self.pitch_axis = _unit(getattr(cfg, 'IMU_PITCH_AXIS', (0, -1, 0)))
        if abs(sum(a*b for a,b in zip(self.yaw_axis, self.pitch_axis))) > 1e-6:
            raise ValueError('IMU axes must be orthogonal')
        self.q = (1., 0., 0., 0.)
        self.previous = self.timestamp = None
        self.bias = (0., 0., 0.)
        self.history = deque(maxlen=512)
        self.reference = None
        self.epoch = 0
        self.bias_state = 'IDLE'
        self._bias_samples = []
        self._bias_started = None

    def reset_reference(self):
        self.reference = None

    def start_bias_calibration(self, now=None):
        # Caller must stop both motors and ensure the base is stationary.
        self._bias_started = self.clock.perf_counter() if now is None else float(now)
        self._bias_samples = []
        self.bias_state = 'CALIBRATING'
        self.reset_reference()

    def _advance(self, gyro, dt):
        rates = [(a+b)*.5*math.pi/180 for a,b in zip(self.previous, gyro)]
        speed = math.sqrt(sum(v*v for v in rates))
        if speed:
            angle = speed*dt/2
            dq = (math.cos(angle), *(v*math.sin(angle)/speed for v in rates))
            q = _mul(self.q, dq)
            norm = math.sqrt(sum(v*v for v in q))
            self.q = tuple(v/norm for v in q)

    def _angles(self):
        # Rotate the mounting-frame forward vector, then project onto left/up.
        yaw, pitch = self.yaw_axis, self.pitch_axis
        forward = (yaw[1]*pitch[2]-yaw[2]*pitch[1], yaw[2]*pitch[0]-yaw[0]*pitch[2], yaw[0]*pitch[1]-yaw[1]*pitch[0])
        vector = _mul(_mul(self.q, (0., *forward)), (self.q[0], *(-v for v in self.q[1:])))[1:]
        dot = lambda a,b: sum(x*y for x,y in zip(a,b))
        return math.degrees(math.atan2(-dot(vector, pitch), dot(vector, forward))), math.degrees(math.asin(max(-1., min(1., dot(vector, yaw)))))

    def update(self, gyro, now=None):
        now = self.clock.perf_counter() if now is None else float(now)
        if gyro is None:
            return None
        raw = tuple(map(float, gyro))
        if len(raw) != 3 or not all(map(math.isfinite, (*raw, now))):
            raise ValueError('non-finite gyro sample')
        if self.bias_state == 'CALIBRATING':
            self._bias_samples.append(raw)
            if now-self._bias_started >= getattr(self.cfg, 'IMU_BIAS_CALIBRATE_S', 1.):
                samples = self._bias_samples
                mean = tuple(sum(v[i] for v in samples)/len(samples) for i in range(3))
                std = tuple(math.sqrt(sum((v[i]-mean[i])**2 for v in samples)/len(samples)) for i in range(3))
                good = len(samples) >= 10 and max(std) <= getattr(self.cfg, 'IMU_BIAS_MAX_STD_DPS', .25) and max(map(abs, mean)) <= getattr(self.cfg, 'IMU_BIAS_MAX_DPS', 2.)
                self.bias_state = 'READY' if good else 'REJECTED'
                if good:
                    self.bias = mean
                self.timestamp = None
        gyro = tuple(a-b for a,b in zip(raw, self.bias))
        dt = None if self.timestamp is None else now-self.timestamp
        if dt is None or not 0 < dt <= getattr(self.cfg, 'IMU_MAX_GAP_S', .2):
            self.q = (1., 0., 0., 0.)
            self.history.clear()
            self.reset_reference()
            self.epoch += 1
        else:
            self._advance(gyro, dt)
        self.previous, self.timestamp = gyro, now
        yaw, pitch = self._angles()
        sample = dict(yaw=yaw, pitch=pitch, captured_at=now, epoch=self.epoch, bias_state=self.bias_state)
        self.history.append(sample)
        return dict(sample)

    def sample_at(self, timestamp):
        # Never use a future sample to explain an already captured camera frame.
        for sample in reversed(self.history):
            if sample['captured_at'] <= timestamp:
                if timestamp-sample['captured_at'] <= getattr(self.cfg, 'IMU_MAX_AGE_S', .2):
                    return dict(sample)
                break
        return None

    def lock_reference(self, now, yaw_offset_deg=0., pitch_offset_deg=0., gimbal_angles=None):
        sample = self.sample_at(now)
        if sample is None or self.bias_state == 'CALIBRATING':
            return False
        if getattr(self.cfg, 'IMU_MOUNT', 'gimbal') == 'base':
            if gimbal_angles is None or not all(map(math.isfinite, gimbal_angles)):
                return False
            yaw_offset_deg += gimbal_angles[0]
            pitch_offset_deg += gimbal_angles[1]
        self.reference = (sample['yaw']+yaw_offset_deg, sample['pitch']+pitch_offset_deg, self.epoch)
        return True

    def correction(self, now, gimbal_angles=None):
        sample = self.sample_at(now)
        if sample is None or self.reference is None or self.reference[2] != self.epoch:
            return None
        yaw, pitch = sample['yaw'], sample['pitch']
        if getattr(self.cfg, 'IMU_MOUNT', 'gimbal') == 'base':
            if gimbal_angles is None:
                return None
            yaw += gimbal_angles[0]
            pitch += gimbal_angles[1]
        return ((self.reference[0]-yaw+180)%360-180, self.reference[1]-pitch)


class HeadingFeedforward:
    def __init__(self, cfg, clock=None, device=None, device_factory=None):
        self.cfg, self.clock = cfg, clock if clock is not None else time
        self.heading = GyroHeading(cfg, clock)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._bias_request = threading.Event()
        self._latest = None
        self.error = None
        self.device = device
        self.device_factory = device_factory or open_imu
        self.thread = threading.Thread(target=self._run, name='imu-heading', daemon=True)
        self.thread.start()

    def _run(self):
        try:
            if self.device is None:
                self.device = self.device_factory()
            while not self._stop.is_set():
                raw = read_gyro(self.device)  # Native I/O never holds the shared state lock.
                now = self.clock.perf_counter()
                with self._lock:
                    if self._bias_request.is_set():
                        self._bias_request.clear()
                        self.heading.start_bias_calibration(now)
                    if raw is not None:
                        self._latest = self.heading.update(raw, now)
                self._stop.wait(min(.02, 1/max(1, getattr(self.cfg, 'IMU_SAMPLE_HZ', 208))))
        except Exception as exc:
            self.error = str(exc)
        finally:
            # Only the owner thread may destroy a device with a native read in flight.
            if self.device is not None:
                close = getattr(self.device, 'close', None)
                if close:
                    close()

    def sample_at(self, now):
        if not self._lock.acquire(False):
            return None
        try:
            return self.heading.sample_at(now)
        finally:
            self._lock.release()

    def snapshot(self, now=None):
        return self.sample_at(self.clock.perf_counter() if now is None else now)

    def lock_reference(self, now, yaw_offset_deg=0., pitch_offset_deg=0., gimbal_angles=None):
        if not self._lock.acquire(False):
            return False
        try:
            return self.heading.lock_reference(now, yaw_offset_deg, pitch_offset_deg, gimbal_angles)
        finally:
            self._lock.release()

    def correction(self, now, gimbal_angles=None):
        if not self._lock.acquire(False):
            return None
        try:
            return self.heading.correction(now, gimbal_angles)
        finally:
            self._lock.release()

    def reset_reference(self):
        with self._lock:
            self.heading.reset_reference()

    def start_bias_calibration(self, now=None):
        self._bias_request.set()

    def close(self):
        self._stop.set()
        self.thread.join(.10)
        if self.thread.is_alive():
            raise RuntimeError('IMU reader still running')
