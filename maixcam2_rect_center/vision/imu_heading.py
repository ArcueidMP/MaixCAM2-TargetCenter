"""Relative inertial heading for the B track, with no UART ownership.

MaixCAM2's onboard IMU is LSM6DSOWTR, exposed through ``maix.ext_dev.imu``.
The adapter below follows Sipeed's documented DUAL/raw API; gyro units are deg/s.
Sources checked at MaixPy a36c0f0 / MaixCDK 30f4b8b:
https://github.com/sipeed/MaixPy/blob/a36c0f0a85ba8b1a4f7eee7ab74af13afcd89bb0/examples/ext_dev/sensors/imu/imu_read_data.py
https://github.com/sipeed/MaixCDK/blob/30f4b8b7e3f66ded9cfa64fb081b46e30bc27248/components/ext_devs/ext_dev/src/lsm6dsowtr/maix_lsm6dsowtr.cpp

Gyros provide short-term *relative* orientation, not an absolute yaw reference.
The initial target lock is the reference; bias, mounting axes and motor degrees
must be calibrated separately. No magnetic heading or drift-free claim is made.
Camera mounting measures body + actual gimbal motion, so subtracting commanded
gimbal motion again would be incorrect. Base mounting instead needs a measured
gimbal position. Degrees returned here are never sent directly to a motor.
"""
import math
import threading
import time
from collections import deque


def _finite_vector(value):
    values = tuple(float(x) for x in value)
    if len(values) != 3 or not all(math.isfinite(x) for x in values):
        raise ValueError("expected three finite numbers")
    return values


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return (w * v - x * i - y * j - z * k,
            w * i + x * v + y * k - z * j,
            w * j - x * k + y * v + z * i,
            w * k + x * j - y * i + z * v)


def _advance(q, rates, dt):
    """Integrate body-frame angular velocity using a normalized quaternion."""
    omega = tuple(math.radians(x) for x in rates)
    speed = math.sqrt(_dot(omega, omega))
    if speed < 1e-12 or dt <= 0:
        return q
    half = speed * dt * 0.5
    scale = math.sin(half) / speed
    result = _multiply(q, (math.cos(half), *(x * scale for x in omega)))
    length = math.sqrt(_dot(result, result))
    return tuple(x / length for x in result)


def _angles(q):
    """Forward/left/up frame: yaw CCW left and pitch up are positive."""
    w, x, y, z = q
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    pitch = -math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
    return yaw, pitch


def _wrapped_delta(value):
    return (value + 180.0) % 360.0 - 180.0


def _offset_quaternion(yaw, pitch):
    yaw, pitch = math.radians(yaw) / 2, -math.radians(pitch) / 2
    return _multiply((math.cos(yaw), 0.0, 0.0, math.sin(yaw)),
                     (math.cos(pitch), 0.0, math.sin(pitch), 0.0))


class MaixIMUSensor:
    """Official MaixPy adapter. Construct/read only in the sensor worker.

    LSM6DSOWTR returns seven zeros when data is not ready; those arrays must
    never replace a fresh moving sample or be integrated as a measured stop.
    The generic header's accel-unit description differs from this board's
    implementation, so this module deliberately does not use accel units.
    """

    def __init__(self):
        from maix.ext_dev import imu
        self.device = imu.IMU("default", mode=imu.Mode.DUAL,
                              acc_scale=imu.AccScale.ACC_SCALE_2G,
                              acc_odr=imu.AccOdr.ACC_ODR_208,
                              gyro_scale=imu.GyroScale.GYRO_SCALE_500DPS,
                              gyro_odr=imu.GyroOdr.GYRO_ODR_208, block=False)
        self.bias_dps = None
        if self.device.calib_gyro_exists():
            bias = self.device.load_calib_gyro()
            self.bias_dps = _finite_vector((bias.x, bias.y, bias.z))

    def read(self):
        raw = self.device.read()
        if not raw:
            return None
        if len(raw) != 7:
            raise ValueError("Maix IMU DUAL read() must return seven values")
        if all(float(x) == 0.0 for x in raw):
            return None
        return _finite_vector(raw[3:6])


class GyroHeading:
    """Pure deterministic estimator. Call only from one thread or under a lock.

    ``update`` takes raw gyro xyz in deg/s and a monotonic sample timestamp.
    Configured unit vectors express physical yaw-left/pitch-up in sensor axes.
    They must be perpendicular and verified on the mounted hardware; defaults
    are mathematical examples, not a claimed MaixCAM2 board mounting.
    Initialize in the calibrated level/home pose. Gyros alone cannot identify
    gravity or infer an unknown initial tilt. Quaternion integration accounts
    for subsequent camera tilt and roll without treating sensor z as world z.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.enabled = bool(getattr(cfg, "ENABLE_IMU_FEEDFORWARD", False))
        self.mount = str(getattr(cfg, "IMU_SENSOR_MOUNT", "camera")).lower()
        if self.mount not in ("camera", "base"):
            raise ValueError("IMU_SENSOR_MOUNT must be camera or base")
        self.yaw_axis = _finite_vector(getattr(cfg, "IMU_YAW_AXIS", (0, 0, 1)))
        up_axis = _finite_vector(getattr(cfg, "IMU_PITCH_AXIS", (0, -1, 0)))
        if (abs(_dot(self.yaw_axis, self.yaw_axis) - 1) > 1e-6 or
                abs(_dot(up_axis, up_axis) - 1) > 1e-6 or
                abs(_dot(self.yaw_axis, up_axis)) > 1e-6):
            raise ValueError("IMU yaw/pitch axes must be orthogonal unit vectors")
        self.down_axis = tuple(-x for x in up_axis)
        self.roll_axis = _cross(self.down_axis, self.yaw_axis)
        self.axes_calibrated = bool(getattr(cfg, "IMU_AXES_CALIBRATED", False))
        self.max_age = max(0.001, float(getattr(cfg, "IMU_MAX_AGE_S", 0.10)))
        self.max_gap = max(self.max_age, float(getattr(cfg, "IMU_MAX_GAP_S", 0.20)))
        self.max_predict = max(0.0, float(getattr(cfg, "IMU_MAX_PREDICT_S", 0.05)))
        self.max_rate = max(1.0, float(getattr(cfg, "IMU_MAX_RATE_DPS", 450.0)))
        self.max_pitch = max(1.0, min(89.0, float(getattr(cfg, "IMU_MAX_PITCH_DEG", 80.0))))
        self.reference_max_age = max(0.001, float(getattr(cfg, "IMU_REFERENCE_MAX_AGE_S", 0.03)))
        self.history_s = max(self.reference_max_age, min(2.0, float(getattr(cfg, "IMU_HISTORY_S", 1.0))))
        history_size = max(2, min(512, int(getattr(cfg, "IMU_HISTORY_MAX_SAMPLES", 256))))
        self.history = deque(maxlen=history_size)
        self.bias = None
        configured_bias = getattr(cfg, "IMU_BIAS_DPS", None)
        if configured_bias is not None:
            self.bias = _finite_vector(configured_bias)
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.last_time = None
        self.rates = (0.0, 0.0, 0.0)
        self.samples = 0
        self.error = None
        self.reference = None
        self.reference_time = None
        self.reference_gimbal = None
        self.yaw_delta = 0.0
        self.pitch_delta = 0.0
        self._last_wrapped_yaw = 0.0
        self._bias_window = None
        self.calibration_state = "IDLE"

    def reset_reference(self):
        self.reference = None
        self.reference_time = None
        self.reference_gimbal = None
        self.yaw_delta = self.pitch_delta = self._last_wrapped_yaw = 0.0

    def set_bias(self, bias_dps):
        self.bias = _finite_vector(bias_dps)
        self.last_time = None
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.history.clear()
        self.reset_reference()
        self.error = None

    def start_bias_calibration(self, now):
        """Explicit stationary calibration; never infer stillness during a turn.

        Hardware must be held still by the caller. Dispersion and a small rate
        threshold reject obvious movement, but cannot detect all slow rotation.
        The returned bias can be saved through application configuration.
        """
        now = float(now)
        if not math.isfinite(now):
            raise ValueError("calibration requires a finite monotonic time")
        self.bias = None
        self.last_time = None
        self.error = None
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.history.clear()
        self.reset_reference()
        self._bias_window = dict(start=now, count=0, sums=[0.0] * 3,
                                 squares=[0.0] * 3)
        self.calibration_state = "COLLECTING"

    def _calibrate(self, raw, now):
        window = self._bias_window
        window["count"] += 1
        for index, value in enumerate(raw):
            window["sums"][index] += value
            window["squares"][index] += value * value
        duration = float(getattr(self.cfg, "IMU_BIAS_CALIBRATE_S", 1.0))
        if now - window["start"] < max(0.1, duration):
            return
        count = window["count"]
        mean = tuple(value / count for value in window["sums"])
        deviation = max(math.sqrt(max(0.0, window["squares"][i] / count - mean[i] ** 2))
                        for i in range(3))
        max_std = float(getattr(self.cfg, "IMU_BIAS_MAX_STD_DPS", 0.25))
        max_bias = float(getattr(self.cfg, "IMU_BIAS_MAX_DPS", 2.0))
        self._bias_window = None
        if count < 10 or deviation > max_std or max(abs(x) for x in mean) > max_bias:
            self.calibration_state = "REJECTED"
            self.error = "bias_calibration_requires_stationary_sensor"
            return
        self.set_bias(mean)
        self.calibration_state = "READY"

    def update(self, gyro_dps, now):
        now = float(now)
        try:
            raw = _finite_vector(gyro_dps)
            if not math.isfinite(now):
                raise ValueError("nonfinite_timestamp")
            if max(abs(x) for x in raw) > self.max_rate:
                raise ValueError("gyro_rate_out_of_range")
        except (ValueError, TypeError, OverflowError) as exc:
            self.fail(str(exc))
            return self.snapshot(now)
        if self._bias_window is not None:
            self._calibrate(raw, now)
            return self.snapshot(now)
        if self.bias is None:
            self.error = None
            return self.snapshot(now)
        corrected = tuple(x - b for x, b in zip(raw, self.bias))
        rates = (_dot(corrected, self.roll_axis), _dot(corrected, self.down_axis),
                 _dot(corrected, self.yaw_axis))
        if self.last_time is not None:
            dt = now - self.last_time
            if dt <= 0 or dt > self.max_gap:
                # Motion during a sensor outage is unknown. Discard the reference
                # instead of inventing rotation across the missing interval.
                self.q = (1.0, 0.0, 0.0, 0.0)
                self.reset_reference()
                self.history.clear()
                self.error = "sample_gap" if dt > 0 else "nonmonotonic_sample"
                self.last_time, self.rates = now, rates
                return self.snapshot(now)
            average = tuple(0.5 * (a + b) for a, b in zip(self.rates, rates))
            self.q = _advance(self.q, average, dt)
        self.last_time, self.rates = now, rates
        self.error = None
        self.samples += 1
        self.history.append((now, self.q, rates))
        while self.history and now - self.history[0][0] > self.history_s:
            self.history.popleft()
        if self.reference is not None:
            current_yaw, current_pitch = _angles(self.q)
            reference_yaw, reference_pitch = _angles(self.reference)
            yaw = _wrapped_delta(current_yaw - reference_yaw)
            self.yaw_delta += _wrapped_delta(yaw - self._last_wrapped_yaw)
            self._last_wrapped_yaw = yaw
            self.pitch_delta = current_pitch - reference_pitch
        return self.snapshot(now)

    def fail(self, error):
        self.error = str(error)
        self.last_time = None
        if self._bias_window is not None:
            self._bias_window = None
            self.calibration_state = "REJECTED"
        self.reset_reference()
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.history.clear()

    def snapshot(self, now):
        now = float(now)
        age = None if self.last_time is None else now - self.last_time
        fresh = age is not None and 0 <= age <= self.max_age
        current_yaw, current_pitch = _angles(self.q)
        if not self.enabled:
            state = "DISABLED"
        elif self.error:
            state = "ERROR"
        elif self._bias_window is not None:
            state = "CALIBRATING"
        elif not self.axes_calibrated:
            state = "AXES_UNCALIBRATED"
        elif self.bias is None:
            state = "BIAS_UNCALIBRATED"
        elif not fresh:
            state = "STALE"
        elif abs(current_pitch) > self.max_pitch:
            state = "ATTITUDE_OUT_OF_RANGE"
        else:
            state = "LOCKED" if self.reference is not None else "READY"
        # Sensor/body rates are not world yaw/pitch rates after a camera tilt.
        # A short quaternion derivative gives the common-frame Euler rates.
        next_yaw, next_pitch = _angles(_advance(self.q, self.rates, 0.0001))
        yaw_rate = _wrapped_delta(next_yaw - current_yaw) / 0.0001
        pitch_rate = (next_pitch - current_pitch) / 0.0001
        return dict(valid=state in ("READY", "LOCKED"), state=state,
                    mount=self.mount, age_s=age, sampled_at=self.last_time,
                    samples=self.samples, bias_dps=self.bias, error=self.error,
                    reference_locked=self.reference is not None,
                    reference_at=self.reference_time, yaw_deg=self.yaw_delta,
                    pitch_deg=self.pitch_delta, rate_yaw_dps=yaw_rate,
                    rate_pitch_dps=pitch_rate, calibration=self.calibration_state)

    def sample_at(self, captured_at):
        """Find a close sample at/before capture; never use a future sample.

        This is a bounded history, not an IMU IO operation. A short interpolation
        with the previous sample's angular rate synchronizes the returned pose.
        ``valid`` only describes capture-time alignment; consumers must also
        check the current sensor's freshness before issuing a motor command.
        """
        captured_at = float(captured_at)
        for sampled_at, q, rates in reversed(self.history):
            if sampled_at <= captured_at:
                age = captured_at - sampled_at
                valid = math.isfinite(captured_at) and age <= self.reference_max_age
                return dict(valid=valid, sampled_at=sampled_at, captured_at=captured_at,
                            age_s=age, quaternion=_advance(q, rates, age) if valid else q,
                            rates_dps=rates, state="ALIGNED" if valid else "HISTORY_STALE")
        return dict(valid=False, sampled_at=None, captured_at=captured_at,
                    age_s=None, quaternion=None, rates_dps=None, state="NO_HISTORY")

    def lock_reference(self, now, yaw_offset_deg=0.0, pitch_offset_deg=0.0,
                       gimbal_deg=None):
        """Lock a capture-aligned target direction with physical angle offsets.

        ``now`` is the visual frame's capture time in the same monotonic clock.
        Yaw offsets are positive CCW-left; pitch offsets positive up. Root must
        convert visual pixels using measured signed pixel/degree calibration.
        """
        if self.last_time is None or not self.snapshot(self.last_time)["valid"]:
            return False
        aligned = self.sample_at(now)
        if not aligned["valid"]:
            return False
        try:
            offsets = (float(yaw_offset_deg), float(pitch_offset_deg))
            if not all(math.isfinite(x) for x in offsets) or max(abs(x) for x in offsets) > 90:
                return False
        except (TypeError, ValueError, OverflowError):
            return False
        measured = self._gimbal(gimbal_deg)
        if self.mount == "base" and measured is None:
            return False
        self.reference = _multiply(aligned["quaternion"], _offset_quaternion(*offsets))
        self.reference_time = float(now)
        self.reference_gimbal = measured
        current_yaw, current_pitch = _angles(self.q)
        reference_yaw, reference_pitch = _angles(self.reference)
        yaw = _wrapped_delta(current_yaw - reference_yaw)
        self.yaw_delta = self._last_wrapped_yaw = yaw
        self.pitch_delta = current_pitch - reference_pitch
        return True

    @staticmethod
    def _gimbal(value):
        if value is None:
            return None
        try:
            result = (float(value["yaw"]), float(value["pitch"])) if isinstance(value, dict) else tuple(float(x) for x in value)
            if len(result) != 2 or not all(math.isfinite(x) for x in result):
                return None
            return result
        except (KeyError, ValueError, TypeError, OverflowError):
            return None

    def correction(self, now, gimbal_deg=None, lead_s=0.0):
        """Remaining heading error, positive motor CCW-left / pitch-up.

        Caller should use the absolute error once, or use a sample-time delta
        after a visual result; adding the full error to visual feedback doubles
        the already-observed rotation. Neither motor limits nor pulse conversion
        belong here. ``base`` needs actual measured relative gimbal angles.
        """
        result = self.snapshot(now)
        result.update(yaw_deg=0.0, pitch_deg=0.0)
        if not result["valid"] or self.reference is None:
            result["valid"] = False
            if result["state"] == "READY":
                result["state"] = "NO_REFERENCE"
            return result
        age = max(0.0, float(now) - self.last_time)
        prediction = min(self.max_predict, age + max(0.0, float(lead_s)))
        predicted = _advance(self.q, self.rates, prediction)
        current_yaw, _ = _angles(self.q)
        predicted_yaw, predicted_pitch = _angles(predicted)
        yaw = self.yaw_delta + _wrapped_delta(predicted_yaw - current_yaw)
        pitch = predicted_pitch - _angles(self.reference)[1]
        # Camera sensor already observes the completed portion of any motor
        # movement; never subtract that motion a second time.
        if self.mount == "base":
            measured = self._gimbal(gimbal_deg)
            if measured is None:
                result.update(valid=False, state="BASE_REQUIRES_GIMBAL_POSITION")
                return result
            yaw += measured[0] - self.reference_gimbal[0]
            pitch += measured[1] - self.reference_gimbal[1]
        result.update(yaw_deg=-yaw, pitch_deg=-pitch, prediction_s=prediction)
        return result


class HeadingFeedforward:
    """One daemon sensor worker, one latest state, nonblocking controller reads.

    The worker integrates each IMU sample even when vision drops frames. It does
    no UART operations. No sensor IO occurs while holding the state mutex, so a
    slow or stuck sensor cannot hold the controller's lock. Native driver's GIL
    behavior/IO timeout still requires board verification. Shutdown has a bounded
    join and never destroys an in-flight native device from another thread.

    Inject a sensor whose ``read()`` returns gyro xyz deg/s or None. Tests may set
    ``threaded=False`` and use ``ingest`` with a deterministic monotonic clock.
    """

    def __init__(self, cfg, sensor=None, clock=None, threaded=True):
        self.cfg = cfg
        self._clock = clock if clock is not None else time
        self._sensor = sensor
        self._estimator = GyroHeading(cfg)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._closed = False
        self._startup_calibration = bool(getattr(cfg, "IMU_CALIBRATE_ON_START", False))
        # Device construction may be slow. Reserve the stationary calibration
        # phase before starting the worker, so controllers cannot move motors
        # during initialization and then contaminate a later bias collection.
        self._startup_calibration_pending = (self._estimator.enabled and
                                             self._startup_calibration and
                                             self._estimator.bias is None and threaded)
        if (self._estimator.enabled and self._startup_calibration and
                self._estimator.bias is None and not threaded):
            # Manual ingestion has no device-constructor phase to resolve.
            self._estimator.start_bias_calibration(self._now())
        if self._estimator.enabled and threaded:
            self._thread = threading.Thread(target=self._run, name="imu-heading", daemon=True)
            self._thread.start()

    def _now(self):
        return float(self._clock.perf_counter())

    @property
    def worker_alive(self):
        """阻塞硬件读未退出时，应用不得另开同一传感器实例。"""
        return self._thread is not None and self._thread.is_alive()

    def _run(self):
        try:
            sensor = self._sensor if self._sensor is not None else MaixIMUSensor()
            self._sensor = sensor
            with self._lock:
                if self._estimator.bias is None:
                    supplied_bias = getattr(sensor, "bias_dps", None)
                    if supplied_bias is not None:
                        self._estimator.set_bias(supplied_bias)
                if self._estimator.bias is None and self._startup_calibration:
                    self._estimator.start_bias_calibration(self._now())
                self._startup_calibration_pending = False
        except Exception as exc:
            with self._lock:
                self._estimator.fail("imu_unavailable: %s" % exc)
                self._startup_calibration_pending = False
            return
        hz = max(1.0, min(400.0, float(getattr(self.cfg, "IMU_POLL_HZ", 200.0))))
        while not self._stop.is_set():
            started = self._now()
            try:
                gyro = sensor.read()               # IO intentionally outside lock
                received = self._now()
                if gyro is not None:
                    with self._lock:
                        max_read = max(0.001, float(getattr(self.cfg, "IMU_READ_MAX_S", self._estimator.max_age)))
                        if received - started > max_read or received < started:
                            self._estimator.fail("imu_read_timeout")
                        elif not self._stop.is_set():
                            self._estimator.update(gyro, received)
            except Exception as exc:
                with self._lock:
                    self._estimator.fail("imu_read_failed: %s" % exc)
            # One bounded poll per iteration, no queue or catch-up bursts.
            self._stop.wait(max(0.001, 1.0 / hz - (self._now() - started)))

    def ingest(self, gyro_dps, now=None):
        """Deterministic injected samples; incompatible with an active worker."""
        if self._thread is not None:
            raise RuntimeError("ingest requires threaded=False")
        with self._lock:
            return self._estimator.update(gyro_dps, self._now() if now is None else now)

    def sample(self, now=None):
        with self._lock:
            result = self._estimator.snapshot(self._now() if now is None else now)
            if self._startup_calibration_pending:
                result.update(valid=False, state="CALIBRATING", calibration="PENDING_SENSOR")
            if self._closed:
                result.update(valid=False, state="CLOSED")
            return result

    def sample_at(self, captured_at):
        with self._lock:
            result = self._estimator.sample_at(captured_at)
            if self._closed:
                result.update(valid=False, state="CLOSED")
            return result

    def lock_reference(self, now=None, yaw_offset_deg=0.0, pitch_offset_deg=0.0,
                       gimbal_deg=None):
        with self._lock:
            if self._closed or not self._estimator.snapshot(self._now())["valid"]:
                return False
            return self._estimator.lock_reference(
                self._now() if now is None else now, yaw_offset_deg, pitch_offset_deg, gimbal_deg)

    def reset_reference(self):
        with self._lock:
            self._estimator.reset_reference()

    def start_bias_calibration(self, now=None):
        with self._lock:
            self._estimator.start_bias_calibration(self._now() if now is None else now)
            self._startup_calibration_pending = False

    def correction(self, now=None, gimbal_deg=None, lead_s=0.0):
        with self._lock:
            result = self._estimator.correction(self._now() if now is None else now,
                                                gimbal_deg, lead_s)
            if self._startup_calibration_pending:
                result.update(valid=False, state="CALIBRATING", calibration="PENDING_SENSOR",
                              yaw_deg=0.0, pitch_deg=0.0)
            if self._closed:
                result.update(valid=False, state="CLOSED", yaw_deg=0.0, pitch_deg=0.0)
            return result

    def close(self):
        self._closed = True
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.10)
