"""Reusable application API: touch during camera gaps and safe resource ownership."""
import contextlib
import io
import sys
import threading
import types
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import Config
from vision import app_runtime
from vision.app_runtime import TrackingApplication
from vision.imu_heading import HeadingFeedforward


class Clock:
    def __init__(self):
        self.current = 0.0

    def perf_counter(self):
        self.current += .01
        return self.current


class Controller:
    def __init__(self, cfg, link, aim, calibrator, uart, clock=None):
        self.request_test = threading.Event()
        self.request_search = threading.Event()
        self.received = []
        self.processed = self.idles = 0
        self.closed = False
        self.clock = clock

    def _events(self):
        for name, event in (("selftest", self.request_test),
                            ("search", self.request_search)):
            if event.is_set():
                self.received.append(name)
                event.clear()

    def process(self, gray, captured):
        self.processed += 1
        self._events()
        return dict(result={"valid": False, "stage": "full_search"}, laser=None,
                    captured_at=captured, completed_at=self.clock.perf_counter(),
                    status="vision", detect_ms=0.0)

    def idle(self):
        self.idles += 1
        self._events()

    def close(self):
        self.closed = True


def fake_sdk(frames, touch_events=(), width=640, height=480, fail_skip=False):
    state = types.SimpleNamespace(reads=0, shown=[], closed=[], touch_reads=0)

    class Camera:
        def __init__(self, *args, **kwargs):
            pass

        def skip_frames(self, n):
            if fail_skip:
                raise RuntimeError("skip failed")

        def read(self):
            frame = frames[state.reads]
            state.reads += 1
            return frame

        def close(self):
            state.closed.append("camera")

    class Display:
        def width(self):
            return width

        def height(self):
            return height

        def show(self, img):
            state.shown.append(np.array(img, copy=True))

        def close(self):
            state.closed.append("display")

    class Touch:
        def read(self):
            index = state.touch_reads
            state.touch_reads += 1
            return touch_events[index] if index < len(touch_events) else (0, 0, False)

        def close(self):
            state.closed.append("touch")

    maix = types.ModuleType("maix")
    maix.app = types.SimpleNamespace(need_exit=lambda: state.reads >= len(frames))
    maix.camera = types.SimpleNamespace(Camera=Camera)
    maix.display = types.SimpleNamespace(Display=Display)
    maix.touchscreen = types.SimpleNamespace(TouchScreen=Touch)
    maix.image = types.SimpleNamespace(
        Format=types.SimpleNamespace(FMT_GRAYSCALE=0),
        image2cv=lambda img, **kwargs: img, cv2image=lambda img, **kwargs: img)
    return maix, state


def config(**changes):
    values = dict(SHOW=True, SHOW_ASYNC=False, VISION_ASYNC=False,
                  ENABLE_GIMBAL=False, ENABLE_UART=False, LOG_INTERVAL=0.0,
                  ENABLE_TOUCH_BUTTON=True, ENABLE_SEARCH_BUTTON=True,
                  DISABLE_MAIXVISION_STREAM=False)
    values.update(changes)
    return type("RuntimeConfig", (Config,), values)


def test_camera_gaps_poll_search_and_selftest_once():
    gray = np.full((480, 640), 200, np.uint8)
    events = [(500, 50, True), (0, 0, False), (500, 150, True),
              (500, 150, True), (0, 0, False), (500, 150, True)]
    maix, state = fake_sdk([gray, None, None, None, None, None], events)
    application = TrackingApplication(config(), "runtime-test", Clock(), Controller)
    with patch.dict(sys.modules, maix=maix), contextlib.redirect_stdout(io.StringIO()) as log:
        application.run()
    assert application.controller.received == ["selftest", "search", "search"]
    assert application.controller.idles == 5, "one idle per missing frame in sync mode"
    assert application.controller.processed == 1
    assert state.touch_reads == state.reads == 6
    assert application.controller.closed and set(state.closed) == {"camera", "display", "touch"}
    assert "[perf] runtime-test capture=" in log.getvalue()
    for field in ("disp=", "detect_fps=", "read=", "draw=", "loop=",
                  "show avg=", "drop(show)=", "drop(vision)="):
        assert field in log.getvalue(), field
    assert not application.request_search(), "closed application must reject requests"


def test_buttons_fit_small_screens_without_overlap():
    for width, height in ((640, 480), (320, 240), (100, 60)):
        maix, _ = fake_sdk([], width=width, height=height)
        application = TrackingApplication(config(), "layout", Clock(), Controller)
        with patch.dict(sys.modules, maix=maix), contextlib.redirect_stdout(io.StringIO()):
            application.run()
        first = application.preview.buttons["selftest"]
        second = application.preview.buttons["search"]
        for x0, y0, x1, y1 in (first, second):
            assert 0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height
        assert first[3] < second[1], (first, second)
        assert application.preview.hit((second[0] + second[2]) // 2,
                                       (second[1] + second[3]) // 2) == "search"


def test_no_display_still_accepts_application_requests():
    maix, state = fake_sdk([None, None])
    application = TrackingApplication(config(SHOW=False), "headless", Clock(), Controller)
    camera_ctor = maix.camera.Camera

    class RequestCamera(camera_ctor):
        def read(self):
            assert application.request_search()
            if state.reads == 0:
                assert application.request_self_test()
            return super().read()

    maix.camera.Camera = RequestCamera
    with patch.dict(sys.modules, maix=maix), contextlib.redirect_stdout(io.StringIO()):
        application.run()
    assert not application.preview.enabled and application.touch is None
    assert application.controller.received == ["selftest", "search", "search"]


def test_startup_failure_closes_camera():
    maix, state = fake_sdk([], fail_skip=True)
    application = TrackingApplication(config(), "startup", Clock(), Controller)
    with patch.dict(sys.modules, maix=maix):
        try:
            application.run()
            raise AssertionError("startup error was swallowed")
        except RuntimeError as exc:
            assert str(exc) == "skip failed"
    assert state.closed == ["camera"] and not application._running


def test_controller_setup_failure_closes_preview_and_camera():
    maix, state = fake_sdk([])

    def broken_controller(*args, **kwargs):
        raise RuntimeError("controller setup failed")

    application = TrackingApplication(config(), "setup", Clock(), broken_controller)
    with patch.dict(sys.modules, maix=maix), contextlib.redirect_stdout(io.StringIO()):
        try:
            application.run()
            raise AssertionError("controller setup error was swallowed")
        except RuntimeError as exc:
            assert str(exc) == "controller setup failed"
    assert set(state.closed) == {"camera", "display", "touch"}


def test_repeated_run_does_not_reclose_resources_after_early_failure():
    application = TrackingApplication(config(), "repeat", Clock(), Controller)
    first_maix, first = fake_sdk([])
    with patch.dict(sys.modules, maix=first_maix), contextlib.redirect_stdout(io.StringIO()):
        application.run()
    previous_controller = application.controller
    second_maix, second = fake_sdk([], fail_skip=True)
    with patch.dict(sys.modules, maix=second_maix):
        try:
            application.run()
            raise AssertionError("second startup error was swallowed")
        except RuntimeError as exc:
            assert str(exc) == "skip failed"
    assert first.closed == ["display", "touch", "camera"]
    assert second.closed == ["camera"]
    assert previous_controller.closed and application.controller is None


def test_repeated_run_refuses_previous_display_owner():
    application = TrackingApplication(config(), "ownership", Clock(), Controller)
    previous_preview = types.SimpleNamespace(
        thread=types.SimpleNamespace(is_alive=lambda: True))
    application.preview = previous_preview
    # Vision already exited; a failed display join still prevents new hardware.
    application.worker = types.SimpleNamespace(
        thread=types.SimpleNamespace(is_alive=lambda: False))
    maix, state = fake_sdk([])
    with patch.dict(sys.modules, maix=maix):
        try:
            application.run()
            raise AssertionError("second run opened hardware beside old display worker")
        except RuntimeError as exc:
            assert str(exc) == "previous display worker still owns hardware"
    assert application.preview is previous_preview
    assert state.closed == [] and not application._running


def test_repeated_run_refuses_blocked_previous_imu_owner():
    entered, release = threading.Event(), threading.Event()

    class Sensor:
        def read(self):
            entered.set()
            assert release.wait(2)
            return (0.0, 0.0, 0.0)

    cfg = config(ENABLE_IMU_FEEDFORWARD=True, IMU_BIAS_DPS=(0.0, 0.0, 0.0),
                 IMU_CALIBRATE_ON_START=False)
    heading = HeadingFeedforward(cfg, sensor=Sensor())
    application = TrackingApplication(cfg, "imu-ownership", Clock(), Controller)
    previous_controller = types.SimpleNamespace(heading=heading)
    application.controller = previous_controller
    application.worker = types.SimpleNamespace(thread=None)
    maix, state = fake_sdk([])
    try:
        assert entered.wait(1)
        heading.close()  # Bounded join leaves the in-flight native owner alive.
        assert heading.worker_alive
        with patch.dict(sys.modules, maix=maix):
            try:
                application.run()
                raise AssertionError("second run opened hardware beside old IMU owner")
            except RuntimeError as exc:
                assert str(exc) == "previous IMU worker still owns hardware"
        assert application.controller is previous_controller
        assert state.closed == [] and not application._running
    finally:
        release.set()
        heading.close()
    assert not heading.worker_alive


def test_async_gaps_do_not_call_controller_from_capture_thread():
    maix, state = fake_sdk([None] * 3)

    class Worker:
        def __init__(self, callback, idle, asynchronous, clock):
            assert asynchronous
            self.alive = True
            self.thread = types.SimpleNamespace(is_alive=lambda: self.alive)
            self.error = None

        def close(self):
            self.alive = False

    class OwnedController(Controller):
        def idle(self):
            raise AssertionError("main thread entered async controller/UART")

        def close(self):
            assert not application.worker.thread.is_alive()
            super().close()

    application = TrackingApplication(config(VISION_ASYNC=True), "async", Clock(), OwnedController)
    with patch.dict(sys.modules, maix=maix), \
            patch.object(app_runtime, "LatestFrameWorker", Worker), \
            contextlib.redirect_stdout(io.StringIO()):
        application.run()
    assert state.reads == 3 and application.controller.closed


def test_worker_shutdown_failure_preserves_uart_ownership_and_camera_cleanup():
    maix, state = fake_sdk([])

    class Worker:
        def __init__(self, callback, idle, asynchronous, clock):
            self.thread = types.SimpleNamespace(is_alive=lambda: True)
            self.error = None

        def close(self):
            raise RuntimeError("worker owns UART")

    application = TrackingApplication(config(), "shutdown", Clock(), Controller)
    with patch.dict(sys.modules, maix=maix), \
            patch.object(app_runtime, "LatestFrameWorker", Worker), \
            contextlib.redirect_stdout(io.StringIO()):
        try:
            application.run()
            raise AssertionError("worker shutdown error was swallowed")
        except RuntimeError as exc:
            assert str(exc) == "worker owns UART"
    assert not application.controller.closed, "live worker still owns controller/IMU"
    assert set(state.closed) == {"camera", "display", "touch"}


if __name__ == "__main__":
    tests = [value for key, value in sorted(globals().items()) if key.startswith("test_")]
    for test in tests:
        test()
        print("PASS:", test.__name__)
    print("app runtime: PASS (%d)" % len(tests))
