"""Reusable application assembly; camera, vision and display keep separate clocks.

MaixPy imports stay in ``run()`` so importing the application API on a PC does
not open hardware. The vision worker exclusively owns controller/UART calls;
touch input only sets request events, including while the camera has no frame.
"""
import time

import cv2
import numpy as np

from .aim_control import VisionControl
from .camera_source import to_gray
from .gimbal_aim import GimbalAim
from .gimbal_calibrate import GimbalCalibrator
from .laser_spot import PANEL_H, draw_laser_overlay
from .preview import (Preview, TouchButtons, apply_maixvision_stream_policy,
                      display_result)
from .rate_meter import FrameRate
from .rectangle_center import draw_result
from .stepper_link import StepperLink
from .telemetry import MAIN_LOOP_SLOW_MS, format_perf_line, format_slow_loop
from .vision_worker import LatestFrameWorker


class TrackingApplication:
    """Run B-track recognition, preview and gimbal control as an application.

    ``cfg`` is a Config-compatible object; ``clock`` supplies ``perf_counter``.
    Hardware is opened by ``run()`` and closed on exit, including startup errors.
    External callers may use ``request_search()`` / ``request_self_test()``
    while running, without touching the UART or needing a display.
    """

    def __init__(self, cfg, build_id, clock=None, controller_factory=None):
        self.cfg, self.build_id = cfg, str(build_id)
        self.clock = clock if clock is not None else time
        self.controller_factory = controller_factory or VisionControl
        self.cam = self.uart = self.link = self.aim = self.calibrator = None
        self.controller = self.worker = self.preview = self.touch = None
        self._running = False

    def request_self_test(self):
        """Queue a self-test request; the controller checks fault/feature guards."""
        return self._request("request_test")

    def request_search(self):
        """Queue a search toggle (start/cancel), safe from any calling thread."""
        return self._request("request_search")

    def _request(self, name):
        event = getattr(self.controller, name, None)
        if not self._running or event is None:
            return False
        event.set()
        return True

    def _poll_touch(self):
        if self.touch is None:
            return
        hit = self.touch.poll(self.preview)
        if hit == "selftest":
            self.request_self_test()
        elif hit == "search":
            self.request_search()

    def _configure_touch(self):
        cfg, preview = self.cfg, self.preview
        if preview.enabled and bool(getattr(cfg, "ENABLE_TOUCH_BUTTON", True)):
            # Screen coordinates, clamped to fit both buttons even on small LCDs.
            width, height = int(preview.width), int(preview.height)
            margin = min(8, max(0, min(width, height) // 8))
            search = bool(getattr(cfg, "ENABLE_SEARCH_BUTTON", True))
            gap = max(1, min(8, height // 12))
            if height - 2 * margin < 3:
                search = False
            count = 2 if search else 1
            bw = min(max(70, int(width * .30)), max(1, width - 2 * margin))
            available = max(1, height - 2 * margin - (gap if search else 0))
            bh = min(max(34, int(height * .20)), max(1, available // count))
            x0, x1 = max(0, width - bw - margin), max(1, width - margin)
            preview.add_button("selftest", x0, margin, x1, margin + bh)
            if search:
                y0 = margin + bh + gap
                preview.add_button("search", x0, y0, x1, y0 + bh)
            self.touch = TouchButtons(cfg)
        elif preview.enabled:
            print("[touch] ENABLE_TOUCH_BUTTON=False -> 不画按钮")
        elif bool(getattr(cfg, "ENABLE_TOUCH_BUTTON", True)):
            print("[touch] 无显示 -> 屏幕按钮不可用（可通过应用 API 请求自检/搜索）")

    def _open_gimbal(self):
        cfg = self.cfg
        if not cfg.ENABLE_GIMBAL:
            return
        try:
            self.link = StepperLink(cfg)
            self.link.enable()
            self.aim = GimbalAim(cfg)
            self.calibrator = GimbalCalibrator(cfg, self.link, self.clock.perf_counter())
            if self.calibrator.done:
                self.calibrator = None
            else:
                print("[gimbal] calibration mode axis=%s（本次只标定，不闭环）" %
                      self.calibrator.axis)
            print("[gimbal] enabled yaw=%s(ID%d) pitch=%s(ID%d) acc=%d" %
                  (cfg.YAW_DEV, cfg.ADDR_YAW, cfg.PITCH_DEV, cfg.ADDR_PITCH,
                   cfg.ACC_DEFAULT))
        except Exception as exc:
            # Preserve any partly opened link for exit cleanup; never drive it
            # when setup did not complete.
            self.aim = self.calibrator = None
            print("[gimbal] disabled after init failure: %s" % exc)

    def run(self):
        """Run until MaixPy requests exit, then release owned resources."""
        if self._running:
            raise RuntimeError("tracking application is already running")
        previous_thread = self.worker.thread if self.worker is not None else None
        if previous_thread is not None and previous_thread.is_alive():
            raise RuntimeError("previous vision worker still owns hardware")
        previous_display = self.preview.thread if self.preview is not None else None
        if previous_display is not None and previous_display.is_alive():
            raise RuntimeError("previous display worker still owns hardware")
        previous_heading = getattr(self.controller, "heading", None)
        if bool(getattr(previous_heading, "worker_alive", False)):
            raise RuntimeError("previous IMU worker still owns hardware")
        # A second run opens fresh resources. A failed early startup must never
        # close or stop already released resources left over from a prior run.
        self.cam = self.uart = self.link = self.aim = self.calibrator = None
        self.controller = self.worker = self.preview = self.touch = None
        self._running = True
        cfg, clock = self.cfg, self.clock
        try:
            apply_maixvision_stream_policy(cfg)
            from maix import app, camera, image

            cv2.setNumThreads(cfg.OPENCV_THREADS)
            self.cam = camera.Camera(cfg.CAMERA_W, cfg.CAMERA_H,
                                     image.Format.FMT_GRAYSCALE,
                                     fps=cfg.CAMERA_FPS, buff_num=cfg.CAMERA_BUFFERS)
            self.cam.skip_frames(cfg.SKIP_FRAMES)
            if cfg.ENABLE_UART:
                try:
                    from maix import uart as maix_uart
                    self.uart = maix_uart.UART(cfg.UART_PORT, cfg.UART_BAUD)
                except Exception as exc:
                    print("[uart] open failed: %s" % exc)
            self._open_gimbal()
            self.preview = Preview(cfg, clock=clock)
            self._configure_touch()
            self.controller = self.controller_factory(
                cfg, self.link, self.aim, self.calibrator, self.uart, clock=clock)
            self.worker = LatestFrameWorker(self.controller.process, self.controller.idle,
                                           cfg.VISION_ASYNC, clock=clock)
            self._log_startup()
            self._capture_loop(app, image)
        finally:
            self._running = False
            self._shutdown()

    def _log_startup(self):
        cfg, preview = self.cfg, self.preview
        print("[rect-center] %s search_width=%d display_width=%d display_fps=%d" %
              (self.build_id, cfg.SEARCH_WIDTH, preview.render_width, cfg.SHOW_FPS))
        print("[preview] async=%s 画布=%dx%d（SHOW_WIDTH=%d；0=用屏幕原始宽度）" %
              (getattr(cfg, "SHOW_ASYNC", True), preview.render_width,
               preview.render_height, int(getattr(cfg, "SHOW_WIDTH", 0) or 0)))
        print("[vision] latest-frame worker=%s max_result_age=%.0fms" %
              (cfg.VISION_ASYNC, cfg.RESULT_MAX_AGE_S * 1000))
        print("[ui] 上电自检已改为屏幕按钮：点显示右上角 SELF-TEST 触发（不再上电自动跑）")
        if "search" in preview.buttons:
            print("[ui] SEARCH 按钮：点击启动搜索，再点击取消（不自动启动）")

    def _capture_loop(self, app, image):
        cfg, clock = self.cfg, self.clock
        worker, preview = self.worker, self.preview
        capture_rate = FrameRate()
        last_log = clock.perf_counter()
        read_times, draw_times, loop_times = [], [], []
        while not app.need_exit():
            started = clock.perf_counter()
            img = self.cam.read()
            captured = clock.perf_counter()
            # Touch input remains live during camera gaps. It only sets events;
            # async UART/control work remains exclusively on the vision worker.
            self._poll_touch()
            if img is None:
                if worker.thread is None:
                    self.controller.idle()
                self._check_worker()
                continue
            read_times.append((captured - started) * 1000)
            gray = to_gray(img, image)
            capture_rate.record(captured)
            worker.submit(gray, captured)
            now = clock.perf_counter()
            snapshot, vision_fps, _ = worker.snapshot(now)
            self._check_worker()
            result, laser_state, age = display_result(snapshot, now, cfg.RESULT_MAX_AGE_S)
            if preview.due(now):
                t_draw = clock.perf_counter()
                self._draw_preview(gray, result, laser_state, snapshot, vision_fps, age, now)
                draw_times.append((clock.perf_counter() - t_draw) * 1000)
            loop_times.append((clock.perf_counter() - started) * 1000)
            now = clock.perf_counter()
            if now - last_log >= cfg.LOG_INTERVAL:
                self._log_performance(now, capture_rate, vision_fps, result, age,
                                      read_times, draw_times, loop_times)
                last_log = now
                read_times.clear()
                draw_times.clear()
                loop_times.clear()

    def _check_worker(self):
        if self.worker.error is not None:
            raise RuntimeError("vision worker failed: " + self.worker.error)

    def _draw_preview(self, gray, result, laser_state, snapshot, vision_fps, age, now):
        preview = self.preview
        view = draw_result(gray, result, preview.render_width,
                           top_offset=PANEL_H if laser_state is not None else 0)
        if laser_state is not None:
            draw_laser_overlay(view, laser_state, result,
                               min(1.0, preview.render_width / float(gray.shape[1])),
                               use_raw=self.cfg.AIM_USE_RAW)
        cv2.putText(view, "disp %.1f low1 %.1f det %.1f age %.0fms" %
                    (preview.fps, preview.low1, vision_fps, age * 1000),
                    (8, view.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX,
                    .40, (0, 255, 255), 1)
        preview.render(view, now, status=snapshot["status"] if snapshot else "waiting")

    def _log_performance(self, now, capture_rate, vision_fps, result, age,
                         read_times, draw_times, loop_times):
        preview, worker = self.preview, self.worker
        capture_fps, _ = capture_rate.values(now)
        display_fps, display_low1 = preview.rate.values(now)
        show_avg, show_p95, show_max, show_n = preview.take_show_stats()
        read_avg = float(np.mean(read_times)) if read_times else 0.0
        draw_avg = float(np.mean(draw_times)) if draw_times else 0.0
        loop_avg = float(np.mean(loop_times)) if loop_times else 0.0
        print(format_perf_line(
            self.build_id, capture_fps, display_fps, display_low1, vision_fps,
            result, read_avg, draw_avg, loop_avg, show_avg, show_p95, show_max,
            show_n, age, preview.shown, preview.dropped, worker.dropped))
        if self.aim is not None:
            print(self.aim.status_line())
        if show_p95 > 100:
            preview.warn_slow_once(show_p95)
        if loop_avg > MAIN_LOOP_SLOW_MS:
            print(format_slow_loop(loop_avg))

    def _close_resource(self, resource, label):
        close = getattr(resource, "close", None)
        if close is not None:
            try:
                close()
            except Exception as exc:
                print("[%s] close on exit failed: %s" % (label, exc))

    def _shutdown(self):
        """Stop worker before UART/IMU; display/camera cleanup is independent."""
        worker_error = None
        if self.worker is not None:
            try:
                self.worker.close()
            except Exception as exc:
                worker_error = exc
        thread = self.worker.thread if self.worker is not None else None
        if thread is None or not thread.is_alive():
            if self.link is not None:
                try:
                    self.link.stop()
                except Exception as exc:
                    print("[gimbal] stop on exit failed: %s" % exc)
            self._close_resource(self.controller, "vision")
            self._close_resource(self.link, "gimbal")
            self._close_resource(self.uart, "uart")
        self._close_resource(self.preview, "preview")
        self._close_resource(getattr(self.touch, "ts", None), "touch")
        self._close_resource(self.cam, "camera")
        if worker_error is not None:
            raise worker_error
