"""MaixCAM2 entry point for perspective robust black rectangle center detection."""

import os
import sys
import time

import cv2
import numpy as np


def _root():
    try:
        return os.path.dirname(os.path.abspath(__file__))
    except NameError:
        return os.getcwd()


ROOT = _root()
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from config import Config
from vision.rectangle_center import RectangleCenterDetector, draw_result


def _to_gray(img, image_module):
    a = image_module.image2cv(img, ensure_bgr=False, copy=False)
    if a.ndim == 2:
        return a
    if a.ndim == 3 and a.shape[2] == 1:
        return a[:, :, 0]
    if a.ndim == 3 and a.shape[2] == 3:
        return cv2.cvtColor(a, cv2.COLOR_RGB2GRAY)
    if a.ndim == 3 and a.shape[2] == 4:
        return cv2.cvtColor(a, cv2.COLOR_RGBA2GRAY)
    b = image_module.image2cv(img, ensure_bgr=True, copy=True)
    if b.ndim == 2:
        return b
    return cv2.cvtColor(b, cv2.COLOR_BGR2GRAY)


def _send_uart(uart, result):
    if uart is None:
        return
    if result.get("valid"):
        payload = "@[%d,%d]%%\r\n" % (int(round(result["u"])), int(round(result["v"])))
    else:
        payload = "@[invalid]%%\r\n"
    uart.write(payload.encode("ascii"))


def main():
    # Imports stay inside main so the geometry module remains PC-testable.
    from maix import app, camera, display, image

    cam = camera.Camera(Config.CAMERA_W, Config.CAMERA_H,
                        image.Format.FMT_GRAYSCALE,
                        fps=Config.CAMERA_FPS,
                        buff_num=Config.CAMERA_BUFFERS)
    cam.skip_frames(Config.SKIP_FRAMES)
    disp = display.Display() if Config.SHOW else None
    uart = None
    if Config.ENABLE_UART:
        try:
            from maix import uart as maix_uart
            uart = maix_uart.UART(Config.UART_PORT, Config.UART_BAUD)
        except Exception as exc:
            print("[uart] open failed: %s" % exc)
    detector = RectangleCenterDetector(Config)
    last = time.time()
    last_log = last
    fps = 0.0
    print("[rect-center] Canny + quad pair + 8x8 homography + 72-point inverse circle")
    while not app.need_exit():
        img = cam.read()
        if img is None:
            continue
        if Config.LENS_CORR:
            try:
                corrected = img.lens_corr(strength=Config.LENS_STRENGTH)
                if corrected is not None:
                    img = corrected
            except Exception as exc:
                # Some MaixPy builds do not expose lens_corr; homography still
                # handles projective distortion, so keep processing the frame.
                if time.time() - last_log > 1.0:
                    last_log = time.time()
                    print("[lens] correction unavailable: %s" % exc)
        gray = _to_gray(img, image)
        result = detector.process(gray)
        _send_uart(uart, result)
        now = time.time()
        dt = now - last
        last = now
        if dt > 1e-4:
            fps = 0.9 * fps + 0.1 / dt
        if disp is not None:
            view = draw_result(gray, result)
            cv2.putText(view, "FPS %.1f" % fps, (12, view.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            disp.show(image.cv2image(view, bgr=True, copy=True))
        if result.get("valid") and now - last_log > 1.0:
            last_log = now
            print("[rect-center] u=%.1f v=%.1f contrast=%.1f" %
                  (result["u"], result["v"], result["min_contrast"]))
    try:
        cam.close()
    except Exception:
        pass


if __name__ == "__main__":
    main()
