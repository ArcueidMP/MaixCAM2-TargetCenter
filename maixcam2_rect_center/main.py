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
from vision.laser_spot import LaserSpotDetector, deviation, draw_laser_overlay, PANEL_H
from vision.gimbal_aim import GimbalAim
from vision.gimbal_self_test import GimbalSelfTest, probe_imu
from vision.gimbal_calibrate import GimbalCalibrator

BUILD_ID = "2026-10-08-target-api-v3"


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

    # 小图上的多线程调度常常比运算本身更贵，也避免 ARM 多核线程争用。
    cv2.setNumThreads(Config.OPENCV_THREADS)

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
    link = None
    aim = None
    self_test = None
    calibrator = None
    if Config.ENABLE_GIMBAL:
        try:
            from vision.stepper_link import StepperLink
            link = StepperLink(Config)
            link.enable()
            aim = GimbalAim(Config)
            imu_info = probe_imu()
            if imu_info["available"]:
                print("[imu] sample via %s: %s (gyro cannot define absolute yaw zero)" %
                      (imu_info["method"], str(imu_info["sample"])))
            else:
                print("[imu] unavailable: %s" % imu_info["error"])
            calibrator = GimbalCalibrator(Config, link, time.perf_counter())
            if calibrator.done:
                # 未配置标定轴：正常进入自检 + 跟踪。
                calibrator = None
                self_test = GimbalSelfTest(Config, link, time.perf_counter())
            else:
                print("[gimbal] calibration mode axis=%s（本次只标定，不闭环）" % calibrator.axis)
            print("[gimbal] enabled yaw=%s pitch=%s addr=%d/%d" %
                  (Config.U1_DEV, Config.U2_DEV, Config.ADDR_YAW, Config.ADDR_PITCH))
        except Exception as exc:
            print("[gimbal] disabled after init failure: %s" % exc)
    detector = RectangleCenterDetector(Config)
    laser_det = LaserSpotDetector(Config) if Config.ENABLE_LASER else None
    # fixed 模式（光斑位置写死）：开机取一次常量，之后全程复用 —— 不做任何图像检测，
    # 每帧只剩"黑框中心 − 该常量"两次减法。detect 模式才需要每帧调用 process()。
    laser_static = (laser_det.mode == "fixed") if laser_det is not None else False
    aim_use_raw = bool(getattr(Config, "AIM_USE_RAW", False))
    last = time.perf_counter()
    last_log = last
    last_show = -1e10
    lens_enabled = Config.LENS_CORR
    totals = dict(read=0.0, lens=0.0, bridge=0.0, detect=0.0, laser=0.0,
                  uart=0.0, rx=0.0, aim=0.0, show=0.0, loop=0.0)
    count, searches = 0, 0
    laser_state = laser_det.static_state() if laser_static else None
    laser_ms, laser_calls, laser_hits = 0.0, 0, 0
    fps = 0.0
    mean_dt = None
    print("[rect-center] %s search_width=%d display_width=%d display_fps=%d" %
          (BUILD_ID, Config.SEARCH_WIDTH, Config.SHOW_WIDTH, Config.SHOW_FPS))
    print("[rect-center] 四边窄带跟踪 / 失锁后低分辨率搜索 / 72 点逆投影")
    print("[aim] 闭环口径=%s（raw_u/raw_v）离群拒收=%spx 死区=%.1fpx 限幅=%dp" %
          ("raw" if aim_use_raw else "smoothed", Config.JUMP_REJECT_PX,
           Config.DEADBAND_PX, Config.MAX_STEP_PULSES))
    if laser_det is not None:
        if laser_static:
            print("[laser] mode=fixed 光斑位置写死 (%d,%d) —— 不做图像检测，每帧只算黑框中心−该常量" %
                  (Config.LASER_U_NOMINAL, Config.LASER_V_NOMINAL))
        else:
            print("[laser] mode=detect 标称位置 (%d,%d) ROI±%d 限频 %.2fs 判据 peak>=max(%.0f, bg+%.0f) 面积<=%d" %
                  (Config.LASER_U_NOMINAL, Config.LASER_V_NOMINAL,
                   Config.LASER_ROI_RADIUS, Config.LASER_INTERVAL_S, Config.LASER_MIN_PEAK,
                   Config.LASER_MIN_CONTRAST, Config.LASER_MAX_PIXELS))
    while not app.need_exit():
        start = time.perf_counter()
        img = cam.read()
        if img is None:
            continue
        after_read = time.perf_counter()
        if lens_enabled:
            try:
                corrected = img.lens_corr(strength=Config.LENS_STRENGTH)
                if corrected is not None:
                    img = corrected
            except Exception as exc:
                lens_enabled = False
                print("[lens] 本次会话停用不支持的 lens_corr: %s" % exc)
        after_lens = time.perf_counter()
        gray = _to_gray(img, image)
        after_bridge = time.perf_counter()
        result = detector.process(gray)
        after_detect = time.perf_counter()
        if laser_det is not None and not laser_static:
            # detect 模式：内部限频，未到 LASER_INTERVAL_S 时不重算，直接沿用上次光斑位置。
            laser_state = laser_det.process(gray, after_detect)
            if laser_state.get("updated"):
                laser_calls += 1
                laser_ms += laser_state["ms"]
                laser_hits += 1 if laser_state["found"] else 0
        dv = None
        if laser_state is not None:
            # fixed 模式走的就是这里：每帧唯一的光斑相关运算 = 两次减法。
            dv = deviation(laser_state, result, use_raw=aim_use_raw)
            if dv is not None:
                # 靶心-光斑偏差；dx/dy 既是串口字段也是闭环输入（符号见 deviation 注释）。
                result["laser_u"], result["laser_v"] = laser_state["u"], laser_state["v"]
                result["dx"], result["dy"] = dv
        after_laser = time.perf_counter()
        result["aim_valid"] = bool(dv is not None)
        _send_uart(uart, result)
        after_uart = time.perf_counter()
        # 下行：非阻塞读两条串口，处理 E2 自动补使能 / ACK 超时 / E2 洪水。
        rx_started = time.perf_counter()
        fb = link.poll(after_uart) if link is not None else None
        rx_elapsed = time.perf_counter() - rx_started
        link_fault = (fb or {}).get("fault")
        aim_started = time.perf_counter()
        if link_fault is not None and aim is not None and aim.state != "FAULT":
            print("[gimbal] link fault: %s -> 立即停止并进入 FAULT" % link_fault)
            link.stop(now=after_uart)
            aim.fault("link:%s" % link_fault)
        if calibrator is not None:
            if not calibrator.done:
                calibrator.update(after_uart, result)
        elif aim is not None and link is not None and aim.state != "FAULT":
            if self_test is not None and not self_test.done:
                self_test.update(after_uart)
            elif self_test is None or self_test.state != "FAULT":
                aim_result = aim.update(result, now=after_uart)
                for axis, command in aim_result["cmds"].items():
                    if command is not None:
                        ccw, pulses, rpm = command
                        link.move(axis, ccw, pulses, rpm, Config.ACC_DEFAULT, now=after_uart)
        aim_elapsed = time.perf_counter() - aim_started
        after_aim = time.perf_counter()
        if disp is not None and (after_uart - last_show >= 1.0 / max(1, Config.SHOW_FPS)):
            last_show = after_uart
            view = draw_result(gray, result, Config.SHOW_WIDTH,
                               top_offset=PANEL_H if laser_state is not None else 0)
            if laser_state is not None:
                draw_laser_overlay(view, laser_state, result,
                                   min(1.0, float(Config.SHOW_WIDTH) / gray.shape[1]),
                                   use_raw=aim_use_raw)
            cv2.putText(view, "FPS %.1f" % fps, (12, view.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            disp.show(image.cv2image(view, bgr=True, copy=True))
        after_show = time.perf_counter()
        now = after_show
        dt = now - last
        last = now
        if dt > 1e-4:
            mean_dt = dt if mean_dt is None else 0.9 * mean_dt + 0.1 * dt
            fps = 1.0 / mean_dt
        phases = (after_read - start, after_lens - after_read,
                  after_bridge - after_lens, after_detect - after_bridge,
                  after_laser - after_detect, after_uart - after_laser,
                  rx_elapsed, aim_elapsed, after_show - after_aim, now - start)
        for key, elapsed in zip(totals, phases):
            totals[key] += elapsed * 1000.0
        count += 1
        searches += result.get("search_count", 0)
        if now - last_log > Config.LOG_INTERVAL:
            window_fps = count / max(1e-6, now - last_log)
            last_log = now
            laser_avg = laser_ms / laser_calls if laser_calls else 0.0
            print("[perf] %s valid=%s FPS=%.1f stage=%s frames=%d searches=%d "
                  "read=%.1f lens=%.1f bridge=%.1f detect=%.1f laser=%.1f "
                  "uart=%.1f rx=%.1f aim=%.1f show=%.1f loop=%.1f ms | laser_call=%.2fms n=%d hit=%d" %
                  ((BUILD_ID, result["valid"], window_fps, result["stage"], count, searches) +
                   tuple(totals[k] / count for k in totals) +
                   (laser_avg, laser_calls, laser_hits)))
            if aim is not None:
                print(aim.status_line())
            count, searches = 0, 0
            laser_ms, laser_calls, laser_hits = 0.0, 0, 0
            for key in totals:
                totals[key] = 0.0
    if link is not None:
        try:
            link.stop()
        except Exception as exc:
            print("[gimbal] stop on exit failed: %s" % exc)
    try:
        cam.close()
    except Exception:
        pass


if __name__ == "__main__":
    main()
