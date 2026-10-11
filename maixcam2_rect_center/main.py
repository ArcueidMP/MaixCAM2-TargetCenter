"""MaixCAM2 entry point for perspective robust black rectangle center detection.

上电行为（v0.4.1 起）：
  使能 → 建显示/触摸 → 进入主循环（识别 + 光斑偏差 + 云台闭环）。
  **上电不再自动跑自检**；自检改为屏幕右上角的 `SELF-TEST` 按钮，点一下才触发
  （见 `Config.ENABLE_TOUCH_BUTTON`）。`CALIBRATE_AXIS` 非空时仍走标定（标定期不闭环）。

画面流转到 MaixVision：由 `_Preview` 在主进程内把带标注的画面按"屏幕尺寸画布"letterbox
后 `display.Display().show()` —— 这是 MaixVision 能看到画面的成熟路径（PC 上无 maix 时自动跳过）。
"""

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

BUILD_ID = "2026-10-11-v0.4.1"


class _Preview:
    """主进程内显示（MaixVision 可见）：标注画面 + 可点击按钮 → 屏幕尺寸画布 → `show()`。

    为什么画布做成"屏幕尺寸"：MaixPy 触摸返回的是**屏幕坐标**，只有画布尺寸 == 屏幕尺寸，
    触摸坐标才能与画布/按钮坐标 **1:1** 对应，按钮命中判定才可靠（MaixPy「触摸屏使用」示例即如此）。
    标注图按等比例 letterbox 居中，不拉伸变形。
    `Config.SHOW=False` 时不建 Display；无显示 / 建失败 / show 失败只打日志并停用，绝不影响识别与闭环。
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.display = None
        self.width = self.height = 0
        self.enabled = False
        self.buttons = {}
        self.shown = 0
        self.fps = 0.0
        self.show_max_ms = 0.0
        self._period = 1.0 / max(1, int(getattr(cfg, "SHOW_FPS", 15)))
        self._t_show = -1e9
        self._fps_t = None
        self._fps_n = 0
        self._canvas = None
        if not bool(getattr(cfg, "SHOW", False)):
            print("[preview] SHOW=False -> 不建显示（纯串口/头部模式，画面不流转）")
            return
        try:
            from maix import display
            self.display = display.Display()
            self.width = int(self.display.width())
            self.height = int(self.display.height())
            self.enabled = True
            print("[preview] mode=in-process %dx%d show_fps=%d —— MaixVision 看不到画面时先确认本行"
                  % (self.width, self.height, int(getattr(cfg, "SHOW_FPS", 15))))
        except Exception as exc:
            print("[preview] 显示初始化失败（不影响识别/闭环，仅无预览）: %s" % exc)

    # ------------------------------------------------------------------ 按钮
    def add_button(self, name, x0, y0, x1, y1):
        """注册一个按钮（画布=屏幕坐标）。"""
        self.buttons[name] = (int(x0), int(y0), int(x1), int(y1))

    def hit(self, x, y):
        """触摸点 (x, y) 命中的按钮名；未命中返回 None。"""
        for name, (x0, y0, x1, y1) in self.buttons.items():
            if x0 <= x <= x1 and y0 <= y <= y1:
                return name
        return None

    # ------------------------------------------------------------------ 渲染
    def due(self, now):
        """是否到了本次显示节拍（按 SHOW_FPS 限频，避免拖累主循环）。"""
        return self.enabled and (float(now) - self._t_show) >= self._period

    def render(self, view, now, status=""):
        """把标注后的 `view`（任意尺寸）letterbox 到屏幕画布、叠加按钮与状态，再 show。"""
        if not self.enabled:
            return
        now = float(now)
        self._t_show = now
        vh, vw = view.shape[:2]
        scale = min(self.width / float(vw), self.height / float(vh))
        nw, nh = max(1, int(round(vw * scale))), max(1, int(round(vh * scale)))
        ox, oy = (self.width - nw) // 2, (self.height - nh) // 2
        if self._canvas is None or self._canvas.shape[:2] != (self.height, self.width):
            self._canvas = np.zeros((self.height, self.width, 3), np.uint8)
        else:
            self._canvas[:] = 0
        self._canvas[oy:oy + nh, ox:ox + nw] = cv2.resize(view, (nw, nh))
        self._draw_buttons(status)
        t0 = time.perf_counter()
        try:
            from maix import image as maix_image
            self.display.show(maix_image.cv2image(self._canvas, bgr=True, copy=True))
        except Exception as exc:
            self.enabled = False
            print("[preview] show 失败，停用显示: %s" % exc)
            return
        ms = (time.perf_counter() - t0) * 1000.0
        self.show_max_ms = max(self.show_max_ms, ms)
        self.shown += 1
        if self._fps_t is None:
            self._fps_t = now
            self._fps_n = 0
        elif now - self._fps_t >= 0.5:
            self.fps = self._fps_n / max(1e-6, now - self._fps_t)
            self._fps_n = 0
            self._fps_t = now
        else:
            self._fps_n += 1

    def _draw_buttons(self, status):
        # 用 ASCII 标签：OpenCV 的 Hershey 字体不支持中文，画中文会变问号/方块。
        for name, (x0, y0, x1, y1) in self.buttons.items():
            cv2.rectangle(self._canvas, (x0, y0), (x1, y1), (46, 150, 60), -1)
            cv2.rectangle(self._canvas, (x0, y0), (x1, y1), (235, 235, 235), 1)
            label = "SELF-TEST" if name == "selftest" else name.upper()
            tw = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)[0][0]
            cv2.putText(self._canvas, label,
                        (x0 + max(4, (x1 - x0 - tw) // 2), y0 + (y1 - y0) // 2 + 7),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
        if status:
            cv2.putText(self._canvas, status, (8, self.height - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

    def close(self):
        if self.display is not None:
            try:
                self.display.close()
            except Exception:
                pass


class _Touch:
    """触摸屏输入：把点击映射到 `_Preview` 的按钮（按下沿触发）。

    `maix.touchscreen.TouchScreen().read()` 立即返回 `(x, y, pressed)`（MaixPy v4）。
    无触摸屏 / 未启用 / 无显示时自动禁用，只打日志。
    """

    def __init__(self, cfg):
        self.ts = None
        self._down = False
        if not bool(getattr(cfg, "ENABLE_TOUCH_BUTTON", True)):
            print("[touch] 触摸按钮已关闭（ENABLE_TOUCH_BUTTON=False）")
            return
        try:
            from maix import touchscreen
            self.ts = touchscreen.TouchScreen()
            print("[touch] 触摸屏就绪：屏幕按钮可点击")
        except Exception as exc:
            print("[touch] 触摸不可用（按钮禁用）: %s" % exc)

    def poll(self, preview):
        """返回本帧**新按下**的按钮名（按下沿）；无则 None。"""
        if self.ts is None or preview is None or not preview.enabled:
            return None
        try:
            x, y, pressed = self.ts.read()
        except Exception:
            return None
        if pressed:
            if not self._down:
                self._down = True
                return preview.hit(int(x), int(y))
            return None
        self._down = False
        return None


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
    from maix import app, camera, image

    # 小图上的多线程调度常常比运算本身更贵，也避免 ARM 多核线程争用。
    cv2.setNumThreads(Config.OPENCV_THREADS)

    cam = camera.Camera(Config.CAMERA_W, Config.CAMERA_H,
                        image.Format.FMT_GRAYSCALE,
                        fps=Config.CAMERA_FPS,
                        buff_num=Config.CAMERA_BUFFERS)
    cam.skip_frames(Config.SKIP_FRAMES)
    uart = None
    if Config.ENABLE_UART:
        try:
            from maix import uart as maix_uart
            uart = maix_uart.UART(Config.UART_PORT, Config.UART_BAUD)
        except Exception as exc:
            print("[uart] open failed: %s" % exc)
    link = None
    aim = None
    self_test = None          # ★ 上电不再自动创建；由屏幕按钮触发时才建
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
                # 未配置标定轴：正常进入（按钮式）自检 + 跟踪。
                calibrator = None
            else:
                print("[gimbal] calibration mode axis=%s（本次只标定，不闭环）" % calibrator.axis)
            print("[gimbal] enabled yaw=%s(ID%d) pitch=%s(ID%d) acc=%d" %
                  (Config.YAW_DEV, Config.ADDR_YAW, Config.PITCH_DEV, Config.ADDR_PITCH,
                   Config.ACC_DEFAULT))
        except Exception as exc:
            print("[gimbal] disabled after init failure: %s" % exc)

    # ---- 显示（把画面流转到 MaixVision）+ 屏幕按钮 ----
    preview = _Preview(Config)
    touch = None
    if preview.enabled and bool(getattr(Config, "ENABLE_TOUCH_BUTTON", True)):
        bw = max(70, int(preview.width * 0.30))
        bh = max(34, int(preview.height * 0.20))
        preview.add_button("selftest", preview.width - bw - 8, 8, preview.width - 8, 8 + bh)
        touch = _Touch(Config)
    elif preview.enabled:
        print("[touch] ENABLE_TOUCH_BUTTON=False -> 不画按钮")
    elif bool(getattr(Config, "ENABLE_TOUCH_BUTTON", True)):
        print("[touch] 无显示 -> 屏幕按钮不可用（自检需有显示才能点击触发；SHOW=True 才有）")

    detector = RectangleCenterDetector(Config)
    laser_det = LaserSpotDetector(Config) if Config.ENABLE_LASER else None
    # fixed 模式（光斑位置写死）：开机取一次常量，之后全程复用 —— 不做任何图像检测，
    # 每帧只剩"黑框中心 − 该常量"两次减法。detect 模式才需要每帧调用 process()。
    laser_static = (laser_det.mode == "fixed") if laser_det is not None else False
    aim_use_raw = bool(getattr(Config, "AIM_USE_RAW", False))
    last = time.perf_counter()
    last_log = last
    totals = dict(read=0.0, bridge=0.0, detect=0.0, laser=0.0,
                  uart=0.0, rx=0.0, aim=0.0, show=0.0, loop=0.0)
    count, searches = 0, 0
    laser_state = laser_det.static_state() if laser_static else None
    laser_ms, laser_calls, laser_hits = 0.0, 0, 0
    fps = 0.0
    mean_dt = None
    print("[rect-center] %s search_width=%d display_width=%d display_fps=%d" %
          (BUILD_ID, Config.SEARCH_WIDTH, Config.SHOW_WIDTH, Config.SHOW_FPS))
    print("[rect-center] 四边窄带跟踪 / 失锁后低分辨率搜索 / 72 点逆投影")
    print("[ui] 上电自检已改为屏幕按钮：点显示右上角 SELF-TEST 触发（不再上电自动跑）")
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
        # ⚠ `_ingest()` 无故障时返回的是 `fault=False`（不是 None）：必须按**真值**判故障，
        #   写成 `is not None` 会让开机第 1 帧就误判 FAULT（v3 基线的回归坑，已修）。
        link_fault = bool(fb.get("fault")) if fb else False
        link_reason = fb.get("reason") if fb else None
        aim_started = time.perf_counter()
        if link_fault and aim is not None and aim.state != "FAULT":
            print("[gimbal] link fault: %s -> 立即停止并进入 FAULT" % link_reason)
            link.stop(now=after_uart)
            aim.fault("link:%s" % link_reason)
        # ---- 屏幕按钮：点击触发上电自检（上电不再自动跑）----
        if touch is not None and link is not None and aim is not None:
            if touch.poll(preview) == "selftest":
                if self_test is not None and not self_test.done:
                    print("[ui] 自检进行中，忽略重复点击")
                elif aim.state == "FAULT":
                    print("[ui] 链路 FAULT，拒绝自检")
                elif not bool(getattr(Config, "GIMBAL_SELF_TEST", True)):
                    print("[ui] GIMBAL_SELF_TEST=False，忽略自检")
                else:
                    self_test = GimbalSelfTest(Config, link, after_uart)
                    print("[ui] 收到点击 -> 启动上电自检（%d 步）" % len(self_test.sequence))
        if calibrator is not None:
            if not calibrator.done:
                calibrator.update(after_uart, result)
        elif self_test is not None and not self_test.done:
            self_test.update(after_uart)
        elif aim is not None and link is not None and aim.state != "FAULT":
            aim_result = aim.update(result, now=after_uart)
            for axis, command in aim_result["cmds"].items():
                if command is not None:
                    ccw, pulses, rpm = command
                    link.move(axis, ccw, pulses, rpm, Config.ACC_DEFAULT, now=after_uart)
        aim_elapsed = time.perf_counter() - aim_started
        after_aim = time.perf_counter()
        # ---- 显示：把带标注的画面 + 按钮流转到 MaixVision ----
        if preview.due(after_uart):
            view = draw_result(gray, result, Config.SHOW_WIDTH,
                               top_offset=PANEL_H if laser_state is not None else 0)
            if laser_state is not None:
                draw_laser_overlay(view, laser_state, result,
                                   min(1.0, float(Config.SHOW_WIDTH) / gray.shape[1]),
                                   use_raw=aim_use_raw)
            cv2.putText(view, "loop %.1f  disp %.1f fps" % (fps, preview.fps),
                        (12, view.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            status = ""
            if self_test is not None and not self_test.done:
                status = "self-test %d/%d" % (self_test.index, len(self_test.sequence))
            elif calibrator is not None and not calibrator.done:
                status = "calibrate %s" % calibrator.axis
            preview.render(view, after_uart, status=status)
        after_show = time.perf_counter()
        now = after_show
        dt = now - last
        last = now
        if dt > 1e-4:
            mean_dt = dt if mean_dt is None else 0.9 * mean_dt + 0.1 * dt
            fps = 1.0 / mean_dt
        phases = (after_read - start,
                  after_bridge - after_read, after_detect - after_bridge,
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
            print("[perf] %s valid=%s FPS=%.1f disp=%.1f stage=%s frames=%d searches=%d "
                  "read=%.1f bridge=%.1f detect=%.1f laser=%.1f "
                  "uart=%.1f rx=%.1f aim=%.1f show=%.1f loop=%.1f ms | laser_call=%.2fms n=%d hit=%d" %
                  ((BUILD_ID, result["valid"], window_fps, preview.fps, result["stage"], count, searches) +
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
    preview.close()
    try:
        cam.close()
    except Exception:
        pass


if __name__ == "__main__":
    main()
