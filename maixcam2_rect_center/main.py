"""MaixCAM2 entry point for perspective robust black rectangle center detection.

上电行为（v0.4.1 起）：
  使能 → 建显示/触摸 → 进入主循环（识别 + 光斑偏差 + 云台闭环）。
  **上电不再自动跑自检**；自检改为屏幕右上角的 `SELF-TEST` 按钮，点一下才触发
  （见 `Config.ENABLE_TOUCH_BUTTON`）。`CALIBRATE_AXIS` 非空时仍走标定（标定期不闭环）。

画面流转到 MaixVision / 板载屏幕：`vision/preview.py` 的 `Preview` 把带标注的画面 letterbox
进"屏幕尺寸画布"，再交给**独立显示线程**去 `display.Display().show()`（仍在主进程内 —— 这是
MaixVision 能看到画面的成熟路径；PC 上无 maix 时自动跳过）。主循环只负责绘制与投递，
**永不等待 show**。

================================================================================
★ 模块地图（v0.4.6 起 main.py 已按功能拆分为独立模块）—— 改哪块只开哪个文件
================================================================================
  vision/rate_meter.py      FrameRate         三路口径的滑动窗口帧率计（capture/disp/detect）
  vision/camera_source.py   to_gray           相机帧 → 灰度 ndarray（零拷贝借用相机缓冲）
  vision/vision_worker.py   LatestFrameWorker 识别线程 + 最新帧槽            （时间轴 B）
  vision/preview.py         Preview / TouchButtons / display_result /
                            apply_maixvision_stream_policy                   （时间轴 C：显示）
  vision/aim_control.py     VisionControl     闭环调度中枢（识别线程独占 UART）
  vision/telemetry.py       format_perf_line  [perf] 日志组装
  vision/rectangle_center.py · laser_spot.py · gimbal_aim.py ·
  gimbal_calibrate.py · gimbal_self_test.py · stepper_link.py  几何与执行层（未改动）

  本文件（main.py）只负责三件事：**BUILD_ID · 装配（相机/云台/显示/线程）· 主循环（时间轴 A）**。
  运行行为与拆分前 100% 一致（日志字符串逐字不变；旧私有名以别名保留，入口测试零改动）。

================================================================================
★ 图像显示运行逻辑（自"取帧"到"上屏"的完整链路）—— 本文件读法总纲
================================================================================
本应用跑着**三条独立的时间轴**。理解显示前必须先分清它们；把三者混在一起，就会把
"图传慢"误诊为"识别慢"（这正是本项目历史上踩过的坑，见 B 轨 README「v9→v17 追踪手感对照」）。

  时间轴 A —— 主循环（`main()`，主线程；决定采图/控制节奏）
    ① cam.read()                      从 MaixCAM2 相机取一帧（灰度 FMT_GRAYSCALE）
    ② to_gray(img, image)             借相机缓冲 → 灰度 ndarray（copy=False，零拷贝）
    ③ touch.poll(preview)             触摸"按下沿"→ 置 controller.request_test（触发自检）
    ④ worker.submit(gray, captured)   把**当帧灰度图的副本**投进识别线程（不等待结果）
    ⑤ worker.snapshot(now)            取回**最近一次**识别结果（可能不是本帧，允许滞后）
    ⑥ display_result(snapshot,...)    结果超过 RESULT_MAX_AGE_S 即降级为 invalid，不画旧框
    ⑦ preview.due(now)                显示节拍（SHOW_FPS）到了**且画布空闲**才画：
         ├─ draw_result()             画外/内框、四角、靶心十字（vision/rectangle_center.py）
         ├─ draw_laser_overlay()      画光斑标记 + 左上角面板 + 光斑→靶心连线（vision/laser_spot.py）
         └─ preview.render()          letterbox 到画布 → **投给显示线程，立刻返回**
    ⑧ 每 LOG_INTERVAL 打印 [perf]     分相耗时 + 三路帧率（口径分开，勿混读）

  时间轴 B —— 识别线程（`LatestFrameWorker`，daemon 线程；决定识别吞吐）
    "最新帧槽位"取一帧 → controller.process()（识别 + 光斑偏差 + 云台闭环）→ 写回"最新结果槽位"
    → 空闲时走 controller.idle()（继续收 ACK、推进按钮自检）。槽位只保留**最新**一份：
    主循环产帧快于识别时，旧帧被丢弃并计入 worker.dropped（丢帧而非积压）。

  时间轴 C —— 显示线程（`Preview._show_loop`，daemon 线程；决定屏幕刷新）
    取"待上屏画布"槽 → `display.show()`（屏幕 + MaixVision 图传）→ 画布还池。
    画布池固定 2 张（显示线程 1 张 + 主线程 1 张）；主循环要画时池空 → **直接跳过该帧**
    （丢帧，不排队、不等待）。所以 show() 再慢也只慢预览本身。

  为什么必须拆成三条轴：
    · display.show() 走屏幕驱动 / MaixVision 图传，单次可长达数百 ms；同线程时一慢全卡。
    · 识别（Canny / 轮廓 / 单应）耗时不确定；同线程时识别一慢屏幕就卡成幻灯片。
    每条轴各自取"最新"，任一侧变慢只表现为**丢帧 / 滞后**，不会互相拖垮（降级但不崩溃）。
    ★ 历史教训：只把"识别"拆出去是不够的 —— 只要 show() 还在主循环里同步调用，
      主循环就被它按在地上，`submit()` 的节奏随之被拖死，识别线程再快也无帧可算
      （表现为 capture / disp / detect_fps **一起**掉到同一个低位）。故 v0.4.3 把显示也拆出去。

  ★ 隔离原则（贯穿本应用）：显示层任何失败 —— 无 maix、Display 建失败、show 抛异常、
    无触摸屏 —— 一律只打日志并把 `preview.enabled` 置 False，**绝不影响识别、串口下发
    与云台闭环**。识别的正确性请以 [perf] 日志与 target_api 回归为准，不要拿屏幕观感判断。
================================================================================
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
from vision.rectangle_center import draw_result
from vision.laser_spot import draw_laser_overlay, PANEL_H
from vision.gimbal_aim import GimbalAim
from vision.gimbal_self_test import probe_imu
from vision.gimbal_calibrate import GimbalCalibrator
from vision.rate_meter import FrameRate
from vision.camera_source import to_gray
from vision.vision_worker import LatestFrameWorker
from vision.preview import (Preview, TouchButtons, display_result,
                            apply_maixvision_stream_policy)
from vision.aim_control import VisionControl, send_uart
from vision.telemetry import format_perf_line, format_slow_loop, MAIN_LOOP_SLOW_MS

BUILD_ID = "2026-10-12-eager-search-v0.5.1"

# ---- 兼容别名（v0.4.6 模块化后保留旧私有名）--------------------------------
# tools/test_runtime_performance.py 直接 `from main import (_FrameRate, _LatestVision,
# _Preview, _VisionControl, _display_result, _apply_maixvision_stream_policy)`，
# test_main_self_test.py 通过 `import main as entry` 跑真实 main()。保留下列别名后，
# 两个入口测试**零改动**即可通过（实现已下沉到 vision/ 各模块）。
_FrameRate = FrameRate
_LatestVision = LatestFrameWorker
_Preview = Preview
_Touch = TouchButtons
_VisionControl = VisionControl
_display_result = display_result
_to_gray = to_gray
_send_uart = send_uart
_apply_maixvision_stream_policy = apply_maixvision_stream_policy


def main():
    # ★ 先把 MaixVision 图传按配置掐断（若处于 MaixVision 模式）——必须早于建 Display，见 preview.py。
    apply_maixvision_stream_policy(Config)

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
    # 按钮几何按**屏幕坐标**定义：右上角一块 30%宽/20%高 的区域（有下限，保证可点）。
    # 只有"有显示 且 启用触摸按钮"时才注册按钮并建触摸；否则各自打印降级原因。
    preview = Preview(Config, clock=time)
    touch = None
    if preview.enabled and bool(getattr(Config, "ENABLE_TOUCH_BUTTON", True)):
        bw = max(70, int(preview.width * 0.30))
        bh = max(34, int(preview.height * 0.20))
        preview.add_button("selftest", preview.width - bw - 8, 8, preview.width - 8, 8 + bh)
        touch = TouchButtons(Config)
    elif preview.enabled:
        print("[touch] ENABLE_TOUCH_BUTTON=False -> 不画按钮")
    elif bool(getattr(Config, "ENABLE_TOUCH_BUTTON", True)):
        print("[touch] 无显示 -> 屏幕按钮不可用（自检需有显示才能点击触发；SHOW=True 才有）")

    # ---- 线程装配：识别线程（时间轴 B）+ 主循环（时间轴 A）----
    controller = VisionControl(Config, link, aim, calibrator, uart, clock=time)
    worker = LatestFrameWorker(controller.process, controller.idle, Config.VISION_ASYNC, clock=time)
    capture_rate = FrameRate()              # 取帧帧率（capture 口径）
    last_log = time.perf_counter()
    read_times, draw_times, loop_times = [], [], []   # 本日志窗口内的分相耗时样本（ms）
    print("[rect-center] %s search_width=%d display_width=%d display_fps=%d" %
          (BUILD_ID, Config.SEARCH_WIDTH, preview.render_width, Config.SHOW_FPS))
    print("[preview] async=%s 画布=%dx%d（SHOW_WIDTH=%d；0=用屏幕原始宽度）"
          % (getattr(Config, "SHOW_ASYNC", True), preview.render_width, preview.render_height,
             int(getattr(Config, "SHOW_WIDTH", 0) or 0)))
    print("[vision] latest-frame worker=%s max_result_age=%.0fms" %
          (Config.VISION_ASYNC, Config.RESULT_MAX_AGE_S * 1000))
    print("[ui] 上电自检已改为屏幕按钮：点显示右上角 SELF-TEST 触发（不再上电自动跑）")
    try:
        # ============================ 时间轴 A：主循环 ============================
        # 每轮 = 取帧 → 提交识别 → 取最新结果 → 按节拍绘制并投递上屏 → 周期打日志。
        # 全程不等待识别结果，**也不等待 show**（真正上屏在显示线程），二者慢只表现为
        # "标注滞后 / 预览掉帧"，都不拖慢循环本身。
        while not app.need_exit():
            started = time.perf_counter()
            img = cam.read()                             # ① 取帧（灰度）
            if img is None:
                continue
            captured = time.perf_counter()
            read_times.append((captured - started) * 1000)   # read 耗时样本
            gray = to_gray(img, image)                   # ② 零拷贝转灰度 ndarray
            capture_rate.record(captured)
            if touch is not None and touch.poll(preview) == "selftest":
                controller.request_test.set()            # ③ 按键沿 → 通知识别线程起自检
            worker.submit(gray, captured)                # ④ 提交当帧副本给识别线程
            now = time.perf_counter()
            # ⑤ 取最新识别结果（每轮只取一次：日志窗口内不再重复取，省一次加锁 + 帧率统计）
            snapshot, vision_fps, vision_low1 = worker.snapshot(now)
            if worker.error is not None:
                raise RuntimeError("vision worker failed: " + worker.error)   # 识别线程挂了 → 终止
            result, laser_state, age = display_result(snapshot, now, Config.RESULT_MAX_AGE_S)  # ⑥ 过期降级
            if preview.due(now):                         # ⑦ 节拍到 + 画布空闲 → 绘制并投递上屏
                t_draw = time.perf_counter()
                # 画进 BGR 视图；render_width == 原图宽时 draw_result 只做灰度→BGR，不缩放
                view = draw_result(gray, result, preview.render_width,
                                   top_offset=PANEL_H if laser_state is not None else 0)
                if laser_state is not None:
                    # 激光叠加：scale = 画布/原图宽，使光斑标记落在与画面对应的位置
                    draw_laser_overlay(view, laser_state, result,
                                       min(1.0, preview.render_width / float(gray.shape[1])),
                                       use_raw=Config.AIM_USE_RAW)
                # 画面左下角一行小字：disp=显示帧率 low1=最慢1% det=识别帧率 age=结果龄
                cv2.putText(view, "disp %.1f low1 %.1f det %.1f age %.0fms" %
                            (preview.fps, preview.low1, vision_fps, age * 1000),
                            (8, view.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX,
                            0.40, (0, 255, 255), 1)
                preview.render(view, now, status=snapshot["status"] if snapshot else "waiting")
                draw_times.append((time.perf_counter() - t_draw) * 1000.0)   # 绘制+投递（不含 show）
            loop_times.append((time.perf_counter() - started) * 1000.0)
            now = time.perf_counter()
            if now - last_log >= Config.LOG_INTERVAL:    # ⑧ 周期日志：分相耗时 + 三路帧率
                capture_fps, _ = capture_rate.values(now)
                display_fps, display_low1 = preview.rate.values(now)
                show_avg, show_p95, show_max, show_n = preview.take_show_stats()
                read_avg = float(np.mean(read_times)) if read_times else 0.0
                draw_avg = float(np.mean(draw_times)) if draw_times else 0.0
                loop_avg = float(np.mean(loop_times)) if loop_times else 0.0
                detect_ms = snapshot["detect_ms"] if snapshot else 0.0
                # ★ 读法：[perf] 里 capture / disp / detect_fps 是三路**不同**口径，
                #   "眼睛看到的刷新率"看 disp，"识别吞吐"看 detect_fps，别互相换算。
                #   read/draw/loop 属于**主循环**，show 属于**显示线程** —— 分开看就能一眼判定
                #   "到底谁在吃时间"，不必再靠猜（这正是 1.5fps 归因的关键）。
                print(format_perf_line(
                    BUILD_ID, capture_fps, display_fps, display_low1, vision_fps,
                    result, read_avg, draw_avg, loop_avg,
                    show_avg, show_p95, show_max, show_n,
                    age, preview.shown, preview.dropped, worker.dropped))
                if aim is not None:
                    print(aim.status_line())
                if show_p95 > 100:
                    # show 的 p95 超过 100ms ⇒ 瓶颈在**显示侧**（屏幕驱动 / MaixVision 图传），
                    # 不是识别慢。预览会掉帧，但识别与闭环不受影响 —— 判据与对策见 README。
                    preview.warn_slow_once(show_p95)
                if loop_avg > MAIN_LOOP_SLOW_MS:
                    # 主循环单轮超过该阈值 ⇒ 已在 15fps 以下，且此数**不含 show**。两个常见来源：
                    # ① 相机 read 本身偏大；② 显示线程在 MaixVision 图传忙等里空转 CPU，
                    # 把整体算力抢走（此时 read 也会被动变大）。
                    print(format_slow_loop(loop_avg))
                last_log = now
                read_times.clear()
                draw_times.clear()
                loop_times.clear()
    finally:
        # worker 独占 UART；停止并 join 后主线程才能下发退出停机命令。
        try:
            worker.close()
        finally:
            # 工作线程退出后再清理硬件，避免与尚未退出的线程并发操作 UART。
            if worker.thread is None or not worker.thread.is_alive():
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
