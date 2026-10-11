"""MaixCAM2 entry point for perspective robust black rectangle center detection.

上电行为（v0.4.1 起）：
  使能 → 建显示/触摸 → 进入主循环（识别 + 光斑偏差 + 云台闭环）。
  **上电不再自动跑自检**；自检改为屏幕右上角的 `SELF-TEST` 按钮，点一下才触发
  （见 `Config.ENABLE_TOUCH_BUTTON`）。`CALIBRATE_AXIS` 非空时仍走标定（标定期不闭环）。

画面流转到 MaixVision / 板载屏幕：`_Preview` 把带标注的画面 letterbox 进"屏幕尺寸画布"，
再交给**独立显示线程**去 `display.Display().show()`（仍在主进程内 —— 这是 MaixVision 能看到
画面的成熟路径；PC 上无 maix 时自动跳过）。主循环只负责绘制与投递，**永不等待 show**。

================================================================================
★ 图像显示运行逻辑（自"取帧"到"上屏"的完整链路）—— 本文件读法总纲
================================================================================
本文件里跑着**三条独立的时间轴**。理解显示前必须先分清它们；把二者混在一起，就会把
"图传慢"误诊为"识别慢"（这正是本项目历史上踩过的坑，见 B 轨 README「v9→v17 追踪手感对照」）。

  时间轴 A —— 主循环（`main()`，主线程；决定采图/控制节奏）
    ① cam.read()                      从 MaixCAM2 相机取一帧（灰度 FMT_GRAYSCALE）
    ② _to_gray(img, image)            借相机缓冲 → 灰度 ndarray（copy=False，零拷贝）
    ③ touch.poll(preview)             触摸"按下沿"→ 置 controller.request_test（触发自检）
    ④ worker.submit(gray, captured)   把**当帧灰度图的副本**投进识别线程（不等待结果）
    ⑤ worker.snapshot(now)            取回**最近一次**识别结果（可能不是本帧，允许滞后）
    ⑥ _display_result(snapshot,...)   结果超过 RESULT_MAX_AGE_S 即降级为 invalid，不画旧框
    ⑦ preview.due(now)                显示节拍（SHOW_FPS）到了**且画布空闲**才画：
         ├─ draw_result()             画外/内框、四角、靶心十字（vision/rectangle_center.py）
         ├─ draw_laser_overlay()      画光斑标记 + 左上角面板 + 光斑→靶心连线（vision/laser_spot.py）
         └─ preview.render()          letterbox 到画布 → **投给显示线程，立刻返回**
    ⑧ 每 LOG_INTERVAL 打印 [perf]     分相耗时 + 三路帧率（口径分开，勿混读）

  时间轴 B —— 识别线程（`_LatestVision`，daemon 线程；决定识别吞吐）
    "最新帧槽位"取一帧 → controller.process()（识别 + 光斑偏差 + 云台闭环）→ 写回"最新结果槽位"
    → 空闲时走 controller.idle()（继续收 ACK、推进按钮自检）。槽位只保留**最新**一份：
    主循环产帧快于识别时，旧帧被丢弃并计入 worker.dropped（丢帧而非积压）。

  时间轴 C —— 显示线程（`_Preview._show_loop`，daemon 线程；决定屏幕刷新）
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

  ★ 隔离原则（贯穿本文件）：显示层任何失败 —— 无 maix、Display 建失败、show 抛异常、
    无触摸屏 —— 一律只打日志并把 `preview.enabled` 置 False，**绝不影响识别、串口下发
    与云台闭环**。识别的正确性请以 [perf] 日志与 target_api 回归为准，不要拿屏幕观感判断。
================================================================================
"""

import os
import sys
import time
import threading
from collections import deque

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

BUILD_ID = "2026-10-11-display-thread-v0.4.3"


class _FrameRate:
    """滑动窗口帧率计。按"成功完成的事件时刻"计时；返回 (平均fps, low1)。

    本类被三处复用（各自独立实例，口径不同、切勿混读）：
        capture_rate —— 主循环 cam.read() 速率   → [perf] capture=
        preview.rate —— 显示 show() 完成速率      → [perf] disp= / 屏幕文字 disp
        worker.rate  —— 识别帧处理完成速率        → [perf] detect_fps=

    low1 = "最慢 1% 帧间隔的倒数"：用于暴露**卡顿**——平均值好看但偶发长停顿（如 JPEG 图传
    抖动）时，fps 可能仍高，而 low1 会被那几帧长间隔拉低，是比平均值更诚实的体感指标。
    """

    def __init__(self):
        self.total = 0
        self._times = deque(maxlen=120)   # 只留最近 120 个时刻，足够覆盖 2s 窗口

    def record(self, now):
        self.total += 1
        self._times.append(float(now))

    def values(self, now):
        ts = list(self._times)
        if len(ts) < 2:
            return 0.0, 0.0
        # 只看最近两秒内的时刻：统计窗口滚动，旧的读数随时间长自然淡出。
        # 关键：把"距上一个事件已过去的时间(stall)"也算作一次间隔 —— 否则一旦卡住
        # （长时间没有新事件），窗口里只剩早先的高速样本，会把上次的快读数永久挂在屏幕上。
        recent = [t for t in ts if t >= float(now) - 2.0]
        if len(recent) < 2:
            return 0.0, 0.0
        gaps = np.diff(recent)
        stall = float(now) - recent[-1]
        if stall > gaps.max():
            gaps = np.append(gaps, stall)          # 把"当前这段停顿"并入间隔序列
        fps = (len(recent) - 1) / max(1e-6, float(now) - recent[0])   # 平均帧率
        slow = np.sort(gaps)[-max(1, int(np.ceil(len(gaps) * 0.01))):]  # 最慢的 1% 间隔
        return fps, 1.0 / max(1e-6, float(np.mean(slow)))              # low1


class _LatestVision:
    """单识别线程 + 两个"最新槽位"：一个待处理帧槽(_pending)、一个结果槽(_latest)。

    线程模型（时间轴 B 的实现，见模块开头总纲）：
        · 主线程（生产者）：submit() 往 _pending 放**最新一帧**的副本；
        · 识别线程（消费者）：取走 _pending → callback() 处理 → 结果写 _latest；
        · 主线程（读者）：snapshot() 无锁等待地取回 _latest（可能滞后于当前摄像头帧）。

    "最新优先、丢弃旧帧"是刻意的：识别跟不上时保留最新画面才有意义，积压旧帧只会让
    追踪越追越偏。Camera / Display / 触摸这些**必须独占主线程**的资源一概不跨线程。
    """

    def __init__(self, callback, idle=None, asynchronous=True):
        self.callback, self.idle = callback, idle
        self._condition = threading.Condition()   # 保护 _pending / _latest / _stop
        self._pending = self._latest = None
        self._stop = False
        self.error = None
        self.dropped = 0                          # 因来不及处理而被覆盖的帧数（丢帧计数）
        self.rate = _FrameRate()                  # 识别帧率（detect_fps 口径）
        self.thread = None
        if asynchronous:
            # daemon=True：进程退出时线程自动结束，不必显式 join 也能安全收摊。
            self.thread = threading.Thread(target=self._run, name="rect-vision", daemon=True)
            self.thread.start()

    def submit(self, gray, captured_at):
        # ★ 必须拷贝：image2cv(copy=False) 借用的相机内部缓冲会被**下一次 read 覆写**，
        #   识别线程还没处理完就可能已被改写 → 后台线程必须持有自己的一份灰度图。
        item = (np.array(gray, dtype=np.uint8, order="C", copy=True), float(captured_at))
        if self.thread is None:
            self._process(item)          # 同步模式（VISION_ASYNC=False）：主线程内直接算
            return
        with self._condition:
            if self._stop:
                return
            if self._pending is not None:
                self.dropped += 1        # 上一帧还没被取走就被覆盖 → 记一次丢帧
            self._pending = item
            self._condition.notify()     # 唤醒识别线程

    def _process(self, item):
        snapshot = self.callback(*item)  # callback = _VisionControl.process（识别+偏差+闭环）
        with self._condition:
            self._latest = snapshot
            self.rate.record(time.perf_counter())

    def _run(self):
        """识别线程主体：循环取最新帧处理；无帧时走 idle() 维持串口保活。"""
        try:
            while True:
                with self._condition:
                    if self._pending is None and not self._stop:
                        # 20ms 超时轮询（而非无限 wait），保证 stop 能被及时看到。
                        self._condition.wait(timeout=0.02)
                    if self._stop:
                        return
                    item, self._pending = self._pending, None
                if item is not None:
                    self._process(item)
                elif self.idle is not None:
                    # 有帧就处理、没帧就保活：空闲时继续收 ACK/推进按钮自检，但不重发旧误差。
                    self.idle()
        except Exception as exc:
            # 识别线程异常不能静默：记下原因，主循环下一轮会读到 error 并抛出终止。
            with self._condition:
                self.error = repr(exc)
                self._latest = None
            print("[vision] worker failed: %s" % exc)

    def snapshot(self, now):
        """取回最近一次识别结果 + 识别帧率（主线程调用，非阻塞）。"""
        with self._condition:
            fps, low1 = self.rate.values(now)
            return self._latest, fps, low1

    def close(self):
        """停线程；必须等它真正退出——UART 归识别线程独占，未退干净就动硬件会打架。"""
        with self._condition:
            self._stop = True
            self._pending = None
            self._condition.notify()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
            if self.thread.is_alive():
                # 线程没停就动 UART/相机是竞态；宁可抛错让上层看见，也不静默降级。
                raise RuntimeError("vision worker did not stop; UART remains owned by worker")


class _Preview:
    """显示层：标注画面 →（letterbox）画布 → `display.show()`，附屏幕可点击按钮。

    ★ 线程模型（时间轴 C，v0.4.3 起）
        主循环（生产者）  `render(view, ...)` —— letterbox + 叠按钮，把画布塞进"待上屏槽"就返回；
        显示线程（消费者） `_show_loop()`     —— 取最新画布 → `show()` → 画布还回池子。
        画布池固定 2 张：显示线程占 1 张、主线程画 1 张；两张都被占用时 `due()` 直接返回 False
        （丢帧，不排队、不等待）。因此 **show() 再慢也只表现为预览掉帧**，绝不拖慢主循环、
        识别线程与云台闭环 —— 这正是"把识别拆出去"之外还必须做的一步（见模块开头总纲）。
        `Config.SHOW_ASYNC=False` 时退回旧行为（主循环内同步 `show()`）。

    为什么画布做成"屏幕尺寸"：MaixPy 触摸返回的是**屏幕坐标**，只有画布尺寸 == 屏幕尺寸，
    触摸坐标才能与画布/按钮坐标 **1:1** 对应，按钮命中判定才可靠（MaixPy「触摸屏使用」示例即如此）。
    标注图按等比例 letterbox 居中，不拉伸变形。
    `Config.SHOW=False` 时不建 Display；无显示 / 建失败 / show 失败只打日志并停用，绝不影响识别与闭环。

    两级尺寸务必分清（`render()` 里同时出现，最易混淆）：
        · display.width/height  —— 物理屏幕尺寸，也是**触摸坐标**所在坐标系（按钮按此定义）
        · render_width/height   —— 实际送去 show() 的**画布**尺寸（SHOW_WIDTH 限制）
    默认 `SHOW_WIDTH=0` ⇒ 画布 == 屏幕：`show()` 不再缩放、触摸不再换算；相机 640×480 与
    MaixCAM2 屏幕 640×480 宽高比一致时，letterbox 也退化成零开销（不 resize、不刷黑边）。
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.display = None
        self.width = self.height = 0            # 物理屏幕尺寸（触摸坐标系）
        self.render_width = self.render_height = 0
        self.enabled = False                    # 显示是否可用；任何一步失败都置 False 并停用
        self.buttons = {}                       # name -> (x0,y0,x1,y1)，屏幕坐标
        self._button_draw = []                  # 预算好的按钮绘制参数（画布坐标），渲染时不再换算
        self.shown = 0                          # 已成功 show() 的帧数（disp 口径）
        self.dropped = 0                        # 因显示线程忙而未画的帧数（丢帧，不排队）
        self.fps = 0.0
        self.low1 = 0.0
        self.rate = _FrameRate()                # 显示帧率（disp 口径，与识别帧率分开）
        self._period = 1.0 / max(1, int(getattr(cfg, "SHOW_FPS", 15)))   # 显示节拍周期
        self._t_show = -1e9                     # 下一次该画的时刻（固定时间轴，见 due/render）
        self._canvas = None                     # 同步模式的单张画布（异步模式走 _free 池）
        self._show_times = []                   # 本日志窗口内的 show() 耗时样本（ms）
        self._show_max = 0.0
        # ---- 显示线程状态（时间轴 C）----
        self.async_show = bool(getattr(cfg, "SHOW_ASYNC", True))
        self._cond = threading.Condition()      # 保护 _free / _pending / _pool / _stop
        self._free = []                         # 空闲画布池
        self._pending = None                    # 待上屏画布（只留最新一张）
        self._pool = 0                          # 已分配画布数（上限 2，由 due() 保证）
        self._stop = False
        self.thread = None
        if not bool(getattr(cfg, "SHOW", False)):
            print("[preview] SHOW=False -> 不建显示（纯串口/头部模式，画面不流转）")
            return
        try:
            from maix import display
            self.display = display.Display()
            self.width = int(self.display.width())
            self.height = int(self.display.height())
            # 画布宽度：SHOW_WIDTH<=0 ⇒ 用屏幕原始宽度（画布＝屏幕 ⇒ show() 零缩放）。
            cap = int(getattr(cfg, "SHOW_WIDTH", 0) or 0)
            self.render_width = self.width if cap <= 0 else min(self.width, cap)
            self.render_height = max(1, int(round(self.height * self.render_width / self.width)))
            if hasattr(display, "set_trans_image_quality"):
                display.set_trans_image_quality(int(getattr(cfg, "SHOW_JPEG_QUALITY", 70)))
            self.enabled = True
            print("[preview] mode=%s screen=%dx%d canvas=%dx%d show_fps=%d jpeg=%d"
                  % ("display-thread" if self.async_show else "sync-inprocess",
                     self.width, self.height, self.render_width, self.render_height,
                     int(getattr(cfg, "SHOW_FPS", 30)), int(getattr(cfg, "SHOW_JPEG_QUALITY", 70))))
            if self.async_show:
                self.thread = threading.Thread(target=self._show_loop, name="display", daemon=True)
                self.thread.start()
        except Exception as exc:
            # 建不了显示不算错误：板子仍照常识别 + 跑闭环，只是没有画面。
            print("[preview] 显示初始化失败（不影响识别/闭环，仅无预览）: %s" % exc)

    # ------------------------------------------------------------------ 按钮
    def add_button(self, name, x0, y0, x1, y1):
        """注册一个按钮（**屏幕坐标**）。几何与文本宽度在此**预计算一次**，渲染时不再换算。"""
        self.buttons[name] = (int(x0), int(y0), int(x1), int(y1))
        # 屏幕坐标 → 画布坐标（默认 SHOW_WIDTH=0 时二者相同，比例恒为 1）
        sx = self.render_width / float(self.width) if self.width else 1.0
        sy = self.render_height / float(self.height) if self.height else 1.0
        bx0, bx1 = int(round(x0 * sx)), int(round(x1 * sx))
        by0, by1 = int(round(y0 * sy)), int(round(y1 * sy))
        # 用 ASCII 标签：OpenCV 的 Hershey 字体不支持中文，画中文会变问号/方块。
        label = "SELF-TEST" if name == "selftest" else name.upper()
        tw = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)[0][0]
        pos = (bx0 + max(4, (bx1 - bx0 - tw) // 2), by0 + (by1 - by0) // 2 + 7)   # 水平居中
        self._button_draw.append((bx0, by0, bx1, by1, label, pos))

    def hit(self, x, y):
        """触摸点 (x, y)（屏幕坐标）命中的按钮名；未命中返回 None。"""
        for name, (x0, y0, x1, y1) in self.buttons.items():
            if x0 <= x <= x1 and y0 <= y <= y1:
                return name
        return None

    # ------------------------------------------------------------------ 渲染
    def due(self, now):
        """是否到了显示节拍**且**有画布可用；显示线程忙时返回 False（丢帧，绝不等待）。"""
        if not self.enabled or float(now) < self._t_show:
            return False
        if not self.async_show:
            return True
        with self._cond:
            if self._free or self._pool < 2:
                return True
        self.dropped += 1        # 节拍到了但两张画布都占用中 → 本帧不画（丢帧而非排队）
        return False

    def _take_canvas(self):
        """取一张画布：同步模式复用固定那张；异步模式从池里取，池空则新分配（上限由 due 保证）。"""
        need = (self.render_height, self.render_width)
        if not self.async_show:
            if self._canvas is None or self._canvas.shape[:2] != need:
                self._canvas = np.empty((need[0], need[1], 3), np.uint8)
            return self._canvas
        with self._cond:
            if self._free:
                return self._free.pop()
            self._pool += 1
        return np.empty((need[0], need[1], 3), np.uint8)

    def render(self, view, now, status=""):
        """把标注后的 `view`（任意尺寸）letterbox 到画布、叠按钮与状态，然后交给显示线程。

        步骤（对应模块总纲时间轴 A 的第 ⑦ 步）：
            1) 推进节拍 _t_show（固定时间轴，见下）
            2) 算 letterbox 缩放与居中偏移 (scale/ox/oy)，把 view 放进画布
            3) 叠按钮/状态条
            4) 交显示 —— 异步：投递后**立即返回**；同步：当场 `show()`
        """
        if not self.enabled:
            return
        now = float(now)
        # ★ 固定时间轴限频：节拍由"上一次预期时刻 + 周期"推进，而不是"show 完成后再等一周期"。
        #   好处：即使某次 show() 慢，也不会让后续节拍越拖越后（避免累积漂移）；
        #   已经错过的节拍用 skipped 一次性跳过，只画最新画面。
        if self._t_show < 0:
            self._t_show = now + self._period
        else:
            skipped = max(1, int((now - self._t_show) / self._period) + 1)
            self._t_show += skipped * self._period
        canvas = self._take_canvas()
        # ---- letterbox：等比缩放 + 居中，两侧留黑边，绝不拉伸（保持几何不变形）----
        vh, vw = view.shape[:2]
        rw, rh = self.render_width, self.render_height
        scale = min(rw / float(vw), rh / float(vh))         # 取两轴较小比例 → 完整装入
        nw, nh = max(1, int(round(vw * scale))), max(1, int(round(vh * scale)))
        ox, oy = (rw - nw) // 2, (rh - nh) // 2             # 居中偏移
        if (nw, nh) != (rw, rh):
            canvas[:] = 0                                   # 只有真存在黑边时才需要刷黑背景
        canvas[oy:oy + nh, ox:ox + nw] = (
            view if (nw, nh) == (vw, vh) else cv2.resize(view, (nw, nh)))
        self._draw_buttons(canvas, status)                   # 按钮与状态条画在画布之上
        if self.async_show:
            self._publish(canvas)                            # 投递即返回：show 交给显示线程
        else:
            self._do_show(canvas)

    def _publish(self, canvas):
        """把画布放进"待上屏槽"（只留最新一张）并唤醒显示线程；主循环不等它。"""
        with self._cond:
            if self._stop:
                return
            if self._pending is not None:
                self._free.append(self._pending)             # 上一张还没上屏 → 退回池（丢帧）
                self.dropped += 1
            self._pending = canvas
            self._cond.notify()

    def _show_loop(self):
        """显示线程主体（时间轴 C）：取最新画布 → show() → 画布还池。**慢也只慢自己**。"""
        try:
            from maix import image as maix_image
        except Exception as exc:
            print("[preview] 显示线程无法导入 maix.image，退化为无预览: %s" % exc)
            return
        while True:
            with self._cond:
                if self._pending is None and not self._stop:
                    self._cond.wait(timeout=0.02)            # 20ms 轮询，保证 stop 能被及时看到
                if self._stop:
                    return
                canvas, self._pending = self._pending, None
            if canvas is None:
                continue
            if not self._do_show(canvas, maix_image):
                return                                       # show 已失败并停用显示 → 线程退出
            with self._cond:
                self._free.append(canvas)

    def _do_show(self, canvas, maix_image=None):
        """真正调用 display.show()，记耗时/帧率；失败即停用显示（不影响识别/闭环）。返回是否成功。"""
        if maix_image is None:
            from maix import image as maix_image
        t0 = time.perf_counter()
        try:
            # bgr=True：cv2 是 BGR 序；copy=True：交给显示的必须是独立缓冲。
            self.display.show(maix_image.cv2image(canvas, bgr=True, copy=True))
        except Exception as exc:
            self.enabled = False
            print("[preview] show 失败，停用显示: %s" % exc)
            return False
        ms = (time.perf_counter() - t0) * 1000.0
        self.shown += 1
        completed = time.perf_counter()
        self.rate.record(completed)
        self.fps, self.low1 = self.rate.values(completed)
        with self._cond:
            self._show_times.append(ms)
            if ms > self._show_max:
                self._show_max = ms
        return True

    def take_show_stats(self):
        """取走并清空本日志窗口的 show() 耗时样本 → (avg, p95, max, n)，单位 ms。"""
        with self._cond:
            times, worst = self._show_times, self._show_max
            self._show_times, self._show_max = [], 0.0
        if not times:
            return 0.0, 0.0, worst, 0
        arr = np.asarray(times, dtype=np.float64)
        return float(arr.mean()), float(np.percentile(arr, 95)), worst, len(times)

    def _draw_buttons(self, canvas, status=""):
        # 按钮几何/文本宽度已在 add_button() 预算好，这里只画（不再每帧 getTextSize/换算）。
        for (x0, y0, x1, y1, label, pos) in self._button_draw:
            cv2.rectangle(canvas, (x0, y0), (x1, y1), (46, 150, 60), -1)   # 填充
            cv2.rectangle(canvas, (x0, y0), (x1, y1), (235, 235, 235), 1)  # 描边
            cv2.putText(canvas, label, pos,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
        if status:
            # 左下角状态串：self-test i/n、calibrate <axis>、或 aim 状态机名。
            cv2.putText(canvas, status, (8, self.render_height - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

    def close(self):
        """先停显示线程（等它把当前一次 show 做完）再关 Display —— 顺序反了会并发操作显示。"""
        if self.async_show:
            with self._cond:
                self._stop = True
                self._pending = None
                self._cond.notify()
            if self.thread is not None:
                self.thread.join(timeout=2.0)
        if self.display is not None:
            try:
                self.display.close()
            except Exception:
                pass


class _Touch:
    """触摸屏输入：把点击映射到 `_Preview` 的按钮（**按下沿**触发，防连点重复启动）。

    `maix.touchscreen.TouchScreen().read()` 立即返回 `(x, y, pressed)`（MaixPy v4）。
    无触摸屏 / 未启用 / 无显示时自动禁用，只打日志。
    坐标口径：read() 返回的是**屏幕坐标**，与按钮注册坐标一致（画布=屏幕尺寸的前提）。
    """

    def __init__(self, cfg):
        self.ts = None
        self._down = False                 # 上一帧按下状态，用于识别"按下沿"
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
            if not self._down:             # 只有"上一帧未按、这一帧按下"才算一次点击
                self._down = True
                return preview.hit(int(x), int(y))
            return None                    # 按住不放：不重复返回，避免反复触发
        self._down = False                 # 松开，允许下一次按下沿
        return None


def _to_gray(img, image_module):
    """MaixPy 相机帧 → cv2 灰度 ndarray（H×W，uint8）。

    优先走 `image2cv(..., copy=False)` **零拷贝**借用相机缓冲（不额外占内存、最快），
    因此返回的数组在下一次 cam.read() 前必须用完；跨线程传递前务必先 copy（见 _LatestVision.submit）。
    按通道数兜底：单通道直接取、3 通道按 RGB2GRAY、4 通道按 RGBA2GRAY；
    都不能识别时退回 copy=True 的 BGR 再转灰度。
    """
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
    """旧版纯视觉 ASCII 输出（ENABLE_GIMBAL=False 的降级路径才用；开云台时 uart 为 None）。"""
    if uart is None:
        return
    if result.get("valid"):
        payload = "@[%d,%d]%%\r\n" % (int(round(result["u"])), int(round(result["v"])))
    else:
        payload = "@[invalid]%%\r\n"
    uart.write(payload.encode("ascii"))


class _VisionControl:
    """识别 + 电机状态由**识别线程独占**；主循环只提交图像、按钮事件、读结果。

    独占理由：UART 读写不可并发（两条轴各有独立串口，但同一轴的收发必须串行），
    把"识别 → 偏差 → 闭环下发 → 收 ACK"全放进识别线程，天然避免与主线程抢串口。
    主线程要触发自检只置 `request_test` 事件，由识别线程在自己的上下文里安全地启动。
    """

    def __init__(self, cfg, link, aim, calibrator, uart):
        self.cfg, self.link, self.aim = cfg, link, aim
        self.calibrator, self.uart = calibrator, uart
        self.self_test = None
        self.request_test = threading.Event()   # 主线程置位、识别线程消费（按钮 → 自检）
        self.detector = RectangleCenterDetector(cfg)
        self.laser = LaserSpotDetector(cfg) if cfg.ENABLE_LASER else None
        self.laser_static = self.laser is not None and self.laser.mode == "fixed"
        # fixed 模式：光斑位置是常量，开机取一次即可，之后每帧只做减法（零检测开销）。
        self.laser_state = self.laser.static_state() if self.laser_static else None
        self.last_capture = -1e9
        self.stale_stopped = True

    def process(self, gray, captured_at):
        """识别线程的单帧主流程：识别 → 光斑 → 偏差 → 串口 → 闭环 → 打包给显示层。"""
        started = time.perf_counter()
        result = self.detector.process(gray)                # ① 黑框识别
        detected = time.perf_counter()
        if self.laser is not None and not self.laser_static:
            self.laser_state = self.laser.process(gray, detected)   # ② detect 模式才实测光斑
        dv = deviation(self.laser_state, result, use_raw=self.cfg.AIM_USE_RAW) if self.laser_state else None
        if dv is not None:
            result["laser_u"], result["laser_v"] = self.laser_state["u"], self.laser_state["v"]
            result["dx"], result["dy"] = dv                 # dx=靶心−光斑（闭环用的那个偏差）
        now = time.perf_counter()
        fresh = now - captured_at <= self.cfg.RESULT_MAX_AGE_S   # 结果是否还"新鲜"
        result["aim_valid"] = bool(dv is not None and fresh)
        self.last_capture = captured_at
        # 过期结果不外发：串口与闭环都用降级后的 output，避免拿旧定位驱动电机。
        output = result if fresh else dict(valid=False, aim_valid=False, stage="stale")
        _send_uart(self.uart, output)
        self._control(output, now, new_measurement=True)
        # 返回给显示层的快照：识别结果 + 光斑 + 计时 + 状态串。
        return dict(result=result, laser=dict(self.laser_state) if self.laser_state else None,
                    captured_at=captured_at, completed_at=time.perf_counter(),
                    detect_ms=(detected - started) * 1000, status=self.status())

    def idle(self):
        # 摄像头/图传停顿时继续收 ACK 与推进按钮自检，禁止重复发送旧视觉误差。
        self._control(None, time.perf_counter(), new_measurement=False)

    def status(self):
        """给屏幕左下角显示的状态串：自检 > 标定 > 闭环状态机的优先级。"""
        if self.self_test is not None and not self.self_test.done:
            return "self-test %d/%d" % (self.self_test.index, len(self.self_test.sequence))
        if self.calibrator is not None and not self.calibrator.done:
            return "calibrate %s" % self.calibrator.axis
        return self.aim.state if self.aim is not None else "vision"

    def _control(self, result, now, new_measurement):
        """闭环调度中枢：故障检查 → 按钮自检 → 标定 → 自检 → 失锁停机 → 追踪下发。

        优先级顺序**不可随意调换**（见 B 轨 AGENTS.md §3.2）；尤其 FAULT 必须在最前，
        因为链路故障要阻断的不只是追踪，还包括已启动的自检/标定。
        """
        link, aim = self.link, self.aim
        if link is None or aim is None:
            return
        feedback = link.poll(now)                            # 非阻塞收两条串口的返回帧
        if feedback and bool(feedback.get("fault")) and aim.state != "FAULT":
            reason = feedback.get("reason")
            link.stop(now=now)
            aim.fault("link:%s" % reason)                    # 链路故障 → 进入 FAULT
        if self.request_test.is_set():                       # 屏幕按钮置位的自检请求
            self.request_test.clear()
            if self.self_test is not None and not self.self_test.done:
                print("[ui] 自检进行中，忽略重复点击")
            elif aim.state == "FAULT":
                print("[ui] 链路 FAULT，拒绝自检")
            elif self.calibrator is not None and not self.calibrator.done:
                print("[ui] 标定进行中，拒绝自检")
            elif not self.cfg.GIMBAL_SELF_TEST:
                print("[ui] GIMBAL_SELF_TEST=False，忽略自检")
            else:
                link.stop(now=now)                           # 先停当前动作再起自检
                self.self_test = GimbalSelfTest(self.cfg, link, now)
                print("[ui] 收到点击 -> 启动上电自检（%d 步）" % len(self.self_test.sequence))
        # 链路故障必须阻断已启动的自检/标定，不只阻断普通追踪。
        if aim.state == "FAULT":
            return
        if self.calibrator is not None and not self.calibrator.done:
            self.calibrator.update(now, result or {"valid": False, "aim_valid": False})
            return
        if self.self_test is not None and not self.self_test.done:
            self.self_test.update(now)
            return
        if not new_measurement:
            # idle 路径：没有新视觉误差。若已超时且尚未停机，则停一次并置 aim 无效（防"漂"）。
            if now - self.last_capture > self.cfg.RESULT_MAX_AGE_S and not self.stale_stopped:
                link.stop(now=now)
                aim.update({"aim_valid": False}, now=now)
                self.stale_stopped = True
            return
        if not result.get("aim_valid"):
            if not self.stale_stopped:                       # 视觉失效：停一次电机，不回零
                link.stop(now=now)
            self.stale_stopped = True
        else:
            self.stale_stopped = False
        commands = aim.update(result, now=now)["cmds"]       # 状态机算出的两轴增量指令
        for axis, command in commands.items():
            if command is not None:
                ccw, pulses, rpm = command
                link.move(axis, ccw, pulses, rpm, self.cfg.ACC_DEFAULT, now=now)


def _display_result(snapshot, now, max_age):
    """把"识别线程的最新快照"翻译成"这一帧显示该画什么"。

    过期定位不能被画成当前帧的有效定位（否则屏幕上框会"粘"在旧位置，误以为还锁着）；
    但视频原帧继续刷新，画面不因此冻住。返回 (result, laser, age)：
        age > max_age → 用 {valid:False, stage:"stale"} 占位，只保留 laser（面板照常显示）
    """
    if snapshot is None:
        return {"valid": False, "stage": "waiting"}, None, 0.0
    age = max(0.0, now - snapshot["captured_at"])
    if age > max_age:
        return {"valid": False, "stage": "stale"}, snapshot["laser"], age
    return snapshot["result"], snapshot["laser"], age


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
    # 按钮几何按**屏幕坐标**定义：右上角一块 30%宽/20%高 的区域（有下限，保证可点）。
    # 只有"有显示 且 启用触摸按钮"时才注册按钮并建触摸；否则各自打印降级原因。
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

    # ---- 线程装配：识别线程（时间轴 B）+ 主循环（时间轴 A）----
    controller = _VisionControl(Config, link, aim, calibrator, uart)
    worker = _LatestVision(controller.process, controller.idle, Config.VISION_ASYNC)
    capture_rate = _FrameRate()             # 取帧帧率（capture 口径）
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
            gray = _to_gray(img, image)                  # ② 零拷贝转灰度 ndarray
            capture_rate.record(captured)
            if touch is not None and touch.poll(preview) == "selftest":
                controller.request_test.set()            # ③ 按键沿 → 通知识别线程起自检
            worker.submit(gray, captured)                # ④ 提交当帧副本给识别线程
            now = time.perf_counter()
            # ⑤ 取最新识别结果（每轮只取一次：日志窗口内不再重复取，省一次加锁 + 帧率统计）
            snapshot, vision_fps, vision_low1 = worker.snapshot(now)
            if worker.error is not None:
                raise RuntimeError("vision worker failed: " + worker.error)   # 识别线程挂了 → 终止
            result, laser_state, age = _display_result(snapshot, now, Config.RESULT_MAX_AGE_S)  # ⑥ 过期降级
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
                print("[perf] %s capture=%.1f disp=%.1f low1=%.1f detect_fps=%.1f "
                      "valid=%s stage=%s | read=%.1f draw=%.1f loop=%.1f ms | "
                      "show avg=%.1f p95=%.1f max=%.1fms n=%d | "
                      "age=%.0fms shown=%d drop(show)=%d drop(vision)=%d" %
                      (BUILD_ID, capture_fps, display_fps, display_low1, vision_fps,
                       result.get("valid", False), result.get("stage"),
                       read_avg, draw_avg, loop_avg,
                       show_avg, show_p95, show_max, show_n,
                       age * 1000, preview.shown, preview.dropped, worker.dropped))
                if aim is not None:
                    print(aim.status_line())
                if show_p95 > 100:
                    # show 的 p95 超过 100ms ⇒ 瓶颈在**显示侧**（屏幕驱动 / MaixVision 图传），
                    # 不是识别慢。预览会掉帧，但识别与闭环不受影响 —— 判据与对策见 README。
                    print("[preview] show slow: p95=%.0fms；预览掉帧，识别/闭环不受影响"
                          "（更省可降 SHOW_FPS 或 SHOW=False）" % show_p95)
                if loop_avg > 66.0:
                    # 主循环单轮超过 66ms ⇒ 已在 15fps 以下，且此数**不含 show**：
                    # 说明卡在取帧/提交侧，看 read= 是否偏大（相机请求帧率或驱动）。
                    print("[preview] main loop slow: loop=%.0fms（不含 show）；查 read= 是否偏大"
                          % loop_avg)
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
