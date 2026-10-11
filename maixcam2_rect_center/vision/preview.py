"""显示层：标注画面 →（letterbox）画布 → display.show()，附屏幕可点击按钮（从 main.py 搬迁，v0.4.6）。

时间轴 C 的实现（见 main.py 模块开头总纲）：
    · Preview      —— 显示线程 + 画布池；主循环 render() 投递即返回，show() 慢也只慢预览。
    · TouchButtons —— 触摸按下沿 → 按钮命中。触摸坐标是屏幕坐标，与画布 1:1。
    · display_result()            —— 把识别线程的最新快照翻译成"这一帧该画什么"（过期降级）。
    · apply_maixvision_stream_policy() —— 建 Display 前掐断 MaixVision 图传（拆 500ms 忙等）。

★ clock 注入：构造可传 `clock`（默认标准库 time）。入口 main() 把自身 time 传进来，
  使 tools/test_main_self_test.py 的假时钟可穿透（同步显示路径在模拟时间轴下可复现）。
★ 隔离原则：显示层任何失败一律只打日志并置 Preview.enabled=False，绝不影响识别/串口/闭环。
"""
import os
import threading
import time

import cv2
import numpy as np

from .rate_meter import FrameRate


def display_result(snapshot, now, max_age):
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


def apply_maixvision_stream_policy(cfg):
    """按配置决定是否禁用 MaixVision 图传；**必须在创建 display.Display() 之前**调用。

    为什么是环境变量而不是别的：MaixCDK 的 `maixvision_mode()` 只读环境变量 MAIXVISION
    （== "1" 才算真），而图传通道 `img_trans` 只在 `Display` 构造时按它创建
    （`maix_display.cpp`：`if(!img_trans && maixvision_mode()) img_trans = new ImageTrans(...)`）。
    一旦建了 img_trans，`show()` 与 `send_to_maixvision()` 都会走 `ImageTrans::send_image()`
    —— 在「MaixVision 模式但连接未就绪」时忙等 500ms 空转 CPU（`maix_image_trans.cpp`）。
    所以想在代码层拆掉它，只有一条路：**在建 Display 之前让 maixvision_mode() 为假**。

    ⚠ 两个易踩点：
      · 必须早于 `display.Display()`；建好之后再改环境变量无效（img_trans 已存在）。
      · `send_to_maixvision()` 并不能绕开忙等 —— 它与 show() 共用同一个 send_image()。
    返回是否实际改动了环境变量。
    """
    if not bool(getattr(cfg, "DISABLE_MAIXVISION_STREAM", True)):
        return False
    if os.environ.get("MAIXVISION") != "1":
        return False
    os.environ["MAIXVISION"] = "0"      # os.environ 赋值会同步 putenv，C 侧 getenv 随即生效
    print("[preview] 已禁用 MaixVision 图传（MAIXVISION 1->0）：本地屏恢复 1~5ms ⇒ 全帧；"
          "MaixVision 画面预览将无图（终端日志照常）。"
          "如需图传：Config.DISABLE_MAIXVISION_STREAM=False，并确保预览已真正连上。")
    return True


class Preview:
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

    def __init__(self, cfg, clock=None):
        self.cfg = cfg
        self._clock = clock if clock is not None else time
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
        self.rate = FrameRate()                 # 显示帧率（disp 口径，与识别帧率分开）
        self._period = 1.0 / max(1, int(getattr(cfg, "SHOW_FPS", 15)))   # 显示节拍周期
        self._t_show = -1e9                     # 下一次该画的时刻（固定时间轴，见 due/render）
        self._canvas = None                     # 同步模式的单张画布（异步模式走 _free 池）
        self._show_times = []                   # 本日志窗口内的 show() 耗时样本（ms）
        self._show_max = 0.0
        self.probe_ms = -1.0                    # 开机 show() 探针耗时（判定显示链路是否健康）
        self._slow_warned = False               # "显示侧慢"告警只打一次，避免每 2s 刷屏
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
            print("[preview] mode=%s screen=%dx%d canvas=%dx%d show_fps=%d jpeg=%d maixvision=%s"
                  % ("display-thread" if self.async_show else "sync-inprocess",
                     self.width, self.height, self.render_width, self.render_height,
                     int(getattr(cfg, "SHOW_FPS", 30)), int(getattr(cfg, "SHOW_JPEG_QUALITY", 70)),
                     "on" if os.environ.get("MAIXVISION") == "1" else "off"))
            # ★ 开机探针：立刻量一次 show() 真实耗时。本地屏典型 1~5ms；若命中 MaixCDK 的
            #   "MaixVision 图传未就绪"分支，send_image() 会在 C++ 里**忙等 500ms 并空转 CPU**
            #   （见 MaixCDK components/vision/src/maix_image_trans.cpp），表现为 show 恒定 ~505ms、
            #   并连带把相机/识别/闭环一起拖慢。这条必须当场说清，不能只报一个 p95 数字。
            self.probe_ms = self._probe_show()
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

    def _probe_show(self):
        """开机量一次 show() 耗时，判定显示链路是否健康；返回毫秒（探针失败返回 -1）。

        判据为何取 300ms：本地屏幕刷新官方数据是 **1~5ms**；而 MaixCDK 的
        `ImageTrans::send_image()` 在「处于 MaixVision 模式、但图传连接未就绪（handle->init 为假）」
        时，会 `while(!init) { if(ticks - t > 500) return ERR_NOT_READY; }` ——
        **忙等 500ms 且空转 CPU**。这正是本项目实测
        `show avg/p95/max=505.8/508.4/509.1ms`、`disp≈1.9fps` 且相机与识别一起变慢的根因：
        500ms × 约 2 次/秒 ≈ **烧掉半个 CPU 核**。
        """
        try:
            from maix import image as maix_image
            frame = np.zeros((self.render_height, self.render_width, 3), np.uint8)
            t0 = self._clock.perf_counter()
            self.display.show(maix_image.cv2image(frame, bgr=True, copy=True))
            ms = (self._clock.perf_counter() - t0) * 1000.0
        except Exception as exc:
            print("[preview] show 探针失败（显示可能不可用）: %s" % exc)
            return -1.0
        if ms >= 300.0:
            print("[preview] ⚠ show 探针 %.0fms —— 命中 MaixCDK「MaixVision 图传未就绪」忙等分支："
                  "每次 show() 空转 500ms（maix_image_trans.cpp），约每秒烧掉半个 CPU 核，"
                  "是相机 read / 识别 / 闭环一起变慢的根因。" % ms)
            print("[preview] ⚠ 对策①：把 Config.DISABLE_MAIXVISION_STREAM 设回 True（默认即 True）——"
                  "main() 会在建 Display 之前把 MAIXVISION 置 0，从源头不建图传，show() 回到 1~5ms。")
            print("[preview] ⚠ 对策②：或改从**设备本地**启动本应用（应用菜单 / 开机自启），"
                  "不要用 MaixVision 的“运行”按钮 —— 本地启动时 maixvision 模式本就关闭。")
            print("[preview] ⚠ 对策③：或让 MaixVision 的画面预览真正连上"
                  "（连上后图传走「压 JPEG 入队」的异步路径，不再忙等）。")
        else:
            print("[preview] show 探针 %.1fms —— 显示链路健康（本地屏典型 1~5ms）" % ms)
        return ms

    def warn_slow_once(self, p95):
        """显示侧变慢只提示一次（每 2s 窗口重复报会刷屏）；点明最可能的根因。"""
        if self._slow_warned:
            return
        self._slow_warned = True
        print("[preview] show slow: p95=%.0fms —— 瓶颈在显示侧（非识别）。"
              "若该值恒定 ≈505ms，即为 MaixCDK 的 MaixVision 图传忙等："
              "保持 Config.DISABLE_MAIXVISION_STREAM=True（默认），或从设备本地启动应用即可消除。"
              "预览掉帧，识别/闭环不受影响。" % p95)

    def _do_show(self, canvas, maix_image=None):
        """真正调用 display.show()，记耗时/帧率；失败即停用显示（不影响识别/闭环）。返回是否成功。"""
        if maix_image is None:
            from maix import image as maix_image
        t0 = self._clock.perf_counter()
        try:
            # bgr=True：cv2 是 BGR 序；copy=True：交给显示的必须是独立缓冲。
            self.display.show(maix_image.cv2image(canvas, bgr=True, copy=True))
        except Exception as exc:
            self.enabled = False
            print("[preview] show 失败，停用显示: %s" % exc)
            return False
        ms = (self._clock.perf_counter() - t0) * 1000.0
        self.shown += 1
        completed = self._clock.perf_counter()
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


class TouchButtons:
    """触摸屏输入：把点击映射到 `Preview` 的按钮（**按下沿**触发，防连点重复启动）。

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
