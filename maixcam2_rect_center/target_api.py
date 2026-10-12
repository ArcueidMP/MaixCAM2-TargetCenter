"""25Efork 视觉能力对外 API —— 识别黑框 / 追踪靶心中心（B 轨）。

把工程内部的「检测器 + 追踪器 + 光斑偏差 + 云台闭环」收成一层稳定门面，
**不同小问直接 import 就能用**，不必关心内部模块路径与参数。

──────────────────────────────── 两个核心 API ────────────────────────────────

① 识别黑框（**无状态**，每帧独立、全图搜索，适合"取一次框"）::

    from target_api import detect_black_frame

    res = detect_black_frame(gray)      # gray: numpy uint8 或 MaixPy image
    if res:                             # res 为真 = 识别到（见"返回值"）
        print(res.center)               # (u, v)
        print(res.quad)                 # 外框四角 (4,2)

② 追踪中心（**有状态**，窄带跟踪 + 失锁自动重捕获，适合连续视频流）::

    from target_api import CenterTracker

    tracker = CenterTracker()
    while True:
        res = tracker.track(gray)
        if res:
            print(res.u, res.v, res.stage)     # stage: track / roi_search / full_search

函数式快捷用法（内部持有一个默认追踪器，小问里一行搞定）::

    from target_api import track_center, reset
    res = track_center(gray)             # 等价于 CenterTracker().track()，实例复用
    reset()                              # 换场景/重开一题时清状态

──────────────────────────────── 可选：激光偏差与云台 ────────────────────────────────

需要"光斑到靶心的偏差"或"自动瞄准"的小问::

    tracker = CenterTracker(with_laser=True)     # res 额外带 laser_u/v、dx、dy
    res = tracker.track(gray)
    print(res.dx, res.dy)                        # dx = 靶心 − 光斑（正=靶心在光斑右侧）

    from target_api import AimLoop               # 完整闭环（会驱动电机，谨慎）
    loop = AimLoop()                             # 需已完成开环标定，见 README
    loop.step(frame)                             # 内部：追踪 → 偏差 → 发脉冲
    loop.stop()

──────────────────────────────── 返回值 ────────────────────────────────

统一返回 ``Result``（dict 子类，支持 ``res["u"]`` 与 ``res.u`` 两种写法）：

    valid      bool    是否识别/追踪到黑框
    u, v       float   靶心像素坐标（已按 CENTER_ALPHA 平滑，供显示/常规使用）
    raw_u/v    float   未平滑中心（闭环用，见 config.AIM_USE_RAW）
    center     tuple   (u, v)；无效时为 None
    quad       (4,2)   外框四角，顺序已规整（左上起逆时针）
    inner_quad (4,2)   内框四角（胶带内沿）
    stage      str     track / roi_search / full_search / no_black_frame
    ms         float   本帧检测耗时（毫秒）
    laser_u/v  float   光斑位置（仅 with_laser=True 且光斑有效时有意义）
    dx, dy     float   靶心 − 光斑（仅 with_laser=True 时；无效为 None）
    aim_valid  bool    靶心与光斑都有效才为 True

``Result`` 的布尔值 = ``valid``，所以 ``if res:`` 直接就是"识别到了吗"。

──────────────────────────────── 输入 ────────────────────────────────

``frame`` 可以是 numpy uint8 灰度图（``(H,W)`` 或 ``(H,W,1)``）、三/四通道图（按 RGB 处理），
也可以是 MaixPy 的 ``image.Image``（自动 ``image2cv``，需要板上环境）。
**本模块顶层不 import maix**，PC 上用 numpy 即可离线跑通、便于单测。

──────────────────────────────── 参数覆盖 ────────────────────────────────

不想改 ``config.py`` 时用 ``load_config(...)`` 造一份覆盖::

    cfg = load_config(MIN_AREA_RATIO=0.02, SEARCH_WIDTH=240)
    tracker = CenterTracker(cfg)
"""

import os
import sys
import time

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from vision.rectangle_center import RectangleCenterDetector
from vision.laser_spot import LaserSpotDetector, deviation
from vision.aim_control import VisionControl
from vision.gimbal_search import SearchSweep
from vision.imu_heading import HeadingFeedforward
from vision.app_runtime import TrackingApplication

__all__ = ["Result", "load_config", "to_gray", "detect_black_frame",
           "CenterTracker", "track_center", "reset", "AimLoop",
           "TrackingApplication", "SearchSweep", "HeadingFeedforward"]

_default_tracker = None


# --------------------------------------------------------------------- 工具
class Result(dict):
    """识别/追踪结果：``res["u"]`` 与 ``res.u`` 均可；``bool(res)`` 等价于 ``res.valid``。"""

    __slots__ = ()

    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError:
            raise AttributeError("Result has no field %r" % name)

    def __setattr__(self, name, value):
        self[name] = value

    def __bool__(self):
        return bool(self.get("valid"))

    def __repr__(self):
        if not self.get("valid"):
            return "Result(invalid, stage=%s)" % self.get("stage")
        return "Result(u=%.1f, v=%.1f, stage=%s)" % (
            self.get("u", float("nan")), self.get("v", float("nan")), self.get("stage"))


def load_config(**overrides):
    """返回 config.Config（可带覆盖项，不改磁盘上的 config.py）。

    ``load_config(MIN_AREA_RATIO=0.02)`` → 派生一个只改了该字段的配置类。
    """
    from config import Config
    if not overrides:
        return Config
    return type("ApiConfig", (Config,), dict(overrides))


def to_gray(frame, image_module=None):
    """把输入统一成 ``(H, W)`` uint8 灰度图；无法转换返回 None。

    兼容：numpy 数组（含 ``(H,W,1)`` 哑轴）、有 ``__array__`` 的对象、MaixPy ``image.Image``。
    三/四通道按 **RGB/RGBA** 解释（MaixPy 相机默认 RGB）。
    """
    if frame is None:
        return None
    a = frame
    if not isinstance(a, np.ndarray):
        if hasattr(a, "__array__"):
            a = np.asarray(a)
        else:
            if image_module is None:
                from maix import image as image_module
            a = image_module.image2cv(frame, ensure_bgr=False, copy=False)
    a = np.asarray(a)
    if a.size == 0:
        return None
    if a.ndim == 3 and a.shape[2] == 1:
        a = a[:, :, 0]
    elif a.ndim == 3 and a.shape[2] == 3:
        a = cv2.cvtColor(a, cv2.COLOR_RGB2GRAY)
    elif a.ndim == 3 and a.shape[2] == 4:
        a = cv2.cvtColor(a, cv2.COLOR_RGBA2GRAY)
    if a.ndim != 2:
        return None
    if a.dtype != np.uint8:
        a = np.clip(a, 0, 255).astype(np.uint8)
    return a


def _wrap(raw, gray):
    """把检测器的 dict 包成 Result，并补 center / image_size 等便捷字段。"""
    res = Result(raw)
    res["center"] = (res["u"], res["v"]) if res.get("valid") else None
    if gray is not None:
        res["image_size"] = (int(gray.shape[1]), int(gray.shape[0]))
    res.setdefault("dx", None)
    res.setdefault("dy", None)
    res.setdefault("aim_valid", False)
    return res


# ------------------------------------------------------------- API ① 识别黑框
def detect_black_frame(frame, cfg=None, detector=None):
    """**无状态**识别黑框，返回 ``Result``（没识别到则 ``valid=False``，``bool(res)`` 为假）。

    每帧都是一次完整的"缩小图多阈值扫描 → 四边暗带验证 → 亚像素精修"，不依赖上一帧，
    适合"只在某几问里取一次黑框"的用法。想复用实例提速可传 ``detector``（但那样就带状态了，
    连续调用会走窄带跟踪路径）。

    :param frame:    numpy uint8 灰度图或 MaixPy image
    :param cfg:      配置类（默认 config.Config；可用 load_config(...) 覆盖）
    :param detector: 复用已有的 RectangleCenterDetector（可选）
    """
    cfg = cfg if cfg is not None else load_config()
    det = detector if detector is not None else RectangleCenterDetector(cfg)
    gray = to_gray(frame)
    if gray is None:
        return Result(valid=False, stage="empty_frame", center=None,
                      dx=None, dy=None, aim_valid=False, ms=0.0)
    started = time.perf_counter()
    raw = det.process(gray)
    out = _wrap(raw, gray)
    out["ms"] = out.get("detector_ms", (time.perf_counter() - started) * 1000.0)
    return out


# ------------------------------------------------------------- API ② 追踪中心
class CenterTracker:
    """**有状态**中心追踪器：窄带跟踪为主，失锁后 ROI 搜索 → 全图重捕获。

    典型用法：连续 ``track(gray)``；换场景/重新开题时 ``reset()``。
    ``with_laser=True`` 时返回值额外带光斑位置与 ``dx/dy``（靶心 − 光斑）。
    """

    def __init__(self, cfg=None, with_laser=False, use_raw=None):
        self.cfg = cfg if cfg is not None else load_config()
        self.detector = RectangleCenterDetector(self.cfg)
        self.laser = LaserSpotDetector(self.cfg) if with_laser else None
        if use_raw is None:
            use_raw = bool(getattr(self.cfg, "AIM_USE_RAW", True))
        self.use_raw = bool(use_raw)

    def reset(self):
        """清空跟踪锁定状态（换靶、换场景时调用，避免拿旧框去搜）。"""
        self.detector = RectangleCenterDetector(self.cfg)
        if self.laser is not None:
            self.laser = LaserSpotDetector(self.cfg)
        return self

    def track(self, frame, now=None):
        """追踪一帧；now 为单调递增的采图时刻（秒），供丢帧后的运动预测使用。"""
        gray = to_gray(frame)
        if gray is None or gray.size == 0:
            self.detector.reset()
            return Result(valid=False, stage="empty_frame", center=None,
                          dx=None, dy=None, aim_valid=False, ms=0.0)
        raw = self.detector.process(gray, timestamp=now)
        res = _wrap(raw, gray)
        if not res.get("valid"):
            return res
        if self.laser is not None:
            ls = self.laser.process(gray, now)
            res["laser"] = ls
            dv = deviation(ls, res, use_raw=self.use_raw)
            if dv is not None:
                res["laser_u"], res["laser_v"] = ls["u"], ls["v"]
                res["dx"], res["dy"] = dv
            res["aim_valid"] = dv is not None
        return res


def track_center(frame, cfg=None, now=None):
    """函数式追踪：内部复用同一个默认 ``CenterTracker``（首次调用时创建）。"""
    global _default_tracker
    if _default_tracker is None:
        _default_tracker = CenterTracker(cfg)
    return _default_tracker.track(frame, now)


def reset():
    """重置 ``track_center()`` 使用的默认追踪器。"""
    if _default_tracker is not None:
        _default_tracker.reset()


# ------------------------------------------------------- 可选：追踪 + 云台闭环
class AimLoop:
    """追踪 + 光斑偏差 + 双轴云台闭环（会驱动 ZDT 电机）。

    ⚠ 使用前必须完成开环标定（``CALIBRATE_AXIS`` 跑一次填回 config），否则方向/增益
    是占位值，可能正反馈甩头。不需要电机的问请只用 ``detect_black_frame`` /
    ``CenterTracker``，不要实例化本类。

    ``step(frame)`` 返回的 ``Result`` 比追踪器多两个字段：
        aim_state  str   状态机状态（FINE/COARSE/SETTLE/HOLD/FAULT…）
        cmds       dict  {"yaw": (ccw, pulses, rpm) or None, "pitch": ...}
    """

    def __init__(self, cfg=None, enable_motors=True, with_laser=True, clock=None, heading=None):
        self.cfg = cfg if cfg is not None else load_config()
        self._clock = clock if clock is not None else time
        self._stopped = True
        self._controller = None
        self.heading = heading
        self.tracker = CenterTracker(self.cfg, with_laser=with_laser)
        self.link = None
        self.aim = None
        if enable_motors:
            from vision.stepper_link import StepperLink
            from vision.gimbal_aim import GimbalAim
            self.link = StepperLink(self.cfg)
            self.link.enable()
            self.aim = GimbalAim(self.cfg)

    def step(self, frame, now=None):
        """处理一帧；now 为采图时刻，与 clock.perf_counter 使用同一单调时钟。"""
        captured = self._clock.perf_counter() if now is None else float(now)
        res = self.tracker.track(frame, captured)
        now = self._clock.perf_counter()
        res["captured_at"] = captured
        res["age_s"] = max(0.0, now - captured)
        if not 0 <= now - captured <= float(getattr(self.cfg, "RESULT_MAX_AGE_S", 0.20)):
            res["aim_valid"] = False
        res["aim_state"] = None
        res["cmds"] = {"yaw": None, "pitch": None}
        if self.aim is None or self.link is None:
            return res
        controller = self._control()
        controller.accept_result(res, captured, res.get("laser"))
        res["aim_state"] = self.aim.state
        res["cmds"] = dict(controller.last_commands)
        res["control_source"] = controller.control_source
        res["search_state"] = controller.search.state if controller.search is not None else None
        self._stopped = controller.stale_stopped
        return res

    def _control(self):
        if (self._controller is None or self._controller.link is not self.link or
                self._controller.aim is not self.aim):
            self._controller = VisionControl(self.cfg, self.link, self.aim, None, None,
                                             clock=self._clock, heading=self.heading)
            self.heading = self._controller.heading
        return self._controller

    def request_search(self):
        """启动/取消 SEARCH；由 step()/idle() 消费请求，不在按钮线程写串口。"""
        if self.link is None or self.aim is None:
            return False
        self._control().request_search.set()
        return True

    def calibrate_imu_bias(self):
        """请求静止偏置标定；先停机，运行时仍需保持底盘和云台静止。"""
        controller = self._control()
        if controller.heading is None or self.link is None or self.aim is None:
            return False
        controller.request_imu_bias.set()
        return True

    def imu_status(self):
        return self._control().imu_status()

    def idle(self):
        """无新图像时维持 ACK、搜索、IMU 补偿和视觉过期停机。"""
        self._control().idle()

    def stop(self, now=None):
        now = self._clock.perf_counter() if now is None else float(now)
        if self._controller is not None:
            accepted = self._controller.stop(now)
            self._stopped = True if accepted else self._stopped
            return accepted
        if self.link is not None:
            if self.link.stop(now=now) is False:
                if self.aim is not None:
                    self.aim.fault("stop:all")
                return False
        if self.aim is not None:
            self.aim.motion_stopped(now=now)
        self._stopped = True
        return True

    def close(self):
        self.stop()
        if self._controller is not None:
            self._controller.close()
        if self.link is not None:
            close = getattr(self.link, "close", None)
            if close is not None:
                close()
