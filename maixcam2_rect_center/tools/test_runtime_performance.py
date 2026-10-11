"""真实完成帧计数、最新帧线程、过期追踪阻断、慢识别下视频连续性、显示线程不阻塞主循环。"""
import json
import os
import sys
import threading
import time
import types
from pathlib import Path
from unittest.mock import patch
import cv2
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from config import Config
from main import (_FrameRate, _LatestVision, _Preview, _VisionControl, _display_result,
                  _apply_maixvision_stream_policy)
from vision.rectangle_center import RectangleCenterDetector
from vision.gimbal_aim import GimbalAim
from vision.stepper_link import StepperLink
from test_performance import scene, acquire


def test_real_fps():
    rate = _FrameRate()
    for t in range(5): rate.record(t)
    assert rate.total == 5 and rate.values(4) == (1.0, 1.0)
    assert rate.values(8) == (0.0, 0.0)
    rate = _FrameRate()
    for i in range(61): rate.record(i / 30)
    fps, low = rate.values(2)
    assert abs(fps - 30) < .01 and abs(low - 30) < .01


def test_latest_frame_ownership():
    entered, release, done = threading.Event(), threading.Event(), threading.Event()
    seen = []
    def callback(gray, captured):
        if not seen:
            entered.set()
            assert release.wait(2)
        seen.append((int(gray[0, 0]), captured))
        if len(seen) == 2: done.set()
        return dict(result={"valid": False}, laser=None, captured_at=captured)
    worker = _LatestVision(callback)
    try:
        gray = np.zeros((5, 5), np.uint8)
        worker.submit(gray, 1)
        assert entered.wait(2)
        gray[:] = 99
        for i in range(2, 12): worker.submit(np.full_like(gray, i), i)
        release.set()
        assert done.wait(2)
        assert seen == [(0, 1), (11, 11)], seen
        assert worker.dropped == 9
    finally:
        release.set()
        worker.close()
    assert not worker.thread.is_alive()


def test_stale_and_single_control_update():
    class Link:
        def __init__(self): self.moves, self.stops = [], 0
        def poll(self, now): return {"fault": False}
        def move(self, *args, **kwargs): self.moves.append(args)
        def stop(self, **kwargs): self.stops += 1
    link = Link()
    cfg = type("Cfg", (Config,), dict(RESULT_MAX_AGE_S=.2))
    control = _VisionControl(cfg, link, GimbalAim(cfg), None, None)
    valid = {"valid": True, "u": 320, "v": 240, "raw_u": 320, "raw_v": 240,
             "stage": "track", "search_count": 0}
    with patch.object(control.detector, "process", return_value=valid):
        snap = control.process(np.zeros((480, 640), np.uint8), time.perf_counter())
    assert link.moves
    count = len(link.moves)
    for _ in range(20): control.idle()
    assert len(link.moves) == count, "idle 不得重复发送旧测量"
    with patch.object(control.detector, "process", return_value=dict(valid)):
        snap = control.process(np.zeros((480, 640), np.uint8), time.perf_counter() - 1)
    assert len(link.moves) == count and link.stops == 1
    result, _, age = _display_result(snap, time.perf_counter(), .2)
    assert not result["valid"] and result["stage"] == "stale" and age >= 1


def test_bounded_search_and_current_evidence():
    det = RectangleCenterDetector(Config)
    noise = np.random.default_rng(6).integers(0, 256, (480, 640), dtype=np.uint8)
    samples = []
    for _ in range(25):
        t0 = time.perf_counter()
        out = det.process(noise)
        samples.append((time.perf_counter() - t0) * 1000)
        assert out["search_count"] <= 1 and out["thresholds"] <= 1, out
        assert out["proposals"] <= Config.SEARCH_MAX_PROPOSALS and not out["valid"]
    image, truth = scene([[180, 70], [445, 80], [475, 400], [150, 420]])
    det = RectangleCenterDetector(Config)
    with patch.object(det, "_verify", return_value=None): det.process(image)
    assert not det.process(np.full_like(image, 175))["valid"]
    out, frames = acquire(det, image)
    assert out["valid"] and frames < 30
    assert np.hypot(out["raw_u"] - truth[0], out["raw_v"] - truth[1]) < 1.5
    print("bounded_clutter", json.dumps(dict(median_ms=float(np.median(samples)),
                                              p95_ms=float(np.percentile(samples, 95)))))


def test_uart_is_nonblocking_and_bounded():
    class Port:
        def __init__(self): self.reads = 0
        def available(self, timeout):
            assert timeout == 0
            return 10000
        def read(self, count, timeout=1000):
            assert timeout == 0 and count <= Config.UART_RX_MAX_BYTES
            self.reads += 1
            return b'X' * count
    link = StepperLink(Config)
    port = Port()
    link._ports = {"yaw": port}
    for _ in range(10): link.poll(1.0)
    link._flush_rx("yaw")
    assert port.reads == 11 and len(link._rxbuf["yaw"]) <= Config.UART_RX_BUFFER_MAX_BYTES


def test_slow_vision_keeps_live_video():
    shows, qualities = [], []
    class Display:
        def width(self): return 640
        def height(self): return 480
        def close(self): pass
        def show(self, image):
            time.sleep(.002)
            shows.append((time.perf_counter(), int(image[100, 100, 0])))
    maix = types.ModuleType("maix")
    maix.display = types.SimpleNamespace(Display=Display, set_trans_image_quality=qualities.append)
    maix.image = types.SimpleNamespace(cv2image=lambda a, **kwargs: a)
    def detect(gray, captured):
        time.sleep(.12)
        return dict(result={"valid": False}, captured_at=captured, laser=None)
    worker = _LatestVision(detect)
    with patch.dict(sys.modules, maix=maix):
        # 本用例量的是**同步**显示路径的节拍（"慢识别下视频仍连续、且如实报帧率"），
        # 所以显式钉住 SHOW_ASYNC=False；异步显示线程的契约由下面单独用例覆盖。
        preview = _Preview(type("Cfg", (Config,), dict(SHOW_ASYNC=False)))
        # v0.4.4 起 _Preview 会在构造时做一次 show() 探针（量显示链路耗时）；那一帧不是
        # 本用例要考察的"渲染帧"，且其灰度值为 0 会与首个渲染帧撞车，故先丢弃。
        shows.clear()
        preview.add_button("selftest", 440, 8, 632, 104)
        assert preview.hit(500, 50) == "selftest"
        started = time.perf_counter()
        deadlines = started
        try:
            for i in range(75):
                deadlines += 1 / 30
                now = time.perf_counter()
                gray = np.full((480, 640), i, np.uint8)
                worker.submit(gray, now)
                if preview.due(now): preview.render(np.full((360, 480, 3), i, np.uint8), now)
                time.sleep(max(0, deadlines - time.perf_counter()))
            measured = len(shows) / (time.perf_counter() - started)
            reported, low1 = preview.rate.values(time.perf_counter())
            assert 20 <= measured <= 31, measured
            assert abs(reported - measured) < 3, (reported, measured)
            assert len(set(value for _, value in shows)) == len(shows), "不能用重复画面凑 FPS"
            assert worker.dropped > 0 and qualities == [70]
            print("slow_vision_video", json.dumps(dict(actual_fps=measured, reported_fps=reported,
                                                      low1=low1, unique_frames=len(shows),
                                                      dropped_detection_inputs=worker.dropped)))
        finally:
            worker.close()
            preview.close()


def test_slow_show_does_not_block_main_loop():
    """显示线程化（v0.4.3）：show() 卡 300ms 也不得阻塞主循环；来不及画的帧被**丢弃**而非排队。

    这正是 "只有 1.5 帧" 那个坑的回归锁：以前 show() 在主循环里同步调用，图传/屏幕驱动一慢，
    主循环、submit 节奏、识别与闭环**一起**被拖死。本用例把 show() 故意做慢，断言：
      ① render() 本身耗时极小（不等 show）；② 有丢帧（说明是丢帧策略而非积压）；③ 显示线程确实在跑。
    """
    shows = []
    class Display:
        def width(self): return 640
        def height(self): return 480
        def close(self): pass
        def show(self, image):
            time.sleep(.3)
            shows.append(time.perf_counter())
    maix = types.ModuleType("maix")
    maix.display = types.SimpleNamespace(Display=Display, set_trans_image_quality=lambda q: None)
    maix.image = types.SimpleNamespace(cv2image=lambda a, **kwargs: a)
    cfg = type("Cfg", (Config,), dict(SHOW_ASYNC=True))
    view = np.zeros((480, 640, 3), np.uint8)                 # 模拟 draw_result 的输出
    with patch.dict(sys.modules, maix=maix):
        preview = _Preview(cfg)
        assert preview.async_show and preview.thread is not None, "未启用显示线程"
        worst, renders = 0.0, 0
        started = time.perf_counter()
        try:
            while time.perf_counter() - started < 1.0:
                now = time.perf_counter()
                if preview.due(now):
                    t0 = time.perf_counter()
                    preview.render(view, now)
                    worst = max(worst, time.perf_counter() - t0)
                    renders += 1
                time.sleep(.002)                             # 主循环本应跑满相机帧率
            elapsed = time.perf_counter() - started
        finally:
            preview.close()
    assert renders > 0, "显示线程模式下主循环一次都没画"
    assert worst < .05, "render() 阻塞主循环 %.1fms —— show 仍在主线程里" % (worst * 1000)
    assert preview.dropped > 0, "慢 show 下应丢帧（不排队积压）"
    assert len(shows) >= 2, "显示线程没在实际上屏: %s" % shows
    print("async_show", json.dumps(dict(renders=renders, worst_render_ms=worst * 1000,
                                        shown=len(shows), dropped=preview.dropped,
                                        elapsed_s=elapsed)))


def test_maixvision_stream_policy():
    """MaixVision 图传策略：默认必须把 MAIXVISION 置 0（从源头不建图传、拆掉 500ms 忙等）。

    契约与 MaixCDK 对齐：maixvision_mode() 只在 MAIXVISION=="1" 时为真；图传通道在建 Display
    时按它创建。故这里锁死三种情形：默认置 0、显式关闭策略则保持 1、非 MaixVision 模式不动。
    """
    old = os.environ.get("MAIXVISION")
    try:
        os.environ["MAIXVISION"] = "1"
        assert _apply_maixvision_stream_policy(Config) is True       # 默认 DISABLE_MAIXVISION_STREAM=True
        assert os.environ.get("MAIXVISION") == "0", "默认策略必须掐断图传"

        os.environ["MAIXVISION"] = "1"
        keep = type("Cfg", (Config,), dict(DISABLE_MAIXVISION_STREAM=False))
        assert _apply_maixvision_stream_policy(keep) is False        # 显式保留图传
        assert os.environ.get("MAIXVISION") == "1", "关闭策略时不得改动环境变量"

        os.environ.pop("MAIXVISION", None)
        assert _apply_maixvision_stream_policy(Config) is False      # 非 MaixVision 模式：无副作用
        assert "MAIXVISION" not in os.environ
    finally:
        if old is None:
            os.environ.pop("MAIXVISION", None)
        else:
            os.environ["MAIXVISION"] = old


def main():
    cv2.setNumThreads(Config.OPENCV_THREADS)
    tests = [test_real_fps, test_latest_frame_ownership, test_stale_and_single_control_update,
             test_bounded_search_and_current_evidence, test_uart_is_nonblocking_and_bounded,
             test_slow_vision_keeps_live_video, test_slow_show_does_not_block_main_loop,
             test_maixvision_stream_policy]
    for test in tests:
        test()
        print("PASS", test.__name__)
    print("runtime performance PASS", len(tests))


if __name__ == '__main__': main()
