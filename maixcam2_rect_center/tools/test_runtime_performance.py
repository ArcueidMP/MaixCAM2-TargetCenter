"""运行时回归：慢显示隔离、搜索工作量有界、显式非阻塞 UART、与 v15 的同机性能比较。

PC 耗时只用于同一进程的新旧比较，不能换算成板端 FPS。
"""
import json
import subprocess
import sys
import time
import types
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from config import Config
from main import AsyncPreview, SyncPreview
from vision.gimbal_aim import GimbalAim
from vision.rectangle_center import RectangleCenterDetector
from vision.stepper_link import StepperLink
from test_performance import scene, acquire, benchmark


def _slow_show(view):
    time.sleep(0.5)  # 重现用户日志的 509ms show 阻塞，发生在独立进程内。


def _fast_show(view):
    return None


def _slow200_show(view):
    time.sleep(0.2)


def test_slow_preview_isolation(preview_class=AsyncPreview):
    preview = preview_class(Config, show_callback=_slow_show)
    try:
        assert preview.ready.wait(timeout=15), "预览进程未就绪"
        assert preview.submit(np.zeros((240, 320, 3), np.uint8))
        # 无论后台开始 show 还是仍在接收图像，父进程都不得等它完成。
        t0 = time.perf_counter()
        accepted = sum(preview.submit(np.zeros((1, 1, 3), np.uint8)) for _ in range(100))
        aim = GimbalAim(type("TestCfg", (Config,), dict(DIVERGE_GUARD=False)))
        for i in range(1000):
            aim.update({"aim_valid": True, "dx": 100, "dy": 0}, i / 30)
        elapsed = time.perf_counter() - t0
        assert accepted == 0, "显示忙时不允许排队"
        assert elapsed < 0.4, "显示阻塞了父进程: %.3fs" % elapsed
        assert preview.ready.wait(timeout=5)
        assert preview.elapsed.value >= 450, preview.elapsed.value
        print("PASS: 500ms 慢显示期间主进程完成 1000 次控制更新 (%.1fms)" % (elapsed * 1000))
    finally:
        preview.close()
    assert not preview.process.is_alive()


def test_search_work_is_bounded_and_current():
    det = RectangleCenterDetector(Config)
    noise = np.random.default_rng(6).integers(0, 256, (480, 640), dtype=np.uint8)
    for _ in range(25):
        out = det.process(noise)
        assert out["search_count"] <= 1 and out["thresholds"] <= 1, out
        assert out["proposals"] <= Config.SEARCH_MAX_PROPOSALS, out
        assert not out["valid"]
    # 不得跨帧复用旧图像的识别结果：上一帧留下候选，下一帧变白必须拒绝。
    q = np.array([[180, 70], [445, 80], [475, 400], [150, 420]], np.float32)
    image, truth = scene(q)
    det = RectangleCenterDetector(Config)
    with patch.object(det, "_verify", return_value=None):
        det.process(image)
    out = det.process(np.full_like(image, 175))
    assert not out["valid"], out
    out, frames = acquire(det, image)
    assert out["valid"] and frames < 30
    assert np.hypot(out["raw_u"] - truth[0], out["raw_v"] - truth[1]) < 1.5
    print("PASS: 杂乱背景每帧 <=1 档扫描 / <=2 候选验证；仅接受当前帧证据")


def test_preview_reports_real_fps_without_long_freeze(preview_class=AsyncPreview):
    """预览必须**实测**显示帧率，且慢帧不得把画面冻结成 ~0.4fps。

    这是针对"显示帧率从来没被真正量过"的回归：旧实现的 delay = max(1/fps, 2.0) 会在
    每次慢帧后睡 2 秒，把预览塌到 1/(show+2.0) ≈ 0.4fps。新实现按"目标周期 − 实际耗时"
    节拍、慢帧只加 SHOW_BACKOFF_S 的小退避。
    """
    frame = np.zeros((60, 80, 3), np.uint8)
    fast = type("FastCfg", (Config,), dict(SHOW_FPS=20, SHOW_SLOW_MS=80.0, SHOW_BACKOFF_S=0.10))
    preview = preview_class(fast, show_callback=_fast_show)
    try:
        assert preview.ready.wait(timeout=15)
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 1.0:
            preview.submit(frame)
            time.sleep(1.0 / 60)  # 模拟 60fps 主循环持续投帧
        measured = preview.shown.value / max(1e-6, time.perf_counter() - t0)
        assert preview.shown.value > 0, "一帧都没显示"
        assert measured > fast.SHOW_FPS * 0.5, "显示帧率被节拍压得过低: %.1f" % measured
        print("PASS: 快速通道实测显示 FPS=%.1f（目标 %d）" % (measured, fast.SHOW_FPS))
    finally:
        preview.close()

    def slow_show(view):
        time.sleep(0.2)  # 200ms 慢帧
    slow = type("SlowCfg", (Config,), dict(SHOW_FPS=10, SHOW_SLOW_MS=80.0, SHOW_BACKOFF_S=0.10))
    preview = preview_class(slow, show_callback=_slow200_show)
    try:
        assert preview.ready.wait(timeout=15)
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 1.2:
            preview.submit(frame)
            time.sleep(1.0 / 60)
        measured = preview.shown.value / max(1e-6, time.perf_counter() - t0)
        # 旧实现给 1/(0.2+2.0)=0.45fps；新实现应 >1fps（约 3.3fps）。
        assert measured > 1.0, "慢帧把预览冻结了: %.2f fps（旧实现的 2s 退避复现）" % measured
        assert preview.slow.value > 0 and preview.max_ms.value >= 150
        print("PASS: 慢帧通道实测显示 FPS=%.2f（不再塌到 0.4fps）show_max=%.0fms"
              % (measured, preview.max_ms.value))
    finally:
        preview.close()


def test_sync_preview_is_inprocess_and_adaptive():
    """默认显示路径必须在**主进程内**直接 `display.Display().show()`
    （这是"MaixVision 能看到画面"的充要条件），并且慢帧时自动降速而不是冻结/拖死主循环。

    背景：v16 把显示挪进 spawn 子进程后，子进程一旦起不来 / 建 Display 失败，
    `can_submit()` 恒 False → 主循环一帧都不画 → 预览面板永远"无数据"。本测试锁死
    "默认走同步路径 + Display 建在主进程"。
    """
    import sys
    import types
    calls = []
    state = {"slow": False}
    fake_maix = types.ModuleType("maix")
    fake_display = types.ModuleType("maix.display")
    fake_image = types.ModuleType("maix.image")

    class FakeDisplay:
        def __init__(self):
            calls.append("display")

        def show(self, img):
            calls.append("show")
            if state["slow"]:
                time.sleep(0.2)

        def close(self):
            calls.append("close")

    fake_display.Display = FakeDisplay
    fake_image.cv2image = lambda view, bgr=True, copy=True: view
    fake_maix.display, fake_maix.image = fake_display, fake_image
    names = ("maix", "maix.display", "maix.image")
    saved = {k: sys.modules.get(k) for k in names}
    sys.modules.update({"maix": fake_maix, "maix.display": fake_display, "maix.image": fake_image})
    frame = np.zeros((40, 60, 3), np.uint8)
    cfg = type("SyncCfg", (Config,),
               dict(SHOW_FPS=20, SHOW_SLOW_MS=80.0, SHOW_SLOW_MIN_FPS=2.0))
    try:
        preview = SyncPreview(cfg)
        assert calls.count("display") == 1, "Display 必须建在主进程（同步路径）"
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 0.8:
            if preview.can_submit():
                preview.submit(frame)
            time.sleep(1.0 / 120)
        fast_fps = preview.shown / max(1e-6, time.perf_counter() - t0)
        assert preview.shown > 0 and calls.count("show") == preview.shown, "同步路径没有真正 show"
        assert fast_fps > cfg.SHOW_FPS * 0.5, "同步预览帧率被节拍压得过低: %.1f" % fast_fps
        print("PASS: 同步预览在主进程内 show，实测显示 FPS=%.1f（目标 %d）" % (fast_fps, cfg.SHOW_FPS))
        preview.close()

        state["slow"] = True
        preview = SyncPreview(cfg)
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 1.5:
            if preview.can_submit():
                preview.submit(frame)
            time.sleep(1.0 / 120)
        slow_fps = preview.shown / max(1e-6, time.perf_counter() - t0)
        assert slow_fps > 1.0, "慢帧把同步预览冻结了: %.2f fps" % slow_fps
        assert preview.slow > 0 and preview.max_ms >= 150
        print("PASS: 慢帧下同步预览自动降速但仍更新，实测 FPS=%.2f show_max=%.0fms"
              % (slow_fps, preview.max_ms))
        preview.close()
    finally:
        for k in names:
            if saved[k] is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = saved[k]


def test_uart_read_is_nonblocking_and_bounded():
    class Port:
        def __init__(self):
            self.reads = []

        def available(self, timeout):
            assert timeout == 0
            return 10000

        def read(self, count, timeout=1000):
            assert timeout == 0 and count <= 256
            self.reads.append((count, timeout))
            return b'X' * count

    link = StepperLink(Config)
    port = Port()
    link._ports = {"yaw": port}
    for _ in range(10):
        link.poll(1.0)
    assert len(port.reads) == 10 and len(link._rxbuf["yaw"]) < 4
    link.flush()
    assert len(port.reads) == 11
    print("PASS: UART0 日志洪水每帧最多读 256 字节，timeout=0")


def compare_v15():
    def load(rel):
        module = types.ModuleType('baseline_' + Path(rel).stem)
        src = subprocess.check_output(['git', 'show', '21ac7b4:maixcam2_rect_center/' + rel], cwd=ROOT)
        exec(compile(src, '<v15/' + rel + '>', 'exec'), module.__dict__)
        return module

    old_detector = load('vision/rectangle_center.py').RectangleCenterDetector
    im, _ = scene([[180,70], [445,80], [475,400], [150,420]])
    noise = np.random.default_rng(6).integers(0, 256, (480,640), dtype=np.uint8)
    report = {}
    for name, factory in [('v15', old_detector), ('v16', RectangleCenterDetector)]:
        report[name] = dict(track=benchmark(factory, im, count=120))
        det = factory(Config)
        samples = []
        for _ in range(30):
            t0 = time.perf_counter()
            det.process(noise)
            samples.append((time.perf_counter() - t0) * 1000)
        report[name]['clutter_median_ms'] = round(float(np.median(samples)), 3)
        report[name]['clutter_p95_ms'] = round(float(np.percentile(samples, 95)), 3)
    print(json.dumps(report, ensure_ascii=False, indent=2))


def main():
    cv2.setNumThreads(Config.OPENCV_THREADS)
    test_slow_preview_isolation()
    test_sync_preview_is_inprocess_and_adaptive()
    test_preview_reports_real_fps_without_long_freeze()
    test_search_work_is_bounded_and_current()
    test_uart_read_is_nonblocking_and_bounded()
    compare_v15()


if __name__ == '__main__':
    main()
