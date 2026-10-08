"""对外 API 用法示例（可直接在 PC 上跑）：`python -B tools/example_api_usage.py`

三种典型"小问"场景各一段，复制改改就能用：
  ① 只想识别黑框取中心          -> detect_black_frame()
  ② 视频流里持续追踪靶心        -> CenterTracker() / track_center()
  ③ 要光斑偏差或自动瞄准        -> CenterTracker(with_laser=True) / AimLoop()

板子上把 `gray = demo_frame()` 换成 `gray = cam.read()` 即可（MaixPy image 会被自动转换）。
"""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from target_api import detect_black_frame, CenterTracker, track_center, reset, load_config


def demo_frame(offset=(0, 0)):
    """合成一张 640x480 靶纸图（PC 演示用；板子上换成 cam.read()）。"""
    q = np.float32([[180, 70], [445, 80], [475, 400], [150, 420]]) + np.float32(offset)
    h = cv2.getPerspectiveTransform(np.float32([[0, 0], [210, 0], [210, 297], [0, 297]]), q)
    target = np.full((298, 211), 20, np.uint8)
    target[18:280, 18:193] = 220
    return cv2.warpPerspective(target, h, (640, 480), borderValue=175)


# ------------------------------------------------------------------ ① 识别黑框
def example_detect():
    print("=== ① 识别黑框（无状态，单帧）===")
    gray = demo_frame()
    res = detect_black_frame(gray)
    if res:                                     # bool(res) 即"识别到了吗"
        print("  靶心 = (%.1f, %.1f)  耗时 %.1fms" % (res.u, res.v, res.ms))
        print("  外框四角 =", np.rint(res.quad).astype(int).tolist())
    else:
        print("  这一帧没有黑框：", res.stage)


# ------------------------------------------------------------------ ② 追踪中心
def example_track():
    print("=== ② 追踪中心（有状态，连续调用）===")
    tracker = CenterTracker()
    for i, dx in enumerate((0, 6, 14, 26)):
        res = tracker.track(demo_frame((dx, 0)))
        print("  第 %d 帧 stage=%-11s 靶心=(%6.1f,%6.1f) valid=%s" %
              (i + 1, res.stage, res.get("u", 0.0), res.get("v", 0.0), res.valid))
    print("  换场景时 tracker.reset() 清掉旧锁定")
    # 函数式一行版（内部复用默认实例）
    reset()
    r = track_center(demo_frame())
    print("  track_center() 一行版 -> (%.1f, %.1f)" % (r.u, r.v))


# ------------------------------------------------------------------ ③ 偏差 / 闭环
def example_aim():
    print("=== ③ 光斑偏差 / 自动瞄准 ===")
    tracker = CenterTracker(with_laser=True)         # fixed 模式：光斑位置是标定量
    res = tracker.track(demo_frame())
    print("  靶心=(%.1f,%.1f) 光斑=(%.1f,%.1f) 偏差 dx=%+.1f dy=%+.1f px  aim_valid=%s" %
          (res.u, res.v, res.laser_u, res.laser_v, res.dx, res.dy, res.aim_valid))
    print("  dx>0 = 靶心在光斑右侧；闭环目标就是把 dx、dy 打到 0")
    # 需要真正驱动电机时（务必先做完开环标定）：
    #   from target_api import AimLoop
    #   loop = AimLoop()              # 打开两条 UART + 使能电机
    #   loop.step(cam.read())         # 每帧调用：追踪 -> 偏差 -> 发脉冲
    #   loop.stop()
    print("  （AimLoop 会驱动电机，示例里不实例化；仅演示如何用 load_config 临时改参）")
    cfg = load_config(MIN_AREA_RATIO=0.05, SEARCH_WIDTH=256)
    print("  load_config(MIN_AREA_RATIO=0.05, SEARCH_WIDTH=256) ->",
          detect_black_frame(demo_frame(), cfg).valid)


if __name__ == "__main__":
    example_detect()
    example_track()
    example_aim()
