"""对外 API 回归：`python -B tools/test_target_api.py`。

纯 PC。合成靶纸图（真值来自独立 OpenCV 投影，不用待测实现生成）验证：
  · 识别黑框 API 的精度与拒绝行为
  · 追踪 API 的收敛 / 复用内部 track 路径 / 失锁重捕获 / reset
  · 输入兼容（2D、(H,W,1)、三通道、None）
  · 光斑偏差 dx/dy 的符号与接线
"""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from target_api import (Result, load_config, to_gray, detect_black_frame,
                        CenterTracker, track_center, reset, AimLoop)

FAILS = []


def check(cond, label, detail=""):
    if cond:
        print("  OK   %s%s" % (label, ("  " + detail) if detail else ""))
    else:
        FAILS.append(label)
        print("  FAIL %s%s" % (label, ("  " + detail) if detail else ""))


def scene(q, paper=220, tape=20, background=175):
    h = cv2.getPerspectiveTransform(np.float32([[0, 0], [210, 0], [210, 297], [0, 297]]), np.float32(q))
    target = np.full((298, 211), tape, np.uint8)
    target[18:280, 18:193] = paper
    out = cv2.warpPerspective(target, h, (640, 480), borderValue=background)
    truth = cv2.perspectiveTransform(np.float32([[[105, 148.5]]]), h).reshape(2)
    return out, truth


BASE_Q = np.array([[180, 70], [445, 80], [475, 400], [150, 420]], np.float32)


def test_to_gray():
    img = np.full((40, 60), 128, np.uint8)
    check(to_gray(img).shape == (40, 60), "to_gray 接受 2D 灰度")
    check(to_gray(img[:, :, None]).shape == (40, 60), "to_gray 去掉 (H,W,1) 哑轴")
    rgb = np.dstack([img, img, img])
    check(to_gray(rgb).shape == (40, 60), "to_gray 接受三通道")
    check(to_gray(None) is None, "to_gray(None) -> None")
    check(to_gray(np.zeros((10, 10), np.float32)).dtype == np.uint8, "to_gray 非 uint8 自动转换")


def test_detect_api():
    image, truth = scene(BASE_Q)
    res = detect_black_frame(image)
    check(isinstance(res, Result) and res.valid, "detect_black_frame 识别到黑框", res.stage)
    err = float(np.hypot(res.raw_u - truth[0], res.raw_v - truth[1]))
    check(err < 1.5, "识别中心精度 < 1.5px", "err=%.3fpx" % err)
    check(res.center == (res.u, res.v), "center 与 u/v 一致")
    check(np.asarray(res.quad).shape == (4, 2), "quad 形状 (4,2)")
    check(bool(res) is True and res.image_size == (640, 480), "Result 真值/尺寸字段")
    check(abs(res["u"] - res.u) < 1e-9, "res['u'] 与 res.u 等价")
    # 无状态：同一实例连调两次都应是全图搜索，不受上次锁定影响
    first = detect_black_frame(image)
    check(first.stage == "full_search", "无状态识别每帧全图搜索", first.stage)


def test_detect_rejects_blank():
    blank = np.full((480, 640), 150, np.uint8)
    res = detect_black_frame(blank)
    check((not res.valid) and bool(res) is False and res.center is None,
          "空白画面正确拒绝", res.stage)
    dark_box = np.full((480, 640), 175, np.uint8)
    cv2.rectangle(dark_box, (150, 70), (460, 400), 20, -1)
    check(not detect_black_frame(dark_box).valid, "实心黑块不误检")


def test_track_api():
    tr = CenterTracker()
    image, _ = scene(BASE_Q)
    for _ in range(3):
        res = tr.track(image)
    check(res.valid, "追踪 API 锁定")
    # 小幅平移 -> 走内部 track 快速路径
    moved, truth = scene(BASE_Q + [8, 0])
    r2 = tr.track(moved)
    check(r2.valid and r2.stage == "track", "小位移走窄带跟踪", r2.stage)
    check(abs(r2.raw_u - truth[0]) < 1.5, "跟踪中心精度 < 1.5px")
    # 大跳 -> 重捕获
    far, far_truth = scene(BASE_Q + [-40, 30])
    r3 = tr.track(far)
    check(r3.valid, "大跳后重捕获（不丢锁）", r3.stage)
    check(abs(r3.raw_u - far_truth[0]) < 2.0, "重捕获中心精度 < 2px")
    # 失锁
    lost = tr.track(np.full((480, 640), 150, np.uint8))
    check(not lost.valid, "丢帧正确置 invalid")
    # reset 后仍能重新锁定
    tr.reset()
    check(tr.track(image).valid, "reset() 后可重新锁定")


def test_signature_functional():
    image, _ = scene(BASE_Q)
    r1 = track_center(image)
    check(r1.valid, "track_center() 函数式可用")
    r2 = track_center(image)
    check(r2.valid and r2.stage == "track", "默认实例被复用（第二次走 track）", r2.stage)
    reset()
    r3 = track_center(image)
    check(r3.valid, "reset() 后默认实例重建")


def test_laser_deviation():
    image, truth = scene(BASE_Q)
    nominal_u, nominal_v = 294, 202
    # fixed 模式：光斑位置写死，dx/dy 应等于 中心 − 标称
    tr = CenterTracker(with_laser=True)
    res = tr.track(image)
    check(res.aim_valid and res.dx is not None, "with_laser 时 aim_valid/dx 可用")
    check(abs(res.dx - (res.raw_u - nominal_u)) < 1e-6 and
          abs(res.dy - (res.raw_v - nominal_v)) < 1e-6, "dx/dy = 靶心 − 光斑（raw 口径）")
    # detect 模式：把亮斑画在"靶心左上 12/9 像素"处，验证实测与符号
    cfg = load_config(LASER_MODE="detect", LASER_MIN_CONTRAST=20.0, LASER_MIN_PEAK=140.0)
    spot = (int(round(truth[0])) - 12, int(round(truth[1])) + 9)   # 靶心右侧 12px、上方 9px
    cv2.circle(image, spot, 3, 255, -1)
    td = CenterTracker(cfg, with_laser=True)
    td.track(image)                      # 第一次：建立基准
    res2 = td.track(image, now=10.0)     # 越过 LASER_INTERVAL_S，真正实测
    check(res2.laser.get("found") and abs(res2.laser["u"] - spot[0]) < 1.5,
          "detect 模式实测到光斑", "u=%.1f v=%.1f" % (res2.laser["u"], res2.laser["v"]))
    check(abs(res2.dx - 12.0) < 2.5 and abs(res2.dy + 9.0) < 2.5,
          "偏差幅值与符号正确（靶心在光斑右侧、上方）",
          "dx=%+.1f dy=%+.1f" % (res2.dx, res2.dy))


def test_config_override():
    image, _ = scene(BASE_Q)
    strict = load_config(MIN_AREA_RATIO=0.95)     # 面积极端限制 -> 应拒检
    check(not detect_black_frame(image, strict).valid, "load_config 覆盖生效（不改磁盘）")
    check(detect_black_frame(image, load_config()).valid, "默认配置仍可识别")


def test_aimloop_without_motors():
    image, _ = scene(BASE_Q)
    loop = AimLoop(enable_motors=False)
    res = loop.step(image)
    check(res.valid and res.aim_state is None, "AimLoop(无电机) 只出视觉结果")
    check(res.cmds == {"yaw": None, "pitch": None}, "无电机时不产生指令")
    loop.close()


def main():
    tests = [test_to_gray, test_detect_api, test_detect_rejects_blank, test_track_api,
             test_signature_functional, test_laser_deviation, test_config_override,
             test_aimloop_without_motors]
    for fn in tests:
        print("[%s]" % fn.__name__)
        fn()
    print("-" * 60)
    if FAILS:
        print("FAILED %d: %s" % (len(FAILS), FAILS))
        raise SystemExit(1)
    print("target_api 回归：全部通过")


if __name__ == "__main__":
    main()
