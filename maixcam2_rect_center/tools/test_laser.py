"""激光光斑读数回归：python -B tools/test_laser.py

全是合成图，真值由构造给出（不做任何"用待测实现生成预期"的事）。检查项：

  T0 出厂默认是 fixed（光斑写死），detect 路径仍可用
  T1 标称位置上有光斑时，亚像素质心误差 < 0.4px
  T2 光斑偏离标称（仍在 ROI 内）时同样跟得住
  T3 单帧跳变超过 LASER_MAX_JUMP_PX 判为误检，保留上次位置
  T4 纯墙面（无光斑）不报找到
  T5 大片高光（面积超 LASER_MAX_PIXELS）不报找到
  T6 限频：30fps 跑 1 秒只算 3~4 次，其余帧沿用上次结果
  T7 单次调用耗时预算（PC 参考值，不代表板端）
  T8 deviation() 的符号约定：dx = u靶心 − u光斑
  T9 叠加绘制不越界，且确实画了面板
  T10 fixed 模式不做检测、恒给标定位置
"""
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from config import Config
from vision.laser_spot import LaserSpotDetector, PANEL_H, deviation, draw_laser_overlay

W, H = 640, 480
SPOT = (Config.LASER_U_NOMINAL, Config.LASER_V_NOMINAL)
FAILED = []


def check(name, ok, detail=""):
    print("  %-4s %s%s" % ("OK" if ok else "FAIL", name, ("  " + detail) if detail else ""))
    if not ok:
        FAILED.append(name)


def scene(spot=None, base=145.0, peak=180.0, sigma=1.5, glare=None, noise=4.0, seed=7):
    """墙面 + 可选高斯光斑 + 可选大片高光；返回 (uint8 图, 光斑真值 or None)。"""
    rng = np.random.default_rng(seed)
    g = np.full((H, W), float(base))
    g += np.linspace(-12, 12, W)[None, :] * 0.5          # 轻微的墙面渐变
    g += rng.normal(0.0, noise, g.shape)
    truth = None
    if spot is not None:
        cx, cy = spot
        r = int(4 * sigma) + 2
        ys, xs = np.mgrid[cy - r:cy + r + 1, cx - r:cx + r + 1]
        blob = (peak - base) * np.exp(-((xs - cx) ** 2 + (ys - cy) ** 2) / (2.0 * sigma ** 2))
        g[cy - r:cy + r + 1, cx - r:cx + r + 1] += blob
        truth = (float(cx), float(cy))
    if glare is not None:
        x, y, w, h = glare
        g[y:y + h, x:x + w] = 255.0
    return np.clip(g, 0, 255).astype(np.uint8), truth


def fresh(**over):
    cfg = type("C", (), {})()
    for k in dir(Config):
        if k.startswith("LASER_"):
            setattr(cfg, k, getattr(Config, k))
    for k, v in over.items():
        setattr(cfg, k, v)
    return LaserSpotDetector(cfg)


def main():
    print("=" * 72)
    print("激光光斑读数回归  nominal=%s  ROI=±%d  限频=%.2fs" %
          (SPOT, Config.LASER_ROI_RADIUS, Config.LASER_INTERVAL_S))
    print("=" * 72)

    # ---------------- T0 出厂默认 = fixed（写死光斑）
    check("T0 出厂默认 LASER_MODE == 'fixed'（光斑写死、不做实时检测）",
          str(Config.LASER_MODE).lower() == "fixed", "LASER_MODE=%r" % (Config.LASER_MODE,))
    st = LaserSpotDetector(Config).static_state()
    check("T0' static_state() 直接给出标定常量且零耗时",
          st["found"] and st["reason"] == "fixed" and not st["updated"] and st["ms"] == 0.0
          and (st["u"], st["v"]) == SPOT, "u=%.0f v=%.0f ms=%.1f" % (st["u"], st["v"], st["ms"]))

    # ---------------- T1 / T2 检出精度（需要 detect 模式）
    for tag, off in (("T1 标称位置", (0, 0)), ("T2 偏离标称(+30,-25)", (30, -25))):
        img, truth = scene(spot=(SPOT[0] + off[0], SPOT[1] + off[1]))
        det = fresh(LASER_MODE="detect")
        out = det.process(img, 1000.0)
        err = np.hypot(out["u"] - truth[0], out["v"] - truth[1]) if out["found"] else 9e9
        check("%s 检出" % tag, out["found"], "reason=%s peak=%.0f bg=%.0f area=%d" %
              (out["reason"], out["peak"], out["bg"], out["area"]))
        check("%s 质心误差 %.2fpx (<0.4)" % (tag, err), err < 0.4)

    # ---------------- T3 跳变门限
    det = fresh(LASER_MODE="detect", LASER_SMOOTH_ALPHA=0.0)
    det.process(scene(spot=SPOT)[0], 1000.0)
    far = (SPOT[0] + Config.LASER_ROI_RADIUS - 10, SPOT[1] - Config.LASER_ROI_RADIUS + 10)
    out = det.process(scene(spot=far)[0], 1001.0)
    ok = out["found"] and out["reason"] == "jumped" and abs(out["u"] - SPOT[0]) < 0.5
    check("T3 超跳变门限判为 jumped 并保留旧位置", ok,
          "reason=%s u=%.1f v=%.1f" % (out["reason"], out["u"], out["v"]))
    det2 = fresh(LASER_MODE="detect", LASER_SMOOTH_ALPHA=0.0, LASER_MAX_JUMP_PX=0)     # 关闭门限
    det2.process(scene(spot=SPOT)[0], 1000.0)
    out2 = det2.process(scene(spot=far)[0], 1001.0)
    check("T3' 门限=0 时关闭该判据（能跟到大位移）",
          out2["found"] and np.hypot(out2["u"] - far[0], out2["v"] - far[1]) < 0.5)

    # ---------------- T4 无光斑
    out = fresh(LASER_MODE="detect").process(scene()[0], 1000.0)
    check("T4 纯墙面不误报", (not out["found"]) and out["reason"] == "no_peak",
          "reason=%s peak=%.0f bg=%.0f" % (out["reason"], out["peak"], out["bg"]))

    # ---------------- T5 大片高光
    out = fresh(LASER_MODE="detect").process(
        scene(spot=(SPOT[0] - 20, SPOT[1] + 15), glare=(SPOT[0] + 16, SPOT[1] - 40, 46, 46))[0], 1000.0)
    check("T5 大片高光按面积拒绝", (not out["found"]) and out["reason"] == "too_big",
          "reason=%s area=%d" % (out["reason"], out["area"]))

    # ---------------- T6 限频
    img, _ = scene(spot=SPOT)
    det = fresh(LASER_MODE="detect")
    calls, stale_ok, last = 0, True, None
    for i in range(30):                      # 30fps 跑 1 秒
        out = det.process(img, i / 30.0)
        calls += 1 if out["updated"] else 0
        if not out["updated"] and last is not None:
            stale_ok &= (out["u"] == last["u"] and out["v"] == last["v"])
        last = out
    want = int(1.0 / Config.LASER_INTERVAL_S) + 1
    check("T6 限频：30 帧只算 %d 次（期望 %d）" % (calls, want), calls <= want + 1,
          "每秒实际计算 %d 次" % calls)
    check("T6' 未更新的帧沿用上次光斑位置", stale_ok and calls < 30)

    # ---------------- T7 耗时
    det = fresh(LASER_MODE="detect")
    det.process(img, 0.0)
    t0 = time.perf_counter()
    for i in range(100):
        det.process(img, 1000.0 + i)         # 每次都重算
    ms = (time.perf_counter() - t0) * 1000.0 / 100
    check("T7 单次耗时 %.3fms（PC 预算 <1.0ms）" % ms, ms < 1.0)

    # ---------------- T8 符号约定
    laser = dict(found=True, u=100.0, v=200.0)
    dv = deviation(laser, dict(valid=True, u=130.0, v=180.0))
    check("T8 deviation = 靶心-光斑", dv == (30.0, -20.0), "dv=%s" % (dv,))
    check("T8' 无靶纸/丢光斑时返回 None",
          deviation(laser, dict(valid=False)) is None and deviation(None, dict(valid=True)) is None)

    # ---------------- T9 叠加绘制
    canvas = np.zeros((360, 480, 3), np.uint8)
    h = draw_laser_overlay(canvas, dict(found=True, u=SPOT[0], v=SPOT[1], peak=180.0, area=9),
                           dict(valid=True, u=SPOT[0] + 30, v=SPOT[1] - 20), 0.75)
    panel = canvas[0:PANEL_H, 0:200]
    check("T9 面板高度/绘制", h == PANEL_H and panel.sum() > 0,
          "h=%d 面板非零像素=%d" % (h, int((panel.sum(axis=2) > 0).sum())))
    canvas2 = np.zeros((360, 480, 3), np.uint8)
    h2 = draw_laser_overlay(canvas2, dict(found=False, reason="no_peak", peak=150.0, bg=145.0),
                            dict(valid=False), 0.75)
    check("T9' 未找到时不画标记只画面板", h2 == PANEL_H and canvas2[200:, :].sum() == 0)

    # ---------------- T10 fixed 模式（光斑位置写死，只看靶心）
    det = fresh(LASER_MODE="fixed")
    img_none, _ = scene()                       # 纯墙面：detect 模式会报丢光斑
    outs = [det.process(img_none, 1000.0 + i) for i in range(5)]
    ok = all(o["found"] and o["reason"] == "fixed" and o["u"] == SPOT[0] and o["v"] == SPOT[1]
             for o in outs)
    check("T10 fixed 模式不做检测、恒给标定位置", ok and not any(o["updated"] for o in outs),
          "u=%.0f v=%.0f updated=%s" % (outs[0]["u"], outs[0]["v"], any(o["updated"] for o in outs)))
    dv = deviation(outs[0], dict(valid=True, u=300.0, v=180.0))
    check("T10' fixed 模式下偏差照常可算", dv == (6.0, -22.0), "dv=%s" % (dv,))

    print("-" * 72)
    print("结果：%s（%d 项失败）" % ("全部通过" if not FAILED else "有失败", len(FAILED)))
    for name in FAILED:
        print("   失败：%s" % name)
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
