#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/tune_real.py · 拿一张**真机拍的照片**离线调参（PC 上跑，不需要板子）
================================================================================
"实际锁框很差" 这类问题，光看代码猜不出来，必须看真图。用法：

    python tools/tune_real.py 照片.jpg
    python tools/tune_real.py 照片.jpg --tape 1.8 --save overlay.jpg

它会告诉你：
    ① 这张图的灰度分布 —— 判断是不是**曝光**问题（黑胶带到底有多黑？纸面有多亮？）
    ② 阈值扫描表      —— 每个阈值下的黑占比、候选结构、是否成功锁定
    ③ 最优结果        —— 用哪个阈值、锁到的是"深色环"还是"亮面矩形"、中心在哪
    ④ 叠加图          —— 绿框=锁定的框，红十字=靶心（存成 overlay.jpg 便于肉眼确认）

如果所有阈值都失败，末尾会给出按可能性排序的处置建议。
"""

import os
import sys

import numpy as np
import cv2

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from config import Config                                            # noqa: E402
from vision.detector import (TargetCenterDetector, contour_features,  # noqa: E402
                             select_frame_contour)


def load_gray(path):
    """读图（兼容中文路径，cv2.imread 在 Windows 下对中文路径会失败）"""
    data = np.fromfile(path, dtype=np.uint8)
    img = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if img is None:
        raise SystemExit("[tune] 读不出这个文件: %s" % path)
    return img, cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def main():
    args = sys.argv[1:]
    if not args:
        raise SystemExit(__doc__)
    path = args[0]
    if "--tape" in args:
        Config.TAPE_W_CM = float(args[args.index("--tape") + 1])
        Config.A4_IN_W_CM = Config.A4_W_CM - 2.0 * Config.TAPE_W_CM
        Config.A4_IN_H_CM = Config.A4_H_CM - 2.0 * Config.TAPE_W_CM
    save = args[args.index("--save") + 1] if "--save" in args else "tune_overlay.jpg"

    color, gray = load_gray(path)
    H, W = gray.shape[:2]
    cfg = Config
    cfg.WORK_SCALE = 1.0                 # 调参用全分辨率，先把图本身的问题看清
    cfg.SELECT_MODE = "score"

    print("=" * 92)
    print(" 真机照片调参: %s   %dx%d" % (os.path.basename(path), W, H))
    print("=" * 92)
    pra = np.percentile(gray, [0, 1, 5, 25, 50, 75, 95, 99, 100]).round(0).astype(int)
    print(" 灰度分布  min/p1/p5/p25/p50/p75/p95/p99/max = "
          + "/".join(str(v) for v in pra))
    p1, p99 = int(pra[1]), int(pra[7])
    if p1 > 100:
        print("   ^^ 提示：最暗的 1% 像素都有 %d 灰 —— 画面里几乎没有「深色」的东西。" % p1)
        print("      大概率是【曝光过高/环境光太强】：先压曝光或降到暗环境，再调阈值。")
    elif p99 < 80:
        print("   ^^ 提示：最亮的 1% 像素只有 %d 灰 —— 整幅偏暗（欠曝）。" % p99)
    print("")

    # ---------------- 阈值扫描 ----------------
    print(" 阈值扫描（黑占比 = 被判为「深色」的像素比例；候选 = 通过结构打分的轮廓）")
    print(" %-7s %-9s %-6s %-58s" % ("阈值", "黑占比", "候选数", "最优候选"))
    print("-" * 92)
    ths = sorted(set(list(cfg.THRESH_SWEEP) + [int(cfg.BLACK_THRESH)]))
    best = None
    for th in ths:
        _, bw = cv2.threshold(gray, int(th), 255, cv2.THRESH_BINARY_INV)
        bk = TargetCenterDetector(cfg)
        bw = bk._morph(bw)
        feats = contour_features(bw, gray, cfg, W, H)
        if feats:
            f = feats[0]
            info = "kind=%-5s 分=%.3f fill=%.2f rim=%.2f ring=%.2f 长宽比=%.2f 对比=%.0f"
            info = info % (f["kind"], f["score"], f["fill"], f["rim"], f["ring"],
                           f["aspect"], f["contrast"])
        else:
            info = "-- 无（轮廓都不成形，或形状完全不符）"
        print(" %-7d %-9.3f %-6d %s" % (th, float((bw > 0).mean()), len(feats), info))
        if feats:
            cfg2 = Config
            d = TargetCenterDetector(cfg2)
            d._th_lock = int(th)
            r = d.process(gray)
            if r["valid"] and best is None:
                best = (th, r)

    # ---------------- 结论（按板端实际档位的缩放比再跑一遍） ----------------
    print("-" * 92)
    cfg.WORK_SCALE = Config.profile()[6]          # 板端默认（standard=0.5）
    print(" 按板端档位 WORK_SCALE=%.2f 复算：" % cfg.WORK_SCALE)
    d = TargetCenterDetector(cfg)
    r = d.process(gray)
    if r["valid"]:
        c = r.get("cand") or {}
        print(" 结论：锁定成功   阈值=%s  候选=%s  靶心=(%.1f, %.1f)  %.2f px/cm  %s"
              % (r["th"], c.get("kind"), r["raw_u"], r["raw_v"], r["scale"], r["stage"]))
        print("       建议把 Config.BLACK_THRESH 改成 %s（锁定后就是它，稳态零额外开销）"
              % r["th"])
        q = r["quad"].astype(int)
        cv2.polylines(color, [q.reshape(-1, 1, 2)], True, (0, 255, 0), 3)
        cv2.drawMarker(color, (int(r["raw_u"]), int(r["raw_v"])), (0, 0, 255),
                       cv2.MARKER_CROSS, 40, 3)
    else:
        print(" 结论：**锁不到框**。按可能性从高到低处置：")
        print("   1. 看上面的灰度分布：p1 很高 -> 压曝光/避强光；p99 很低 -> 补光/提增益。")
        print("   2. 把 Config.BLACK_THRESH 设到「扫描表里有候选的那个阈值」附近。")
        print("   3. 深度相机预览不够，把这张照片发我，我按它精确调 SCORE_* 参数。")
    out = os.path.join(os.path.dirname(os.path.abspath(path)), os.path.basename(save))
    ok, buf = cv2.imencode(".jpg", color, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if ok:
        buf.tofile(out)
        print(" [图] 叠加结果: %s" % out)
    print("=" * 92)


if __name__ == "__main__":
    main()
