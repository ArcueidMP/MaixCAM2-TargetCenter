#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/test_hardscene.py · 「真实光照硬场景」鲁棒性测试（PC 上跑，不需要板子）
================================================================================
tools/test_pc.py 用的是"理想光照"合成图（浅色背景、曝光正好、无干扰），所以它一直
全绿；但**真机上锁框很差**。差在哪，必须用能复现真机的场景来量。

本脚本按**反射率模型**生成靶纸图，逐项复现真实会遇到的恶劣条件：

    gray = clip( 照度场(x,y) * 反射率 + 环境杂散光(flare) + 噪声 )

    反射率：黑胶带 0.07 / 纸面 0.85 / 墙面 0.55(亮墙) 或 0.06(暗墙)
    => 只要调「照度」就能自然复现 过曝 / 欠曝 / 对比度不足，而不用硬凑灰度值。

覆盖的硬场景：
    理想(基准)      浅色背景 + 曝光正好
    过曝            黑胶带被照亮到阈值以上 -> 二值图里框"消失"
    欠曝            整体偏暗、纸面也暗
    环境光大        环境杂散光把黑胶带抬到阈值以上（雾蒙蒙/白天强散光）
    暗背景          周围也是深色（深色墙/桌面/机箱）-> 与胶带连成一片
    暗背景+高照度    暗背景也进二值图，且比胶带框更大
    光照不均        一侧明显更暗（单侧光源）
    暗角            镜头渐晕，四角偏暗
    反光            胶带上一个高光斑，把环打断
    干扰物          画面里有一块比靶纸更大的深色矩形
    远端小目标      靶纸只占画面很小一块
    近端大目标      靶纸几乎占满画面

每个场景对比两种 ② 描边策略：
    largest = 旧行为：取面积最大的外轮廓
    score   = 新行为：按"深色细环围住亮面且外沿接近矩形"的结构分挑轮廓

运行：  python tools/test_hardscene.py            # 只跑新策略
        python tools/test_hardscene.py --mode largest
        python tools/test_hardscene.py --ab       # 两种策略对照表（推荐）
输出：  tools/hardscene_montage.jpg（各场景首帧拼图，便于肉眼确认场景是否逼真）
"""

import os
import sys

import numpy as np
import cv2

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from config import Config                                       # noqa: E402
from vision.detector import TargetCenterDetector                # noqa: E402

A4W, A4H = 21.0, 29.7
OUT_DIR = os.path.dirname(os.path.abspath(__file__))
W, H = 1280, 720
S = 40.0                                    # cm 画布的分辨率 px/cm

# 默认靶纸姿态（占屏中等，带轻微透视）
DST_DEF = [[420, 110], [880, 150], [915, 610], [405, 655]]
# 轻量"远端"：约 165x225px = 4.0% 画面。**这个尺寸其实不算远**，2% 的面积门也拦不住它，
# 所以它一直是绿的 —— 真机上"远距离锁不住"根本不是这个量级。
DST_SMALL = [[560, 235], [725, 240], [725, 460], [560, 465]]
# 真·远距离：62x83px = 0.56% 画面（约 3.4m 外）。曾经 MIN_AREA_RATIO=0.02 时，
# 这个尺寸连候选轮廓都产生不出来 -> 任何阈值都"未检出"。加进来专门守这个回归。
DST_FAR = [[600, 300], [662, 302], [662, 385], [600, 383]]
# 极限：40x56px = 0.24% 画面（约 5.2m 外），用来确认"远处会优雅地拒检而不是乱锁"
DST_FAR2 = [[610, 310], [650, 311], [650, 366], [610, 365]]
# 近端大目标：竖版 A4 几乎占满画面高度（不能映成"横版"，那不是真实相机能拍出的像）
DST_BIG = [[395, 6], [905, 14], [925, 714], [375, 706]]

# 反射率：胶带 0.07 / 纸面 0.85 / 亮墙 0.62 / 暗墙 0.055
PRESETS = [
    ("理想(基准)",     dict(wall=0.62, illum=300)),
    ("过曝",           dict(wall=0.62, illum=1500)),
    ("欠曝",           dict(wall=0.62, illum=95)),
    ("环境光大",       dict(wall=0.62, illum=300, flare=100)),
    ("暗背景",         dict(wall=0.055, illum=300)),
    ("暗背景+高照度",  dict(wall=0.055, illum=1200)),
    ("光照不均",       dict(wall=0.62, illum=330, uneven=0.75)),
    ("暗角",           dict(wall=0.62, illum=430, vignette=0.72)),
    ("反光",           dict(wall=0.62, illum=300, glare=1.0)),
    ("干扰物更大",     dict(wall=0.62, illum=300, clutter=[(10, 40, 560, 700, 0.05)])),
    ("远端小目标",     dict(wall=0.62, illum=300, dst=DST_SMALL)),
    ("近端大目标",     dict(wall=0.62, illum=300, dst=DST_BIG)),
    # ---- 远距离组：真机"远距离完全框不住"就是这几个（面积 <2%，曾被面积门全部砍掉）----
    ("远距(3.4m)",     dict(wall=0.62, illum=300, dst=DST_FAR)),
    ("远距+亮墙",      dict(wall=0.92, illum=300, dst=DST_FAR)),
    ("远距+暗背景",    dict(wall=0.055, illum=300, dst=DST_FAR)),
    ("远距+环境光",    dict(wall=0.62, illum=300, flare=90, dst=DST_FAR)),
    ("极远(5.2m)",     dict(wall=0.62, illum=300, dst=DST_FAR2)),
    # ---- 亮墙背景：纸面与墙都接近饱和，阈值分不开，只能靠"深色环"这一族 ----
    ("近距+亮墙",      dict(wall=0.92, illum=300)),
]


def project(Hm, x, y):
    v = Hm @ np.array([x, y, 1.0], dtype=np.float64)
    return np.array([v[0] / v[2], v[1] / v[2]])


def make_hard_scene(p, seed=1, tape_refl=0.07, paper_refl=0.85, tape_cm=1.8):
    """按反射率模型生成一张"真机可能拍到的"靶纸灰度图。"""
    rng = np.random.default_rng(seed)
    dst = np.float32(p.get("dst", DST_DEF))

    # ---- cm 画布：胶带环 + 纸面（反射率 0~1）----
    cw, ch = int(A4W * S), int(A4H * S)
    refl = np.full((ch, cw), paper_refl, np.float32)
    t = int(tape_cm * S)
    refl[:t, :] = tape_refl
    refl[-t:, :] = tape_refl
    refl[:, :t] = tape_refl
    refl[:, -t:] = tape_refl

    Ms = cv2.getPerspectiveTransform(
        np.float32([[0, 0], [cw, 0], [cw, ch], [0, ch]]), dst)
    wr = cv2.warpPerspective(refl, Ms, (W, H), flags=cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_CONSTANT, borderValue=0.0)
    m = cv2.warpPerspective(np.ones((ch, cw), np.float32), Ms, (W, H),
                            flags=cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT, borderValue=0.0)
    m = np.clip(m, 0.0, 1.0)

    # ---- 墙面（背景）----
    wall = np.full((H, W), float(p.get("wall", 0.62)), np.float32)
    for (x0, y0, x1, y1, g) in p.get("clutter", ()):        # 干扰物（比靶纸更大的深色块）
        wall[y0:y1, x0:x1] = float(g)

    img = wall * (1.0 - m) + wr * m

    # ---- 照度场：全局 + 单侧不均 + 镜头渐晕 ----
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    illum = np.full((H, W), float(p.get("illum", 300.0)), np.float32)
    if p.get("uneven"):
        illum *= 1.0 + float(p["uneven"]) * (2.0 * xx / W - 1.0)
    if p.get("vignette"):
        r2 = ((xx - W / 2.0) / (W / 2.0)) ** 2 + ((yy - H / 2.0) / (H / 2.0)) ** 2
        illum *= np.clip(1.0 - float(p["vignette"]) * r2, 0.05, None)
    img = img * illum

    # ---- 环境杂散光（把黑胶带整体抬灰）----
    if p.get("flare"):
        img = img + float(p["flare"])

    # ---- 胶带反光斑（把环打断）----
    if p.get("glare"):
        cx, cy = 0.5 * (dst[0][0] + dst[1][0]), 0.5 * (dst[0][1] + dst[1][1])
        s = 34.0
        blob = float(p["glare"]) * 165.0 * np.exp(
            -(((xx - cx) ** 2 + (yy - cy) ** 2) / (2.0 * s * s)))
        img = img + blob

    scene = np.clip(img, 0, 255).astype(np.uint8)
    if p.get("blur", 0.8) > 0:
        scene = cv2.GaussianBlur(scene, (0, 0), float(p.get("blur", 0.8)))
    if p.get("noise", 2.0) > 0:
        scene = np.clip(scene.astype(np.float32)
                        + rng.normal(0, float(p.get("noise", 2.0)), scene.shape),
                        0, 255).astype(np.uint8)

    truth = project(Ms, (cw - 1) / 2.0, (ch - 1) / 2.0)
    return cv2.cvtColor(scene, cv2.COLOR_GRAY2BGR), scene, truth, dst


def run_one(cfg, mode, gray, truth):
    cfg.SELECT_MODE = mode
    cfg.WORK_SCALE = Config.profile()[6]
    d = TargetCenterDetector(cfg)
    r = d.process(gray)
    if not r["valid"]:
        return dict(ok=False, err=np.nan, th=None, stage=r["stage"], det=d, res=r)
    err = float(np.hypot(r["raw_u"] - truth[0], r["raw_v"] - truth[1]))
    return dict(ok=True, err=err, th=r["th"], stage=r["stage"], det=d, res=r)


def mask_stats(cfg, gray):
    """诊断：当前固定阈值下二值图的黑色占比（>0.5 说明整幅几乎全黑；近 0 说明框没了）"""
    _, bw = cv2.threshold(gray, cfg.BLACK_THRESH, 255, cv2.THRESH_BINARY_INV)
    return float((bw > 0).mean())


def make_negative(kind, seed=1):
    """
    生成"**画面里根本没有靶纸**"的场景，用来验证不会误锁（误检比漏检更糟）。
    这些场景都含有"像框但不是框"的东西。
    """
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    wall = np.full((H, W), 0.62, np.float32)
    if kind == "深色实心块":
        wall[230:490, 540:740] = 0.05          # 实心深色矩形：有矩形外形，但**不是环**
    elif kind == "亮块贴亮墙":
        wall[230:490, 540:740] = 0.85          # 亮矩形，但周围不暗 -> 不构成"深色围亮面"
    elif kind == "一条深色横线":
        wall[350:380, :] = 0.05                # 贯穿画面的深色带（且贴边）
    elif kind == "大黑板":
        wall[90:630, 290:990] = 0.04           # 比靶纸大的深色实心块
    # "纯墙"：什么都不加
    ill = np.full((H, W), 300.0, np.float32) * (1.0 + 0.30 * (2.0 * xx / W - 1.0))
    scene = np.clip(wall * ill, 0, 255).astype(np.uint8)
    scene = cv2.GaussianBlur(scene, (0, 0), 0.8)
    scene = np.clip(scene.astype(np.float32) + rng.normal(0, 2.0, scene.shape),
                    0, 255).astype(np.uint8)
    return scene


NEGATIVES = ["纯墙", "深色实心块", "亮块贴亮墙", "一条深色横线", "大黑板"]


def check_negatives(modes):
    """反向验证：画面里没有靶纸时**必须拒检**（返回 False 表示锁到了不该锁的东西）"""
    print("\n" + "=" * 96)
    print(" 反向验证 · 画面里没有靶纸时会不会误锁   （正确行为 = 全部拒检）")
    print("=" * 96)
    allok = True
    for m in modes:
        bad = []
        for k in NEGATIVES:
            g = make_negative(k)
            r = run_one(Config, m, g, np.array([W / 2.0, H / 2.0]))
            if r["ok"]:
                bad.append("%s(锁到 %.0f,%.0f)" % (k, r["res"]["raw_u"], r["res"]["raw_v"]))
        tag = "旧(largest)" if m == "largest" else "新(score)"
        print("  %-12s %s" % (tag, "全部正确拒检" if not bad else "误锁: " + "、".join(bad)))
        allok = allok and not bad
    print("=" * 96)
    return allok


def report(modes):
    cfg = Config
    print("=" * 96)
    print(" 真实光照硬场景 · 锁框鲁棒性  （分辨率 %dx%d，WORK_SCALE=%.2f）"
          % (W, H, Config.profile()[6]))
    print("=" * 96)
    hdr = "%-14s %7s" % ("场景", "黑占比")
    for m in modes:
        hdr += " | %-22s" % ("largest" if m == "largest" else "score")
    print(hdr)
    print("-" * 96)

    tiles, fails = [], {m: [] for m in modes}
    for name, p in PRESETS:
        _, gray, truth, dst = make_hard_scene(p, seed=1)
        line = "%-14s %7.3f" % (name, mask_stats(cfg, gray))
        tile = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR).copy()
        cv2.polylines(tile, [np.float32(dst).astype(int).reshape(-1, 1, 2)],
                      True, (0, 255, 0), 1)
        for m in modes:
            r = run_one(cfg, m, gray, truth)
            if r["ok"]:
                line += " | %8.2fpx th=%-4s      " % (r["err"], r["th"])
                if r["err"] > 3.0:
                    fails[m].append("%s(偏%.0fpx)" % (name, r["err"]))
                if m == "score":
                    q = r["res"]["quad"].astype(int)
                    cv2.polylines(tile, [q.reshape(-1, 1, 2)], True, (0, 200, 255), 2)
                    cv2.drawMarker(tile, (int(r["res"]["raw_u"]), int(r["res"]["raw_v"])),
                                   (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
            else:
                line += " | %-22s" % "未检出"
                fails[m].append("%s(未检出)" % name)
                if m == "score":
                    cv2.putText(tile, "NOT FOUND", (12, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(tile, name, (12, H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2)
        tiles.append(cv2.resize(tile, (400, 225), interpolation=cv2.INTER_AREA))
        print(line)

    print("-" * 96)
    for m in modes:
        tag = "旧(largest)" if m == "largest" else "新(score)"
        n_bad = len(fails[m])
        print("  %-12s 失败/偏大 %2d/%d  %s"
              % (tag, n_bad, len(PRESETS), "全通过" if not n_bad else "、".join(fails[m])))
    print("=" * 96)

    if tiles:
        rows = [np.hstack(tiles[i:i + 3]) for i in range(0, len(tiles), 3)]
        w = min(t.shape[1] for t in rows)     # 注意是 shape[1]（宽），用 len() 会取成高度
        rows = [r[:, :w] for r in rows]
        mont = np.vstack(rows)
        ok, buf = cv2.imencode(".jpg", mont, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
        if ok:
            path = os.path.join(OUT_DIR, "hardscene_montage.jpg")
            buf.tofile(path)
            print(" [图] 各场景拼图（绿框=靶纸真值，橙框=新策略检出）: %s" % path)
    return fails


def main():
    args = [a for a in sys.argv[1:]]
    if "--mode" in args:
        modes = [args[args.index("--mode") + 1]]
    elif "--ab" in args:
        modes = ["largest", "score"]
    else:
        modes = ["score"]
    report(modes)
    if "--neg" in args or "--ab" in args or not args:
        check_negatives(modes)


if __name__ == "__main__":
    main()
