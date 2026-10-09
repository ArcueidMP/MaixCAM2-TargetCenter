"""图片级端到端验收：`python -B tools/test_images_e2e.py`。

把「合成靶纸图（多视角/光照/反光/噪声，真值来自独立 OpenCV 投影）」加上
「真实拍摄的负样本照片」喂给 B 轨对外 API，逐图输出：
    是否检出 / stage / 中心误差(px) / 耗时(ms)
并生成一张标注总览图 tools/e2e_report/e2e_overview.png 供人工目视核对。

纯 PC，不依赖 maix / 相机。用于回答「B 轨功能现在能否正常用」。
"""
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from target_api import detect_black_frame, CenterTracker
from vision.rectangle_center import draw_result

OUT = ROOT / "tools" / "e2e_report"
OUT.mkdir(parents=True, exist_ok=True)
W, H = 640, 480


# --------------------------------------------------------------- 合成场景
def scene(q, paper=220, tape=20, background=175, glare=False, noise=False, low=False):
    """把 210x297mm 的正视靶纸按单应 q 投到 640x480；真值由独立投影给出。"""
    h = cv2.getPerspectiveTransform(
        np.float32([[0, 0], [210, 0], [210, 297], [0, 297]]), np.float32(q))
    if low:
        paper, tape = 150, 92          # 低对比：亮面灰、胶带灰
    target = np.full((298, 211), tape, np.uint8)
    target[18:280, 18:193] = paper
    cv2.circle(target, (105, 148), 38, int(paper * .72), 1)
    if glare:
        cv2.ellipse(target, (85, 13), (23, 18), 0, 0, 360, min(255, paper + 20), -1)
        cv2.ellipse(target, (120, 140), (28, 9), 25, 0, 360, 250, -1)
    img = cv2.warpPerspective(target, h, (W, H), borderValue=background)
    if noise:
        img = np.clip(img.astype(np.float32) + np.random.default_rng(7).normal(0, 3, img.shape),
                      0, 255).astype(np.uint8)
    truth = cv2.perspectiveTransform(np.float32([[[105, 148.5]]]), h).reshape(2)
    return img, truth


BASE = np.array([[180, 70], [445, 80], [475, 400], [150, 420]], np.float32)


def build_cases():
    cases = []

    def add(name, img, truth, expect):
        cases.append(dict(name=name, img=img, truth=truth, expect=expect))

    # ---- 正样本：应当检出 ----
    add("frontal", *scene(BASE), True)
    add("strong_tilt", *scene([[120, 40], [520, 60], [540, 430], [90, 410]]), True)
    add("far_small", *scene([[250, 160], [390, 165], [395, 320], [245, 315]]), True)
    add("near_large", *scene([[40, 25], [600, 30], [595, 450], [35, 445]]), True)
    add("dark_bg", *scene(BASE, background=25), True)
    add("low_contrast", *scene(BASE, low=True), True)
    add("glare", *scene(BASE, glare=True, background=185), True)
    add("noise", *scene(BASE, noise=True), True)
    for ang in (42, 90, 135, 180):
        a = np.deg2rad(ang)
        rot = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        q = np.array([[-105, -148.5], [105, -148.5], [105, 148.5], [-105, 148.5]]) @ rot.T + [320, 240]
        add("rot%d" % ang, *scene(q), True)

    # ---- 负样本：画面里没有靶纸，必须拒绝 ----
    add("blank", np.full((H, W), 150, np.uint8), None, False)
    solid = np.full((H, W), 175, np.uint8)
    cv2.rectangle(solid, (150, 70), (460, 400), 20, -1)
    add("solid_dark_block", solid, None, False)

    # ---- 真实拍摄照片（桌面/笔记本，无靶纸）----
    # 路径含中文，cv2.imread 会失败，改用 imdecode。
    for fn in ("negative_e421.png", "negative_e880.png"):
        p = ROOT / "tools" / fn
        if not p.exists():
            continue
        buf = np.fromfile(str(p), dtype=np.uint8)
        im = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if im is not None:
            add("photo_" + fn.split("_")[-1].replace(".png", ""),
                cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), None, False)
    return cases


# --------------------------------------------------------------- 可视化
def tile(img, name, verdict, line2, ok):
    tw, th = 320, 240
    body = cv2.resize(img, (tw, th), interpolation=cv2.INTER_AREA)
    banner = np.zeros((38, tw, 3), np.uint8)
    color = (0, 210, 0) if ok else (0, 0, 235)
    cv2.rectangle(banner, (0, 0), (tw, 3), color, -1)
    cv2.putText(banner, name, (6, 15), cv2.FONT_HERSHEY_SIMPLEX, .46, (240, 240, 240), 1)
    cv2.putText(banner, verdict, (6, 32), cv2.FONT_HERSHEY_SIMPLEX, .40, color, 1)
    cv2.putText(banner, line2, (120, 32), cv2.FONT_HERSHEY_SIMPLEX, .38, (200, 200, 200), 1)
    return np.vstack([banner, body])


def montage(tiles, cols=4, pad=6):
    while len(tiles) % cols:
        tiles.append(np.zeros_like(tiles[0]))
    rows = [np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)]
    grid = np.vstack(rows)
    return cv2.copyMakeBorder(grid, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=(30, 30, 30))


# --------------------------------------------------------------- 主流程
def main():
    cases = build_cases()
    print("=" * 78)
    print("B 轨图片级端到端验收  %d 张（正样本 %d / 负样本 %d）" %
          (len(cases), sum(c["expect"] for c in cases), sum(not c["expect"] for c in cases)))
    print("=" * 78)
    print("%-20s %-7s %-13s %-11s %-9s %s" %
          ("case", "expect", "result", "stage", "err(px)", "ms"))

    rows, tiles = [], []
    n_pass = 0
    for c in cases:
        res = detect_black_frame(c["img"])
        valid = bool(res)
        if c["expect"]:
            err = float(np.hypot(res.raw_u - c["truth"][0], res.raw_v - c["truth"][1])) if valid else None
            ok = valid and err is not None and err < 3.0
            res_txt = "HIT" if valid else "MISS"
            err_txt = ("%.3f" % err) if err is not None else "-"
        else:
            ok = not valid
            res_txt = "reject" if not valid else "FALSE+"
            err_txt = "-"
        n_pass += ok
        ms = res.get("ms", 0.0)
        print("%-20s %-7s %-13s %-11s %-9s %.3f" %
              (c["name"], "YES" if c["expect"] else "NO", res_txt,
               res.get("stage"), err_txt, ms))
        rows.append(dict(case=c["name"], expect=c["expect"], valid=valid,
                         stage=res.get("stage"),
                         err=err_txt if c["truth"] is not None else None,
                         ms=round(ms, 3), ok=ok))

        vis = draw_result(c["img"], res, show_width=W)
        vtxt = ("OK  %s" % res.get("stage")) if ok else "!!  %s" % res.get("stage")
        tiles.append(tile(vis, c["name"], vtxt,
                          ("err=%s" % err_txt) if c["truth"] is not None else "", ok))

    overview = montage(tiles, cols=4)
    # 仓库路径含中文，cv2.imwrite 会静默失败，用 imencode + tofile。
    ok, buf = cv2.imencode(".png", overview)
    assert ok, "encode overview failed"
    (OUT / "e2e_overview.png").write_bytes(buf.tobytes())

    # ---- 连续帧跟踪烟测：同一目标平移 -> 应全程 track，不丢锁 ----
    print("-" * 78)
    tr = CenterTracker()
    seq_ok, stages, errs = True, [], []
    for dx in (0, 4, 8, 12, 16, 40, 4, 0):
        im, truth = scene(BASE + [dx, 0])
        r = tr.track(im)
        if not r.valid:
            seq_ok = False
        stages.append(r.get("stage"))
        errs.append(round(float(np.hypot(r.raw_u - truth[0], r.raw_v - truth[1])), 3) if r.valid else None)
    print("连续帧跟踪  stages=%s" % stages)
    print("            err(px)=%s" % errs)
    print("            最大误差=%.3fpx  全程锁定=%s" % (max(e for e in errs if e), seq_ok))

    passed = n_pass == len(cases)
    print("=" * 78)
    print("逐图验收: %d/%d 通过" % (n_pass, len(cases)))
    print("连续帧跟踪: %s" % ("通过" if seq_ok else "失败"))
    print("总览图 -> %s" % (OUT / "e2e_overview.png"))
    print("=" * 78)
    (OUT / "e2e_result.json").write_text(
        json.dumps(dict(rows=rows, seq_stages=stages, seq_errs=errs,
                        passed=n_pass, total=len(cases)), ensure_ascii=False, indent=2),
        encoding="utf-8")
    if not (passed and seq_ok):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
