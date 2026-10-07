"""PC 回归/性能比较：python -B tools/test_performance.py [--legacy PATH]。

合成图中心真值来自独立 OpenCV 投影，不用待测实现生成预期结果。
PC 耗时只比较改动前后；不能换算为 MaixCAM2 实测帧率。
"""
import argparse
import importlib.util
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from config import Config
from vision.rectangle_center import RectangleCenterDetector, draw_result


def scene(q, paper=220, tape=20, background=175, glare=False, noise=False):
    h = cv2.getPerspectiveTransform(np.float32([[0, 0], [210, 0], [210, 297], [0, 297]]), np.float32(q))
    target = np.full((298, 211), tape, np.uint8)
    target[18:280, 18:193] = paper
    cv2.circle(target, (105, 148), 38, int(paper * .72), 1)
    if glare:
        cv2.ellipse(target, (85, 13), (23, 18), 0, 0, 360, min(255, paper + 20), -1)
        cv2.ellipse(target, (120, 140), (28, 9), 25, 0, 360, 250, -1)
    result = cv2.warpPerspective(target, h, (640, 480), borderValue=background)
    if noise:
        noise_arr = np.random.default_rng(11).normal(0, 2, result.shape)
        result = np.clip(result.astype(np.float32) + noise_arr, 0, 255).astype(np.uint8)
    truth = cv2.perspectiveTransform(np.float32([[[105, 148.5]]]), h).reshape(2)
    return result, truth


def fixtures():
    rng = np.random.default_rng(20261005)
    out = []
    for i in range(24):
        q = [[rng.uniform(70,190), rng.uniform(35,100)],
             [rng.uniform(395,550), rng.uniform(30,120)],
             [rng.uniform(425,560), rng.uniform(355,445)],
             [rng.uniform(60,185), rng.uniform(335,445)]]
        paper, tape, bg = (120, 20, 95) if i % 4 == 0 else (220, 20, 175)
        if i % 4 == 1:
            bg = 20  # 黑环外沿与暗背景粘连，亮面孔必须可用。
        out.append(scene(q, paper, tape, bg, glare=i % 4 == 2, noise=True))
    for angle in (0, 42, 45, 90, 135, 180):
        a = np.deg2rad(angle)
        rot = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        q = np.array([[-105,-148.5], [105,-148.5], [105,148.5], [-105,148.5]]) @ rot.T + [320,240]
        out.append(scene(q))
    return out


def benchmark(factory, image, count=60):
    det = factory(Config)
    acquire(det, image)  # 连续确认完成后才计入稳态耗时。
    times = []
    tracked = 0
    for _ in range(count):
        t = time.perf_counter()
        r = det.process(image)
        times.append((time.perf_counter() - t) * 1000)
        tracked += r.get('stage') == 'track'
    return dict(median_ms=round(float(np.median(times)), 3),
                p95_ms=round(float(np.percentile(times, 95)), 3), tracked=tracked)


def acquire(det, image, limit=120):
    for i in range(limit):
        result = det.process(image)
        if result['valid']:
            return result, i + 1
    return result, limit


def run(legacy_path=None):
    cv2.setNumThreads(Config.OPENCV_THREADS)
    data = fixtures()
    errors, failed = [], []
    for i, (image, truth) in enumerate(data):
        result, _ = acquire(RectangleCenterDetector(Config), image)
        if not result.get('valid'):
            failed.append(i)
        else:
            errors.append(float(np.linalg.norm([result['raw_u'] - truth[0], result['raw_v'] - truth[1]])))
    negatives = [np.full((480,640), v, np.uint8) for v in (20,120,220)]
    bright_box = np.full((480,640), 175, np.uint8)
    cv2.rectangle(bright_box, (150,70), (460,400), 225, -1)
    negatives.append(bright_box)
    dark_box = np.full((480,640), 175, np.uint8)
    cv2.rectangle(dark_box, (150,70), (460,400), 20, -1)
    negatives.append(dark_box)
    false_positives = sum(acquire(RectangleCenterDetector(Config), im)[0]['valid'] for im in negatives)

    base_q = np.array([[180,70],[445,80],[475,400],[150,420]], np.float32)
    det = RectangleCenterDetector(Config)
    acquire(det, scene(base_q)[0])
    movement_errors, stages = [], []
    for dx in (0,3,6,9,12,15,18,21,24,27,90,94,98):
        im, truth = scene(base_q + [dx,0])
        r = det.process(im)
        if not r['valid']:
            r, _ = acquire(det, im)
        stages.append(r['stage'])
        movement_errors.append(float(np.linalg.norm(np.array([r.get('raw_u',-999),r.get('raw_v',-999)]) - truth)))
    lost = det.process(negatives[-1])
    reacquired, _ = acquire(det, data[0][0])
    report = dict(fixtures=len(data), detected=len(errors), failed=failed,
                  max_center_error_px=round(max(errors, default=999),3),
                  negative_cases=len(negatives), false_positives=false_positives,
                  motion_max_error_px=round(max(movement_errors),3), motion_stages=stages,
                  lost_valid=lost['valid'], reacquired=reacquired['valid'],
                  optimized=benchmark(RectangleCenterDetector, data[0][0]))
    if legacy_path:
        spec = importlib.util.spec_from_file_location('legacy', legacy_path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        class LegacyConfig(Config):
            MIN_ANGLE_DEG = 55.0
            MIN_INNER_RATIO = 0.30
            MAX_INNER_RATIO = 0.94
            DARK_CENTER_MIN = 90.0
        report['legacy'] = benchmark(lambda cfg: mod.RectangleCenterDetector(LegacyConfig), data[0][0])
        report['speedup'] = round(report['legacy']['median_ms'] / report['optimized']['median_ms'],2)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    assert len(failed) <= 8, 'too many missed fixtures'
    assert max(errors) < 3, 'corner/center regression'
    assert not false_positives, 'solid rectangles falsely accepted'
    assert max(movement_errors) < 3, 'motion tracking drift'
    assert not lost['valid'] and reacquired['valid'], 'loss/reacquisition failed'
    assert report['optimized']['tracked'] == 60, 'steady state must not scan thresholds'


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--legacy')
    args = parser.parse_args()
    run(args.legacy)
