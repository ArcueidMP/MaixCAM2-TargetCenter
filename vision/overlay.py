#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
overlay.py · 结果可视化（OpenCV 绘制，预分配画布，零逐帧内存分配）
================================================================================
为什么不用 MaixPy 原生 draw_* ？
    ① 相机现在输出 GRAYSCALE，在其上画彩色标注行为不确定；
    ② 直接在 1280x720 原图上绘制会一直持有一张大缓冲。
做法：把灰度图缩小到 SHOW_W 宽（默认 640），在**复用的** RGB 画布上画标注，
      再交给 display 显示。画布只在启动时分配一次，之后每帧复用同一块内存。
"""

import numpy as np
import cv2


class Overlay:
    def __init__(self, cfg):
        self.cfg = cfg
        self._gray = None       # HxW uint8，复用
        self._canvas = None     # HxWx3 RGB888，复用

    def _ensure(self, h, w):
        if self._gray is None or self._gray.shape[0] != h or self._gray.shape[1] != w:
            self._gray = np.empty((h, w), np.uint8)
            self._canvas = np.empty((h, w, 3), np.uint8)

    def render(self, gray, res, fps=None, extra=""):
        """
        gray : 全分辨率灰度图（HxW）
        res  : detector.process() 的返回（坐标为全分辨率像素）
        返回 : RGB888 的 numpy 画布（内存复用，下一帧会被覆盖）
        """
        H, W = gray.shape[:2]
        cw = int(self.cfg.SHOW_W)
        ch = max(2, int(round(cw * H / float(max(1, W)))))
        self._ensure(ch, cw)
        cv2.resize(gray, (cw, ch), dst=self._gray, interpolation=cv2.INTER_AREA)
        cv2.cvtColor(self._gray, cv2.COLOR_GRAY2RGB, dst=self._canvas)
        img = self._canvas
        s = cw / float(max(1, W))          # 全分辨率 -> 画布 的缩放

        if res.get("valid"):
            q = np.rint(np.asarray(res["quad"], dtype=np.float64) * s).astype(np.int32)
            # 板框 4 条边（绿）
            cv2.polylines(img, [q.reshape(-1, 1, 2)], True, (0, 255, 0), 2)
            # 求交用的两条对角线（蓝）
            cv2.line(img, tuple(q[0]), tuple(q[2]), (255, 60, 60), 1)
            cv2.line(img, tuple(q[1]), tuple(q[3]), (255, 60, 60), 1)
            # 4 个顶点（黄）
            for p, lb in zip(q, ("TL", "TR", "BR", "BL")):
                cv2.circle(img, (int(p[0]), int(p[1])), 5, (0, 255, 255), 2)
                cv2.putText(img, lb, (int(p[0]) + 7, int(p[1]) + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
            # 靶心（红）
            c = (int(round(res["u"] * s)), int(round(res["v"] * s)))
            cv2.drawMarker(img, c, (255, 0, 0), cv2.MARKER_CROSS, 22, 2)
            cv2.circle(img, c, 2, (255, 0, 0), -1)
            txt = "center=(%.1f, %.1f)  %.2f px/cm  %s" % (
                res["u"], res["v"], res["scale"], res["stage"])
        else:
            cv2.putText(img, "TARGET NOT FOUND", (8, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2, cv2.LINE_AA)
            txt = res.get("stage", "")

        cv2.rectangle(img, (0, 0), (cw - 1, 20), (0, 0, 0), -1)
        cv2.putText(img, txt, (6, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (255, 255, 255), 1, cv2.LINE_AA)
        if fps is not None:
            cv2.putText(img, "%.1f fps" % fps, (cw - 74, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1, cv2.LINE_AA)
        if extra:
            cv2.putText(img, extra, (6, ch - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (200, 200, 200), 1, cv2.LINE_AA)
        return img
