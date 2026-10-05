#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/make_icon.py · 生成应用图标 assets/icon.png（128x128）

MaixVision / MaixHub 的应用图标要求是 PNG（或 lottie json）。
图案：黑胶带矩形框 + 红色准星，和本应用做的事情一致。
运行：python tools/make_icon.py
"""

import os
import sys

import numpy as np
import cv2

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(_ROOT, "assets", "icon.png")

S = 128
img = np.full((S, S, 3), 245, np.uint8)          # 纸面浅色
t = 13                                            # 黑胶带宽度
cv2.rectangle(img, (0, 0), (S - 1, S - 1), (25, 25, 25), -1)
cv2.rectangle(img, (t, t), (S - 1 - t, S - 1 - t), (250, 250, 250), -1)
p0, p1 = t, S - 1 - t
cv2.rectangle(img, (p0, p0), (p1, p1), (25, 25, 25), 2)
c = (S // 2, S // 2)
cv2.circle(img, c, 22, (60, 60, 220), 3)
cv2.line(img, (c[0] - 34, c[1]), (c[0] + 34, c[1]), (60, 60, 220), 3)
cv2.line(img, (c[0], c[1] - 34), (c[0], c[1] + 34), (60, 60, 220), 3)
cv2.circle(img, c, 3, (60, 60, 220), -1)

os.makedirs(os.path.dirname(OUT), exist_ok=True)
# 路径含中文时 cv2.imwrite 在 Windows 下会静默失败，改用 imencode + tofile
ok, buf = cv2.imencode(".png", img)
if not ok:
    sys.exit("PNG 编码失败")
buf.tofile(OUT)
print("图标已生成: %s (%d bytes)" % (OUT, os.path.getsize(OUT)))
