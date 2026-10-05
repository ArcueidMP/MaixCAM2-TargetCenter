#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vision 包：靶纸矩形框中心点（靶心）识别的纯算法层。

模块划分（都不 import maix，因此可在 PC 上用合成图直接验证，见 tools/test_pc.py）：
    geometry.py  纯 NumPy 几何：极值顶点 / 排序 / 对角线求交 / 尺度 / One Euro 滤波
    tracker.py   抗抖跟踪：常数速度卡尔曼 + 新息门限 + 尺度/清晰度一致性（时序稳定化）
    detector.py  OpenCV 部分：二值化 / 描边找轮廓 / 角点亚像素精修 + 检测器主流程
    overlay.py   OpenCV 绘制：预分配画布复用，零逐帧分配

⚠ 本文件的 __init__ 故意不 import 子模块，
   这样只需要 geometry（无 cv2 依赖）的场景不会被强制拉入 opencv。
   使用时请显式写 `from vision.geometry import ...` / `from vision.detector import ...`。
"""
