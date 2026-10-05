# MaixCAM2 矩形黑框中心识别

这是一个独立的 MaixPy 应用目录，放在 `25Efork/maixcam2_rect_center/`。入口固定为 `main.py`，可在 MaixVision 中用“运行项目”下发整个文件夹。

## 算法

1. 相机直出灰度图，先用 Canny 提取黑框边缘，同时用 Otsu/多档暗阈值生成黑色掩膜；这样反光把边缘打断时仍能保留黑框候选。
2. `findContours + approxPolyDP` 筛选凸四边形，并按面积、直角和嵌套关系配对外框/内框；若只得到一个可靠黑色轮廓，则按 A4 黑框宽度合成另一条边。
3. 沿四条边在“白底-黑框-白底”方向采样灰度剖面，使用四条边的内外灰度差拒绝桌面、阴影和普通矩形误检。
4. 把外框四角对应到 1 px=1 mm 的正视矩形，使用 8 个方程和高斯消元求 8 个单应性参数。
5. 在正视面以矩形中心为圆心生成 72 个标准圆采样点，逐点用逆单应性映射回原图；实际中心使用“正视面圆心”的直接逆映射，避免强透视下平均圆周点产生偏移。这个步骤对相机视角、旋转和透视畸变有效。

代码不依赖 `cv2.getPerspectiveTransform`，单应性方程在 `vision/rectangle_center.py` 的 `homography_8x8()` 中显式实现。识别成功后会显示绿色外框、黄色内沿，以及 `P1~P4` 四个顶点坐标。

## MaixCAM2 运行

在 MaixVision 中打开本目录，选择“运行项目”。相机默认 640x480、灰度、双缓冲；需要串口输出时修改 `config.py` 的 `ENABLE_UART`、`UART_PORT`。

若只运行一个文件，请把 `main.py`、`config.py` 和 `vision/` 一起下发；只下发 `main.py` 会缺少算法模块。

## 调参

现场优先调整 `config.py` 中的 `CANNY_LOW/HIGH` 和 `MIN_EDGE_CONTRAST`。`TARGET_SHORT_MM/TARGET_LONG_MM` 应与靶纸实际比例一致；中心点本身对整体毫米尺度不敏感，但长宽比会影响透视校正。
