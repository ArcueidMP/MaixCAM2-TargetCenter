class Config:
    # Camera profile. GRAYSCALE keeps the MaixCAM2 frame buffer small.
    CAMERA_W = 640
    CAMERA_H = 480
    CAMERA_FPS = 30
    CAMERA_BUFFERS = 2
    SKIP_FRAMES = 10
    LENS_CORR = True
    LENS_STRENGTH = 1.5
    SHOW = True
    # 检测不跳帧；显示独立限频，减少 RGB 转换和 MaixVision 图像传输开销。
    SHOW_WIDTH = 480
    SHOW_FPS = 15
    OPENCV_THREADS = 1
    LOG_INTERVAL = 2.0
    ENABLE_UART = False
    UART_PORT = "/dev/ttyS0"
    UART_BAUD = 115200

    # Canny and polygon gates. Tune only these values on a new lens.
    CANNY_LOW = 35
    CANNY_HIGH = 120
    # 失锁时先试上次有效阈值，再试 Otsu 和备用阈值；成功即停止。
    DARK_THRESHOLDS = (25, 40, 55, 70, 85, 100, 120, 145)
    # 稳态只采样四边窄带；失锁时缩小图搜索，每档验证成功就立即停止。
    SEARCH_WIDTH = 320
    # 合作式搜索：每帧至多做一次扫描或一次候选验证，时间预算是软预算。
    SEARCH_BUDGET_MS = 25.0
    SEARCH_MAX_PROPOSALS = 1
    SEARCH_GEOMETRY_STEPS = 8
    MAX_CONTOURS = 128
    LINE_MAX_PROPOSALS = 36
    ENABLE_LINE_RECOVERY = False
    LINE_SEARCH_INTERVAL = 15
    ACQUIRE_CONFIRM_FRAMES = 3
    CONFIRM_CORNER_PX = 12.0
    RECHECK_INTERVAL = 20
    MAX_LINE_RESIDUAL_PX = 2.0
    MIN_TRANSITION_COVERAGE = 0.50
    MAX_BOUNDARY_WIDTH_ERROR = 0.30
    MAX_CANDIDATES = 8
    EDGE_SAMPLES = 24
    REFINE_RADIUS = 6
    TRACK_RADIUS = 12
    ROI_PADDING = 0.20
    MIN_EDGE_COVERAGE = 0.55
    MAX_DARK_BRIGHT_RATIO = 0.72
    WEAK_EDGE_COVERAGE = 0.25
    FILTER_RESET_PX = 25.0
    MIN_AREA_RATIO = 0.025
    MAX_AREA_RATIO = 0.92
    APPROX_EPSILON = 0.025
    MIN_ANGLE_DEG = 55.0
    MIN_INNER_RATIO = 0.30
    MAX_INNER_RATIO = 0.94
    DARK_CENTER_MIN = 90.0
    MIN_EDGE_CONTRAST = 20.0
    BLACK_FRAME_MM = 18.0

    # Physical target coordinates. The center is invariant to scale, but the
    # aspect ratio must match the printed target for a perspective correction.
    TARGET_SHORT_MM = 210.0
    TARGET_LONG_MM = 297.0
    CIRCLE_SAMPLES = 72
    CIRCLE_RADIUS_MM = 2.0

    # Optional temporal smoothing. Zero disables it.
    CENTER_ALPHA = 0.70

    # ---- 激光光斑读数（相机与激光固连 -> 光斑位置近似固定）------------------
    # 只在标称位置周围 LASER_ROI_RADIUS 内找最亮点，输出亚像素质心，并给出它到
    # 黑框中心的水平/垂直偏差，画在屏幕左上角。2026-10-07 现场标定：光斑约在
    # (294, 202)，背景灰度 145 / 峰值 177。换镜头或重装激光笔后重新标定这两个值，
    # 也能直接看板端日志 [perf] ... laser= 与屏幕读数反推真实位置。
    ENABLE_LASER = True
    LASER_U_NOMINAL = 294
    LASER_V_NOMINAL = 202
    LASER_ROI_RADIUS = 64
    # 光斑位置从哪来：
    #   "fixed"（默认） 不检测，直接用上面的标定常量 —— 光斑位置写死，每帧只算
    #                  "黑框中心 − 该常量"的像素差，零检测开销；
    #   "detect"        每 LASER_INTERVAL_S 实测一次光斑（自校准，吸收机构/温度漂移）。
    # 需要重新标定 U/V 时：临时改成 "detect"，把屏幕左上角读到的 `LASER (u,v)` 填回上面两行。
    LASER_MODE = "fixed"
    # 判据：峰值 >= max(LASER_MIN_PEAK, 背景 + LASER_MIN_CONTRAST)，且亮点面积在
    # [MIN_PIXELS, MAX_PIXELS] 内。面积上限用来挡白墙高光/灯罩反光（那些会很大）。
    LASER_MIN_PEAK = 140.0
    LASER_MIN_CONTRAST = 20.0
    LASER_MIN_PIXELS = 2
    LASER_MAX_PIXELS = 160
    # 限频：默认 0.30s ≈ 每秒算 3 次，其余帧沿用上次结果（显示仍是每帧画）。
    LASER_INTERVAL_S = 0.30
    # 单次更新跳变上限（像素）：超过判为误检，保留上次位置。0 = 关闭该门限。
    LASER_MAX_JUMP_PX = 40.0
    # 位置平滑（0 = 不平滑，直接用最新质心）。
    LASER_SMOOTH_ALPHA = 0.60
