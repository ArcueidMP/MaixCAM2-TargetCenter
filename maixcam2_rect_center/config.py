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
    # Dark mask fallback. Otsu is always tried first, these values cover
    # underexposed MaixCAM2 frames and bright-reflection scenes.
    DARK_THRESHOLDS = (25, 40, 55, 70, 85, 100, 120, 145)
    DARK_CENTER_MIN = 90.0
    # 稳态只采样四边窄带；失锁时缩小图搜索，每档验证成功就立即停止。
    SEARCH_WIDTH = 320
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
