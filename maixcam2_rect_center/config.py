class Config:
    # Camera profile. GRAYSCALE keeps the MaixCAM2 frame buffer small.
    CAMERA_W = 640
    CAMERA_H = 480
    CAMERA_FPS = 30
    CAMERA_BUFFERS = 2
    SKIP_FRAMES = 10
    # 镜头畸变校正：默认关闭 —— 未按本镜头标定过的软件畸变校正会**引入**畸变
    # （识别黑框时画面外凸/边缘拉伸，2026-10-10 实测定案）。确需把边缘直线拉直、
    # 且已按本镜头标定 LENS_STRENGTH 时，再临时打开。
    LENS_CORR = False
    LENS_STRENGTH = 1.5
    SHOW = True
    # 视频主循环与后台识别/串口解耦；后台忙时只保留最新帧，不积压旧图像。
    # ★ 画布宽度：0 = 用**屏幕原始宽度**（推荐）。MaixCAM2 屏幕 640×480 与本工程相机
    #   (CAMERA_W/H=640×480) 宽高比一致，画布=屏幕尺寸时 show() 拿到与屏幕同尺寸的图，
    #   **驱动层不必再每帧缩放一次**（旧值 480 会让 640×480 先被缩到 480×360、再被驱动
    #   放大回 640×480，纯属来回折腾）；画布坐标还与触摸坐标天然 1:1，省掉比例换算。
    #   >0 才按比例缩小 —— 只有"MaixVision 图传带宽吃紧"时才值得用（本机屏幕会被放大回全屏）。
    SHOW_WIDTH = 0
    SHOW_FPS = 30
    SHOW_JPEG_QUALITY = 70   # 只影响 MaixVision JPEG 图传，检测始终用原始灰度图。
    # ★ MaixVision 图传总开关（决定"能跑满全帧"还是"被 500ms 忙等拖垮"）。
    #   背景：MaixCDK 的 maixvision_mode() 只看**环境变量 MAIXVISION 是否为 "1"**；
    #   display.Display() 构造时据此创建图传通道 img_trans，而 show() 的第一步就是
    #   img_trans->send_image()。若处于 MaixVision 模式但画面预览**没真正连上**，
    #   该函数会 `while(!init){ if(ticks-t>500) return; }` **忙等 500ms 并空转 CPU**
    #   （源码 components/vision/src/maix_image_trans.cpp）——以 ~2 次/秒计 ≈ 烧掉近一个核，
    #   相机 read / 识别 / 闭环随之塌方（板端实测 show≈505ms、read≈107ms、detect_fps=0）。
    #   True（默认）：main() 在建 Display **之前**把 MAIXVISION 置 "0" ⇒ 不建图传 ⇒ show() 只
    #     画本地屏（官方 1~5ms）⇒ 全帧。代价：MaixVision 的"画面预览"面板收不到图（终端日志照常）。
    #   False：保留图传。想用 MaixVision 看画面、且能保证预览真连上（连上后是异步入队，不忙等）时再设。
    #   注：从设备本地启动应用时本来就没有图传，此开关无副作用。
    DISABLE_MAIXVISION_STREAM = True
    VISION_ASYNC = True
    # ★ 显示线程化：True = 绘制完成后把画布交给**独立显示线程**去 show()，主循环**永不等待 show**。
    #   屏幕驱动或 MaixVision 图传慢时，只表现为"预览掉帧"（丢帧而非排队），
    #   识别、串口与云台闭环仍跑满相机帧率。False = 旧行为（主循环内同步 show，慢就整体卡住）。
    SHOW_ASYNC = True
    RESULT_MAX_AGE_S = 0.20  # 以采图时刻计龄；过期结果不画框、不下发追踪指令。
    UART_RX_MAX_BYTES = 256
    UART_RX_BUFFER_MAX_BYTES = 1024
    # ---- 屏幕交互：把「上电自检」做成屏幕上的可点击按钮（不再上电自动跑）----
    # True=在显示画布右上角画 SELF-TEST 按钮，点一下才触发自检；无显示/无触摸屏时自动禁用。
    ENABLE_TOUCH_BUTTON = True
    OPENCV_THREADS = 1
    LOG_INTERVAL = 2.0
    # ---- 双 ZDT 云台链路 -------------------------------------------------
    # ★ 实测轴映射定案（2026-10-09 实机测定，见 AGENTS.md §2；勿凭直觉改）：
    #     yaw   = UART2 / B0,B1 / /dev/ttyS2 / 驱动器 ID=2
    #     pitch = UART0 / A28,A29 / /dev/ttyS0 / 驱动器 ID=1
    #   起因：最初以为 "U0=yaw、U2=pitch"，实机发现正好相反。地址跟着"电机"走，不跟着端口。
    #   首次上板请先把云台架空，并确认驱动器已设为 UART_FUN/115200/0x6B。
    ENABLE_GIMBAL = True
    YAW_DEV, YAW_BAUD = "/dev/ttyS2", 115200
    YAW_PINS = {"B0": "UART2_TX", "B1": "UART2_RX"}
    PITCH_DEV, PITCH_BAUD = "/dev/ttyS0", 115200
    # ⚠ U0(A28/A29) 并非默认复用成 UART0，走空表会静默不通 —— 必须显式 pinmap；
    #    且 ttyS0 还是系统日志口（开机有内核日志/终端回显），故下面 FLUSH_RX_ON_START=True。
    PITCH_PINS = {"A28": "UART0_TX", "A29": "UART0_RX"}
    ADDR_YAW, ADDR_PITCH = 2, 1
    # pinmap 失败时：True=报错中止；False=只告警、仍尝试打开 UART（避免把真故障藏成"打不开"）。
    PINMAP_STRICT = False
    # 开机清空两条串口的 RX 缓冲（U0 是日志口，会混入内核启动日志/终端回显）。
    FLUSH_RX_ON_START = True
    # 闭环方向符号：两轴正方向均为 CCW（yaw CCW=左 / pitch CCW=上）。按几何推断取 -1
    # （相机与激光同装在云台上 ⇒ 云台转过去时靶心在画面里反向移动）。
    # ⚠ 未经标定：上电追踪若"越追越偏"（误差持续变大、云台一路跑向一侧），把对应那项改成 +1。
    DIR_SIGN_YAW = -1
    DIR_SIGN_PITCH = -1
    PULSE_PER_PX_YAW = 3.67       # 必须用实机开环标定替换
    PULSE_PER_PX_PITCH = 3.67     # 必须用实机开环标定替换
    GAIN_YAW = 0.5
    GAIN_PITCH = 0.5
    DEADBAND_PX = 2.0
    MAX_STEP_PULSES = 360
    RPM_FINE = 300
    # 曲线加减速档位（位置模式）：0 = 不用曲线、直接以设定速度瞬时启停（卡顿/冲击根源，勿用）；
    # ≠0 时每 (256-acc)*50us 变 1RPM，acc 越小越柔（常用 20~80）。自检与闭环共用本值。
    ACC_DEFAULT = 80
    COMMAND_INTERVAL_S = 0.08

    # 上电自检：度数对应的脉冲数必须按实际减速机构标定。
    # 0 表示未标定，程序跳过自动扫动，避免用错误比例撞限位。
    GIMBAL_SELF_TEST = True
    SELF_TEST_YAW_DEG = 15.0
    SELF_TEST_PITCH_DEG = 30.0
    # 固定脉冲自检幅度（AGENTS.md §3.2）：>0 时**不依赖**"脉冲/度"标定即可动作，
    # 因此屏幕按钮点了就能跑（推荐）。置 0 才回退到按 SELF_TEST_*_DEG × PULSES_PER_DEG 换算角度。
    SELF_TEST_PULSES = 400
    YAW_PULSES_PER_DEG = 0.0
    PITCH_PULSES_PER_DEG = 0.0
    # 两轴正方向均为 CCW：yaw CCW=左 / CW=右，pitch CCW=上(仰) / CW=下（见 AGENTS.md §2）。
    YAW_POSITIVE_CCW = True
    PITCH_POSITIVE_CCW = True
    SELF_TEST_RPM = 900
    SELF_TEST_SETTLE_S = 0.35
    MOTOR_PULSES_PER_REV = 3200
    # 减速比未知时留 0：自检角度换算优先用上面的实测 PULSES_PER_DEG，不猜齿轮比。
    # 若已知 i = 大齿轮齿数/小齿轮齿数，填进来即可让自检在未做脉冲/度标定时也生效。
    GIMBAL_GEAR_RATIO = 0.0

    # ---- 闭环口径（需求文档 §5.2）-----------------------------------------
    # True = 用未平滑的 raw_u/raw_v 算 dx/dy（推荐）：CENTER_ALPHA 只留给显示，
    # 闭环用平滑值会附加 ≈28ms 滞后，底盘运动时直接变成瞄准误差。
    # 切换坐标系后 PULSE_PER_PX_* 必须在同一套坐标下重新标定。
    AIM_USE_RAW = True
    # 离群拒收：|误差| 超过该值先当误检丢弃，连续 JUMP_CONFIRM_FRAMES 帧才采信。
    # 防止单帧误检（反光/干扰）给出 dx=2000px 而甩头。
    JUMP_REJECT_PX = 400.0
    JUMP_CONFIRM_FRAMES = 3
    # 粗/精两段增益：|e| > COARSE_PX 用 GAIN_COARSE（大步快速逼近），否则用 GAIN_YAW/PITCH。
    # 必须保持 gain <= 1，否则增量闭环不收敛（需求文档 §7.4）。
    COARSE_PX = 24.0
    GAIN_COARSE = 0.9
    RPM_COARSE = 1200

    # ---- 链路健康（ACK / E2 看门狗，需求文档 §7.9）------------------------
    ENABLE_ACK_WATCHDOG = True
    E2_REENABLE = True          # 收到 E2（未使能/堵转）自动补发一次 0xF3
    E2_WINDOW_S = 2.0
    E2_FAULT_COUNT = 10         # 窗口内 E2 超过该次数 -> FAULT 停机
    # ACK 硬超时（ms）：自动识别 —— 只有在收到过确认帧（驱动器 Response=Receive/Reached/Both）
    # 之后才启用；Response=None 时收不到确认帧，该判据自动不生效，不会误判 FAULT。0 = 关闭。
    RX_TIMEOUT_MS = 300
    RX_TIMEOUT_LIMIT = 20

    # ---- 失锁重捕获（可选，默认关）---------------------------------------
    # 上电目标不在视野内时可开启自动慢扫；扫动为小步低转速、有累计脉冲上限，
    # 避免大幅甩动。开启前先确认机械行程与限位。
    ENABLE_SCAN = False
    LOST_FRAMES = 15
    SCAN_STEP_DEG = 1.5
    SCAN_LIMIT_DEG = 15.0
    SCAN_STEP_PULSES = 40       # 未标定脉冲/度时用的脉冲步长
    SCAN_LIMIT_PULSES = 400
    SCAN_RPM = 60
    SCAN_STEP_INTERVAL_S = 0.15

    # ---- 开环标定（一次性；标完把 CALIBRATE_AXIS 置回 ""）----------------
    # "yaw"/"pitch"：开机只做该轴开环标定，实测并打印 PULSE_PER_PX/DIR_SIGN/交叉项/回差。
    # 标定期不发闭环指令。跑一次拿到数值填进本文件后，务必置回 ""。
    CALIBRATE_AXIS = ""
    CALIBRATE_PULSES = 2000
    CALIBRATE_RPM = 300
    CALIBRATE_SAMPLES = 5
    CALIBRATE_SETTLE_S = 1.0
    CALIBRATE_TIMEOUT_S = 20.0

    # 兼容旧版纯视觉 ASCII 输出；开启云台时默认关闭，避免同一串口混发两种协议。
    # ⚠ 本组只在 ENABLE_GIMBAL=False 的降级路径使用；ENABLE_GIMBAL=True 时电机走
    #    YAW_DEV/PITCH_DEV 两条串口，本条的 /dev/ttyS0 是系统日志口，切勿改成实际接线。
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
