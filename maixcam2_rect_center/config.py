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
    # ---- 双 ZDT 云台链路 -------------------------------------------------
    # 轴 1 = yaw，轴 2 = pitch。首次上板请先把云台架空，并确认驱动器已设为
    # UART_FUN / 115200 / Checksum=固定 0x6B。
    #
    # ★ MaixCAM2 排针丝印 ↔ 引脚 ↔ 设备（2026-10-09 核对官方 UART 表 + 引脚图 v1.0）：
    #   串口   丝印      引脚        设备路径        说明
    #   UART0  U0T/U0R   A28 / A29   /dev/ttyS0     系统终端/日志口。官方**不推荐外接**
    #                                                （开机打印日志、TX 被拉低会导致不启动）；
    #                                                但它是本板唯一"默认就是串口、免 pinmap"的口。
    #   UART1  U1T/U1R   A30 / A31   /dev/ttyS1     官方 UART 表标"无引脚"，引脚图 v1.0 标
    #                                                A30/A31 可作 UART1_TX/RX（与 SPI2_CS3、
    #                                                PWM7 复用）；要用必须先 pinmap。
    #   UART2  U2T/U2R   B0  / B1    /dev/ttyS2     专用脚，需 pinmap 切到 UART2。
    #   UART3  U3T/U3R   B2  / B3    /dev/ttyS3     B2=UART3_TX / B3=UART3_RX（与 SPI2_MOSI/
    #                                                MISO 复用）。
    #   UART4  U4T/U4R   A21 / A22   /dev/ttyS4     底部 6P 1.25mm 扩展座默认串口，备用。
    # ⚠ 三路 GND 必须共地；TX↔RX 交叉连接。
    # ⚠ UART0 上可能混有内核日志/终端回显 —— 已在 stepper_link 里做开机清空 RX，
    #   若现场出现偶发乱动或丢帧，优先改用 UART1(A30/A31) 或 UART3(B2/B3)。
    ENABLE_GIMBAL = True
    # ★★ 2026-10-09 实机实测：**两台电机的接线与预期相反** ——
    #   U0(`/dev/ttyS0`, 驱动器 ID=1) 上挂的是 **pitch** 轴；U2(`/dev/ttyS2`, ID=2) 上挂的是 **yaw** 轴。
    #   证据：自检 4 步发出的命令是 "pitch, pitch, yaw, yaw"，而实际动作是 "右→左→下→上"
    #   （右/左 = yaw，下/上 = pitch）。故这里把 yaw/pitch 的**端口 + 引脚 + 地址整体互换**，
    #   让代码里的 "yaw" 真的驱动 yaw 轴。
    #   ⚠ 等效的另一种做法：把两根串口线对调，并把驱动器菜单 ID_Addr 设回 yaw=1 / pitch=2，
    #     那时把下面两组改回来即可 —— 二者只需一处对上。
    #   ⚠ U0 是系统日志口（有内核日志噪声），现在承载 pitch；若想避开，请改用上面的"对调接线"方案。
    # 代码里的 yaw 轴 → UART2(U2T/U2R)：B0/B1 需 pinmap 切到 UART2 功能。
    YAW_DEV, YAW_BAUD = "/dev/ttyS2", 115200
    YAW_PINS = {"B0": "UART2_TX", "B1": "UART2_RX"}
    # pinmap 失败是否致命：False(默认) = 只警告、**仍打开 UART**（UART0 这类可能默认已复用的口，
    # 不该因功能名不匹配而整条轴不可用）；True = 失败即不打开该轴。
    PINMAP_STRICT = False
    # 代码里的 pitch 轴 → UART0(U0T/U0R)：**必须显式 pinmap**（实测走空表会静默不通）。
    PITCH_DEV, PITCH_BAUD = "/dev/ttyS0", 115200
    PITCH_PINS = {"A28": "UART0_TX", "A29": "UART0_RX"}
    # 开机先丢弃该轴串口缓冲区里的陈旧字节（U0 上有内核启动日志/终端回显，必须清掉）。
    FLUSH_RX_ON_START = True
    # 地址跟着**电机**走（不是跟着端口）：yaw 电机挂在 U2 上、ID=2；pitch 电机挂在 U0 上、ID=1。
    ADDR_YAW, ADDR_PITCH = 2, 1
    # 闭环方向符号：`ccw = (误差 × DIR_SIGN > 0)`，其中 误差 = 靶心 − 光斑。
    # ★ 未做标定时按几何推断（相机与激光同装在云台上，云台转过去时靶心在画面里**反向**移动）：
    #   yaw 右转 -> 靶心左移(u↓) -> 要消掉 dx>0 必须右转 -> 发 CW -> sign = -1；
    #   pitch 下转 -> 靶心上移(v↓) -> 要消掉 dy>0 必须下转 -> 发 CW -> sign = -1。
    # ⚠ 自检后刚开始追踪时请盯一下：若**越追越偏**（误差持续变大、云台一路跑向一侧），
    #   说明推断反了 —— 把对应那项改成 +1（`DIVERGE_GUARD` 也会在这种情况下自动停机保护）。
    DIR_SIGN_YAW = -1
    DIR_SIGN_PITCH = -1
    # ==================== ★ 追踪手感（现场只改这一段）====================
    # 换算链：像素误差 -> 旋转角 -> 电机脉冲
    #     脉冲 = |误差px| × DEG_PER_PX_x × PULSES_PER_DEG_x × GAIN_x（再受 MAX_STEP_PULSES 限幅）
    # ① 图像每 1px 偏差对应云台转多少度。**越大跟得越快，也越容易冲过头（超调）**。
    #    想更柔和就调小（先动这个）；0 = 不启用宏，回退到下面的 PULSE_PER_PX_*。
    DEG_PER_PX_YAW = 0.30
    DEG_PER_PX_PITCH = 0.30
    # ② 每度对应多少电机脉冲 = 3200(脉冲/圈) × 减速比 / 360。
    #    无减速机构时 = 8.89；装了 i:1 减速就填 8.89 × i。
    PULSES_PER_DEG_YAW = 8.89
    PULSES_PER_DEG_PITCH = 8.89
    # ③ 单帧只修正误差的这个比例（必须 ≤1，否则增量闭环不收敛；0.2~0.4 是稳妥区）。
    #    ⚠ 2026-10-10 由 0.5 降到 0.3 —— 现场反馈"调节过于激烈、旋转角度过大、超调"。
    GAIN_YAW = 0.3
    GAIN_PITCH = 0.3
    # ④ 转速与曲线：慢 + 带曲线，让单帧修正真正走完再发下一帧，避免指令堆叠成"冲过头"。
    #    acc=0 会让位置模式**瞬时启停**（急停瞬间激光会晃开），故用 ACC_TRACK。
    RPM_FINE = 60
    RPM_COARSE = 150
    ACC_TRACK = 200        # 非 0 = 走曲线加减速；acc 越小越柔
    ACC_DEFAULT = 0        # 仅其它机会性发帧用；闭环追踪请用 ACC_TRACK
    # ⑤ 死区与单帧限幅：死区应 ≥1.5×实测回差（齿隙里来回抖）；限幅防单帧猛甩。
    DEADBAND_PX = 3.0
    MAX_STEP_PULSES = 150
    COMMAND_INTERVAL_S = 0.10
    # 兼容旧口径 / 标定输出：**仅当上面 DEG_PER_PX_x = 0 时**才回退到这里。
    # CALIBRATE_AXIS 实测完会把脉冲/像素写回这两个（可在日志里反算 DEG_PER_PX 填回 ①）。
    PULSE_PER_PX_YAW = 0.0
    PULSE_PER_PX_PITCH = 0.0

    # 上电自检：度数对应的脉冲数必须按实际减速机构标定。
    # 0 表示未标定，程序跳过自动扫动，避免用错误比例撞限位。
    GIMBAL_SELF_TEST = True
    SELF_TEST_YAW_DEG = 15.0
    SELF_TEST_PITCH_DEG = 30.0
    # 计算参考：3200 pulses/rev * 减速比 / 360；减速比未知时不要猜。
    YAW_PULSES_PER_DEG = 0.0
    PITCH_PULSES_PER_DEG = 0.0
    # 以"yaw 向左 / pitch 仰起"为正方向。
    # ★ 2026-10-09 实机实测：两轴的**正方向都是 CCW** ——
    #   yaw 驱动器 CCW=左 / CW=右；pitch 驱动器 CCW=上(仰) / CW=下。
    #   自检序列因此是 pitch 上→下、yaw 左→右（各回到起始位姿）。
    YAW_POSITIVE_CCW = True
    PITCH_POSITIVE_CCW = True
    # ★ 自检必须"慢 + 带曲线"。位置模式若 acc=0，驱动器会**直接以设定速度启停**，
    #   实机表现就是明显卡顿/冲击（现场反馈）。ZDT 官方：acc≠0 时每 (256-acc)*50us 变化 1 RPM，
    #   即 **acc 越小越柔和**（3D 打印常用 20-80）；acc=0 才是"不用曲线"。
    # 自检动作：慢 + 带曲线（acc≠0）。rpm/acc 决定每步快慢，settle 是步间停顿。
    SELF_TEST_RPM = 60
    SELF_TEST_ACC = 60
    SELF_TEST_SETTLE_S = 0.3
    # 上电后等多久才开始自检（秒）：给驱动器留出处理使能帧的时间，避免使能与运动帧贴太近被丢。
    SELF_TEST_START_DELAY_S = 0.3
    # 自检**防死锁**超时（秒）：自检本身不依赖视觉，正常几秒就跑完；这里只兜底"某步迟迟不返回"
    #   之类的异常（到点强制结束自检、继续往下走）。**不是性能目标**，够宽即可；0 = 关闭。
    SELF_TEST_TIMEOUT_S = 10.0
    # 自检幅度（固定脉冲量）：上电自检**不依赖视觉（不需要靶子）、也不依赖标定**，
    #   直接用这个脉冲量走一遍：**pitch 先上转再下转，yaw 先左转再右转**（各回到起始位姿）。
    #   400 脉冲 = 电机轴 45°（3200 脉冲/圈）；有减速比时输出轴更小。
    #   ⚠ 首次务必架空云台确认行程够用；置 0 则回退到"按 YAW/PITCH_PULSES_PER_DEG 换算角度"的老路径。
    SELF_TEST_PULSES = 400
    MOTOR_PULSES_PER_REV = 3200
    # 减速比未知时留 0：自检角度换算优先用上面的实测 PULSES_PER_DEG，不猜齿轮比。
    # 若已知 i = 大齿轮齿数/小齿轮齿数，填进来即可让自检在未做脉冲/度标定时也生效。
    GIMBAL_GEAR_RATIO = 0.0

    # ---- 闭环口径（需求文档 §5.2）-----------------------------------------
    # True = 用未平滑的 raw_u/raw_v 算 dx/dy（推荐）：CENTER_ALPHA 只留给显示，
    # 闭环用平滑值会附加 ≈28ms 滞后，底盘运动时直接变成瞄准误差。
    # 切换坐标系后 PULSE_PER_PX_* 必须在同一套坐标下重新标定。
    AIM_USE_RAW = True
    # 安全闸：未完成开环标定时禁止发电机指令（PULSE_PER_PX_*/DIR_SIGN_* 还是占位值时
    # 直接闭环可能正反馈甩头）。标定完把 GIMBAL_CALIBRATED 置 True 解锁。
    AIM_REQUIRE_CALIBRATION = True
    # ★ 当前 True：换算参数由**手动宏**（DEG_PER_PX_* × PULSES_PER_DEG_*）给出，视为"已就绪"，
    #   故上电自检一跑完就**直接解锁闭环进入追踪**，不需要先标定。
    #   若改回"必须标定后才追"，把这里置 False（未标定会锁在 UNCALIBRATED、不发任何指令）。
    GIMBAL_CALIBRATED = True
    # ★ 发散看护：持续发指令却始终不收敛（误差还在变大）= 方向反了 -> 自动 FAULT 停机，
    #   避免云台一路冲到底。窗口内 |误差| 的最小值被放大 DIVERGE_RATIO 倍即判定。
    DIVERGE_GUARD = True
    DIVERGE_FRAMES = 30
    DIVERGE_RATIO = 1.5
    # 离群拒收：|误差| 超过该值先当误检丢弃，连续 JUMP_CONFIRM_FRAMES 帧才采信。
    # 防止单帧误检（反光/干扰）给出 dx=2000px 而甩头。
    JUMP_REJECT_PX = 400.0
    JUMP_CONFIRM_FRAMES = 3
    # 粗/精两段增益：|e| > COARSE_PX 用 GAIN_COARSE（大步快速逼近），否则用 GAIN_YAW/PITCH。
    # 必须保持 gain <= 1，否则增量闭环不收敛（需求文档 §7.4）。
    # ⚠ 2026-10-10 由 0.9 降到 0.5 —— 大误差时同样要避免冲过头。
    # （转速 RPM_COARSE / RPM_FINE 已统一到上方「★ 追踪手感」④）
    COARSE_PX = 24.0
    GAIN_COARSE = 0.5

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

    # ---- 开环标定（闭环能不能收敛全看这里）--------------------------------
    # ""      : 不标定，直接用本文件里的 PULSE_PER_PX_* / DIR_SIGN_*（默认占位值 3.67 / +1）。
    # "yaw"   : 开机只标该轴，实测并打印 PULSE_PER_PX / DIR_SIGN / 交叉项 / 回差；标定期不发闭环指令。
    # "pitch" : 同上，另一轴。
    # "auto"  : ★ 依次自动标定两轴，**标完自动写回运行时 config 并解锁闭环进入跟踪** ——
    #           不需要人工读数回填、不需要重启。任一轴失败则整体保持锁定（宁可不追，不用错参数甩头）。
    #           前提：开机时画面里能看到靶心（否则 dx/dy 无效 -> 超时失败）。
    # 人工标定拿到数值后，请填进本文件并把 CALIBRATE_AXIS 置回 ""。
    # ★ 当前设 ""：**不做自动标定**。
    #   原因（用户实测）：标定必须"画面里看得见靶心"，而云台一动/抬头后黑框常跑出视野，
    #   标定就卡在等 dv（视觉无效）上直到 20s 超时 —— 即"校准抬头后黑框消失导致堵塞"。
    #   现改用手动宏（DEG_PER_PX_*）直接换算，**上电自检完即进入追踪**。
    #   需要重新标定时临时改回 "auto"（或 "yaw"/"pitch" 单轴），标完再置回 ""。
    CALIBRATE_AXIS = ""
    AUTO_CALIBRATE_AXES = ("yaw", "pitch")
    AUTO_CALIBRATE_TIMEOUT_S = 150.0
    # ★ 标定行程宁小勿大：脉冲给太多会把靶心推出视野 -> 测到位移无效 -> 标定失败。
    #   400 脉冲 = 电机轴 45°（3200 脉冲/圈）；只要 |Δpx| > 0.5 就能算出 PULSE_PER_PX。
    CALIBRATE_PULSES = 400
    # 标定动作同样要"慢 + 带曲线"，否则同样卡顿（机理见 SELF_TEST_ACC 处的注释）。
    CALIBRATE_RPM = 60
    CALIBRATE_ACC = 60
    CALIBRATE_SAMPLES = 5
    CALIBRATE_SETTLE_S = 1.0
    CALIBRATE_TIMEOUT_S = 20.0
    # ★ 视觉无效**快速失败**（秒）：标定每一步都要 dx/dy，画面里没有靶心时**不该干等 20s**
    #   （现场"抬头后黑框出视野 -> 卡住不动"就是这么来的）。超过该时长没有任何有效视觉
    #   就直接判失败、立刻交还给主流程（接着自检/追踪），不再阻塞。0 = 关闭。
    CALIBRATE_VISUAL_TIMEOUT_S = 3.0

    # 兼容旧版纯视觉 ASCII 输出；开启云台时默认关闭，避免同一串口混发两种协议。
    # ⚠ 本组只在 ENABLE_GIMBAL=False 的降级路径使用；ENABLE_GIMBAL=True 时电机走
    #    YAW_DEV/PITCH_DEV 两条串口（当前 pitch 用的就是 /dev/ttyS0，但那是给电机的，
    #    不要在这里再开一次 —— 同一端口开两个 UART 对象会互相抢字节）。
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
