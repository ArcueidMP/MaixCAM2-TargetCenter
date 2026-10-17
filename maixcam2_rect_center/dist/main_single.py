#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main_single.py · 25E 靶心识别（轨道 B）—— 单文件自包含版
================================================================================
★ 本文件由 tools/build_single.py 自动生成，请勿手改！
    改参数 -> config.py ；改算法 -> vision/*.py ；改流程 -> main.py
    改完在本目录重新执行：  python tools/build_single.py

用途：在 MaixVision 里点「运行当前文件」即可运行（不必用「运行项目」）。
      行为与模块化工程完全一致，输出也就完全一致。

生成时间：2026-10-17 13:30:11
    config.py                    sha1=9f3eae32a114
    main.py                      sha1=2fbfd1a8523a
    target_api.py                sha1=17f47cf48d5f
    vision/__init__.py           sha1=abc847cb4264
    vision/aim_control.py        sha1=6dab67c57421
    vision/app_runtime.py        sha1=323bd7d8b015
    vision/camera_source.py      sha1=b398fedefe6e
    vision/contour_locator.py    sha1=a314d762d719
    vision/gimbal_aim.py         sha1=543b2d46a1e8
    vision/gimbal_calibrate.py   sha1=c7db6a254e40
    vision/gimbal_search.py      sha1=fc5c09964c19
    vision/gimbal_self_test.py   sha1=1094ca64bdc0
    vision/imu_heading.py        sha1=a1193ada2173
    vision/kalman_filter.py      sha1=9104ffbec3f7
    vision/laser_spot.py         sha1=c131498c5ff9
    vision/maixvision_stream.py  sha1=7150d30f9d67
    vision/preview.py            sha1=3f9d923a5e12
    vision/rate_meter.py         sha1=6f2a054c97a0
    vision/rect_geometry.py      sha1=183fd26c6666
    vision/rectangle_center.py   sha1=912eded4bbf8
    vision/stepper_link.py       sha1=ac6c57cb0a25
    vision/telemetry.py          sha1=9a047c2372a2
    vision/vision_worker.py      sha1=f5133a4e390d
"""

import sys as _sys
import types as _types

_SRC_SHA1 = {'config.py': '9f3eae32a114b60f3b1c8eb8d40c7f30573a6abe', 'vision/rate_meter.py': '6f2a054c97a00470409a59de9dea42b0bff9c6ba', 'vision/camera_source.py': 'b398fedefe6eaf931d372abe00ba267f7d9697b5', 'vision/vision_worker.py': 'f5133a4e390d6f73723ad673daf40f2d9b82bcbd', 'vision/telemetry.py': '9a047c2372a2ce7d2e91928574310095395144aa', 'vision/maixvision_stream.py': '7150d30f9d679557371ca952563dd27317035720', 'vision/preview.py': '3f9d923a5e122f2af740edd488a73f3fcd0d5473', 'vision/rect_geometry.py': '183fd26c666668c9da135d346f5ac0156c6a2c14', 'vision/contour_locator.py': 'a314d762d7192a3a5beeb33a6e7b6dc004f3d7da', 'vision/kalman_filter.py': '9104ffbec3f72996407be9caca85324abeab19e8', 'vision/rectangle_center.py': '912eded4bbf8200120c38b5e16022725e34b6415', 'vision/laser_spot.py': 'c131498c5ff980e594b1c8d5d2feda6195caee23', 'vision/stepper_link.py': 'ac6c57cb0a258d7b60a87d036ae6c744b76133be', 'vision/imu_heading.py': 'a1193ada2173f113452781bcebc5a5e5b8745db3', 'vision/gimbal_self_test.py': '1094ca64bdc09f087f659b197c1085767a050952', 'vision/gimbal_aim.py': '543b2d46a1e86c861e0ff4c7f4e481950b8a8242', 'vision/gimbal_search.py': 'fc5c09964c192bd9c68e7e837dc3dc37b9bc4579', 'vision/aim_control.py': '6dab67c574216816d723822c82d4a37e4e7fb881', 'vision/gimbal_calibrate.py': 'c7db6a254e404cadea5c1ddd892afe7ae8834aa3', 'vision/app_runtime.py': '323bd7d8b01531de43e3cae2efe5cf3a53344f34', 'vision/__init__.py': 'abc847cb4264bf79d47c0dae40f584bf4baba64f', 'target_api.py': '17f47cf48d5f4647d3bed09c7bf4d9451e7feee1', 'main.py': '2fbfd1a8523a5d0527a17b41e0d2c3322de54012'}

# --------------------------------------------------------------------------
# ① 各模块源码原样嵌入，运行时装进 sys.modules（保持原 import 语义）
# --------------------------------------------------------------------------
_MOD_SRC = {}

# ===== config  (config.py) =====
_MOD_SRC['config'] = r'''
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
    # 本地 Display 构造时临时 MAIXVISION=0，完成后恢复会话。
    # False: 独立进程连接本机 MaixVision 桥；True: 只用本地屏。
    # JPEG/网络不进入原生 show()，预览断连不会触发 500ms 忙等。
    DISABLE_MAIXVISION_STREAM = False
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
    # Canny 一张掩膜；未找到完整四边时才补 Otsu 一张。所有轮廓在低分辨率处理。
    SEARCH_WIDTH = 320
    SEARCH_SPLIT_FRAMES = False  # 历史兼容项；新搜索始终在当前帧完成。
    SEARCH_MAX_PROPOSALS = 128   # 仅 SEARCH_EXHAUSTIVE=False 时生效；ROI 不阻断全图重搜。
    MAX_CANDIDATES = 8
    EDGE_SAMPLES = 24
    REFINE_RADIUS = 6
    TRACK_RADIUS = 12
    # 运动预测（治"速度快捕获不到"）：窄带精修的搜索半径 TRACK_RADIUS 保持小值（定位精度高），
    # 靠"外推本帧位置"来跟上快速移动：用上两帧四角的**逐角位移**（含平移/缩放/旋转）预测本帧
    # 四角，再在预测位置附近做窄带精修；预测失准时自动回落到上一帧位置。
    # PREDICT_MAX_PX 是单帧预测位移上限，防噪点把预测甩飞（板端帧率低、目标位移可能很大时才需调大）。
    ENABLE_MOTION_PREDICT = True
    PREDICT_MAX_PX = 60.0
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

    # v0.8: bounded Canny/Otsu search; exhaustive verification remains enabled.
    EDGE_CLOSE_PX = 3
    SEARCH_MAX_CONTOURS = 16
    SEARCH_EXHAUSTIVE = True
    # 仅供旧工具读取，新检测器不使用以下旧搜索参数。
    DARK_THRESHOLDS = (25, 40, 55, 70, 85, 100, 120, 145)
    SEARCH_BUDGET_MS = 25.0
    MAX_CONTOURS = 128
    ENABLE_LINE_RECOVERY = False
    HULL_FILL_RATIO = 0.60
    MIN_EDGE_LENGTH_PX = 12.0
    STRONG_EDGE_COVERAGE = 0.85
    REFINE_ITERATIONS = 2
    REFINE_RADIUS_WIDE = 20
    RECOVER_MIN_CONTRAST = 80
    RECOVER_MAX_DARK = 100
    PREDICT_MAX_GAP_S = 0.25
    KALMAN_ENABLE = True
    KALMAN_PROCESS_PX = 40.0
    KALMAN_MEASURE_PX = 0.9
    KALMAN_RESET_PX = 25.0
    KALMAN_MAX_PREDICT_S = 0.25
    KALMAN_VELOCITY_LIMIT_PX_S = 6000
    FRONT_VIEW_SHORT_PX = 0
    FRONT_VIEW_MAX_ERROR_PX = 5.0

    # Relative commands need both motion completion AND a newer captured frame.
    AIM_PROFILE = "smooth"
    AIM_PROGRESSIVE = True
    ACC_TRACK = 80
    AIM_WAIT_MOTION = True
    AIM_MOTION_MARGIN_S = 0.03
    AIM_HISTORY_MAX_GAP_S = 0.30
    DIVERGE_GUARD = True
    DIVERGE_CONFIRM_FRAMES = 6
    DIVERGE_RATIO = 1.15
    DIVERGE_MIN_PX = 4.0
    GIMBAL_CALIBRATED = True
    CALIBRATE_VISUAL_TIMEOUT_S = 3.0
    SELF_TEST_TIMEOUT_S = 10.0
    ENABLE_SEARCH_BUTTON = True
    SEARCH_YAW_DEG = SEARCH_PITCH_DEG = 45.0
    SEARCH_RPM = SEARCH_ACC = 60
    SEARCH_SETTLE_S = 0.10
    SEARCH_TIMEOUT_S = 60.0
    SEARCH_PULSES = 400
    SEARCH_USE_IMU = True
    SEARCH_IMU_MARGIN_DEG = 3.0

    ENABLE_IMU_FEEDFORWARD = False
    IMU_YAW_AXIS = (0, 0, 1)
    IMU_PITCH_AXIS = (0, -1, 0)
    IMU_MAX_GAP_S = 0.20
    IMU_MAX_AGE_S = 0.20
    IMU_SAMPLE_HZ = 208
    IMU_MOUNT = "gimbal"
    IMU_BIAS_CALIBRATE_S = 1.0
    IMU_BIAS_MAX_STD_DPS = 0.25
    IMU_BIAS_MAX_DPS = 2.0
    IMU_HOLD_MAX_S = 0.25
    SHOW_STREAM_HOST = "127.0.0.1"
    SHOW_STREAM_PORT = 7899
    SHOW_STREAM_WIDTH = 480
    SHOW_STREAM_SLOW_MS = 300
    SHOW_STREAM_RETRY_S = 2.0
    SHOW_STREAM_RETRY_MAX_S = 15.0


def apply_aim_profile(cfg):
    """Only these six values belong to a profile; unknown names fail at startup."""
    profiles = {"smooth": (.30, .30, .65, 60, 150, 80),
                "responsive": (.50, .50, .90, 120, 400, 360)}
    name = cfg.AIM_PROFILE
    if name not in profiles:
        raise ValueError("unknown AIM_PROFILE: %s" % name)
    for key, value in zip(("GAIN_YAW", "GAIN_PITCH", "GAIN_COARSE", "RPM_FINE", "RPM_COARSE", "MAX_STEP_PULSES"), profiles[name]):
        setattr(cfg, key, value)
    return cfg


apply_aim_profile(Config)
'''

# ===== vision.rate_meter  (vision/rate_meter.py) =====
_MOD_SRC['vision.rate_meter'] = r'''
"""滑动窗口帧率计（从 main.py 原样搬迁，v0.4.6 模块化）。

本模块无第三方依赖、无 I/O，可在 PC 上单独单测（tools/test_runtime_performance.py 的 test_real_fps）。
"""
from collections import deque

import numpy as np


class FrameRate:
    """滑动窗口帧率计。按"成功完成的事件时刻"计时；返回 (平均fps, low1)。

    本类被三处复用（各自独立实例，口径不同、切勿混读）：
        capture_rate —— 主循环 cam.read() 速率   → [perf] capture=
        preview.rate —— 显示 show() 完成速率      → [perf] disp= / 屏幕文字 disp
        worker.rate  —— 识别帧处理完成速率        → [perf] detect_fps=

    low1 = "最慢 1% 帧间隔的倒数"：用于暴露**卡顿**——平均值好看但偶发长停顿（如 JPEG 图传
    抖动）时，fps 可能仍高，而 low1 会被那几帧长间隔拉低，是比平均值更诚实的体感指标。
    """

    def __init__(self):
        self.total = 0
        self._times = deque(maxlen=120)   # 只留最近 120 个时刻，足够覆盖 2s 窗口

    def record(self, now):
        self.total += 1
        self._times.append(float(now))

    def values(self, now):
        ts = list(self._times)
        if len(ts) < 2:
            return 0.0, 0.0
        # 只看最近两秒内的时刻：统计窗口滚动，旧的读数随时间长自然淡出。
        # 关键：把"距上一个事件已过去的时间(stall)"也算作一次间隔 —— 否则一旦卡住
        # （长时间没有新事件），窗口里只剩早先的高速样本，会把上次的快读数永久挂在屏幕上。
        recent = [t for t in ts if t >= float(now) - 2.0]
        if len(recent) < 2:
            return 0.0, 0.0
        gaps = np.diff(recent)
        stall = float(now) - recent[-1]
        if stall > gaps.max():
            gaps = np.append(gaps, stall)          # 把"当前这段停顿"并入间隔序列
        fps = (len(recent) - 1) / max(1e-6, float(now) - recent[0])   # 平均帧率
        slow = np.sort(gaps)[-max(1, int(np.ceil(len(gaps) * 0.01))):]  # 最慢的 1% 间隔
        return fps, 1.0 / max(1e-6, float(np.mean(slow)))              # low1
'''

# ===== vision.camera_source  (vision/camera_source.py) =====
_MOD_SRC['vision.camera_source'] = r'''
"""相机取帧辅助：MaixPy 帧 → 灰度 ndarray（从 main.py 原样搬迁，v0.4.6 模块化）。

`to_gray` 与 maix 运行时无关（只依赖 numpy / cv2 与调用方传入的 image 模块），
因此可在 PC 上以假 image 模块单测。相机对象的创建仍留在入口 main()（见 AGENTS.md 装配约定）。
"""
import cv2


def to_gray(img, image_module):
    """MaixPy 相机帧 → cv2 灰度 ndarray（H×W，uint8）。

    优先走 `image2cv(..., copy=False)` **零拷贝**借用相机缓冲（不额外占内存、最快），
    因此返回的数组在下一次 cam.read() 前必须用完；跨线程传递前务必先 copy（见 LatestFrameWorker.submit）。
    按通道数兜底：单通道直接取、3 通道按 RGB2GRAY、4 通道按 RGBA2GRAY；
    都不能识别时退回 copy=True 的 BGR 再转灰度。
    """
    a = image_module.image2cv(img, ensure_bgr=False, copy=False)
    if a.ndim == 2:
        return a
    if a.ndim == 3 and a.shape[2] == 1:
        return a[:, :, 0]
    if a.ndim == 3 and a.shape[2] == 3:
        return cv2.cvtColor(a, cv2.COLOR_RGB2GRAY)
    if a.ndim == 3 and a.shape[2] == 4:
        return cv2.cvtColor(a, cv2.COLOR_RGBA2GRAY)
    b = image_module.image2cv(img, ensure_bgr=True, copy=True)
    if b.ndim == 2:
        return b
    return cv2.cvtColor(b, cv2.COLOR_BGR2GRAY)
'''

# ===== vision.vision_worker  (vision/vision_worker.py) =====
_MOD_SRC['vision.vision_worker'] = r'''
"""识别工作线程 + 最新帧槽位（从 main.py 原样搬迁，v0.4.6 模块化）。

线程模型（时间轴 B）：主线程 submit() 最新帧 → 识别线程取走处理 → 结果写回最新槽位，
主线程 snapshot() 非阻塞取回。**最新优先、丢弃旧帧**是刻意设计（见类 docstring）。

★ clock 注入：构造可传 `clock`（默认标准库 time）。入口 main() 会把自身模块的 time
传进来，从而让 tools/test_main_self_test.py 的假时钟（patch main.time）能穿透到本模块，
使端到端用例在模拟时间轴下可复现。不传时行为与搬迁前完全一致（真 time）。
"""
import threading
import time

import numpy as np

from .rate_meter import FrameRate


class LatestFrameWorker:
    """单识别线程 + 两个"最新槽位"：一个待处理帧槽(_pending)、一个结果槽(_latest)。

    线程模型（时间轴 B 的实现，见 main.py 模块开头总纲）：
        · 主线程（生产者）：submit() 往 _pending 放**最新一帧**的副本；
        · 识别线程（消费者）：取走 _pending → callback() 处理 → 结果写 _latest；
        · 主线程（读者）：snapshot() 无锁等待地取回 _latest（可能滞后于当前摄像头帧）。

    "最新优先、丢弃旧帧"是刻意的：识别跟不上时保留最新画面才有意义，积压旧帧只会让
    追踪越追越偏。Camera / Display / 触摸这些**必须独占主线程**的资源一概不跨线程。
    """

    def __init__(self, callback, idle=None, asynchronous=True, clock=None):
        self.callback, self.idle = callback, idle
        self._clock = clock if clock is not None else time
        self._condition = threading.Condition()   # 保护 _pending / _latest / _stop
        self._pending = self._latest = None
        self._stop = False
        self.error = None
        self._snapshot_cache = (None, 0., 0.)
        self.dropped = 0                          # 因来不及处理而被覆盖的帧数（丢帧计数）
        self.rate = FrameRate()                   # 识别帧率（detect_fps 口径）
        self.thread = None
        if asynchronous:
            # daemon=True：进程退出时线程自动结束，不必显式 join 也能安全收摊。
            self.thread = threading.Thread(target=self._run, name="rect-vision", daemon=True)
            self.thread.start()

    def submit(self, gray, captured_at):
        # ★ 必须拷贝：image2cv(copy=False) 借用的相机内部缓冲会被**下一次 read 覆写**，
        #   识别线程还没处理完就可能已被改写 → 后台线程必须持有自己的一份灰度图。
        item = (np.array(gray, dtype=np.uint8, order="C", copy=True), float(captured_at))
        if self.thread is None:
            self._process(item)          # 同步模式（VISION_ASYNC=False）：主线程内直接算
            return
        with self._condition:
            if self._stop:
                return
            if self._pending is not None:
                self.dropped += 1        # 上一帧还没被取走就被覆盖 → 记一次丢帧
            self._pending = item
            self._condition.notify()     # 唤醒识别线程

    def _process(self, item):
        snapshot = self.callback(*item)  # callback = VisionControl.process（识别+偏差+闭环）
        with self._condition:
            self._latest = snapshot
            self.rate.record(self._clock.perf_counter())

    def _run(self):
        """识别线程主体：循环取最新帧处理；无帧时走 idle() 维持串口保活。"""
        try:
            while True:
                with self._condition:
                    if self._pending is None and not self._stop:
                        # 20ms 超时轮询（而非无限 wait），保证 stop 能被及时看到。
                        self._condition.wait(timeout=0.02)
                    if self._stop:
                        return
                    item, self._pending = self._pending, None
                if item is not None:
                    self._process(item)
                elif self.idle is not None:
                    # 有帧就处理、没帧就保活：空闲时继续收 ACK/推进按钮自检，但不重发旧误差。
                    self.idle()
        except Exception as exc:
            # 识别线程异常不能静默：记下原因，主循环下一轮会读到 error 并抛出终止。
            with self._condition:
                self.error = repr(exc)
                self._latest = None
            print("[vision] worker failed: %s" % exc)

    def snapshot(self, now):
        """取回最近一次识别结果 + 识别帧率（主线程调用，非阻塞）。"""
        if not self._condition.acquire(False):
            return self._snapshot_cache
        try:
            fps, low1 = self.rate.values(now)
            self._snapshot_cache = self._latest, fps, low1
            return self._snapshot_cache
        finally:
            self._condition.release()

    def close(self):
        """停线程；必须等它真正退出——UART 归识别线程独占，未退干净就动硬件会打架。"""
        with self._condition:
            self._stop = True
            self._pending = None
            self._condition.notify()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
            if self.thread.is_alive():
                # 线程没停就动 UART/相机是竞态；宁可抛错让上层看见，也不静默降级。
                raise RuntimeError("vision worker did not stop; UART remains owned by worker")
'''

# ===== vision.telemetry  (vision/telemetry.py) =====
_MOD_SRC['vision.telemetry'] = r'''
"""周期日志组装（从 main.py 抽出，v0.4.6 模块化）。

只做字符串拼装，不碰任何硬件/时间源 —— 便于单独审阅"日志口径"是否与代码一致。
★ 日志文案逐字冻结（tools/test_main_self_test.test_perf_log_branch_formatting 会校验字段），
  改这里等于改板端日志格式，务必同步 README 的读法说明。
"""
# 主循环单轮超过该毫秒数（不含 show）即判为"慢"，打一次归因提示。
MAIN_LOOP_SLOW_MS = 66.0


def format_perf_line(build_id, capture_fps, display_fps, display_low1, vision_fps,
                     result, read_avg, draw_avg, loop_avg,
                     show_avg, show_p95, show_max, show_n,
                     age_s, shown, drop_show, drop_vision, detector_result=None):
    """拼装 [perf] 行。

    读法：[perf] 里 capture / disp / detect_fps 是三路**不同**口径 ——
      "眼睛看到的刷新率"看 disp，"识别吞吐"看 detect_fps，别互相换算。
      read/draw/loop 属于**主循环**，show 属于**显示线程** —— 分开看就能一眼判定
      "到底谁在吃时间"（这正是 1.5fps 归因的关键）。
    """
    det = result if detector_result is None else detector_result
    return ("[perf] %s capture=%.1f disp=%.1f low1=%.1f detect_fps=%.1f "
            "valid=%s stage=%s | read=%.1f draw=%.1f loop=%.1f ms | "
            "show avg=%.1f p95=%.1f max=%.1fms n=%d | "
            "age=%.0fms shown=%d drop(show)=%d drop(vision)=%d" %
            (build_id, capture_fps, display_fps, display_low1, vision_fps,
             result.get("valid", False), result.get("stage"),
             read_avg, draw_avg, loop_avg,
             show_avg, show_p95, show_max, show_n,
             age_s * 1000, shown, drop_show, drop_vision)) + (
            " det_valid=%s det_stage=%s det_ms=%.1f det_masks=%d" %
            (det.get("valid", False), det.get("stage"), det.get("detector_ms", 0.), det.get("masks", 0)))


def format_slow_loop(loop_avg):
    """主循环偏慢（且此数**不含 show**）时的归因提示。"""
    return ("[preview] main loop slow: loop=%.0fms（不含 show）；看 read= 是否偏大；"
            "若 show 也恒定 ~505ms 则是图传忙等抢 CPU" % loop_avg)
'''

# ===== vision.maixvision_stream  (vision/maixvision_stream.py) =====
_MOD_SRC['vision.maixvision_stream'] = r'''
"""MaixVision protocol-0 CLIENT; JPEG/WebSocket run in a fresh process, no maix.

Protocol source: Sipeed/MaixCDK components/vision/src/maix_image_trans.cpp.
The parent only copies into one mmap slot under a nonblocking flock. The child
reopens the inherited fd: flock on a shared open-file-description cannot exclude
its parent, even though it is in another process.
"""
import base64
import hashlib
import mmap
import os
import select
import socket
import struct
import subprocess
import sys
import tempfile
import time
try:
    import fcntl
except ImportError:  # Windows PC tools can import the application without POSIX I/O.
    fcntl = None

HEADER = 128
MAGIC = b'\xac\xbe\xcb\xca'
MAX_MESSAGE = 4*1024*1024


def packet(message, payload=b''):
    frame = MAGIC+struct.pack('<I', len(payload)+3)+bytes((0, message))+payload
    return frame+bytes((sum(frame)&255,))


def unpack_packet(frame):
    if len(frame) < 11 or frame[:4] != MAGIC or struct.unpack_from('<I', frame, 4)[0] != len(frame)-8 or frame[8] != 0 or sum(frame[:-1])&255 != frame[-1]:
        raise ValueError('invalid protocol-0 packet')
    return frame[9], frame[10:-1]


class WebSocketClient:
    def __init__(self, host, port):
        self.socket = socket.create_connection((host, port), timeout=.5)
        self.buffer = bytearray()
        self.fragments = bytearray()
        self.fragment_opcode = None
        try:
            key = base64.b64encode(os.urandom(16)).decode('ascii')
            request = ('GET / HTTP/1.1\r\nHost: %s:%d\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n' % (host, port, key))
            self.socket.sendall(request.encode('ascii'))
            while b'\r\n\r\n' not in self.buffer:
                data = self.socket.recv(4096)
                if not data or len(self.buffer) > 16384:
                    raise OSError('invalid WebSocket handshake')
                self.buffer.extend(data)
            headers, rest = bytes(self.buffer).split(b'\r\n\r\n', 1)
            self.buffer = bytearray(rest)
            lines = headers.decode('ascii').split('\r\n')
            fields = {line.split(':',1)[0].lower(): line.split(':',1)[1] for line in lines[1:] if ':' in line}
            expected = base64.b64encode(hashlib.sha1((key+'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()).decode()
            if ' 101 ' not in lines[0] or fields.get('sec-websocket-accept', '').strip() != expected:
                raise OSError('WebSocket upgrade rejected')
        except BaseException:
            self.close()
            raise

    def send(self, payload, opcode=2):
        mask = os.urandom(4)
        size = len(payload)
        head = bytes((0x80|opcode, 0x80|size)) if size < 126 else (bytes((0x80|opcode, 0x80|126))+struct.pack('>H', size) if size < 65536 else bytes((0x80|opcode, 0x80|127))+struct.pack('>Q', size))
        encoded = bytearray(payload)
        for i in range(4):
            encoded[i::4] = bytes(v^mask[i] for v in encoded[i::4])
        self.socket.sendall(head+mask+encoded)

    def receive(self, timeout=.01):
        if select.select([self.socket], [], [], timeout)[0]:
            data = self.socket.recv(65536)
            if not data:
                raise OSError('WebSocket closed')
            self.buffer.extend(data)
        messages = []
        while len(self.buffer) >= 2:
            b0, b1 = self.buffer[:2]
            opcode, size, offset = b0&15, b1&127, 2
            if b0 & 0x70 or b1 & 0x80:
                raise ValueError('unsupported WebSocket frame')
            if size in (126, 127):
                count = 2 if size == 126 else 8
                if len(self.buffer) < offset+count:
                    break
                size = int.from_bytes(self.buffer[offset:offset+count], 'big')
                offset += count
            if size > MAX_MESSAGE:
                raise ValueError('WebSocket message too large')
            if len(self.buffer) < offset+size:
                break
            payload = bytes(self.buffer[offset:offset+size])
            del self.buffer[:offset+size]
            if opcode == 8:
                raise OSError('WebSocket close')
            if opcode == 9:
                self.send(payload, 10)
                continue
            if opcode == 10:
                continue
            if opcode in (1, 2):
                if self.fragment_opcode is not None:
                    raise ValueError('interleaved WebSocket message')
                self.fragment_opcode = opcode
            elif opcode != 0 or self.fragment_opcode is None:
                raise ValueError('invalid continuation')
            self.fragments.extend(payload)
            if len(self.fragments) > MAX_MESSAGE:
                raise ValueError('WebSocket fragments too large')
            if b0 & 0x80:
                if self.fragment_opcode == 2:
                    messages.append(bytes(self.fragments))
                self.fragments.clear()
                self.fragment_opcode = None
        return messages

    def close(self):
        self.socket.close()


class MaixVisionStream:
    def __init__(self, cfg, width, height, clock=None):
        self.cfg, self.width, self.height = cfg, int(width), int(height)
        self.clock = clock if clock is not None else time
        self.process = self.mapping = self.file = None
        self.sequence = self.dropped = 0
        self.enabled = False
        if fcntl is None:
            print('[stream] POSIX flock unavailable; local display only')
            return
        try:
            self.file = tempfile.TemporaryFile()
            size = HEADER+self.width*self.height*3
            self.file.truncate(size)
            self.mapping = mmap.mmap(self.file.fileno(), size)
            # __source__ is supplied by build_single for virtual modules.
            source = getattr(sys.modules.get(__name__), '__source__', None)
            if source is None:
                with open(__file__, encoding='utf-8') as handle:
                    source = handle.read()
            command = source+'\n_child(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), sys.argv[4], int(sys.argv[5]), int(sys.argv[6]), int(sys.argv[7]), float(sys.argv[8]), float(sys.argv[9]))\n'
            self.process = subprocess.Popen([sys.executable, '-c', command, str(self.file.fileno()), str(self.width), str(self.height),
                getattr(cfg, 'SHOW_STREAM_HOST', '127.0.0.1'), str(getattr(cfg, 'SHOW_STREAM_PORT', 7899)),
                str(getattr(cfg, 'SHOW_STREAM_WIDTH', 480)), str(getattr(cfg, 'SHOW_JPEG_QUALITY', 70)),
                str(getattr(cfg, 'SHOW_STREAM_RETRY_S', 2.)), str(getattr(cfg, 'SHOW_STREAM_RETRY_MAX_S', 15.))],
                pass_fds=(self.file.fileno(),))
            self.enabled = True
        except Exception as exc:
            print('[stream] disabled: %s' % exc)
            self.close()

    def submit(self, canvas):
        if not self.enabled or self.process.poll() is not None:
            return False
        if canvas.shape != (self.height, self.width, 3) or str(canvas.dtype) != 'uint8':
            raise ValueError('stream frame shape/dtype mismatch')
        try:
            fcntl.flock(self.file.fileno(), fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            self.dropped += 1
            return False
        try:
            self.sequence += 1
            self.mapping[HEADER:] = canvas.tobytes()
            struct.pack_into('<QII', self.mapping, 0, self.sequence, self.width, self.height)
        finally:
            fcntl.flock(self.file.fileno(), fcntl.LOCK_UN)
        return True

    def close(self):
        self.enabled = False
        if self.process is not None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1.)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1.)
            self.process = None
        if self.mapping is not None:
            self.mapping.close()
            self.mapping = None
        if self.file is not None:
            self.file.close()
            self.file = None


def _child(inherited_fd, width, height, host, port, stream_width, quality, retry, retry_max):
    import cv2
    import numpy as np
    cv2.setNumThreads(1)
    fd = os.open('/proc/self/fd/%d' % inherited_fd, os.O_RDWR)
    os.close(inherited_fd)
    memory = mmap.mmap(fd, HEADER+width*height*3)
    last = 0
    delay = retry
    while True:
        client = None
        try:
            client = WebSocketClient(host, port)
            client.send(packet(1, b'code'))
            accepted, fmt, deadline = False, 1, time.monotonic()+10
            while True:
                for message in client.receive(.01):
                    ident, payload = unpack_packet(message)
                    if ident == 2:
                        if payload != b'\x01':
                            raise OSError('MaixVision registration rejected')
                        accepted, delay = True, retry
                    elif ident == 14 and len(payload) == 1:
                        value = payload[0]
                        if value <= 2:
                            fmt = value
                        client.send(packet(15, bytes((int(value <= 2), value))))
                    elif ident == 21 and len(payload) == 1:
                        quality = max(1, min(100, payload[0]))
                        client.send(packet(22, bytes((1, quality))))
                if not accepted:
                    if time.monotonic() > deadline:
                        raise OSError('MaixVision registration timeout')
                    continue
                if fmt == 0:
                    continue
                try:
                    fcntl.flock(fd, fcntl.LOCK_EX|fcntl.LOCK_NB)
                except BlockingIOError:
                    continue
                try:
                    sequence, w, h = struct.unpack_from('<QII', memory, 0)
                    raw = bytes(memory[HEADER:]) if sequence != last and sequence and (w,h)==(width,height) else None
                finally:
                    fcntl.flock(fd, fcntl.LOCK_UN)
                if raw is None:
                    continue
                last = sequence
                frame = np.frombuffer(raw, np.uint8).reshape(height, width, 3)
                if 0 < stream_width < width:
                    frame = cv2.resize(frame, (stream_width, max(1, round(height*stream_width/width))), interpolation=cv2.INTER_AREA)
                ok, encoded = cv2.imencode('.jpg' if fmt == 1 else '.png', frame,
                                          [cv2.IMWRITE_JPEG_QUALITY, quality] if fmt == 1 else [])
                if ok:
                    client.send(packet(6, bytes((fmt,))+encoded.tobytes()))
        except (OSError, ValueError) as exc:
            print('[stream] reconnect: %s' % exc, flush=True)
        finally:
            if client is not None:
                client.close()
        time.sleep(delay)
        delay = min(retry_max, max(.05, delay*2))
'''

# ===== vision.preview  (vision/preview.py) =====
_MOD_SRC['vision.preview'] = r'''
"""显示层：标注画面 →（letterbox）画布 → display.show()，附屏幕可点击按钮（从 main.py 搬迁，v0.4.6）。

时间轴 C 的实现（见 main.py 模块开头总纲）：
    · Preview      —— 显示线程 + 画布池；主循环 render() 投递即返回，show() 慢也只慢预览。
    · TouchButtons —— 触摸按下沿 → 按钮命中。触摸坐标是屏幕坐标，与画布 1:1。
    · display_result()            —— 把识别线程的最新快照翻译成"这一帧该画什么"（过期降级）。
    · apply_maixvision_stream_policy() —— 建 Display 前掐断 MaixVision 图传（拆 500ms 忙等）。

★ clock 注入：构造可传 `clock`（默认标准库 time）。入口 main() 把自身 time 传进来，
  使 tools/test_main_self_test.py 的假时钟可穿透（同步显示路径在模拟时间轴下可复现）。
★ 隔离原则：显示层任何失败一律只打日志并置 Preview.enabled=False，绝不影响识别/串口/闭环。
"""
import os
import math
import threading
import time

import cv2
import numpy as np

from .rate_meter import FrameRate


def display_result(snapshot, now, max_age):
    """把"识别线程的最新快照"翻译成"这一帧显示该画什么"。

    过期定位不能被画成当前帧的有效定位（否则屏幕上框会"粘"在旧位置，误以为还锁着）；
    但视频原帧继续刷新，画面不因此冻住。返回 (result, laser, age)：
        age > max_age → 用 {valid:False, stage:"stale"} 占位，只保留 laser（面板照常显示）
    """
    if snapshot is None:
        return {"valid": False, "stage": "waiting"}, None, 0.0
    age = now - snapshot["captured_at"]
    if not math.isfinite(age) or not 0 <= age <= max_age:
        return {"valid": False, "stage": "stale"}, snapshot["laser"], age
    return snapshot["result"], snapshot["laser"], age


def apply_maixvision_stream_policy(cfg):
    """Compatibility hook: stream policy is now local to Display construction."""
    return False


def _local_display_call(callback):
    previous = os.environ.get("MAIXVISION")
    try:
        os.environ["MAIXVISION"] = "0"
        return callback()
    finally:
        if previous is None:
            os.environ.pop("MAIXVISION", None)
        else:
            os.environ["MAIXVISION"] = previous


class Preview:
    """显示层：标注画面 →（letterbox）画布 → `display.show()`，附屏幕可点击按钮。

    ★ 线程模型（时间轴 C，v0.4.3 起）
        主循环（生产者）  `render(view, ...)` —— letterbox + 叠按钮，把画布塞进"待上屏槽"就返回；
        显示线程（消费者） `_show_loop()`     —— 取最新画布 → `show()` → 画布还回池子。
        画布池固定 2 张：显示线程占 1 张、主线程画 1 张；两张都被占用时 `due()` 直接返回 False
        （丢帧，不排队、不等待）。因此 **show() 再慢也只表现为预览掉帧**，绝不拖慢主循环、
        识别线程与云台闭环 —— 这正是"把识别拆出去"之外还必须做的一步（见模块开头总纲）。
        `Config.SHOW_ASYNC=False` 时退回旧行为（主循环内同步 `show()`）。

    为什么画布做成"屏幕尺寸"：MaixPy 触摸返回的是**屏幕坐标**，只有画布尺寸 == 屏幕尺寸，
    触摸坐标才能与画布/按钮坐标 **1:1** 对应，按钮命中判定才可靠（MaixPy「触摸屏使用」示例即如此）。
    标注图按等比例 letterbox 居中，不拉伸变形。
    `Config.SHOW=False` 时不建 Display；无显示 / 建失败 / show 失败只打日志并停用，绝不影响识别与闭环。

    两级尺寸务必分清（`render()` 里同时出现，最易混淆）：
        · display.width/height  —— 物理屏幕尺寸，也是**触摸坐标**所在坐标系（按钮按此定义）
        · render_width/height   —— 实际送去 show() 的**画布**尺寸（SHOW_WIDTH 限制）
    默认 `SHOW_WIDTH=0` ⇒ 画布 == 屏幕：`show()` 不再缩放、触摸不再换算；相机 640×480 与
    MaixCAM2 屏幕 640×480 宽高比一致时，letterbox 也退化成零开销（不 resize、不刷黑边）。
    """

    def __init__(self, cfg, clock=None):
        self.cfg = cfg
        self._clock = clock if clock is not None else time
        self.stream = None
        self._flash = {}
        self.display = None
        self.width = self.height = 0            # 物理屏幕尺寸（触摸坐标系）
        self.render_width = self.render_height = 0
        self.enabled = False                    # 显示是否可用；任何一步失败都置 False 并停用
        self.buttons = {}                       # name -> (x0,y0,x1,y1)，屏幕坐标
        self._button_draw = []                  # 预算好的按钮绘制参数（画布坐标），渲染时不再换算
        self.shown = 0                          # 已成功 show() 的帧数（disp 口径）
        self.dropped = 0                        # 因显示线程忙而未画的帧数（丢帧，不排队）
        self.fps = 0.0
        self.low1 = 0.0
        self.rate = FrameRate()                 # 显示帧率（disp 口径，与识别帧率分开）
        self._period = 1.0 / max(1, int(getattr(cfg, "SHOW_FPS", 15)))   # 显示节拍周期
        self._t_show = -1e9                     # 下一次该画的时刻（固定时间轴，见 due/render）
        self._canvas = None                     # 同步模式的单张画布（异步模式走 _free 池）
        self._show_times = []                   # 本日志窗口内的 show() 耗时样本（ms）
        self._show_max = 0.0
        self.probe_ms = -1.0                    # 开机 show() 探针耗时（判定显示链路是否健康）
        self._slow_warned = False               # "显示侧慢"告警只打一次，避免每 2s 刷屏
        # ---- 显示线程状态（时间轴 C）----
        self.async_show = bool(getattr(cfg, "SHOW_ASYNC", True))
        self._cond = threading.Condition()      # 保护 _free / _pending / _pool / _stop
        self._free = []                         # 空闲画布池
        self._pending = None                    # 待上屏画布（只留最新一张）
        self._pool = 0                          # 已分配画布数（上限 2，由 due() 保证）
        self._stop = False
        self.thread = None
        if not bool(getattr(cfg, "SHOW", False)):
            print("[preview] SHOW=False -> 不建显示（纯串口/头部模式，画面不流转）")
            return
        try:
            from maix import display
            self.display = _local_display_call(display.Display)
            self.width = int(self.display.width())
            self.height = int(self.display.height())
            # 画布宽度：SHOW_WIDTH<=0 ⇒ 用屏幕原始宽度（画布＝屏幕 ⇒ show() 零缩放）。
            cap = int(getattr(cfg, "SHOW_WIDTH", 0) or 0)
            self.render_width = self.width if cap <= 0 else min(self.width, cap)
            self.render_height = max(1, int(round(self.height * self.render_width / self.width)))
            if hasattr(display, "set_trans_image_quality"):
                display.set_trans_image_quality(int(getattr(cfg, "SHOW_JPEG_QUALITY", 70)))
            self.enabled = True
            print("[preview] mode=%s screen=%dx%d canvas=%dx%d show_fps=%d jpeg=%d maixvision=%s"
                  % ("display-thread" if self.async_show else "sync-inprocess",
                     self.width, self.height, self.render_width, self.render_height,
                     int(getattr(cfg, "SHOW_FPS", 30)), int(getattr(cfg, "SHOW_JPEG_QUALITY", 70)),
                     "on" if os.environ.get("MAIXVISION") == "1" else "off"))
            # ★ 开机探针：立刻量一次 show() 真实耗时。本地屏典型 1~5ms；若命中 MaixCDK 的
            #   "MaixVision 图传未就绪"分支，send_image() 会在 C++ 里**忙等 500ms 并空转 CPU**
            #   （见 MaixCDK components/vision/src/maix_image_trans.cpp），表现为 show 恒定 ~505ms、
            #   并连带把相机/识别/闭环一起拖慢。这条必须当场说清，不能只报一个 p95 数字。
            self.probe_ms = self._probe_show()
            if not bool(getattr(cfg, "DISABLE_MAIXVISION_STREAM", False)) and os.environ.get("MAIXVISION") == "1":
                from .maixvision_stream import MaixVisionStream
                self.stream = MaixVisionStream(cfg, self.render_width, self.render_height, clock=self._clock)
            if self.async_show:
                self.thread = threading.Thread(target=self._show_loop, name="display", daemon=True)
                self.thread.start()
        except Exception as exc:
            self.enabled = False
            # 建不了显示不算错误：板子仍照常识别 + 跑闭环，只是没有画面。
            print("[preview] 显示初始化失败（不影响识别/闭环，仅无预览）: %s" % exc)

    # ------------------------------------------------------------------ 按钮
    def add_button(self, name, x0, y0, x1, y1):
        """注册一个按钮（**屏幕坐标**）。几何与文本宽度在此**预计算一次**，渲染时不再换算。"""
        self.buttons[name] = (int(x0), int(y0), int(x1), int(y1))
        # 屏幕坐标 → 画布坐标（默认 SHOW_WIDTH=0 时二者相同，比例恒为 1）
        sx = self.render_width / float(self.width) if self.width else 1.0
        sy = self.render_height / float(self.height) if self.height else 1.0
        bx0, bx1 = int(round(x0 * sx)), int(round(x1 * sx))
        by0, by1 = int(round(y0 * sy)), int(round(y1 * sy))
        # 用 ASCII 标签：OpenCV 的 Hershey 字体不支持中文，画中文会变问号/方块。
        label = "SELF-TEST" if name == "selftest" else name.upper()
        tw = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)[0][0]
        pos = (bx0 + max(4, (bx1 - bx0 - tw) // 2), by0 + (by1 - by0) // 2 + 7)   # 水平居中
        self._button_draw.append((name, bx0, by0, bx1, by1, label, pos))

    def flash_button(self, name, now=None, hold=.25):
        self._flash[name] = (self._clock.perf_counter() if now is None else now)+hold

    def hit(self, x, y):
        """触摸点 (x, y)（屏幕坐标）命中的按钮名；未命中返回 None。"""
        for name, (x0, y0, x1, y1) in self.buttons.items():
            if x0 <= x <= x1 and y0 <= y <= y1:
                return name
        return None

    # ------------------------------------------------------------------ 渲染
    def due(self, now):
        """是否到了显示节拍**且**有画布可用；显示线程忙时返回 False（丢帧，绝不等待）。"""
        if not self.enabled or float(now) < self._t_show:
            return False
        if not self.async_show:
            return True
        with self._cond:
            if self._free or self._pool < 2:
                return True
        self.dropped += 1        # 节拍到了但两张画布都占用中 → 本帧不画（丢帧而非排队）
        return False

    def _take_canvas(self):
        """取一张画布：同步模式复用固定那张；异步模式从池里取，池空则新分配（上限由 due 保证）。"""
        need = (self.render_height, self.render_width)
        if not self.async_show:
            if self._canvas is None or self._canvas.shape[:2] != need:
                self._canvas = np.empty((need[0], need[1], 3), np.uint8)
            return self._canvas
        with self._cond:
            if self._free:
                return self._free.pop()
            if self._pool >= 2:
                return None
            self._pool += 1
        return np.empty((need[0], need[1], 3), np.uint8)

    def render(self, view, now, status=""):
        """把标注后的 `view`（任意尺寸）letterbox 到画布、叠按钮与状态，然后交给显示线程。

        步骤（对应模块总纲时间轴 A 的第 ⑦ 步）：
            1) 推进节拍 _t_show（固定时间轴，见下）
            2) 算 letterbox 缩放与居中偏移 (scale/ox/oy)，把 view 放进画布
            3) 叠按钮/状态条
            4) 交显示 —— 异步：投递后**立即返回**；同步：当场 `show()`
        """
        if not self.enabled:
            return
        now = float(now)
        # ★ 固定时间轴限频：节拍由"上一次预期时刻 + 周期"推进，而不是"show 完成后再等一周期"。
        #   好处：即使某次 show() 慢，也不会让后续节拍越拖越后（避免累积漂移）；
        #   已经错过的节拍用 skipped 一次性跳过，只画最新画面。
        if self._t_show < 0:
            self._t_show = now + self._period
        else:
            skipped = max(1, int((now - self._t_show) / self._period) + 1)
            self._t_show += skipped * self._period
        canvas = self._take_canvas()
        if canvas is None:
            self.dropped += 1
            return
        # ---- letterbox：等比缩放 + 居中，两侧留黑边，绝不拉伸（保持几何不变形）----
        vh, vw = view.shape[:2]
        rw, rh = self.render_width, self.render_height
        scale = min(rw / float(vw), rh / float(vh))         # 取两轴较小比例 → 完整装入
        nw, nh = max(1, int(round(vw * scale))), max(1, int(round(vh * scale)))
        ox, oy = (rw - nw) // 2, (rh - nh) // 2             # 居中偏移
        if (nw, nh) != (rw, rh):
            canvas[:] = 0                                   # 只有真存在黑边时才需要刷黑背景
        canvas[oy:oy + nh, ox:ox + nw] = (
            view if (nw, nh) == (vw, vh) else cv2.resize(view, (nw, nh)))
        self._draw_buttons(canvas, status)
        if self.stream is not None:
            try:
                self.stream.submit(canvas)
            except Exception as exc:
                print("[stream] submit failed: %s" % exc)
                self.stream.close()
                self.stream = None
        #                   # 按钮与状态条画在画布之上
        if self.async_show:
            self._publish(canvas)                            # 投递即返回：show 交给显示线程
        else:
            self._do_show(canvas)

    def _publish(self, canvas):
        """把画布放进"待上屏槽"（只留最新一张）并唤醒显示线程；主循环不等它。"""
        with self._cond:
            if self._stop:
                return
            if self._pending is not None:
                self._free.append(self._pending)             # 上一张还没上屏 → 退回池（丢帧）
                self.dropped += 1
            self._pending = canvas
            self._cond.notify()

    def _show_loop(self):
        """显示线程主体（时间轴 C）：取最新画布 → show() → 画布还池。**慢也只慢自己**。"""
        try:
            from maix import image as maix_image
        except Exception as exc:
            self.enabled = False
            print("[preview] 显示线程无法导入 maix.image，退化为无预览: %s" % exc)
            return
        while True:
            with self._cond:
                if self._pending is None and not self._stop:
                    self._cond.wait(timeout=0.02)            # 20ms 轮询，保证 stop 能被及时看到
                if self._stop:
                    return
                canvas, self._pending = self._pending, None
            if canvas is None:
                continue
            if not self._do_show(canvas, maix_image):
                return                                       # show 已失败并停用显示 → 线程退出
            with self._cond:
                self._free.append(canvas)

    def _probe_show(self):
        """开机量一次 show() 耗时，判定显示链路是否健康；返回毫秒（探针失败返回 -1）。

        判据为何取 300ms：本地屏幕刷新官方数据是 **1~5ms**；而 MaixCDK 的
        `ImageTrans::send_image()` 在「处于 MaixVision 模式、但图传连接未就绪（handle->init 为假）」
        时，会 `while(!init) { if(ticks - t > 500) return ERR_NOT_READY; }` ——
        **忙等 500ms 且空转 CPU**。这正是本项目实测
        `show avg/p95/max=505.8/508.4/509.1ms`、`disp≈1.9fps` 且相机与识别一起变慢的根因：
        500ms × 约 2 次/秒 ≈ **烧掉半个 CPU 核**。
        """
        try:
            from maix import image as maix_image
            frame = np.zeros((self.render_height, self.render_width, 3), np.uint8)
            t0 = self._clock.perf_counter()
            self.display.show(maix_image.cv2image(frame, bgr=True, copy=True))
            ms = (self._clock.perf_counter() - t0) * 1000.0
        except Exception as exc:
            print("[preview] show 探针失败（显示可能不可用）: %s" % exc)
            self.enabled = False
            return -1.0
        if ms >= float(getattr(self.cfg, "SHOW_STREAM_SLOW_MS", 300.)):
            print("[preview] show 探针 %.0fms：本地显示偏慢；检查 SDK 是否复用旧图传对象。" % ms)
            print("[preview] 对策①：确认 Display 构造时 MAIXVISION=0；本应用已临时隔离并恢复会话。")
            print("[preview] 对策②：从设备本地启动，对照本地屏 show 耗时。")
            print("[preview] 对策③：检查 VO/屏幕驱动；独立图传连接与编码不在 show 线程。")
        else:
            print("[preview] show 探针 %.1fms —— 显示链路健康（本地屏典型 1~5ms）" % ms)
        return ms

    def warn_slow_once(self, p95):
        """显示侧变慢只提示一次（每 2s 窗口重复报会刷屏）；点明最可能的根因。"""
        if self._slow_warned:
            return
        self._slow_warned = True
        print("[preview] show slow: p95=%.0fms；检查本地屏驱动和 SDK 的图传对象缓存。"
              "图传进程独立，预览丢帧不会积压识别结果。" % p95)

    def _do_show(self, canvas, maix_image=None):
        """真正调用 display.show()，记耗时/帧率；失败即停用显示（不影响识别/闭环）。返回是否成功。"""
        t0 = self._clock.perf_counter()
        try:
            if maix_image is None:
                from maix import image as maix_image
            # bgr=True：cv2 是 BGR 序；copy=True：交给显示的必须是独立缓冲。
            self.display.show(maix_image.cv2image(canvas, bgr=True, copy=True))
        except Exception as exc:
            self.enabled = False
            print("[preview] show 失败，停用显示: %s" % exc)
            return False
        ms = (self._clock.perf_counter() - t0) * 1000.0
        self.shown += 1
        completed = self._clock.perf_counter()
        self.rate.record(completed)
        self.fps, self.low1 = self.rate.values(completed)
        with self._cond:
            self._show_times.append(ms)
            if ms > self._show_max:
                self._show_max = ms
        return True

    def take_show_stats(self):
        """取走并清空本日志窗口的 show() 耗时样本 → (avg, p95, max, n)，单位 ms。"""
        with self._cond:
            times, worst = self._show_times, self._show_max
            self._show_times, self._show_max = [], 0.0
        if not times:
            return 0.0, 0.0, worst, 0
        arr = np.asarray(times, dtype=np.float64)
        return float(arr.mean()), float(np.percentile(arr, 95)), worst, len(times)

    def _draw_buttons(self, canvas, status=""):
        # 按钮几何/文本宽度已在 add_button() 预算好，这里只画（不再每帧 getTextSize/换算）。
        for (name, x0, y0, x1, y1, label, pos) in self._button_draw:
            cv2.rectangle(canvas, (x0, y0), (x1, y1), (0, 110, 235) if self._clock.perf_counter() < self._flash.get(name, -1e9) else (46, 150, 60), -1)   # 填充
            cv2.rectangle(canvas, (x0, y0), (x1, y1), (235, 235, 235), 1)  # 描边
            cv2.putText(canvas, label, pos,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
        if status:
            # 左下角状态串：self-test i/n、calibrate <axis>、或 aim 状态机名。
            cv2.putText(canvas, status, (8, self.render_height - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

    def close(self):
        """先停显示线程（等它把当前一次 show 做完）再关 Display —— 顺序反了会并发操作显示。"""
        if self.async_show:
            with self._cond:
                self._stop = True
                self._pending = None
                self._cond.notify()
            if self.thread is not None:
                self.thread.join(timeout=2.0)
                if self.thread.is_alive():
                    raise RuntimeError("display thread did not stop")
        if self.stream is not None:
            self.stream.close()
        if self.display is not None:
            try:
                self.display.close()
            except Exception:
                pass


class TouchButtons:
    """触摸屏输入：把点击映射到 `Preview` 的按钮（**按下沿**触发，防连点重复启动）。

    `maix.touchscreen.TouchScreen().read()` 立即返回 `(x, y, pressed)`（MaixPy v4）。
    无触摸屏 / 未启用 / 无显示时自动禁用，只打日志。
    坐标口径：read() 返回的是**屏幕坐标**，与按钮注册坐标一致（画布=屏幕尺寸的前提）。
    """

    MAX_HOLD_S = 2.0

    def __init__(self, cfg, clock=None):
        self._clock = clock if clock is not None else time
        self._pressed_at = -1e9
        self.ts = None
        self._down = False                 # 上一帧按下状态，用于识别"按下沿"
        if not bool(getattr(cfg, "ENABLE_TOUCH_BUTTON", True)):
            print("[touch] 触摸按钮已关闭（ENABLE_TOUCH_BUTTON=False）")
            return
        try:
            from maix import touchscreen
            self.ts = touchscreen.TouchScreen()
            print("[touch] 触摸屏就绪：屏幕按钮可点击")
        except Exception as exc:
            print("[touch] 触摸不可用（按钮禁用）: %s" % exc)

    def poll(self, preview):
        """返回本帧**新按下**的按钮名（按下沿）；无则 None。"""
        if self.ts is None or preview is None or not preview.enabled:
            return None
        try:
            x, y, pressed = self.ts.read()
        except Exception as exc:
            self._down = False
            print("[touch] read failed: %s" % exc)
            return None
        now = self._clock.perf_counter()
        if self._down and now-self._pressed_at > self.MAX_HOLD_S:
            self._down = False
        if pressed:
            if not self._down:             # 只有"上一帧未按、这一帧按下"才算一次点击
                self._down = True
                self._pressed_at = now
                name = preview.hit(int(x), int(y))
                print("[touch] x=%d y=%d hit=%s" % (x, y, name or "none"))
                if name:
                    preview.flash_button(name, now)
                return name
            return None                    # 按住不放：不重复返回，避免反复触发
        self._down = False                 # 松开，允许下一次按下沿
        return None
'''

# ===== vision.rect_geometry  (vision/rect_geometry.py) =====
_MOD_SRC['vision.rect_geometry'] = r'''
"""Pure projective geometry; all homographies map physical mm to image pixels."""
import cv2
import numpy as np


def polygon_area(p):
    """带符号面积的 2 倍（>0 = 逆时针 CCW，<0 = 顺时针 CW）。用于把四角规整成统一方向。"""
    p = np.asarray(p)
    return float(p[0,0]*(p[1,1]-p[3,1]) + p[1,0]*(p[2,1]-p[0,1]) +
                 p[2,0]*(p[3,1]-p[1,1]) + p[3,0]*(p[0,1]-p[2,1]))


def order_quad(points):
    """把 4 个点规整为统一顺序：以质心为极点按极角排序 → 保证 CCW → 旋转到"左上角"开头。

    统一顺序是后续所有几何运算（边向量、对角线、单应四点对应）的前提。
    返回顺序语义：**[左上, 右上, 右下, 左下]**（逆时针），与 side_lengths 的边 0/1/2/3 对应。
    """
    p = np.asarray(points, np.float32).reshape(4, 2)
    c = p.mean(axis=0)
    p = p[np.argsort(np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0]))]   # 按极角排序
    if polygon_area(p) < 0:
        p = p[::-1]                                                  # 反了则翻转成 CCW
    # np.roll 到"x+y 最小"的点（即左上角）打头。
    i = int(np.argmin(p.sum(axis=1)))
    return np.concatenate((p[i:], p[:i]))


def side_lengths(q):
    """四边长度 [边0, 边1, 边2, 边3]，边 i = 从点 i 指向点 i+1（roll -1 取下一个点）。"""
    return np.linalg.norm(np.roll(q, -1, axis=0) - q, axis=1)


def gaussian_solve(a, b):
    """带部分主元的高斯消元；消去行向量化，避免 64 次 Python 行循环。

    a 为 n×n、b 为 n，返回解向量。主元过小（矩阵退化，如四点共线）抛 ValueError，
    由调用方捕获后判该候选无效 —— 不做伪解。
    """
    m = np.column_stack((np.asarray(a, np.float64), np.asarray(b, np.float64)))
    for col in range(len(m)):
        pivot = col + int(np.argmax(np.abs(m[col:, col])))    # 选列内绝对值最大的行做主元
        if abs(m[pivot, col]) < 1e-10:
            raise ValueError("degenerate quadrilateral")
        if pivot != col:
            m[[col, pivot]] = m[[pivot, col]]                 # 换行
        m[col] /= m[col, col]                                 # 归一化主元行
        factors = m[:, col].copy()
        factors[col] = 0.0
        m -= factors[:, None] * m[col]                        # 一次消去整列（行向量化）
    return m[:, -1]


def homography(src, dst):
    """解 4 点对应的 3×3 单应 H（使 H·src ≡ dst）。

    做法：先对 src/dst **归一化**（平移质心到原点、缩放到量级 1）再建 8×8 线性方程，
    求得 H_n 后用 T_dst·H_n·T_src⁻¹ 还原 —— 归一化能显著提升大像素坐标下的数值稳定性。
    退化情形（任一边集合尺度过小/共线）抛 ValueError。
    """
    src, dst = np.asarray(src, np.float64), np.asarray(dst, np.float64)
    sc, dc = src.mean(axis=0), dst.mean(axis=0)
    ss, ds = float(np.max(np.abs(src - sc))), float(np.max(np.abs(dst - dc)))
    if min(ss, ds) < 1e-8:
        raise ValueError("zero-size rectangle")
    xy, uv = (src - sc) / ss, (dst - dc) / ds                 # 归一化坐标
    a = np.zeros((8, 8), np.float64)
    a[0::2, :3] = np.column_stack((xy, np.ones(4)))           # 偶数行：x 分量方程
    a[1::2, 3:6] = a[0::2, :3]                                # 奇数行：y 分量方程
    a[0::2, 6:] = -uv[:, :1] * xy                             # 第 3 列（透视项）
    a[1::2, 6:] = -uv[:, 1:] * xy
    v = gaussian_solve(a, uv.reshape(-1))
    hn = np.append(v, 1.0).reshape(3, 3)                      # 归一化域下的 H
    ts = np.array([[1 / ss, 0, -sc[0] / ss], [0, 1 / ss, -sc[1] / ss], [0, 0, 1]])
    td_inv = np.array([[ds, 0, dc[0]], [0, ds, dc[1]], [0, 0, 1]])
    h = td_inv @ hn @ ts                                      # 还原到原坐标域
    return h / h[2, 2] if abs(h[2, 2]) > 1e-12 else h


def homography_8x8(src_quad, width_mm, height_mm):
    """把物理矩形 (0,0)-(w,h) 映射到图像四边形 src_quad 的单应（mm → px）。"""
    return homography(rectangle_corners(width_mm, height_mm), src_quad)


def project_points(matrix, points):
    """齐次投影：把 (N,2) 点经 3×3 矩阵映射，做透视除法得 (N,2)。落无穷远点抛错。"""
    p = np.asarray(points, np.float64).reshape(-1, 2)
    z = np.column_stack((p, np.ones(len(p)))) @ np.asarray(matrix).T
    if np.any(np.abs(z[:, 2]) < 1e-9):
        raise ValueError("point at projective infinity")
    return z[:, :2] / z[:, 2:]


def inverse_map_points(h, points):
    """单应逆映射（px → mm）；不做透视除法前的越界检查，由调用方负责。"""
    return project_points(np.linalg.inv(h), points)


def standard_circle(width_mm, height_mm, radius_mm, samples=72):
    """正视面坐标下、以矩形中心为圆心、半径 radius_mm 的等分圆采样点（(samples,2)）。

    用途：把该圆逆映射回原图，用于可视化/交叉校验中心；**圆心本身直接取矩形中心投影**，
    不用圆周平均（见 process 内注释）。
    """
    a = np.arange(samples) * (2 * np.pi / samples)
    return np.array([width_mm / 2, height_mm / 2]) + radius_mm * np.column_stack((np.cos(a), np.sin(a)))


def rectangle_corners(w, h, inset=0.0):
    """物理矩形四角（mm），inset>0 时向内缩进（用于取胶带内沿 = 内框）。顺序 LL→LR→UR→UL。"""
    return np.array([[inset, inset], [w - inset, inset],
                     [w - inset, h - inset], [inset, h - inset]], np.float64)


def sample_batch(gray, points):
    """一次 OpenCV remap 双线性采样整批点，越界由调用方拒绝。

    相比逐点 Python 循环，remap 一次调用即可采样成百上千个点（四边验证/精修的关键提速）。
    borderMode=REPLICATE 保证边缘点也能返回数值（越界判据在调用方单独做）。
    """
    shape = points.shape[:-1]
    p = points.reshape(-1, 2).astype(np.float32)
    values = cv2.remap(gray, p[:, 0].reshape(1, -1), p[:, 1].reshape(1, -1),
                       cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    return values.reshape(shape)


def quad_is_valid(q, shape, margin=1, min_edge=12., min_sine=.20):
    q = np.asarray(q, np.float32)
    if q.shape != (4, 2) or not np.isfinite(q).all():
        return False
    h, w = shape[:2]
    if q[:,0].min() < margin or q[:,1].min() < margin or q[:,0].max() >= w-margin or q[:,1].max() >= h-margin:
        return False
    if not cv2.isContourConvex(q.reshape(-1, 1, 2)):
        return False
    e = np.concatenate((q[1:], q[:1])) - q
    lengths = np.linalg.norm(e, axis=1)
    if lengths.min() < min_edge:
        return False
    nxt = np.concatenate((e[1:], e[:1]))
    sine = np.abs(e[:,0]*nxt[:,1]-e[:,1]*nxt[:,0]) / (lengths*np.r_[lengths[1:], lengths[:1]])
    return bool(sine.min() >= min_sine)


def quad_from_contour(cnt, eps_ratio, hull_fill_ratio, factors=(1., 1.6, 2.4, 3.5, 5.)):
    def fit(contour):
        perimeter = cv2.arcLength(contour, True)
        for factor in factors:
            ap = cv2.approxPolyDP(contour, eps_ratio*factor*perimeter, True)
            if len(ap) == 4 and cv2.isContourConvex(ap):
                return order_quad(ap)
        return None
    q = fit(cnt)
    if q is not None:
        return q
    hull = cv2.convexHull(cnt)
    # A thin broken ring can be repaired; all hull proposals still need dark-band evidence.
    if abs(cv2.contourArea(cnt)) < hull_fill_ratio*abs(cv2.contourArea(hull)):
        return fit(hull)
    return None


def front_view_size(w, h, short_px):
    scale = float(short_px) / min(w, h)
    return max(2, round(w*scale)), max(2, round(h*scale))


def warp_front_view(gray, rect_to_image, w, h, short_px):
    fw, fh = front_view_size(w, h, short_px)
    mm_to_front = np.diag([(fw-1)/w, (fh-1)/h, 1.])
    # OpenCV takes src -> dst; reversing this matrix silently warps background.
    return cv2.warpPerspective(gray, mm_to_front @ np.linalg.inv(rect_to_image), (fw, fh))
'''

# ===== vision.contour_locator  (vision/contour_locator.py) =====
_MOD_SRC['vision.contour_locator'] = r'''
"""Bounded low-resolution proposals: one Canny mask, one Otsu fallback."""
import time
import cv2
import numpy as np
from .rect_geometry import order_quad, quad_from_contour, quad_is_valid

LEVEL_EDGE, LEVEL_MASK = 0, 1


class ContourLocator:
    LEVEL_EDGE, LEVEL_MASK = LEVEL_EDGE, LEVEL_MASK

    def __init__(self, cfg):
        self.cfg = cfg
        self.last_stats = {}
        size = max(1, int(getattr(cfg, "EDGE_CLOSE_PX", 3)))
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))

    def _quads_from_mask(self, mask):
        contours = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
        self.last_stats["contours"] = len(contours)
        h, w = mask.shape
        cfg, quads = self.cfg, []
        for cnt in sorted(contours, key=lambda c: abs(cv2.contourArea(c)), reverse=True)[:getattr(cfg, "SEARCH_MAX_CONTOURS", 16)]:
            _, _, cw, ch = cv2.boundingRect(cnt)
            if min(cw, ch) < max(8, .04*min(w, h)):
                continue
            area = abs(cv2.contourArea(cnt))
            if not w*h*cfg.MIN_AREA_RATIO <= area <= w*h*cfg.MAX_AREA_RATIO:
                continue
            q = quad_from_contour(cnt, cfg.APPROX_EPSILON, getattr(cfg, "HULL_FILL_RATIO", .6))
            if q is None or not quad_is_valid(q, mask.shape, min_edge=6):
                continue
            if any(np.max(np.linalg.norm(q-old, axis=1)) < 2 for old in quads):
                continue
            quads.append(q)
            if len(quads) >= cfg.MAX_CANDIDATES:
                break  # Bound expensive polygon fitting, not accepted-candidate stability.
        return quads

    def locate(self, gray, roi=None, level=LEVEL_EDGE):
        started = time.perf_counter()
        x, y, w, h = roi or (0, 0, gray.shape[1], gray.shape[0])
        self.last_stats = dict(masks=0, contours=0, quads=0, ms=0., level=level)
        if w <= 0 or h <= 0:
            return []
        scale = min(1., self.cfg.SEARCH_WIDTH / float(w))
        sw, sh = max(1, round(w*scale)), max(1, round(h*scale))
        crop = gray[y:y+h, x:x+w]
        small = cv2.resize(crop, (sw, sh), interpolation=cv2.INTER_AREA) if scale < 1 else crop
        if level == LEVEL_EDGE:
            mask = cv2.Canny(small, self.cfg.CANNY_LOW, self.cfg.CANNY_HIGH)
        elif level == LEVEL_MASK:
            mask = cv2.threshold(small, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
        else:
            raise ValueError("unknown proposal level")
        if self.kernel.shape[0] > 1:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        self.last_stats["masks"] = 1
        quads = self._quads_from_mask(mask)
        self.last_stats.update(quads=len(quads), ms=(time.perf_counter()-started)*1000)
        mapped = [(q*np.array([w/sw, h/sh], np.float32)+[x, y]).astype(np.float32) for q in quads]
        return [(q, kind) for q in mapped for kind in ("inner", "outer")]
'''

# ===== vision.kalman_filter  (vision/kalman_filter.py) =====
_MOD_SRC['vision.kalman_filter'] = r'''
"""Constant-velocity center filter. Raw observations remain the control input."""
import numpy as np


class ConstantVelocityKalman2D:
    def __init__(self, process_px=40., measure_px=.9, reset_px=25.,
                 max_predict_s=.25, velocity_limit_px_s=6000.):
        self.q, self.r = float(process_px), float(measure_px)
        self.reset_px, self.max_gap = float(reset_px), float(max_predict_s)
        self.velocity_limit = float(velocity_limit_px_s)
        if not np.isfinite([self.q, self.r, self.reset_px, self.max_gap, self.velocity_limit]).all() or min(self.r, self.max_gap, self.velocity_limit) <= 0 or self.q < 0:
            raise ValueError("invalid Kalman parameters")
        self.reset()

    def reset(self):
        self.x = self.p = self.timestamp = self.last_measurement = None

    def seed(self, position, timestamp):
        z = np.asarray(position, dtype=float).reshape(2)
        if not np.isfinite(z).all() or not np.isfinite(timestamp):
            raise ValueError("non-finite observation")
        self.x = np.r_[z, 0., 0.]
        self.p = np.diag([self.r**2]*2 + [(self.r*30)**2]*2)
        self.timestamp, self.last_measurement = float(timestamp), z.copy()
        return z.copy()

    def update(self, position, timestamp):
        z = np.asarray(position, dtype=float).reshape(2)
        if not np.isfinite(z).all() or not np.isfinite(timestamp):
            raise ValueError("non-finite observation")
        dt = 0 if self.timestamp is None else timestamp - self.timestamp
        if self.x is None or not 0 < dt <= self.max_gap or np.linalg.norm(z-self.last_measurement) > self.reset_px:
            return self.seed(z, timestamp)
        f = np.eye(4)
        f[0, 2] = f[1, 3] = dt
        a = np.array([[dt*dt/2, 0], [0, dt*dt/2], [dt, 0], [0, dt]])
        x = f @ self.x
        p = f @ self.p @ f.T + self.q**2 * (a @ a.T)
        k = np.linalg.solve(p[:2, :2] + np.eye(2)*self.r**2, p[:, :2].T).T
        self.x = x + k @ (z-x[:2])
        # Joseph form keeps the covariance positive even after long runs.
        ikh = np.eye(4)
        ikh[:, :2] -= k
        p = ikh @ p @ ikh.T + self.r**2*(k @ k.T)
        self.p = .5*(p+p.T)
        speed = np.linalg.norm(self.x[2:])
        if speed > self.velocity_limit:
            self.x[2:] *= self.velocity_limit/speed
        self.timestamp, self.last_measurement = float(timestamp), z.copy()
        return self.x[:2].copy()

    def predict(self, timestamp):
        if self.x is None or not np.isfinite(timestamp):
            return None
        dt = timestamp-self.timestamp
        if not 0 <= dt <= self.max_gap:
            return None
        return self.x[:2] + self.x[2:]*dt
'''

# ===== vision.rectangle_center  (vision/rectangle_center.py) =====
_MOD_SRC['vision.rectangle_center'] = r'''
"""Canny proposals -> dark-band verification -> subpixel lines -> projective center.

Tracking samples only four narrow strips. Searches always finish in the same frame;
ROI failure never consumes the right to attempt a full-frame reacquisition.
"""
import time
import cv2
import numpy as np
from .rect_geometry import (polygon_area as _polygon_area, order_quad as _order_quad,
    side_lengths as _side_lengths, gaussian_solve as _gaussian_solve,
    homography as _homography, rectangle_corners as _rectangle,
    sample_batch as _sample, homography_8x8, project_points, inverse_map_points,
    standard_circle, quad_is_valid, warp_front_view)
from .contour_locator import ContourLocator
from .kalman_filter import ConstantVelocityKalman2D


class RectangleCenterDetector:
    """有状态检测器：跨帧记住锁定框(_track)与平滑中心(_smooth)，稳态只做窄带精修。

    状态字段：
        _track       上一帧验证通过的外框（dict 候选）；None = 未锁定
        _prev_quad   再上一帧的候选四角（运动预测用）；失锁时清空
        _lost        连续未检出帧数（>0 = 刚失锁，禁用预测，避免用旧速度外推）
        _smooth      上一帧输出中心，用于 CENTER_ALPHA 一阶平滑（仅显示口径）
        _locked_th   上次成功的二值化阈值（下次优先复用，减少阈值轮换）
        _roi_retry   失锁后是否还允许一次"ROI 搜索"（用一次即置 False，防反复局部搜）
        _search_jobs 每 ROI 的阈值轮换进度（持续失败时跨帧轮换候选）
    每帧重置的临时量：_search_count / _threshold_count / _proposals / _deadline（时间预算）。
    """

    def __init__(self, config):
        self.cfg = config
        self.last = None
        self._smooth = None
        self._track = None
        self._prev_quad = None                                 # 上上帧的候选四角（运动预测用）
        self._lost = 0                                         # 连续未检出帧数（>0 时不预测）
        self._locked_th = None
        self._frame = 0
        self._kernel = np.ones((3, 3), np.uint8)               # 二值化后闭运算核
        # 四边上均匀分布的参数 t ∈ (0.12, 0.88)：避开四角（角点附近因透视最易模糊）。
        self._t = np.linspace(0.12, 0.88, config.EDGE_SAMPLES, dtype=np.float32)
        self._circle_cache = {}                                # 正视面圆周采样点缓存（按尺存）
        self._search_count = 0
        self._threshold_count = 0
        self._search_jobs = {}
        self._proposals = 0
        self._deadline = float("inf")
        self._roi_retry = True
        self.locator = ContourLocator(config)
        self._prev_timestamp = self._timestamp = self._frame_shape = None
        self._kalman = ConstantVelocityKalman2D(
            getattr(config, "KALMAN_PROCESS_PX", 40.), getattr(config, "KALMAN_MEASURE_PX", .9),
            getattr(config, "KALMAN_RESET_PX", 25.), getattr(config, "KALMAN_MAX_PREDICT_S", .25),
            getattr(config, "KALMAN_VELOCITY_LIMIT_PX_S", 6000.))

    def reset(self):
        self.last = self._smooth = self._track = self._prev_quad = None
        self._prev_timestamp = self._timestamp = self._frame_shape = None
        self._lost = 0
        self._roi_retry = True
        self._kalman.reset()

    def _valid_quad(self, q, shape, margin=1):
        return quad_is_valid(q, shape, margin, getattr(self.cfg, "MIN_EDGE_LENGTH_PX", 12.))

    def _layout(self, q, kind):
        """由四边形朝向推断物理长宽比，并给出该 kind 对应的物理矩形四角。

        horizontal：两条"横向边长之和" > 两条"纵向边长之和" → 长边水平 → w=长边(mm)。
        kind=="inner" 时物理矩形向内缩进一个胶带宽度 BLACK_FRAME_MM（对应内框角点）。
        """
        lengths = _side_lengths(q)
        horizontal = lengths[0] + lengths[2] > lengths[1] + lengths[3]
        w, h = ((self.cfg.TARGET_LONG_MM, self.cfg.TARGET_SHORT_MM) if horizontal
                else (self.cfg.TARGET_SHORT_MM, self.cfg.TARGET_LONG_MM))
        inset = self.cfg.BLACK_FRAME_MM if kind == "inner" else 0.0
        return w, h, _rectangle(w, h, inset)

    def _verify(self, gray, q, kind, diagnostic=False):
        """在正视面选取四边稀疏采样点并投影，不生成整幅 warp 图。

        每条边检查胶带厚度方向的暗带覆盖率与内侧亮区对比，允许局部反光；
        背景可以是黑色，不再强制外侧为白色。普通纸边及实心黑块会失败。
        """
        if diagnostic:
            self.last_diagnostic = {"reason": "invalid_geometry"}
        if not self._valid_quad(q, gray.shape):
            return None
        w, h, physical_quad = self._layout(q, kind)
        tape = float(self.cfg.BLACK_FRAME_MM)
        if tape <= 0 or 2 * tape >= min(w, h):
            raise ValueError("BLACK_FRAME_MM must be between 0 and half the short side")
        try:
            # 求 mm → px 单应：把物理矩形映射到图像候选四边形 q。
            rect_to_image = _homography(physical_quad, q)
            t = self._t
            # 沿每条边的法向采 5 个偏移：0.20/0.40/0.60×胶带宽 = 黑带内部三点（应为暗），
            # 1.22/1.45×胶带宽 = 越过内沿的亮面两点（应为亮）。暗亮对比即边证据。
            offsets = tape * np.array([0.20, 0.40, 0.60, 1.22, 1.45])
            p = np.empty((4, len(t), 5, 2), np.float64)
            # 四条边分别参数化：(边0 上/边1 右/边2 下/边3 左)，每条边沿 t 铺采样点。
            p[0, :, :, 0] = (tape + t * (w - 2 * tape))[:, None]
            p[0, :, :, 1] = offsets
            p[1, :, :, 0] = w - offsets
            p[1, :, :, 1] = (tape + t * (h - 2 * tape))[:, None]
            p[2, :, :, 0] = (tape + (1 - t) * (w - 2 * tape))[:, None]
            p[2, :, :, 1] = h - offsets
            p[3, :, :, 0] = offsets
            p[3, :, :, 1] = (tape + (1 - t) * (h - 2 * tape))[:, None]
            mapped = project_points(rect_to_image, p.reshape(-1, 2)).reshape(p.shape)
        except (ValueError, np.linalg.LinAlgError):
            return None
        gh, gw = gray.shape
        # 任何采样点落到图像外 → 该候选不可信（中心也无法可靠外推）。
        if (mapped[..., 0].min() < 0 or mapped[..., 1].min() < 0 or
                mapped[..., 0].max() >= gw - 1 or mapped[..., 1].max() >= gh - 1):
            return None
        samples = _sample(gray, mapped).astype(np.float32)
        # 每条边、每个 t：暗带取三点最大值（黑带里最亮的那个仍应暗），亮面取两点中位数。
        dark = np.max(samples[:, :, :3], axis=2)
        bright = np.median(samples[:, :, 3:], axis=2)
        delta = bright - dark
        # "该证据成立" = 对比度达标 且 暗带确实比亮面暗（比值约束，排除两侧都亮的纸边）。
        coverage = ((delta >= self.cfg.MIN_EDGE_CONTRAST) &
                    (dark <= bright * self.cfg.MAX_DARK_BRIGHT_RATIO)).mean(axis=1)
        edge_contrast = np.median(delta, axis=1)
        if diagnostic:
            self.last_diagnostic = dict(reason="dark_band_gates", coverage=coverage.tolist(),
                contrast=edge_contrast.tolist(), dark=np.median(dark,axis=1).tolist(),
                inside=np.median(bright,axis=1).tolist())
        # 至少三边的大部分采样成立，第四边也必须有证据，不接受凭亮中心猜框。
        if (np.count_nonzero(coverage >= self.cfg.MIN_EDGE_COVERAGE) < 3 or
                np.median(edge_contrast) < self.cfg.MIN_EDGE_CONTRAST):
            return None
        # Missing-edge exemption needs three strong sides, never a lone bright stripe.
        if coverage.min() < self.cfg.WEAK_EDGE_COVERAGE and np.sort(coverage)[1] < getattr(self.cfg, "STRONG_EDGE_COVERAGE", .85):
            return None
        med_dark, med_bright = np.median(dark, axis=1), np.median(bright, axis=1)
        rows = [dict(dark=float(med_dark[i]), inside=float(med_bright[i]),
                     contrast=float(edge_contrast[i]), coverage=float(coverage[i])) for i in range(4)]
        outer = project_points(rect_to_image, _rectangle(w, h)).astype(np.float32)
        if not self._valid_quad(outer, gray.shape):
            return None
        # 返回候选：外框(图像)、内框(胶带内沿)、mm→px 单应、四边剖面、质量分。
        # 打分 = 中位对比度 + 60×平均覆盖率，用于多候选时择优（分数越高越像真靶纸）。
        return dict(quad=q, outer_quad=outer,
                    inner_quad=project_points(rect_to_image, _rectangle(w, h, tape)).astype(np.float32),
                    kind=kind, rect_to_image=rect_to_image, width_mm=w, height_mm=h,
                    edge_profile=rows, contrast=float(np.median(edge_contrast)),
                    min_contrast=float(edge_contrast.min()), coverage=coverage,
                    score=float(np.median(edge_contrast) + 60 * coverage.mean()),
                    rank=float(np.median(edge_contrast) + 60 * coverage.mean() + 40 * coverage.min()))

    def _refine(self, gray, q, kind, radius):
        """当前帧四条灰度窄带 → 有方向的边缘跳变 → fitLine → 四角交点。

        与 _verify 的区别：_verify 只判"像不像"，_refine 才真正**重新定位**每条边。
        步骤：沿四边已有点 (base) 的法向 ±radius 采样灰度 → 求有向梯度 → 每列取
        "最近预测位置"的跳变（offset 越小越优先，防止误锁到内侧印刷圆或另一侧胶带边）
        → 用 fitLine 拟合四条直线 → 相邻直线两两求交 = 亚像素四角。
        精修结果偏离原 q 超过 radius*2.8 或四角体检不过 → 判失败（返回 None）。
        """
        q = np.asarray(q, np.float32)
        nxt = np.roll(q, -1, axis=0)
        e = nxt - q
        lengths = np.linalg.norm(e, axis=1)
        if lengths.min() < 10:
            return None
        normal = np.column_stack((-e[:, 1], e[:, 0])) / lengths[:, None]   # 各边外/内法向
        base = q[:, None, :] * (1 - self._t[None, :, None]) + nxt[:, None, :] * self._t[None, :, None]
        offsets = np.arange(-radius, radius + 1, dtype=np.float32)
        # 每条边、每个 t、每个法向偏移 → 采样点网格 (4, len(t), 2r+1, 2)。
        points = base[:, :, None, :] + normal[:, None, None, :] * offsets[None, None, :, None]
        gh, gw = gray.shape
        inside = ((points[..., 0] >= 1) & (points[..., 0] < gw - 1) &
                  (points[..., 1] >= 1) & (points[..., 1] < gh - 1)).all(axis=2)
        values = _sample(gray, points).astype(np.float32)
        gradient = values[:, :, 2:] - values[:, :, :-2]                    # 中心差分近似梯度
        if kind == "outer":
            gradient *= -1                                                 # 外框要求暗→亮方向相反
        # 近预测边缘优先，避免捕获靶纸内部印刷圆或另一侧的胶带边缘。
        penalty = np.abs(offsets[1:-1])[None, None, :] * 0.45              # 偏离预测位置的惩罚
        indices = np.argmax(gradient - penalty, axis=2)                    # 每股取最强跳变
        strength = np.take_along_axis(gradient, indices[..., None], axis=2)[..., 0]
        selected = offsets[indices + 1]
        lines = []
        refitted = 0
        for i in range(4):
            good = inside[i] & (strength[i] >= self.cfg.MIN_EDGE_CONTRAST * 0.7)   # 有效采样
            if good.sum() < max(5, len(self._t) * self.cfg.WEAK_EDGE_COVERAGE):
                # One missing/glared edge may retain its input line; three must be refitted.
                lines.append(np.cross(np.r_[q[i], 1.], np.r_[nxt[i], 1.]))
                continue
            shifts = selected[i, good]
            med = float(np.median(shifts))
            # 去掉偏离中位数过大的离群跳变（反光/杂物），再确认仍够点。
            good &= np.abs(selected[i] - med) <= max(2.5, float(np.median(np.abs(shifts - med))) * 3)
            if good.sum() < 5:
                return None
            samples = base[i, good] + selected[i, good, None] * normal[i]
            # 用鲁棒 Huber 回归拟合该边直线；fitLine 返回 (vx, vy, x0, y0)。
            vx, vy, x, y = cv2.fitLine(samples, cv2.DIST_HUBER, 0, 0.01, 0.01).reshape(4)
            # 直线 ax+by+c=0。
            lines.append(np.array([-vy, vx, vy * x - vx * y], np.float64))
            refitted += 1
        if refitted < 3:
            return None
        lines = np.asarray(lines)
        # 相邻两条直线（roll 1 使其错位一位）叉积求交点：交点齐次坐标 = 两直线系数叉积。
        crosses = np.cross(np.roll(lines, 1, axis=0), lines)
        if np.any(np.abs(crosses[:, 2]) < 1e-8):
            return None                                                    # 有平行边 → 退化
        refined = (crosses[:, :2] / crosses[:, 2:]).astype(np.float32)
        if (np.linalg.norm(refined - q, axis=1).max() > radius * 2.8 or
                not self._valid_quad(refined, gray.shape)):
            return None                                                    # 跑太远或形状不合法
        return refined

    def _four_edges_complete(self, candidate):
        return (min(candidate['coverage']) >= self.cfg.MIN_EDGE_COVERAGE and
                candidate['contrast'] >= self.cfg.MIN_EDGE_CONTRAST)

    def _rescue(self, gray, q, kind):
        radius = getattr(self.cfg, 'REFINE_RADIUS_WIDE', 20)
        if radius <= self.cfg.REFINE_RADIUS:
            return None
        q = self._refine(gray, q, kind, radius)
        candidate = self._verify(gray, q, kind) if q is not None else None
        # These two extra gates are ONLY for wide recovery; keep normal contrast sensitivity.
        if candidate is not None and candidate['contrast'] >= getattr(self.cfg, 'RECOVER_MIN_CONTRAST', 80) and max(r['dark'] for r in candidate['edge_profile']) <= getattr(self.cfg, 'RECOVER_MAX_DARK', 100):
            candidate['recovered'] = True
            return candidate
        return None

    def _accept(self, gray, q, kind):
        return self._verify(gray, q, kind) or self._rescue(gray, q, kind)

    def _finalize(self, gray, candidate, iterations):
        for _ in range(iterations):
            q = self._refine(gray, candidate['quad'], candidate['kind'], self.cfg.REFINE_RADIUS)
            precise = self._verify(gray, q, candidate['kind']) if q is not None else None
            if precise is None:
                break
            shift = np.linalg.norm(q-candidate['quad'], axis=1).max()
            candidate = precise
            if shift < .2:
                break
        return candidate

    def _search(self, gray, roi=None):
        self._search_count += 1
        cap = None if getattr(self.cfg, 'SEARCH_EXHAUSTIVE', True) else self.cfg.SEARCH_MAX_PROPOSALS
        # A local cap cannot veto the required full search. Counters still accumulate.
        proposals_at_start = self._proposals
        best = None
        for level in (0, 1):
            candidates = self.locator.locate(gray, roi, level)
            self._mask_count += self.locator.last_stats['masks']
            for q, kind in candidates:
                if cap is not None and self._proposals-proposals_at_start >= cap:
                    break
                self._proposals += 1
                candidate = self._accept(gray, q, kind)
                if candidate is None:
                    continue
                if self._four_edges_complete(candidate) and not candidate.get('recovered'):
                    return self._finalize(gray, candidate, getattr(self.cfg, 'REFINE_ITERATIONS', 2))
                if best is None or candidate['rank'] > best['rank']:
                    best = candidate
        return self._finalize(gray, best, getattr(self.cfg, 'REFINE_ITERATIONS', 2)) if best else None

    def _continuous_time(self, newer, older):
        return older is not None and np.isfinite([newer, older]).all() and 0 < newer-older <= getattr(self.cfg, 'PREDICT_MAX_GAP_S', .25)

    def _motion_delta(self, timestamp=None):
        timestamp = time.perf_counter() if timestamp is None else timestamp
        if not getattr(self.cfg, 'ENABLE_MOTION_PREDICT', True) or self._prev_quad is None or self._track is None or self._lost:
            return None
        if not self._continuous_time(timestamp, self._timestamp) or not self._continuous_time(self._timestamp, self._prev_timestamp):
            return None
        delta = (self._track['quad']-self._prev_quad)*(timestamp-self._timestamp)/(self._timestamp-self._prev_timestamp)
        step, cap = np.linalg.norm(delta, axis=1).max(), getattr(self.cfg, 'PREDICT_MAX_PX', 60.)
        if cap > 0 and step > cap:
            delta *= cap/step
        return delta

    def _filter_center(self, raw, timestamp):
        if getattr(self.cfg, 'KALMAN_ENABLE', True):
            self._smooth = self._kalman.update(raw, timestamp)
        else:
            alpha = self.cfg.CENTER_ALPHA
            if self._smooth is None or alpha <= 0 or np.linalg.norm(raw-self._smooth) > self.cfg.FILTER_RESET_PX or not self._continuous_time(timestamp, self._timestamp):
                self._smooth = raw.copy()
            else:
                self._smooth += alpha*(raw-self._smooth)
        return self._smooth

    @staticmethod
    def _midline_mm(profile, extent_mm):
        profile = np.asarray(profile, float)
        low, high = np.percentile(profile, [10, 65])
        if high-low < 5:
            raise ValueError('front view lacks dark-band contrast')
        threshold = (low+high)/2
        n = len(profile)
        # Only the first/last third can contain the tape inner edges, not printed rings.
        def crossing(indices, rising):
            for i in indices:
                a, b = profile[i:i+2]
                if (a <= threshold < b) if rising else (a >= threshold > b):
                    return i+(threshold-a)/(b-a)
            raise ValueError('front-view edge missing')
        left = crossing(range(n//3), True)
        right = crossing(range(n-2, 2*n//3, -1), False)
        return (left+right)*.5*extent_mm/(n-1)

    def _center(self, gray, candidate):
        w, h, matrix = candidate['width_mm'], candidate['height_mm'], candidate['rect_to_image']
        raw = project_points(matrix, [[w/2, h/2]])[0]
        front = int(getattr(self.cfg, 'FRONT_VIEW_SHORT_PX', 0))
        if front > 0:
            view = warp_front_view(gray, matrix, w, h, front)
            candidate['front_view'] = view
            try:
                center_mm = [self._midline_mm(np.median(view[view.shape[0]//3:2*view.shape[0]//3], axis=0), w),
                             self._midline_mm(np.median(view[:, view.shape[1]//3:2*view.shape[1]//3], axis=1), h)]
                center = project_points(matrix, [center_mm])[0]
                error = float(np.linalg.norm(center-raw))
                candidate.update(front_view_center=center, front_view_error_px=error,
                                 geometry_consistent=error <= getattr(self.cfg, 'FRONT_VIEW_MAX_ERROR_PX', 5.))
            except ValueError:
                candidate.update(front_view_center=None, geometry_consistent=False)
        return raw

    def process(self, gray, timestamp=None):
        started = time.perf_counter()
        timestamp = started if timestamp is None else float(timestamp)
        g = np.asarray(gray)
        if g.ndim == 3 and g.shape[2] == 1:
            g = g[:, :, 0]
        if g.ndim != 2 or g.dtype != np.uint8:
            raise ValueError('process expects a uint8 grayscale image')
        if not np.isfinite(timestamp):
            raise ValueError('non-finite timestamp')
        if self._frame_shape != g.shape:
            self.reset()
            self._frame_shape = g.shape
        self._search_count = self._mask_count = self._proposals = 0
        candidate, stage = None, 'full_search'
        if g.size == 0:
            self.reset()
            return dict(valid=False, stage='empty_frame', masks=0, search_count=0, proposals=0, detector_ms=0.)
        delta = self._motion_delta(timestamp)
        if self._track is not None:
            kind = self._track['kind']
            guesses = [self._track['quad']+delta] if delta is not None else []
            guesses.append(self._track['quad'])
            for guess in guesses:
                q = self._refine(g, guess, kind, self.cfg.TRACK_RADIUS)
                candidate = self._verify(g, q, kind) if q is not None else None
                if candidate is not None:
                    stage = 'track'
                    break
        if candidate is None and self._track is not None and self._roi_retry:
            q = self._track['outer_quad'] + (delta if delta is not None else 0)
            lo, hi = q.min(axis=0), q.max(axis=0)
            pad = np.maximum((hi-lo)*self.cfg.ROI_PADDING, 24)
            lo = np.maximum(0, np.floor(lo-pad)).astype(int)
            hi = np.minimum([g.shape[1], g.shape[0]], np.ceil(hi+pad)).astype(int)
            self._roi_retry = False
            candidate = self._search(g, tuple(map(int, (*lo, *(hi-lo)))))
            stage = 'roi_search'
        # Never use elif: ROI failure must still attempt a complete full-frame search.
        if candidate is None:
            candidate, stage = self._search(g), 'full_search'
        if candidate is None:
            self._smooth = self._prev_quad = self._prev_timestamp = None
            self._kalman.reset()
            self._lost += 1
            result = dict(valid=False, stage='no_black_frame')
        else:
            previous = self._track
            raw = self._center(g, candidate)
            center = self._filter_center(raw, timestamp)
            self._prev_quad = self._prev_timestamp = None
            if previous is not None and not self._lost and previous['kind'] == candidate['kind'] and self._continuous_time(timestamp, self._timestamp):
                q = previous['quad']
                if stage != 'track':
                    q = min((np.roll(q, i, axis=0) for i in range(4)), key=lambda p: np.sum((p-candidate['quad'])**2))
                self._prev_quad, self._prev_timestamp = q.copy(), self._timestamp
            self._track, self._timestamp, self._lost, self._roi_retry = candidate, timestamp, 0, True
            key = (candidate['width_mm'], candidate['height_mm'], self.cfg.CIRCLE_RADIUS_MM, self.cfg.CIRCLE_SAMPLES)
            if key not in self._circle_cache:
                self._circle_cache[key] = standard_circle(*key)
            result = dict(candidate)
            result.update(valid=True, stage=stage, u=float(center[0]), v=float(center[1]),
                          raw_u=float(raw[0]), raw_v=float(raw[1]), quad=candidate['outer_quad'],
                          detected_quad=candidate['quad'], outer_inferred=candidate['kind']=='inner',
                          circle_points=project_points(candidate['rect_to_image'], self._circle_cache[key]))
        result.update(threshold=None, search_count=self._search_count, masks=self._mask_count,
                      thresholds=self._mask_count, proposals=self._proposals,
                      detector_ms=(time.perf_counter()-started)*1000)
        self.last = result
        return result


def draw_result(gray, result, show_width=480, top_offset=0):
    """缩小后再转 BGR 和描边，输出坐标仍为相机原图坐标。

    top_offset：屏幕上方的文字整体下移量（像素）。激光读数面板占用左上角时由
    main.py 传 vision.laser_spot.PANEL_H，避免两处文字叠在一起。

    绘制元素（均按 scale 换算到缩小图坐标，避免在原始分辨率上画）：
        · 绿色外框 quad、青色内框 inner_quad
        · 洋红四角圆点 + "P1..P4 (x,y)" 标签（原图坐标）
        · 蓝色靶心十字 + "(u,v) stage 耗时" 文字
    返回 BGR 画布，交由 _Preview.render 再 letterbox 到屏幕画布。
    """
    g = np.asarray(gray).squeeze()
    scale = min(1.0, float(show_width) / g.shape[1])        # 原图 → 缩小图 的比例
    if scale < 1:
        g = cv2.resize(g, (int(round(g.shape[1] * scale)), int(round(g.shape[0] * scale))))
    canvas = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    if not result.get("valid"):
        # 无效：只提示一行红字，不画任何框（过期结果已在 main._display_result 降级）。
        cv2.putText(canvas, "NO BLACK RECTANGLE", (8, 24 + top_offset), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 255), 1)
        return canvas
    q = np.rint(result["quad"] * scale).astype(np.int32)          # 外框（缩小图坐标）
    iq = np.rint(result["inner_quad"] * scale).astype(np.int32)   # 内框（缩小图坐标）
    cv2.polylines(canvas, [q], True, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.polylines(canvas, [iq], True, (0, 220, 220), 1)
    for i, (x, y) in enumerate(result["quad"]):
        p = tuple(q[i])
        cv2.circle(canvas, p, 4, (255, 0, 255), -1)               # 洋红实心点
        # 标签用**原图坐标**（x,y），便于与日志/回算对照；位置做边界夹取防出画。
        label = "P%d (%d,%d)" % (i + 1, round(float(x)), round(float(y)))
        tw = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .36, 1)[0][0]
        pos = (max(0, min(int(p[0]) + 5, canvas.shape[1] - tw - 1)),
               max(12, min(int(p[1]) - 7, canvas.shape[0] - 5)))
        cv2.putText(canvas, label, pos, cv2.FONT_HERSHEY_SIMPLEX, .36, (255, 255, 0), 1, cv2.LINE_AA)
    c = tuple(np.rint(np.array([result["u"], result["v"]]) * scale).astype(int))
    cv2.drawMarker(canvas, c, (255, 0, 0), cv2.MARKER_CROSS, 14, 2)   # 蓝色靶心十字
    cv2.putText(canvas, "(%d,%d) %s %.1fms" % (round(result["u"]), round(result["v"]),
                result["stage"], result["detector_ms"]), (8, 22 + top_offset), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 220, 0), 1)
    return canvas
'''

# ===== vision.laser_spot  (vision/laser_spot.py) =====
_MOD_SRC['vision.laser_spot'] = r'''
"""激光光斑读数：标称位置附近的小 ROI 内找最亮点，输出亚像素质心与到靶心偏差。

为什么只搜一个小 ROI：相机与激光笔刚性固连（同装在俯仰平台上），光斑在图像里的
位置近似固定 —— 2026-10-07 现场一帧实测约 (294, 202)（640x480），背景灰度 145、
峰值 177。因此不必全图找"最亮的东西"（那会把白墙高光、灯光、反光都算进去），
只要在标称位置周围 LASER_ROI_RADIUS 像素内做一次阈值比较 + 几十个像素的质心，
代价可忽略。标称值只决定"往哪儿看"，找到的真实位置仍然逐次更新。

为什么要限频：即使单次只有零点几毫秒，也不该塞进每帧关键路径。process() 内部
按 LASER_INTERVAL_S 限频（默认 0.30s ≈ 每秒 3 次），两次更新之间直接沿用上次
结果（updated=False），所以显示仍然是每帧画的 —— 画面流畅度不受影响。

两种取位方式（`LASER_MODE`），云台侧拿到的 (u,v) 语义完全相同，只是来源不同：
    "fixed"（默认、当前方案） 光斑位置写死 = (LASER_U_NOMINAL, LASER_V_NOMINAL)，
        全程不做任何图像检测；每帧唯一的运算是"黑框中心 − 该常量"的两次减法。
        之所以可行：相机与激光刚性同转 → 光斑在图像里的位置本来就是常量（需求文档 §7.1）。
    "detect"        每 LASER_INTERVAL_S 在标称位附近 ROI 实测一次（自校准，吸收漂移）。
        只在需要重新标定 U/V、或想验证"写死的值还准不准"时临时启用。

输出契约（dict）：
    found    bool    本次是否有可信光斑（未更新时间沿用的仍是它）
    updated  bool    本次是否真的重算了（False = 限频沿用 / fixed 模式）
    u, v     float   光斑位置，相机原图坐标。**调用方必须先看 found**：found=False 时
                     u/v 只是标称值的占位，绝不能当测量结果用（deviation() 已代查）。
    peak     float   ROI 最大灰度；bg 为 ROI 背景灰度（隔行子采样中位数）
    area     int     超过阈值的像素数
    reason   str     原因码：ok / fixed / no_peak / too_big / too_small / jumped / roi
    ms       float   本次计算耗时（毫秒），未更新时为 0
"""
import time

import cv2
import numpy as np

_PANEL_BG = (32, 32, 32)
PANEL_LINE = 18
PANEL_H = 2 * PANEL_LINE + 10   # 面板高度（显示画布坐标）；main 用它把框文字整体下移
_TEXT_C1 = (255, 255, 0)     # BGR 青：光斑状态行
_TEXT_C2 = (120, 255, 120)   # BGR 浅绿：偏差行（有效）
_TEXT_C3 = (120, 120, 255)   # BGR 浅红：偏差行（无靶纸）
_MARK = (0, 0, 255)          # BGR 红：光斑标记（跟激光同色）
_LINK = (0, 160, 255)        # BGR 橙：光斑->靶心连线


class LaserSpotDetector:
    """标称位置附近找最亮点，亚像素质心 + 限频 + 跳变门限。"""

    def __init__(self, config):
        self.cfg = config
        self.mode = str(getattr(config, "LASER_MODE", "detect")).strip().lower()
        self.nominal = (float(config.LASER_U_NOMINAL), float(config.LASER_V_NOMINAL))
        self.interval = float(getattr(config, "LASER_INTERVAL_S", 0.30))
        self.alpha = float(getattr(config, "LASER_SMOOTH_ALPHA", 0.0))
        self.max_jump = float(getattr(config, "LASER_MAX_JUMP_PX", 40.0))
        self.last = dict(found=False, updated=False, u=0.0, v=0.0, peak=0.0, bg=0.0,
                         area=0, reason="init", ms=0.0)
        self._t = None
        self._uv = None

    # ------------------------------------------------------------------ 内部
    def _detect(self, g):
        """在标称位附近的方形 ROI 内做一次"阈值 + 加权质心"，返回 (u,v)。

        判据链（任一不满足即判无光斑，并带 reason 便于排障）：
            峰值 >= max(LASER_MIN_PEAK, 背景 + LASER_MIN_CONTRAST)  → 否则 no_peak
            亮点像素数在 [LASER_MIN_PIXELS, LASER_MAX_PIXELS] 内    → 过大 too_big（挡高光）/过小 too_small
        质心用 (灰度−阈值) 加权，越亮的像素权重越大 → 亚像素级、抗噪。
        """
        cfg = self.cfg
        H, W = g.shape[:2]
        cx, cy = float(cfg.LASER_U_NOMINAL), float(cfg.LASER_V_NOMINAL)
        r = int(cfg.LASER_ROI_RADIUS)
        # ROI = 标称位 ± r，并夹取到图像内（标称值决定"往哪儿看"，不决定结果）。
        x0 = max(0, int(cx - r))
        y0 = max(0, int(cy - r))
        x1 = min(W, int(cx + r) + 1)
        y1 = min(H, int(cy + r) + 1)
        if x1 - x0 < 5 or y1 - y0 < 5:
            return dict(found=False, reason="roi", peak=0.0, bg=0.0, area=0, u=cx, v=cy)
        roi = g[y0:y1, x0:x1]
        # 背景用 1/4 子采样中位数：抗光斑/噪点污染，又不必对 16k 像素排序。
        bg = float(np.median(roi[::4, ::4]))
        peak = float(roi.max())
        thr = max(float(cfg.LASER_MIN_PEAK), bg + float(cfg.LASER_MIN_CONTRAST))
        if peak < thr:
            return dict(found=False, reason="no_peak", peak=peak, bg=bg, area=0, u=cx, v=cy)
        mask = roi >= thr
        area = int(mask.sum())
        if area > int(cfg.LASER_MAX_PIXELS):
            # 大片高光/反光，不是激光点（激光点在 165cm 处只有几个像素）。
            return dict(found=False, reason="too_big", peak=peak, bg=bg, area=area, u=cx, v=cy)
        if area < int(cfg.LASER_MIN_PIXELS):
            return dict(found=False, reason="too_small", peak=peak, bg=bg, area=area, u=cx, v=cy)
        ys, xs = np.nonzero(mask)
        w = roi[ys, xs].astype(np.float32) - thr
        total = float(w.sum())
        if total <= 0:
            return dict(found=False, reason="flat", peak=peak, bg=bg, area=area, u=cx, v=cy)
        # 加权质心：权重 = 灰度−阈值；坐标加 ROI 原点 (x0,y0) 回到全图坐标。
        return dict(found=True, reason="ok", peak=peak, bg=bg, area=area,
                    u=x0 + float((xs * w).sum() / total), v=y0 + float((ys * w).sum() / total))

    # ------------------------------------------------------------------ 对外
    @staticmethod
    def _fixed_dict(nominal):
        return dict(found=True, updated=False, u=nominal[0], v=nominal[1],
                    peak=0.0, bg=0.0, area=0, reason="fixed", ms=0.0)

    def static_state(self):
        """fixed 模式的常量结果：光斑位置写死，不做任何图像运算。

        main.py 在 fixed 模式下只在开机取一次这个 dict，之后每帧复用；每帧唯一的
        运算是 deviation() 里的两次减法（黑框中心 − 该常量）。
        """
        return self._fixed_dict(self.nominal)

    def process(self, gray, now=None):
        """每帧调用；fixed 模式恒返回标定常量，detect 模式内部按 LASER_INTERVAL_S 限频。

        限频逻辑：距上次实测不足 interval 时**直接沿用上次结果**（updated=False, ms=0），
        因此每帧调用的代价近似为零；真正重算时依次做跳变门限与平滑。
        """
        if self.mode == "fixed":
            return self.static_state()
        now = time.perf_counter() if now is None else float(now)
        if self._t is not None and (now - self._t) < self.interval:
            out = dict(self.last)                 # 沿用上次：位置不变，仅标记未更新
            out["updated"] = False
            out["ms"] = 0.0
            return out
        self._t = now
        g = np.asarray(gray)
        if g.ndim == 3 and g.shape[2] == 1:
            g = g[:, :, 0]
        started = time.perf_counter()
        res = self._detect(g)
        res["ms"] = (time.perf_counter() - started) * 1000.0
        res["updated"] = True
        if res["found"] and self._uv is not None and self.max_jump > 0:
            if abs(res["u"] - self._uv[0]) > self.max_jump or abs(res["v"] - self._uv[1]) > self.max_jump:
                # 单次跳太远：判为误检（高光/干扰物），保留上次位置。
                keep = dict(res)
                keep.update(found=True, reason="jumped", u=self._uv[0], v=self._uv[1])
                self.last = keep
                return keep
        if res["found"]:
            if self._uv is not None and 0.0 < self.alpha < 1.0:
                # 位置一阶低通（仅 detect 模式）；alpha=0 时关闭平滑，直接用最新质心。
                self._uv = (self._uv[0] + self.alpha * (res["u"] - self._uv[0]),
                            self._uv[1] + self.alpha * (res["v"] - self._uv[1]))
            else:
                self._uv = (res["u"], res["v"])
            res["u"], res["v"] = self._uv
        self.last = res
        return res


def deviation(laser, result, use_raw=False):
    """靶心相对光斑的偏差：dx>0 靶心在光斑右侧，dy>0 靶心在光斑下方。

    约定取自需求文档 §7.2（dx = u靶心 − u光斑），与后续串口直控云台同一套符号：
    两轴都是"误差为正发 CCW"，因此屏幕上这个符号可以直接进控制律。

    use_raw：闭环必须用这一档（True）—— 取 result["raw_u"]/["raw_v"]（未平滑）。
    result["u"]/["v"] 是经 CENTER_ALPHA 一阶低通的值，只适合显示；拿它闭环会附加
    约 28ms 滞后（需求文档 §5.2），底盘运动时直接变成瞄准误差。
    激光丢失或黑框无效时返回 None。
    """
    if not laser or not laser.get("found"):
        return None                        # ★ 必须先看 found：found=False 时 u/v 只是标称占位
    if not result or not result.get("valid"):
        return None
    if use_raw and result.get("raw_u") is not None and result.get("raw_v") is not None:
        tx, ty = float(result["raw_u"]), float(result["raw_v"])   # 闭环档：未平滑中心
    else:
        tx, ty = float(result["u"]), float(result["v"])           # 显示档：CENTER_ALPHA 平滑中心
    return (tx - float(laser["u"]), ty - float(laser["v"]))       # dx=靶心−光斑, dy 同理


def draw_laser_overlay(canvas, laser, result=None, scale=1.0, use_raw=False):
    """在显示画布左上角画激光读数面板 + 光斑标记 + 到靶心的连线。

    use_raw 与 deviation() 保持一致：True 时连线终点与 dx/dy 都用未平滑的 raw 中心，
    屏幕上看到的就是控制律真正在用的那个点。

    scale：**原图坐标 → 画布坐标** 的比例（main.py 传 render_width/原图宽）。
    本函数内部所有坐标（光斑 p、连线终点 c）都要乘 scale，才能与 draw_result 画的框对齐。
    **注意**：面板本身（左上角矩形与文字）用画布固定像素坐标，不随 scale 缩放。

    返回面板占用的高度（像素，显示画布坐标），调用方据此把 draw_result 的文字
    整体下移，避免和本面板重叠。
    """
    if not laser:
        return 0
    found = bool(laser.get("found"))
    # 第 1 行：光斑位置与来源（fixed=标定常量 / 实测 peak,area / 丢失原因）。
    if laser.get("reason") == "fixed":
        line1 = "LASER (%d,%d) [fixed/calib]" % (round(laser["u"]), round(laser["v"]))
    elif found:
        line1 = "LASER (%d,%d) peak=%d area=%d" % (
            round(laser["u"]), round(laser["v"]), round(laser["peak"]), laser.get("area", 0))
    else:
        line1 = "LASER -- [%s] peak=%d bg=%d" % (
            laser.get("reason", "?"), round(laser.get("peak", 0.0)), round(laser.get("bg", 0.0)))
    # 第 2 行：光斑→靶心偏差（用同一 use_raw 口径，与闭环一致）。
    dv = deviation(laser, result, use_raw=use_raw)
    if dv is not None:
        line2 = "D-CENTER dx=%+.1f dy=%+.1f px" % (dv[0], dv[1])
        line2_color = _TEXT_C2                       # 浅绿：有效
    elif result is not None and result.get("valid"):
        line2 = "D-CENTER -- (laser lost)"
        line2_color = _TEXT_C3                       # 浅红：有靶心无光斑
    else:
        line2 = "D-CENTER -- (no black rect)"
        line2_color = _TEXT_C3                       # 浅红：无靶心
    # 面板宽度按两行文字较宽者自适应（固定画布像素，不随 scale 缩放）。
    w = max(cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, .45, 1)[0][0] for t in (line1, line2))
    cv2.rectangle(canvas, (2, 2), (min(canvas.shape[1] - 1, w + 14), PANEL_H - 2), _PANEL_BG, -1)
    cv2.putText(canvas, line1, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, .45, _TEXT_C1, 1, cv2.LINE_AA)
    cv2.putText(canvas, line2, (8, 18 + PANEL_LINE), cv2.FONT_HERSHEY_SIMPLEX, .45, line2_color, 1, cv2.LINE_AA)
    if not found:
        return PANEL_H                              # 无光斑：只留面板，不画标记/连线
    # 光斑标记：原图坐标 × scale → 画布坐标。
    p = (int(round(laser["u"] * scale)), int(round(laser["v"] * scale)))
    if result is not None and result.get("valid"):
        # 连线终点 = 靶心，同样按 use_raw 选 raw 或平滑中心，再 × scale。
        if use_raw and result.get("raw_u") is not None and result.get("raw_v") is not None:
            c = (int(round(float(result["raw_u"]) * scale)), int(round(float(result["raw_v"]) * scale)))
        else:
            c = (int(round(float(result["u"]) * scale)), int(round(float(result["v"]) * scale)))
        cv2.line(canvas, p, c, _LINK, 1, cv2.LINE_AA)   # 橙色连线：光斑 → 靶心
    cv2.circle(canvas, p, 7, _MARK, 1, cv2.LINE_AA)     # 红色圆圈
    cv2.drawMarker(canvas, p, _MARK, cv2.MARKER_CROSS, 5, 1, cv2.LINE_AA)   # 红十字
    return PANEL_H
'''

# ===== vision.stepper_link  (vision/stepper_link.py) =====
_MOD_SRC['vision.stepper_link'] = r'''
"""ZDT Emm_V5.0 双串口链路（协议帧构造 + 非阻塞收发 + 链路健康）。

本模块可以在 PC 上导入；只有创建 StepperLink 时才会导入 MaixPy。
协议使用固定 0x6B 校验、大端参数、相对位置命令（`MaixCAM2串口直控云台需求文档.md` §6）。

帧构造是纯函数（``cmd_*``），返回帧解析也做成纯函数 + 可注入的 ``_ingest``，
这样即使没有 MaixPy 也能在 PC 上逐字节回归。

返回帧：[地址][功能码][状态][校验]，状态 `0x02`=成功、`0xE2`=未使能/堵转、
`0x00 0xEE`=错误命令（见 §6.1）。
"""

import time
import math

CR = 0x6B
STATUS_OK = 0x02
STATUS_E2 = 0xE2


def _frame(addr, func, data=()):
    return bytes([int(addr) & 0xFF, int(func) & 0xFF] +
                 [int(x) & 0xFF for x in data] + [CR])


def cmd_enable(addr, enable=True, sync=False):
    return _frame(addr, 0xF3, (0xAB, 1 if enable else 0, 1 if sync else 0))


def cmd_speed(addr, ccw, rpm, acc=0, sync=False):
    rpm = max(0, min(5000, int(rpm)))
    return _frame(addr, 0xF6, (1 if ccw else 0, rpm >> 8, rpm,
                               int(acc), 1 if sync else 0))


def cmd_position(addr, ccw, rpm, pulses, acc=0, absolute=False, sync=False):
    rpm = max(0, min(5000, int(rpm)))
    pulses = max(0, min(0xFFFFFFFF, int(abs(pulses))))
    return _frame(addr, 0xFD, (
        1 if ccw else 0, rpm >> 8, rpm, int(acc),
        pulses >> 24, pulses >> 16, pulses >> 8, pulses,
        1 if absolute else 0, 1 if sync else 0))


def cmd_stop_now(addr, sync=False):
    return _frame(addr, 0xFE, (0x98, 1 if sync else 0))


def cmd_sync_motion(addr=0):
    return _frame(addr, 0xFF, (0x66,))


def cmd_read(addr, func):
    return _frame(addr, func, ())


def frame_status(frame):
    """返回帧第 3 字节 = 命令状态；帧太短返回 None。"""
    return int(frame[2]) if frame is not None and len(frame) >= 3 else None


def _readback_pin(pinmap, pin):
    """回读引脚当前实际功能（AGENTS.md §6.4：'某轴不通'的第一手证据）。失败返回 '?'。"""
    try:
        return pinmap.get_pin_function(pin)
    except Exception:
        return "?"


class StepperLink:
    """两条独立 UART 链路；任一轴打开失败不会阻塞另一轴。"""

    def __init__(self, cfg, clock=None):
        self._clock = clock if clock is not None else time
        self.cfg = cfg
        self._ports = {}
        self._rxbuf = {"yaw": bytearray(), "pitch": bytearray()}
        self._stats = {"tx": 0, "rx": 0, "ok": 0, "e2": 0, "err": 0,
                       "timeout": 0, "yaw": True, "pitch": True}
        self._last_status = {"yaw": None, "pitch": None}
        self._last_cmd = {"yaw": 0.0, "pitch": 0.0}
        self._last_ack = {"yaw": 0.0, "pitch": 0.0}
        self._pending_ack_since = {"yaw": None, "pitch": None}
        self._timeout_count = {"yaw": 0, "pitch": 0}
        self._e2_times = []
        self.fault_reason = None
        try:
            from maix import uart, pinmap
        except Exception as exc:
            self._stats["err"] += 1
            self._stats["yaw"] = self._stats["pitch"] = False
            self._init_error = exc
            return
        # ★ 实测轴映射定案（AGENTS.md §2）：yaw=UART2/B0B1/ttyS2/ID2，pitch=UART0/A28A29/ttyS0/ID1。
        #   两轴都要显式 pinmap（U0 走空表会静默不通）；ID 跟着"电机"走，不跟着端口。
        pin_sets = {
            "yaw": getattr(cfg, "YAW_PINS", {"B0": "UART2_TX", "B1": "UART2_RX"}),
            "pitch": getattr(cfg, "PITCH_PINS", {"A28": "UART0_TX", "A29": "UART0_RX"}),
        }
        devices = {"yaw": (getattr(cfg, "YAW_DEV", "/dev/ttyS2"),
                             getattr(cfg, "YAW_BAUD", 115200)),
                   "pitch": (getattr(cfg, "PITCH_DEV", "/dev/ttyS0"),
                               getattr(cfg, "PITCH_BAUD", 115200))}
        strict = bool(getattr(cfg, "PINMAP_STRICT", False))
        flush_rx = bool(getattr(cfg, "FLUSH_RX_ON_START", False))
        for axis in ("yaw", "pitch"):
            try:
                for pin, func in pin_sets[axis].items():
                    try:
                        result = pinmap.set_pin_function(pin, func)
                        failed = (result is not None and hasattr(result, "value")
                                  and result.value != 0)
                    except Exception as exc:              # 无该引脚 / 固件不支持
                        result, failed = exc, True
                    if failed:
                        message = "pinmap %s=%s failed: %s" % (pin, func, result)
                        if strict:
                            raise RuntimeError(message)
                        # 非严格模式：只告警，仍尝试打开 UART —— 否则真故障会被藏成"打不开"。
                        print("[gimbal] WARN " + message)
                    else:
                        print("[gimbal] pinmap %s=%s ok (readback=%s)" %
                              (pin, func, _readback_pin(pinmap, pin)))
                self._ports[axis] = uart.UART(*devices[axis])
                if flush_rx:
                    # U0(pitch) 是系统日志口：开机先丢掉内核启动日志/终端回显等 RX 残留。
                    self._flush_rx(axis)
            except Exception as exc:
                self._stats[axis] = False
                self._stats["err"] += 1
                print("[gimbal] %s UART open failed: %s" % (axis, exc))

    # ------------------------------------------------------------- 发送
    def _flush_rx(self, axis):
        """清空该轴串口开机前的 RX 残留（U0=系统日志口，会混内核启动日志/终端回显）。"""
        port = self._ports.get(axis)
        if port is None:
            return
        try:
            available = int(port.available(0))
            if available > 0:
                port.read(min(available, int(getattr(self.cfg, "UART_RX_MAX_BYTES", 256))), timeout=0)
        except Exception:
            pass
        self._rxbuf[axis] = bytearray()

    def _write(self, axis, payload, now=None):
        port = self._ports.get(axis)
        if port is None:
            return False
        try:
            n = port.write(payload)
            if n is not None and int(n) != len(payload):
                raise IOError("write returned %s" % n)
            self._stats["tx"] += 1
            self._last_cmd[axis] = self._clock.perf_counter() if now is None else float(now)
            pending = getattr(self, "_pending_ack_since", None)
            if pending is None:
                pending = self._pending_ack_since = {"yaw": None, "pitch": None}
            if pending[axis] is None:
                pending[axis] = self._last_cmd[axis]
            return True
        except Exception as exc:
            self._stats["err"] += 1
            print("[gimbal] %s TX failed: %s" % (axis, exc))
            return False

    def enable(self, axis=None, now=None):
        axes = ("yaw", "pitch") if axis is None else (axis,)
        ok = True
        for name in axes:
            ok = self._write(name, cmd_enable(self._addr(name)), now=now) and ok
        return ok

    def move(self, axis, ccw, pulses, rpm, acc=0, now=None):
        if axis not in ("yaw", "pitch"):
            raise ValueError("axis must be yaw or pitch")
        if int(pulses) <= 0:
            return False
        return self._write(axis, cmd_position(self._addr(axis), ccw, rpm, pulses, acc=acc),
                           now=now)

    def speed(self, axis, ccw, rpm, acc=0, now=None):
        return self._write(axis, cmd_speed(self._addr(axis), ccw, rpm, acc=acc), now=now)

    def stop(self, axis=None, now=None):
        axes = ("yaw", "pitch") if axis is None else (axis,)
        ok = True
        for name in axes:
            ok = self._write(name, cmd_stop_now(self._addr(name)), now=now) and ok
        return ok

    def _addr(self, axis):
        return int(getattr(self.cfg, "ADDR_YAW" if axis == "yaw" else "ADDR_PITCH",
                           1 if axis == "yaw" else 2))

    # ------------------------------------------------------------- 接收
    def _drain(self):
        """非阻塞读走两条串口现有字节，切出完整返回帧。"""
        frames = []
        for axis, port in tuple(self._ports.items()):
            try:
                available = int(port.available(0))
                if available > 0:
                    data = port.read(min(available, int(getattr(self.cfg, "UART_RX_MAX_BYTES", 256))), timeout=0)
                    if data:
                        self._rxbuf[axis].extend(bytes(data))
                buf = self._rxbuf[axis]
                limit = max(4, int(getattr(self.cfg, "UART_RX_BUFFER_MAX_BYTES", 1024)))
                if len(buf) > limit:
                    del buf[:-limit]
                while len(buf) >= 4:
                    if buf[0] != self._addr(axis):
                        del buf[0]
                        continue
                    end = buf.find(bytes([CR]), 1)
                    if end < 0:
                        break
                    if end < 3:
                        del buf[0]
                        continue
                    frames.append((axis, bytes(buf[:end + 1])))
                    del buf[:end + 1]
                    self._stats["rx"] += 1
            except Exception:
                self._stats["err"] += 1
        return frames

    def _ingest(self, frames, now):
        """解析返回帧：统计成功/E2、E2 自动补使能、E2 洪水与 ACK 超时判障。

        返回 dict(ok, e2, reenabled, fault, reason)。本方法不碰 MaixPy，可直接单测。
        """
        now = float(now)
        if not hasattr(self, "_pending_ack_since"):
            self._pending_ack_since = {"yaw": None, "pitch": None}
        out = {"frames": list(frames), "ok": 0, "e2": 0,
               "reenabled": [], "fault": False, "reason": None}
        e2_axes = []
        for axis, frame in frames:
            status = frame_status(frame)
            self._last_status[axis] = status
            if status is None:
                continue
            # 凡是能解析出状态的帧都证明链路活着 -> 刷新该轴 ACK 时刻。
            self._last_ack[axis] = now
            self._pending_ack_since[axis] = None
            self._timeout_count[axis] = 0
            if status == STATUS_OK:
                out["ok"] += 1
                self._stats["ok"] += 1
            elif status == STATUS_E2:
                out["e2"] += 1
                self._stats["e2"] += 1
                e2_axes.append(axis)
                self._e2_times.append(now)
        if not bool(getattr(self.cfg, "ENABLE_ACK_WATCHDOG", True)):
            return out
        if e2_axes and bool(getattr(self.cfg, "E2_REENABLE", True)):
            for axis in sorted(set(e2_axes)):
                if self.enable(axis, now=now):
                    out["reenabled"].append(axis)
        window = float(getattr(self.cfg, "E2_WINDOW_S", 2.0))
        self._e2_times = [t for t in self._e2_times if now - t <= window]
        limit = int(getattr(self.cfg, "E2_FAULT_COUNT", 10))
        if limit > 0 and len(self._e2_times) > limit:
            out["fault"] = True
            out["reason"] = "e2_flood(%d in %.1fs)" % (len(self._e2_times), window)
            self.fault_reason = out["reason"]
            return out
        # ACK 硬超时：只在收到过确认帧（Response=Receive/Reached/Both）之后才生效，
        # Response=None 时永远收不到确认帧，该判据自动不启用，不会误判 FAULT。
        timeout_ms = float(getattr(self.cfg, "RX_TIMEOUT_MS", 0) or 0)
        if timeout_ms > 0 and self._stats["ok"] > 0:
            for axis in self._ports:
                pending = self._pending_ack_since.get(axis)
                if pending is None and self._last_cmd[axis] > self._last_ack[axis]:
                    pending = self._last_cmd[axis]
                if pending is None or now - pending <= timeout_ms / 1000.0:
                    self._timeout_count[axis] = 0
                    continue
                if self._last_ack[axis] >= self._last_cmd[axis]:
                    self._timeout_count[axis] = 0
                    continue
                self._timeout_count[axis] += 1
                self._stats["timeout"] += 1
                if self._timeout_count[axis] > int(getattr(self.cfg, "RX_TIMEOUT_LIMIT", 20)):
                    out["fault"] = True
                    out["reason"] = "ack_timeout:%s" % axis
                    self.fault_reason = out["reason"]
                    return out
        return out

    def poll(self, now=None):
        """非阻塞读两条串口并处理反馈；返回 `_ingest` 的 dict（含 frames）。"""
        now = self._clock.perf_counter() if now is None else float(now)
        return self._ingest(self._drain(), now)

    def stats(self):
        return dict(self._stats)

    def close(self):
        for port in tuple(self._ports.values()):
            try:
                port.close()
            except Exception:
                pass
        self._ports.clear()


def estimate_move_seconds(pulses, rpm, acc=0, pulses_per_rev=3200):
    values = tuple(map(float, (pulses, rpm, acc, pulses_per_rev)))
    if not all(math.isfinite(v) for v in values):
        raise ValueError("non-finite motion parameters")
    pulses, rpm, acc, ppr = values
    if ppr <= 0 or rpm <= 0 or not 0 <= acc <= 255:
        raise ValueError("invalid motion parameters")
    pulses, rpm = abs(pulses), min(5000., rpm)
    speed = rpm*ppr/60
    if acc == 0:
        return pulses/speed
    ramp = (256-acc)*50e-6
    acceleration = ppr/(60*ramp)
    ramp_seconds = rpm*ramp
    ramp_distance = speed*ramp_seconds
    if pulses <= ramp_distance:
        return 2*math.sqrt(pulses/acceleration)
    return 2*ramp_seconds+(pulses-ramp_distance)/speed
'''

# ===== vision.imu_heading  (vision/imu_heading.py) =====
_MOD_SRC['vision.imu_heading'] = r'''
"""Relative gyro heading, never an absolute compass. Native I/O has one owner."""
import math
import threading
import time
from collections import deque


def _mul(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return (w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v)


def _unit(v):
    n = math.sqrt(sum(x*x for x in v))
    if not math.isfinite(n) or abs(n-1) > 1e-6:
        raise ValueError('IMU axes must be unit vectors')
    return tuple(v)


def open_imu():
    from maix.ext_dev import imu
    return imu.IMU('lsm6dsowtr', mode=imu.Mode.DUAL,
                   gyro_scale=imu.GyroScale.GYRO_SCALE_500DPS, gyro_odr=imu.GyroOdr.GYRO_ODR_208, block=False)


def read_gyro(device):
    raw = device.read()
    if raw is None or len(raw) != 7 or not any(raw):
        return None  # DUAL seven-value all-zero sample means not ready, not zero rotation.
    gyro = tuple(float(v) for v in raw[3:6])
    return gyro if all(map(math.isfinite, gyro)) else None


class GyroHeading:
    def __init__(self, cfg, clock=None):
        self.cfg, self.clock = cfg, clock if clock is not None else time
        self.yaw_axis = _unit(getattr(cfg, 'IMU_YAW_AXIS', (0, 0, 1)))
        self.pitch_axis = _unit(getattr(cfg, 'IMU_PITCH_AXIS', (0, -1, 0)))
        if abs(sum(a*b for a,b in zip(self.yaw_axis, self.pitch_axis))) > 1e-6:
            raise ValueError('IMU axes must be orthogonal')
        self.q = (1., 0., 0., 0.)
        self.previous = self.timestamp = None
        self.bias = (0., 0., 0.)
        self.history = deque(maxlen=512)
        self.reference = None
        self.epoch = 0
        self.bias_state = 'IDLE'
        self._bias_samples = []
        self._bias_started = None

    def reset_reference(self):
        self.reference = None

    def start_bias_calibration(self, now=None):
        # Caller must stop both motors and ensure the base is stationary.
        self._bias_started = self.clock.perf_counter() if now is None else float(now)
        self._bias_samples = []
        self.bias_state = 'CALIBRATING'
        self.reset_reference()

    def _advance(self, gyro, dt):
        rates = [(a+b)*.5*math.pi/180 for a,b in zip(self.previous, gyro)]
        speed = math.sqrt(sum(v*v for v in rates))
        if speed:
            angle = speed*dt/2
            dq = (math.cos(angle), *(v*math.sin(angle)/speed for v in rates))
            q = _mul(self.q, dq)
            norm = math.sqrt(sum(v*v for v in q))
            self.q = tuple(v/norm for v in q)

    def _angles(self):
        # Rotate the mounting-frame forward vector, then project onto left/up.
        yaw, pitch = self.yaw_axis, self.pitch_axis
        forward = (yaw[1]*pitch[2]-yaw[2]*pitch[1], yaw[2]*pitch[0]-yaw[0]*pitch[2], yaw[0]*pitch[1]-yaw[1]*pitch[0])
        vector = _mul(_mul(self.q, (0., *forward)), (self.q[0], *(-v for v in self.q[1:])))[1:]
        dot = lambda a,b: sum(x*y for x,y in zip(a,b))
        return math.degrees(math.atan2(-dot(vector, pitch), dot(vector, forward))), math.degrees(math.asin(max(-1., min(1., dot(vector, yaw)))))

    def update(self, gyro, now=None):
        now = self.clock.perf_counter() if now is None else float(now)
        if gyro is None:
            return None
        raw = tuple(map(float, gyro))
        if len(raw) != 3 or not all(map(math.isfinite, (*raw, now))):
            raise ValueError('non-finite gyro sample')
        if self.bias_state == 'CALIBRATING':
            self._bias_samples.append(raw)
            if now-self._bias_started >= getattr(self.cfg, 'IMU_BIAS_CALIBRATE_S', 1.):
                samples = self._bias_samples
                mean = tuple(sum(v[i] for v in samples)/len(samples) for i in range(3))
                std = tuple(math.sqrt(sum((v[i]-mean[i])**2 for v in samples)/len(samples)) for i in range(3))
                good = len(samples) >= 10 and max(std) <= getattr(self.cfg, 'IMU_BIAS_MAX_STD_DPS', .25) and max(map(abs, mean)) <= getattr(self.cfg, 'IMU_BIAS_MAX_DPS', 2.)
                self.bias_state = 'READY' if good else 'REJECTED'
                if good:
                    self.bias = mean
                self.timestamp = None
        gyro = tuple(a-b for a,b in zip(raw, self.bias))
        dt = None if self.timestamp is None else now-self.timestamp
        if dt is None or not 0 < dt <= getattr(self.cfg, 'IMU_MAX_GAP_S', .2):
            self.q = (1., 0., 0., 0.)
            self.history.clear()
            self.reset_reference()
            self.epoch += 1
        else:
            self._advance(gyro, dt)
        self.previous, self.timestamp = gyro, now
        yaw, pitch = self._angles()
        sample = dict(yaw=yaw, pitch=pitch, captured_at=now, epoch=self.epoch, bias_state=self.bias_state)
        self.history.append(sample)
        return dict(sample)

    def sample_at(self, timestamp):
        # Never use a future sample to explain an already captured camera frame.
        for sample in reversed(self.history):
            if sample['captured_at'] <= timestamp:
                if timestamp-sample['captured_at'] <= getattr(self.cfg, 'IMU_MAX_AGE_S', .2):
                    return dict(sample)
                break
        return None

    def lock_reference(self, now, yaw_offset_deg=0., pitch_offset_deg=0., gimbal_angles=None):
        sample = self.sample_at(now)
        if sample is None or self.bias_state == 'CALIBRATING':
            return False
        if getattr(self.cfg, 'IMU_MOUNT', 'gimbal') == 'base':
            if gimbal_angles is None or not all(map(math.isfinite, gimbal_angles)):
                return False
            yaw_offset_deg += gimbal_angles[0]
            pitch_offset_deg += gimbal_angles[1]
        self.reference = (sample['yaw']+yaw_offset_deg, sample['pitch']+pitch_offset_deg, self.epoch)
        return True

    def correction(self, now, gimbal_angles=None):
        sample = self.sample_at(now)
        if sample is None or self.reference is None or self.reference[2] != self.epoch:
            return None
        yaw, pitch = sample['yaw'], sample['pitch']
        if getattr(self.cfg, 'IMU_MOUNT', 'gimbal') == 'base':
            if gimbal_angles is None:
                return None
            yaw += gimbal_angles[0]
            pitch += gimbal_angles[1]
        return ((self.reference[0]-yaw+180)%360-180, self.reference[1]-pitch)


class HeadingFeedforward:
    def __init__(self, cfg, clock=None, device=None, device_factory=None):
        self.cfg, self.clock = cfg, clock if clock is not None else time
        self.heading = GyroHeading(cfg, clock)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._bias_request = threading.Event()
        self._latest = None
        self.error = None
        self.device = device
        self.device_factory = device_factory or open_imu
        self.thread = threading.Thread(target=self._run, name='imu-heading', daemon=True)
        self.thread.start()

    def _run(self):
        try:
            if self.device is None:
                self.device = self.device_factory()
            while not self._stop.is_set():
                raw = read_gyro(self.device)  # Native I/O never holds the shared state lock.
                now = self.clock.perf_counter()
                with self._lock:
                    if self._bias_request.is_set():
                        self._bias_request.clear()
                        self.heading.start_bias_calibration(now)
                    if raw is not None:
                        self._latest = self.heading.update(raw, now)
                self._stop.wait(min(.02, 1/max(1, getattr(self.cfg, 'IMU_SAMPLE_HZ', 208))))
        except Exception as exc:
            self.error = str(exc)
        finally:
            # Only the owner thread may destroy a device with a native read in flight.
            if self.device is not None:
                close = getattr(self.device, 'close', None)
                if close:
                    close()

    def sample_at(self, now):
        if not self._lock.acquire(False):
            return None
        try:
            return self.heading.sample_at(now)
        finally:
            self._lock.release()

    def snapshot(self, now=None):
        return self.sample_at(self.clock.perf_counter() if now is None else now)

    def lock_reference(self, now, yaw_offset_deg=0., pitch_offset_deg=0., gimbal_angles=None):
        if not self._lock.acquire(False):
            return False
        try:
            return self.heading.lock_reference(now, yaw_offset_deg, pitch_offset_deg, gimbal_angles)
        finally:
            self._lock.release()

    def correction(self, now, gimbal_angles=None):
        if not self._lock.acquire(False):
            return None
        try:
            return self.heading.correction(now, gimbal_angles)
        finally:
            self._lock.release()

    def reset_reference(self):
        with self._lock:
            self.heading.reset_reference()

    def start_bias_calibration(self, now=None):
        self._bias_request.set()

    def close(self):
        self._stop.set()
        self.thread.join(.10)
        if self.thread.is_alive():
            raise RuntimeError('IMU reader still running')
'''

# ===== vision.gimbal_self_test  (vision/gimbal_self_test.py) =====
_MOD_SRC['vision.gimbal_self_test'] = r'''
"""上电云台小幅自检：yaw ±15°，pitch ±30°，最后回到启动姿态。

两种幅度口径（`config.SELF_TEST_PULSES` 决定用哪种）：
    · >0 = **固定脉冲**（推荐，见 AGENTS.md §3.2）：不依赖"脉冲/度"标定即可动作，
      所以可由屏幕按钮直接触发；顺序与角度模式一致（yaw 左→右、pitch 上→下，各回到起点）。
    · ==0 = 按 `SELF_TEST_*_DEG × *_PULSES_PER_DEG` 换算角度；未标定（两者均 0）则跳过。
"""

from .stepper_link import estimate_move_seconds


class GimbalSelfTest:
    """非阻塞自检状态机。角度通过实测的电机脉冲/输出轴度数换算。"""

    def __init__(self, cfg, link, now=0.0):
        self.cfg = cfg
        self.link = link
        self.state = "IDLE"
        self.index = 0
        self.started = float(now)
        self.deadline = float(now)
        self.sequence = []
        self.reason = None
        self._prepare()

    def _prepare(self):
        if not bool(getattr(self.cfg, "GIMBAL_SELF_TEST", True)):
            self.state, self.reason = "DONE", "disabled"
            return
        # ---- 固定脉冲模式（不依赖标定；屏幕按钮触发走这条）----
        fixed = int(getattr(self.cfg, "SELF_TEST_PULSES", 0) or 0)
        if fixed > 0:
            yaw_ccw = bool(getattr(self.cfg, "YAW_POSITIVE_CCW", False))
            pitch_ccw = bool(getattr(self.cfg, "PITCH_POSITIVE_CCW", False))
            self.sequence = [
                ("yaw", yaw_ccw, fixed), ("yaw", not yaw_ccw, 2 * fixed), ("yaw", yaw_ccw, fixed),
                ("pitch", pitch_ccw, fixed), ("pitch", not pitch_ccw, 2 * fixed), ("pitch", pitch_ccw, fixed),
            ]
            self.state = "RUNNING"
            print("[self-test] 固定脉冲模式 pulses=%d rpm=%d" %
                  (fixed, int(getattr(self.cfg, "SELF_TEST_RPM", 900))))
            return
        yaw_ppd = float(getattr(self.cfg, "YAW_PULSES_PER_DEG", 0.0))
        pitch_ppd = float(getattr(self.cfg, "PITCH_PULSES_PER_DEG", 0.0))
        # 未做"脉冲/度"标定时，允许用已知减速比推导：脉冲/度 = 每圈脉冲 × 减速比 / 360。
        # 减速比未知（0）则保持跳过 —— 不用错误比例去撞限位。
        ratio = float(getattr(self.cfg, "GIMBAL_GEAR_RATIO", 0.0))
        if ratio > 0:
            per_rev = float(getattr(self.cfg, "MOTOR_PULSES_PER_REV", 3200))
            derived = per_rev * ratio / 360.0
            if yaw_ppd <= 0:
                yaw_ppd = derived
            if pitch_ppd <= 0:
                pitch_ppd = derived
        if yaw_ppd <= 0 or pitch_ppd <= 0:
            self.state, self.reason = "SKIPPED", "pulse_per_degree_not_calibrated"
            print("[self-test] skipped: set YAW_PULSES_PER_DEG / PITCH_PULSES_PER_DEG")
            print("[self-test]           或用 GIMBAL_GEAR_RATIO 提供减速比，或先用 CALIBRATE_AXIS 实测")
            return
        yaw_angle = abs(float(getattr(self.cfg, "SELF_TEST_YAW_DEG", 15.0)))
        pitch_angle = abs(float(getattr(self.cfg, "SELF_TEST_PITCH_DEG", 30.0)))
        yaw_ccw_positive = bool(getattr(self.cfg, "YAW_POSITIVE_CCW", False))
        pitch_ccw_positive = bool(getattr(self.cfg, "PITCH_POSITIVE_CCW", False))
        self.sequence = [
            ("yaw", yaw_ccw_positive, yaw_angle * yaw_ppd),
            ("yaw", not yaw_ccw_positive, 2 * yaw_angle * yaw_ppd),
            ("yaw", yaw_ccw_positive, yaw_angle * yaw_ppd),
            ("pitch", pitch_ccw_positive, pitch_angle * pitch_ppd),
            ("pitch", not pitch_ccw_positive, 2 * pitch_angle * pitch_ppd),
            ("pitch", pitch_ccw_positive, pitch_angle * pitch_ppd),
        ]
        self.state = "RUNNING"

    @property
    def done(self):
        return self.state in ("DONE", "SKIPPED", "FAULT")

    def update(self, now):
        """每帧调用；到达估算运动时间后发下一段，不阻塞视觉主循环。"""
        if self.done:
            return self.state
        now = float(now)
        if now-self.started > getattr(self.cfg, "SELF_TEST_TIMEOUT_S", 10.):
            self.state, self.reason = "FAULT", "timeout"
            self.link.stop()
            return self.state
        if now < self.deadline:
            return self.state
        if self.index >= len(self.sequence):
            self.state = "DONE"
            print("[self-test] complete; returned to startup pose")
            return self.state
        axis, ccw, raw_pulses = self.sequence[self.index]
        pulses = int(round(raw_pulses))
        if pulses <= 0:
            self.state, self.reason = "FAULT", "invalid_pulse_count"
            self.link.stop(axis)
            return self.state
        rpm = int(getattr(self.cfg, "SELF_TEST_RPM", 900))
        if not self.link.move(axis, ccw, pulses, rpm,
                              int(getattr(self.cfg, "ACC_DEFAULT", 0))):
            self.state, self.reason = "FAULT", "uart_write_failed"
            self.link.stop(axis)
            print("[self-test] fault: %s command failed" % axis)
            return self.state
        pulses_per_rev = float(getattr(self.cfg, "MOTOR_PULSES_PER_REV", 3200))
        move_s = estimate_move_seconds(pulses, rpm, getattr(self.cfg, "ACC_DEFAULT", 80), pulses_per_rev)
        settle_s = float(getattr(self.cfg, "SELF_TEST_SETTLE_S", 0.35))
        self.deadline = now + move_s + settle_s
        self.index += 1
        print("[self-test] %s %s %dp rpm=%d step=%d/%d" %
              (axis, "CCW" if ccw else "CW", pulses, rpm,
               self.index, len(self.sequence)))
        return self.state


def probe_imu():
    """Probe the official DUAL interface; release the device in the same thread."""
    device = None
    try:
        from .imu_heading import open_imu, read_gyro
        device = open_imu()
        return dict(available=True, method="maix.ext_dev.imu", sample=read_gyro(device), error=None)
    except Exception as exc:
        return dict(available=False, method="maix.ext_dev.imu", sample=None, error=str(exc))
    finally:
        if device is not None and hasattr(device, "close"):
            device.close()
'''

# ===== vision.gimbal_aim  (vision/gimbal_aim.py) =====
_MOD_SRC['vision.gimbal_aim'] = r'''
"""基于像素误差的两轴增量比例控制器 + 状态机。

对照 `MaixCAM2串口直控云台需求文档.md` §7 实现：

- 闭环输入 = ``result["dx"]/["dy"]``（靶心 − 光斑，见 ``laser_spot.deviation``）；
- 每帧只发一帧**相对位置**命令（0xFD、mode=0），单帧脉冲限幅，``ACC_TRACK`` 曲线加减速；
- 状态机 ``BOOT / COARSE / FINE / SETTLE / HOLD / SCAN / FAULT``；
- 三重保护：离群拒收（``JUMP_REJECT_PX``）、死区（``DEADBAND_PX``）、指令限频
  （``COMMAND_INTERVAL_S``）、单帧限幅（``MAX_STEP_PULSES``）。

本模块**保持纯 Python（不 import maix）**，可在 PC 上离线仿真与回归。
控制律可整体替换：``controller(axis, error_px, now) -> 带符号脉冲数``，传 None 用内置比例律。
"""

import math
from .stepper_link import estimate_move_seconds

# 状态语义（给日志和上层判断用）
BOOT = "BOOT"        # 上电，等第一帧合法误差
COARSE = "COARSE"    # 大误差，大步快速逼近
FINE = "FINE"        # 小误差，小步逼近
SETTLE = "SETTLE"    # 进入死区，停止发指令
HOLD = "HOLD"        # 视觉失效，保持不动
SCAN = "SCAN"        # 失锁后慢扫重捕获（需 ENABLE_SCAN）
FAULT = "FAULT"      # 链路/机械故障，停机不再发指令


class GimbalAim:
    def __init__(self, cfg, controller=None):
        self.cfg = cfg
        self.controller = controller
        self.state = BOOT
        self.fault_reason = None
        self.last_command_time = {"yaw": -1e9, "pitch": -1e9}
        self.last = {"state": self.state, "dx": None, "dy": None,
                     "cmds": {"yaw": None, "pitch": None}}
        self._jump = {"yaw": 0, "pitch": 0}
        self._lost = 0
        self._settle = 0
        self._scan_dir = 1
        self._scan_pos = 0
        self._busy_until = {"yaw": -1e9, "pitch": -1e9}
        self._capture_after = {"yaw": -1e9, "pitch": -1e9}
        self._direction = {"yaw": 0., "pitch": 0.}
        self._last_measurement_time = None
        self._stops = []
        self._guard_error = {"yaw": None, "pitch": None}
        self._diverge = {"yaw": 0, "pitch": 0}

    # ------------------------------------------------------------- 外部接口
    def set_controller(self, controller):
        """设置后续 PID/前馈控制器；传入 None 恢复内置比例律。"""
        self.controller = controller

    def fault(self, reason):
        """进入 FAULT：不再发任何指令，由上层决定是否复位。"""
        self.state = FAULT
        self.fault_reason = str(reason)
        print("[aim] FAULT: %s" % self.fault_reason)
        return self.state

    def reset(self):
        self.__init__(self.cfg, self.controller)
        self.state = BOOT
        self.fault_reason = None
        self._lost = self._settle = self._scan_pos = 0
        self._scan_dir = 1
        self._jump = {"yaw": 0, "pitch": 0}
        self.last_command_time = {"yaw": -1e9, "pitch": -1e9}
        return self.state

    def update(self, result, now=0.0):
        """输入视觉 result，返回 ``cmds``；无效结果按状态机处理（HOLD / SCAN）。"""
        now = float(now)
        self._stops = []
        if self.state == FAULT:
            return self._emit(None, None, {"yaw": None, "pitch": None})
        valid = bool(result and result.get("aim_valid") and
                     result.get("dx") is not None and result.get("dy") is not None)
        if not valid:
            return self._on_lost(now)
        dx, dy = float(result["dx"]), float(result["dy"])
        captured = float(result.get("captured_at", now))
        if not all(map(math.isfinite, (dx, dy, now, captured))) or captured > now:
            return self._on_lost(now)
        if self._last_measurement_time is not None:
            if captured <= self._last_measurement_time:
                return self._emit(dx, dy, {"yaw": None, "pitch": None})
            if captured-self._last_measurement_time > getattr(self.cfg, "AIM_HISTORY_MAX_GAP_S", .30):
                self._jump = {"yaw": 0, "pitch": 0}
                self._guard_error = {"yaw": None, "pitch": None}
                self._diverge = {"yaw": 0, "pitch": 0}
        self._last_measurement_time = self._captured_at = captured
        self._lost = 0
        self._scan_pos = 0
        for axis, error in (("yaw", dx), ("pitch", dy)):
            if self._busy_until[axis] > now and (abs(error) <= self._deadband() or error*self._direction[axis] < 0):
                self._stops.append(axis)
                self._busy_until[axis] = now
                self._capture_after[axis] = now
        deadband = self._deadband()
        if abs(dx) <= deadband and abs(dy) <= deadband:
            self._settle += 1
            self.state = SETTLE
            return self._emit(dx, dy, {"yaw": None, "pitch": None})
        self._settle = 0
        coarse_px = float(getattr(self.cfg, "COARSE_PX", 0.0))
        self.state = COARSE if (coarse_px > 0 and max(abs(dx), abs(dy)) > coarse_px) else FINE
        cmds = {"yaw": self._axis_command("yaw", dx, now),
                "pitch": self._axis_command("pitch", dy, now)}
        return self._emit(dx, dy, cmds)

    def status_line(self):
        def fmt(cmd):
            if not cmd:
                return "-"
            return ("CCW" if cmd[0] else "CW") + "/%dp" % cmd[1]
        return "[aim] state=%s dx=%s dy=%s yaw=%s pit=%s lost=%d" % (
            self.last.get("state", self.state), self.last.get("dx"), self.last.get("dy"),
            fmt(self.last.get("cmds", {}).get("yaw")),
            fmt(self.last.get("cmds", {}).get("pitch")), self._lost)

    # ------------------------------------------------------------- 内部实现
    def _emit(self, dx, dy, cmds):
        out = {"state": self.state, "dx": dx, "dy": dy, "cmds": cmds, "stops": list(self._stops)}
        self.last = out
        return out

    def _deadband(self):
        return float(getattr(self.cfg, "DEADBAND_PX", 2.0))

    def _on_lost(self, now):
        self._guard_error = {"yaw": None, "pitch": None}
        self._diverge = {"yaw": 0, "pitch": 0}
        self._lost += 1
        self._settle = 0
        scan_on = bool(getattr(self.cfg, "ENABLE_SCAN", False))
        if scan_on and self._lost >= int(getattr(self.cfg, "LOST_FRAMES", 15)):
            self.state = SCAN
            cmd = self._scan_command("yaw", now)
            return self._emit(None, None, {"yaw": cmd, "pitch": None})
        self.state = HOLD
        return self._emit(None, None, {"yaw": None, "pitch": None})

    def _mix(self, error):
        fine, coarse = self._deadband(), float(getattr(self.cfg, "COARSE_PX", 24.))
        if not getattr(self.cfg, "AIM_PROGRESSIVE", True):
            return float(abs(error) > coarse)
        return max(0., min(1., (abs(error)-fine)/max(.001, coarse-fine)))

    def _gain(self, axis, error):
        fine = float(getattr(self.cfg, "GAIN_"+axis.upper(), .3))
        coarse = float(getattr(self.cfg, "GAIN_COARSE", .65))
        if not 0 < fine <= 1 or not 0 < coarse <= 1:
            raise ValueError("gain must be in (0, 1]")
        return fine + self._mix(error)*(coarse-fine)

    def _rpm(self, error=0):
        fine, coarse = self.cfg.RPM_FINE, self.cfg.RPM_COARSE
        return max(1, round(fine+self._mix(error)*(coarse-fine)))

    def mark_stopped(self, now, axis=None):
        for name in ("yaw", "pitch") if axis is None else (axis,):
            self._busy_until[name] = now
            self._capture_after[name] = now
            self._direction[name] = 0.

    def _axis_command(self, axis, error, now):
        cfg = self.cfg
        if self.state == FAULT:
            return None
        # 离群拒收：单帧超大误差先当误检丢弃，连续若干帧才采信（防甩头）。
        jump = float(getattr(cfg, "JUMP_REJECT_PX", 0.0))
        if jump > 0 and abs(error) > jump:
            self._jump[axis] += 1
            if self._jump[axis] < int(getattr(cfg, "JUMP_CONFIRM_FRAMES", 3)):
                return None
        else:
            self._jump[axis] = 0
        if abs(error) <= self._deadband():
            return None
        interval = float(getattr(cfg, "COMMAND_INTERVAL_S", 0.08))
        if now - self.last_command_time[axis] < interval:
            return None
        if getattr(cfg, "AIM_WAIT_MOTION", True) and (now < self._busy_until[axis] or self._captured_at <= max(self._busy_until[axis], self._capture_after[axis])):
            return None
        if getattr(cfg, "DIVERGE_GUARD", False):
            previous = self._guard_error[axis]
            if previous is not None:
                grew = abs(error) > abs(previous)*getattr(cfg, "DIVERGE_RATIO", 1.15)+getattr(cfg, "DIVERGE_MIN_PX", 4.)
                self._diverge[axis] = self._diverge[axis]+1 if grew and error*previous > 0 else 0
                if self._diverge[axis] >= getattr(cfg, "DIVERGE_CONFIRM_FRAMES", 6):
                    self.fault("diverge:"+axis)
                    return None
            self._guard_error[axis] = error
        sign = float(getattr(cfg, "DIR_SIGN_YAW" if axis == "yaw" else "DIR_SIGN_PITCH", 1))
        max_pulses = int(getattr(cfg, "MAX_STEP_PULSES", 360))
        if self.controller is not None:
            output = self.controller(axis, float(error), now)
            if output is None or float(output) == 0:
                return None
            pulses = min(max_pulses, max(1, int(round(abs(float(output))))))
            ccw = float(output) * sign > 0
        else:
            ppp = float(getattr(cfg, "PULSE_PER_PX_YAW" if axis == "yaw"
                                else "PULSE_PER_PX_PITCH", 3.67))
            if not math.isfinite(ppp) or ppp <= 0:
                self.fault("invalid_scale:"+axis)
                return None
            pulses = min(max_pulses, max(1, int(round(abs(error) * ppp * self._gain(axis, error)))))
            ccw = error * sign > 0
        if not math.isfinite(sign) or abs(sign) != 1:
            self.fault("invalid_direction:"+axis)
            return None
        self.last_command_time[axis] = now
        rpm = self._rpm(error)
        self._direction[axis] = error
        self._busy_until[axis] = now+estimate_move_seconds(pulses, rpm, getattr(cfg, "ACC_TRACK", 80), getattr(cfg, "MOTOR_PULSES_PER_REV", 3200))+getattr(cfg, "AIM_MOTION_MARGIN_S", .03)
        return ccw, pulses, rpm

    def _scan_command(self, axis, now):
        cfg = self.cfg
        if (getattr(cfg, "AIM_WAIT_MOTION", True) and now < self._busy_until[axis]) or now - self.last_command_time[axis] < float(getattr(cfg, "SCAN_STEP_INTERVAL_S", 0.15)):
            return None
        ppd = float(getattr(cfg, "YAW_PULSES_PER_DEG", 0.0))
        if ppd > 0:
            step = max(1, int(round(float(getattr(cfg, "SCAN_STEP_DEG", 1.5)) * ppd)))
            limit = max(step, int(round(float(getattr(cfg, "SCAN_LIMIT_DEG", 15.0)) * ppd)))
        else:
            step = max(1, int(getattr(cfg, "SCAN_STEP_PULSES", 40)))
            limit = max(step, int(getattr(cfg, "SCAN_LIMIT_PULSES", 400)))
        if self._scan_pos >= limit:
            self._scan_dir = -1
        elif self._scan_pos <= -limit:
            self._scan_dir = 1
        sign = float(getattr(cfg, "DIR_SIGN_YAW", 1))
        ccw = (self._scan_dir * sign) > 0
        self._scan_pos += self._scan_dir * step
        self.last_command_time[axis] = now
        rpm = int(getattr(cfg, "SCAN_RPM", 60))
        self._busy_until[axis] = now+estimate_move_seconds(step, rpm, getattr(cfg, "ACC_TRACK", 80), getattr(cfg, "MOTOR_PULSES_PER_REV", 3200))
        return ccw, step, rpm
'''

# ===== vision.gimbal_search  (vision/gimbal_search.py) =====
_MOD_SRC['vision.gimbal_search'] = r'''
"""Button-triggered nonblocking search. Cancellation stops in place, never homes."""
import math
from .stepper_link import estimate_move_seconds


class SearchSweep:
    def __init__(self, cfg, link, now=0., heading=None):
        self.cfg, self.link, self.heading = cfg, link, heading
        self.started_at = float(now)
        self.state, self.reason = 'SEARCH', None
        self.index, self.deadline = 0, float(now)
        self.sequence = []
        self.segment = None
        self.measurements = []
        for axis in ('yaw', 'pitch'):
            limit = abs(float(getattr(cfg, 'SEARCH_'+axis.upper()+'_DEG', 45.)))
            ppd = float(getattr(cfg, axis.upper()+'_PULSES_PER_DEG', 0.))
            pulses = max(1, round(limit*ppd)) if ppd > 0 else max(1, int(getattr(cfg, 'SEARCH_PULSES', 400)))
            positive = bool(getattr(cfg, axis.upper()+'_POSITIVE_CCW', True))
            for sign, factor in ((1, 1), (-1, 2), (1, 1)):
                self.sequence.append((axis, positive if sign > 0 else not positive, pulses*factor, sign*factor*limit))

    @property
    def done(self):
        return self.state != 'SEARCH'

    def _sample(self, now):
        return self.heading.sample_at(now) if self.heading is not None else None

    def _stop(self, now, state, reason):
        ok = self.link.stop(now=now)
        self.state, self.reason = (state, reason) if ok is not False else ('FAULT', 'stop:all')
        return self.state

    def cancel(self, now):
        return self._stop(now, 'CANCELLED', 'button')

    def _target_is_fresh(self, result, now):
        if not result or not result.get('valid') or result.get('held') or result.get('predicted'):
            return False
        captured = result.get('captured_at', float('-inf'))
        return math.isfinite(captured) and self.started_at <= captured <= now and now-captured <= self.cfg.RESULT_MAX_AGE_S

    def update(self, now, result=None):
        if self.done:
            return self.state
        if self._target_is_fresh(result, now):
            return self._stop(now, 'FOUND', 'visual')
        if now-self.started_at >= getattr(self.cfg, 'SEARCH_TIMEOUT_S', 60.):
            return self._stop(now, 'DONE', 'timeout')
        sample = self._sample(now)
        if self.segment is not None:
            axis, pulses, angle, initial, sent = self.segment
            measured = None
            if sample is not None and initial is not None and sample.get('epoch') == initial.get('epoch'):
                measured = (sample[axis]-initial[axis]+180)%360-180
            early = (getattr(self.cfg, 'SEARCH_USE_IMU', True) and measured is not None and measured*angle > 0 and abs(measured) >= abs(angle)-getattr(self.cfg, 'SEARCH_IMU_MARGIN_DEG', 3.))
            if now < self.deadline and not early:
                return self.state
            if early and self.link.stop(axis, now=now) is False:
                return self._stop(now, 'FAULT', 'stop:'+axis)
            if measured is not None and abs(measured) > 1:
                value = pulses/abs(measured)
                # If stopped early, requested pulses are an upper bound, not measured travel.
                self.measurements.append(dict(axis=axis, pulses_per_deg=value, upper_bound=early))
                print('[search] %s pulses/degree%s=%.3f' % (axis, ' upper-bound' if early else '', value))
            self.segment = None
            self.deadline = now+getattr(self.cfg, 'SEARCH_SETTLE_S', .1)
            return self.state
        if now < self.deadline:
            return self.state
        if self.index >= len(self.sequence):
            self.state, self.reason = 'DONE', 'sequence_complete'
            return self.state
        axis, ccw, pulses, angle = self.sequence[self.index]
        rpm, acc = getattr(self.cfg, 'SEARCH_RPM', 60), getattr(self.cfg, 'SEARCH_ACC', 60)
        if self.link.move(axis, ccw, pulses, rpm, acc, now=now) is False:
            return self._stop(now, 'FAULT', 'tx:'+axis)
        self.segment = axis, pulses, angle, sample, now
        self.deadline = now+estimate_move_seconds(pulses, rpm, acc, self.cfg.MOTOR_PULSES_PER_REV)
        self.index += 1
        return self.state
'''

# ===== vision.aim_control  (vision/aim_control.py) =====
_MOD_SRC['vision.aim_control'] = r'''
"""Single owner of UART, priority arbitration, and captured-frame freshness gates."""
import math
import threading
import time
from .gimbal_self_test import GimbalSelfTest
from .gimbal_search import SearchSweep
from .laser_spot import LaserSpotDetector, deviation
from .rectangle_center import RectangleCenterDetector


def send_uart(uart, result):
    if uart is not None:
        payload = '@[%d,%d]%%\r\n' % (round(result['u']), round(result['v'])) if result.get('valid') else '@[invalid]%%\r\n'
        uart.write(payload.encode('ascii'))


class VisionControl:
    def __init__(self, cfg, link, aim, calibrator=None, uart=None, clock=None, heading=None):
        self.cfg, self.link, self.aim, self.calibrator, self.uart = cfg, link, aim, calibrator, uart
        self._clock = clock if clock is not None else time
        self.heading = heading
        self.self_test = self.search = None
        self.request_test = threading.Event()
        self.request_search = threading.Event()
        self.request_bias = threading.Event()
        self.detector = RectangleCenterDetector(cfg)
        self.laser = LaserSpotDetector(cfg) if cfg.ENABLE_LASER else None
        self.laser_static = self.laser is not None and self.laser.mode == 'fixed'
        self.laser_state = self.laser.static_state() if self.laser_static else None
        self.last_capture = -1e9
        self.stale_stopped = True
        self.notice = ''
        self.control_source = 'vision'
        self.commands = {'yaw': None, 'pitch': None}
        self._bias_active = False
        self._auto_axis = None

    def process(self, gray, captured_at):
        started = self._clock.perf_counter()
        result = self.detector.process(gray, timestamp=captured_at)
        detected = self._clock.perf_counter()
        if self.laser is not None and not self.laser_static:
            self.laser_state = self.laser.process(gray, captured_at)
        dv = deviation(self.laser_state, result, use_raw=self.cfg.AIM_USE_RAW) if self.laser_state else None
        if dv is not None:
            result.update(laser_u=self.laser_state['u'], laser_v=self.laser_state['v'], dx=dv[0], dy=dv[1])
        result['aim_valid'] = dv is not None
        output = self.accept_result(result, captured_at)
        return dict(result=output, detector_result=result,
                    laser=dict(self.laser_state) if self.laser_state else None,
                    captured_at=captured_at, completed_at=self._clock.perf_counter(),
                    detect_ms=(detected-started)*1000, status=self.status())

    def accept_result(self, result, captured_at, now=None):
        now = self._clock.perf_counter() if now is None else float(now)
        captured_at = float(captured_at)
        fresh = (math.isfinite(now) and math.isfinite(captured_at) and
                 0 <= now-captured_at <= self.cfg.RESULT_MAX_AGE_S and captured_at > self.last_capture)
        output = dict(result) if fresh else dict(valid=False, aim_valid=False, stage='stale')
        output.update(captured_at=captured_at, age_s=now-captured_at)
        if fresh:
            self.last_capture = captured_at
        send_uart(self.uart, output)
        self._control(output, now, new_measurement=fresh)
        output.update(aim_state=self.aim.state if self.aim else None, cmds=dict(self.commands),
                      control_source=self.control_source, search_state=self.search.state if self.search else None)
        return output

    def idle(self):
        self._control(None, self._clock.perf_counter(), new_measurement=False)

    def status(self):
        if self.aim is not None and self.aim.state == 'FAULT':
            return 'FAULT'
        if self.self_test is not None and not self.self_test.done:
            return 'self-test %d/%d' % (self.self_test.index, len(self.self_test.sequence))
        if self.calibrator is not None and not self.calibrator.done:
            return 'calibrate '+self.calibrator.axis
        if self.search is not None and not self.search.done:
            return 'SEARCH %d/%d' % (self.search.index, len(self.search.sequence))
        if self.control_source == 'imu_hold':
            return 'IMU HOLD '+self.aim.state
        return self.notice or (self.aim.state if self.aim else 'vision')

    def _stop_motion(self, now, axis=None):
        if self.link is None:
            return True
        try:
            ok = self.link.stop(now=now) if axis is None else self.link.stop(axis, now=now)
        except Exception:
            ok = False
        if ok is False:
            if axis is not None:
                try:
                    self.link.stop('pitch' if axis == 'yaw' else 'yaw', now=now)
                except Exception:
                    pass
            self.aim.fault('stop:'+(axis or 'all'))
            return False
        if hasattr(self.aim, 'mark_stopped'):
            self.aim.mark_stopped(now, axis)
        return True

    def _notice(self, text):
        self.notice = text
        print('[ui] '+text)

    def _degree_scales(self):
        if not getattr(self.cfg, 'ENABLE_IMU_FEEDFORWARD', False) or self.heading is None:
            return None
        # A base-mounted sensor needs measured gimbal angles; requested pulses are not feedback.
        if getattr(self.cfg, 'IMU_MOUNT', 'gimbal') != 'gimbal':
            return None
        scales = []
        for axis in ('yaw', 'pitch'):
            ppd = float(getattr(self.cfg, axis.upper()+'_PULSES_PER_DEG', 0))
            ppp = float(getattr(self.cfg, 'PULSE_PER_PX_'+axis.upper(), 0))
            sign = float(getattr(self.cfg, 'DIR_SIGN_'+axis.upper(), 0))
            if not all(map(math.isfinite, (ppd, ppp, sign))) or min(ppd, ppp) <= 0 or abs(sign) != 1:
                return None
            scales.append(ppd/(ppp*sign))
        return scales

    def _with_heading(self, result, now, new_measurement):
        self.control_source = 'vision'
        scales = self._degree_scales()
        if scales is None or (result is not None and result.get('stage') == 'stale'):
            return result
        if new_measurement and result and result.get('aim_valid'):
            captured = result['captured_at']
            at_capture, current = self.heading.sample_at(captured), self.heading.sample_at(now)
            if at_capture is not None and current is not None and at_capture.get('epoch') == current.get('epoch'):
                self.heading.lock_reference(captured, result['dx']/scales[0], result['dy']/scales[1])
                result = dict(result)
                # Add ONLY motion since exposure; adding absolute error double counts rotation.
                result['dx'] -= ((current['yaw']-at_capture['yaw']+180)%360-180)*scales[0]
                result['dy'] -= (current['pitch']-at_capture['pitch'])*scales[1]
                self.control_source = 'vision_imu'
            return result
        if now-self.last_capture <= getattr(self.cfg, 'IMU_HOLD_MAX_S', .25):
            correction, sample = self.heading.correction(now), self.heading.sample_at(now)
            if correction is not None and sample is not None:
                self.control_source = 'imu_hold'
                return dict(aim_valid=True, dx=correction[0]*scales[0], dy=correction[1]*scales[1], captured_at=sample['captured_at'])
        return result

    def _control(self, result, now, new_measurement):
        self.commands = {'yaw': None, 'pitch': None}
        link, aim = self.link, self.aim
        if link is None or aim is None:
            return
        # FAULT before poll: even E2 re-enable is forbidden once motion is faulted.
        if aim.state == 'FAULT':
            self.request_test.clear()
            self.request_search.clear()
            return
        feedback = link.poll(now)
        if feedback and feedback.get('fault'):
            self._stop_motion(now)
            aim.fault('link:%s' % feedback.get('reason'))
            return
        busy_cal = self.calibrator is not None and not self.calibrator.done
        busy_test = self.self_test is not None and not self.self_test.done
        busy_search = self.search is not None and not self.search.done
        if self.request_test.is_set():
            self.request_test.clear()
            if busy_cal or busy_test or busy_search or self._bias_active or not self.cfg.GIMBAL_SELF_TEST:
                self._notice('SELF-TEST rejected: busy or disabled')
            elif self._stop_motion(now):
                self.self_test = GimbalSelfTest(self.cfg, link, now)
                self._notice('收到点击 -> 启动上电自检 SELF-TEST')
                busy_test = not self.self_test.done
        if self.request_search.is_set():
            self.request_search.clear()
            if busy_search:
                self.search.cancel(now)
                self._stop_motion(now)
                if self.search.state == 'FAULT':
                    aim.fault(self.search.reason)
                self._notice('SEARCH cancelled')
                busy_search = False
            elif busy_cal or busy_test or self._bias_active or not getattr(self.cfg, 'ENABLE_SEARCH_BUTTON', True):
                self._notice('SEARCH rejected: busy or disabled')
            elif self._stop_motion(now):
                self.search = SearchSweep(self.cfg, link, now, self.heading)
                self._notice('SEARCH started')
                busy_search = True
        if self.request_bias.is_set():
            self.request_bias.clear()
            if self.heading is None or busy_cal or busy_test or busy_search:
                self._notice('IMU bias rejected: unavailable or busy')
            elif self._stop_motion(now):
                self.heading.start_bias_calibration(now)
                self._bias_active = True
                self._bias_started = now
                self._notice('IMU bias: keep base stationary')
        if aim.state == 'FAULT':
            return
        if self._bias_active:
            sample = self.heading.sample_at(now)
            if sample and sample.get('bias_state') in ('READY', 'REJECTED'):
                self._bias_active = False
                self._notice('IMU bias '+sample['bias_state'])
            elif now-self._bias_started > getattr(self.cfg, 'IMU_BIAS_CALIBRATE_S', 1.)+1:
                self._bias_active = False
                self._notice('IMU bias timeout')
            return
        if busy_cal:
            self.calibrator.update(now, result if new_measurement else None)
            if self.calibrator.done:
                if self.calibrator.state == 'FAULT' and (self.calibrator.reason or '').startswith(('tx:', 'stop:', 'uart_')):
                    self._stop_motion(now)
                    aim.fault(self.calibrator.reason)
                elif self.calibrator.state == 'DONE' and self._auto_axis == 'yaw':
                    from .gimbal_calibrate import GimbalCalibrator
                    cfg = type('PitchCalibration', (self.cfg,), {'CALIBRATE_AXIS': 'pitch'})
                    self.calibrator = GimbalCalibrator(cfg, link, now)
                    self._auto_axis = 'pitch'
            return
        if busy_test:
            self.self_test.update(now)
            if self.self_test.state == 'FAULT':
                self._stop_motion(now)
                aim.fault('self-test:'+str(self.self_test.reason))
            return
        if busy_search:
            self.search.update(now, result if new_measurement else None)
            if self.search.state == 'FAULT':
                aim.fault(self.search.reason)
            elif self.search.done:
                self._stop_motion(now)
                self._notice('SEARCH '+self.search.state)
            return
        if not getattr(self.cfg, 'GIMBAL_CALIBRATED', True):
            if not self.stale_stopped:
                self._stop_motion(now)
            self.stale_stopped = True
            aim.update({'aim_valid': False}, now=now)
            return
        result = self._with_heading(result, now, new_measurement)
        if not new_measurement and self.control_source != 'imu_hold':
            if (result is not None and result.get('stage') == 'stale' or now-self.last_capture > self.cfg.RESULT_MAX_AGE_S) and not self.stale_stopped:
                self._stop_motion(now)
                aim.update({'aim_valid': False}, now=now)
                self.stale_stopped = True
            return
        if not result or not result.get('aim_valid'):
            if not self.stale_stopped:
                self._stop_motion(now)
            self.stale_stopped = True
        else:
            self.stale_stopped = False
        if aim.state == 'FAULT':
            return
        output = aim.update(result or {'aim_valid': False}, now=now)
        if aim.state == 'FAULT':
            self._stop_motion(now)
            return
        for axis in output.get('stops', ()):
            if not self._stop_motion(now, axis):
                return
        for axis, command in output['cmds'].items():
            if command is not None:
                ccw, pulses, rpm = command
                if link.move(axis, ccw, pulses, rpm, self.cfg.ACC_TRACK, now=now) is False:
                    self._stop_motion(now)
                    aim.fault('tx:'+axis)
                    return
                self.commands[axis] = command

    def close(self):
        if self.heading is not None:
            self.heading.close()
'''

# ===== vision.gimbal_calibrate  (vision/gimbal_calibrate.py) =====
_MOD_SRC['vision.gimbal_calibrate'] = r'''
"""开环标定（一次性）：在图像上实测 PULSE_PER_PX / DIR_SIGN / 交叉项 / 回差。

为什么必须有这一步：闭环能否收敛、方向是否正确，全取决于 ``PULSE_PER_PX_*`` 与
``DIR_SIGN_*``（需求文档 §7.2/§7.3）。理论换算要数齿数、查细分、量安装角，任一错就全错；
本模块直接在图像上实测 —— **"走 N 个脉冲，看靶心在图像里动了多少像素"**，一次性把
齿轮比、细分、焦距、安装角全部吸收掉。

流程（非阻塞状态机，与主循环共用链路、不阻塞视觉）::

    静置取基线 (dx0,dy0)
      -> 正方向走 N 脉冲 -> 静置 -> 测 (dx1,dy1)
      -> 反方向走 N 脉冲回位 -> 静置 -> 测 (dx2,dy2)

输出：
    PULSE_PER_PX = N / |Δ|；DIR_SIGN（走正方向后 |误差| 反而变大 ⇒ 符号反 ⇒ -1）；
    交叉项 C = Δdy/Δdx（判相机横滚）；回差估计 |Δ2−Δ0| / 2（决定 DEADBAND 下限）。

标定期 main 会跳过闭环（见 ``CALIBRATE_AXIS`` 非空）。标完把打印出的数值填进
``config.py``，再把 ``CALIBRATE_AXIS`` 置回 ``""``，重新上电即进入正常跟踪。

保持纯 Python（不 import maix），可在 PC 上用假链路回归。
"""

import math
from .stepper_link import estimate_move_seconds


class GimbalCalibrator:
    def __init__(self, cfg, link, now=0.0):
        self.cfg = cfg
        self.link = link
        self.axis = str(getattr(cfg, "CALIBRATE_AXIS", "")).strip().lower()
        self.pulses = int(getattr(cfg, "CALIBRATE_PULSES", 2000))
        self.need = max(2, int(getattr(cfg, "CALIBRATE_SAMPLES", 5)))
        self.samples = []
        self.base = self.out = self.back = None
        self.report = None
        self.reason = None
        self.last_visual = self.started = float(now)
        self.deadline = float(now) + float(getattr(cfg, "CALIBRATE_SETTLE_S", 1.0))
        self.limit_s = float(getattr(cfg, "CALIBRATE_TIMEOUT_S", 20.0))
        self.state = "BASELINE"
        if self.axis not in ("yaw", "pitch") or self.pulses <= 0:
            self.state, self.reason = "SKIPPED", "CALIBRATE_AXIS unset or invalid"
        else:
            print("[calib] start axis=%s pulses=%d samples=%d rpm=%d" %
                  (self.axis, self.pulses, self.need, int(getattr(cfg, "CALIBRATE_RPM", 300))))

    # ------------------------------------------------------------- 接口
    @property
    def done(self):
        return self.state in ("DONE", "SKIPPED", "FAULT")

    def update(self, now, result=None):
        """每帧调用；到达估算运动时间后推进状态机，不阻塞主循环。"""
        now = float(now)
        if self.done:
            return self.state
        if now - self.started > self.limit_s:
            return self._fault("timeout_no_valid_aim(%.1fs)" % self.limit_s)
        if result and result.get("aim_valid"):
            self.last_visual = now
        elif now-self.last_visual > getattr(self.cfg, "CALIBRATE_VISUAL_TIMEOUT_S", 3.):
            return self._fault("visual_timeout")
        if now < self.deadline:
            return self.state
        dv = None
        if result and result.get("aim_valid") and result.get("dx") is not None:
            dv = (float(result["dx"]), float(result["dy"]))
            if not all(map(math.isfinite, dv)):
                return self._fault("non_finite_measurement")
        if self.state == "BASELINE":
            if dv is None:
                return self.state
            self.samples.append(dv)
            if len(self.samples) < self.need:
                return self.state
            self.base = self._avg()
            print("[calib] baseline  dx=%+.2f dy=%+.2f" % self.base)
            if not self._send(ccw=True):
                return self.state
            self.state = "MOVE_OUT"
            self.deadline = now + self._move_seconds()
        elif self.state == "MOVE_OUT":
            self.samples = []
            self.state = "MEASURE_OUT"
        elif self.state == "MEASURE_OUT":
            if dv is None:
                return self.state
            self.samples.append(dv)
            if len(self.samples) < self.need:
                return self.state
            self.out = self._avg()
            print("[calib] after +%dp dx=%+.2f dy=%+.2f" % (self.pulses, self.out[0], self.out[1]))
            if not self._send(ccw=False):
                return self.state
            self.state = "MOVE_BACK"
            self.deadline = now + self._move_seconds()
        elif self.state == "MOVE_BACK":
            self.samples = []
            self.state = "MEASURE_BACK"
        elif self.state == "MEASURE_BACK":
            if dv is None:
                return self.state
            self.samples.append(dv)
            if len(self.samples) < self.need:
                return self.state
            self.back = self._avg()
            print("[calib] after -%dp dx=%+.2f dy=%+.2f" % (self.pulses, self.back[0], self.back[1]))
            self._finish()
        return self.state

    # ------------------------------------------------------------- 内部
    def _idx(self):
        return 0 if self.axis == "yaw" else 1

    def _avg(self):
        n = float(len(self.samples))
        return (sum(s[0] for s in self.samples) / n,
                sum(s[1] for s in self.samples) / n)

    def _move_seconds(self):
        rpm = max(1, int(getattr(self.cfg, "CALIBRATE_RPM", 300)))
        ppr = max(1.0, float(getattr(self.cfg, "MOTOR_PULSES_PER_REV", 3200)))
        travel = estimate_move_seconds(self.pulses, rpm, getattr(self.cfg, "ACC_DEFAULT", 80), ppr)
        settle = float(getattr(self.cfg, "CALIBRATE_SETTLE_S", 1.0))
        return travel + settle

    def _send(self, ccw):
        ok = self.link.move(self.axis, ccw, self.pulses,
                            int(getattr(self.cfg, "CALIBRATE_RPM", 300)),
                            int(getattr(self.cfg, "ACC_DEFAULT", 0)))
        if not ok:
            self._fault("uart_write_failed")
        return bool(ok)

    def _fault(self, reason):
        self.state, self.reason = "FAULT", str(reason)
        if self.link.stop(self.axis) is False:
            self.reason = "stop:"+self.axis
        print("[calib] fault: %s" % self.reason)
        return self.state

    def _finish(self):
        i = self._idx()
        delta = self.out[i] - self.base[i]
        cross = (self.out[1] - self.base[1]) if i == 0 else (self.out[0] - self.base[0])
        if abs(delta) < 0.5:
            return self._fault("no_motion(%.2fpx after %dp)" % (delta, self.pulses))
        ppp = self.pulses / abs(delta)
        sign = 1 if delta < 0 else -1
        c = cross / delta
        residual = self.back[i] - self.base[i]
        backlash = abs(residual) / 2.0
        upper = self.axis.upper()
        self.report = dict(axis=self.axis, pulse_per_px=ppp, dir_sign=sign,
                           cross=c, backlash_px=backlash, residual_px=residual)
        print("[calib] ---------------- 结果（填进 config.py）----------------")
        print("[calib]   PULSE_PER_PX_%s = %.2f" % (upper, ppp))
        print("[calib]   DIR_SIGN_%s = %d" % (upper, sign))
        print("[calib]   交叉项 C=%.3f %s" % (
            c, "→ |C|>0.25，相机横滚没装正，优先机械调整" if abs(c) > 0.25 else "(正常)"))
        print("[calib]   回差估计 ≈ %.2fpx（DEADBAND_PX 建议 ≥ 1.5×该值，下限 1.5px）" % backlash)
        print("[calib]   换轴标定请把 CALIBRATE_AXIS 改成另一轴重跑；标完置回 \"\"")
        self.state = "DONE"
        return self.state
'''

# ===== vision.app_runtime  (vision/app_runtime.py) =====
_MOD_SRC['vision.app_runtime'] = r'''
"""Application assembly, three independent time axes, and ordered hardware release."""
import time
import cv2
import numpy as np
from .camera_source import to_gray
from .rate_meter import FrameRate
from .rectangle_center import draw_result
from .laser_spot import draw_laser_overlay, PANEL_H
from .preview import display_result
from .telemetry import format_perf_line, format_slow_loop, MAIN_LOOP_SLOW_MS

_active = []


def _assert_idle():
    live = [r for r in _active if getattr(r, 'thread', None) is not None and r.thread.is_alive()]
    _active[:] = live
    if live:
        raise RuntimeError('previous worker/display/IMU thread is still running')


def _configure_touch(cfg, preview, touch_class, clock):
    if not preview.enabled or not getattr(cfg, 'ENABLE_TOUCH_BUTTON', True):
        return None
    w,h = preview.width, preview.height
    margin = min(8,max(0,min(w,h)//8))
    gap = max(1,min(8,h//12))
    count = 2 if getattr(cfg,'ENABLE_SEARCH_BUTTON',True) else 1
    bw = min(max(70,int(w*.30)),w-2*margin)
    bh = min(max(34,int(h*.20)),(h-2*margin-gap*(count-1))//count)
    if bh < 3 or bw < 3:
        return None
    preview.add_button('selftest',w-bw-margin,margin,w-margin,margin+bh)
    if count == 2:
        y=margin+bh+gap
        preview.add_button('search',w-bw-margin,y,w-margin,y+bh)
    return touch_class(cfg,clock=clock)


def _draw_preview(cfg, preview, gray, result, laser, age, fps, status, now):
    view=draw_result(gray,result,preview.render_width,top_offset=PANEL_H if laser else 0)
    if laser is not None:
        draw_laser_overlay(view,laser,result,min(1.,preview.render_width/float(gray.shape[1])),use_raw=cfg.AIM_USE_RAW)
    # Keep the performance line above Preview's bottom status line.
    cv2.putText(view,'disp %.1f low1 %.1f det %.1f age %.0fms' % (preview.fps,preview.low1,fps,age*1000),
                (8,view.shape[0]-30),cv2.FONT_HERSHEY_SIMPLEX,.40,(0,255,255),1)
    preview.render(view,now,status=status)


def _capture_loop(cfg, clock, app, image, state, build_id):
    cam,worker,controller,preview,touch = (state[k] for k in ('camera','worker','controller','preview','touch'))
    capture_rate=FrameRate()
    last_log=clock.perf_counter()
    read_times,draw_times,loop_times=[],[],[]
    while not app.need_exit():
        started=clock.perf_counter()
        img=cam.read()
        if worker.error is not None:
            raise RuntimeError('vision worker failed: '+worker.error)
        if img is None:
            if not cfg.VISION_ASYNC:
                controller.idle()
            continue
        captured=clock.perf_counter()
        read_times.append((captured-started)*1000)
        if getattr(cfg,'LENS_CORR',False):
            img.lens_corr(strength=cfg.LENS_STRENGTH)
        gray=to_gray(img,image)
        capture_rate.record(captured)
        button=touch.poll(preview) if touch is not None else None
        if button == 'selftest': controller.request_test.set()
        elif button == 'search': controller.request_search.set()
        worker.submit(gray,captured)
        now=clock.perf_counter()
        snapshot,fps,_=worker.snapshot(now)
        if worker.error is not None:
            raise RuntimeError('vision worker failed: '+worker.error)
        result,laser,age=display_result(snapshot,now,cfg.RESULT_MAX_AGE_S)
        if preview.due(now):
            t_draw=clock.perf_counter()
            try:
                _draw_preview(cfg,preview,gray,result,laser,age,fps,snapshot['status'] if snapshot else 'waiting',now)
            except Exception as exc:
                # Drawing/conversion errors belong to display, never to motor control.
                preview.enabled=False
                print('[preview] render disabled: %s' % exc)
            draw_times.append((clock.perf_counter()-t_draw)*1000)
        loop_times.append((clock.perf_counter()-started)*1000)
        now=clock.perf_counter()
        if now-last_log >= cfg.LOG_INTERVAL:
            capture_fps,_=capture_rate.values(now)
            display_fps,low1=preview.rate.values(now)
            avg,p95,worst,n=preview.take_show_stats()
            mean=lambda values: float(np.mean(values)) if values else 0.
            loop_avg=mean(loop_times)
            detector_result=snapshot.get('detector_result',snapshot['result']) if snapshot else {}
            print(format_perf_line(build_id,capture_fps,display_fps,low1,fps,result,
                mean(read_times),mean(draw_times),loop_avg,avg,p95,worst,n,age,
                preview.shown,preview.dropped,worker.dropped,detector_result=detector_result))
            if state['aim'] is not None: print(state['aim'].status_line())
            if p95 > 100: preview.warn_slow_once(p95)
            if loop_avg > MAIN_LOOP_SLOW_MS: print(format_slow_loop(loop_avg))
            last_log=now
            read_times.clear(); draw_times.clear(); loop_times.clear()


def _release(state):
    errors=[]
    def close(resource):
        if resource is not None and hasattr(resource,'close'):
            try: resource.close()
            except Exception as exc: errors.append(exc)
    worker=state.get('worker')
    close(worker)
    if worker is None or worker.thread is None or not worker.thread.is_alive():
        link=state.get('link')
        if link is not None:
            try:
                if link.stop() is False: errors.append(RuntimeError('stop on exit failed'))
            except Exception as exc: errors.append(exc)
        # Controller owns heading only after successful construction.
        if state.get('controller') is not None: close(state['controller'])
        else: close(state.get('heading'))
        close(link)
        close(state.get('uart'))
    close(state.get('preview'))
    touch=state.get('touch')
    close(touch.ts if touch is not None else None)
    close(state.get('camera'))
    if errors: raise RuntimeError('cleanup: '+'; '.join(map(str,errors)))


def run(entry, build_id=None):
    _assert_idle()
    cfg,clock=entry.Config,entry.time
    state=dict(camera=None,link=None,aim=None,uart=None,heading=None,controller=None,worker=None,preview=None,touch=None)
    try:
        from maix import app,camera,image
        cv2.setNumThreads(cfg.OPENCV_THREADS)
        cam=state['camera']=camera.Camera(cfg.CAMERA_W,cfg.CAMERA_H,image.Format.FMT_GRAYSCALE,fps=cfg.CAMERA_FPS,buff_num=cfg.CAMERA_BUFFERS)
        cam.skip_frames(cfg.SKIP_FRAMES)
        calibrator=None
        if cfg.ENABLE_GIMBAL:
            from .stepper_link import StepperLink
            link=state['link']=StepperLink(cfg,clock=clock)
            aim=state['aim']=entry.GimbalAim(cfg)
            if link.enable() is False: aim.fault('enable_failed')
            calibration_cfg=type('YawCalibration',(cfg,),{'CALIBRATE_AXIS':'yaw'}) if cfg.CALIBRATE_AXIS=='auto' else cfg
            calibrator=entry.GimbalCalibrator(calibration_cfg,link,clock.perf_counter())
            if calibrator.done: calibrator=None
            info=entry.probe_imu()
            print('[imu] %s' % ('available' if info['available'] else 'unavailable: '+str(info['error'])))
            print('[gimbal] enabled yaw=%s(ID%d) pitch=%s(ID%d) acc=%d' % (cfg.YAW_DEV,cfg.ADDR_YAW,cfg.PITCH_DEV,cfg.ADDR_PITCH,cfg.ACC_TRACK))
        elif cfg.ENABLE_UART:
            from maix import uart
            state['uart']=uart.UART(cfg.UART_PORT,cfg.UART_BAUD)
        preview=state['preview']=entry.Preview(cfg,clock=clock)
        _active.append(preview)
        state['touch']=_configure_touch(cfg,preview,entry.TouchButtons,clock)
        if getattr(cfg,'ENABLE_IMU_FEEDFORWARD',False) or (getattr(cfg,'ENABLE_SEARCH_BUTTON',True) and getattr(cfg,'SEARCH_USE_IMU',True)):
            from .imu_heading import HeadingFeedforward
            state['heading']=HeadingFeedforward(cfg,clock=clock)
            _active.append(state['heading'])
        controller=state['controller']=entry.VisionControl(cfg,state['link'],state['aim'],calibrator,state['uart'],clock=clock,heading=state['heading'])
        if cfg.CALIBRATE_AXIS=='auto': controller._auto_axis='yaw'
        worker=state['worker']=entry.LatestFrameWorker(controller.process,controller.idle,cfg.VISION_ASYNC,clock=clock)
        _active.append(worker)
        print('[rect-center] %s search_width=%d display_width=%d display_fps=%d' % (build_id or entry.BUILD_ID,cfg.SEARCH_WIDTH,preview.render_width,cfg.SHOW_FPS))
        print('[preview] async=%s canvas=%dx%d' % (cfg.SHOW_ASYNC,preview.render_width,preview.render_height))
        print('[vision] latest-frame worker=%s max_result_age=%.0fms' % (cfg.VISION_ASYNC,cfg.RESULT_MAX_AGE_S*1000))
        print('[ui] 上电自检已改为屏幕按钮：SELF-TEST / SEARCH')
        _capture_loop(cfg,clock,app,image,state,build_id or entry.BUILD_ID)
    finally:
        _release(state)
'''

# ===== vision  (vision/__init__.py) =====
_MOD_SRC['vision'] = r'''
from .rectangle_center import RectangleCenterDetector
from .laser_spot import LaserSpotDetector, deviation, draw_laser_overlay
from .gimbal_aim import GimbalAim
from .gimbal_self_test import GimbalSelfTest, probe_imu
from .gimbal_calibrate import GimbalCalibrator
from .stepper_link import StepperLink

__all__ = ["RectangleCenterDetector", "LaserSpotDetector", "deviation",
           "draw_laser_overlay", "GimbalAim", "GimbalSelfTest", "probe_imu",
           "GimbalCalibrator", "StepperLink"]
'''

# ===== target_api  (target_api.py) =====
_MOD_SRC['target_api'] = r'''
"""25Efork 视觉能力对外 API —— 识别黑框 / 追踪靶心中心（B 轨）。

把工程内部的「检测器 + 追踪器 + 光斑偏差 + 云台闭环」收成一层稳定门面，
**不同小问直接 import 就能用**，不必关心内部模块路径与参数。

──────────────────────────────── 两个核心 API ────────────────────────────────

① 识别黑框（**无状态**，每帧独立、全图搜索，适合"取一次框"）::

    from target_api import detect_black_frame

    res = detect_black_frame(gray)      # gray: numpy uint8 或 MaixPy image
    if res:                             # res 为真 = 识别到（见"返回值"）
        print(res.center)               # (u, v)
        print(res.quad)                 # 外框四角 (4,2)

② 追踪中心（**有状态**，窄带跟踪 + 失锁自动重捕获，适合连续视频流）::

    from target_api import CenterTracker

    tracker = CenterTracker()
    while True:
        res = tracker.track(gray)
        if res:
            print(res.u, res.v, res.stage)     # stage: track / roi_search / full_search

函数式快捷用法（内部持有一个默认追踪器，小问里一行搞定）::

    from target_api import track_center, reset
    res = track_center(gray)             # 等价于 CenterTracker().track()，实例复用
    reset()                              # 换场景/重开一题时清状态

──────────────────────────────── 可选：激光偏差与云台 ────────────────────────────────

需要"光斑到靶心的偏差"或"自动瞄准"的小问::

    tracker = CenterTracker(with_laser=True)     # res 额外带 laser_u/v、dx、dy
    res = tracker.track(gray)
    print(res.dx, res.dy)                        # dx = 靶心 − 光斑（正=靶心在光斑右侧）

    from target_api import AimLoop               # 完整闭环（会驱动电机，谨慎）
    loop = AimLoop()                             # 需已完成开环标定，见 README
    loop.step(frame)                             # 内部：追踪 → 偏差 → 发脉冲
    loop.stop()

──────────────────────────────── 返回值 ────────────────────────────────

统一返回 ``Result``（dict 子类，支持 ``res["u"]`` 与 ``res.u`` 两种写法）：

    valid      bool    是否识别/追踪到黑框
    u, v       float   靶心像素坐标（已按卡尔曼/可选 EMA 平滑，供显示/常规使用）
    raw_u/v    float   未平滑中心（闭环用，见 config.AIM_USE_RAW）
    center     tuple   (u, v)；无效时为 None
    quad       (4,2)   外框四角，顺序已规整（左上起逆时针）
    inner_quad (4,2)   内框四角（胶带内沿）
    stage      str     track / roi_search / full_search / no_black_frame
    ms         float   本帧检测耗时（毫秒）
    laser_u/v  float   光斑位置（仅 with_laser=True 且光斑有效时有意义）
    dx, dy     float   靶心 − 光斑（仅 with_laser=True 时；无效为 None）
    aim_valid  bool    靶心与光斑都有效才为 True

``Result`` 的布尔值 = ``valid``，所以 ``if res:`` 直接就是"识别到了吗"。

──────────────────────────────── 输入 ────────────────────────────────

``frame`` 可以是 numpy uint8 灰度图（``(H,W)`` 或 ``(H,W,1)``）、三/四通道图（按 RGB 处理），
也可以是 MaixPy 的 ``image.Image``（自动 ``image2cv``，需要板上环境）。
**本模块顶层不 import maix**，PC 上用 numpy 即可离线跑通、便于单测。

──────────────────────────────── 参数覆盖 ────────────────────────────────

不想改 ``config.py`` 时用 ``load_config(...)`` 造一份覆盖::

    cfg = load_config(MIN_AREA_RATIO=0.02, SEARCH_WIDTH=240)
    tracker = CenterTracker(cfg)
"""

import os
import sys
import time

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from vision.rectangle_center import RectangleCenterDetector
from vision.laser_spot import LaserSpotDetector, deviation

__all__ = ["Result", "load_config", "to_gray", "detect_black_frame",
           "CenterTracker", "track_center", "reset", "AimLoop"]

_default_tracker = None


# --------------------------------------------------------------------- 工具
class Result(dict):
    """识别/追踪结果：``res["u"]`` 与 ``res.u`` 均可；``bool(res)`` 等价于 ``res.valid``。"""

    __slots__ = ()

    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError:
            raise AttributeError("Result has no field %r" % name)

    def __setattr__(self, name, value):
        self[name] = value

    def __bool__(self):
        return bool(self.get("valid"))

    def __repr__(self):
        if not self.get("valid"):
            return "Result(invalid, stage=%s)" % self.get("stage")
        return "Result(u=%.1f, v=%.1f, stage=%s)" % (
            self.get("u", float("nan")), self.get("v", float("nan")), self.get("stage"))


def load_config(**overrides):
    """返回 config.Config（可带覆盖项，不改磁盘上的 config.py）。

    ``load_config(MIN_AREA_RATIO=0.02)`` → 派生一个只改了该字段的配置类。
    """
    from config import Config
    if not overrides:
        return Config
    cfg = type("ApiConfig", (Config,), dict(overrides))
    if "AIM_PROFILE" in overrides:
        from config import apply_aim_profile
        apply_aim_profile(cfg)
        for key, value in overrides.items():
            setattr(cfg, key, value)
    return cfg


def to_gray(frame, image_module=None):
    """把输入统一成 ``(H, W)`` uint8 灰度图；无法转换返回 None。

    兼容：numpy 数组（含 ``(H,W,1)`` 哑轴）、有 ``__array__`` 的对象、MaixPy ``image.Image``。
    三/四通道按 **RGB/RGBA** 解释（MaixPy 相机默认 RGB）。
    """
    if frame is None:
        return None
    a = frame
    if not isinstance(a, np.ndarray):
        if hasattr(a, "__array__"):
            a = np.asarray(a)
        else:
            if image_module is None:
                from maix import image as image_module
            a = image_module.image2cv(frame, ensure_bgr=False, copy=False)
    a = np.asarray(a)
    if a.ndim == 3 and a.shape[2] == 1:
        a = a[:, :, 0]
    elif a.ndim == 3 and a.shape[2] == 3:
        a = cv2.cvtColor(a, cv2.COLOR_RGB2GRAY)
    elif a.ndim == 3 and a.shape[2] == 4:
        a = cv2.cvtColor(a, cv2.COLOR_RGBA2GRAY)
    if a.ndim != 2 or a.size == 0:
        return None
    if a.dtype != np.uint8:
        a = np.clip(a, 0, 255).astype(np.uint8)
    return a


def _wrap(raw, gray):
    """把检测器的 dict 包成 Result，并补 center / image_size 等便捷字段。"""
    res = Result(raw)
    res["center"] = (res["u"], res["v"]) if res.get("valid") else None
    if gray is not None:
        res["image_size"] = (int(gray.shape[1]), int(gray.shape[0]))
    res.setdefault("ms", res.get("detector_ms", 0.))
    res.setdefault("dx", None)
    res.setdefault("dy", None)
    res.setdefault("aim_valid", False)
    return res


# ------------------------------------------------------------- API ① 识别黑框
def detect_black_frame(frame, cfg=None, detector=None):
    """**无状态**识别黑框，返回 ``Result``（没识别到则 ``valid=False``，``bool(res)`` 为假）。

    每帧都是一次完整的"缩小图Canny/Otsu 候选 → 四边暗带验证 → 亚像素精修"，不依赖上一帧，
    适合"只在某几问里取一次黑框"的用法。想复用实例提速可传 ``detector``（但那样就带状态了，
    连续调用会走窄带跟踪路径）。

    :param frame:    numpy uint8 灰度图或 MaixPy image
    :param cfg:      配置类（默认 config.Config；可用 load_config(...) 覆盖）
    :param detector: 复用已有的 RectangleCenterDetector（可选）
    """
    cfg = cfg if cfg is not None else load_config()
    det = detector if detector is not None else RectangleCenterDetector(cfg)
    gray = to_gray(frame)
    if gray is None:
        return Result(valid=False, stage="empty_frame", center=None,
                      dx=None, dy=None, aim_valid=False, ms=0.0)
    started = time.perf_counter()
    det.reset()  # This API is stateless even when callers reuse a detector allocation.
    raw = det.process(gray)
    out = _wrap(raw, gray)
    out["ms"] = out.get("detector_ms", (time.perf_counter() - started) * 1000.0)
    return out


# ------------------------------------------------------------- API ② 追踪中心
class CenterTracker:
    """**有状态**中心追踪器：窄带跟踪为主，失锁后 ROI 搜索 → 全图重捕获。

    典型用法：连续 ``track(gray)``；换场景/重新开题时 ``reset()``。
    ``with_laser=True`` 时返回值额外带光斑位置与 ``dx/dy``（靶心 − 光斑）。
    """

    def __init__(self, cfg=None, with_laser=False, use_raw=None):
        self.cfg = cfg if cfg is not None else load_config()
        self.detector = RectangleCenterDetector(self.cfg)
        self.laser = LaserSpotDetector(self.cfg) if with_laser else None
        if use_raw is None:
            use_raw = bool(getattr(self.cfg, "AIM_USE_RAW", True))
        self.use_raw = bool(use_raw)

    def reset(self):
        """清空跟踪锁定状态（换靶、换场景时调用，避免拿旧框去搜）。"""
        self.detector = RectangleCenterDetector(self.cfg)
        if self.laser is not None:
            self.laser = LaserSpotDetector(self.cfg)
        return self

    def track(self, frame, now=None):
        """追踪一帧：返回 ``Result``；``valid`` 表示本帧是否拿到靶心。"""
        gray = to_gray(frame)
        if gray is None:
            self.detector.reset()
            return Result(valid=False, stage="empty_frame", center=None,
                          dx=None, dy=None, aim_valid=False, ms=0.0)
        raw = self.detector.process(gray, timestamp=now)
        res = _wrap(raw, gray)
        if not res.get("valid"):
            return res
        if self.laser is not None:
            ls = self.laser.process(gray, now)
            res["laser"] = ls
            dv = deviation(ls, res, use_raw=self.use_raw)
            if dv is not None:
                res["laser_u"], res["laser_v"] = ls["u"], ls["v"]
                res["dx"], res["dy"] = dv
            res["aim_valid"] = dv is not None
        return res


def track_center(frame, cfg=None, now=None):
    """函数式追踪：内部复用同一个默认 ``CenterTracker``（首次调用时创建）。"""
    global _default_tracker
    if _default_tracker is None:
        _default_tracker = CenterTracker(cfg)
    return _default_tracker.track(frame, now)


def reset():
    """重置 ``track_center()`` 使用的默认追踪器。"""
    if _default_tracker is not None:
        _default_tracker.reset()


# ------------------------------------------------------- 可选：追踪 + 云台闭环
class AimLoop:
    """追踪 + 光斑偏差 + 双轴云台闭环（会驱动 ZDT 电机）。

    ⚠ 使用前必须完成开环标定（``CALIBRATE_AXIS`` 跑一次填回 config），否则方向/增益
    是占位值，可能正反馈甩头。不需要电机的问请只用 ``detect_black_frame`` /
    ``CenterTracker``，不要实例化本类。

    ``step(frame)`` 返回的 ``Result`` 比追踪器多两个字段：
        aim_state  str   状态机状态（FINE/COARSE/SETTLE/HOLD/FAULT…）
        cmds       dict  {"yaw": (ccw, pulses, rpm) or None, "pitch": ...}
    """

    def __init__(self, cfg=None, enable_motors=True, with_laser=True, clock=None, heading=None):
        self.cfg = cfg if cfg is not None else load_config()
        self.clock = clock if clock is not None else time
        self.tracker = CenterTracker(self.cfg, with_laser=with_laser)
        self.link = self.aim = None
        self.closed = False
        if enable_motors:
            from vision.stepper_link import StepperLink
            from vision.gimbal_aim import GimbalAim
            self.link = StepperLink(self.cfg, clock=self.clock)
            self.aim = GimbalAim(self.cfg)
            if self.link.enable() is False:
                self.aim.fault("enable_failed")
        from vision.aim_control import VisionControl
        calibrator = None
        if enable_motors:
            from vision.gimbal_calibrate import GimbalCalibrator
            cal_cfg = type("YawCalibration", (self.cfg,), {"CALIBRATE_AXIS": "yaw"}) if self.cfg.CALIBRATE_AXIS == "auto" else self.cfg
            calibrator = GimbalCalibrator(cal_cfg, self.link, self.clock.perf_counter())
            if calibrator.done:
                calibrator = None
            if heading is None and (getattr(self.cfg, "ENABLE_IMU_FEEDFORWARD", False) or (getattr(self.cfg, "ENABLE_SEARCH_BUTTON", True) and getattr(self.cfg, "SEARCH_USE_IMU", True))):
                from vision.imu_heading import HeadingFeedforward
                heading = HeadingFeedforward(self.cfg, clock=self.clock)
        self.control = VisionControl(self.cfg, self.link, self.aim, calibrator, clock=self.clock, heading=heading)
        if self.cfg.CALIBRATE_AXIS == "auto":
            self.control._auto_axis = "yaw"

    def step(self, frame, now=None):
        if self.closed:
            raise RuntimeError("AimLoop is closed")
        captured = self.clock.perf_counter() if now is None else float(now)
        res = self.tracker.track(frame, captured)
        out = self.control.accept_result(res, captured)
        return _wrap(out, to_gray(frame))

    def idle(self):
        if not self.closed:
            self.control.idle()

    def request_search(self):
        if not self.closed:
            self.control.request_search.set()

    def stop(self):
        if self.link is not None:
            self.control._stop_motion(self.clock.perf_counter())

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.stop()
        finally:
            try:
                self.control.close()
            finally:
                if self.link is not None:
                    self.link.close()
'''

_MAIN_SRC = r'''
"""MaixPy v4 entry. Configuration is in config.py; runtime lives in vision/."""
import os
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from config import Config
from vision.gimbal_aim import GimbalAim
from vision.gimbal_self_test import probe_imu
from vision.gimbal_calibrate import GimbalCalibrator
from vision.rate_meter import FrameRate
from vision.camera_source import to_gray
from vision.vision_worker import LatestFrameWorker
from vision.preview import Preview, TouchButtons, display_result, apply_maixvision_stream_policy
from vision.aim_control import VisionControl, send_uart
from vision.app_runtime import run

BUILD_ID = "2026-10-17-canny-kalman-v0.8.0"

# Compatibility aliases for existing PC tools and board integrations.
_FrameRate, _LatestVision, _Preview, _Touch = FrameRate, LatestFrameWorker, Preview, TouchButtons
_VisionControl, _display_result, _to_gray = VisionControl, display_result, to_gray
_send_uart, _apply_maixvision_stream_policy = send_uart, apply_maixvision_stream_policy


def main():
    return run(sys.modules[__name__], BUILD_ID)


if __name__ == "__main__":
    try:
        main()
    except ModuleNotFoundError as exc:
        if exc.name != "maix":
            raise
        print("MaixPy SDK unavailable. PC: python -B tools/check_pc.py")
'''

_MOD_ORDER = ('config', 'vision.rate_meter', 'vision.camera_source', 'vision.vision_worker', 'vision.telemetry', 'vision.maixvision_stream', 'vision.preview', 'vision.rect_geometry', 'vision.contour_locator', 'vision.kalman_filter', 'vision.rectangle_center', 'vision.laser_spot', 'vision.stepper_link', 'vision.imu_heading', 'vision.gimbal_self_test', 'vision.gimbal_aim', 'vision.gimbal_search', 'vision.aim_control', 'vision.gimbal_calibrate', 'vision.app_runtime', 'vision', 'target_api')

_REL_OF = {'config': 'config.py', 'vision.rate_meter': 'vision/rate_meter.py', 'vision.camera_source': 'vision/camera_source.py', 'vision.vision_worker': 'vision/vision_worker.py', 'vision.telemetry': 'vision/telemetry.py', 'vision.maixvision_stream': 'vision/maixvision_stream.py', 'vision.preview': 'vision/preview.py', 'vision.rect_geometry': 'vision/rect_geometry.py', 'vision.contour_locator': 'vision/contour_locator.py', 'vision.kalman_filter': 'vision/kalman_filter.py', 'vision.rectangle_center': 'vision/rectangle_center.py', 'vision.laser_spot': 'vision/laser_spot.py', 'vision.stepper_link': 'vision/stepper_link.py', 'vision.imu_heading': 'vision/imu_heading.py', 'vision.gimbal_self_test': 'vision/gimbal_self_test.py', 'vision.gimbal_aim': 'vision/gimbal_aim.py', 'vision.gimbal_search': 'vision/gimbal_search.py', 'vision.aim_control': 'vision/aim_control.py', 'vision.gimbal_calibrate': 'vision/gimbal_calibrate.py', 'vision.app_runtime': 'vision/app_runtime.py', 'vision': 'vision/__init__.py', 'target_api': 'target_api.py'}

_pkg = _types.ModuleType("vision")          # 造出虚拟包 vision
_pkg.__path__ = []
_sys.modules["vision"] = _pkg
for _full in _MOD_ORDER:
    if _full == "vision":
        continue
    if "." in _full:                       # vision.rectangle_center 这类子模块
        _parent, _leaf = _full.split(".", 1)
        _m = _types.ModuleType(_full)
        setattr(_sys.modules[_parent], _leaf, _m)
        _sys.modules[_full] = _m
    else:                                  # config / target_api 这类顶层模块
        _sys.modules[_full] = _types.ModuleType(_full)
for _full in _MOD_ORDER:
    _m = _sys.modules[_full]
    _m.__dict__["__name__"] = _full
    _m.__dict__["__file__"] = _REL_OF[_full]   # 源码里若用 __file__（如 target_api）
    _m.__dict__["__source__"] = _MOD_SRC[_full]
    _m.__dict__["__package__"] = "vision" if _full == "vision" else _full.rpartition(".")[0]
    exec(compile(_MOD_SRC[_full], _full.replace(".", "/") + ".py", "exec"),
         _m.__dict__)
del _m, _full, _MOD_SRC

# --------------------------------------------------------------------------
# ② 主程序：main.py 原文照搬，exec 进本文件全局命名空间
#    这样 __name__ == "__main__" 判断、traceback 文件名都与原来一致
# --------------------------------------------------------------------------
exec(compile(_MAIN_SRC, 'main.py', "exec"), globals())
