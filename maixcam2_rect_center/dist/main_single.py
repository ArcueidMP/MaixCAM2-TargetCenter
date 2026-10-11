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

生成时间：2026-10-12 00:10:17
    config.py                    sha1=f295d9761c03
    main.py                      sha1=00d27042f9d0
    target_api.py                sha1=9c5a71aa209b
    vision/gimbal_aim.py         sha1=0b0ae21ebd2e
    vision/gimbal_calibrate.py   sha1=9c4bb08a1b99
    vision/gimbal_self_test.py   sha1=17b72e0935bb
    vision/laser_spot.py         sha1=c131498c5ff9
    vision/rectangle_center.py   sha1=8aa5e868f9ea
    vision/stepper_link.py       sha1=29501ea53b74
"""

import sys as _sys
import types as _types

_SRC_SHA1 = {'config.py': 'f295d9761c036475dbe0cfe4ac07617f40cab868', 'vision/rectangle_center.py': '8aa5e868f9ea2a9a7d243ff4879e8295652d4c0f', 'vision/laser_spot.py': 'c131498c5ff980e594b1c8d5d2feda6195caee23', 'vision/gimbal_self_test.py': '17b72e0935bbb7786f7c936d17b55c291795990b', 'vision/gimbal_aim.py': '0b0ae21ebd2e6cce2d7839fccf03dc8ada5c02d3', 'vision/stepper_link.py': '29501ea53b740bb704bbb8da166b5b4b6eabfffb', 'vision/gimbal_calibrate.py': '9c4bb08a1b99d6feaef77bc8cf3fc56655e87676', 'target_api.py': '9c5a71aa209bb2716f1c0c4ac2fadd3eec23241d', 'main.py': '00d27042f9d01aef8305b2a351a0f0f57d779d2a'}

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
'''

# ===== vision.rectangle_center  (vision/rectangle_center.py) =====
_MOD_SRC['vision.rectangle_center'] = r'''
"""黑环/亮面双候选、稀疏透视验证、四边直线跟踪。仅依赖 cv2/numpy。

搜索在缩小图上执行；精修和坐标输出始终使用当前帧的原始分辨率。
已锁定时只采样四条窄带，不运行全图 Canny、轮廓扫描或阈值遍历。

================================================================================
几何原理（为什么靶心能"算"出来）
================================================================================
靶纸四周贴 1.8cm 黑胶带形成一个矩形环，环的两个"内框角点"构成一个**平面四边形**。
相机斜看时该四边形在图像里是透视投影，但两条对角线的交点 = 靶心，这是**射影不变量**
（任意透视变换下，四边对角线的交点都映射到对应四边形的对角线交点）。因此：
    检测四边形四角 → 求对角线交点 = 靶心像素坐标 (u,v)，
无需解单应、也无需知道相机内外参。本模块即按此实现。

================================================================================
单帧处理链（process 的骨架，四段）
================================================================================
    ① 候选生成 _candidate_quads : 阈值化 → 轮廓树 → 取四边形轮廓，按内外框打标
    ② 搜索 _search              : 在缩小图上轮换阈值/候选，时间预算内做有限次验证
    ③ 四边验证 _verify          : 用"胶带厚度方向的暗带 + 内侧亮面"四边覆盖率判真伪
    ④ 亚像素精修 _refine        : 当前帧四条灰度窄带 → 有向边缘跳变 → fitLine → 四角交点
    末：把四角规整为外框，取正视面中心投影回原图 → (u,v)，并按 CENTER_ALPHA 做一阶平滑。

三种进入方式（process 里按上一帧是否锁定决定，越靠前越省算力）：
    track       已锁定 → 只沿四边窄带精修（不跑 Canny/轮廓/阈值遍历，稳态 ~1ms）
    roi_search  刚失锁 → 在上一帧外框外扩 ROI_PADDING 的窗口内搜索（一次）
    full_search 彻底丢失 → 在全图缩小图上多阈值扫描 + Canny 兜底
"""
import time

import cv2
import numpy as np


def _polygon_area(p):
    """带符号面积的 2 倍（>0 = 逆时针 CCW，<0 = 顺时针 CW）。用于把四角规整成统一方向。"""
    return float(cv2.contourArea(np.asarray(p, np.float32).reshape(-1, 1, 2), oriented=True))


def _order_quad(points):
    """把 4 个点规整为统一顺序：以质心为极点按极角排序 → 保证 CCW → 旋转到"左上角"开头。

    统一顺序是后续所有几何运算（边向量、对角线、单应四点对应）的前提。
    返回顺序语义：**[左上, 右上, 右下, 左下]**（逆时针），与 _side_lengths 的边 0/1/2/3 对应。
    """
    p = np.asarray(points, np.float32).reshape(4, 2)
    c = p.mean(axis=0)
    p = p[np.argsort(np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0]))]   # 按极角排序
    if _polygon_area(p) < 0:
        p = p[::-1]                                                  # 反了则翻转成 CCW
    # np.roll 到"x+y 最小"的点（即左上角）打头。
    return np.roll(p, -int(np.argmin(p.sum(axis=1))), axis=0).copy()


def _side_lengths(q):
    """四边长度 [边0, 边1, 边2, 边3]，边 i = 从点 i 指向点 i+1（roll -1 取下一个点）。"""
    return np.linalg.norm(np.roll(q, -1, axis=0) - q, axis=1)


def _gaussian_solve(a, b):
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


def _homography(src, dst):
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
    v = _gaussian_solve(a, uv.reshape(-1))
    hn = np.append(v, 1.0).reshape(3, 3)                      # 归一化域下的 H
    ts = np.array([[1 / ss, 0, -sc[0] / ss], [0, 1 / ss, -sc[1] / ss], [0, 0, 1]])
    td_inv = np.array([[ds, 0, dc[0]], [0, ds, dc[1]], [0, 0, 1]])
    h = td_inv @ hn @ ts                                      # 还原到原坐标域
    return h / h[2, 2] if abs(h[2, 2]) > 1e-12 else h


def homography_8x8(src_quad, width_mm, height_mm):
    """把物理矩形 (0,0)-(w,h) 映射到图像四边形 src_quad 的单应（mm → px）。"""
    return _homography(src_quad, [[0, 0], [width_mm, 0],
                                 [width_mm, height_mm], [0, height_mm]])


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


def _rectangle(w, h, inset=0.0):
    """物理矩形四角（mm），inset>0 时向内缩进（用于取胶带内沿 = 内框）。顺序 LL→LR→UR→UL。"""
    return np.array([[inset, inset], [w - inset, inset],
                     [w - inset, h - inset], [inset, h - inset]], np.float64)


def _sample(gray, points):
    """一次 OpenCV remap 双线性采样整批点，越界由调用方拒绝。

    相比逐点 Python 循环，remap 一次调用即可采样成百上千个点（四边验证/精修的关键提速）。
    borderMode=REPLICATE 保证边缘点也能返回数值（越界判据在调用方单独做）。
    """
    shape = points.shape[:-1]
    p = points.reshape(-1, 2).astype(np.float32)
    values = cv2.remap(gray, p[:, 0].reshape(1, -1), p[:, 1].reshape(1, -1),
                       cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    return values.reshape(shape)


class RectangleCenterDetector:
    """有状态检测器：跨帧记住锁定框(_track)与平滑中心(_smooth)，稳态只做窄带精修。

    状态字段：
        _track       上一帧验证通过的外框（dict 候选）；None = 未锁定
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

    def _valid_quad(self, q, shape, margin=1):
        """四边形体检：不越界、凸、边长下限、无明显退化尖角。

        允许任意透视（不要求接近直角/矩形），只排除两类坏形状：
          · 贴边/出界：角点落在图像边缘 margin 内 → 采样会越界、中心不可信；
          · 退化尖角：相邻边几乎平行（sin 很小）→ 四角拟合不稳定。
        """
        h, w = shape
        if (q[:, 0].min() < margin or q[:, 1].min() < margin or
                q[:, 0].max() >= w - margin or q[:, 1].max() >= h - margin):
            return False
        if not cv2.isContourConvex(np.asarray(q, np.float32).reshape(-1, 1, 2)):
            return False
        e = np.roll(q, -1, axis=0) - q
        lengths = np.linalg.norm(e, axis=1)
        if lengths.min() < 12:                                 # 最短边至少 12px
            return False
        # 透视图不要求四角接近 90 度，只排除退化尖角和过扁的四边形。
        # 相邻边向量叉积 / (边长积) = sin(夹角)，要求每个角 > ~11.5°。
        sine = np.abs(e[:, 0] * np.roll(e[:, 1], -1) -
                      e[:, 1] * np.roll(e[:, 0], -1)) / (lengths * np.roll(lengths, -1))
        return bool(sine.min() > 0.20)

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

    def _verify(self, gray, q, kind):
        """在正视面选取四边稀疏采样点并投影，不生成整幅 warp 图。

        每条边检查胶带厚度方向的暗带覆盖率与内侧亮区对比，允许局部反光；
        背景可以是黑色，不再强制外侧为白色。普通纸边及实心黑块会失败。
        """
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
        # 至少三边的大部分采样成立，第四边也必须有证据，不接受凭亮中心猜框。
        if (np.count_nonzero(coverage >= self.cfg.MIN_EDGE_COVERAGE) < 3 or
                coverage.min() < self.cfg.WEAK_EDGE_COVERAGE or
                np.median(edge_contrast) < self.cfg.MIN_EDGE_CONTRAST):
            return None
        rows = [dict(dark=float(np.median(dark[i])), inside=float(np.median(bright[i])),
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
                    score=float(np.median(edge_contrast) + 60 * coverage.mean()))

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
        for i in range(4):
            good = inside[i] & (strength[i] >= self.cfg.MIN_EDGE_CONTRAST * 0.7)   # 有效采样
            if good.sum() < max(5, len(self._t) * self.cfg.WEAK_EDGE_COVERAGE):
                return None                                                # 证据太少，整框弃用
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

    def _candidate_quads(self, mask):
        """用轮廓树的孔/黑环关系代替候选两两组合。

        输入是二值掩膜（暗区为前景）。输出 [(quad, kind), ...]：
          · 借 findContours 的 RETR_TREE 层级：黑胶带环 = 一个"孔"（有子级）；
            深度奇偶决定 kind —— 偶数深度=outer(外框)，奇数=inner(内框)。
          · 用**包围框/面积**先廉价预筛（剔过小、过扁），只对最大若干个轮廓做 approxPolyDP，
            控制昂贵的多边形逼近次数。
          · 排除"实心黑块"（outer 却没有亮孔），因为靶纸黑环中心必是亮的。
        返回排序把 inner 排前：亮面孔比与背景粘连的黑环更稳定。
        """
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        if hierarchy is None:
            return []
        hierarchy = hierarchy[0]
        h, w = mask.shape
        candidates = []
        # 先用廉价的包围框/面积筛选，仅拟合最大的少数轮廓。
        eligible = []
        # findContours 本身是原子调用；后续 Python 遍历只看最大的有限个轮廓。
        ids = sorted(range(len(contours)), key=lambda i: abs(cv2.contourArea(contours[i])),
                     reverse=True)[:self.cfg.MAX_CONTOURS]
        for i in ids:
            cnt = contours[i]
            x, y, cw, ch = cv2.boundingRect(cnt)
            if min(cw, ch) < 15 or cw * ch < w * h * self.cfg.MIN_AREA_RATIO:
                continue                                   # 太小/太扁：不是候选框
            area = abs(cv2.contourArea(cnt))
            if w * h * self.cfg.MIN_AREA_RATIO <= area <= w * h * self.cfg.MAX_AREA_RATIO:
                eligible.append((area, i))
        for area, i in sorted(eligible, reverse=True)[:self.cfg.MAX_CANDIDATES]:
            cnt = contours[i]
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, self.cfg.APPROX_EPSILON * peri, True)   # 多边形逼近
            # 不使用凸包修补任意背景：凸包可把桌面阴影拼成错误大四边形。
            if len(approx) != 4 or not cv2.isContourConvex(approx):
                continue                                   # 只接受凸四边形
            q = _order_quad(approx)
            if not self._valid_quad(q, mask.shape):
                continue
            parent, depth = int(hierarchy[i][3]), 0        # 沿父链数深度
            while parent >= 0:
                depth += 1
                parent = int(hierarchy[parent][3])
            kind = "inner" if depth % 2 else "outer"       # 奇偶定内外框
            if kind == "outer" and hierarchy[i][2] < 0:
                continue  # 实心黑块没有亮孔。
            candidates.append((q, kind))
        # 亮面孔比与背景粘连的黑环更稳定。
        return sorted(candidates, key=lambda c: c[1] != "inner")

    def _search(self, gray, roi=None):
        """在缩小图上找候选并验证；**合作式**：每帧只推进有限步，跨帧延续。

        关键设计（性能与鲁棒性的平衡）：
          · 缩小到 SEARCH_WIDTH 宽再阈值化，全图扫描代价大幅下降；坐标末了乘回原分辨率。
          · 阈值列表 = [上次成功阈值] + [Otsu] + DARK_THRESHOLDS；持续失败时**跨帧轮换**
            （job["index"] 在 _search_jobs 里记进度），避免单帧把全部阈值跑一遍。
          · SEARCH_MAX_PROPOSALS=1：每帧最多验证一个候选；时间预算 SEARCH_BUDGET_MS 是软预算
            （至少验证一个，防止原子调用稍慢导致永远不推进）。
          · 全部阈值候选耗尽后，才用 Canny 轮廓兜底（最贵，只在彻底找不到时用）。
        找到即锁定 _locked_th 并清空该 ROI 的待验证队列，下次直接复用。
        """
        self._search_count += 1
        x, y, w, h = roi if roi is not None else (0, 0, gray.shape[1], gray.shape[0])
        crop = gray[y:y + h, x:x + w]
        scale = min(1.0, self.cfg.SEARCH_WIDTH / float(w))
        sw, sh = max(32, int(round(w * scale))), max(24, int(round(h * scale)))
        small = cv2.resize(crop, (sw, sh), interpolation=cv2.INTER_AREA) if scale < 1 else crop
        sx, sy = w / float(sw), h / float(sh)              # 缩小图 → 原图 的坐标比例
        # 阈值候选：优先复用上次成功值，再并入 Otsu 与固定档位（去掉与已有值太接近的）。
        thresholds = [self._locked_th] if self._locked_th is not None else []
        otsu, _ = cv2.threshold(small, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        for th in (int(otsu),) + tuple(self.cfg.DARK_THRESHOLDS):
            if not any(abs(th - t) < 6 for t in thresholds):
                thresholds.append(th)
        # 持续失败时跨帧轮换阈值；只缓存候选几何，不保存旧图像或旧验证结果。
        key = tuple(roi) if roi is not None else "full"
        job = self._search_jobs.setdefault(key, dict(index=0, pending=[], threshold=None))
        if len(self._search_jobs) > 4:
            self._search_jobs = {key: job}                 # 最多只留 4 个 ROI 的进度
        pending = job["pending"]
        if not pending:
            index = job["index"] % (len(thresholds) + 1)   # 最后一档留作 Canny 兜底
            job["index"] += 1
            if index < len(thresholds):
                th = thresholds[index]
                job["threshold"] = th
                self._threshold_count += 1
                _, mask = cv2.threshold(small, th, 255, cv2.THRESH_BINARY_INV)   # 暗区为前景
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)     # 闭运算补断线
                # 候选坐标从缩小图坐标乘回原图（+ROI 偏移）。
                pending.extend((q * np.array([sx, sy], np.float32) + [x, y], kind)
                               for q, kind in self._candidate_quads(mask))
            else:
                job["threshold"] = None
                # Canny 兜底：边缘图直接找四边形轮廓，内/外框都试。
                edges = cv2.Canny(small, self.cfg.CANNY_LOW, self.cfg.CANNY_HIGH)
                contours = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
                for cnt in sorted(contours, key=cv2.contourArea, reverse=True)[:self.cfg.MAX_CANDIDATES]:
                    if cv2.contourArea(cnt) < sw * sh * self.cfg.MIN_AREA_RATIO:
                        continue
                    ap = cv2.approxPolyDP(cnt, self.cfg.APPROX_EPSILON * cv2.arcLength(cnt, True), True)
                    if len(ap) == 4 and cv2.isContourConvex(ap):
                        q = _order_quad(ap) * [sx, sy] + [x, y]
                        pending.extend(((q, "inner"), (q, "outer")))
        # 逐个验证候选；每帧最多 SEARCH_MAX_PROPOSALS 个。
        while pending and self._proposals < self.cfg.SEARCH_MAX_PROPOSALS:
            # 至少验证一个候选，避免原子轮廓扫描稍慢导致永远没有进展。
            if self._proposals and time.perf_counter() >= self._deadline:
                break
            q, kind = pending.pop(0)
            self._proposals += 1
            verified = self._verify(gray, q, kind)         # 四边暗带验证
            if verified is None:
                continue
            refined = self._refine(gray, q, kind, self.cfg.REFINE_RADIUS)   # 亚像素精修
            if refined is not None:
                precise = self._verify(gray, refined, kind)                 # 精修后再验证
                if precise is not None:
                    verified = precise                    # 精修通过则用更准的结果
            self._locked_th = job["threshold"]             # 记住本次成功阈值
            job["pending"] = []
            job["index"] = 0
            return verified
        return None

    def _legacy_search(self, gray, roi=None):
        """保留可选线段恢复路径使用的完整搜索；常规帧不调用。"""
        x, y, w, h = roi if roi is not None else (0, 0, gray.shape[1], gray.shape[0])
        crop = gray[y:y + h, x:x + w]
        scale = min(1.0, self.cfg.SEARCH_WIDTH / float(w))
        sw, sh = max(32, int(round(w * scale))), max(24, int(round(h * scale)))
        small = cv2.resize(crop, (sw, sh), interpolation=cv2.INTER_AREA) if scale < 1 else crop
        sx, sy = w / float(sw), h / float(sh)
        thresholds = [self._locked_th] if self._locked_th is not None else []
        otsu, _ = cv2.threshold(small, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        for th in (int(otsu),) + tuple(self.cfg.DARK_THRESHOLDS):
            if not any(abs(th - t) < 6 for t in thresholds):
                thresholds.append(th)
        for th in thresholds:
            self._threshold_count += 1
            _, mask = cv2.threshold(small, th, 255, cv2.THRESH_BINARY_INV)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
            for q, kind in self._candidate_quads(mask):
                q = q * np.array([sx, sy], np.float32) + np.array([x, y], np.float32)
                verified = self._verify(gray, q, kind)
                if verified is None:
                    continue
                refined = self._refine(gray, q, kind, self.cfg.REFINE_RADIUS)
                if refined is not None:
                    precise = self._verify(gray, refined, kind)
                    if precise is not None:
                        verified = precise
                self._locked_th = th
                return verified
        # Canny 是失锁后的最后辅助，避免每帧全图计算梯度。
        edges = cv2.Canny(small, self.cfg.CANNY_LOW, self.cfg.CANNY_HIGH)
        contours = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:self.cfg.MAX_CANDIDATES]
        for cnt in contours:
            if cv2.contourArea(cnt) < sw * sh * self.cfg.MIN_AREA_RATIO:
                continue
            ap = cv2.approxPolyDP(cnt, self.cfg.APPROX_EPSILON * cv2.arcLength(cnt, True), True)
            if len(ap) != 4 or not cv2.isContourConvex(ap):
                continue
            q = _order_quad(ap) * [sx, sy] + [x, y]
            for kind in ("inner", "outer"):
                verified = self._verify(gray, q, kind)
                if verified is not None:
                    refined = self._refine(gray, q, kind, self.cfg.REFINE_RADIUS)
                    precise = self._verify(gray, refined, kind) if refined is not None else None
                    return precise or verified
        # 透明膜的反光可能把整条黑环切开，内孔也不再闭合。
        # 仅失锁时从长线段拼四边，并通过相同暗带验证，绝不凭亮中心猜框。
        if not getattr(self.cfg, "ENABLE_LINE_RECOVERY", False):
            return None
        line_edges = cv2.Canny(cv2.GaussianBlur(small, (5, 5), 0),
                               self.cfg.CANNY_LOW, self.cfg.CANNY_HIGH)
        return self._line_search(gray, line_edges, sx, sy, x, y)

    def _line_search(self, gray, edges, sx, sy, x, y):
        h, w = edges.shape
        segments = cv2.HoughLinesP(edges, 1, np.pi / 180, 25,
                                   minLineLength=max(25, int(min(w, h) * .16)), maxLineGap=12)
        if segments is None:
            return None
        p = segments.reshape(-1, 2, 2).astype(np.float64)
        delta = p[:, 1] - p[:, 0]
        lengths = np.linalg.norm(delta, axis=1)
        ids = np.argsort(lengths)[-64:]
        p, delta, lengths = p[ids], delta[ids], lengths[ids]
        lines = np.cross(np.column_stack((p[:, 0], np.ones(len(p)))),
                         np.column_stack((p[:, 1], np.ones(len(p)))))
        angles = np.arctan2(delta[:, 1], delta[:, 0]) % np.pi
        # 以长线段为种子，最多尝试三组主方向，兼容靶紙绕光轴旋转。
        tried, best = [], None
        for seed in np.argsort(lengths)[::-1]:
            angle = angles[seed]
            distance = lambda a, b: np.abs((a - b + np.pi / 2) % np.pi - np.pi / 2)
            if any(distance(angle, old) < np.deg2rad(20) for old in tried):
                continue
            tried.append(angle)
            groups = []
            for direction in (angle, (angle + np.pi / 2) % np.pi):
                mask = distance(angles, direction) < np.deg2rad(28)
                normal = np.array([-np.sin(direction), np.cos(direction)])
                positions = p.mean(axis=1) @ normal
                group = []
                for index in np.argsort(lengths)[::-1]:
                    if not mask[index]:
                        continue
                    if any(abs(positions[index] - positions[old]) < 5 for old in group):
                        continue
                    group.append(index)
                    if len(group) >= 8:
                        break
                pairs = []
                for ii, a in enumerate(group):
                    for b in group[ii + 1:]:
                        if abs(positions[a] - positions[b]) >= min(w, h) * .24:
                            pairs.append((lengths[a] + lengths[b], a, b))
                groups.append(sorted(pairs, reverse=True)[:6])
            for _, a, b in groups[0]:
                for _, c, d in groups[1]:
                    intersections = np.cross(lines[[a, b, b, a]], lines[[c, c, d, d]])
                    if np.any(np.abs(intersections[:, 2]) < 1e-8):
                        continue
                    q = _order_quad(intersections[:, :2] / intersections[:, 2:])
                    q = q * [sx, sy] + [x, y]
                    for kind in ("inner", "outer"):
                        check = self._verify(gray, q, kind)
                        if check is None:
                            continue
                        refined = self._refine(gray, q, kind, self.cfg.REFINE_RADIUS)
                        precise = self._verify(gray, refined, kind) if refined is not None else None
                        check = precise or check
                        if best is None or check["score"] > best["score"]:
                            best = check
            if best is not None or len(tried) >= 3:
                break
        return best

    def process(self, gray, timestamp=None):
        """单帧入口：按"track → roi_search → full_search"三级降级找框，输出中心与结果 dict。"""
        started = time.perf_counter()
        g = np.asarray(gray)
        if g.ndim == 3 and g.shape[2] == 1:
            g = g[:, :, 0]
        if g.ndim != 2 or g.dtype != np.uint8:
            raise ValueError("process expects a uint8 grayscale image")
        self._frame += 1
        # 每帧重置的临时统计与时间预算（预算内没找到就留给下一帧继续）。
        self._search_count = self._threshold_count = 0
        self._proposals = 0
        self._deadline = started + self.cfg.SEARCH_BUDGET_MS / 1000.0
        candidate, stage = None, "search"
        # ---- ① track：上一帧已锁定 → 只用四边窄带精修（最省，稳态走这里）----
        if self._track is not None:
            q, kind = self._track["quad"], self._track["kind"]
            q = self._refine(g, q, kind, self.cfg.TRACK_RADIUS)
            if q is not None:
                candidate = self._verify(g, q, kind)
                stage = "track"
        # ---- ② roi_search：刚失锁 → 在上一帧外框外扩的窗口内搜一次 ----
        if candidate is None and self._track is not None and self._roi_retry:
            q = self._track["outer_quad"]
            lo, hi = q.min(axis=0), q.max(axis=0)
            pad = np.maximum((hi - lo) * self.cfg.ROI_PADDING, 24)         # 外扩量（至少 24px）
            lo = np.maximum(0, np.floor(lo - pad)).astype(int)
            hi = np.minimum([g.shape[1], g.shape[0]], np.ceil(hi + pad)).astype(int)
            candidate = self._search(g, (int(lo[0]), int(lo[1]), int(hi[0] - lo[0]), int(hi[1] - lo[1])))
            stage = "roi_search"
            self._roi_retry = False                                        # 只允许一次 ROI 搜索
        elif candidate is None:
            # ---- ③ full_search：彻底丢失 → 全图多阈值 + Canny 兜底 ----
            candidate = self._search(g)
            stage = "full_search"
        if candidate is None:
            # 三路都没找到：清空平滑中心，返回无效结果（不沿用旧框，避免"粘住"）。
            self._smooth = None
            self.last = dict(valid=False, stage="no_black_frame", search_count=self._search_count,
                             thresholds=self._threshold_count, proposals=self._proposals,
                             detector_ms=(time.perf_counter() - started) * 1000)
            return self.last
        # ---- 找到：锁定、清进度、算中心 ----
        self._track = candidate
        self._roi_retry = True
        self._search_jobs.clear()
        w, h = candidate["width_mm"], candidate["height_mm"]
        matrix = candidate["rect_to_image"]
        # 直接映射正视面圆心才是正确中心，圆周采样不用于平均中心。
        # （中心 = 矩形中心的对角线交点；由 mm→px 单应把 (w/2,h/2) 投回原图即得。）
        raw = project_points(matrix, [[w / 2, h / 2]])[0]
        alpha = float(self.cfg.CENTER_ALPHA)
        # 一阶低通：平滑值只在显示口径使用；raw 供闭环（见 config.AIM_USE_RAW）。
        # 跳变过大（FILTER_RESET_PX）直接重置，避免平滑把真运动拖成滞后。
        if (self._smooth is None or alpha <= 0 or
                np.linalg.norm(raw - self._smooth) > self.cfg.FILTER_RESET_PX):
            center = raw
        else:
            center = self._smooth + alpha * (raw - self._smooth)
        self._smooth = center
        # 圆周采样点（仅可视化/交叉校验用）按尺寸缓存，避免每帧重算。
        key = (w, h, self.cfg.CIRCLE_RADIUS_MM, self.cfg.CIRCLE_SAMPLES)
        if key not in self._circle_cache:
            self._circle_cache[key] = standard_circle(*key)
        result = dict(candidate)
        result.update(valid=True, u=float(center[0]), v=float(center[1]), raw_u=float(raw[0]), raw_v=float(raw[1]),
                      quad=candidate["outer_quad"], detected_quad=candidate["quad"],
                      outer_inferred=candidate["kind"] == "inner",
                      circle_points=project_points(matrix, self._circle_cache[key]),
                      stage=stage, threshold=self._locked_th, search_count=self._search_count,
                      thresholds=self._threshold_count, proposals=self._proposals,
                      detector_ms=(time.perf_counter() - started) * 1000)
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

# ===== vision.gimbal_self_test  (vision/gimbal_self_test.py) =====
_MOD_SRC['vision.gimbal_self_test'] = r'''
"""上电云台小幅自检：yaw ±15°，pitch ±30°，最后回到启动姿态。

两种幅度口径（`config.SELF_TEST_PULSES` 决定用哪种）：
    · >0 = **固定脉冲**（推荐，见 AGENTS.md §3.2）：不依赖"脉冲/度"标定即可动作，
      所以可由屏幕按钮直接触发；顺序与角度模式一致（yaw 左→右、pitch 上→下，各回到起点）。
    · ==0 = 按 `SELF_TEST_*_DEG × *_PULSES_PER_DEG` 换算角度；未标定（两者均 0）则跳过。
"""


class GimbalSelfTest:
    """非阻塞自检状态机。角度通过实测的电机脉冲/输出轴度数换算。"""

    def __init__(self, cfg, link, now=0.0):
        self.cfg = cfg
        self.link = link
        self.state = "IDLE"
        self.index = 0
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
        move_s = pulses / max(1.0, pulses_per_rev * max(1, rpm) / 60.0)
        settle_s = float(getattr(self.cfg, "SELF_TEST_SETTLE_S", 0.35))
        self.deadline = now + move_s + settle_s
        self.index += 1
        print("[self-test] %s %s %dp rpm=%d step=%d/%d" %
              (axis, "CCW" if ccw else "CW", pulses, rpm,
               self.index, len(self.sequence)))
        return self.state


def probe_imu():
    """尽力实例化 MaixPy IMU 并读取一次样本；不影响主程序运行。"""
    try:
        import maix
        imu = getattr(maix, "imu", None)
        if imu is None:
            import importlib
            imu = importlib.import_module("maix.imu")
        names = [name for name in dir(imu) if not name.startswith("_")]
        device = None
        sample = None
        errors = []
        for class_name in ("IMU", "Imu", "imu"):
            constructor = getattr(imu, class_name, None)
            if not callable(constructor):
                continue
            try:
                device = constructor()
                break
            except Exception as exc:
                errors.append("%s(): %s" % (class_name, exc))
        if device is None:
            return {"available": False, "module": imu, "names": names,
                    "error": "; ".join(errors) or "module has no recognized IMU constructor"}
        for method_name in ("read", "get_data", "get_accel_gyro", "get_gyro"):
            method = getattr(device, method_name, None)
            if callable(method):
                try:
                    sample = method()
                    return {"available": True, "module": imu, "names": names,
                            "device": device, "method": method_name,
                            "sample": sample, "error": None}
                except Exception as exc:
                    errors.append("%s(): %s" % (method_name, exc))
        return {"available": False, "module": imu, "names": names,
                "device": device, "error": "; ".join(errors) or
                "IMU object has no recognized read method"}
    except Exception as exc:
        return {"available": False, "module": None, "names": [], "error": str(exc)}
'''

# ===== vision.gimbal_aim  (vision/gimbal_aim.py) =====
_MOD_SRC['vision.gimbal_aim'] = r'''
"""基于像素误差的两轴增量比例控制器 + 状态机。

对照 `MaixCAM2串口直控云台需求文档.md` §7 实现：

- 闭环输入 = ``result["dx"]/["dy"]``（靶心 − 光斑，见 ``laser_spot.deviation``）；
- 每帧只发一帧**相对位置**命令（0xFD、mode=0），单帧脉冲限幅，``acc=0``（增量闭环）；
- 状态机 ``BOOT / COARSE / FINE / SETTLE / HOLD / SCAN / FAULT``；
- 三重保护：离群拒收（``JUMP_REJECT_PX``）、死区（``DEADBAND_PX``）、指令限频
  （``COMMAND_INTERVAL_S``）、单帧限幅（``MAX_STEP_PULSES``）。

本模块**保持纯 Python（不 import maix）**，可在 PC 上离线仿真与回归。
控制律可整体替换：``controller(axis, error_px, now) -> 带符号脉冲数``，传 None 用内置比例律。
"""

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
        if self.state == FAULT:
            return self._emit(None, None, {"yaw": None, "pitch": None})
        valid = bool(result and result.get("aim_valid") and
                     result.get("dx") is not None and result.get("dy") is not None)
        if not valid:
            return self._on_lost(now)
        dx, dy = float(result["dx"]), float(result["dy"])
        self._lost = 0
        self._scan_pos = 0
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
        out = {"state": self.state, "dx": dx, "dy": dy, "cmds": cmds}
        self.last = out
        return out

    def _deadband(self):
        return float(getattr(self.cfg, "DEADBAND_PX", 2.0))

    def _on_lost(self, now):
        self._lost += 1
        self._settle = 0
        scan_on = bool(getattr(self.cfg, "ENABLE_SCAN", False))
        if scan_on and self._lost >= int(getattr(self.cfg, "LOST_FRAMES", 15)):
            self.state = SCAN
            cmd = self._scan_command("yaw", now)
            return self._emit(None, None, {"yaw": cmd, "pitch": None})
        self.state = HOLD
        return self._emit(None, None, {"yaw": None, "pitch": None})

    def _gain(self, axis, error):
        cfg = self.cfg
        fine = float(getattr(cfg, "GAIN_YAW" if axis == "yaw" else "GAIN_PITCH", 0.5))
        coarse_px = float(getattr(cfg, "COARSE_PX", 0.0))
        if coarse_px > 0 and abs(error) > coarse_px:
            return float(getattr(cfg, "GAIN_COARSE", min(1.0, fine * 1.8)))
        return fine

    def _rpm(self):
        if self.state == COARSE:
            return int(getattr(self.cfg, "RPM_COARSE", 1200))
        return int(getattr(self.cfg, "RPM_FINE", 300))

    def _axis_command(self, axis, error, now):
        cfg = self.cfg
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
            pulses = min(max_pulses, max(1, int(round(abs(error) * ppp * self._gain(axis, error)))))
            ccw = error * sign > 0
        self.last_command_time[axis] = now
        return ccw, pulses, self._rpm()

    def _scan_command(self, axis, now):
        cfg = self.cfg
        if now - self.last_command_time[axis] < float(getattr(cfg, "SCAN_STEP_INTERVAL_S", 0.15)):
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
        return ccw, step, int(getattr(cfg, "SCAN_RPM", 60))
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

    def __init__(self, cfg):
        self.cfg = cfg
        self._ports = {}
        self._rxbuf = {"yaw": bytearray(), "pitch": bytearray()}
        self._stats = {"tx": 0, "rx": 0, "ok": 0, "e2": 0, "err": 0,
                       "timeout": 0, "yaw": True, "pitch": True}
        self._last_status = {"yaw": None, "pitch": None}
        self._last_cmd = {"yaw": 0.0, "pitch": 0.0}
        self._last_ack = {"yaw": 0.0, "pitch": 0.0}
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
            if n is not None and int(n) < 0:
                raise IOError("write returned %s" % n)
            self._stats["tx"] += 1
            self._last_cmd[axis] = time.perf_counter() if now is None else float(now)
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
                if now - self._last_cmd[axis] <= timeout_ms / 1000.0:
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
        now = time.perf_counter() if now is None else float(now)
        return self._ingest(self._drain(), now)

    def stats(self):
        return dict(self._stats)
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
        self.started = float(now)
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
        if now < self.deadline:
            return self.state
        dv = None
        if result and result.get("aim_valid") and result.get("dx") is not None:
            dv = (float(result["dx"]), float(result["dy"]))
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
        travel = self.pulses / (ppr * rpm / 60.0)
        settle = float(getattr(self.cfg, "CALIBRATE_SETTLE_S", 1.0))
        return travel * 1.4 + settle

    def _send(self, ccw):
        ok = self.link.move(self.axis, ccw, self.pulses,
                            int(getattr(self.cfg, "CALIBRATE_RPM", 300)),
                            int(getattr(self.cfg, "ACC_DEFAULT", 0)))
        if not ok:
            self._fault("uart_write_failed")
        return bool(ok)

    def _fault(self, reason):
        self.state, self.reason = "FAULT", str(reason)
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
    u, v       float   靶心像素坐标（已按 CENTER_ALPHA 平滑，供显示/常规使用）
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
    return type("ApiConfig", (Config,), dict(overrides))


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
    if a.ndim != 2:
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
    res.setdefault("dx", None)
    res.setdefault("dy", None)
    res.setdefault("aim_valid", False)
    return res


# ------------------------------------------------------------- API ① 识别黑框
def detect_black_frame(frame, cfg=None, detector=None):
    """**无状态**识别黑框，返回 ``Result``（没识别到则 ``valid=False``，``bool(res)`` 为假）。

    每帧都是一次完整的"缩小图多阈值扫描 → 四边暗带验证 → 亚像素精修"，不依赖上一帧，
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
            return Result(valid=False, stage="empty_frame", center=None,
                          dx=None, dy=None, aim_valid=False, ms=0.0)
        raw = self.detector.process(gray)
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

    def __init__(self, cfg=None, enable_motors=True, with_laser=True):
        self.cfg = cfg if cfg is not None else load_config()
        self.tracker = CenterTracker(self.cfg, with_laser=with_laser)
        self.link = None
        self.aim = None
        if enable_motors:
            from vision.stepper_link import StepperLink
            from vision.gimbal_aim import GimbalAim
            self.link = StepperLink(self.cfg)
            self.link.enable()
            self.aim = GimbalAim(self.cfg)

    def step(self, frame, now=None):
        now = time.perf_counter() if now is None else float(now)
        res = self.tracker.track(frame, now)
        res["aim_state"] = None
        res["cmds"] = {"yaw": None, "pitch": None}
        if self.aim is None or self.link is None:
            return res
        if self.aim.state != "FAULT":
            out = self.aim.update(res, now=now)
            res["aim_state"] = out["state"]
            for axis, command in out["cmds"].items():
                if command is not None:
                    ccw, pulses, rpm = command
                    self.link.move(axis, ccw, pulses, rpm,
                                   getattr(self.cfg, "ACC_DEFAULT", 0), now=now)
                    res["cmds"][axis] = command
        fb = self.link.poll(now)
        if fb.get("fault"):
            self.link.stop(now=now)
            self.aim.fault("link:%s" % fb["reason"])
            res["aim_state"] = "FAULT"
        return res

    def stop(self):
        if self.link is not None:
            self.link.stop()

    def close(self):
        self.stop()
'''

_MAIN_SRC = r'''
"""MaixCAM2 entry point for perspective robust black rectangle center detection.

上电行为（v0.4.1 起）：
  使能 → 建显示/触摸 → 进入主循环（识别 + 光斑偏差 + 云台闭环）。
  **上电不再自动跑自检**；自检改为屏幕右上角的 `SELF-TEST` 按钮，点一下才触发
  （见 `Config.ENABLE_TOUCH_BUTTON`）。`CALIBRATE_AXIS` 非空时仍走标定（标定期不闭环）。

画面流转到 MaixVision / 板载屏幕：`_Preview` 把带标注的画面 letterbox 进"屏幕尺寸画布"，
再交给**独立显示线程**去 `display.Display().show()`（仍在主进程内 —— 这是 MaixVision 能看到
画面的成熟路径；PC 上无 maix 时自动跳过）。主循环只负责绘制与投递，**永不等待 show**。

================================================================================
★ 图像显示运行逻辑（自"取帧"到"上屏"的完整链路）—— 本文件读法总纲
================================================================================
本文件里跑着**三条独立的时间轴**。理解显示前必须先分清它们；把二者混在一起，就会把
"图传慢"误诊为"识别慢"（这正是本项目历史上踩过的坑，见 B 轨 README「v9→v17 追踪手感对照」）。

  时间轴 A —— 主循环（`main()`，主线程；决定采图/控制节奏）
    ① cam.read()                      从 MaixCAM2 相机取一帧（灰度 FMT_GRAYSCALE）
    ② _to_gray(img, image)            借相机缓冲 → 灰度 ndarray（copy=False，零拷贝）
    ③ touch.poll(preview)             触摸"按下沿"→ 置 controller.request_test（触发自检）
    ④ worker.submit(gray, captured)   把**当帧灰度图的副本**投进识别线程（不等待结果）
    ⑤ worker.snapshot(now)            取回**最近一次**识别结果（可能不是本帧，允许滞后）
    ⑥ _display_result(snapshot,...)   结果超过 RESULT_MAX_AGE_S 即降级为 invalid，不画旧框
    ⑦ preview.due(now)                显示节拍（SHOW_FPS）到了**且画布空闲**才画：
         ├─ draw_result()             画外/内框、四角、靶心十字（vision/rectangle_center.py）
         ├─ draw_laser_overlay()      画光斑标记 + 左上角面板 + 光斑→靶心连线（vision/laser_spot.py）
         └─ preview.render()          letterbox 到画布 → **投给显示线程，立刻返回**
    ⑧ 每 LOG_INTERVAL 打印 [perf]     分相耗时 + 三路帧率（口径分开，勿混读）

  时间轴 B —— 识别线程（`_LatestVision`，daemon 线程；决定识别吞吐）
    "最新帧槽位"取一帧 → controller.process()（识别 + 光斑偏差 + 云台闭环）→ 写回"最新结果槽位"
    → 空闲时走 controller.idle()（继续收 ACK、推进按钮自检）。槽位只保留**最新**一份：
    主循环产帧快于识别时，旧帧被丢弃并计入 worker.dropped（丢帧而非积压）。

  时间轴 C —— 显示线程（`_Preview._show_loop`，daemon 线程；决定屏幕刷新）
    取"待上屏画布"槽 → `display.show()`（屏幕 + MaixVision 图传）→ 画布还池。
    画布池固定 2 张（显示线程 1 张 + 主线程 1 张）；主循环要画时池空 → **直接跳过该帧**
    （丢帧，不排队、不等待）。所以 show() 再慢也只慢预览本身。

  为什么必须拆成三条轴：
    · display.show() 走屏幕驱动 / MaixVision 图传，单次可长达数百 ms；同线程时一慢全卡。
    · 识别（Canny / 轮廓 / 单应）耗时不确定；同线程时识别一慢屏幕就卡成幻灯片。
    每条轴各自取"最新"，任一侧变慢只表现为**丢帧 / 滞后**，不会互相拖垮（降级但不崩溃）。
    ★ 历史教训：只把"识别"拆出去是不够的 —— 只要 show() 还在主循环里同步调用，
      主循环就被它按在地上，`submit()` 的节奏随之被拖死，识别线程再快也无帧可算
      （表现为 capture / disp / detect_fps **一起**掉到同一个低位）。故 v0.4.3 把显示也拆出去。

  ★ 隔离原则（贯穿本文件）：显示层任何失败 —— 无 maix、Display 建失败、show 抛异常、
    无触摸屏 —— 一律只打日志并把 `preview.enabled` 置 False，**绝不影响识别、串口下发
    与云台闭环**。识别的正确性请以 [perf] 日志与 target_api 回归为准，不要拿屏幕观感判断。
================================================================================
"""

import os
import sys
import time
import threading
from collections import deque

import cv2
import numpy as np


def _root():
    try:
        return os.path.dirname(os.path.abspath(__file__))
    except NameError:
        return os.getcwd()


ROOT = _root()
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from config import Config
from vision.rectangle_center import RectangleCenterDetector, draw_result
from vision.laser_spot import LaserSpotDetector, deviation, draw_laser_overlay, PANEL_H
from vision.gimbal_aim import GimbalAim
from vision.gimbal_self_test import GimbalSelfTest, probe_imu
from vision.gimbal_calibrate import GimbalCalibrator

BUILD_ID = "2026-10-11-maixvision-off-v0.4.5"


class _FrameRate:
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


class _LatestVision:
    """单识别线程 + 两个"最新槽位"：一个待处理帧槽(_pending)、一个结果槽(_latest)。

    线程模型（时间轴 B 的实现，见模块开头总纲）：
        · 主线程（生产者）：submit() 往 _pending 放**最新一帧**的副本；
        · 识别线程（消费者）：取走 _pending → callback() 处理 → 结果写 _latest；
        · 主线程（读者）：snapshot() 无锁等待地取回 _latest（可能滞后于当前摄像头帧）。

    "最新优先、丢弃旧帧"是刻意的：识别跟不上时保留最新画面才有意义，积压旧帧只会让
    追踪越追越偏。Camera / Display / 触摸这些**必须独占主线程**的资源一概不跨线程。
    """

    def __init__(self, callback, idle=None, asynchronous=True):
        self.callback, self.idle = callback, idle
        self._condition = threading.Condition()   # 保护 _pending / _latest / _stop
        self._pending = self._latest = None
        self._stop = False
        self.error = None
        self.dropped = 0                          # 因来不及处理而被覆盖的帧数（丢帧计数）
        self.rate = _FrameRate()                  # 识别帧率（detect_fps 口径）
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
        snapshot = self.callback(*item)  # callback = _VisionControl.process（识别+偏差+闭环）
        with self._condition:
            self._latest = snapshot
            self.rate.record(time.perf_counter())

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
        with self._condition:
            fps, low1 = self.rate.values(now)
            return self._latest, fps, low1

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


class _Preview:
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

    def __init__(self, cfg):
        self.cfg = cfg
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
        self.rate = _FrameRate()                # 显示帧率（disp 口径，与识别帧率分开）
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
            self.display = display.Display()
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
            if self.async_show:
                self.thread = threading.Thread(target=self._show_loop, name="display", daemon=True)
                self.thread.start()
        except Exception as exc:
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
        self._button_draw.append((bx0, by0, bx1, by1, label, pos))

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
        self._draw_buttons(canvas, status)                   # 按钮与状态条画在画布之上
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
            t0 = time.perf_counter()
            self.display.show(maix_image.cv2image(frame, bgr=True, copy=True))
            ms = (time.perf_counter() - t0) * 1000.0
        except Exception as exc:
            print("[preview] show 探针失败（显示可能不可用）: %s" % exc)
            return -1.0
        if ms >= 300.0:
            print("[preview] ⚠ show 探针 %.0fms —— 命中 MaixCDK「MaixVision 图传未就绪」忙等分支："
                  "每次 show() 空转 500ms（maix_image_trans.cpp），约每秒烧掉半个 CPU 核，"
                  "是相机 read / 识别 / 闭环一起变慢的根因。" % ms)
            print("[preview] ⚠ 对策①：把 Config.DISABLE_MAIXVISION_STREAM 设回 True（默认即 True）——"
                  "main() 会在建 Display 之前把 MAIXVISION 置 0，从源头不建图传，show() 回到 1~5ms。")
            print("[preview] ⚠ 对策②：或改从**设备本地**启动本应用（应用菜单 / 开机自启），"
                  "不要用 MaixVision 的“运行”按钮 —— 本地启动时 maixvision 模式本就关闭。")
            print("[preview] ⚠ 对策③：或让 MaixVision 的画面预览真正连上"
                  "（连上后图传走「压 JPEG 入队」的异步路径，不再忙等）。")
        else:
            print("[preview] show 探针 %.1fms —— 显示链路健康（本地屏典型 1~5ms）" % ms)
        return ms

    def warn_slow_once(self, p95):
        """显示侧变慢只提示一次（每 2s 窗口重复报会刷屏）；点明最可能的根因。"""
        if self._slow_warned:
            return
        self._slow_warned = True
        print("[preview] show slow: p95=%.0fms —— 瓶颈在显示侧（非识别）。"
              "若该值恒定 ≈505ms，即为 MaixCDK 的 MaixVision 图传忙等："
              "保持 Config.DISABLE_MAIXVISION_STREAM=True（默认），或从设备本地启动应用即可消除。"
              "预览掉帧，识别/闭环不受影响。" % p95)

    def _do_show(self, canvas, maix_image=None):
        """真正调用 display.show()，记耗时/帧率；失败即停用显示（不影响识别/闭环）。返回是否成功。"""
        if maix_image is None:
            from maix import image as maix_image
        t0 = time.perf_counter()
        try:
            # bgr=True：cv2 是 BGR 序；copy=True：交给显示的必须是独立缓冲。
            self.display.show(maix_image.cv2image(canvas, bgr=True, copy=True))
        except Exception as exc:
            self.enabled = False
            print("[preview] show 失败，停用显示: %s" % exc)
            return False
        ms = (time.perf_counter() - t0) * 1000.0
        self.shown += 1
        completed = time.perf_counter()
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
        for (x0, y0, x1, y1, label, pos) in self._button_draw:
            cv2.rectangle(canvas, (x0, y0), (x1, y1), (46, 150, 60), -1)   # 填充
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
        if self.display is not None:
            try:
                self.display.close()
            except Exception:
                pass


class _Touch:
    """触摸屏输入：把点击映射到 `_Preview` 的按钮（**按下沿**触发，防连点重复启动）。

    `maix.touchscreen.TouchScreen().read()` 立即返回 `(x, y, pressed)`（MaixPy v4）。
    无触摸屏 / 未启用 / 无显示时自动禁用，只打日志。
    坐标口径：read() 返回的是**屏幕坐标**，与按钮注册坐标一致（画布=屏幕尺寸的前提）。
    """

    def __init__(self, cfg):
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
        except Exception:
            return None
        if pressed:
            if not self._down:             # 只有"上一帧未按、这一帧按下"才算一次点击
                self._down = True
                return preview.hit(int(x), int(y))
            return None                    # 按住不放：不重复返回，避免反复触发
        self._down = False                 # 松开，允许下一次按下沿
        return None


def _to_gray(img, image_module):
    """MaixPy 相机帧 → cv2 灰度 ndarray（H×W，uint8）。

    优先走 `image2cv(..., copy=False)` **零拷贝**借用相机缓冲（不额外占内存、最快），
    因此返回的数组在下一次 cam.read() 前必须用完；跨线程传递前务必先 copy（见 _LatestVision.submit）。
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


def _send_uart(uart, result):
    """旧版纯视觉 ASCII 输出（ENABLE_GIMBAL=False 的降级路径才用；开云台时 uart 为 None）。"""
    if uart is None:
        return
    if result.get("valid"):
        payload = "@[%d,%d]%%\r\n" % (int(round(result["u"])), int(round(result["v"])))
    else:
        payload = "@[invalid]%%\r\n"
    uart.write(payload.encode("ascii"))


class _VisionControl:
    """识别 + 电机状态由**识别线程独占**；主循环只提交图像、按钮事件、读结果。

    独占理由：UART 读写不可并发（两条轴各有独立串口，但同一轴的收发必须串行），
    把"识别 → 偏差 → 闭环下发 → 收 ACK"全放进识别线程，天然避免与主线程抢串口。
    主线程要触发自检只置 `request_test` 事件，由识别线程在自己的上下文里安全地启动。
    """

    def __init__(self, cfg, link, aim, calibrator, uart):
        self.cfg, self.link, self.aim = cfg, link, aim
        self.calibrator, self.uart = calibrator, uart
        self.self_test = None
        self.request_test = threading.Event()   # 主线程置位、识别线程消费（按钮 → 自检）
        self.detector = RectangleCenterDetector(cfg)
        self.laser = LaserSpotDetector(cfg) if cfg.ENABLE_LASER else None
        self.laser_static = self.laser is not None and self.laser.mode == "fixed"
        # fixed 模式：光斑位置是常量，开机取一次即可，之后每帧只做减法（零检测开销）。
        self.laser_state = self.laser.static_state() if self.laser_static else None
        self.last_capture = -1e9
        self.stale_stopped = True

    def process(self, gray, captured_at):
        """识别线程的单帧主流程：识别 → 光斑 → 偏差 → 串口 → 闭环 → 打包给显示层。"""
        started = time.perf_counter()
        result = self.detector.process(gray)                # ① 黑框识别
        detected = time.perf_counter()
        if self.laser is not None and not self.laser_static:
            self.laser_state = self.laser.process(gray, detected)   # ② detect 模式才实测光斑
        dv = deviation(self.laser_state, result, use_raw=self.cfg.AIM_USE_RAW) if self.laser_state else None
        if dv is not None:
            result["laser_u"], result["laser_v"] = self.laser_state["u"], self.laser_state["v"]
            result["dx"], result["dy"] = dv                 # dx=靶心−光斑（闭环用的那个偏差）
        now = time.perf_counter()
        fresh = now - captured_at <= self.cfg.RESULT_MAX_AGE_S   # 结果是否还"新鲜"
        result["aim_valid"] = bool(dv is not None and fresh)
        self.last_capture = captured_at
        # 过期结果不外发：串口与闭环都用降级后的 output，避免拿旧定位驱动电机。
        output = result if fresh else dict(valid=False, aim_valid=False, stage="stale")
        _send_uart(self.uart, output)
        self._control(output, now, new_measurement=True)
        # 返回给显示层的快照：识别结果 + 光斑 + 计时 + 状态串。
        return dict(result=result, laser=dict(self.laser_state) if self.laser_state else None,
                    captured_at=captured_at, completed_at=time.perf_counter(),
                    detect_ms=(detected - started) * 1000, status=self.status())

    def idle(self):
        # 摄像头/图传停顿时继续收 ACK 与推进按钮自检，禁止重复发送旧视觉误差。
        self._control(None, time.perf_counter(), new_measurement=False)

    def status(self):
        """给屏幕左下角显示的状态串：自检 > 标定 > 闭环状态机的优先级。"""
        if self.self_test is not None and not self.self_test.done:
            return "self-test %d/%d" % (self.self_test.index, len(self.self_test.sequence))
        if self.calibrator is not None and not self.calibrator.done:
            return "calibrate %s" % self.calibrator.axis
        return self.aim.state if self.aim is not None else "vision"

    def _control(self, result, now, new_measurement):
        """闭环调度中枢：故障检查 → 按钮自检 → 标定 → 自检 → 失锁停机 → 追踪下发。

        优先级顺序**不可随意调换**（见 B 轨 AGENTS.md §3.2）；尤其 FAULT 必须在最前，
        因为链路故障要阻断的不只是追踪，还包括已启动的自检/标定。
        """
        link, aim = self.link, self.aim
        if link is None or aim is None:
            return
        feedback = link.poll(now)                            # 非阻塞收两条串口的返回帧
        if feedback and bool(feedback.get("fault")) and aim.state != "FAULT":
            reason = feedback.get("reason")
            link.stop(now=now)
            aim.fault("link:%s" % reason)                    # 链路故障 → 进入 FAULT
        if self.request_test.is_set():                       # 屏幕按钮置位的自检请求
            self.request_test.clear()
            if self.self_test is not None and not self.self_test.done:
                print("[ui] 自检进行中，忽略重复点击")
            elif aim.state == "FAULT":
                print("[ui] 链路 FAULT，拒绝自检")
            elif self.calibrator is not None and not self.calibrator.done:
                print("[ui] 标定进行中，拒绝自检")
            elif not self.cfg.GIMBAL_SELF_TEST:
                print("[ui] GIMBAL_SELF_TEST=False，忽略自检")
            else:
                link.stop(now=now)                           # 先停当前动作再起自检
                self.self_test = GimbalSelfTest(self.cfg, link, now)
                print("[ui] 收到点击 -> 启动上电自检（%d 步）" % len(self.self_test.sequence))
        # 链路故障必须阻断已启动的自检/标定，不只阻断普通追踪。
        if aim.state == "FAULT":
            return
        if self.calibrator is not None and not self.calibrator.done:
            self.calibrator.update(now, result or {"valid": False, "aim_valid": False})
            return
        if self.self_test is not None and not self.self_test.done:
            self.self_test.update(now)
            return
        if not new_measurement:
            # idle 路径：没有新视觉误差。若已超时且尚未停机，则停一次并置 aim 无效（防"漂"）。
            if now - self.last_capture > self.cfg.RESULT_MAX_AGE_S and not self.stale_stopped:
                link.stop(now=now)
                aim.update({"aim_valid": False}, now=now)
                self.stale_stopped = True
            return
        if not result.get("aim_valid"):
            if not self.stale_stopped:                       # 视觉失效：停一次电机，不回零
                link.stop(now=now)
            self.stale_stopped = True
        else:
            self.stale_stopped = False
        commands = aim.update(result, now=now)["cmds"]       # 状态机算出的两轴增量指令
        for axis, command in commands.items():
            if command is not None:
                ccw, pulses, rpm = command
                link.move(axis, ccw, pulses, rpm, self.cfg.ACC_DEFAULT, now=now)


def _display_result(snapshot, now, max_age):
    """把"识别线程的最新快照"翻译成"这一帧显示该画什么"。

    过期定位不能被画成当前帧的有效定位（否则屏幕上框会"粘"在旧位置，误以为还锁着）；
    但视频原帧继续刷新，画面不因此冻住。返回 (result, laser, age)：
        age > max_age → 用 {valid:False, stage:"stale"} 占位，只保留 laser（面板照常显示）
    """
    if snapshot is None:
        return {"valid": False, "stage": "waiting"}, None, 0.0
    age = max(0.0, now - snapshot["captured_at"])
    if age > max_age:
        return {"valid": False, "stage": "stale"}, snapshot["laser"], age
    return snapshot["result"], snapshot["laser"], age


def _apply_maixvision_stream_policy(cfg):
    """按配置决定是否禁用 MaixVision 图传；**必须在创建 display.Display() 之前**调用。

    为什么是环境变量而不是别的：MaixCDK 的 `maixvision_mode()` 只读环境变量 MAIXVISION
    （== "1" 才算真），而图传通道 `img_trans` 只在 `Display` 构造时按它创建
    （`maix_display.cpp`：`if(!img_trans && maixvision_mode()) img_trans = new ImageTrans(...)`）。
    一旦建了 img_trans，`show()` 与 `send_to_maixvision()` 都会走 `ImageTrans::send_image()`
    —— 在「MaixVision 模式但连接未就绪」时忙等 500ms 空转 CPU（`maix_image_trans.cpp`）。
    所以想在代码层拆掉它，只有一条路：**在建 Display 之前让 maixvision_mode() 为假**。

    ⚠ 两个易踩点：
      · 必须早于 `display.Display()`；建好之后再改环境变量无效（img_trans 已存在）。
      · `send_to_maixvision()` 并不能绕开忙等 —— 它与 show() 共用同一个 send_image()。
    返回是否实际改动了环境变量。
    """
    if not bool(getattr(cfg, "DISABLE_MAIXVISION_STREAM", True)):
        return False
    if os.environ.get("MAIXVISION") != "1":
        return False
    os.environ["MAIXVISION"] = "0"      # os.environ 赋值会同步 putenv，C 侧 getenv 随即生效
    print("[preview] 已禁用 MaixVision 图传（MAIXVISION 1->0）：本地屏恢复 1~5ms ⇒ 全帧；"
          "MaixVision 画面预览将无图（终端日志照常）。"
          "如需图传：Config.DISABLE_MAIXVISION_STREAM=False，并确保预览已真正连上。")
    return True


def main():
    # ★ 先把 MaixVision 图传按配置掐断（若处于 MaixVision 模式）——必须早于建 Display，见上。
    _apply_maixvision_stream_policy(Config)

    # Imports stay inside main so the geometry module remains PC-testable.
    from maix import app, camera, image

    # 小图上的多线程调度常常比运算本身更贵，也避免 ARM 多核线程争用。
    cv2.setNumThreads(Config.OPENCV_THREADS)

    cam = camera.Camera(Config.CAMERA_W, Config.CAMERA_H,
                        image.Format.FMT_GRAYSCALE,
                        fps=Config.CAMERA_FPS,
                        buff_num=Config.CAMERA_BUFFERS)
    cam.skip_frames(Config.SKIP_FRAMES)
    uart = None
    if Config.ENABLE_UART:
        try:
            from maix import uart as maix_uart
            uart = maix_uart.UART(Config.UART_PORT, Config.UART_BAUD)
        except Exception as exc:
            print("[uart] open failed: %s" % exc)
    link = None
    aim = None
    self_test = None          # ★ 上电不再自动创建；由屏幕按钮触发时才建
    calibrator = None
    if Config.ENABLE_GIMBAL:
        try:
            from vision.stepper_link import StepperLink
            link = StepperLink(Config)
            link.enable()
            aim = GimbalAim(Config)
            imu_info = probe_imu()
            if imu_info["available"]:
                print("[imu] sample via %s: %s (gyro cannot define absolute yaw zero)" %
                      (imu_info["method"], str(imu_info["sample"])))
            else:
                print("[imu] unavailable: %s" % imu_info["error"])
            calibrator = GimbalCalibrator(Config, link, time.perf_counter())
            if calibrator.done:
                # 未配置标定轴：正常进入（按钮式）自检 + 跟踪。
                calibrator = None
            else:
                print("[gimbal] calibration mode axis=%s（本次只标定，不闭环）" % calibrator.axis)
            print("[gimbal] enabled yaw=%s(ID%d) pitch=%s(ID%d) acc=%d" %
                  (Config.YAW_DEV, Config.ADDR_YAW, Config.PITCH_DEV, Config.ADDR_PITCH,
                   Config.ACC_DEFAULT))
        except Exception as exc:
            print("[gimbal] disabled after init failure: %s" % exc)

    # ---- 显示（把画面流转到 MaixVision）+ 屏幕按钮 ----
    # 按钮几何按**屏幕坐标**定义：右上角一块 30%宽/20%高 的区域（有下限，保证可点）。
    # 只有"有显示 且 启用触摸按钮"时才注册按钮并建触摸；否则各自打印降级原因。
    preview = _Preview(Config)
    touch = None
    if preview.enabled and bool(getattr(Config, "ENABLE_TOUCH_BUTTON", True)):
        bw = max(70, int(preview.width * 0.30))
        bh = max(34, int(preview.height * 0.20))
        preview.add_button("selftest", preview.width - bw - 8, 8, preview.width - 8, 8 + bh)
        touch = _Touch(Config)
    elif preview.enabled:
        print("[touch] ENABLE_TOUCH_BUTTON=False -> 不画按钮")
    elif bool(getattr(Config, "ENABLE_TOUCH_BUTTON", True)):
        print("[touch] 无显示 -> 屏幕按钮不可用（自检需有显示才能点击触发；SHOW=True 才有）")

    # ---- 线程装配：识别线程（时间轴 B）+ 主循环（时间轴 A）----
    controller = _VisionControl(Config, link, aim, calibrator, uart)
    worker = _LatestVision(controller.process, controller.idle, Config.VISION_ASYNC)
    capture_rate = _FrameRate()             # 取帧帧率（capture 口径）
    last_log = time.perf_counter()
    read_times, draw_times, loop_times = [], [], []   # 本日志窗口内的分相耗时样本（ms）
    print("[rect-center] %s search_width=%d display_width=%d display_fps=%d" %
          (BUILD_ID, Config.SEARCH_WIDTH, preview.render_width, Config.SHOW_FPS))
    print("[preview] async=%s 画布=%dx%d（SHOW_WIDTH=%d；0=用屏幕原始宽度）"
          % (getattr(Config, "SHOW_ASYNC", True), preview.render_width, preview.render_height,
             int(getattr(Config, "SHOW_WIDTH", 0) or 0)))
    print("[vision] latest-frame worker=%s max_result_age=%.0fms" %
          (Config.VISION_ASYNC, Config.RESULT_MAX_AGE_S * 1000))
    print("[ui] 上电自检已改为屏幕按钮：点显示右上角 SELF-TEST 触发（不再上电自动跑）")
    try:
        # ============================ 时间轴 A：主循环 ============================
        # 每轮 = 取帧 → 提交识别 → 取最新结果 → 按节拍绘制并投递上屏 → 周期打日志。
        # 全程不等待识别结果，**也不等待 show**（真正上屏在显示线程），二者慢只表现为
        # "标注滞后 / 预览掉帧"，都不拖慢循环本身。
        while not app.need_exit():
            started = time.perf_counter()
            img = cam.read()                             # ① 取帧（灰度）
            if img is None:
                continue
            captured = time.perf_counter()
            read_times.append((captured - started) * 1000)   # read 耗时样本
            gray = _to_gray(img, image)                  # ② 零拷贝转灰度 ndarray
            capture_rate.record(captured)
            if touch is not None and touch.poll(preview) == "selftest":
                controller.request_test.set()            # ③ 按键沿 → 通知识别线程起自检
            worker.submit(gray, captured)                # ④ 提交当帧副本给识别线程
            now = time.perf_counter()
            # ⑤ 取最新识别结果（每轮只取一次：日志窗口内不再重复取，省一次加锁 + 帧率统计）
            snapshot, vision_fps, vision_low1 = worker.snapshot(now)
            if worker.error is not None:
                raise RuntimeError("vision worker failed: " + worker.error)   # 识别线程挂了 → 终止
            result, laser_state, age = _display_result(snapshot, now, Config.RESULT_MAX_AGE_S)  # ⑥ 过期降级
            if preview.due(now):                         # ⑦ 节拍到 + 画布空闲 → 绘制并投递上屏
                t_draw = time.perf_counter()
                # 画进 BGR 视图；render_width == 原图宽时 draw_result 只做灰度→BGR，不缩放
                view = draw_result(gray, result, preview.render_width,
                                   top_offset=PANEL_H if laser_state is not None else 0)
                if laser_state is not None:
                    # 激光叠加：scale = 画布/原图宽，使光斑标记落在与画面对应的位置
                    draw_laser_overlay(view, laser_state, result,
                                       min(1.0, preview.render_width / float(gray.shape[1])),
                                       use_raw=Config.AIM_USE_RAW)
                # 画面左下角一行小字：disp=显示帧率 low1=最慢1% det=识别帧率 age=结果龄
                cv2.putText(view, "disp %.1f low1 %.1f det %.1f age %.0fms" %
                            (preview.fps, preview.low1, vision_fps, age * 1000),
                            (8, view.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX,
                            0.40, (0, 255, 255), 1)
                preview.render(view, now, status=snapshot["status"] if snapshot else "waiting")
                draw_times.append((time.perf_counter() - t_draw) * 1000.0)   # 绘制+投递（不含 show）
            loop_times.append((time.perf_counter() - started) * 1000.0)
            now = time.perf_counter()
            if now - last_log >= Config.LOG_INTERVAL:    # ⑧ 周期日志：分相耗时 + 三路帧率
                capture_fps, _ = capture_rate.values(now)
                display_fps, display_low1 = preview.rate.values(now)
                show_avg, show_p95, show_max, show_n = preview.take_show_stats()
                read_avg = float(np.mean(read_times)) if read_times else 0.0
                draw_avg = float(np.mean(draw_times)) if draw_times else 0.0
                loop_avg = float(np.mean(loop_times)) if loop_times else 0.0
                detect_ms = snapshot["detect_ms"] if snapshot else 0.0
                # ★ 读法：[perf] 里 capture / disp / detect_fps 是三路**不同**口径，
                #   "眼睛看到的刷新率"看 disp，"识别吞吐"看 detect_fps，别互相换算。
                #   read/draw/loop 属于**主循环**，show 属于**显示线程** —— 分开看就能一眼判定
                #   "到底谁在吃时间"，不必再靠猜（这正是 1.5fps 归因的关键）。
                print("[perf] %s capture=%.1f disp=%.1f low1=%.1f detect_fps=%.1f "
                      "valid=%s stage=%s | read=%.1f draw=%.1f loop=%.1f ms | "
                      "show avg=%.1f p95=%.1f max=%.1fms n=%d | "
                      "age=%.0fms shown=%d drop(show)=%d drop(vision)=%d" %
                      (BUILD_ID, capture_fps, display_fps, display_low1, vision_fps,
                       result.get("valid", False), result.get("stage"),
                       read_avg, draw_avg, loop_avg,
                       show_avg, show_p95, show_max, show_n,
                       age * 1000, preview.shown, preview.dropped, worker.dropped))
                if aim is not None:
                    print(aim.status_line())
                if show_p95 > 100:
                    # show 的 p95 超过 100ms ⇒ 瓶颈在**显示侧**（屏幕驱动 / MaixVision 图传），
                    # 不是识别慢。预览会掉帧，但识别与闭环不受影响 —— 判据与对策见 README。
                    preview.warn_slow_once(show_p95)
                if loop_avg > 66.0:
                    # 主循环单轮超过 66ms ⇒ 已在 15fps 以下，且此数**不含 show**。两个常见来源：
                    # ① 相机 read 本身偏大；② 显示线程在 MaixVision 图传忙等里空转 CPU，
                    # 把整体算力抢走（此时 read 也会被动变大）。
                    print("[preview] main loop slow: loop=%.0fms（不含 show）；看 read= 是否偏大；"
                          "若 show 也恒定 ~505ms 则是图传忙等抢 CPU" % loop_avg)
                last_log = now
                read_times.clear()
                draw_times.clear()
                loop_times.clear()
    finally:
        # worker 独占 UART；停止并 join 后主线程才能下发退出停机命令。
        try:
            worker.close()
        finally:
            # 工作线程退出后再清理硬件，避免与尚未退出的线程并发操作 UART。
            if worker.thread is None or not worker.thread.is_alive():
                if link is not None:
                    try:
                        link.stop()
                    except Exception as exc:
                        print("[gimbal] stop on exit failed: %s" % exc)
            preview.close()
            try:
                cam.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
'''

_MOD_ORDER = ('config', 'vision.rectangle_center', 'vision.laser_spot', 'vision.gimbal_self_test', 'vision.gimbal_aim', 'vision.stepper_link', 'vision.gimbal_calibrate', 'target_api')

_REL_OF = {'config': 'config.py', 'vision.rectangle_center': 'vision/rectangle_center.py', 'vision.laser_spot': 'vision/laser_spot.py', 'vision.gimbal_self_test': 'vision/gimbal_self_test.py', 'vision.gimbal_aim': 'vision/gimbal_aim.py', 'vision.stepper_link': 'vision/stepper_link.py', 'vision.gimbal_calibrate': 'vision/gimbal_calibrate.py', 'target_api': 'target_api.py'}

_pkg = _types.ModuleType("vision")          # 造出虚拟包 vision
_pkg.__path__ = []
_sys.modules["vision"] = _pkg
for _full in _MOD_ORDER:
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
    exec(compile(_MOD_SRC[_full], _full.replace(".", "/") + ".py", "exec"),
         _m.__dict__)
del _m, _full, _MOD_SRC

# --------------------------------------------------------------------------
# ② 主程序：main.py 原文照搬，exec 进本文件全局命名空间
#    这样 __name__ == "__main__" 判断、traceback 文件名都与原来一致
# --------------------------------------------------------------------------
exec(compile(_MAIN_SRC, 'main.py', "exec"), globals())
