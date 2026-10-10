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

生成时间：2026-10-10 23:14:52
    config.py                    sha1=084360a01ed2
    main.py                      sha1=3082f43b61a9
    target_api.py                sha1=e334a5b938ac
    vision/gimbal_aim.py         sha1=9231e9b98ce5
    vision/gimbal_calibrate.py   sha1=819bd3980592
    vision/gimbal_self_test.py   sha1=b21e1ea05d03
    vision/laser_spot.py         sha1=d8d722904443
    vision/rectangle_center.py   sha1=f779e749d2d8
    vision/stepper_link.py       sha1=c41fd48b7a2f
"""

import sys as _sys
import types as _types

_SRC_SHA1 = {'config.py': '084360a01ed2591a1997330abe61f3f41074bef5', 'vision/rectangle_center.py': 'f779e749d2d853f982429c0a88a92b551bcbb46a', 'vision/laser_spot.py': 'd8d722904443201c539507baca6d69be668ade93', 'vision/gimbal_self_test.py': 'b21e1ea05d03c7cf760304a3b72d26ee4dae92a2', 'vision/gimbal_aim.py': '9231e9b98ce54a4015f7610256050a56cfb9a948', 'vision/stepper_link.py': 'c41fd48b7a2f45be83d8557f61ed35380ca3e006', 'vision/gimbal_calibrate.py': '819bd3980592796289004288d6e95fee5e9aa92f', 'target_api.py': 'e334a5b938ac1315d2111c16b90dd30467fc11f6', 'main.py': '3082f43b61a98e005684c245b788b8df16fb3a49'}

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
    # ★★ 镜头畸变校正（`img.lens_corr(strength=…)`，**软件重映射**）★★
    #   它是把整幅画面按径向重新采样 —— 既改检测用的图，也改 MaixVision 里看到的画面。
    #   ⚠ 2026-10-10 由 True(1.5) 改为 **False**，原因有三：
    #     ① `strength` 必须**按实际镜头**实测（官方口径："调 strength 直到画面不再畸变"）；
    #        直接写 1.5 在非鱼眼镜头上属于**过校正** → 表现为画面外凸/边缘拉伸（现场反馈的"画面畸变"）。
    #     ② 官方 `lens_corr(strength, zoom)` 的 `zoom` 默认 1.0，强校正会把**边角拉出黑边/拉伸**
    #        （官方示例在 strength 2.1 时要用 zoom=0.9 规避）—— 本项目从未调过 zoom。
    #     ③ **离线回归基线（29/30、0.803px、0 误检）是在"无校正"的合成图上测的**，
    #        板上开校正等于让实际管线偏离已验证配置（测试无法覆盖这一层）。
    #   何时才开：画面里**靶纸长边明显弯曲**（鱼眼/广角边缘）时，改 True 并把 `LENS_STRENGTH`
    #   从 0.6~0.8 起逐步调大，直到直线变直、且不再外凸；必要时再引入 zoom 参数。
    #   A 轨同样默认关（`LENS_CORR = False`，注释"边缘直线弯时再开"）。
    LENS_CORR = False
    LENS_STRENGTH = 1.5
    SHOW = True
    # ★★ 显示通道选择（决定 MaixVision 能不能看到画面）★★
    #   False（默认）= **主进程内**直接 `display.Display().show()` —— 这是 v9/v15 的成熟路径，
    #                  **MaixVision 一定能看到摄像头画面**；按 SHOW_FPS 限频，且图传变慢时
    #                  节拍会**自动降速**（最低 SHOW_SLOW_MIN_FPS），不会把主循环拖死。
    #   True          = 独立**子进程**显示（`AsyncPreview`，v16 引入）：主循环绝不被 show 阻塞，
    #                  **但依赖子进程能成功建 `Display` 并把帧推给 MaixVision**。MaixVision 下
    #                  由子进程建 Display 属于未验证用法：一旦子进程起不来 / 建 Display 失败，
    #                  `ready` 永远不置位 → `can_submit()` 恒 False → **主循环一帧都不画** →
    #                  预览面板永远"无数据"。**现场若看不到画面，先把这里设回 False。**
    SHOW_ASYNC = False
    # 检测不跳帧；显示独立限频，减少 RGB 转换和 MaixVision 图像传输开销。
    SHOW_WIDTH = 320
    SHOW_FPS = 10
    # 同步显示在"图传变慢"时的自动最低帧率：show 越慢，节拍越慢（最多降到这个值），
    # 保证主循环不被 0.5s 级的 show 拖死，同时画面仍然会更新。
    # ★ 2026-10-10 由 1.0 提到 2.0：真机 MaixVision 图传常 >80ms，下限 1.0 时画面会**长期停在
    #   1fps**（现场"帧率过低"的主因之一）。2.0 下 show≈150ms 时显示仅占主循环约 30%，仍安全。
    SHOW_SLOW_MIN_FPS = 2.0
    # Display.show 含 MaixVision 图传，网络超时可能阻塞 500ms。
    SHOW_SLOW_MS = 80.0
    # ★ 慢帧的额外退避（秒）：仅 `SHOW_ASYNC=True` 的子进程路径使用（只补一点点）。
    #   旧值 2.0s 会让"图传一慢就睡 2 秒"，预览直接塌到 ~0.4fps（现场"画面极其卡顿"的来源之一）。
    SHOW_BACKOFF_S = 0.10
    # ★ 预览看门狗：累计慢帧（show > SHOW_SLOW_MS）达到该值时，在 [perf] 旁打印一次
    #   醒目提示，说明"画面卡顿来自 MaixVision 图传链路，不是算法"，并建议 SHOW=False。
    #   0 = 不提示。
    SHOW_STALL_LIMIT = 30
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
    ACC_TRACK = 160        # 缩短 v15 过慢的曲线，仍保留非零加减速
    ACC_DEFAULT = 0        # 仅其它机会性发帧用；闭环追踪请用 ACC_TRACK
    # ⑤ 死区与单帧限幅：死区应 ≥1.5×实测回差（齿隙里来回抖）；限幅防单帧猛甩。
    DEADBAND_PX = 3.0
    MAX_STEP_PULSES = 80
    COMMAND_INTERVAL_S = 0.06
    # 对最终运动量滤波：增大时缓慢跟随，减小/进入死区/换向时立即收缩，避免低通拖尾。
    # 时间常数越大越柔（0 = 关闭低通）；同时限制每秒运动量增长，0 = 关闭增长限速。
    AIM_OUTPUT_LPF_TAU_S = 0.10
    AIM_OUTPUT_SLEW_PPS = 400.0
    AIM_START_PULSES = 16
    # 下一条相对位置命令等上一段曲线运动的估算时间 + 余量，避免覆盖尚未完成的命令。
    # 这是时间估算，不是驱动器到位反馈；机构慢时可增大余量。
    AIM_WAIT_MOTION = True
    AIM_MOTION_MARGIN_S = 0.005
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
    SEARCH_BUDGET_MS = 12.0
    SEARCH_MAX_PROPOSALS = 2
    TRACK_RETRY_FRAMES = 2
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
"""
import time

import cv2
import numpy as np


def _polygon_area(p):
    return float(cv2.contourArea(np.asarray(p, np.float32).reshape(-1, 1, 2), oriented=True))


def _order_quad(points):
    p = np.asarray(points, np.float32).reshape(4, 2)
    c = p.mean(axis=0)
    p = p[np.argsort(np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0]))]
    if _polygon_area(p) < 0:
        p = p[::-1]
    return np.roll(p, -int(np.argmin(p.sum(axis=1))), axis=0).copy()


def _side_lengths(q):
    return np.linalg.norm(np.roll(q, -1, axis=0) - q, axis=1)


def _gaussian_solve(a, b):
    """显式 8×8 方程交给 OpenCV 原生 LU，避免 ARM 上逐行 Python 消元。"""
    ok, solution = cv2.solve(np.asarray(a, np.float64),
                             np.asarray(b, np.float64).reshape(-1, 1), flags=cv2.DECOMP_LU)
    if not ok:
        raise ValueError("degenerate quadrilateral")
    return solution[:, 0]


def _homography(src, dst):
    """显式构建 8×8 方程；先归一化坐标以提高大像素坐标下的稳定性。"""
    src, dst = np.asarray(src, np.float64), np.asarray(dst, np.float64)
    sc, dc = src.mean(axis=0), dst.mean(axis=0)
    ss, ds = float(np.max(np.abs(src - sc))), float(np.max(np.abs(dst - dc)))
    if min(ss, ds) < 1e-8:
        raise ValueError("zero-size rectangle")
    xy, uv = (src - sc) / ss, (dst - dc) / ds
    a = np.zeros((8, 8), np.float64)
    a[0::2, :3] = np.column_stack((xy, np.ones(4)))
    a[1::2, 3:6] = a[0::2, :3]
    a[0::2, 6:] = -uv[:, :1] * xy
    a[1::2, 6:] = -uv[:, 1:] * xy
    v = _gaussian_solve(a, uv.reshape(-1))
    hn = np.append(v, 1.0).reshape(3, 3)
    ts = np.array([[1 / ss, 0, -sc[0] / ss], [0, 1 / ss, -sc[1] / ss], [0, 0, 1]])
    td_inv = np.array([[ds, 0, dc[0]], [0, ds, dc[1]], [0, 0, 1]])
    h = td_inv @ hn @ ts
    return h / h[2, 2] if abs(h[2, 2]) > 1e-12 else h


def homography_8x8(src_quad, width_mm, height_mm):
    return _homography(src_quad, [[0, 0], [width_mm, 0],
                                 [width_mm, height_mm], [0, height_mm]])


def project_points(matrix, points):
    p = np.asarray(points, np.float64).reshape(-1, 2)
    z = np.column_stack((p, np.ones(len(p)))) @ np.asarray(matrix).T
    if np.any(np.abs(z[:, 2]) < 1e-9):
        raise ValueError("point at projective infinity")
    return z[:, :2] / z[:, 2:]


def inverse_map_points(h, points):
    return project_points(np.linalg.inv(h), points)


def standard_circle(width_mm, height_mm, radius_mm, samples=72):
    a = np.arange(samples) * (2 * np.pi / samples)
    return np.array([width_mm / 2, height_mm / 2]) + radius_mm * np.column_stack((np.cos(a), np.sin(a)))


def _rectangle(w, h, inset=0.0):
    return np.array([[inset, inset], [w - inset, inset],
                     [w - inset, h - inset], [inset, h - inset]], np.float64)


def _sample(gray, points):
    """一次 OpenCV remap 双线性采样整批点，越界由调用方拒绝。"""
    shape = points.shape[:-1]
    p = points.reshape(-1, 2).astype(np.float32)
    values = cv2.remap(gray, p[:, 0].reshape(1, -1), p[:, 1].reshape(1, -1),
                       cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    return values.reshape(shape)


class RectangleCenterDetector:
    def __init__(self, config):
        self.cfg = config
        self.last = None
        self._smooth = None
        self._track = None
        self._locked_th = None
        self._frame = 0
        self._kernel = np.ones((3, 3), np.uint8)
        self._t = np.linspace(0.12, 0.88, config.EDGE_SAMPLES, dtype=np.float32)
        self._circle_cache = {}
        self._search_count = 0
        self._threshold_count = 0
        self._search_work = None
        self._search_roi = None
        self._misses = 0
        self._proposals = 0
        self._profiles = {}

    def _valid_quad(self, q, shape, margin=1):
        h, w = shape
        if (q[:, 0].min() < margin or q[:, 1].min() < margin or
                q[:, 0].max() >= w - margin or q[:, 1].max() >= h - margin):
            return False
        if not cv2.isContourConvex(np.asarray(q, np.float32).reshape(-1, 1, 2)):
            return False
        e = np.roll(q, -1, axis=0) - q
        lengths = np.linalg.norm(e, axis=1)
        if lengths.min() < 12:
            return False
        # 透视图不要求四角接近 90 度，只排除退化尖角和过扁的四边形。
        sine = np.abs(e[:, 0] * np.roll(e[:, 1], -1) -
                      e[:, 1] * np.roll(e[:, 0], -1)) / (lengths * np.roll(lengths, -1))
        return bool(sine.min() > 0.20)

    def _layout(self, q, kind):
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
            rect_to_image = _homography(physical_quad, q)
            t = self._t
            # 三个点覆盖黑带的中部，再在内边缘后采两个亮面点。
            key = (w, h, tape)
            p = self._profiles.get(key)
            if p is None:
                offsets = tape * np.array([0.20, 0.40, 0.60, 1.22, 1.45])
                p = np.empty((4, len(t), 5, 2), np.float64)
                p[0, :, :, 0] = (tape + t * (w - 2 * tape))[:, None]
                p[0, :, :, 1] = offsets
                p[1, :, :, 0] = w - offsets
                p[1, :, :, 1] = (tape + t * (h - 2 * tape))[:, None]
                p[2, :, :, 0] = (tape + (1 - t) * (w - 2 * tape))[:, None]
                p[2, :, :, 1] = h - offsets
                p[3, :, :, 0] = offsets
                p[3, :, :, 1] = (tape + (1 - t) * (h - 2 * tape))[:, None]
                self._profiles[key] = p
            mapped = project_points(rect_to_image, p.reshape(-1, 2)).reshape(p.shape)
        except (ValueError, np.linalg.LinAlgError):
            return None
        gh, gw = gray.shape
        if (mapped[..., 0].min() < 0 or mapped[..., 1].min() < 0 or
                mapped[..., 0].max() >= gw - 1 or mapped[..., 1].max() >= gh - 1):
            return None
        samples = _sample(gray, mapped).astype(np.float32)
        dark = np.max(samples[:, :, :3], axis=2)
        # 两个数的中位数就是均值；避免进入 NumPy 通用中位数路径。
        bright = (samples[:, :, 3] + samples[:, :, 4]) * 0.5
        delta = bright - dark
        coverage = ((delta >= self.cfg.MIN_EDGE_CONTRAST) &
                    (dark <= bright * self.cfg.MAX_DARK_BRIGHT_RATIO)).mean(axis=1)
        edge_contrast = np.median(delta, axis=1)
        contrast = float(np.median(edge_contrast))
        # 至少三边的大部分采样成立，第四边也必须有证据，不接受凭亮中心猜框。
        if (np.count_nonzero(coverage >= self.cfg.MIN_EDGE_COVERAGE) < 3 or
                coverage.min() < self.cfg.WEAK_EDGE_COVERAGE or
                contrast < self.cfg.MIN_EDGE_CONTRAST):
            return None
        dark_median, bright_median = np.median(dark, axis=1), np.median(bright, axis=1)
        rows = [dict(dark=float(dark_median[i]), inside=float(bright_median[i]),
                     contrast=float(edge_contrast[i]), coverage=float(coverage[i])) for i in range(4)]
        outer = project_points(rect_to_image, _rectangle(w, h)).astype(np.float32)
        if not self._valid_quad(outer, gray.shape):
            return None
        return dict(quad=q, outer_quad=outer,
                    inner_quad=project_points(rect_to_image, _rectangle(w, h, tape)).astype(np.float32),
                    kind=kind, rect_to_image=rect_to_image, width_mm=w, height_mm=h,
                    edge_profile=rows, contrast=contrast,
                    min_contrast=float(edge_contrast.min()), coverage=coverage,
                    score=float(contrast + 60 * coverage.mean()))

    def _refine(self, gray, q, kind, radius):
        """当前帧四条灰度窄带 -> 有方向的边缘跳变 -> fitLine -> 四角交点。"""
        q = np.asarray(q, np.float32)
        nxt = np.roll(q, -1, axis=0)
        e = nxt - q
        lengths = np.linalg.norm(e, axis=1)
        if lengths.min() < 10:
            return None
        normal = np.column_stack((-e[:, 1], e[:, 0])) / lengths[:, None]
        base = q[:, None, :] * (1 - self._t[None, :, None]) + nxt[:, None, :] * self._t[None, :, None]
        offsets = np.arange(-radius, radius + 1, dtype=np.float32)
        points = base[:, :, None, :] + normal[:, None, None, :] * offsets[None, None, :, None]
        gh, gw = gray.shape
        inside = ((points[..., 0] >= 1) & (points[..., 0] < gw - 1) &
                  (points[..., 1] >= 1) & (points[..., 1] < gh - 1)).all(axis=2)
        values = _sample(gray, points).astype(np.float32)
        gradient = values[:, :, 2:] - values[:, :, :-2]
        if kind == "outer":
            gradient *= -1
        # 近预测边缘优先，避免捕获靶纸内部印刷圆或另一侧的胶带边缘。
        penalty = np.abs(offsets[1:-1])[None, None, :] * 0.45
        indices = np.argmax(gradient - penalty, axis=2)
        strength = np.take_along_axis(gradient, indices[..., None], axis=2)[..., 0]
        selected = offsets[indices + 1]
        lines = []
        for i in range(4):
            good = inside[i] & (strength[i] >= self.cfg.MIN_EDGE_CONTRAST * 0.7)
            if good.sum() < max(5, len(self._t) * self.cfg.WEAK_EDGE_COVERAGE):
                return None
            shifts = selected[i, good]
            med = float(np.median(shifts))
            good &= np.abs(selected[i] - med) <= max(2.5, float(np.median(np.abs(shifts - med))) * 3)
            if good.sum() < 5:
                return None
            samples = base[i, good] + selected[i, good, None] * normal[i]
            vx, vy, x, y = cv2.fitLine(samples, cv2.DIST_HUBER, 0, 0.01, 0.01).reshape(4)
            # 直线 ax+by+c=0。
            lines.append(np.array([-vy, vx, vy * x - vx * y], np.float64))
        lines = np.asarray(lines)
        crosses = np.cross(np.roll(lines, 1, axis=0), lines)
        if np.any(np.abs(crosses[:, 2]) < 1e-8):
            return None
        refined = (crosses[:, :2] / crosses[:, 2:]).astype(np.float32)
        if (np.linalg.norm(refined - q, axis=1).max() > radius * 2.8 or
                not self._valid_quad(refined, gray.shape)):
            return None
        return refined

    def _candidate_quads(self, mask):
        """用轮廓树的孔/黑环关系代替候选两两组合。"""
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        if hierarchy is None:
            return []
        hierarchy = hierarchy[0]
        h, w = mask.shape
        candidates = []
        # 先用廉价的包围框/面积筛选，仅拟合最大的少数轮廓。
        eligible = []
        # 按轮廓原始长度先筛选，限制 Python 包围框/面积调用次数。
        indices = sorted(range(len(contours)), key=lambda i: len(contours[i]), reverse=True)
        for i in indices[:int(getattr(self.cfg, "MAX_CONTOURS", 128))]:
            cnt = contours[i]
            x, y, cw, ch = cv2.boundingRect(cnt)
            if min(cw, ch) < 15 or cw * ch < w * h * self.cfg.MIN_AREA_RATIO:
                continue
            area = abs(cv2.contourArea(cnt))
            if w * h * self.cfg.MIN_AREA_RATIO <= area <= w * h * self.cfg.MAX_AREA_RATIO:
                eligible.append((area, i))
        for area, i in sorted(eligible, reverse=True)[:self.cfg.MAX_CANDIDATES]:
            cnt = contours[i]
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, self.cfg.APPROX_EPSILON * peri, True)
            # 不使用凸包修补任意背景：凸包可把桌面阴影拼成错误大四边形。
            if len(approx) != 4 or not cv2.isContourConvex(approx):
                continue
            q = _order_quad(approx)
            if not self._valid_quad(q, mask.shape):
                continue
            parent, depth = int(hierarchy[i][3]), 0
            while parent >= 0:
                depth += 1
                parent = int(hierarchy[parent][3])
            kind = "inner" if depth % 2 else "outer"
            if kind == "outer" and hierarchy[i][2] < 0:
                continue  # 实心黑块没有亮孔。
            candidates.append((q, kind))
        # 亮面孔比与背景粘连的黑环更稳定。
        return sorted(candidates, key=lambda c: c[1] != "inner")

    def _search(self, gray, roi=None):
        self._search_count += 1
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

    def _search_step(self, gray, roi, deadline):
        """每帧最多扫描一档阈值并验证少量候选；跨帧只保留几何，不保留旧图像。"""
        work = self._search_work
        if work is None:
            self._search_roi = roi
            work = self._search_work = dict(index=0, thresholds=None, pending=[], th=None)
        roi = self._search_roi
        stage = "roi_search" if roi is not None else "full_search"
        limit = max(1, int(getattr(self.cfg, "SEARCH_MAX_PROPOSALS", 2)))
        if not work["pending"]:
            x, y, w, h = roi if roi is not None else (0, 0, gray.shape[1], gray.shape[0])
            crop = gray[y:y + h, x:x + w]
            scale = min(1.0, self.cfg.SEARCH_WIDTH / float(w))
            sw, sh = max(32, int(round(w * scale))), max(24, int(round(h * scale)))
            small = cv2.resize(crop, (sw, sh), interpolation=cv2.INTER_AREA) if scale < 1 else crop
            sx, sy = w / float(sw), h / float(sh)
            if work["thresholds"] is None:
                otsu, _ = cv2.threshold(small, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
                thresholds = [self._locked_th] if self._locked_th is not None else []
                for th in (int(otsu),) + tuple(self.cfg.DARK_THRESHOLDS):
                    if not any(abs(th - t) < 6 for t in thresholds):
                        thresholds.append(th)
                if roi is not None:
                    thresholds = thresholds[:2]  # ROI 快速失败后切全图，避免困在已移出目标的 ROI。
                work["thresholds"] = thresholds
            index, thresholds = work["index"], work["thresholds"]
            if index > len(thresholds):
                # ROI 无候选时下一帧切全图；全图一轮结束后下一帧重新算 Otsu。
                self._search_work = None
                self._track = None
                return None, stage, False
            self._search_count += 1
            work["index"] += 1
            offset, factor = np.array([x, y], np.float32), np.array([sx, sy], np.float32)
            if index < len(thresholds):
                th = work["th"] = thresholds[index]
                self._threshold_count += 1
                _, mask = cv2.threshold(small, th, 255, cv2.THRESH_BINARY_INV)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
                work["pending"] = [(q * factor + offset, kind) for q, kind in self._candidate_quads(mask)]
            else:
                work["th"] = None
                edges = cv2.Canny(small, self.cfg.CANNY_LOW, self.cfg.CANNY_HIGH)
                contours = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
                contours = sorted(contours, key=cv2.contourArea, reverse=True)[:self.cfg.MAX_CANDIDATES]
                for cnt in contours:
                    if cv2.contourArea(cnt) < sw * sh * self.cfg.MIN_AREA_RATIO:
                        continue
                    ap = cv2.approxPolyDP(cnt, self.cfg.APPROX_EPSILON * cv2.arcLength(cnt, True), True)
                    if len(ap) == 4 and cv2.isContourConvex(ap):
                        q = _order_quad(ap) * factor + offset
                        work["pending"].extend([(q, "inner"), (q, "outer")])
            # 一次轮廓扫描也算一个不可拆操作；预算已用尽则本帧不再解几何。
        while work["pending"] and self._proposals < limit and time.perf_counter() < deadline:
            q, kind = work["pending"].pop(0)
            self._proposals += 1
            verified = self._verify(gray, q, kind)
            if verified is None:
                continue
            refined = self._refine(gray, q, kind, self.cfg.REFINE_RADIUS)
            if refined is not None:
                precise = self._verify(gray, refined, kind)
                if precise is not None:
                    verified = precise
            self._locked_th = work["th"]
            self._search_work = None
            return verified, stage, False
        return None, stage, True

    def process(self, gray, timestamp=None, exhaustive=False):
        started = time.perf_counter()
        g = np.asarray(gray)
        if g.ndim == 3 and g.shape[2] == 1:
            g = g[:, :, 0]
        if g.ndim != 2 or g.dtype != np.uint8:
            raise ValueError("process expects a uint8 grayscale image")
        self._frame += 1
        self._search_count = self._threshold_count = self._proposals = 0
        pending = False
        candidate, stage = None, "search"
        if self._track is not None:
            q, kind = self._track["quad"], self._track["kind"]
            q = self._refine(g, q, kind, self.cfg.TRACK_RADIUS)
            if q is not None:
                candidate = self._verify(g, q, kind)
                stage = "track"
        roi = None
        if candidate is None and self._track is not None:
            q = self._track["outer_quad"]
            lo, hi = q.min(axis=0), q.max(axis=0)
            pad = np.maximum((hi - lo) * self.cfg.ROI_PADDING, 24)
            lo = np.maximum(0, np.floor(lo - pad)).astype(int)
            hi = np.minimum([g.shape[1], g.shape[0]], np.ceil(hi + pad)).astype(int)
            roi = (int(lo[0]), int(lo[1]), int(hi[0] - lo[0]), int(hi[1] - lo[1]))
        if candidate is None and exhaustive:
            if roi is not None:
                candidate = self._search(g, roi)
                stage = "roi_search"
            if candidate is None:
                candidate = self._search(g)
                stage = "full_search"
        elif candidate is None:
            budget = max(0.1, float(getattr(self.cfg, "SEARCH_BUDGET_MS", 12))) / 1000.0
            candidate, stage, pending = self._search_step(g, roi, started + budget)
        if candidate is None:
            self._smooth = None
            self._misses += 1
            if self._misses > int(getattr(self.cfg, "TRACK_RETRY_FRAMES", 2)):
                self._track = None
            self.last = dict(valid=False, stage="search_pending" if pending else "no_black_frame",
                             search_stage=stage, search_count=self._search_count,
                             proposals=self._proposals, thresholds=self._threshold_count,
                             detector_ms=(time.perf_counter() - started) * 1000)
            return self.last
        self._misses = 0
        self._search_work = None
        self._track = candidate
        w, h = candidate["width_mm"], candidate["height_mm"]
        matrix = candidate["rect_to_image"]
        # 直接映射正视面圆心才是正确中心，圆周采样不用于平均中心。
        raw = project_points(matrix, [[w / 2, h / 2]])[0]
        alpha = float(self.cfg.CENTER_ALPHA)
        if (self._smooth is None or alpha <= 0 or
                np.linalg.norm(raw - self._smooth) > self.cfg.FILTER_RESET_PX):
            center = raw
        else:
            center = self._smooth + alpha * (raw - self._smooth)
        self._smooth = center
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
    """
    g = np.asarray(gray).squeeze()
    scale = min(1.0, float(show_width) / g.shape[1])
    if scale < 1:
        g = cv2.resize(g, (int(round(g.shape[1] * scale)), int(round(g.shape[0] * scale))))
    canvas = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    if not result.get("valid"):
        cv2.putText(canvas, "NO BLACK RECTANGLE", (8, 24 + top_offset), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 255), 1)
        return canvas
    q = np.rint(result["quad"] * scale).astype(np.int32)
    iq = np.rint(result["inner_quad"] * scale).astype(np.int32)
    cv2.polylines(canvas, [q], True, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.polylines(canvas, [iq], True, (0, 220, 220), 1)
    for i, (x, y) in enumerate(result["quad"]):
        p = tuple(q[i])
        cv2.circle(canvas, p, 4, (255, 0, 255), -1)
        label = "P%d (%d,%d)" % (i + 1, round(float(x)), round(float(y)))
        tw = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .36, 1)[0][0]
        pos = (max(0, min(int(p[0]) + 5, canvas.shape[1] - tw - 1)),
               max(12, min(int(p[1]) - 7, canvas.shape[0] - 5)))
        cv2.putText(canvas, label, pos, cv2.FONT_HERSHEY_SIMPLEX, .36, (255, 255, 0), 1, cv2.LINE_AA)
    c = tuple(np.rint(np.array([result["u"], result["v"]]) * scale).astype(int))
    cv2.drawMarker(canvas, c, (255, 0, 0), cv2.MARKER_CROSS, 14, 2)
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
        cfg = self.cfg
        H, W = g.shape[:2]
        cx, cy = float(cfg.LASER_U_NOMINAL), float(cfg.LASER_V_NOMINAL)
        r = int(cfg.LASER_ROI_RADIUS)
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
        """每帧调用；fixed 模式恒返回标定常量，detect 模式内部按 LASER_INTERVAL_S 限频。"""
        if self.mode == "fixed":
            return self.static_state()
        now = time.perf_counter() if now is None else float(now)
        if self._t is not None and (now - self._t) < self.interval:
            out = dict(self.last)
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
        return None
    if not result or not result.get("valid"):
        return None
    if use_raw and result.get("raw_u") is not None and result.get("raw_v") is not None:
        tx, ty = float(result["raw_u"]), float(result["raw_v"])
    else:
        tx, ty = float(result["u"]), float(result["v"])
    return (tx - float(laser["u"]), ty - float(laser["v"]))


def draw_laser_overlay(canvas, laser, result=None, scale=1.0, use_raw=False):
    """在显示画布左上角画激光读数面板 + 光斑标记 + 到靶心的连线。

    use_raw 与 deviation() 保持一致：True 时连线终点与 dx/dy 都用未平滑的 raw 中心，
    屏幕上看到的就是控制律真正在用的那个点。

    返回面板占用的高度（像素，显示画布坐标），调用方据此把 draw_result 的文字
    整体下移，避免和本面板重叠。
    """
    if not laser:
        return 0
    found = bool(laser.get("found"))
    if laser.get("reason") == "fixed":
        line1 = "LASER (%d,%d) [fixed/calib]" % (round(laser["u"]), round(laser["v"]))
    elif found:
        line1 = "LASER (%d,%d) peak=%d area=%d" % (
            round(laser["u"]), round(laser["v"]), round(laser["peak"]), laser.get("area", 0))
    else:
        line1 = "LASER -- [%s] peak=%d bg=%d" % (
            laser.get("reason", "?"), round(laser.get("peak", 0.0)), round(laser.get("bg", 0.0)))
    dv = deviation(laser, result, use_raw=use_raw)
    if dv is not None:
        line2 = "D-CENTER dx=%+.1f dy=%+.1f px" % (dv[0], dv[1])
        line2_color = _TEXT_C2
    elif result is not None and result.get("valid"):
        line2 = "D-CENTER -- (laser lost)"
        line2_color = _TEXT_C3
    else:
        line2 = "D-CENTER -- (no black rect)"
        line2_color = _TEXT_C3
    w = max(cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, .45, 1)[0][0] for t in (line1, line2))
    cv2.rectangle(canvas, (2, 2), (min(canvas.shape[1] - 1, w + 14), PANEL_H - 2), _PANEL_BG, -1)
    cv2.putText(canvas, line1, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, .45, _TEXT_C1, 1, cv2.LINE_AA)
    cv2.putText(canvas, line2, (8, 18 + PANEL_LINE), cv2.FONT_HERSHEY_SIMPLEX, .45, line2_color, 1, cv2.LINE_AA)
    if not found:
        return PANEL_H
    p = (int(round(laser["u"] * scale)), int(round(laser["v"] * scale)))
    if result is not None and result.get("valid"):
        if use_raw and result.get("raw_u") is not None and result.get("raw_v") is not None:
            c = (int(round(float(result["raw_u"]) * scale)), int(round(float(result["raw_v"]) * scale)))
        else:
            c = (int(round(float(result["u"]) * scale)), int(round(float(result["v"]) * scale)))
        cv2.line(canvas, p, c, _LINK, 1, cv2.LINE_AA)
    cv2.circle(canvas, p, 7, _MARK, 1, cv2.LINE_AA)
    cv2.drawMarker(canvas, p, _MARK, cv2.MARKER_CROSS, 5, 1, cv2.LINE_AA)
    return PANEL_H
'''

# ===== vision.gimbal_self_test  (vision/gimbal_self_test.py) =====
_MOD_SRC['vision.gimbal_self_test'] = r'''
"""上电云台小幅自检：**pitch 先上转再下转、yaw 先左转再右转**，最后回到启动姿态。

默认走**固定脉冲**（``SELF_TEST_PULSES``）——**不需要看到靶子、也不依赖脉冲/度标定**，
上电即可验证"串口通 + 两轴机械行程"；置 0 才回退到按角度换算的精细自检。

两条设计红线（2026-10-09 修复"U2 只使能、不自校"时定下）：

1. **只对链路真的打开的轴做自检** —— 一条轴不通不该拖累另一条；
2. **单轴写失败只跳过该轴剩余步骤**，另一轴继续走完。旧实现遇到第一条 yaw 指令
   写失败就整机 FAULT，导致 pitch 一步都收不到指令（现场表现就是"只使能、不动"）。

第三条（2026-10-09 现场反馈"自检把电机抖得卡顿"后定下）：**自检运动必须慢 + 带曲线**。
位置模式命令的加速度档位 `acc=0` 表示"不使用曲线加减速、直接以设定速度启停"，
对云台这种小惯量机构就是硬启硬停；`acc≠0` 时按 ZDT 官方公式每 `(256-acc)*50us`
变化 1 RPM，**acc 越小越柔和**。见 `estimate_move_seconds()`。
"""


def estimate_move_seconds(pulses, rpm, acc, pulses_per_rev=3200.0):
    """估算一段位置命令的运动时间（秒），供自检排程使用（纯函数，可单测）。

    模型（ZDT Emm_V5.0 官方说明）：
      · `acc == 0`：不使用曲线加减速，直接以设定速度运行 → 只有匀速段；
      · `acc != 0`：曲线加减速，速度变化率 a = 1 / ((256 - acc) * 50us) [RPM/s]。
    行程不足时自动退化为**三角速度曲线**（实际峰值低于设定速度），此时按
    `2 * V_peak / a` 计算，不会高估成梯形时间。
    """
    pulses = max(0.0, float(pulses))
    rpm = max(1e-6, float(rpm))
    per_rev = max(1.0, float(pulses_per_rev))
    rate = rpm * per_rev / 60.0                     # 脉冲/秒（匀速段）
    cruise_s = pulses / rate
    acc = int(acc)
    if acc <= 0:
        return cruise_s
    # 秒/RPM：每 (256-acc)*50us 变化 1 RPM
    k = (256 - max(0, min(255, acc))) * 50e-6
    t_ramp = rpm * k                                # 从 0 加到设定速度所需时间
    s_ramp = 0.5 * rpm * t_ramp * per_rev / 60.0    # 加速段走过的脉冲（平均速度 = rpm/2）
    if 2.0 * s_ramp <= pulses:                      # 能加到设定速度 -> 梯形
        return 2.0 * t_ramp + (pulses - 2.0 * s_ramp) / rate
    v_peak = (pulses * 60.0 / (k * per_rev)) ** 0.5  # 三角：峰值转速（RPM）
    return 2.0 * v_peak * k


class GimbalSelfTest:
    """非阻塞自检状态机。角度通过实测的电机脉冲/输出轴度数换算。"""

    def __init__(self, cfg, link, now=0.0):
        self.cfg = cfg
        self.link = link
        self.state = "IDLE"
        self.index = 0
        self.sent = 0
        self.total = 0
        self.deadline = float(now) + float(getattr(cfg, "SELF_TEST_START_DELAY_S", 0.2))
        # ★ 硬超时：到点**无条件**结束自检（"上电 3s 内跑完"的兜底），0 = 关闭。
        self._t0 = float(now)
        self.timeout_s = float(getattr(cfg, "SELF_TEST_TIMEOUT_S", 0.0) or 0.0)
        self.sequence = []
        self.reason = None
        self._failed = set()
        self._planned = set()
        # 可用轴：链路报告了就把不通的轴排除在自检之外；没有该接口（PC 假链路）则不筛。
        self.available = None
        probe = getattr(link, "available_axes", None)
        if callable(probe):
            try:
                self.available = set(probe())
            except Exception:
                self.available = None
        self._prepare()

    def _install(self, seq):
        """装入自检序列；剔除链路未打开的轴（一条轴不通不该拖累另一条）。"""
        missing = sorted({a for a, _c, _p in seq
                          if self.available is not None and a not in self.available})
        if missing:
            print("[self-test] 跳过链路未打开的轴: %s（不参与自检，另一轴照常）" % ",".join(missing))
            seq = [s for s in seq if s[0] not in missing]
        info_map = getattr(self.link, "axis_info", None)
        if isinstance(info_map, dict):
            for axis in ("yaw", "pitch"):
                info = info_map.get(axis) or {}
                if not info.get("opened"):
                    print("[self-test] %-5s 链路未就绪: dev=%s err=%s" %
                          (axis, info.get("device"), info.get("error") or "not opened"))
        if not seq:
            self.state, self.reason = "SKIPPED", "no_available_axis"
            print("[self-test] 两轴串口都没打开 -> 自检跳过（不会看到任何电机动作）")
            return
        self.sequence = list(seq)
        self.total = len(self.sequence)
        self._planned = {s[0] for s in self.sequence}
        self.state = "RUNNING"

    def _prepare(self):
        if not bool(getattr(self.cfg, "GIMBAL_SELF_TEST", True)):
            self.state, self.reason = "DONE", "disabled"
            return
        yaw_ccw_positive = bool(getattr(self.cfg, "YAW_POSITIVE_CCW", False))
        pitch_ccw_positive = bool(getattr(self.cfg, "PITCH_POSITIVE_CCW", False))
        # ★ 首选「固定脉冲自检」：**不需要看到靶子（不依赖视觉）、也不依赖脉冲/度标定**，
        #   上电即可验证两轴链路与机械行程。顺序按现场要求：
        #   **pitch 先上转再下转，yaw 先左转再右转**；每轴两段等长，结束时回到起始位姿。
        pulses = int(getattr(self.cfg, "SELF_TEST_PULSES",
                             getattr(self.cfg, "SELF_TEST_FALLBACK_PULSES", 0)))
        if pulses > 0:
            print("[self-test] 固定脉冲自检（无需靶子 / 无需标定）："
                  "pitch 上转 %d → 下转 %d，yaw 左转 %d → 右转 %d"
                  % (pulses, pulses, pulses, pulses))
            self._install([
                ("pitch", pitch_ccw_positive, pulses),        # pitch 上转（仰起）
                ("pitch", not pitch_ccw_positive, pulses),    # pitch 下转（回到起始）
                ("yaw", yaw_ccw_positive, pulses),            # yaw 左转
                ("yaw", not yaw_ccw_positive, pulses),        # yaw 右转（回到起始）
            ])
            return
        # 备选：按"脉冲/度"换算角度的精细自检（需先 CALIBRATE_AXIS 实测，或给 GIMBAL_GEAR_RATIO）。
        yaw_ppd = float(getattr(self.cfg, "YAW_PULSES_PER_DEG", 0.0))
        pitch_ppd = float(getattr(self.cfg, "PITCH_PULSES_PER_DEG", 0.0))
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
            print("[self-test] skipped: 置 SELF_TEST_PULSES>0 走固定脉冲自检（推荐，无需靶子）")
            print("[self-test]           或填 YAW/PITCH_PULSES_PER_DEG / GIMBAL_GEAR_RATIO 走角度自检")
            return
        yaw_angle = abs(float(getattr(self.cfg, "SELF_TEST_YAW_DEG", 15.0)))
        pitch_angle = abs(float(getattr(self.cfg, "SELF_TEST_PITCH_DEG", 30.0)))
        self._install([
            ("yaw", yaw_ccw_positive, yaw_angle * yaw_ppd),
            ("yaw", not yaw_ccw_positive, 2 * yaw_angle * yaw_ppd),
            ("yaw", yaw_ccw_positive, yaw_angle * yaw_ppd),
            ("pitch", pitch_ccw_positive, pitch_angle * pitch_ppd),
            ("pitch", not pitch_ccw_positive, 2 * pitch_angle * pitch_ppd),
            ("pitch", pitch_ccw_positive, pitch_angle * pitch_ppd),
        ])

    @property
    def done(self):
        return self.state in ("DONE", "SKIPPED", "FAULT")

    def update(self, now):
        """每帧调用；到达估算运动时间后发下一段，不阻塞视觉主循环。"""
        if self.done:
            return self.state
        now = float(now)
        # ★ 硬超时优先于一切：无论卡在哪一步（链路抖动、帧率骤降、某步不返回），
        #   到点就结束自检并交给上层继续（接着标定 / 直接追踪）。
        # 只在"**还有步没发完**"时才算卡住并强制结束；4 步都发出后应正常等最后一步走完，
        # 否则最后一段机械动作会被截断、紧接着的追踪指令会与之堆叠。
        if self.timeout_s > 0 and self.sequence and now - self._t0 >= self.timeout_s:
            self.state = "DONE"
            print("[self-test] 硬超时 %.1fs -> 强制结束（已发 %d/%d 步），转入下一阶段"
                  % (self.timeout_s, self.sent, self.total))
            return self.state
        if now < self.deadline:
            return self.state
        if not self.sequence:
            self.state = "DONE"
            print("[self-test] complete; returned to startup pose")
            return self.state
        axis, ccw, raw_pulses = self.sequence.pop(0)
        pulses = int(round(raw_pulses))
        self.sent += 1
        if pulses <= 0:
            print("[self-test] %s 脉冲数非法(%d) -> 跳过该步" % (axis, pulses))
            self.deadline = now + 0.05
            return self.state
        # ★ 慢速 + 曲线加减速：SELF_TEST_RPM 默认 60（旧 900 太快、肉眼像"弹"一下），
        #   SELF_TEST_ACC 默认 60（非 0 才启用 ZDT 曲线加减速；0 = 直接启停 = 卡顿）。
        rpm = int(getattr(self.cfg, "SELF_TEST_RPM", 60))
        acc = int(getattr(self.cfg, "SELF_TEST_ACC", 0))
        if not self.link.move(axis, ccw, pulses, rpm, acc):
            # 关键：单轴写失败只跳过该轴剩余步骤，另一轴继续 —— 旧实现直接整机 FAULT，
            # 结果"yaw 链路不通 -> pitch 一步指令都发不出去"（现场=只使能、不动）。
            self._failed.add(axis)
            self.sequence = [s for s in self.sequence if s[0] not in self._failed]
            print("[self-test] %s 指令发送失败 -> 跳过该轴剩余步骤（另一轴继续）" % axis)
            # 判据是"**所有参与轴都失败**"才算 FAULT，而不是"序列恰好走空" ——
            # 否则"最后一轴才失败"（例如 pitch 先动完、yaw 才写失败）会被误判成整机故障。
            if self._planned and self._failed >= self._planned:
                self.state, self.reason = "FAULT", "uart_write_failed"
                print("[self-test] fault: 所有参与轴的指令都发不出去")
            self.deadline = now + 0.05
            return self.state
        pulses_per_rev = float(getattr(self.cfg, "MOTOR_PULSES_PER_REV", 3200))
        move_s = estimate_move_seconds(pulses, rpm, acc, pulses_per_rev)
        settle_s = float(getattr(self.cfg, "SELF_TEST_SETTLE_S", 0.8))
        self.deadline = now + move_s + settle_s
        print("[self-test] %s %s %dp rpm=%d acc=%d step=%d/%d 预计%.2fs" %
              (axis, "CCW" if ccw else "CW", pulses, rpm, acc,
               self.sent, self.total, move_s + settle_s))
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
- 相对位置命令（0xFD、mode=0），运动量低通/增长限速，等待曲线运动估算完成；
- 状态机 ``BOOT / COARSE / FINE / SETTLE / HOLD / SCAN / FAULT``；
- 三重保护：离群拒收（``JUMP_REJECT_PX``）、死区（``DEADBAND_PX``）、指令限频
  （``COMMAND_INTERVAL_S``）、单帧限幅（``MAX_STEP_PULSES``）。

本模块**保持纯 Python（不 import maix）**，可在 PC 上离线仿真与回归。
控制律可整体替换：``controller(axis, error_px, now) -> 带符号脉冲数``，传 None 用内置比例律。
"""

import math

from vision.gimbal_self_test import estimate_move_seconds

# 状态语义（给日志和上层判断用）
BOOT = "BOOT"        # 上电，等第一帧合法误差
COARSE = "COARSE"    # 大误差，大步快速逼近
FINE = "FINE"        # 小误差，小步逼近
SETTLE = "SETTLE"    # 进入死区，停止发指令
HOLD = "HOLD"        # 视觉失效，保持不动
SCAN = "SCAN"        # 失锁后慢扫重捕获（需 ENABLE_SCAN）
FAULT = "FAULT"      # 链路/机械故障，停机不再发指令
UNCALIBRATED = "UNCALIBRATED"  # 未标定 PULSE_PER_PX/DIR_SIGN，闭环锁定、只出视觉结果


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
        # 安全闸：未完成开环标定时禁止发电机指令。PULSE_PER_PX_*/DIR_SIGN_* 若还是占位值，
        # 直接闭环可能正反馈甩头（README §四）。标定完把 GIMBAL_CALIBRATED=True 解锁。
        self.require_calibration = bool(getattr(cfg, "AIM_REQUIRE_CALIBRATION", False))
        self.calibrated = bool(getattr(cfg, "GIMBAL_CALIBRATED", False))
        self.locked = self.require_calibration and not self.calibrated
        if self.locked:
            self.state = UNCALIBRATED
            print("[aim] 未标定：闭环已锁定（只输出视觉结果，不发电机指令）")
            print("[aim] 先跑 CALIBRATE_AXIS 标定 PULSE_PER_PX_*/DIR_SIGN_*，再置 GIMBAL_CALIBRATED=True")
        # ★ 发散看护：一直发指令却始终不收敛（误差反而变大）= 方向反了（正反馈）
        #   -> 自动 FAULT 停机，避免云台一路冲到底。详见 `_guard_check()`。
        self._guard_on = bool(getattr(cfg, "DIVERGE_GUARD", False))
        self._guard_frames = int(getattr(cfg, "DIVERGE_FRAMES", 30))
        self._guard_ratio = float(getattr(cfg, "DIVERGE_RATIO", 1.5))
        self._guard_n = 0
        self._guard_min = None
        self._motion_until = {"yaw": -1e9, "pitch": -1e9}
        self._pending_stops = set()
        self._clear_output()

    # ------------------------------------------------------------- 外部接口
    def set_controller(self, controller):
        """设置后续 PID/前馈控制器；传入 None 恢复内置比例律。"""
        self.controller = controller
        self._clear_output()

    def fault(self, reason):
        """进入 FAULT：不再发任何指令，由上层决定是否复位。"""
        self.state = FAULT
        self.fault_reason = str(reason)
        self._clear_output()
        print("[aim] FAULT: %s" % self.fault_reason)
        return self.state

    def unlock(self, reason="calibrated"):
        """标定完成 -> 解除未标定锁，允许发电机指令。返回是否真的发生了状态变化。

        由 ``AutoCalibrate`` 在两轴全部标定成功后调用。只有在
        ``AIM_REQUIRE_CALIBRATION=True`` 且 ``GIMBAL_CALIBRATED=False`` 时才处于锁定态，
        因此重复调用是幂等的（第二次返回 False）。
        """
        if not self.locked:
            return False
        self.locked = False
        self.calibrated = True
        if self.state == UNCALIBRATED:
            self.state = BOOT
        self.fault_reason = None
        self._lost = self._settle = self._scan_pos = 0
        self._scan_dir = 1
        self._jump = {"yaw": 0, "pitch": 0}
        self.last_command_time = {"yaw": -1e9, "pitch": -1e9}
        self._clear_output()
        print("[aim] 闭环解锁（%s）：PULSE_PER_PX_*/DIR_SIGN_* 已按实测值写入，开始跟踪" % reason)
        return True

    def reset(self):
        self.state = UNCALIBRATED if self.locked else BOOT
        self.fault_reason = None
        self._lost = self._settle = self._scan_pos = 0
        self._scan_dir = 1
        self._jump = {"yaw": 0, "pitch": 0}
        self.last_command_time = {"yaw": -1e9, "pitch": -1e9}
        self._guard_n = 0
        self._guard_min = None
        self._clear_output()
        return self.state

    def update(self, result, now=0.0):
        """输入视觉 result，返回 ``cmds``；无效结果按状态机处理（HOLD / SCAN）。"""
        now = float(now)
        self._pending_stops = set()
        if self.locked:
            valid = bool(result and result.get("aim_valid") and
                         result.get("dx") is not None and result.get("dy") is not None)
            dx = float(result["dx"]) if valid else None
            dy = float(result["dy"]) if valid else None
            self.state = UNCALIBRATED
            return self._emit(dx, dy, {"yaw": None, "pitch": None})
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
            for axis in ("yaw", "pitch"):
                self._cancel_motion(axis, now)
            self._clear_output()
            self._guard_n, self._guard_min = 0, None
            self._settle += 1
            self.state = SETTLE
            return self._emit(dx, dy, {"yaw": None, "pitch": None})
        self._settle = 0
        # 发散看护：只在"确实要发指令"时检查（死区内已提前 return，不参与判断）。
        if self._guard_check(max(abs(dx), abs(dy))):
            for axis in ("yaw", "pitch"):
                self._cancel_motion(axis, now)
            return self._emit(dx, dy, {"yaw": None, "pitch": None})
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
        out = {"state": self.state, "dx": dx, "dy": dy, "cmds": cmds,
               "stops": sorted(self._pending_stops),
               "filtered_pulses": dict(self._output)}
        self.last = out
        return out

    def _deadband(self):
        return float(getattr(self.cfg, "DEADBAND_PX", 2.0))

    def _clear_output(self, axis=None):
        """清除滤波记忆，但保留尚未结束的机械运动等待时间。"""
        if axis is None:
            self._output = {"yaw": 0.0, "pitch": 0.0}
            self._output_time = {"yaw": None, "pitch": None}
            self._error_sign = {"yaw": 0, "pitch": 0}
        else:
            self._output[axis] = 0.0
            self._output_time[axis] = None
            self._error_sign[axis] = 0

    def _cancel_motion(self, axis, now):
        if now < self._motion_until[axis]:
            self._pending_stops.add(axis)
            # 停止后留出一帧间隔再反向，避免同帧 stop/move 贴在一起。
            self.last_command_time[axis] = now
            self._motion_until[axis] = now

    def _filter_output(self, axis, requested, now):
        """时间低通 + 增长限速；缩小运动量立即生效，避免滤波拖尾越过靶心。"""
        tau = max(0.0, float(getattr(self.cfg, "AIM_OUTPUT_LPF_TAU_S", 0.0)))
        slew = max(0.0, float(getattr(self.cfg, "AIM_OUTPUT_SLEW_PPS", 0.0)))
        previous = self._output[axis]
        last = self._output_time[axis]
        max_dt = max(0.001, float(getattr(self.cfg, "COMMAND_INTERVAL_S", 0.1)))
        if last is None:
            dt = max_dt
        else:
            # 以实际控制周期计算，不再把 0.5s 的间隔压成 0.1s 导致低帧率下永远小步爬行。
            dt = min(max(0.25, max_dt), max(0.0, now - last))
        self._output_time[axis] = now
        if abs(requested) <= abs(previous):
            value = requested
        else:
            alpha = 1.0 - math.exp(-dt / tau) if tau > 0 else 1.0
            magnitude = abs(previous) + alpha * (abs(requested) - abs(previous))
            if slew > 0:
                magnitude = min(magnitude, abs(previous) + slew * dt)
            start = float(getattr(self.cfg, "AIM_START_PULSES", 0.0))
            if last is None and start > 0:
                magnitude = min(magnitude, start)
            value = math.copysign(magnitude, requested)
        self._output[axis] = value
        return value

    def _guard_check(self, err_abs):
        """发散看护：判断"一直在发指令却始终不收敛"，是则置 FAULT。

        判据：观察窗内记录 |误差| 的**最小值**；窗口满之后若当前值涨到 `最小值 × DIVERGE_RATIO`
        以上，说明"追了这么久反而更偏"，与收敛的预期相反 —— 多半是 `DIR_SIGN_*` 反了。
        误差每次创出新低就重新计数，因此正常波动/正常收敛不会被误判。
        """
        if not self._guard_on:
            return False
        if self._guard_min is None or err_abs < self._guard_min:
            self._guard_min = err_abs
            self._guard_n = 0
            return False
        self._guard_n += 1
        if self._guard_n < self._guard_frames:
            return False
        if self._guard_min > 0 and err_abs >= self._guard_min * self._guard_ratio:
            self.fault("diverge(|误差| 从 %.1fpx 涨到 %.1fpx，趋势不收敛 -> 检查 DIR_SIGN_*)"
                       % (self._guard_min, err_abs))
            return True
        self._guard_n = 0
        return False

    def _on_lost(self, now):
        self._clear_output()
        self._guard_n, self._guard_min = 0, None
        self._jump = {"yaw": 0, "pitch": 0}
        for axis in ("yaw", "pitch"):
            self._cancel_motion(axis, now)
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

    def _pulse_per_px(self, axis):
        """像素 -> 脉冲 的换算系数，也是"调节激烈程度"的总闸门。

        取值优先级（现场调参入口见 `config.py` 的「★ 追踪手感」）：
          ① `DEG_PER_PX_x × PULSES_PER_DEG_x` —— **手感宏，推荐**（每像素多少度 × 每度多少脉冲）；
          ② 回退 `PULSE_PER_PX_x` —— CALIBRATE_AXIS 实测写回的值 / 旧口径。

        标定实测出的脉冲/像素可反算回宏：`DEG_PER_PX = PULSE_PER_PX / PULSES_PER_DEG`。
        """
        cfg = self.cfg
        upper = axis.upper()
        deg = float(getattr(cfg, "DEG_PER_PX_%s" % upper, 0.0) or 0.0)
        ppd = float(getattr(cfg, "PULSES_PER_DEG_%s" % upper, 0.0) or 0.0)
        if deg > 0 and ppd > 0:
            return deg * ppd
        return float(getattr(cfg, "PULSE_PER_PX_%s" % upper, 3.67))

    def _rpm(self):
        if self.state == COARSE:
            return int(getattr(self.cfg, "RPM_COARSE", 150))
        return int(getattr(self.cfg, "RPM_FINE", 60))

    def _axis_command(self, axis, error, now):
        cfg = self.cfg
        if abs(error) <= self._deadband():
            self._cancel_motion(axis, now)
            self._clear_output(axis)
            self._jump[axis] = 0
            return None
        direction = 1 if error > 0 else -1
        if self._error_sign[axis] and direction != self._error_sign[axis]:
            self._cancel_motion(axis, now)
            self._clear_output(axis)
        self._error_sign[axis] = direction
        # 离群拒收：单帧超大误差先当误检丢弃，连续若干帧才采信（防甩头）。
        jump = float(getattr(cfg, "JUMP_REJECT_PX", 0.0))
        if jump > 0 and abs(error) > jump:
            self._jump[axis] += 1
            if self._jump[axis] < int(getattr(cfg, "JUMP_CONFIRM_FRAMES", 3)):
                self._clear_output(axis)
                return None
        else:
            self._jump[axis] = 0
        # 等待只比较时刻，不 sleep/循环。只有准备发送追踪命令时才运行低通/控制律。
        interval = float(getattr(cfg, "COMMAND_INTERVAL_S", 0.08))
        if now - self.last_command_time[axis] < interval or now < self._motion_until[axis]:
            return None
        sign = float(getattr(cfg, "DIR_SIGN_YAW" if axis == "yaw" else "DIR_SIGN_PITCH", 1))
        max_pulses = int(getattr(cfg, "MAX_STEP_PULSES", 360))
        if self.controller is not None:
            output = self.controller(axis, float(error), now)
            if output is None or float(output) == 0:
                self._clear_output(axis)
                return None
            output = float(output)
        else:
            ppp = self._pulse_per_px(axis)
            output = error * ppp * self._gain(axis, error)
        output = max(-max_pulses, min(max_pulses, output))
        if self._output[axis] * output < 0:
            self._cancel_motion(axis, now)
            self._clear_output(axis)
        output = self._filter_output(axis, output, now)
        if axis in self._pending_stops:
            return None
        pulses = min(max_pulses, int(round(abs(output))))
        if pulses <= 0:
            return None
        ccw = output * sign > 0
        rpm = self._rpm()
        if bool(getattr(cfg, "AIM_WAIT_MOTION", False)):
            acc = int(getattr(cfg, "ACC_TRACK", getattr(cfg, "ACC_DEFAULT", 0)))
            duration = estimate_move_seconds(pulses, rpm, acc,
                                            getattr(cfg, "MOTOR_PULSES_PER_REV", 3200))
            margin = max(0.0, float(getattr(cfg, "AIM_MOTION_MARGIN_S", 0.03)))
            self._motion_until[axis] = now + duration + margin
        self.last_command_time[axis] = now
        return ccw, pulses, rpm

    def _scan_command(self, axis, now):
        cfg = self.cfg
        if now < self._motion_until[axis]:
            return None
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

⚠ **MaixCAM2 可用串口（2026-10-09 核对官方 UART 表 + 引脚图 v1.0）**：

    串口   丝印      引脚        设备路径        pinmap
    UART0  U0T/U0R   A28 / A29   /dev/ttyS0     免（默认已复用）；系统终端/日志口
    UART1  U1T/U1R   A30 / A31   /dev/ttyS1     需（官方 UART 表标"无引脚"）
    UART2  U2T/U2R   B0  / B1    /dev/ttyS2     需
    UART3  U3T/U3R   B2  / B3    /dev/ttyS3     需
    UART4  U4T/U4R   A21 / A22   /dev/ttyS4     需；底部 6P 扩展座默认串口

本工程：轴 1 (yaw) = UART0(U0T/U0R)，轴 2 (pitch) = UART2(B0/B1)。
配置项 `YAW_DEV/YAW_BAUD/YAW_PINS` 与 `PITCH_DEV/PITCH_BAUD/PITCH_PINS`（旧名
`U1_*`/`U2_*` 仍兼容）。**pins 传空表 = 该轴不做 pinmap**；两轴默认都显式声明引脚功能，
且 `PINMAP_STRICT=False`（默认）时 pinmap 失败只警告、**仍打开 UART**，
避免因功能名不匹配把整条轴变成"打不开"（那会把真正的故障点藏起来）。

打开后还会用 `pinmap.get_pin_function()` **回读实测功能**（MaixCAM2 支持从硬件读），
启动日志以 `└ 引脚实测:` 一行打印 —— 这是判断"某一轴为什么不通"的第一手证据。


帧构造是纯函数（``cmd_*``），返回帧解析也做成纯函数 + 可注入的 ``_ingest``，
这样即使没有 MaixPy 也能在 PC 上逐字节回归。

返回帧：[地址][功能码][状态][校验]，状态 `0x02`=成功、`0xE2`=未使能/堵转、
`0x00 0xEE`=错误命令（见 §6.1）。
"""

import time

CR = 0x6B
STATUS_OK = 0x02
STATUS_E2 = 0xE2

# 各轴的"标称引脚"（仅用于诊断回读）：即使 config 里 pins 为空，也去查这些脚**实测**是什么功能。
NOMINAL_PINS = {
    "yaw": {"A28": "UART0_TX", "A29": "UART0_RX"},
    "pitch": {"B0": "UART2_TX", "B1": "UART2_RX"},
}


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


def fault_reason(fb):
    """从 ``poll()``/``_ingest()`` 的结果里取出故障原因；**无故障返回 None**。

    ★ 必须走这个函数，不要直接写 ``fb.get("fault") is not None``：
    ``_ingest()`` 的约定是 **"fault: False 表示无故障"**（见其返回值），而
    ``False is not None`` 恒为 True —— 2026-10-09 现场 bug 就是 main.py 用
    ``is not None`` 判断，导致**开机第 1 帧就误判 FAULT 并停机**，
    云台从此不接收任何指令（表现："电机上电有劲、但永不自检、永不追踪"）。
    """
    if not fb:
        return None
    return fb.get("reason") if fb.get("fault") else None


def axis_cfg(cfg, axis):
    """读取某轴的 (device, baud, pins)。新命名 `YAW_*`/`PITCH_*` 优先，兼容旧 `U1_*`/`U2_*`。

    pins 为空 dict 表示**该轴不做 pinmap**（设备默认已复用为串口，如 UART0/U0T-U0R）。
    """
    if axis == "yaw":
        dev = getattr(cfg, "YAW_DEV", getattr(cfg, "U1_DEV", "/dev/ttyS0"))
        baud = getattr(cfg, "YAW_BAUD", getattr(cfg, "U1_BAUD", 115200))
        pins = getattr(cfg, "YAW_PINS", getattr(cfg, "U1_PINS", None))
        if pins is None:
            pins = {}
    else:
        dev = getattr(cfg, "PITCH_DEV", getattr(cfg, "U2_DEV", "/dev/ttyS2"))
        baud = getattr(cfg, "PITCH_BAUD", getattr(cfg, "U2_BAUD", 115200))
        pins = getattr(cfg, "PITCH_PINS", getattr(cfg, "U2_PINS", None))
        if pins is None:
            pins = {"B0": "UART2_TX", "B1": "UART2_RX"}
    return str(dev), int(baud), dict(pins)


def _pins_text(pins):
    """把引脚映射打印成可读串；空表表示"免 pinmap（设备默认已复用为串口）"。"""
    if not pins:
        return "(默认复用,免pinmap)"
    return ",".join("%s=%s" % kv for kv in pins.items())


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
        self._init_error = None
        # 逐轴链路诊断（启动打印 / uart_probe 用）：设备、引脚、pinmap 结果、是否打开、失败原因。
        self.axis_info = {}
        for axis in ("yaw", "pitch"):
            dev, baud, pins = axis_cfg(cfg, axis)
            self.axis_info[axis] = {
                "device": dev, "baud": baud, "pins": pins,
                "pinmap_ok": None, "pinmap_error": None, "pin_funcs": {},
                "opened": False, "error": None,
            }
        try:
            from maix import uart, pinmap
            try:
                from maix import err
            except Exception:
                err = None
        except Exception as exc:
            self._stats["err"] += 1
            self._stats["yaw"] = self._stats["pitch"] = False
            self._init_error = exc
            for info in self.axis_info.values():
                info["error"] = "maix import failed: %s" % exc
            return
        strict = bool(getattr(cfg, "PINMAP_STRICT", False))
        for axis in ("yaw", "pitch"):
            info = self.axis_info[axis]
            # ---- 1) pinmap（失败是否致命由 PINMAP_STRICT 决定）----
            try:
                for pin, func in info["pins"].items():
                    result = pinmap.set_pin_function(pin, func)
                    # 严格判定：成功返回 None/0；失败返回 err.Err（err.check_raise 会抛）。
                    # 旧写法只在"返回对象带 .value 且非 0"时报错，遇到未知返回类型会静默吞掉失败。
                    if err is not None:
                        err.check_raise(result, "pinmap set %s=%s failed" % (pin, func))
                    elif result is not None and int(getattr(result, "value", 0) or 0) != 0:
                        raise RuntimeError("pinmap set %s=%s failed: %s" % (pin, func, result))
                info["pinmap_ok"] = True
            except Exception as exc:
                info["pinmap_ok"] = False
                info["pinmap_error"] = str(exc)
                if strict:
                    info["error"] = "pinmap: %s" % exc
                    self._stats[axis] = False
                    self._stats["err"] += 1
                    print("[gimbal] %s 链路不可用（PINMAP_STRICT=True）| dev=%s pins=%s | %s" %
                          (axis, info["device"], _pins_text(info["pins"]), exc))
                    continue
                print("[gimbal] %s pinmap 设置失败（按 PINMAP_STRICT=False 继续打开）：%s"
                      % (axis, exc))
            # ---- 2) 回读实测引脚功能（诊断；MaixCAM2 支持直接从硬件读）----
            watch = dict(info["pins"]) or dict(NOMINAL_PINS.get(axis, {}))
            funcs = {}
            for pin in watch:
                try:
                    funcs[pin] = pinmap.get_pin_function(pin)
                except Exception as exc:
                    funcs[pin] = "?err:%s" % exc
            info["pin_funcs"] = funcs
            # ---- 3) 打开 UART ----
            try:
                self._ports[axis] = uart.UART(info["device"], info["baud"])
                info["opened"] = True
            except Exception as exc:
                info["error"] = str(exc)
                self._stats[axis] = False
                self._stats["err"] += 1
                print("[gimbal] %s 串口打开失败 | dev=%s | %s" % (axis, info["device"], exc))
        # 开机清空缓冲：UART0(U0) 上常有内核启动日志/终端回显，必须丢掉再发指令。
        if bool(getattr(cfg, "FLUSH_RX_ON_START", True)):
            self.flush()

    def flush(self, axis=None):
        """丢弃串口缓冲区里的陈旧字节（U0 上可能有内核日志/终端回显），返回丢弃字节数。"""
        axes = tuple(self._ports) if axis is None else (axis,)
        dropped = 0
        for name in axes:
            port = self._ports.get(name)
            self._rxbuf[name] = bytearray()
            if port is None:
                continue
            try:
                avail = int(port.available(0))
                if avail > 0:
                    data = port.read(min(avail, 256), 0)
                    dropped += len(bytes(data)) if data else 0
            except Exception:
                pass
        return dropped

    def health_lines(self):
        """返回两轴链路自检的可读行（启动时打印，用于现场确认串口是否真的开上了）。"""
        lines = []
        for axis in ("yaw", "pitch"):
            info = self.axis_info[axis]
            if info["opened"]:
                state = "OK"
            elif self._init_error is not None:
                state = "NO-MAIX"
            else:
                state = "FAIL"
            lines.append("[gimbal] %-5s %-4s dev=%-12s baud=%d pins=%-30s pinmap=%s -> %s%s" % (
                axis, state, info["device"], info["baud"], _pins_text(info["pins"]),
                info["pinmap_ok"], "opened" if info["opened"] else "NOT opened",
                ("  err=%s" % info["error"]) if info["error"] else ""))
            # 实测引脚功能：判断"该轴明明打开了却不通"的关键证据（例如 A28 实测不是 UART0_TX）。
            funcs = info.get("pin_funcs") or {}
            if funcs:
                lines.append("[gimbal]   └ 引脚实测: %s" %
                             "  ".join("%s=%s" % kv for kv in funcs.items()))
        return lines

    def available_axes(self):
        return [axis for axis in ("yaw", "pitch") if self.axis_info[axis]["opened"]]

    # ------------------------------------------------------------- 发送
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
                    # 明确 timeout=0；每轴每帧最多 256 字节，避免 UART0 日志洪水拖长循环。
                    data = port.read(min(available, 256), 0)
                    if data:
                        self._rxbuf[axis].extend(bytes(data))
                buf = self._rxbuf[axis]
                while len(buf) >= 4:
                    if buf[0] != self._addr(axis):
                        del buf[0]
                        continue
                    end = buf.find(bytes([CR]), 1)
                    if end < 3:
                        break
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

    def status_line(self):
        """单行链路心跳（周期日志用）：证明"是否真的发过帧 / 是否收到过回帧"。

        判读：tx=0 -> 一条指令都没发出（链路没打开或被上层拦住）；
              tx>0 rx=0 -> 发出去了但驱动器一个字节都没回（查 P_Serial/地址/波特率/接线）；
              ok>0      -> 驱动器确实在应答，链路通。
        """
        s = self._stats
        axes = []
        for axis in ("yaw", "pitch"):
            info = self.axis_info[axis]
            if not info["opened"]:
                axes.append("%s:FAIL" % axis)
            elif self._last_status[axis] is None:
                axes.append("%s:open" % axis)
            else:
                axes.append("%s:open,0x%02X" % (axis, self._last_status[axis]))
        return "[gimbal] 链路 tx=%d rx=%d ok=%d e2=%d to=%d drop=%d | %s" % (
            s["tx"], s["rx"], s["ok"], s["e2"], s["timeout"], s["err"],
            " ".join(axes))
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

标定期 main 会跳过闭环（见 ``CALIBRATE_AXIS`` 非空）。两种用法：

* **人工**：``CALIBRATE_AXIS = "yaw"``（或 ``"pitch"``）跑一次，把打印出的数值填进
  ``config.py``，再置回 ``""``，重新上电进入跟踪；
* **自动**：``CALIBRATE_AXIS = "auto"`` —— ``AutoCalibrate`` 依次标定两轴，成功后把结果
  写回**运行时** cfg 并解锁闭环，**上电一次即进入跟踪**（无需回填、无需重启）。

保持纯 Python（不 import maix），可在 PC 上用假链路回归。
"""


def _estimate_seconds(pulses, rpm, acc, pulses_per_rev):
    """复用自检的时间建模；单文件打包等 import 不可用场景退化为保守估计（曲线留 1.4 倍余量）。"""
    try:
        from vision.gimbal_self_test import estimate_move_seconds
        return estimate_move_seconds(pulses, rpm, acc, pulses_per_rev)
    except Exception:
        travel = pulses / max(1e-6, pulses_per_rev * max(1, rpm) / 60.0)
        return travel * (1.4 if int(acc) > 0 else 1.0)


class GimbalCalibrator:
    def __init__(self, cfg, link, now=0.0, axis=None):
        self.cfg = cfg
        self.link = link
        # axis 显式传入时优先于 cfg.CALIBRATE_AXIS —— AutoCalibrate 靠它逐轴切换。
        self.axis = (str(axis) if axis is not None
                     else str(getattr(cfg, "CALIBRATE_AXIS", ""))).strip().lower()
        self.pulses = int(getattr(cfg, "CALIBRATE_PULSES", 2000))
        self.need = max(2, int(getattr(cfg, "CALIBRATE_SAMPLES", 5)))
        self.samples = []
        self.base = self.out = self.back = None
        self.report = None
        self.reason = None
        self.started = float(now)
        self.deadline = float(now) + float(getattr(cfg, "CALIBRATE_SETTLE_S", 1.0))
        # ★ 视觉无效快速失败：标定每一步都要 dx/dy，画面里没有靶心时不该干等 CALIBRATE_TIMEOUT_S
        #   （现场"抬头后黑框出视野 -> 卡在等 dv"就是这么来的）。
        self._last_dv = float(now)
        self.visual_timeout_s = float(getattr(cfg, "CALIBRATE_VISUAL_TIMEOUT_S", 0.0) or 0.0)
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
        # ★ 视觉无效**快速失败**：不再等到 20s 总超时。只要连续 visual_timeout_s 收不到有效
        #   dx/dy，就立刻判失败、把控制权交还主流程 —— 这是"没靶心 / 抬头后黑框跑出视野
        #   导致校准卡住"的直接解法。
        if self.visual_timeout_s > 0 and now - self._last_dv >= self.visual_timeout_s:
            return self._fault("no_vision(%.1fs 无有效 dx/dy)" % self.visual_timeout_s)
        if now - self.started > self.limit_s:
            return self._fault("timeout_no_valid_aim(%.1fs)" % self.limit_s)
        if now < self.deadline:
            return self.state
        dv = None
        if result and result.get("aim_valid") and result.get("dx") is not None:
            dv = (float(result["dx"]), float(result["dy"]))
            self._last_dv = now          # 有有效视觉 -> 刷新，视觉超时不会误伤正常标定
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
        rpm = max(1, int(getattr(self.cfg, "CALIBRATE_RPM", 60)))
        acc = int(getattr(self.cfg, "CALIBRATE_ACC", getattr(self.cfg, "SELF_TEST_ACC", 0)))
        ppr = max(1.0, float(getattr(self.cfg, "MOTOR_PULSES_PER_REV", 3200)))
        # 与自检同一套建模（含曲线加减速；行程不足自动退化三角曲线），避免早发下一条。
        travel = _estimate_seconds(self.pulses, rpm, acc, ppr)
        settle = float(getattr(self.cfg, "CALIBRATE_SETTLE_S", 1.0))
        return travel + settle

    def _send(self, ccw):
        rpm = int(getattr(self.cfg, "CALIBRATE_RPM", 60))
        acc = int(getattr(self.cfg, "CALIBRATE_ACC", getattr(self.cfg, "SELF_TEST_ACC", 0)))
        ok = self.link.move(self.axis, ccw, self.pulses, rpm, acc)
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


class AutoCalibrate:
    """开机自动逐轴标定（``CALIBRATE_AXIS = "auto"``）：yaw -> pitch，成功后自动解锁闭环。

    与人工标定的差别：不需要看屏幕读数、不需要回填 config 再重启 —— 实测结果直接写回
    **运行时**的 cfg 对象，上层据此解除 ``GimbalAim`` 的未标定锁，上电一次即可从
    "自检" 直接进入 "跟踪"。

    安全口径：**任一轴失败就整体不解锁**（宁可不追，也不用错误参数正反馈甩头）。
    标定前提是画面里能看到靶心，否则 ``dx/dy`` 无效 -> 超时失败。
    """

    def __init__(self, cfg, link, now=0.0, axes=None):
        self.cfg = cfg
        self.link = link
        self.axes = [str(a).lower() for a in
                     (axes or getattr(cfg, "AUTO_CALIBRATE_AXES", ("yaw", "pitch")))]
        self.axis = None
        self.current = None
        self.results = {}
        self.failed = {}
        self.state = "RUNNING"
        self.reason = None
        self._started = float(now)
        self._limit_s = float(getattr(cfg, "AUTO_CALIBRATE_TIMEOUT_S", 150.0))
        self._next(float(now))

    # ------------------------------------------------------------- 接口
    @property
    def done(self):
        return self.state in ("DONE", "FAILED", "SKIPPED")

    @property
    def applied(self):
        """是否已把结果写回 cfg 且**两轴全部成功** —— 只有此时才允许解锁闭环。"""
        return bool(self.results) and not self.failed

    @property
    def deadline(self):
        """当前正在标定的那一轴的截止时刻（供上层推进时钟/测试用）。"""
        return self.current.deadline if self.current is not None else self._started

    def update(self, now, result=None):
        now = float(now)
        if self.done:
            return self.state
        if now - self._started > self._limit_s:
            return self._fail("auto_calibrate_timeout(%.0fs)" % self._limit_s)
        if self.current is None:
            return self.state
        self.current.update(now, result)
        if not self.current.done:
            return self.state
        if self.current.state == "DONE" and self.current.report:
            self.results[self.axis] = dict(self.current.report)
            print("[calib-auto] %s 标定成功" % self.axis)
        else:
            self.failed[self.axis] = self.current.reason or self.current.state
            print("[calib-auto] %s 标定失败: %s" % (self.axis, self.failed[self.axis]))
        self._next(now)
        return self.state

    def apply(self):
        """把实测值写回运行时 cfg（**不改磁盘 config.py**），并打印可粘贴片段。"""
        for axis, rep in self.results.items():
            upper = axis.upper()
            setattr(self.cfg, "PULSE_PER_PX_%s" % upper, round(float(rep["pulse_per_px"]), 2))
            setattr(self.cfg, "DIR_SIGN_%s" % upper, int(rep["dir_sign"]))
        print("[calib-auto] 已写入运行时 config（磁盘文件未改）。要固定下来请复制到 config.py：")
        for axis, rep in self.results.items():
            upper = axis.upper()
            print("[calib-auto]   PULSE_PER_PX_%s = %.2f" % (upper, rep["pulse_per_px"]))
            print("[calib-auto]   DIR_SIGN_%s = %d" % (upper, rep["dir_sign"]))
        return len(self.results)

    # ------------------------------------------------------------- 内部
    def _next(self, now):
        remaining = [a for a in self.axes if a not in self.results and a not in self.failed]
        if not remaining:
            return self._finish()
        self.axis = remaining[0]
        self.current = GimbalCalibrator(self.cfg, self.link, now, axis=self.axis)
        if self.current.done:
            # 该轴不可标（脉冲数非法等）-> 记失败后继续下一轴
            self.failed[self.axis] = self.current.reason or "unavailable"
            print("[calib-auto] %s 无法标定: %s" % (self.axis, self.failed[self.axis]))
            return self._next(now)
        idx = len(self.results) + len(self.failed) + 1
        print("[calib-auto] 开始标定 %s（第 %d/%d 轴，走 %d 脉冲，请确认云台有行程余量）" %
              (self.axis, idx, len(self.axes), self.current.pulses))
        return None

    def _finish(self):
        print("[calib-auto] ================== 自动标定汇总 ==================")
        for axis in self.axes:
            if axis in self.results:
                rep = self.results[axis]
                print("[calib-auto]   %-5s OK    PULSE_PER_PX=%7.2f  DIR_SIGN=%+d  交叉=%+.3f  回差=%.2fpx"
                      % (axis, rep["pulse_per_px"], rep["dir_sign"],
                         rep["cross"], rep["backlash_px"]))
            else:
                print("[calib-auto]   %-5s 失败  %s" % (axis, self.failed.get(axis)))
        if not self.results:
            self.state, self.reason = "FAILED", "no_axis_calibrated"
            print("[calib-auto] 没有任何轴标定成功 -> 闭环保持锁定（不会发电机指令）")
            return self.state
        self.apply()
        if self.failed:
            self.state = "DONE"
            self.reason = "partial:%s" % ",".join(sorted(self.failed))
            print("[calib-auto] 有轴未标定成功(%s) -> 闭环仍**保持锁定**，请修好后重跑"
                  % ",".join(sorted(self.failed)))
            return self.state
        self.state = "DONE"
        return self.state

    def _fail(self, reason):
        self.state, self.reason = "FAILED", str(reason)
        print("[calib-auto] 失败：%s" % self.reason)
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

_HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
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
    # 无状态单次检测必须完成整轮；连续视频走有预算的 process()/CenterTracker。
    raw = det.process(gray, exhaustive=detector is None)
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

    ⚠ 使用前必须完成开环标定（``CALIBRATE_AXIS`` 跑一次，或设 ``"auto"`` 自动标定）。
    未标定时方向/增益是占位值、可能正反馈甩头，闭环会被锁成 ``UNCALIBRATED``
    （只出视觉结果、不发指令）；确认已标定后用 ``calibrated=True`` 或 ``.unlock()`` 解锁。
    不需要电机的场景请只用 ``detect_black_frame`` / ``CenterTracker``，不要实例化本类。

    ``step(frame)`` 返回的 ``Result`` 比追踪器多两个字段：
        aim_state  str   状态机状态（FINE/COARSE/SETTLE/HOLD/FAULT…）
        cmds       dict  {"yaw": (ccw, pulses, rpm) or None, "pitch": ...}
    """

    def __init__(self, cfg=None, enable_motors=True, with_laser=True, calibrated=None):
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
            # calibrated=True：显式声明"PULSE_PER_PX_*/DIR_SIGN_* 已是实测值" -> 直接解锁闭环。
            # 不传则沿用 config.GIMBAL_CALIBRATED 的判定（见 GimbalAim.__init__）。
            if calibrated is True and self.aim.locked:
                self.aim.unlock("AimLoop(calibrated=True)")

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
            res["filtered_pulses"] = out["filtered_pulses"]
            res["stops"] = out["stops"]
            for axis in out["stops"]:
                self.link.stop(axis, now=now)
            if out["state"] == "FAULT":
                self.link.stop(now=now)
            for axis, command in out["cmds"].items():
                if command is not None:
                    ccw, pulses, rpm = command
                    if not self.link.move(axis, ccw, pulses, rpm,
                                          getattr(self.cfg, "ACC_TRACK",
                                                  getattr(self.cfg, "ACC_DEFAULT", 0)), now=now):
                        self.link.stop(now=now)
                        self.aim.fault("uart_write_failed:%s" % axis)
                        res["aim_state"] = "FAULT"
                        break
                    res["cmds"][axis] = command
        fb = self.link.poll(now)
        from vision.stepper_link import fault_reason
        fault = fault_reason(fb)
        if fault is not None:
            self.link.stop(now=now)
            self.aim.fault("link:%s" % fault)
            res["aim_state"] = "FAULT"
        return res

    def stop(self):
        if self.link is not None:
            self.link.stop()

    def close(self):
        self.stop()
'''

_MAIN_SRC = r'''
"""MaixCAM2 entry point for perspective robust black rectangle center detection."""

import os
import sys
import time

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
from vision.gimbal_calibrate import GimbalCalibrator, AutoCalibrate

BUILD_ID = "2026-10-10-fps-honest-v20"


def _preview_loop(frames, ready, closing, elapsed, max_ms, shown, slow,
                  fps, slow_ms, backoff, show_callback=None):
    """独立进程隔离 Display.show 的网络超时，避免原生绑定持有 GIL 时线程也被阻塞。

    统计量（elapsed=本帧 show 毫秒 / max_ms=历史最大 / shown=已显示帧数 / slow=慢帧计数）
    都通过共享 Value 回传主进程，用于**真实地**度量显示帧率 —— 不再只有主循环帧率。
    """
    disp = None
    try:
        if show_callback is None:
            from maix import display, image
            disp = display.Display()
            show_callback = lambda view: disp.show(image.cv2image(view, bgr=True, copy=True))
        ready.set()
        while not closing.is_set():
            try:
                view = frames.get(timeout=0.1)
            except Exception:
                continue
            started = time.perf_counter()
            try:
                show_callback(view)
            except Exception as exc:
                print("[preview] show failed: %s" % exc, flush=True)
            show_ms = (time.perf_counter() - started) * 1000.0
            elapsed.value = show_ms
            if show_ms > max_ms.value:
                max_ms.value = show_ms
            shown.value += 1
            if show_ms > slow_ms:
                slow.value += 1
            # 节拍 = 目标周期 **减去本帧 show 实际耗时**（旧版没减，实际帧率总是低于 SHOW_FPS）；
            # 慢帧只加一个很小的退避。队列已是 maxsize=1 的"忙则丢帧"，不需要长睡。
            delay = max(0.001, 1.0 / max(1, fps) - show_ms / 1000.0)
            if show_ms > slow_ms:
                delay += max(0.0, backoff)
            if closing.wait(delay):
                break
            ready.set()
    except Exception as exc:
        print("[preview] disabled: %s" % exc, flush=True)
    finally:
        ready.clear()
        if disp is not None:
            disp.close()


class AsyncPreview:
    """最多一张在途画面；忙时不画图、不排队、不等预览，退出时有界清理。"""
    def __init__(self, cfg, show_callback=None):
        import multiprocessing as mp
        # spawn 不继承 Maix/OpenCV 原生库锁，也兼容单文件版的 __mp_main__ 入口。
        ctx = mp.get_context("spawn")
        self.frames = ctx.Queue(maxsize=1)
        self.ready = ctx.Event()
        self.closing = ctx.Event()
        # lock=False：只作日志用的近似读数，避免每帧加锁；读到略旧的值无所谓。
        self.elapsed = ctx.Value("d", 0.0, lock=False)
        self.max_ms = ctx.Value("d", 0.0, lock=False)
        self.shown = ctx.Value("q", 0, lock=False)
        self.slow = ctx.Value("q", 0, lock=False)
        self.process = ctx.Process(target=_preview_loop, args=(
            self.frames, self.ready, self.closing, self.elapsed, self.max_ms,
            self.shown, self.slow, cfg.SHOW_FPS, cfg.SHOW_SLOW_MS,
            cfg.SHOW_BACKOFF_S, show_callback), daemon=True)
        self.process.start()

    def can_submit(self):
        return self.ready.is_set() and self.process.is_alive()

    def submit(self, view):
        if not self.can_submit():
            return False
        self.ready.clear()
        try:
            self.frames.put_nowait(view)
            return True
        except Exception:
            self.ready.set()
            return False

    def close(self):
        self.closing.set()
        self.process.join(timeout=0.1)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=0.1)
        self.frames.cancel_join_thread()
        self.frames.close()


class SyncPreview:
    """**主进程内**直接 `Display.show`（MaixVision 一定能看到画面），按帧率限频 + 慢帧自动降速。

    与 `AsyncPreview` 同接口，用 `Config.SHOW_ASYNC` 切换。`show()` 是阻塞调用，无法被打断，
    因此限频在主循环侧完成：`can_submit()` 只在"到点"时返回 True，`submit()` 里真正 show。
    图传变慢时自动拉长节拍（最低 `SHOW_SLOW_MIN_FPS`），保证主循环不被 0.5s 级的 show 拖死。
    """

    def __init__(self, cfg):
        from maix import display
        self._display = display.Display()
        self._base_period = 1.0 / max(1, cfg.SHOW_FPS)
        self._period = self._base_period
        self._min_period = 1.0 / max(0.1, float(getattr(cfg, "SHOW_SLOW_MIN_FPS", 1.0)))
        self._slow_ms = float(cfg.SHOW_SLOW_MS)
        self._next = 0.0
        # 与 AsyncPreview 对齐的统计量（子进程版是 mp.Value，用 _stat() 统一读取）。
        self.elapsed = 0.0
        self.max_ms = 0.0
        self.shown = 0
        self.slow = 0

    def can_submit(self):
        return time.perf_counter() >= self._next

    def submit(self, view):
        from maix import image as maix_image
        started = time.perf_counter()
        try:
            self._display.show(maix_image.cv2image(view, bgr=True, copy=True))
        except Exception as exc:
            print("[preview] show failed: %s" % exc, flush=True)
        show_ms = (time.perf_counter() - started) * 1000.0
        self.elapsed = show_ms
        if show_ms > self.max_ms:
            self.max_ms = show_ms
        self.shown += 1
        if show_ms > self._slow_ms:
            self.slow += 1
            # 慢帧：节拍翻倍（封顶到 _min_period），让主循环尽快脱身；只偶发打印，避免刷屏。
            self._period = min(self._period * 2.0, self._min_period)
            if self.slow == 1 or self.slow % 20 == 0:
                print("[preview] ⚠ show=%.0fms 偏慢 -> 显示节拍自动降到 %.1ffps（MaixVision 图传瓶颈）"
                      "；需要主循环完全不受显示影响可设 SHOW_ASYNC=True，不需要预览可设 SHOW=False"
                      % (show_ms, 1.0 / self._period), flush=True)
        elif show_ms < self._slow_ms * 0.8:
            # 恢复：图传回到"明显不慢"（< 0.8×阈值）即较快回落到目标帧率。
            # 旧阈值 0.5×(=40ms) 在真机 MaixVision 上几乎不满足 → 一旦降速就长期钉在最低帧率
            # （现场"帧率过低"的来源之一）。0.7 的步进也让恢复更快。
            self._period = max(self._base_period, self._period * 0.7)
        # 下一次显示 = 现在 + (目标周期 − 本次 show 耗时)，实际帧率才真正贴近目标。
        self._next = time.perf_counter() + max(0.0, self._period - show_ms / 1000.0)
        return True

    def close(self):
        try:
            self._display.close()
        except Exception:
            pass


def _stat(preview, name):
    """统一读取同步/异步预览的统计量（同步是 float，异步是 mp.Value）。"""
    value = getattr(preview, name, 0)
    return value.value if hasattr(value, "value") else value


def _to_gray(img, image_module):
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
    if uart is None:
        return
    if result.get("valid"):
        payload = "@[%d,%d]%%\r\n" % (int(round(result["u"])), int(round(result["v"])))
    else:
        payload = "@[invalid]%%\r\n"
    uart.write(payload.encode("ascii"))


def main():
    # Imports stay inside main so the geometry module remains PC-testable.
    from maix import app, camera, image

    # 小图上的多线程调度常常比运算本身更贵，也避免 ARM 多核线程争用。
    cv2.setNumThreads(Config.OPENCV_THREADS)

    # ★ 显示：默认"主进程内"路径（MaixVision 一定能看到画面）；SHOW_ASYNC=True 才用子进程隔离。
    preview = None
    preview_mode = "off"
    if Config.SHOW:
        try:
            if getattr(Config, "SHOW_ASYNC", False):
                preview = AsyncPreview(Config)
                preview_mode = "async-process"
            else:
                preview = SyncPreview(Config)
                preview_mode = "sync-inprocess"
        except Exception as exc:
            preview = None
            preview_mode = "off(init-failed: %s)" % exc
            print("[preview] disabled after init failure: %s" % exc)

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
    self_test = None
    calibrator = None
    auto_cal = None
    auto_cal_done = False
    if Config.ENABLE_GIMBAL:
        try:
            from vision.stepper_link import StepperLink, axis_cfg, fault_reason
            link = StepperLink(Config)
            # 先亮出两条串口的真实状态：设备路径 / 引脚 / pinmap 结果 / 是否打开。
            for line in link.health_lines():
                print(line)
            axes = link.available_axes()
            if not axes:
                print("[gimbal] 两条串口都打不开 -> 云台停用（视觉照常运行）；"
                      "请核对接线：yaw=U0(U0T/U0R,A28/A29)、pitch=U2(U2T/U2R,B0/B1)，"
                      "TX↔RX 交叉、三路 GND 共地")
            enabled = link.enable()
            print("[gimbal] 使能帧 0xF3 已发: %s（True=两轴 write 未报错；驱动器 Response≠None 才会有回执帧）"
                  % enabled)
            aim = GimbalAim(Config)
            imu_info = probe_imu()
            if imu_info["available"]:
                print("[imu] sample via %s: %s (gyro cannot define absolute yaw zero)" %
                      (imu_info["method"], str(imu_info["sample"])))
            else:
                print("[imu] unavailable: %s" % imu_info["error"])
            axis_sel = str(getattr(Config, "CALIBRATE_AXIS", "")).strip().lower()
            calibrator = GimbalCalibrator(Config, link, time.perf_counter())
            if calibrator.done:
                calibrator = None
                # ★ 上电自检与视觉/标定**完全解耦**：只要本次不是"人工标定"模式，
                #   上电后立刻按固定脉冲走一遍（**pitch 上→下、yaw 左→右**，各回到起始位姿）。
                #   自检只用链路 + 时间，**不需要靶纸、不等黑框识别**；4 步跑完才轮到
                #   自动标定 / 追踪（见主循环：自检分支优先）。
                self_test = GimbalSelfTest(Config, link, time.perf_counter())
                if self_test.state == "RUNNING":
                    axes_txt = "+".join(sorted({s[0] for s in self_test.sequence}))
                    print("[gimbal] 上电自检 %d 步，参与轴=%s（固定脉冲，不等黑框/靶纸）"
                          % (self_test.total, axes_txt))
                elif self_test.state == "SKIPPED":
                    print("[gimbal] 自检已跳过(%s) -> 上电不会看到电机动作。"
                          "要么填 YAW/PITCH_PULSES_PER_DEG、要么给 GIMBAL_GEAR_RATIO，"
                          "要么设 SELF_TEST_PULSES>0 用固定脉冲验证链路"
                          % self_test.reason)
                if axis_sel == "auto":
                    print("[gimbal] 自检完成后自动标定 %s（标定期才需要画面里有靶心，"
                          "否则按 AUTO_CALIBRATE_TIMEOUT_S 超时失败）"
                          % "+".join(getattr(Config, "AUTO_CALIBRATE_AXES", ("yaw", "pitch"))))
            else:
                print("[gimbal] calibration mode axis=%s（本次只人工标定：不自检、不闭环）"
                      % calibrator.axis)
            print("[gimbal] axes yaw=%s@%s pitch=%s@%s addr=%d/%d" %
                  (axis_cfg(Config, "yaw")[0], axis_cfg(Config, "yaw")[2] or "默认复用",
                   axis_cfg(Config, "pitch")[0], axis_cfg(Config, "pitch")[2] or "默认复用",
                   Config.ADDR_YAW, Config.ADDR_PITCH))
        except Exception as exc:
            print("[gimbal] disabled after init failure: %s" % exc)
    detector = RectangleCenterDetector(Config)
    laser_det = LaserSpotDetector(Config) if Config.ENABLE_LASER else None
    # fixed 模式（光斑位置写死）：开机取一次常量，之后全程复用 —— 不做任何图像检测，
    # 每帧只剩"黑框中心 − 该常量"两次减法。detect 模式才需要每帧调用 process()。
    laser_static = (laser_det.mode == "fixed") if laser_det is not None else False
    aim_use_raw = bool(getattr(Config, "AIM_USE_RAW", False))
    last = time.perf_counter()
    last_log = last
    lens_enabled = Config.LENS_CORR
    totals = dict(read=0.0, lens=0.0, bridge=0.0, detect=0.0, laser=0.0,
                  uart=0.0, rx=0.0, aim=0.0, show=0.0, loop=0.0)
    # 每个阶段的本窗口**最大值**：均值会掩盖"某一帧突然卡 500ms"这类卡顿。
    maxima = {k: 0.0 for k in totals}
    shown_prev = 0
    # ★ 屏幕叠加用的**实测显示帧率**：与主循环速率分开统计，0.5s 短窗口刷新。
    #   旧版 putText 直接画主循环 fps —— 由于多数帧只算不画，该值被快帧拉高（虚高到上百），
    #   而眼睛看到的刷新率可能只有 1~2fps。此处改画"窗口内真正 show 成功的帧数 / 时间"。
    disp_fps = 0.0
    disp_t = last
    disp_shown0 = 0
    count, searches = 0, 0
    laser_state = laser_det.static_state() if laser_static else None
    laser_ms, laser_calls, laser_hits = 0.0, 0, 0
    fps = 0.0
    mean_dt = None
    print("[rect-center] %s search_width=%d display_width=%d display_fps=%d" %
          (BUILD_ID, Config.SEARCH_WIDTH, Config.SHOW_WIDTH, Config.SHOW_FPS))
    print("[preview] SHOW=%s mode=%s —— 看不到画面时先确认这里是 sync-inprocess；"
          "sync 路径在 MaixVision 一定能看到，async 依赖子进程建 Display"
          % (Config.SHOW, preview_mode))
    print("[rect-center] 四边窄带跟踪 / 失锁后低分辨率搜索 / 72 点逆投影")
    # ★ 镜头畸变校正必须显式可见：它是**软件重映射**，会把整幅画面（含预览）非线性地改写，
    #   未按实际镜头调好 strength 时，看起来就是"画面畸变/边缘拉伸"。检测与显示喂的是**同一张**校正图。
    print("[lens] LENS_CORR=%s strength=%.2f —— %s" %
          (Config.LENS_CORR, Config.LENS_STRENGTH,
           "已对每帧做软件畸变校正（画面是矫正后的形态；若看起来畸变，置 False 或调小 strength）"
           if Config.LENS_CORR else
           "未做畸变校正，画面/检测均为原始帧（离线回归基线用的就是这一档）"))
    print("[aim] 闭环口径=%s（raw_u/raw_v）离群拒收=%spx 死区=%.1fpx 限幅=%dp" %
          ("raw" if aim_use_raw else "smoothed", Config.JUMP_REJECT_PX,
            Config.DEADBAND_PX, Config.MAX_STEP_PULSES))
    print("[aim] 运动量低通 tau=%.2fs 增长限速=%.0fp/s 曲线acc=%d 等待运动=%s 余量=%.2fs" %
          (Config.AIM_OUTPUT_LPF_TAU_S, Config.AIM_OUTPUT_SLEW_PPS, Config.ACC_TRACK,
           Config.AIM_WAIT_MOTION, Config.AIM_MOTION_MARGIN_S))
    if laser_det is not None:
        if laser_static:
            print("[laser] mode=fixed 光斑位置写死 (%d,%d) —— 不做图像检测，每帧只算黑框中心−该常量" %
                  (Config.LASER_U_NOMINAL, Config.LASER_V_NOMINAL))
        else:
            print("[laser] mode=detect 标称位置 (%d,%d) ROI±%d 限频 %.2fs 判据 peak>=max(%.0f, bg+%.0f) 面积<=%d" %
                  (Config.LASER_U_NOMINAL, Config.LASER_V_NOMINAL,
                   Config.LASER_ROI_RADIUS, Config.LASER_INTERVAL_S, Config.LASER_MIN_PEAK,
                   Config.LASER_MIN_CONTRAST, Config.LASER_MAX_PIXELS))
    while not app.need_exit():
        start = time.perf_counter()
        img = cam.read()
        if img is None:
            continue
        after_read = time.perf_counter()
        if lens_enabled:
            try:
                corrected = img.lens_corr(strength=Config.LENS_STRENGTH)
                if corrected is not None:
                    img = corrected
            except Exception as exc:
                lens_enabled = False
                print("[lens] 本次会话停用不支持的 lens_corr: %s" % exc)
        after_lens = time.perf_counter()
        gray = _to_gray(img, image)
        after_bridge = time.perf_counter()
        result = detector.process(gray)
        after_detect = time.perf_counter()
        if laser_det is not None and not laser_static:
            # detect 模式：内部限频，未到 LASER_INTERVAL_S 时不重算，直接沿用上次光斑位置。
            laser_state = laser_det.process(gray, after_detect)
            if laser_state.get("updated"):
                laser_calls += 1
                laser_ms += laser_state["ms"]
                laser_hits += 1 if laser_state["found"] else 0
        dv = None
        if laser_state is not None:
            # fixed 模式走的就是这里：每帧唯一的光斑相关运算 = 两次减法。
            dv = deviation(laser_state, result, use_raw=aim_use_raw)
            if dv is not None:
                # 靶心-光斑偏差；dx/dy 既是串口字段也是闭环输入（符号见 deviation 注释）。
                result["laser_u"], result["laser_v"] = laser_state["u"], laser_state["v"]
                result["dx"], result["dy"] = dv
        after_laser = time.perf_counter()
        result["aim_valid"] = bool(dv is not None)
        _send_uart(uart, result)
        after_uart = time.perf_counter()
        # 下行：非阻塞读两条串口，处理 E2 自动补使能 / ACK 超时 / E2 洪水。
        rx_started = time.perf_counter()
        fb = link.poll(after_uart) if link is not None else None
        rx_elapsed = time.perf_counter() - rx_started
        link_fault = fault_reason(fb) if link is not None else None
        aim_started = time.perf_counter()
        if link_fault is not None and aim is not None and aim.state != "FAULT":
            print("[gimbal] link fault: %s -> 立即停止并进入 FAULT" % link_fault)
            link.stop(now=after_uart)
            aim.fault("link:%s" % link_fault)
        if calibrator is not None:
            if not calibrator.done and aim is not None and aim.state != "FAULT":
                calibrator.update(after_uart, result)
        elif self_test is not None and not self_test.done:
            # ★ 上电自检优先：固定脉冲 4 步（pitch 上→下、yaw 左→右）先走完，
            #   期间不发任何标定/闭环指令；**不依赖视觉结果**（无黑框也照走）。
            #   唯一例外：链路已被判 FAULT 时不再下发任何指令。
            if aim is not None and aim.state != "FAULT":
                self_test.update(after_uart)
        elif self_test is not None and self_test.state == "FAULT":
            # 自检判定链路故障（所有参与轴都写不出去）-> 保持静默：不标定、不追踪。
            pass
        elif (auto_cal is None and aim is not None and aim.state != "FAULT"
              and str(getattr(Config, "CALIBRATE_AXIS", "")).strip().lower() == "auto"):
            # 自检结束后才真正开始自动标定：时间窗口从"这一刻"算起，
            # 不被上电自检的机械动作耗时挤占（否则自检慢时会吞掉标定预算）。
            auto_cal = AutoCalibrate(Config, link, after_uart)
            print("[gimbal] 上电自检完成 -> 开始自动标定 %s（标定期不闭环，需画面里有靶心）"
                  % "+".join(auto_cal.axes))
        elif auto_cal is not None and not auto_cal.done:
            # 自动标定期间只标定、不闭环（与人工标定同一口径）。
            if aim is not None and aim.state != "FAULT":
                auto_cal.update(after_uart, result)
        elif aim is not None and link is not None and aim.state != "FAULT":
            if auto_cal is not None and not auto_cal_done:
                # 自动标定刚结束：只有两轴全部标定成功才解锁闭环（否则保持锁定）。
                auto_cal_done = True
                if auto_cal.applied and aim.unlock("auto-calibrate"):
                    print("[gimbal] 自动标定完成 -> 闭环已解锁，开始跟踪")
                else:
                    print("[gimbal] 自动标定未全部成功(%s) -> 闭环保持锁定，不会发电机指令"
                          % (auto_cal.reason or auto_cal.state))
            aim_result = aim.update(result, now=after_uart)
            for axis in aim_result["stops"]:
                link.stop(axis, now=after_uart)
            if aim_result["state"] == "FAULT":
                link.stop(now=after_uart)
            for axis, command in aim_result["cmds"].items():
                if command is not None:
                    ccw, pulses, rpm = command
                    if not link.move(axis, ccw, pulses, rpm, Config.ACC_TRACK, now=after_uart):
                        link.stop(now=after_uart)
                        aim.fault("uart_write_failed:%s" % axis)
                        break
        aim_elapsed = time.perf_counter() - aim_started
        after_aim = time.perf_counter()
        if preview is not None and preview.can_submit():
            # 屏幕上的 FPS = **实测显示帧率**（0.5s 窗口）；loop = 主循环速率，二者分开标注，
            # 不再把"主循环跑得快"当成"画面刷新快"。
            disp_now = time.perf_counter()
            if disp_now - disp_t >= 0.5:
                cur_shown = _stat(preview, "shown")
                disp_fps = (cur_shown - disp_shown0) / max(1e-6, disp_now - disp_t)
                disp_shown0, disp_t = cur_shown, disp_now
            view = draw_result(gray, result, Config.SHOW_WIDTH,
                               top_offset=PANEL_H if laser_state is not None else 0)
            if laser_state is not None:
                draw_laser_overlay(view, laser_state, result,
                                   min(1.0, float(Config.SHOW_WIDTH) / gray.shape[1]),
                                   use_raw=aim_use_raw)
            cv2.putText(view, "FPS %.1f loop %.0f" % (disp_fps, fps),
                        (12, view.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            preview.submit(view)
        after_show = time.perf_counter()
        now = after_show
        dt = now - last
        last = now
        if dt > 1e-4:
            mean_dt = dt if mean_dt is None else 0.9 * mean_dt + 0.1 * dt
            fps = 1.0 / mean_dt
        phases = (after_read - start, after_lens - after_read,
                  after_bridge - after_lens, after_detect - after_bridge,
                  after_laser - after_detect, after_uart - after_laser,
                  rx_elapsed, aim_elapsed, after_show - after_aim, now - start)
        for key, elapsed in zip(totals, phases):
            ms = elapsed * 1000.0
            totals[key] += ms
            if ms > maxima[key]:
                maxima[key] = ms
        count += 1
        searches += result.get("search_count", 0)
        if now - last_log > Config.LOG_INTERVAL:
            window = max(1e-6, now - last_log)
            window_fps = count / window
            last_log = now
            laser_avg = laser_ms / laser_calls if laser_calls else 0.0
            # 显示帧率是**实测**的（真正 show 成功的帧数 / 窗口），与主循环帧率分开报。
            shown_now = _stat(preview, "shown") if preview is not None else 0
            show_fps = (shown_now - shown_prev) / window
            shown_prev = shown_now
            print("[perf] %s valid=%s FPS=%.1f stage=%s frames=%d searches=%d "
                  "read=%.1f lens=%.1f bridge=%.1f detect=%.1f laser=%.1f "
                  "uart=%.1f rx=%.1f aim=%.1f show=%.1f loop=%.1f ms | laser_call=%.2fms n=%d hit=%d" %
                  ((BUILD_ID, result["valid"], window_fps, result["stage"], count, searches) +
                   tuple(totals[k] / count for k in totals) +
                   (laser_avg, laser_calls, laser_hits)))
            print("[perf-max] loop=%.1f read=%.1f lens=%.1f detect=%.1f show=%.1f ms "
                  "（窗口内单帧最大值；显著高于均值即为卡顿点）" %
                  (maxima["loop"], maxima["read"], maxima["lens"], maxima["detect"], maxima["show"]))
            if aim is not None:
                print(aim.status_line())
            if link is not None:
                print(link.status_line())
            if preview is not None:
                slow = _stat(preview, "slow")
                print("[preview] 实测显示 FPS=%.1f shown=%d show_last=%.1fms show_max=%.1fms "
                      "slow=%d mode=%s" %
                      (show_fps, shown_now, _stat(preview, "elapsed"),
                       _stat(preview, "max_ms"), slow, preview_mode))
                if getattr(Config, "SHOW_STALL_LIMIT", 0) and slow >= Config.SHOW_STALL_LIMIT:
                    print("[preview] ⚠ 显示通道持续缓慢（show 累计 %d 次 > %.0fms）："
                          "预览卡顿来自 MaixVision 图传链路，与本工程算法无关。"
                          "正式追踪请设 SHOW=False（或再降 SHOW_WIDTH/SHOW_FPS）。"
                          % (slow, Config.SHOW_SLOW_MS))
            count, searches = 0, 0
            laser_ms, laser_calls, laser_hits = 0.0, 0, 0
            for key in totals:
                totals[key] = 0.0
                maxima[key] = 0.0
    if link is not None:
        try:
            link.stop()
        except Exception as exc:
            print("[gimbal] stop on exit failed: %s" % exc)
    if preview is not None:
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
