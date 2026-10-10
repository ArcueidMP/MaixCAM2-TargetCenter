# B 轨云台调参手册（像素 → 角度 宏定义版）

> 面向现场调参：**追踪太猛/太慢、超调、停下瞬间激光偏一下**，改这一页就够了。
> 参数全在 `config.py`，**主入口是「★ 追踪手感」那一段（第 61–89 行）**。
> 对应 `BUILD_ID = 2026-10-10-smooth-track-v15`。旧表行号仅供参考，以参数名搜索为准。

## v15 运动量低通与防超调

滤波作用在最终的带符号运动量上，原始误差仍用于死区、方向和发散判断。
运动量变大时缓慢跟随，变小时立即缩小；丢靶、到达死区、换向时清掉滤波记忆，
并向估计仍在运动的轴发送停止帧。重新捕获从小步起动，避免沿用上次的大运动量。

| 参数 | 当前 | 调节建议 |
|---|---|---|
| `AIM_OUTPUT_LPF_TAU_S` | 0.25s | 突变仍较猛可加到 0.35s；跟手偏慢可降到 0.15s；0 关闭低通 |
| `AIM_OUTPUT_SLEW_PPS` | 120 | 单次运动量每秒最多增长 120 脉冲；更柔可降到 80；0 关闭增长限速 |
| `MAX_STEP_PULSES` | 60 | 大偏差仍容易越过停止点可降到 40 |
| `ACC_TRACK` | 80 | 驱动器曲线更柔；非 0，越小加减速越慢 |
| `AIM_WAIT_MOTION` | True | 等待估算运动结束再发送下一条，建议保持开启 |
| `AIM_MOTION_MARGIN_S` | 0.03s | 机构比模型慢可增加到 0.05～0.10s |

正常追踪是小步曲线运动；死区/换向/丢靶时的取消帧是 `0xFE` 立即停止，不属于柔性减速。
运动时间来自软件估算，未使用驱动器到位反馈。一次只调一个参数，上板看实际过冲与跟手速度。
如果小步也持续沿错误方向越追越偏，应重新核对 `DIR_SIGN_*`，低通不能修正控制方向。

---

## 0. 一分钟上手

只改这 3 处（`config.py` 第 66–67、74–75 行）：

| 你想要的效果 | 改哪个 | 往哪改 |
|---|---|---|
| 动作更柔和、别冲过头 | `DEG_PER_PX_YAW` / `DEG_PER_PX_PITCH` | 调**小**（0.30 → 0.20） |
| 跟得更快、别迟钝 | `DEG_PER_PX_YAW` / `DEG_PER_PX_PITCH` | 调**大**（0.30 → 0.45） |
| 仍偶发过冲 | `GAIN_YAW` / `GAIN_PITCH` | 调**小**（0.3 → 0.2） |

改完**必须**跑（B 轨契约，否则板子跑的还是旧代码）：

```bash
python tools/build_single.py
python tools/check_app.py --manifest
```

---

## 0.5 上电时序与「防阻塞」设计（2026-10-10 重要变更）

```
上电 → 使能 → 自检 4 步（pitch 上→下、yaw 左→右）→ 直接进入追踪
                                    ↑
                      不自动标定（避免被"看不到靶心"卡住）
```

### 阻塞是怎么来的

标定（`GimbalCalibrator`）每一步都要 `aim_valid` 且 `dx/dy` 有效才推进
（`vision/gimbal_calibrate.py` 中 `if dv is None: return self.state`）。于是：

- 画面里没有靶纸 → 卡在 `BASELINE`；
- 标定要驱动云台走 N 脉冲，**抬头后黑框跑出视野** → 卡在 `MEASURE_OUT`。

两条路都只能等 `CALIBRATE_TIMEOUT_S = 20s` 才失败 —— 现场表现就是"校准卡住不动"。

### 两条解法（均已落地）

| 措施 | 参数 / 位置 | 作用 |
|---|---|---|
| **视觉无效快速失败** | `CALIBRATE_VISUAL_TIMEOUT_S = 3.0` | 连续 3s 收不到有效 `dx/dy` 就判 `no_vision` 失败，**不再干等 20s** |
| **默认不自动标定** | `CALIBRATE_AXIS = ""` | 直接用手动宏换算，跳过会卡住的环节 |
| 直接解锁闭环 | `GIMBAL_CALIBRATED = True` | 自检完即进入追踪 |
| 自检**防死锁**超时 | `SELF_TEST_TIMEOUT_S = 10.0` | 只兜底"某步迟迟不返回"，**不是性能目标**（自检正常几秒跑完） |
| 发散看护 | `DIVERGE_GUARD = True` | 持续发指令却不收敛 → 自动 FAULT 停机，防方向反了冲向限位 |

> 想重新标定时把 `CALIBRATE_AXIS` 改回 `"auto"`（或 `"yaw"`/`"pitch"`）：即使画面里没有靶心，
> 也会在 3s 内失败并**把控制权交还主流程**，不会再卡住。

## 1. 换算链：像素 → 角度 → 脉冲

```
单帧脉冲 = |误差px| × DEG_PER_PX_x × PULSES_PER_DEG_x × GAIN_x
                       └──── ① 每px多少度 ────┘ └── ② 每度多少脉冲 ──┘ └ ③ 单帧修正比例 ┘
           （结果再受限幅、运动量低通、增长限速，等待上一段运动估算完成后发送）

误差px = 靶心(u,v) − 光斑(u,v)
```

对应代码位置：

| 环节 | 文件 | 行 | 原代码 |
|---|---|---|---|
| ①② 换算系数 | `vision/gimbal_aim.py` | **161** | `def _pulse_per_px(self, axis):` |
| ③ 增益（含粗/精切换） | `vision/gimbal_aim.py` | **153** | `def _gain(self, axis, error):` |
| **运动量计算与滤波** | `vision/gimbal_aim.py` | 搜索函数名 | `_axis_command()` → `_filter_output()` → 曲线运动时间等待 |
| 转速选择 | `vision/gimbal_aim.py` | **178** | `def _rpm(self):` |
| 死区 | `vision/gimbal_aim.py` | **139** | `def _deadband(self):` |
| 离群拒收门控 | `vision/gimbal_aim.py` | **183** | `def _axis_command(self, axis, error, now):` |
| **实际发帧（含曲线档位）** | `main.py` | **253** | `link.move(axis, ccw, pulses, rpm, Config.ACC_TRACK, now=after_uart)` |

---

## 2. 参数总表（含原代码位置）

### 2.1 ★ 追踪手感 —— 现场只改这一段（`config.py` 61–89 行）

| 参数 | 当前值 | 行 | 含义 / 调法 |
|---|---|---|---|
| `DEG_PER_PX_YAW` | **0.30** | 66 | **图像每 1px 偏差 → yaw 转多少度**。调小=柔和，调大=更跟手 |
| `DEG_PER_PX_PITCH` | **0.30** | 67 | 同上，pitch 轴 |
| `PULSES_PER_DEG_YAW` | 8.89 | 70 | **每度对应多少脉冲** = 3200(脉冲/圈) × 减速比 ÷ 360；无减速机构 = 8.89 |
| `PULSES_PER_DEG_PITCH` | 8.89 | 71 | 同上，pitch 轴 |
| `GAIN_YAW` | **0.3** | 74 | 单帧只修正误差的这个比例（必须 ≤1；0.2–0.4 是稳妥区） |
| `GAIN_PITCH` | **0.3** | 75 | 同上，pitch 轴 |
| `RPM_FINE` | **60** | 78 | 精调段转速。**调小 = 每帧走得更慢更稳** |
| `RPM_COARSE` | 150 | 79 | 粗调段（误差 > `COARSE_PX`）转速 |
| `ACC_TRACK` | 80 | 搜索参数名 | 曲线加减速档位。**必须非 0**（0 = 瞬时启停，激光会晃开）；越小越柔 |
| `DEADBAND_PX` | 3.0 | 83 | 死区。应 ≥ 1.5×实测回差，否则在齿隙里来回抖 |
| `MAX_STEP_PULSES` | **60** | 搜索参数名 | 单帧脉冲上限。调小 = 单帧最多走这么多，防猛甩 |
| `COMMAND_INTERVAL_S` | 0.10 | 85 | 发帧间隔。调大 = 更慢更稳 |
| `PULSE_PER_PX_YAW` / `PITCH` | 0.0 | 88–89 | 旧口径 / 标定写回值。**仅当上面 `DEG_PER_PX_x = 0` 时才生效** |

### 2.2 粗/精分段（`config.py` 136–141 行）

| 参数 | 当前值 | 行 | 含义 |
|---|---|---|---|
| `COARSE_PX` | 24.0 | 140 | 误差大于此值走"粗调"（`GAIN_COARSE` + `RPM_COARSE`） |
| `GAIN_COARSE` | **0.5** | 141 | 粗调增益（原 0.9 → 0.5，防大误差冲过头） |

### 2.3 方向与电机顺序（`config.py` 36–60、99–104 行）

| 项 | 当前值 | 行 | 说明 |
|---|---|---|---|
| `YAW_DEV` | `/dev/ttyS2` | 45 | **实机实测**：yaw 电机挂在 U2（ID=2） |
| `YAW_PINS` | `{"B0":"UART2_TX","B1":"UART2_RX"}` | 46 | U2 需 pinmap |
| `PITCH_DEV` | `/dev/ttyS0` | 51 | **实机实测**：pitch 电机挂在 U0（ID=1） |
| `PITCH_PINS` | `{"A28":"UART0_TX","A29":"UART0_RX"}` | 52 | U0 **必须显式 pinmap**，走空表会静默不通 |
| `ADDR_YAW` / `ADDR_PITCH` | 2 / 1 | 56 | 地址跟着**电机**走，不是跟着端口 |
| `DIR_SIGN_YAW` / `DIR_SIGN_PITCH` | **-1** | 63–64 | 闭环方向（按几何推断，未经标定）。**反了会正反馈发散**；上电后"越追越偏"就改成 **+1** |
| `YAW_POSITIVE_CCW` | True | 103 | yaw 正方向（左）= CCW；CW = 右 |
| `PITCH_POSITIVE_CCW` | True | 104 | pitch 正方向（上/仰）= CCW；CW = 下 |

### 2.4 上电自检（`config.py` 约 105–127 行）

| 参数 | 当前值 | 约行 | 说明 |
|---|---|---|---|
| `SELF_TEST_RPM` | 60 | 114 | 自检转速（慢） |
| `SELF_TEST_ACC` | 60 | 115 | 自检曲线档位（非 0 = 走曲线；越小越柔） |
| `SELF_TEST_SETTLE_S` | 0.3 | 116 | 每步后停留时间 |
| `SELF_TEST_START_DELAY_S` | 0.3 | 118 | 上电后延迟多久开始自检 |
| `SELF_TEST_TIMEOUT_S` | **10.0** | 121 | **防死锁**超时（不是性能目标）：某步迟迟不返回时强制结束，0 = 关闭 |
| `SELF_TEST_PULSES` | **400** | 126 | 自检幅度（固定脉冲）。**不依赖靶子、不依赖标定**；置 0 才回退角度模式 |
| `MOTOR_PULSES_PER_REV` | 3200 | 127 | 16 细分下电机每圈脉冲 |

标定的防阻塞参数（`config.py` 约 200–210 行）：

| 参数 | 当前值 | 说明 |
|---|---|---|
| `CALIBRATE_VISUAL_TIMEOUT_S` | **3.0** | ★ 连续这么久收不到有效 `dx/dy` 就判 `no_vision` 快速失败（0 = 关闭） |
| `CALIBRATE_TIMEOUT_S` | 20.0 | 标定整体超时（视觉快速失败生效时一般用不到） |
| `CALIBRATE_PULSES` | 400 | 标定行程；太大容易把靶心推出视野 |
| `CALIBRATE_RPM` / `CALIBRATE_ACC` | 60 / 60 | 标定动作（同样慢 + 曲线） |

> 自检顺序固定为 **pitch 上转 → 下转 → yaw 左转 → 右转**（各两段等长，回到起始位姿），
> 由 `vision/gimbal_self_test.py` 的 `_prepare()` 装入（第 103–113 行）。

### 2.5 安全闸与口径（`config.py` 约 138–152 行）

| 参数 | 当前值 | 行 | 说明 |
|---|---|---|---|
| `AIM_USE_RAW` | True | ~139 | 闭环用未平滑的 raw 值（省掉 ≈28ms 滞后） |
| `AIM_REQUIRE_CALIBRATION` | True | ~142 | 未标定禁止发电机指令 |
| `GIMBAL_CALIBRATED` | **True** | 143 | 当前视为"参数已就绪"→ **上电自检完直接追踪**；想改回"必须标定后才追"就置 False |
| `DIVERGE_GUARD` | **True** | 146 | ★ 发散看护：持续发指令却始终不收敛 → 自动 FAULT 停机 |
| `DIVERGE_FRAMES` / `DIVERGE_RATIO` | 30 / 1.5 | 147–148 | 观察窗帧数 / |误差| 涨到窗口最小值多少倍即判发散 |
| `JUMP_REJECT_PX` | 400 | ~151 | 单帧离群拒收（防误检甩头） |
| `JUMP_CONFIRM_FRAMES` | 3 | ~152 | 连续这么多帧才采信离群值 |

---

## 3. 常见现象怎么调

| 现象 | 先动这个 | 再动这些 |
|---|---|---|
| 转得太猛 / 冲过头 / 来回摆 | `DEG_PER_PX_*` ↓（0.30 → 0.20） | `GAIN_*` ↓（0.3 → 0.2）、`MAX_STEP_PULSES` ↓ |
| 停下瞬间激光偏一下 | 确认 `ACC_TRACK = 80`（非 0） | `RPM_FINE` ↓（60 → 40） |
| 跟不上移动的靶心 | `DEG_PER_PX_*` ↑ | `RPM_FINE` ↑、`GAIN_*` ↑（**别超 0.5**） |
| 静止时在靶心附近小抖 | `DEADBAND_PX` ↑（3.0 → 4.5） | 用标定日志的"回差估计"×1.5 作为下限 |
| 只在大误差时冲过头 | `GAIN_COARSE` ↓、`COARSE_PX` ↑ | `RPM_COARSE` ↓ |
| 完全不动 | 看日志是否 `state=UNCALIBRATED` | 标定没成功 → 闭环锁定（**有意设计**） |

> ⚠ **所有增益必须 ≤ 1**，否则增量闭环不收敛（误差会越修越大 → 正反馈甩头）。

---

## 4. 手动宏 vs 标定值

- **默认走手动宏**：只要 `DEG_PER_PX_YAW/PITCH > 0`，`PULSE_PER_PX_*` 就被忽略 —— 也就是
  **你手改的宏永远优先**（这是本轮的刻意设计，方便现场调手感）。
- 跑过 `CALIBRATE_AXIS`（当前默认为 `"auto"`）后，日志会打印实测脉冲/像素。想把它换算成宏：

```
DEG_PER_PX_x = PULSE_PER_PX_x ÷ PULSES_PER_DEG_x
```

  例：实测 `PULSE_PER_PX_YAW = 2.67`，`PULSES_PER_DEG_YAW = 8.89` → `DEG_PER_PX_YAW = 0.30`。
- 想让标定值**直接生效**（跳过宏）：把 `DEG_PER_PX_YAW/PITCH` 都置 0。

判定逻辑在 `vision/gimbal_aim.py` 第 161–176 行（`_pulse_per_px`）。

---

## 5. 改完必做（B 轨契约）

```bash
"D:/Python311/python.exe" -B tools/build_single.py            # 重生成单文件版
"D:/Python311/python.exe" -B tools/build_single.py --check    # 确认已同步
"D:/Python311/python.exe" -B tools/check_app.py --manifest     # 门禁 C1..C10（C8 核 BUILD_ID）
"D:/Python311/python.exe" -B tools/test_gimbal_aim.py          # 云台回归（含手感宏/曲线断言）
```

⚠ **`main.py` 会在 MaixVision 烧录/打开工程时被改写**（实测多次退回旧版）。若发现
"上电不再自检"或"追踪不再柔和"，八成是接线被覆盖了，跑：

```bash
"D:/Python311/python.exe" -B tools/apply_gimbal_track_patch.py --check   # 看哪几处缺失
"D:/Python311/python.exe" -B tools/apply_gimbal_track_patch.py           # 一键补回 + 同步 BUILD_ID
```

---

## 6. 上电预期日志（对照检查）

```
[gimbal] 引脚实测: B0=UART2_TX   B1=UART2_RX        <- yaw   在 U2 (ID=2)
[gimbal] 引脚实测: A28=UART0_TX  A29=UART0_RX       <- pitch 在 U0 (ID=1)
[gimbal] 上电自检 4 步，参与轴=pitch+yaw（固定脉冲，不等黑框/靶纸）
[self-test] pitch CCW 400p rpm=60 acc=60 step=1/4 预计0.84s
[self-test] pitch CW  400p ... step=2/4
[self-test] yaw   CCW 400p ... step=3/4
[self-test] yaw   CW  400p ... step=4/4
[self-test] complete; returned to startup pose     <- 约 3~4s；若见"硬超时 10.0s"说明某步卡住
[aim] state=FINE dx=+3.2 dy=-1.1 yaw=CCW/12p pit=CW/4p lost=0   <- 自检完直接进入追踪
```

判读：

- `state=SETTLE` 长期保持 = 已到位（正常）；
- 在 `FINE` 与 `SETTLE` 之间反复横跳 = 增益偏大或死区偏小；
- `state=UNCALIBRATED` = 标定未成功，闭环锁定（不会发任何电机指令）；
- `state=FAULT` = 链路/机械故障（看 `[gimbal] 链路 tx/rx/ok/e2` 那行）。
