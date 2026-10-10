# MaixCAM2 黑框中心：四边精修、快速跟踪与双轴云台瞄准

构建号 `2026-10-10-smooth-track-v15`，应用版本 `1.4.0`。在 MaixVision 中打开本文件夹，选择“运行项目”。
必须一起下发应用包内的 **12 个文件**（`app.yaml` / `main.py` / `config.py` / `target_api.py` /
`assets/icon.png` / `vision/` 下 7 个 `.py`），不能只运行当前 `main.py`。

> 只想临时试参数时，也可以先 `python tools/build_single.py` 生成 `dist/main_single.py`，
> 再用「运行当前文件」—— 那是自包含的单文件版，语义与分模块完全一致。改了源码记得重新生成。

## 当前追踪控制：运动量低通与防超调（v15）

`dx/dy` 继续使用原始靶心坐标。先计算增量比例控制的带符号脉冲，再限幅、低通和限制增长速度，
最后等待上一段曲线运动的估算时间再发下一条相对位置命令；`main.py` 和 `AimLoop` 共用这一控制器。

| 参数 | 默认 | 作用 |
|---|---|---|
| `AIM_OUTPUT_LPF_TAU_S` | 0.25s | 运动量增大时使用 `alpha=1-exp(-dt/tau)` 低通；越大越柔，0 关闭 |
| `AIM_OUTPUT_SLEW_PPS` | 120 | 每秒最多增加多少脉冲的单次运动量；越小起步越慢，0 关闭 |
| `MAX_STEP_PULSES` | 60 | 原 150 降到 60，限制大误差下单次转动 |
| `ACC_TRACK` | 80 | 原 200 降到 80，使驱动器曲线加减速更柔和 |
| `AIM_WAIT_MOTION` | True | 每轴等待上一段运动的估算时间，防相对位置命令重叠 |
| `AIM_MOTION_MARGIN_S` | 0.03s | 运动完成后的额外等待余量 |

两轴各有独立滤波记忆。首次锁定/重捕获从零逐渐增加运动量；运动量减小时立即收缩，
进入 3px 死区、换向或失锁时清空记忆，并取消估计仍在执行的运动。
取消使用驱动器的 `0xFE` 立即停止命令，属于防越界措施；平常移动的柔和程度由小步长与 `ACC_TRACK` 决定。
FAULT 和 UART 写失败也会真实发送停止帧。低通不会修正错误的 `DIR_SIGN_*`，方向反了仍会发散并触发保护。

等待依赖软件估算，并非编码器到位反馈；若实机完成动作较慢，增加 `AIM_MOTION_MARGIN_S`。
首次 250px 误差在默认 30fps 下输出 4 脉冲。有限运动时间的离线模型中，150px 初始误差
经 14 次小步修正收敛到 2.81px，约 2.17s，无运动指令重叠；实机惯量/齿隙仍需上板验证。

## 代码结构（应用包 12 文件 + 离线工具）

```text
maixcam2_rect_center/
├── app.yaml                  应用描述（id=maixcam2_rect_center，version 1.4.0）
├── main.py                   入口：采图 → 检测 → 光斑偏差 → 云台闭环 → 显示 / [perf][aim] 日志
├── config.py                 全部可调参数（现场唯一需要改的文件）
├── target_api.py             ★ 对外 API：detect_black_frame / CenterTracker / AimLoop
├── assets/icon.png           应用图标
└── vision/
    ├── __init__.py           统一导出 9 个符号
    ├── rectangle_center.py   黑框检测：多阈值扫描 → 四边暗带验证 → 亚像素精修 → 单应解中心
    ├── laser_spot.py         光斑读数（fixed 写死 / detect 实测）+ deviation(dx,dy) + 面板绘制
    ├── stepper_link.py       ZDT Emm_V5.0 双串口：帧构造 + 非阻塞收发 + ACK/E2 看门狗
    ├── gimbal_aim.py         两轴增量比例控制 + 状态机 + 离群拒收
    ├── gimbal_calibrate.py   开环标定：实测 PULSE_PER_PX / DIR_SIGN / 交叉项 / 回差
    └── gimbal_self_test.py   上电小幅自检 + IMU 探测

dist/
└── main_single.py            单文件自包含版（由 tools/build_single.py 生成，**不进应用包**）

tools/                        离线验证 / 调参 / 打包（**不进应用包**）
├── build_single.py           生成/校验/自检单文件版（--check / --smoke）
├── test_performance.py       视觉回归：30 场景 + 5 反例 + 运动/失锁/性能
├── test_laser.py             光斑与偏差回归
├── test_gimbal_aim.py        云台：协议帧 / 闭环收敛 / 符号反发散 / ACK·E2 / 标定器
├── test_target_api.py        对外 API 回归
├── example_api_usage.py      三种小问场景的可运行示例
└── check_app.py              MaixCAM 交付门禁 C1..C10（`--manifest` 打印包内清单）
```

对外 API 一览（供不同小问直接复用）：

| API | 语义 |
|---|---|
| `detect_black_frame(frame, cfg=None)` | **无状态**识别黑框，每帧独立全图搜索，返回 `Result` |
| `CenterTracker(cfg=None, with_laser=False)` | **有状态**追踪器，`.track(frame)`；`.reset()` 清锁定 |
| `track_center(frame)` / `reset()` | 函数式快捷写法（内部复用默认追踪器实例） |
| `load_config(**overrides)` | 派生一份覆盖参数的配置，**不改磁盘 config.py** |
| `AimLoop(cfg=None, enable_motors=True)` | 追踪 + 偏差 + 双轴云台闭环，`.step(frame)` |

`Result` 是 dict 子类：`res["u"]` 与 `res.u` 等价，`bool(res)` 等价于 `res.valid`。
详细字段与用法见下文「对外 API」一节、`target_api.py` 模块 docstring 及 `tools/example_api_usage.py`。

## 2026-10-10 改动⑧：修「自检/标定卡住」——标定视觉无效快速失败 + 默认免标定直接追踪

**现场现象**：上电后自检/校准会**卡住不动**。用户定位准确：**标定环节依赖视觉** ——
"没识别到黑框就不校准；而校准一抬头黑框跑出视野，又卡在等 dv 上"。

### 一、根因

`GimbalCalibrator.update()` 每一步都要 `result["aim_valid"]` 且 `dx/dy` 有效才推进
（`if dv is None: return self.state`）。于是：
- 画面里没有靶纸 → 卡在 `BASELINE`；
- 标定要驱动云台走 N 脉冲，**抬头后黑框出视野** → 卡在 `MEASURE_OUT`。

两条路都只能等 `CALIBRATE_TIMEOUT_S = 20s` 超时才失败 —— 表现就是"卡住"。

### 二、改法（解决"阻塞"本身，而不是压缩时间）

| 项 | 旧 | 新 |
|---|---|---|
| **标定视觉无效快速失败** | （无，只能干等 20s） | **`CALIBRATE_VISUAL_TIMEOUT_S = 3.0`**：连续 3s 无有效 `dx/dy` → 判 `no_vision` 失败，**立刻交还主流程** |
| 自动标定 | `CALIBRATE_AXIS = "auto"` | **`""`**（默认不标定，直接用手动宏换算） |
| 闭环解锁 | `GIMBAL_CALIBRATED = False` | **`True`**（参数由手动宏给出，自检完直接追） |
| 自检防死锁超时 | （无） | **`SELF_TEST_TIMEOUT_S = 10.0`**：只兜底"某步迟迟不返回"，**不是性能目标** |
| 闭环方向 | `DIR_SIGN_* = +1` | **`-1`**（几何推断：云台转过去时靶心在画面里反向移动） |
| 发散保护 | （无） | **`DIVERGE_GUARD`**：持续发指令却不收敛 → 自动 FAULT 停机 |

时序：**上电 → 自检 4 步（慢 + 曲线，约 3~4s）→ 直接进入追踪**，全程不等黑框。
自检参数保持"慢 + 曲线"手感，没有为抢时间而压紧。

> 自检超时的判据是"**还有步没发完**才算卡住" —— 4 步都发出后正常等最后一步走完，
> 否则会截断机械动作、并与紧随其后的追踪指令堆叠。
>
> 即使把 `CALIBRATE_AXIS` 改回 `"auto"`（想重新标定），画面里没靶心也只会在 3s 内失败并
> **把控制权交还主流程** —— 不会再出现"卡住不动"。

- `test_gimbal_aim` **PASS(24)**（新增 `test_diverge_guard`、`test_calibrator_fails_fast_without_vision`）；
  `test_main_self_test` 2 条 PASS（其模拟时钟步长同步改为真实量级，否则超时兜底会被瞬间触发）。
- `BUILD_ID = 2026-10-10-no-block-v14`；门禁 **74 OK/0 FAIL**；性能无回归。

> ⚠ 因跳过标定，`DIR_SIGN_*` 是**按几何推断**的。上电后若"越追越偏"，把对应项改成 `+1`；
> `DIVERGE_GUARD` 会在这种情况下自动 FAULT 停机保护。需要重新标定时把 `CALIBRATE_AXIS`
> 临时改回 `"auto"`（或 `"yaw"`/`"pitch"` 单轴）。

## 2026-10-09 改动⑦：追踪回归（按实测轴映射）+ 闭环速度曲线（防急停晃开激光）

### 一、追踪已恢复

`CALIBRATE_AXIS` 改回 `"auto"`：上电自动标定两轴 → 写回运行时 config → 解锁闭环进入追踪。
标定按**实测轴映射**进行（`pitch` 在 `U0`/`ttyS0`/ID=1，`yaw` 在 `U2`/`ttyS2`/ID=2 —— 见上方接线表），
所以实测出的 `PULSE_PER_PX_*` / `DIR_SIGN_*` 会落在**正确的轴**上，不会再出现"yaw 的系数用在 pitch 上"。

### 二、闭环加"速度曲线"：防止急停把激光晃开

闭环每 `COMMAND_INTERVAL_S`（80ms）发一条位置命令。此前用的是 `ACC_DEFAULT = 0`，而位置模式
`acc = 0` 的含义是**不使用曲线加减速、直接以设定速度启停** —— 每条命令都是一次硬启硬停，
机械冲击让激光在停下的瞬间偏离（现场反馈的"急停对不准"）。

现在闭环改用新的 **`ACC_TRACK`** 并整体降速：

| 参数 | 旧 | 新 | 作用 |
|---|---|---|---|
| `RPM_FINE` | 300 | **120** | 精调段转速，更慢更稳 |
| `RPM_COARSE` | 1200 | **400** | 粗调段转速 |
| `ACC_TRACK` | （借用 `ACC_DEFAULT = 0`） | **200** | 闭环曲线加减速档位 |

`acc ≠ 0` 时，**短行程会自动退化为三角速度曲线**：以 18 脉冲的小步为例，从 acc=0 的约 2.8ms
硬走，变为 acc=200 的约 62ms 平滑起停（峰值仅约 11 RPM）。这个时长恰好与 80ms 的命令间隔衔接 ——
既柔和不冲、又不拖慢跟踪；误差大时行程更长、峰值更高，自然连成平滑跟随。

- `test_gimbal_aim` **PASS(21)**：新增 `test_track_uses_curve_config`，它同时是 **`main.py`
  被并发覆盖的探针**（旧版用 `ACC_DEFAULT`，一被覆盖测试即红）。
- `BUILD_ID = 2026-10-09-track-curve-v9`；门禁 **74 OK/0 FAIL**；性能无回归（29/30、0.803px、0 误检）。
- `tools/apply_gimbal_track_patch.py` 的补丁清单同步扩到 **5 项**（含 `ACC_TRACK` 这一处）。

## 2026-10-09 改动⑥：暂关追踪；自检改为「固定脉冲、不需要靶子」的 pitch 上下 / yaw 左右

按现场要求调整两处。追踪的代码与能力**全部保留**，只是配置关掉。

### 一、关掉追踪

`CALIBRATE_AXIS` 由 `"auto"` 改回 `""`。上电不再自动标定，闭环保持 `UNCALIBRATED` 锁定
（`AIM_REQUIRE_CALIBRATION=True` + `GIMBAL_CALIBRATED=False`）—— **只输出视觉结果、不发任何电机指令**。
要恢复追踪：把 `CALIBRATE_AXIS` 改回 `"auto"` 即可，其余不用动。

### 二、自检改为固定脉冲，且与视觉/标定完全解耦

自检本来就是纯开环动作、与视觉无关；这次把它变成**唯一默认**路径：

| 项 | 旧 | 新 |
|---|---|---|
| 判定依据 | 先看 `PULSES_PER_DEG`，未标定才用固定脉冲兜底 | **直接用固定脉冲**，完全不看标定 |
| 参数名 | `SELF_TEST_FALLBACK_PULSES` | **`SELF_TEST_PULSES`**（旧名仍兼容读取） |
| 动作顺序 | yaw ×3 → pitch ×3（+F / −2F / +F） | **pitch 上转 → 下转；yaw 左转 → 右转**（每轴两段等长，回到起始位姿） |
| 步数 | 6 | **4** |

上电日志：`[self-test] 固定脉冲自检（无需靶子 / 无需标定）：pitch 上转 400 → 下转 400，yaw 左转 400 → 右转 400`。
只有 `SELF_TEST_PULSES = 0` 时才回退到"按 `YAW/PITCH_PULSES_PER_DEG` 换算角度"的 6 步老路径。

**顺带修掉一个逻辑缺陷**：原判据是"序列走空即 FAULT"，在新顺序下会误伤 —— pitch 已走完、
yaw 写失败（此时序列恰好为空）会被判成整机故障。现改为 **"所有参与轴都失败才算 FAULT"**
（`self._failed >= self._planned`），某轴不通不再抹掉另一轴的结果。

- `BUILD_ID = 2026-10-09-self-test-pulse-v7`；`test_gimbal_aim` **PASS(20)**；
  性能无回归（29/30、0.803px、0 误检）；门禁 **74 OK/0 FAIL**。

## 2026-10-09 改动⑤：加追踪 —— `CALIBRATE_AXIS="auto"` 自动标定并自动解锁闭环

两轴能动之后，剩下的一步是**让闭环真正工作**。此前卡在两点：① 人工标定需要看屏幕读数、
回填 config 再重启；② `AIM_REQUIRE_CALIBRATION=True` + `GIMBAL_CALIBRATED=False` 把闭环锁成
`UNCALIBRATED`（只出视觉、不发指令）。

现在设一行即可从"上电"直接走到"跟踪"：

```python
CALIBRATE_AXIS = "auto"
```

流程：`AutoCalibrate` 依次标定 yaw → pitch（每轴仍是"静置取基线 → 正走 N 脉冲 → 回位 → 再测"），
两轴**全部成功**后把实测 `PULSE_PER_PX_*` / `DIR_SIGN_*` 写回**运行时** cfg，并调用
`GimbalAim.unlock()` —— 无需回填、无需重启，本帧起进入 `aim.update()` 跟踪。

**安全口径（重要）**：**任一轴失败就整体不解锁**，闭环保持 `UNCALIBRATED`、一条指令也不发。
所以"画面里没有靶心"时不会盲目闭环；日志会明确给出
`自动标定未全部成功 -> 闭环保持锁定，不会发电机指令`。

| 参数 | 旧 | 新 | 说明 |
|---|---|---|---|
| `CALIBRATE_AXIS` | `""` / `"yaw"` / `"pitch"` | 增加 **`"auto"`** | 自动逐轴标定 + 自动解锁 |
| `CALIBRATE_PULSES` | 2000 | **400** | 行程宁小勿大：太多会把靶心推出视野 → 标定失败 |
| `CALIBRATE_RPM` | 300 | **60** | 标定动作也要慢 |
| `CALIBRATE_ACC` | （无） | **60** | 新增：标定也走曲线加减速 |
| `AUTO_CALIBRATE_AXES` / `AUTO_CALIBRATE_TIMEOUT_S` | — | `("yaw","pitch")` / 150s | auto 范围与总超时 |

- `GimbalAim.unlock(reason)`：解除未标定锁，幂等（已解锁再调返回 False）。
- `target_api.AimLoop(calibrated=True)` / `AimLoop.unlock()`：API 侧同样可解锁。
- `test_gimbal_aim` **PASS(20)**（新增 `test_auto_calibrate_success`、
  `test_auto_calibrate_failure_keeps_locked`、`test_unlock_after_calibration`）。
- `BUILD_ID = 2026-10-09-auto-calib-track-v6`；门禁 **74 OK/0 FAIL**；性能无回归（29/30、0.803px）。
- ★ **`tools/apply_gimbal_track_patch.py`**：本接线的 4 处 main.py 改动做成**幂等补丁**。
  两轨共用一个工作树时，`main.py` 曾被并发写入**整份覆盖**（`vision/*`、`config.py` 不受影响），
  现象是"自动标定代码明明在，上电却不追踪"。怀疑被覆盖时先跑
  `python tools/apply_gimbal_track_patch.py --check`，需要时去掉 `--check` 一键补上，
  再按契约跑 `build_single.py` + `check_app.py --manifest`（C8 会核对 BUILD_ID 一致性）。

> **想固定标定结果**（免得每次上电都标一遍）：把日志 `[calib-auto]` 打印的两组数值复制进
> `config.py`，再把 `CALIBRATE_AXIS` 置回 `""`、`GIMBAL_CALIBRATED = True`。

## 2026-10-09 改动④：自检改「慢速 + 曲线加减速」；yaw 显式 pinmap + 引脚实测回读

**现场反馈两件事**：① pitch 已能动作、**yaw 仍不动**；② 上电自检把电机**抖得卡顿**。
两件事各自定位到明确原因。

### 一、自检卡顿：位置模式 `acc=0` 就是"硬启硬停"

ZDT 位置模式（`FD`）第 5 字节是**加速度档位 0–255**。官方说明书原文：

> 加速度为 0 表示**不使用曲线加减速，直接按照设定的速度启动和运行**；不为 0 则曲线加减速
> 时间 `t2 - t1 = (256 - acc) * 50(us)`，`Vt2 = Vt1 + 1(RPM)` —— **acc 越大加速越快**（越生硬）。

旧值 `SELF_TEST_RPM=900` 且沿用 `ACC_DEFAULT=0` → 400 脉冲在约 8ms 内"弹"完，必然抖。现改为：

| 参数 | 旧 | 新 | 说明 |
|---|---|---|---|
| `SELF_TEST_RPM` | 900 | **60** | 自检专用慢速（不影响闭环 `RPM_FINE=300`） |
| `SELF_TEST_ACC` | （无，走 `ACC_DEFAULT=0`） | **60** | 非 0 = 启用曲线加减速；越小越柔和（3D 打印常用 20–80） |
| `SELF_TEST_SETTLE_S` | 0.35 | **0.8** | 动完多停一会儿再发下一步 |

新增纯函数 `gimbal_self_test.estimate_move_seconds()`：按官方公式建模，**行程不足时自动
退化为三角速度曲线**（实际峰值低于设定速度），用于排程、避免早发指令造成堆叠。
400 脉冲 @60RPM/acc=60 由 0.13s 变为 **≈0.54s**，观感即"缓缓移动"。

### 二、yaw 仍不动：去掉「默认已复用」这个未经证实的假设

pitch（`U2`/B0-B1，**显式 pinmap**）能动作，而 yaw（`U0`/A28-A29，**空 pins 跳过 pinmap**）
不能 —— 同一套代码下只剩一种解释：**A28/A29 当前功能不是 UART0**。改法：

- `YAW_PINS` 由 `{}` 改为 **`{"A28": "UART0_TX", "A29": "UART0_RX"}`**，不再依赖默认复用。
- 新增 `PINMAP_STRICT = False`：**pinmap 失败只警告、仍打开 UART**。避免功能名不匹配时
  把该轴变成"打不开"，把真正的故障点藏起来。
- **回读实测引脚功能**：打开后用 `pinmap.get_pin_function()` 读硬件真值（MaixCAM2 支持），
  启动日志新增一行：

  ```
  [gimbal] yaw   OK   dev=/dev/ttyS0 baud=115200 pins=A28=UART0_TX,A29=UART0_RX pinmap=True -> opened
  [gimbal]   └ 引脚实测: A28=UART0_TX  A29=UART0_RX
  ```

  **期望 ≠ 实测** = 该脚没切到串口功能 → 这是"某轴打开了却不通"的直接证据。
- `tools/uart_probe.py` 同步：pins 为空时也按标称引脚回读，并标出 `<-- 不一致!`。

- `BUILD_ID = 2026-10-09-self-test-curve-v5`；`test_gimbal_aim` **PASS(17)**
  （新增 `test_estimate_move_seconds`、`test_self_test_uses_curve`）；
  性能无回归（29/30、0.803px、0 误检）；门禁 **74 OK/0 FAIL**。

> yaw 若仍不通，按序排查：① 启动日志 `└ 引脚实测` 是否为 `A28=UART0_TX`；② `uart_probe --listen 5`
> 有无回帧；③ `uart_probe --loop yaw`（先把 A28-A29 短接）验证 MaixCAM 侧 UART。
> 仍不行就换 **UART3(B2/B3)**：`YAW_DEV="/dev/ttyS3"`、`YAW_PINS={"B2":"UART3_TX","B3":"UART3_RX"}`。

## 2026-10-09 改动③：★ 修「开机第 1 帧就误判 FAULT → 云台永不自检/永不追踪」

**这是"模式对、接线对、电机有劲，却完全不动"的真正根因**（与硬件无关，所以怎么改硬件都没用）。

`main.py` 判断链路故障时写的是 ``link_fault is not None``，而 `StepperLink._ingest()` 的约定是
**"无故障返回 `fault=False`"**。`False is not None` 恒为 `True` → **开机第 1 帧**就执行

```
link.stop()  +  aim.fault("link:False")   ->  云台进入 FAULT
```

而 FAULT 之后，主循环里 `self_test.update()` / `aim.update()` 的分支整块被跳过 →
**自检一帧指令都没发出去、闭环也永远不工作**。现场表现 =「上电电机使能有劲，但完全不动」。

- 新增 `stepper_link.fault_reason(fb)`：**无故障返回 `None`**，故障返回原因字符串，
  `main.py` 与 `target_api.py` 统一走它（杜绝再次写成 `is not None`）。
- `main.py`：故障判断改为 `fault_reason(fb) is not None` 且打印具体原因。
- 新增 `StepperLink.status_line()`，把**链路心跳**打进周期日志：
  `[gimbal] 链路 tx=N rx=N ok=N e2=N to=N drop=N | yaw:open,0x02 pitch:open`
  —— `tx=0` = 一条指令都没发出；`tx>0 rx=0` = 发出去了但驱动器一字不回。
- 回归 `test_gimbal_aim` **PASS(15)**：新增 `test_fault_reason_contract` 把这条口径锁死。
- `BUILD_ID = 2026-10-09-gimbal-fault-fix-v3`；门禁 **74 OK/0 FAIL**（包内 12 文件）。

> 判读：若启动日志出现 `[aim] 未标定：闭环已锁定` —— 那是**有意为之的安全闸**（防占位
> `PULSE_PER_PX` 正反馈甩头），不是故障。自检应先动 6 步，再进入"只出视觉、不发指令"。

## 2026-10-09 改动②：yaw 改走 UART0 + 修「一轴不通拖死另一轴」

**背景**：MaixCAM2 外接排针上能用的串口只有 `U0`(U0T/U0R) 与 `U2`(U2T/U2R) 两组 ——
官方《UART》串口表里 MaixCAM2 的 `UART1` TX/RX 两列是 `—`（无引脚）。
旧配置把 yaw 指到 `UART1(A30/A31, ttyS1)`，在这块板上等于**没接**。

- **配置改名 + 重新映射**（旧名 `U1_*`/`U2_*` 仍兼容，`axis_cfg()` 统一读）：
  轴 1 **yaw → `UART0` `/dev/ttyS0`**，`YAW_PINS = {}`（**空表 = 跳过 pinmap**，UART0 默认已复用）；
  轴 2 **pitch → `UART2` `/dev/ttyS2`**，`PITCH_PINS = {"B0":"UART2_TX","B1":"UART2_RX"}`。
- **修掉真正让 U2 电机"只使能、不自校"的缺陷**：自检序列第一步是 yaw，旧实现一旦 yaw 指令
  写失败就**整机 `FAULT`**，pitch 一步指令都发不出去。现改为：① 只对链路**已打开**的轴做自检；
  ② 单轴写失败只**跳过该轴剩余步骤**，另一轴照常走完。
- `stepper_link`：新增 `axis_cfg()`（设备/波特率/引脚，空 pins 跳过 pinmap）与 `flush()`
  （开机清空 UART0 上的内核日志/终端回显，开关 `FLUSH_RX_ON_START`）。
- `gimbal_self_test`：新增 `SELF_TEST_START_DELAY_S`（默认 0.5s，等驱动器处理完使能帧再动）。
- `tools/uart_probe.py` 重写：直接读 `config.py`、复用 `StepperLink` 同一套打开逻辑，
  支持 `--listen N`（盯原始 RX）/ `--move [yaw|pitch]` / `--loop <axis>`。
- 回归 `test_gimbal_aim` **PASS(14)**：新增「单轴写失败另一轴仍走完」「链路未打开的轴被跳过」
  「两轴都写失败→FAULT」「两轴都打不开→SKIPPED」「`axis_cfg` 新旧命名兼容」5 条。
- `BUILD_ID = 2026-10-09-uart-u0-yaw-v2`；门禁 `check_app --manifest` **74 OK/0 FAIL**。

> ⚠ **UART0 是系统终端/日志口**（官方不推荐外接，内核日志会串进电机 RX）。选它是因为现场
> 排针上只有 U0/U2。若出现偶发乱动/丢帧，改用 `UART1(A30/A31)` 或 `UART3(B2/B3)`
> （官方引脚图 v1.0 标这两个有 UART 复用，与 SPI2 冲突需先释放）—— **只改 `YAW_DEV`/`YAW_PINS`
> 两行**，其余代码不用动。

## 2026-10-09 改动①：串口诊断与上电自检

- **定位「电机使能但不自校」第二重原因**：未标定 `PULSES_PER_DEG`/`GEAR_RATIO` 时自检被
  **主动跳过**（已加固定脉冲兜底）。~~① 接在 U0 上收不到帧~~ —— 结论已在改动②修正。
- `vision/stepper_link.py`：pinmap 改严格判定（`err.check_raise`，不再静默吞失败）；新增
  `health_lines()` 启动打印两轴设备/引脚/pinmap/打开状态。
- `vision/gimbal_self_test.py`：未标定时用 `SELF_TEST_FALLBACK_PULSES`（默认 400）做上电自检。
- `vision/gimbal_aim.py`：新增未标定锁（`AIM_REQUIRE_CALIBRATION` → `UNCALIBRATED`），未标定不发指令。
- 新增 `tools/uart_probe.py`（板端双串口诊断）；`BUILD_ID = 2026-10-09-uart-diag-v1`；回归与门禁全绿。

## 2026-10-08 改动

**① 新增对外 API `target_api.py`** —— 把「检测器 + 追踪器 + 光斑偏差 + 云台闭环」收成一层稳定门面，
不同小问 `import` 即用，不必拼内部模块路径与参数。顶层不 import maix；输入兼容 numpy
`(H,W)`/`(H,W,1)`/RGB/RGBA 与 MaixPy `image`，PC 上可离线跑通并回归。

**② 闭环口径改为 `raw_u/raw_v`** —— `deviation()/draw_laser_overlay()` 增加 `use_raw`，
`config.AIM_USE_RAW=True` 默认生效。旧写法用带 `CENTER_ALPHA=0.70` 一阶低通的 `u/v` 参与闭环，
附加约 0.83 帧 ≈ 28 ms 滞后，底盘运动时直接变成瞄准误差；现在 `CENTER_ALPHA` 只服务显示。

**③ 控制状态机补全（`gimbal_aim.py` 重写）** —— 新增
`BOOT / COARSE / FINE / SETTLE / HOLD / SCAN / FAULT` 七态、粗/精两段增益与转速；
`JUMP_REJECT_PX=400` 离群拒收（连续 `JUMP_CONFIRM_FRAMES=3` 帧才采信），防单帧误检甩头；
保留死区、指令限频、单帧脉冲限幅，并新增 `fault() / reset() / status_line()`。
`SCAN` 失锁慢扫默认关闭（`ENABLE_SCAN=False`），确认识别机械行程后再开。

**④ 链路看门狗（`stepper_link.py`）** —— 解析返回帧状态：`0x02` 计成功；`0xE2`（未使能/堵转）
**自动补发一次 `0xF3`**；窗口 `E2_WINDOW_S` 内超过 `E2_FAULT_COUNT` 次 → `FAULT` 并停机。
ACK 硬超时做成**自动识别**：只有收到过确认帧（驱动器 `Response=Receive/Reached/Both`）后才启用，
`Response=None` 时该判据自动不生效，不会误判。`_drain/_ingest` 拆分后返回帧解析可在 PC 上纯函数回归。

**⑤ 新增开环标定 `gimbal_calibrate.py`** —— `CALIBRATE_AXIS="yaw"|"pitch"` 开机一键实测并打印
`PULSE_PER_PX_*`、`DIR_SIGN_*`、交叉项（判相机横滚）与回差，非阻塞，标定期 `main` 跳过闭环。
**这是"上电即可准确追踪"的前提** —— `3.67` 只是占位值，不标定直接闭环有正反馈甩头风险。
自检另加 `GIMBAL_GEAR_RATIO` 兜底：已知减速比时不必先做脉冲/度标定也能做上电自检。

**⑥ 补齐 B 轨的"改完三件事"配套** —— 新增 `tools/build_single.py`（与 A 轨同款）：
把 `config.py + vision/*.py + target_api.py + main.py` 共 9 个源文件合成自包含的
`dist/main_single.py`，支持 `--check`（是否过期）与 `--smoke`（PC 自检：8 个嵌入模块能否就位、
`BUILD_ID` 是否与 `main.py` 一致）。`dist/` 在 `app.yaml` 的 `exclude` 里，**不进应用包**。

**⑦ 其余** —— `vision/__init__.py` 统一导出 9 个符号；`app.yaml` 版本 1.3.0 → **1.4.0**；
`BUILD_ID` → `2026-10-08-target-api-v3`（板端日志须逐字一致）；应用包 **7 → 12 文件**。

## 上一版改动（2026-10-07 · 激光光斑读数）

1. **新增激光光斑读数面板**（`vision/laser_spot.py`）：默认 `LASER_MODE="fixed"`，把光斑
   位置**写死**成 `(294, 202)`，每帧只算"黑框中心 − 固定光斑"的两次减法，屏幕左上角显示
   `LASER (294,202) [fixed/calib]` 与 `D-CENTER dx dy px`，画面上用红圈标出该位置、橙线连到靶心。
   **不做任何图像检测**；需要标定或校验时才临时切 `LASER_MODE="detect"`（标称位 ±64px ROI 内实测
   亚像素质心，`LASER_INTERVAL_S=0.30` 限频）。详见下文“激光光斑读数”一节。
2. `vision/rectangle_center.py` 的 `draw_result()` 新增 `top_offset` 参数：把 "NO BLACK RECTANGLE"
   与中心读数整体下移，避免与左上角面板叠字。
3. 新增 `tools/test_laser.py`（激光回归 19 项）与 `tools/check_app.py`（MaixCAM 应用交付门禁 C1..C10）。
4. 以下两处是更早一轮已实现、随该版一并入库的改动：结构验证的暗带采样点由 `0.25/0.50/0.75` 改为
   **`0.20/0.40/0.60`** 且暗带统计量由中位数改为**最大值**（更贴近黑带最黑处，反光按最坏情况从严）；
   `ENABLE_LINE_RECOVERY=False` 时 `_recover` 直接返回，不再做无用的 Canny；`tools/test_performance.py`
   增加 `acquire()` —— 连续确认拿到有效结果后才计入稳态耗时，避免把首次锁定混进帧耗时。


## 这次修复和优化

旧版每帧在全图扫描最多 9 个黑色阈值，最多 64 个四边形两两配对，还在 Python 循环中逐点读取灰度。它的宽松回退只检查中心够亮，可能把靶纸边缘或桌面阴影识别成黑框。本版删除了该回退。

| 阶段 | 当前策略 |
|---|---|
| 初次搜索/失锁 | 将画面或 ROI 缩至宽 320 像素，先试上次阈值和 Otsu；每档一旦找到通过结构验证的候选就停止 |
| 黑环/亮面双候选 | 使用 `RETR_TREE` 的黑色外轮廓与内部亮孔。每档最多处理 8 个大轮廓，避免全组合配对 |
| 精度恢复 | 将粗定位四角映射到原图，在四条窄带里寻找真实灰度跳变，用 `fitLine` 拟合直线，相邻直线求交得到顶点 |
| 稳态跟踪 | 每帧仅在上一帧四边附近 ±12 像素内搜索，再检查当前帧黑带；不扫描阈值，不调用全图 Canny，不复用旧帧坐标 |
| 快速移动 | 边带跟踪失败时先搜索扩展 ROI，仍失败才重新全图搜索 |
| 反光断环 | Hough 长线段恢复默认关闭，避免失锁时一次计算阻塞数秒；确认现场需要时才将 `ENABLE_LINE_RECOVERY=True` |
| 结构验证 | 在物理正视面选择四边黑带和内侧亮区的稀疏点，映射回当前帧，一次 `remap` 批量采样；不生成整张正视图 |
| 解算 | 归一化坐标后构建 8×8 方程，向量化高斯消元；标准圆只创建一次，72 点每帧批量映射 |
| 显示 | 灰度图先缩至宽 480，再转 BGR 和画标记；显示最多 15 FPS，检测和串口每帧运行 |

结构验证要求至少三边的暗带覆盖率达到 55%，第四边达到 25%，并同时检查绝对灰度差与黑/白亮度比。允许局部反光和暗背景，但不以“中心够亮”作为通过依据。

## 中心和四角的定义

正视面为 1 px=1 mm 的物理坐标系。中心是物理圆心的逆投影，不能对投影后的圆周点求均值。72 点标准圆保留在 `result['circle_points']`，所有输出均使用处理图像的完整分辨率。

绿色描边与 P1～P4 表示黑框外沿；黄色线表示黑框内沿。当只能检测亮面内孔时，外沿根据 `BLACK_FRAME_MM` 和单应性推算，结果中的 `outer_inferred=True`。内孔对应的实际检测顶点在 `detected_quad` 中。P1 是图像左上方向的角，之后按顺时针编号；绕光轴旋转时编号可能轮换。

`TARGET_SHORT_MM=210`、`TARGET_LONG_MM=297`、`BLACK_FRAME_MM=18` 必须与实际靶纸/胶带匹配，尤其内沿推算外沿依赖胶带宽度。已裁断四边或大面积反光没有足够证据时返回无效，不向控制端输出旧中心。中心时间平滑采用 `CENTER_ALPHA=0.70`，大跳变会重置；需无延迟可设为 0，或直接使用 `raw_u/raw_v`。

## 上板检查与性能日志

先关闭旧工程再重新打开本文件夹并“运行项目”，核对开机日志构建号。每 2 秒输出：

```text
[rect-center] 2026-10-09-gimbal-fault-fix-v3 search_width=... display_width=... display_fps=...
[aim] 闭环口径=raw（raw_u/raw_v）离群拒收=400.0px 死区=2.0px 限幅=360p
[perf] 2026-10-09-gimbal-fault-fix-v3 valid=True FPS=... stage=track frames=... searches=0 read=... lens=... bridge=... detect=... laser=... uart=... rx=... aim=... show=... loop=... ms | laser_call=...ms n=... hit=...
[aim] state=FINE dx=+3.2 dy=-1.1 yaw=CCW/12p pit=CW/4p lost=0
[gimbal] 链路 tx=18 rx=6 ok=6 e2=0 to=0 drop=0 | yaw:open,0x02 pitch:open,0x02
```

> **云台不动时的第一判据是 `[gimbal] 链路` 那行**：`tx=0` = 一条指令都没发出去（链路没打开或被
> 上层拦住）；`tx>0` 而 `rx=0` = 帧发出去了但驱动器一个字节都没回（查 `P_Serial=UART_FUN`、
> `ID_Addr`、`Checksum=0x6B`、波特率、TX/RX 交叉与共地）；`ok>0` = 驱动器在应答，链路肯定通。
> 启动时还会逐轴打印 `health_lines()`（设备/引脚/pinmap/是否打开）。

| 字段 | 意义/处理方法 |
|---|---|
| `stage=track`、`searches=0` | 四边跟踪工作正常，稳态无全图搜索 |
| `detect` | 检测每帧平均毫秒数；一直搜索时检查光照、靶纸尺寸与边缘完整性 |
| `read` | 相机采图等待；可能受相机请求帧率限制 |
| `lens` | MaixPy `lens_corr` 的整图校正耗时；保留旧版开关。镜头直线几乎不弯时可设 `LENS_CORR=False` 对照测试 |
| `show` | 平均到所有检测帧的显示成本；可用 `SHOW=False` 测试纯检测/串口，或进一步降低 `SHOW_WIDTH/SHOW_FPS` |
| `loop` | 含采图、校正、检测、串口及显示的完整循环耗时；比纯检测耗时更接近实际帧率 |

FPS 根据完整循环计算，包含显示开销；显示限频不会跳过检测帧。`LENS_STRENGTH=1.5` 是旧工程的经验值，并非这台相机的标定参数，不能保证真实镜头的畸变精度。镜头校正启用时坐标属于校正后的图像；关闭时属于相机原帧。

## 主要配置

| 参数 | 默认值 | 调整建议 |
|---|---|---|
| `SEARCH_WIDTH` | 320 | 很小的远处目标可增加到 400/480，代价是失锁搜索更慢 |
| `EDGE_SAMPLES` | 24 | 每条边采样点数；一般无需增加 |
| `TRACK_RADIUS` | 12 | 像素，快速移动可设 16；过大会搜索到印刷线或另一侧黑边 |
| `REFINE_RADIUS` | 6 | 初次粗定位后的原图精修半径 |
| `MIN_EDGE_CONTRAST` | 20 | 暗场/低对比可降至 15，降低过多会增加误检 |
| `MAX_DARK_BRIGHT_RATIO` | 0.72 | 黑带/白面最大灰度比，越低越严格 |
| `SHOW_FPS` / `SHOW_WIDTH` | 15 / 480 | 减少 RGB 拷贝与 MaixVision 传输；不改变检测输出坐标 |
| `OPENCV_THREADS` | 1 | 减少 ARM 小图运算的线程调度开销 |
| `ENABLE_UART` | False | 开启后每检测帧输出 `@[u,v]%`，无效输出 `@[invalid]%`（仅 `ENABLE_GIMBAL=False` 的降级路径用） |

云台闭环相关（详细见 `MaixCAM2串口直控云台需求文档.md`）：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `ENABLE_GIMBAL` | True | 打开两条 UART 并驱动两个 ZDT 电机 |
| `AIM_USE_RAW` | True | 闭环用未平滑的 `raw_u/raw_v`（显示仍用平滑值） |
| `DIR_SIGN_YAW/PITCH` | -1 | 当前几何推断，方向仍需实机确认；反了会正反馈甩头 |
| `PULSE_PER_PX_YAW/PITCH` | 0.0 | 当前用 `DEG_PER_PX=0.30 × PULSES_PER_DEG=8.89`；宏关闭后才回退此项 |
| `GAIN_YAW/PITCH`、`GAIN_COARSE` | 0.3 / 0.5 | 精/粗两段增益；最终运动量另受低通与增长限速 |
| `DEADBAND_PX` | 3.0 | 应 ≥ 1.5 × 实测回差，否则在齿隙里极限环 |
| `MAX_STEP_PULSES` | 60 | 单次脉冲限幅，防误检猛甩 |
| `RPM_FINE` / `RPM_COARSE` / `ACC_TRACK` | 60 / 150 / 80 | 追踪转速与**曲线加减速档位**；`acc=0` 会让闭环急停、激光晃开 |
| `JUMP_REJECT_PX` | 400 | 单帧离群拒收门限（连续 3 帧才采信） |
| `CALIBRATE_AXIS` | `""`（当前值） | `""` = 不标定，自检完直接追踪；`"yaw"`/`"pitch"` 只标该轴；`"auto"` = 自动标定两轴 |
| `AUTO_CALIBRATE_AXES` / `_TIMEOUT_S` | ("yaw","pitch") / 150 | auto 模式的标定范围与总超时 |
| `CALIBRATE_PULSES` / `_RPM` / `_ACC` | 400 / 60 / 60 | 标定行程与动作（宁小勿大；与自检同样走曲线加减速） |
| `ENABLE_SCAN` | False | 失锁自动慢扫重捕获；确认机械行程后再开 |
| `SELF_TEST_RPM` / `SELF_TEST_ACC` | 60 / 60 | 自检转速与**曲线加减速档位**；`acc=0` 是"硬启硬停"会卡顿，**越小越柔和** |
| `PINMAP_STRICT` | False | pinmap 失败是否致命；False=警告后仍打开 UART（避免整轴"打不开"掩盖真故障） |
| `SELF_TEST_PULSES` | 400 | **上电自检幅度（固定脉冲）**：pitch 上转→下转、yaw 左转→右转，**不需要靶子、不看标定**；置 0 才回退角度模式 |
| `AIM_REQUIRE_CALIBRATION` | True | 未标定禁止发电机指令，闭环锁定（防正反馈甩头） |
| `GIMBAL_CALIBRATED` | True | 当前手动宏视为已就绪，允许自检完追踪；需强制标定时置 False |

### 串口接线（MaixCAM2 丝印 ↔ 引脚，2026-10-09 复核官方 UART 表 + 引脚图 v1.0）

| 串口 | 丝印 | 引脚 | 设备 | 本工程用途 / 备注 |
|---|---|---|---|---|
| UART0 | `U0T` / `U0R` | A28 / A29 | `/dev/ttyS0` | **pitch**（★ 实测挂此口，驱动器 ID=1）。系统终端/日志口；**显式 pinmap** |
| UART1 | `U1T` / `U1R` | A30 / A31 | `/dev/ttyS1` | 官方 UART 表标「无引脚」，引脚图标可作 UART1（与 SPI2_CS3/PWM7 复用），需 pinmap |
| UART2 | `U2T` / `U2R` | B0 / B1 | `/dev/ttyS2` | **yaw**（★ 实测挂此口，驱动器 ID=2）。需 pinmap 到 UART2 功能 |
| UART3 | `U3T` / `U3R` | B2 / B3 | `/dev/ttyS3` | 备用（与 SPI2_MOSI/MISO 复用） |
| UART4 | `U4T` / `U4R` | A21 / A22 | `/dev/ttyS4` | 底部 6P 1.25mm 扩展座默认串口，备用 |

> ★ **实测接线：pitch 电机接 `U0`、yaw 电机接 `U2`**（与最初设想相反；代码已按此把端口+引脚+地址
>   整体互换，见 `config.py` 顶部注释）。**TX↔RX 交叉、三路 GND 共地**。
>   若把两根串口线对调、并把驱动器 `ID_Addr` 设回 yaw=1 / pitch=2，则把 `YAW_DEV/PITCH_DEV` 两组改回即可。
> 现场排查：板上跑 `python tools/uart_probe.py`（`--listen 3` 盯回帧、`--move` 让两轴小幅动一下）。
> ⚠ yaw 用的 `UART0` 是系统日志口，内核日志会串进电机 RX。若出现**偶发乱动**，或启动日志
> `└ 引脚实测` 显示 A28/A29 不是 `UART0_TX/RX`，就把配置切到 **UART3(B2/B3)**：
> `YAW_DEV="/dev/ttyS3"`、`YAW_PINS={"B2":"UART3_TX","B3":"UART3_RX"}`，其余代码不用动。

### ★ 驱动器必须先开串口模式（否则「上电有反应、发指令没反应」）

`Emm_V5.0` / `ZDT_X42_V1.2` 出厂 `P_Serial` 默认是 **`RxTx_OFF`（通信口关闭）**。此时
**串口发出的任何帧都不会被解析** —— 现象正是「电机通电锁轴（像"已使能"）、但对串口指令毫无反应、
`uart_probe --listen` 一个字节都收不到」。**这不是 MaixCAM 侧的问题，是驱动器菜单没配。**

> 另注：`En` 端口不接线时默认 `Hold`（一直使能）→ 上电即锁轴，**「电机有劲」并不代表串口通了**。

**用驱动器上的 OLED + 按键设置**（上电后用 UP/DOWN 翻页、Enter 进入/确认；改完自动保存、立即生效）：

| 菜单项 | 设成 | 说明 |
|---|---|---|
| `Cal` | 先跑一次 | **未做过编码器校准的板子只有 6 个菜单项**（`Cal/MotType/P_Pul/P_Serial/Restore/Exit`），校完才解锁完整菜单。上电若提示 `Not Cal` 必须先做，**空载/轻载**下执行，电机会自动正转一圈再反转一圈 |
| `P_Serial` | **`UART_FUN`** | ← **最关键**。默认 `RxTx_OFF`。改为把 R/T 口复用成串口 |
| `UartBaud` | **`115200`** | 与 MaixCAM2 一致（默认就是 115200） |
| `ID_Addr` | yaw=**1**、pitch=**2** | 与 `config.ADDR_YAW/ADDR_PITCH` 一致 |
| `Checksum` | **`0x6B`** | 代码帧尾就是固定 `0x6B`；改成 XOR/CRC-8 会让所有帧被拒 |
| `Response` | **`Receive`** | 调试期设这个才能收到回帧/E2；量产可设 `None` |
| `MotType` | **`1.8°`** | 42 电机 |
| `MStep` | **`16`** | 16 细分 → 3200 脉冲/圈，与 `MOTOR_PULSES_PER_REV` 一致 |
| `P_Pul` | `PUL_OFF` | 纯串口控制时建议关掉脉冲口，避免干扰 |
| `Ma_Limit` | **电机额定相电流**（云台轻载可 **1000mA**） | FOC 闭环堵转最大电流，默认 `3000mA` 偏大；云台负载很轻，调到 1A 左右即可，越小越不烫 |
| `Ma` | 同 `Ma_Limit`（仅开环用） | 开环模式工作电流，默认 `1000mA`；本项目走 FOC 闭环，主要看 `Ma_Limit` |

**供电与电流（可调电源怎么设）**：

- 驱动器供电 **7–32V，官方推荐 24V**。`11.8V`(≈12V) 能跑（云台低速小负载够用），
  但电压越低**高速扭矩越小**、越容易欠压告警；有 24V 档就直接用 24V。
- **两个层面别混淆**：① 驱动器 `Ma_Limit` = **单台电机的相电流**；② 可调电源限流 =
  **两台之和的母线电流上限**。二者不是一回事（母线电流通常远小于相电流）。
- 可调电源限流建议 **≥ 1.5–2.0A**：两台同时动作时若限流太小会被拉进 CC（恒流），
  母线电压塌陷 → 驱动器欠压 → 表现就是「更不动」。**若限流被设在 0.3A，一定要调大。**
- 判据：电源 **CV 灯亮、电压稳在设定值 → 没限流，够用**；**CC 灯亮、电压被拉低 → 已在限流，调到 1.5A 以上**。
  电机外壳 **>70°C** 就下调 `Ma_Limit` 或改善散热。

**共地（GND 参考）—— UART 的隐形前提**：

- **必须共地**。UART 是**单端信号**：逻辑 0/1 是相对 **GND** 判定的。两边各用各的地参考，
  接收端就没有公共基准 → 电平判读错乱，表现为**乱码、或一个字节都收不到**（不是"收到但解析失败"）。
- 接法：MaixCAM2 的 `GND` 与**两台驱动器**的 `GND` 连到同一点（单点/星形接地），即需求文档
  所谓"三路 GND 全连一起"。驱动器的功率 `GND`（与 `V+` 同端子）**就是信号地**，不用另找一根。
- **两个独立电源时尤其注意**：MaixCAM2 由 Type-C/USB 供电、电机由可调电源供电时，二者地并不相通，
  必须把**两个电源的 GND 也连起来**，否则两边地各自浮动，照样不通。
- 万用表验证：① **断电**用通断档测 MaixCAM2 `GND` ↔ 各驱动器 `GND`，应导通；
  ② **上电**测两点间电压应 **≈ 0V（< 50mV）**；读到几百 mV～几 V = 没共地或地线过细/过长压降大。
- ⚠ 共地是**必要条件、不是充分条件**：不共地会乱码，但"共地正确"**不代表** `P_Serial` 已开、
  地址/校验/波特率正确 —— 那几项仍要按上一节逐一核对，别把两种故障混成一种。

**零回帧三步二分法**（板端 `python tools/uart_probe.py`）：

1. 两轴都 `opened` 但发使能收不到任何回帧 → 先怀疑 **`P_Serial` 没设 `UART_FUN`**（其次看地址/校验/波特率）；
2. `--loop yaw`（**先把该轴 TX-RX 短接**）能收回自己发的字节 → 说明 MaixCAM2 侧 UART 正常，问题在驱动器端；
3. 回环也不通 → 查接线（TX↔RX 有没有交叉、GND 有没有共地）或该串口被系统占用。

板端须可导入 `cv2`、`numpy` 和 `maix`。算法代码没有桌面窗口 API。
首次上板顺序：① `CALIBRATE_AXIS="yaw"` 跑一次 → ② `"pitch"` 跑一次 → ③ 把打印出的
`PULSE_PER_PX_*`/`DIR_SIGN_*` 填回 `config.py`、按回差调 `DEADBAND_PX`、`CALIBRATE_AXIS` 置回 `""`
→ ④ 正常上电即进入追踪。未标定就开闭环属于危险操作。

## 激光光斑读数（屏幕左上角面板）

**当前方案（`LASER_MODE="fixed"`）：光斑位置写死，不做任何实时检测。** 每帧唯一的"光斑相关运算"
是 `deviation()` 里的两次减法 —— 黑框中心 `(u,v)` 减去固定光斑 `(LASER_U_NOMINAL, LASER_V_NOMINAL)`，
结果写在屏幕左上角。之所以能写死：相机与激光笔刚性固连（同装在俯仰平台上）→ 云台转动时两者一起动，
**光斑在图像里的位置本来就是常量**（需求文档 §7.1 的"定点跟踪"前提），动的是靶心。

```text
LASER (294,202) [fixed/calib]          ← 写死的光斑位置（不做图像检测）
D-CENTER dx=-32.8 dy=+27.2 px          ← 靶心相对光斑的偏差 = 每帧唯一的运算
```

- **符号约定**：`dx = u靶心 − u光斑`（>0 靶心在光斑右侧）、`dy = v靶心 − v光斑`（>0 靶心在下），
  与 `MaixCAM2串口直控云台需求文档.md` §7.2 完全一致 —— 屏幕上的符号可直接进控制律。
  黑框未检出时显示 `D-CENTER -- (no black rect)`。
- **重新标定 / 备选方案**（`LASER_MODE`）：

  | 模式 | 行为 | 每帧开销 | 用途 |
  |---|---|---|---|
  | `"fixed"`（当前） | 直接用标定常量 | 两次减法 | 比赛/调试的正式口径 |
  | `"detect"` | 每 `LASER_INTERVAL_S` 在标称位 ±`LASER_ROI_RADIUS` 内实测一次光斑质心（限频，其余帧沿用） | 0.08ms / 次 | **重新标定 U/V**：改成 detect 跑一次，把屏幕读到的 `LASER (u,v)` 填回 config；也用于验证"写死的值还准不准" |

- 标定值来自 2026-10-07 现场一帧：光斑约在 **(294, 202)**（640×480，背景灰度 145 / 峰值 177）。
  换镜头、重装激光笔、动了相机/激光的相对装配后必须重新标定。
- `result` 另外带出 `laser_u/laser_v/dx/dy` 四个字段供控制端使用，**串口协议本版不变**（仍是 `@[u,v]%`）；
  黑框无效时不写入这些字段。

## 对外 API（识别黑框 / 追踪中心 / 云台闭环）· 2026-10-08

跨小问复用同一个视觉能力时，**直接 import `target_api`**，不要再去拼 `vision.*` 的路径与参数。
门面模块 `target_api.py` 在应用根目录，顶层不 import maix，PC 上可用 numpy 离线跑。

```python
from target_api import detect_black_frame, CenterTracker, track_center, load_config

# ① 识别黑框：无状态，每帧独立全图搜索
res = detect_black_frame(gray)        # gray 可为 numpy uint8 或 MaixPy image
if res:                               # bool(res) == res.valid
    print(res.center, res.quad, res.stage)

# ② 追踪中心：有状态，窄带跟踪 + 失锁自动重捕获
tracker = CenterTracker()
res = tracker.track(gray)             # 连续调用；stage: track/roi_search/full_search
tracker.reset()                       # 换靶/换场景时清锁定

# ③ 光斑偏差 / 自动瞄准（可选）
t = CenterTracker(with_laser=True)    # res 多出 laser_u/v 与 dx/dy（靶心 − 光斑）
from target_api import AimLoop        # 会驱动 ZDT 电机，须先完成开环标定
```

| API | 语义 | 典型用法 |
|---|---|---|
| `detect_black_frame(frame, cfg=None)` | 无状态识别黑框，返回 `Result`（`valid` 为假即没识别到） | 只在某几问取一次框、离线单帧处理 |
| `CenterTracker(cfg=None, with_laser=False)` | 有状态追踪器，`.track(frame)`；`.reset()` 清锁 | 视频流里持续追踪靶心 |
| `track_center(frame)` / `reset()` | 内部复用默认追踪器的函数式写法 | 一行版快捷调用 |
| `load_config(**overrides)` | 派生一份覆盖参数的配置，**不改磁盘 config.py** | 不同小问用不同阈值 |
| `AimLoop(cfg=None, enable_motors=True)` | 追踪 + 偏差 + 双轴云台闭环，`.step(frame)` | 自动瞄准类小问 |

`Result` 是 dict 子类：`res["u"]` 与 `res.u` 等价，`bool(res)` 即 `res.valid`；
主要字段 `valid / u / v / raw_u / raw_v / center / quad / inner_quad / stage / ms / dx / dy / aim_valid`。

## 可复现验证

```text
python -B tools/build_single.py               # 生成 dist/main_single.py（「运行当前文件」用）
python -B tools/build_single.py --check       # 单文件版是否已过期（改了源码没重生成会报错）
python -B tools/build_single.py --smoke       # PC 自检：嵌入模块是否全部就位、BUILD_ID 是否同步
python -B tools/check_app.py --manifest       # 机检 C1..C10 + 打印应用包内清单
python -B tools/test_performance.py           # 黑框检测：30 合成场景 + 5 反例 + 运动/失锁
python -B tools/test_performance.py --legacy 改动前的rectangle_center.py路径
python -B tools/test_laser.py                 # 激光：默认 fixed 口径 + detect 精度/限频/反光拒绝/耗时
python -B tools/test_gimbal_aim.py            # 云台：协议帧 / 闭环收敛 / 符号反发散 / ACK·E2 / 标定器
python -B tools/test_target_api.py            # 对外 API：识别/追踪/偏差符号/参数覆盖
python -B tools/example_api_usage.py          # 三种小问场景的可运行调用示例
```

测试包含 30 个已知真值的合成场景（透视、42°/45°/90°/135°旋转、暗背景、低曝光、局部反光、噪声），5 个反例，连续移动、突然平移、目标消失及重新出现。真值由独立 OpenCV 投影生成；测试检查未经时间平滑的原始中心，不能用旧坐标混过测试。

本版针对失锁卡顿的关键点是：默认不进入 Hough 恢复；失锁时只在缩小图上做有限阈值/轮廓搜索，搜索耗时在 PC 上约 5～17 ms，不再出现秒级单帧。已加入黑带全宽采样，避免笔记本屏幕等细黑边被锁定。PC 的绝对耗时受系统负载影响，板上以 `[perf]` 日志为准。
