# AGENTS.md · 轨道 B（maixcam2_rect_center）协作契约

> 本文件是**轨道 B 自己的工作契约**：本轨负责什么、电机怎么接/怎么调、改完必须做什么、
> **怎么每轮交 git 以便回滚**。面向在本目录工作的 AI 编码代理。
>
> 三层文档各管一段，别记混：
> - **根 [`../AGENTS.md`](../AGENTS.md)** —— 跨轨公共契约（双轨分工、仓库级 git 规范、打包口径）；
> - **本文件** —— 轨道 B 的工作细则（**改本目录前先读这一份**）；
> - **同目录 [`README.md`](README.md)** —— 人类可读的算法与上板说明；
> - **同目录 [`B轨云台调参手册.md`](B轨云台调参手册.md)** —— 云台每个参数的逐条说明与调法。
>
> 冲突时：**本文件对"B 轨工作细则"生效**，根 AGENTS.md 对"跨轨协作"生效，README 对"算法与上板"生效。

---

## 0. 本轨是什么

2025 全国大学生电子设计竞赛本科组 **E 题《简易自行瞄准装置》** 视觉模块的**第二实现**
（与根目录的 A 轨 DeepSeek 版并行）。技术路线：**Canny + `approxPolyDP` 凸四边形**配对内外框 +
四边灰度剖面校验 → **显式 8×8 单应**解算 → 正视面上取 **72 点圆**逆映射回原图求靶心，
输出 `(u, v)`；另带**激光光斑读数面板**与**双轴 ZDT 云台闭环瞄准**，对外提供 `target_api.py`。

- 应用入口：`main.py`（自带 `app.yaml`，`id = maixcam2_rect_center`，`app_id` 与 A 轨不同）。
  **v0.4.6 起入口已按功能模块化**：`main.py` 只留 BUILD_ID + 装配 + 主循环；识别线程、显示层、
  闭环调度、相机、帧率计、日志组装各自成 `vision/` 下的独立模块
  （`rate_meter` / `camera_source` / `vision_worker` / `preview` / `aim_control` / `telemetry`）。
- 应用包 = **18 文件**（`app.yaml` / `main.py` / `config.py` / `target_api.py` / `assets/icon.png` /
  `vision/` 下 13 个 `.py`）；`tools/` `dist/` `*.md` 不进包。

---

## 1. ★ 工作边界（先看这条，最容易出事）

1. **只改 `maixcam2_rect_center/` 目录内的一切**。绝对不要"顺手"去改根目录的
   `main.py` / `config.py` / `vision/` / `app.yaml` / `tools/` —— 那是 **A 轨**的地盘。
2. **不要把自己打进 A 轨的应用包**，也不要把 A 轨文件 import 进本轨（两条轨各自独立运行）。
3. **两条轨道不能同时装到板子上、也不能同时跑**：`app_id` 与入口都不同，同跑会互相干扰
   日志与串口，误判"改了没用"。**同一时刻只调试一条。**
4. **`main.py` 是唯一反复被整份覆写的文件**（见 §5）—— 本项目两轨共用一个工作树、并发写盘，
   **不要与其它进程/会话/编辑器同时写 `maixcam2_rect_center/main.py`**。
   v0.4.6 模块化后 `main.py` 由 923 行降到约 296 行，实现下沉到从未被覆写的 `vision/` 下，
   整份覆写的损失面缩小约 70% —— 但入口仍是覆写高发区，改完**立刻** `build_single.py` + 门禁。
5. **确需跨轨改动**（根 AGENTS.md / 根 README.md 等公共文档）时，**先说清意图、只做最小改动**，
   并在提交信息里点明（前缀用 `repo:`）。

---

## 2. ★ 电机顺序与轴映射（2026-10-09 实机测定的**定案**，别凭直觉改）

> 起因：最初以为 "U0=yaw、U2=pitch"，实机发现**正好相反**。判定依据：自检 4 步发出的命令是
> `pitch, pitch, yaw, yaw`，而实际动作是 `右 → 左 → 下 → 上`（右/左 = yaw，下/上 = pitch）
> —— 前两步同轴反向、动的是 yaw，后两步动的是 pitch。**代码里叫 "yaw" 的必须真的驱动 yaw 轴**，
> 否则闭环的 `PULSE_PER_PX_*` / `DIR_SIGN_*` 会张冠李戴 → 必发散。

| 代码里的轴 | 串口 | 丝印 | 引脚 | 设备路径 | 驱动器 ID | pinmap |
|---|---|---|---|---|---|---|
| **pitch** | UART0 | `U0T`/`U0R` | A28 / A29 | `/dev/ttyS0` | **1** | ★ **必须显式 pinmap**（走空表会静默不通；⚠ 也是系统日志口） |
| **yaw** | UART2 | `U2T`/`U2R` | B0 / B1 | `/dev/ttyS2` | **2** | 需 pinmap 切到 `UART2` 功能 |

对应的 `config.py` 落点（改硬件前先看这里）：

| 配置项 | 值 | 说明 |
|---|---|---|
| `PITCH_DEV` / `PITCH_PINS` | `/dev/ttyS0` / `{"A28":"UART0_TX","A29":"UART0_RX"}` | `config.py:51–52` |
| `YAW_DEV` / `YAW_PINS` | `/dev/ttyS2` / `{"B0":"UART2_TX","B1":"UART2_RX"}` | `config.py:45–46` |
| `ADDR_PITCH` / `ADDR_YAW` | `1` / `2` | `config.py:56`（**地址跟着电机走，不是跟着端口**） |
| `PINMAP_STRICT` | `False` | `config.py:49`：pinmap 失败只警告、仍打开 UART，避免把真故障藏成"打不开" |
| `FLUSH_RX_ON_START` | `True` | `config.py:54`：开机清空 RX（U0 有内核启动日志/终端回显） |

**方向（`config.py:103–108`）**：两轴的**正方向都是 CCW** —— `yaw CCW = 左 / CW = 右`，
`pitch CCW = 上(仰) / CW = 下`。故 `YAW_POSITIVE_CCW = PITCH_POSITIVE_CCW = True`，
自检序列 = **pitch 上→下**、**yaw 左→右**（各回到起始位姿）。

**闭环方向符号（`config.py:63–64`）**：`DIR_SIGN_YAW = DIR_SIGN_PITCH = -1`，是**按几何推断**的
（相机与激光同装在云台上 ⇒ 云台转过去时靶心在画面里**反向**移动）。
⚠ **未经标定**：上电追踪时若**越追越偏**（误差持续变大、云台一路跑向一侧），
把对应那项改成 `+1`；`DIVERGE_GUARD` 会自动兜底停机，不会一路撞到底。

### 等效的另一种修法（只选一种，二者不要都做）

把**两根串口线对调**，并把驱动器菜单 `ID_Addr` 设回 `yaw=1 / pitch=2`。
那时把上面 `PITCH_DEV`/`YAW_DEV`/`ADDR_*` 改回"yaw=U0/ttyS0/1、pitch=U2/ttyS2/2"即可。
**代码侧与接线侧只需一处对上** —— 本目录现在走"代码侧互换"这条路（好处：U0 口腾给了 pitch，
但代价是 U0 是系统日志口，pitch 上会混内核日志噪声；介意就改接线）。

### 驱动器菜单必设（否则串口完全无效）

`P_Serial = UART_FUN`（出厂是 `RxTx_OFF`！）、`UartBaud = 115200`、`ID_Addr`（见上表）、
`Checksum = 0x6B`、`Response = Receive`、`MotType = 1.8°`、`MStep = 16`、`P_Pul = PUL_OFF`。
未做过 `Cal` 的板子只有 6 项菜单 —— 先空载 `Cal` 一次解锁完整菜单。

---

## 3. 调参方式（现场只改 `config.py` 一个文件）

### 3.1 追踪手感（现场最常动，`config.py:65–93`「★ 追踪手感」段）

换算链：**脉冲 = |误差px| × `DEG_PER_PX_x` × `PULSES_PER_DEG_x` × `GAIN_x`**（再受 `MAX_STEP_PULSES` 限幅）。

| 参数 | 当前 | 作用 / 怎么调 |
|---|---|---|
| `DEG_PER_PX_YAW` / `_PITCH` | 0.30 | ★ **主参数**：图像 1px 偏差 → 云台转几度。**越大越快也越易超调**；想更柔和先动这个。**置 0 = 回退到 `PULSE_PER_PX_*`** |
| `PULSES_PER_DEG_YAW` / `_PITCH` | 8.89 | 每度多少脉冲 = `3200 × 减速比 ÷ 360`；装了 i:1 减速就填 `8.89 × i` |
| `GAIN_YAW` / `_PITCH` | 0.3 | 单帧只修正误差的这个比例（必须 ≤1，0.2~0.4 稳妥） |
| `GAIN_COARSE` / `COARSE_PX` | 0.5 / 24.0 | `|e| > COARSE_PX` 时用大步增益（同样 ≤1） |
| `RPM_FINE` / `RPM_COARSE` | 60 / 150 | 精调 / 粗调段转速 |
| `ACC_TRACK` | 80 | **闭环专用**曲线加减速档位；`acc=0` 会瞬时启停。越小越柔 |
| `DEADBAND_PX` | 3.0 | 死区，应 **≥1.5× 实测回差**（否则在齿隙里来回抖） |
| `MAX_STEP_PULSES` | 60 | 单帧限幅，防误检猛甩；输出另经过低通/增长限速 |
| `COMMAND_INTERVAL_S` | 0.10 | 发帧间隔；太小会指令堆叠成"冲过头" |
| `AIM_OUTPUT_LPF_TAU_S` | 0.25 | 运动量增长的时间低通；缩小/死区/换向立即生效 |
| `AIM_OUTPUT_SLEW_PPS` | 120 | 单次运动量每秒最多增加的脉冲数 |
| `AIM_WAIT_MOTION` / `AIM_MOTION_MARGIN_S` | True / 0.03 | 按曲线运动时间估算等待，防尚未完成的指令重叠 |

> **改手感一次只动一个数**（推荐先只改 `DEG_PER_PX_*`），跑一轮看效果再动别的 ——
> 一次改多个会分不清是谁起的作用。逐条说明与反算方法见 `B轨云台调参手册.md`。

### 3.2 上电时序（`main.py` 主循环）

> ★ **2026-10-11 变更（v0.4.1）**：取消"上电自检优先级最高、开机自动跑"。自检改为
> **屏幕右上角可点击按钮 `SELF-TEST`**，点一下才触发（`Config.ENABLE_TOUCH_BUTTON`）。
> 上电流程 = 使能 → 建显示/触摸 → 进入主循环（识别 + 闭环）；**上电不发自检脉冲**。
> 画面由 `main._Preview` 在主进程内按"屏幕尺寸画布"letterbox 后 `display.show()` 流转到
> MaixVision（`[preview] mode=in-process WxH`）。下述"上电自检 4 步自动跑"的旧描述**已作废**，
> 仅保留 SELF_TEST_PULSES/固定脉冲的口径供按钮复用。

旧口径（已作废，仅供对照）：

```
上电 → 使能 → 自检 4 步（pitch 上→下、yaw 左→右，固定脉冲，约 3~4s）
                        ↑ 不需要靶纸/黑框、不依赖标定；有 10s 防死锁兜底
     →（仅当 CALIBRATE_AXIS=="auto"）自动标定两轴
     → 追踪（闭环每 COMMAND_INTERVAL_S 发一帧，走 ACC_TRACK 曲线）
```

- **自检与视觉/标定彻底解耦**（调度在 `vision/aim_control.py` 的 `VisionControl._control`，
  主循环在 `main.py`）：
  无论画面里有没有靶心，上电都会先走完 4 步固定脉冲自检。
- **标定不再阻塞**：`CALIBRATE_VISUAL_TIMEOUT_S = 3.0`（`config.py:209`）—— 连续 3s 收不到有效
  `dx/dy` 就立即失败并**交还主流程**，不会干等 20s（现场"抬头后黑框出视野 → 卡住"就是这么来的）。
- **当前默认不做自动标定**：`CALIBRATE_AXIS = ""`（`config.py:194`）+ `GIMBAL_CALIBRATED = True`
  （`config.py:142`），直接用手感宏换算，**自检一完就进追踪**。
  若要重新标定：临时改回 `"auto"`（或 `"yaw"`/`"pitch"` 单轴），标完再置回 `""`。
- **自检幅度/手感**（`config.py:113–125`）：`SELF_TEST_PULSES = 400`（固定脉冲，**不依赖标定**）、
  `SELF_TEST_RPM/ACC = 60/60`（慢 + 曲线）、`SELF_TEST_SETTLE_S = 0.3`、
  `SELF_TEST_TIMEOUT_S = 10.0`（**防死锁兜底，不是性能目标**）。
  置 `SELF_TEST_PULSES = 0` 才回退到"按 `*_PULSES_PER_DEG` 换算角度"的老路径。

### 3.3 光斑读数（`config.py:269–295`）

`LASER_MODE = "fixed"`（**默认写死**）：全程不检测，每帧只算"黑框中心 − 常量"两次减法，
常量 = `LASER_U_NOMINAL/LASER_V_NOMINAL = (294, 202)`。重装激光笔/换镜头后要重标：
临时改 `"detect"`，把屏幕左上角读到的 `LASER (u,v)` 填回这两行。

---

## 4. 改完**必须**做的三件事（对齐根 AGENTS.md §3′）

1. **提升 `main.py` 顶部 `BUILD_ID`**（当前 `2026-10-12-motion-predict-v0.5.2`，格式 `YYYY-MM-DD-<slug>-vN`）。
   板端会打印 `[rect-center] <BUILD_ID> ...`，**与本地逐字一致才算跑的是新代码**；
   不更新就无法判断是否生效。**改了 `main.py`/`config.py`/`vision/*`/`target_api.py` 任一个都要做。**
2. **重新生成单文件版**（`dist/` 不进应用包，但"运行当前文件"要用它）：
   ```bash
   "D:/Python311/python.exe" -B tools/build_single.py           # 生成 dist/main_single.py
   "D:/Python311/python.exe" -B tools/build_single.py --check   # 单文件是否已过期
   "D:/Python311/python.exe" -B tools/build_single.py --smoke   # PC 自检：嵌入模块是否全部就位
   ```
   ⚠ **新增/改名 `vision/*.py` 或根级模块后，必须同步 `tools/build_single.py` 的 `MODULES` 清单**。
3. **跑门禁**（一律在 `maixcam2_rect_center/` 下、用 `"D:/Python311/python.exe"`）：
   ```bash
   "D:/Python311/python.exe" -B tools/check_app.py --manifest   # C1..C10 机检 + 打印包内清单（应 18 文件）
   "D:/Python311/python.exe" -B tools/test_performance.py       # 视觉回归（基线：29/30、max 0.803px、0 误检）
   "D:/Python311/python.exe" -B tools/test_target_api.py        # 对外 API 回归
   "D:/Python311/python.exe" -B tools/test_gimbal_aim.py        # 云台协议/闭环/看门狗/标定/自检/低通（当前 PASS(29)）
   "D:/Python311/python.exe" -B tools/test_main_self_test.py    # 端到端跑 main()：上电自检优先 / FAULT 阻断
   "D:/Python311/python.exe" -B tools/test_runtime_performance.py  # 显示隔离 + 实测显示帧率 + 搜索/串口有界
   ```
   改光斑（`laser_spot.py` / `config.LASER_*`）还要跑 `tools/test_laser.py`；
   改视觉阈值（`MIN_*`/`MAX_*`/`SCORE_*` 类）**必须**确认 `test_performance` 无回归；
   改主循环控制流（自检/标定/追踪顺序）**必须**跑 `tools/test_main_self_test.py`；
   改显示/预览（`AsyncPreview`、`config.SHOW_*`）或搜索预算 **必须**跑 `tools/test_runtime_performance.py`。

> 只有 `config.py` 需要现场改。修改前先想清楚是否真该动阈值 —— 优先调曝光/位置，别一上来放宽判据。

---

## 5. ★ 每轮交 git（本项目最痛的坑：**入口会被并发覆盖**）

### 5.1 为什么必须每轮交

`maixcam2_rect_center/main.py` 在本项目被**整份覆写了多次**（另一个会话 / IDE 未保存标签页 /
MaixVision 烧录 / 并行代理），写入者用的都是"上一轮"的快照。典型症状：

| 时间 | 现象 |
|---|---|
| 22:46 | `main.py` 退回 v4（自动标定接线整段消失） |
| 23:01:51 | 退回 v5（**距生成 dist 仅 6 秒**） |
| 23:16 | 退回 v6（接线在，仅构建号旧） |
| 次日 | 退回更早版本，`target_api.py` 的闭环发送点丢 `ACC_TRACK` |

`vision/*` / `config.py` 从不受影响 —— **只有入口文件会丢**，迷惑性极强
（"代码明明写了，上电却不生效"）。**`git` 是目前唯一可靠的回滚点**，所以**每完成一轮就提交一次**。

### 5.2 一条命令提交（推荐）

```bash
# 在 maixcam2_rect_center/ 下
"D:/Python311/python.exe" -B tools/git_round.py --check              # 只检查：BUILD_ID / build_single / patch / 待提交清单
"D:/Python311/python.exe" -B tools/git_round.py -m "gpt: <摘要>"     # 检查通过后精确 add 本轨目录并 commit
"D:/Python311/python.exe" -B tools/git_round.py --rollback main.py   # 打印"回滚 main.py"的命令（不自作主张执行）
```

脚本**默认只 add `maixcam2_rect_center/`**，绝不用 `git add -A`（避免卷入另一条轨的半成品）。
需要连记忆一起提交时加 `--with-memory`。

### 5.3 手工提交（与根 AGENTS.md §5 同口径）

```bash
# 仓库根 = 25Efork/（不是上层 电赛/）；提交前先看暂存区只含自己的路径
git status --short
git add maixcam2_rect_center/
git commit -m "gpt: <摘要>"
```

- **提交前缀 `gpt:`**（本轨一切）；纯公共文档改动用 `repo:` / `docs:`；
- **绝不 `git add -A` / `git add .`**（并发写盘，会提交别人写到一半的文件）；
- **提交时机：门禁全绿后立即提交** —— 拖太久可能被并发覆盖，把坏版本入库；
- 仓库已配 `core.quotepath=false` / `core.autocrlf=false`，`.gitattributes` 强制 `eol=lf`（**别改**）。

### 5.4 回滚

```bash
git log --oneline -15 -- maixcam2_rect_center/main.py           # 看该文件的提交历史
git checkout <sha> -- maixcam2_rect_center/main.py              # 把某文件恢复到某次提交
git revert <sha>                                                # 撤销某次提交（保留历史，更安全）
```
恢复后**必须重跑** `tools/build_single.py --check` + `tools/check_app.py --manifest`（C8 会核对
dist 与 `main.py` 的 `BUILD_ID` 是否一致），确认没有半新半旧。

### 5.5 进库 / 不进库

- ✅ 进库：应用本体（`app.yaml`/`main.py`/`config.py`/`target_api.py`/`vision/*.py`/`assets/`）、
  `tools/*.py`、`tools/negative_e*.png`（**测试输入固件**）、`dist/main_single.py`、本目录 `*.md`；
- ❌ 不进库：`__pycache__/`、可重新生成的调试图（`tools/e2e_report/*.png`、`screenshot*.png`、
  `frame*.png`、`mask*.png`、`debug_*.png`）—— 由本目录 `.gitignore` 兜底。

---

## 6. 常见坑（都真实踩过）

1. **「运行当前文件」只下发一个文件** → `ModuleNotFoundError: No module named 'config'`。
   改用「运行项目」，或先 `build_single.py` 再跑 `dist/main_single.py`。
2. **MaixVision 烧录会把 `main.py` 改回旧内容** → 看起来"改了没用"，其实板子在跑旧代码。
   上板前跑 `build_single.py --check`（会当场报过期）；改完在 MaixVision 里关掉再重开该文件。
   保留防覆盖脚本 `tools/apply_gimbal_track_patch.py`（`--check` 核对 / 无参一键补齐）。
3. **灰度形状**：MaixCAM2 的 `image2cv()` 对灰度图返回 `(H,W,1)` 三维哑轴，绝不能按 `ndim==2` 判灰度。
4. **U0 引脚必须显式 pinmap**：`A28/A29` **并非默认复用**成 UART0，走空表会静默不通
   （这是"yaw 一直不动"的历史根因之一）；打开后用 `pinmap.get_pin_function()` 回读实测值打进启动日志。
5. **位置模式 `acc=0` = 不用曲线、瞬时启停** → 卡顿/冲击。自检与闭环都要给非 0 的 acc
   （`SELF_TEST_ACC` / `ACC_TRACK`），且 **acc 越小越柔**。
6. **标定依赖视觉会阻塞**：抬头后黑框出视野 → 卡在等 `dv`。已加 `CALIBRATE_VISUAL_TIMEOUT_S` 快速失败。
7. **`app.yaml` 的 `exclude` 漏目录** → 文档/嵌套目录混进应用包；改完核 `check_app.py --manifest`。
8. **调试抓屏会迅速堆爆工作区**（`tools/e2e_report/*.png` 实测数小时内 4.7M→9.6M）→
   定期跑 `"D:/Python311/python.exe" -B tools/clean_artifacts.py`（扫描）/ `--apply`（删除），
   目标保持 B 轨目录 **≤5MB**。该脚本只删可再生/已忽略产物，**保留 `negative_e*.png` 测试固件**。
9. **显示线程必须在主进程内、且与"子进程"严格区分**：v0.4.3 起默认 `SHOW_ASYNC=True`，把 `show()`
   移到**主进程内的 daemon 线程**（`vision/preview.py` 的 `Preview._show_loop`），主循环只投递不等待
   —— 已实测生效（启动日志 `[preview] mode=display-thread`），**v18 的"默认必须走 `SHOW_ASYNC=False`"
   结论已被推翻**。⚠ 但**不要把 `Display()` 放进子进程**：子进程起不来就 `can_submit()` 恒 False，
   一帧都不画且无报错（历史 v16 的坑，与本条无关）。若 MaixVision 看不到画面：先看 `[preview] mode=`，
   必要时临时 `SHOW_ASYNC=False` 退回主进程内同步 show 以排除显示线程因素。

---

## 7. 完成定义（DoD）

- **改完源码**：`BUILD_ID` 已更新 → `dist/main_single.py` 已重生成 → 门禁全绿
  （`check_app --manifest` 应为 **18 文件 / 0 FAIL**）→ **按精确路径提交**（`gpt:` 前缀）。
- **上板**：日志 `[rect-center] <BUILD_ID>` 与本地一致；上电应看到 **4 步自检**（pitch 上→下、yaw 左→右），
  之后（若 `CALIBRATE_AXIS="auto"`）标定、然后进入追踪；追踪起停应柔和、停下时激光不晃。
- **留痕**：往 `../.workbuddy/memory/YYYY-MM-DD.md` **追加**一行本轮做了什么、怎么验证的
  （只记有长期价值的结论与坑，不记流水）；跨会话必须知道的约定 → `../.workbuddy/memory/MEMORY.md`。
