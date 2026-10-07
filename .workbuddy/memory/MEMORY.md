# 项目长期记忆 —— 25Efork（MaixCAM2 靶心识别）

> 只记"跨会话必须知道、且不在 AGENTS.md 里"的结论；流水见同目录 `YYYY-MM-DD.md`。
> 协作契约 / 目录归属 / 门禁 / git 见仓库根 **`AGENTS.md`（每轮自动注入）**。
> **本文件必须保持精炼（超长会在注入时被截断）**，细节放各轨 README 与专题文档。

## 0 · 当前状态（先读这一节）

| 轨 | 目录 | BUILD_ID | 状态 |
|---|---|---|---|
| A（DeepSeek） | 根 | `2026-10-05-r7` | 性能治理 + 抗抖跟踪已完成 |
| B（名义 GPT） | `maixcam2_rect_center/` | `2026-10-07-laser-readout-v1` | 黑框检测 + **新增激光光斑读数面板** |

- 10-07 21:15 曾把 B 轨整套「激光瞄准」摘掉（`vision/aiming.py`、`tools/test_aiming.py`、
  `config.UART_PROTOCOL` 与旧 `LASER_*`、main 的 `[aim]` 接线）。**整体备份在
  `电赛/25Efork_rollback_backup_2026-10-07_2100/`，要恢复瞄准从这里取。**
- 10-07 21:45 按用户要求**新增**只读的 `vision/laser_spot.py`（不控制电机），
  **22:05 起用户定死为"光斑写死"口径**：`LASER_MODE="fixed"`（默认）→ 全程不做图像检测，
  每帧唯一运算 = `deviation()` 两次减法（黑框中心 − 固定光斑），屏幕左上角显示
  `LASER (294,202) [fixed/calib]` 与 `D-CENTER dx dy px`。标定/校验时才临时切 `"detect"`
  （标称位 ROI±64 实测、`LASER_INTERVAL_S=0.30` 限频、跳变门限、面积上下限挡反光）。
  **现场实测光斑 ≈ (294, 202)**（640×480，背景 145 / 峰值 177）。回归 `tools/test_laser.py`
  19 项全过（含 T0：出厂默认必须是 fixed）；`check_app.py --manifest` 52 OK/0 FAIL；
  **应用包 = 7 文件**。串口协议仍是 `@[u,v]%\r\n`；`result` 额外带
  `laser_u/laser_v/dx/dy`（黑框无效时不写；`found=False` 时 dict 里的 u/v 是标称占位，别当测量用）。

## 1 · 项目一句话

2025 电赛 E 题视觉模块：靶心 = A4 靶纸四周 1.8cm 黑胶带矩形的**对角线交点**（射影不变量，
不需要解单应）；输出 `(u,v)`（A 轨另给 `px_per_cm`）。详见 `AGENTS.md §0`。

## 2 · A 轨改动纪律（AGENTS §3/§4 之外容易忘的两条）

- 改 `vision/*` / `config.py` / `main.py` 后：改 `BUILD_ID` → `tools/build_single.py` 重生成
  `dist/main_single.py` → 跑门禁，顺序不能反。
- `tools/*.jpg`、`screenshot_*.png` 是脚本/调试产物：别当源文件改，别进包。

## 3 · 双轨并发

同库不同目录、双轨都跟踪；**严禁 `git add -A`**（B 轨目录随时被别的进程写盘），按路径精确 add，
提交信息带 `deepseek:` / `gpt:` / `repo:` 前缀。细节 `AGENTS.md §1/§5`。

## 4 · A 轨性能预算（机理与实测见 `README.md §三′`）

- 症状入口 = 板端 `[fps]` 行末尾 **`掩膜=N`**：稳态个位数（锁定 = 1）；长期 12/24 = 每帧全扫。
- 铁律① 提速只动性能组（`MAX_MASKS_FULL` / `FRAG_BOUNDARY_EST` / `MAX_SCAN_*` / `ROI_TRACK*`）；
  动 `MIN_AREA_RATIO` / `SCORE_*` / `MORPH_*` 必须重跑 `test_hardscene.py --ab`。
- 铁律② 性能结论必须**同进程交错 A/B**（跨时间对比被机器负载污染，实测差 7 倍）。
- 别踩回去：`findContours` 代价 ∝ 创建出的轮廓对象数（2 万条 ≈190ms）→ 用 `boundary_estimate()`
  预筛（阈值 13000 是两条路径的实测交叉点）；`MORPH_SEARCH=True` 时 lvl=1 档不做开运算（最易爆）；
  `_masks` 必须是生成器；锁定值必须排扫描表最前（否则 `force_sweep` 每帧 24 掩膜）。

## 5 · A 轨抗抖动（见 `README.md §三″`）

- 症状入口 = `[fps]` 的 **`保持=Y/N`**；长期 Y 说明该查**检测**（阈值/曝光/干扰物）不是滤波器。
- 核心认知：抖动位移是**必须跟踪的真实运动**，不是噪声；用重低通"滤掉"它 = 照几帧前的位置打
  （旧 One Euro(static) 让中心偏 8~27px，已实测证伪）。
- 改 `TRACK_*` / `KIND_BONUS` 必跑 `tools/test_jitter.py --unit`，看**四项指标**（互相拉扯，只看一项必调坏）；
  `TRACK_JUMP_*` 别往小调（级联锁死可漂 200px）；机动判据要扣掉 `meas_var`；q 映射必须按对数。

## 6 · MaixCAM 交付门禁（两轨通用）

规范正文 `maixcam2_rect_center/MaixCAM应用代码规范.md`（R-1..R-8）；可执行门禁 `tools/check_app.py`（C1..C10）：

```bash
"D:/Python311/python.exe" -B tools/check_app.py --manifest    # 机检 + 打印应用包内清单
```

三条硬规则：① `app.yaml` 的 `exclude` 是黑名单，**打完必须核包内清单**（曾漏 `screen_check.png`）；
② `image2cv` 按实际 `shape` 判通道（MaixCAM2 灰度是 `(H,W,1)`，不能靠 `ndim==2`）；
③ 改源码必须更新 `BUILD_ID`，板端日志与本地逐字一致才算新代码。

## 7 · B 轨要点（详细见 `B轨代码技术解析与功能分析.md`）

- 差异化能力 = 像素↔毫米：`result["rect_to_image"]` 是 mm→px 单应，`inverse_map_points()` /
  `homography_8x8()` 已写好未被调用；**不输出**标量 `px_per_cm`（有 36% 方向性误差）。
- **12 个死参数**（改了无效）：`SEARCH_BUDGET_MS` `SEARCH_MAX_PROPOSALS` `SEARCH_GEOMETRY_STEPS`
  `MAX_CONTOURS` `LINE_MAX_PROPOSALS` `LINE_SEARCH_INTERVAL` `ACQUIRE_CONFIRM_FRAMES`
  `CONFIRM_CORNER_PX` `RECHECK_INTERVAL` `MAX_LINE_RESIDUAL_PX` `MIN_TRANSITION_COVERAGE`
  `MAX_BOUNDARY_WIDTH_ERROR`。
- 最大性能缺口：`_candidate_quads` 对全部轮廓算 `boundingRect`/`contourArea`，无上界（噪声画面 7.49ms）。
- 基线（PC，`tools/test_performance.py`）：30 场景检出 **29**（仅 fixture#2 强反光，属安全失败）、
  max 中心误差 **0.803px**、5 负样本 **0 误检**、稳态 0.85ms/帧。
- 两轨互补：A 轨跟踪/性能成熟，B 轨几何/毫米度量完整 —— 补短板先去另一轨抄已验证结论。

## 8 · 下一步：串口直控步进云台（文档 `MaixCAM2串口直控云台需求文档.md`）

硬件（用户已定死）：MaixCAM2 当瞄准主控，**UART1→1 号电机(yaw)、UART2→2 号电机(pitch)** 各自直驱
ZDT Emm_V5.0 闭环步进；与 MSPM0G3507 走 **SPI**（所以两条 UART 全给电机）；相机与激光同装俯仰平台。

- 引脚：`ttyS1`=UART1(A30 TX/A31 RX，需 pinmap)、`ttyS2`=UART2(B0 TX/B1 RX，免 pinmap)、
  `ttyS0` 是系统口**禁用**；SPI 只能用 SPI2(B18/19/20/21)；115200 8N1。
  ⚠ B 轨 `config.UART_PORT` 现在仍是错的 `/dev/ttyS0`。
- ZDT：`Checksum=0x6B` 时校验字节就是固定 0x6B；多字节**大端**；`F3` 使能 / `F6` 速度 /
  `FD` 位置(mode 0 相对 1 绝对) / `FE` 停止；`dir` 0x00=CW、0x01=CCW；**增量闭环必须 acc=0**；
  16 细分下 3200 脉冲 = 电机 1 圈；返回 `02` 成功 / `E2` 未使能或堵转。
- 五条最容易走回头路：① 闭环 = **定点跟踪**（光斑在图像里几乎不动 → 不需要小车位置、不需要视差标定）；
  ② `PULSE_PER_PX` **必须实测**（走 2000 脉冲量 dx 变化，一次性吸收齿轮比/细分/焦距），顺手算交叉项
  查相机横滚；③ 主用**相对位置模式 + 增量比例律**并加单帧脉冲限幅；④ **死区 ≥ 1.5×回差**
  （3D 打印齿轮回差 1°≈9.7px，直接吃掉整个预算）；⑤ 闭环必须用 `raw_u/raw_v` 并**前馈底盘角速度**
  （`CENTER_ALPHA=0.70` 滞后≈28ms，底盘 90°/s 时 ≈6.6cm@1.5m）。
- 符号：`dx = u靶心 − u光斑`、`dy = v靶心 − v光斑` → 两轴统一"**误差为正发 CCW(0x01)**"；
  实测反了只改 `DIR_SIGN_*`，别改接线、别再加负号。
- 未决前置：① **光斑极性**（405nm 打紫外感光纸可能是暗斑，现实现只认亮斑）；② 大偏差先粗瞄
  再切检测；③ 接串口云台前需把 `aiming` 从备份取回或按文档重写。
