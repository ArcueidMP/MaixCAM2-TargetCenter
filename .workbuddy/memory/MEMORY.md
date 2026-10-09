# 项目长期记忆 —— 25Efork（MaixCAM2 靶心识别）

> 只记"跨会话必须知道、且不在 AGENTS.md 里"的结论；流水见同目录 `YYYY-MM-DD.md`。
> 协作契约 / 目录归属 / 门禁 / git 见仓库根 **`AGENTS.md`（每轮自动注入）**。
> **本文件必须保持精炼（超长会在注入时被截断）**，细节放各轨 README 与专题文档。

## 0 · 当前状态

| 轨 | 目录 | BUILD_ID | 状态 |
|---|---|---|---|
| A（DeepSeek） | 根 | `2026-10-05-r7` | 性能治理 + 抗抖跟踪已完成 |
| B（名义 GPT） | `maixcam2_rect_center/` | `2026-10-10-no-block-v14` | 黑框检测 + 光斑(fixed) + 双轴云台闭环 + **自检后免标定直接追踪**（标定加视觉快速失败防阻塞；见 `B轨云台调参手册.md`） |

- B 轨 v11（10-10）：**上电自检与视觉/标定彻底解耦** —— `CALIBRATE_AXIS="auto"` 时旧代码
  **只建 AutoCalibrate、不建 GimbalSelfTest**，导致"上电无动作、要等画面出现靶心"。
  现改为：初始化**无条件建自检**；主循环顺序 = **人工标定 → 自检 → auto_cal 延迟创建 →
  自动标定 → 追踪解锁**；自检只用链路+时间（**无黑框照走**），但 `aim.state=="FAULT"`
  时仍不发任何指令（重排时新加的门控，勿丢）。`AutoCalibrate` 自检结束后才创建，
  标定超时窗口不被自检耗时挤占。顺序仍是固定脉冲 4 步 **pitch 上→下、yaw 左→右**。
  改动主循环控制流必跑 `tools/test_main_self_test.py`（端到端跑真实 main()）。
- B 轨 v9（10-09）：`CALIBRATE_AXIS="auto"` 自动标定两轴 → 解锁闭环进入追踪；闭环发帧
  带 **ZDT 曲线加减速 `ACC_TRACK=200`**（acc=0 是瞬时启停，会让激光在停下瞬间晃开），
  短行程自动退化为三角速度曲线；`RPM_FINE=120`/`RPM_COARSE=400`。
  `main.py` 发送点必须用 `Config.ACC_TRACK`（`test_track_uses_curve_config` 静态锁死，
  同时是 main.py 被覆盖的探针）。
- 光斑口径（10-07 定死）：`LASER_MODE="fixed"` 全程不检测，每帧仅"黑框中心−常量"两次减法；
  实测光斑 ≈ (294,202)。重新标定才临时切 "detect"。整套旧瞄准代码备份在
  `电赛/25Efork_rollback_backup_2026-10-07_2100/`。
- 对外 API `target_api.py`：`detect_black_frame` / `CenterTracker` / `AimLoop`，
  返回 `Result` 是 dict 子类。闭环发送点也走 `ACC_TRACK`。
- 上板默认流（v11）：上电 → 0.5s 后固定脉冲自检 4 步（pitch 上→下、yaw 左→右，约 6s，
  **不需要靶纸/黑框**）→ 自检完成才开始 `CALIBRATE_AXIS="auto"` 自动标定两轴（此时才需要
  画面里有靶心）→ 成功即解锁闭环进入追踪；任一轴失败保持锁定（宁可不追，不甩头）。

## 1 · 项目一句话

2025 电赛 E 题视觉模块：靶心 = A4 靶纸四周 1.8cm 黑胶带矩形**对角线交点**（射影不变量），
输出 `(u,v)`（A 轨另给 `px_per_cm`）。详见 `AGENTS.md §0`。

## 2 · 改动纪律（AGENTS §3/§3′ 之外的两条）

- 改 `vision/*` / `config.py` / `main.py` / `target_api.py` 后：**改 BUILD_ID →
  `build_single.py` 重生成 → 跑门禁**，顺序不能反。B 轨新增/改名模块必须同步
  `build_single.py` 的 `MODULES` 清单（单文件版按序 exec，被依赖者须在前）。
- `tools/*.jpg`、`screenshot_*.png` 是调试产物：别当源文件改，别进包。

## 3 · 双轨并发

严禁 `git add -A`（B 轨目录随时被别的进程写盘），按路径精确 add，提交带
`deepseek:` / `gpt:` / `repo:` 前缀。
- ⚠ **B 轨 main.py 被外部进程多次整份覆盖**（vision/config 不受影响，只有入口丢）：
  实测退回过 **v6**（10-09）与 **v9**（10-10，距 build 仅 35s）。改完入口**立刻**
  `build_single.py` + `check_app.py --manifest`；**C8（dist 与 main.py 的 BUILD_ID 一致）
  是覆盖探针**；`tools/apply_gimbal_track_patch.py` 的补丁锚点已升级为**多形态**
  （每项 `old` 可为 v6/v9 等若干形态的 tuple，命中任一即替换为当前形态），
  故**一条 `--check` 即可核对、一条无参调用即可从旧形态恢复**。
- ★ **B 轨每轮交 git（10-10 定案）**：覆写是持续的，**git 是唯一可靠回滚点** →
  B 轨每完成一轮（改码→BUILD_ID→build_single→门禁→**立即提交**）即 commit 一次。
  一条命令：`"D:/Python311/python.exe" -B tools/git_round.py -m "gpt: 摘要"`
  （`--check` 只核对；`--rollback <文件>` 看历史+回滚命令）。**只 add `maixcam2_rect_center/`，绝不 `-A`**。
- **B 轨有自己的 `AGENTS.md`**（`maixcam2_rect_center/AGENTS.md`，10-10 新增）= B 轨工作契约：
  工作边界 / **电机轴映射定案表** / 调参方式 / 改完三件事 / 每轮交 git / 常见坑。
  根 `AGENTS.md` 是跨轨契约；**改 B 轨先读 B 轨 AGENTS.md**。B 轨另有 `.gitignore`（子树专属，
  已被 app.yaml exclude）：忽略 `tools/e2e_report/`，**保留 `tools/negative_e*.png`（测试固件）**。

## 4 · A 轨性能（README §三′）

- 症状入口 = `[fps]` 的 `掩膜=N`：稳态个位数（锁定=1）；长期 12/24 = 每帧全扫。
- 铁律：提速只动性能组；动 `MIN_AREA_RATIO`/`SCORE_*`/`MORPH_*` 必跑 `test_hardscene --ab`；
  性能结论必须同进程交错 A/B。`findContours` 代价∝轮廓数（2 万条≈190ms）→
  `boundary_estimate()` 预筛（阈值 13000 实测交叉点）；`_masks` 必须生成器；锁定值排扫描表最前。

## 5 · A 轨抗抖（README §三″）

- 症状入口 = `保持=Y/N`；长期 Y 查**检测**不是滤波器。
- 抖动位移是**必须跟踪的真实运动**；重低通"滤掉"= 照旧位置打（已实测证伪）。
- 改 `TRACK_*`/`KIND_BONUS` 必跑 `test_jitter.py --unit` 看**四项指标**；
  `TRACK_JUMP_*` 别往小调；机动判据扣 `meas_var`；q 映射按对数。

## 6 · MaixCAM 交付门禁（两轨通用）

`"D:/Python311/python.exe" -B tools/check_app.py --manifest`（C1..C10 + 包内清单）。
B 轨包内 **12 文件**（app.yaml/main/config/target_api/assets/vision×7），dist 已排除；
C8 核对 dist 与 main.py 的 BUILD_ID。改源码必更新 BUILD_ID，板端日志与本地逐字一致才算新代码。
WorkBuddy 托管 venv 装不上 numpy/cv2 —— 一律用 `"D:/Python311/python.exe"`。

## 7 · B 轨要点（详见 `B轨代码技术解析与功能分析.md`）

- 差异化 = 像素↔毫米（`rect_to_image` mm→px 单应已写好未用；不输出标量 px_per_cm）。
- **12 个死参数**（改了无效）：`SEARCH_BUDGET_MS` `SEARCH_MAX_PROPOSALS`
  `SEARCH_GEOMETRY_STEPS` `MAX_CONTOURS` `LINE_MAX_PROPOSALS` `LINE_SEARCH_INTERVAL`
  `ACQUIRE_CONFIRM_FRAMES` `CONFIRM_CORNER_PX` `RECHECK_INTERVAL` `MAX_LINE_RESIDUAL_PX`
  `MIN_TRANSITION_COVERAGE` `MAX_BOUNDARY_WIDTH_ERROR`。
- 基线：30 场景检出 29（fixture#2 强反光属安全失败）、max 0.803px、0 误检、稳态 ~1ms。
- 两轨互补：A 轨跟踪/性能成熟，B 轨几何/毫米完整 —— 补短板去另一轨抄已验证结论。

## 8 · 云台/ZDT 速查（详见各专题文档 + 10-09 日志）

- **实测轴映射（v8 定案）**：`ttyS0`(U0,ID=1)=**pitch**、`ttyS2`(U2,ID=2)=**yaw**（与最初
  设想相反）。代码侧已整体互换：`YAW_DEV=ttyS2`+`ADDR_YAW=2`、`PITCH_DEV=ttyS0`+`ADDR_PITCH=1`；
  两轴正方向都是 CCW（yaw CCW=左、pitch CCW=上）。等效方案：对调串口线+ID 设回 1/2。
- 引脚：U0(A28/A29) **必须显式 pinmap**（走空表静默不通）、是系统日志口（开机
  `FLUSH_RX_ON_START` 清 RX）；U2(B0/B1) 需 pinmap；UART1 无外接引脚（勿再踩）；
  备用 U3(B2/B3)/U4(A21/A22)。`PINMAP_STRICT=False` + 引脚回读日志是"某轴不通"第一手证据。
- ZDT 协议：Checksum=0x6B 固定字节；多字节大端；F3 使能/F6 速度/FD 位置(FD mode0 相对
  mode1 绝对)/FE 停止；dir 0x00=CW 0x01=CCW；16 细分 3200 脉冲=1 圈；返回 02 成功/E2 未使能堵转。
- **acc 档位语义（v5 实测定案）**：位置/速度模式 `acc=0` = 不用曲线、**直接以设定速度启停**
  （= 卡顿根源）；≠0 时每 `(256-acc)*50us` 变 1RPM，**acc 越小越柔和**（常用 20-80）。
  短行程自动退化三角曲线。`estimate_move_seconds()`（stepper_link）按官方公式建模。
- 驱动器菜单必设（否则串口完全无效）：`P_Serial=UART_FUN`（出厂 RxTx_OFF！）、`UartBaud=115200`、
  `ID_Addr`、`Checksum=0x6B`、`Response=Receive`、`MotType=1.8°`、`MStep=16`、`P_Pul=PUL_OFF`；
  未做 `Cal` 的板子只有 6 项菜单，先 Cal（空载）解锁完整菜单；`En` 不接默认一直使能，"有劲"≠"串口通"。
- `_ingest()` 无故障返回 `fault=False`：判断必须走 `stepper_link.fault_reason()`（None=无故障），
  写 `is not None` 会让开机第 1 帧就误判 FAULT（已修，测试锁死）。
- 五条回头路：① 闭环=定点跟踪（光斑几乎不动，不需小车位置/视差标定）；② `PULSE_PER_PX`
  必须实测（走 2000 脉冲看 Δpx，吸收齿轮比/细分/焦距）；③ 主用相对位置+增量比例律+单帧限幅；
  ④ 死区 ≥1.5×回差（3D 打印齿轮回差 1°≈9.7px）；⑤ 闭环必须用 raw_u/raw_v 并前馈底盘角速度
  （CENTER_ALPHA 平滑有 ≈28ms 滞后）。
- 符号：`dx=u靶心−u光斑` → 误差为正发 CCW；实测反了只改 `DIR_SIGN_*`。
- 未决：FAULT 需手动重启恢复（有意保守）；ENABLE_SCAN 默认关（防上电甩头）；
  光斑极性（405nm 打紫外感光纸可能是暗斑，fixed 模式已绕过）。

## 9 · 下一步（板端）

上板验证 v11 上电流程：① 上电应立即看到 4 步自检（pitch 上→下、yaw 左→右，约 6s），
**此时画面里可以没有靶纸**；② 日志出现 `[gimbal] 上电自检完成 -> 开始自动标定` 之后
才需要把靶纸/靶心放进画面；③ 标定成功后打印"闭环已解锁，开始跟踪"，随后追踪起停
应柔和（`ACC_TRACK=200` 短行程三角曲线）；④ 若停下瞬间仍晃 → `ACC_TRACK` 调小（更柔）
或 `RPM_FINE` 再降；⑤ P4 依实测回差调 `DEADBAND_PX`；P7 移动中瞄准需 3507 经 SPI
给角速度前馈（接口预留）。
