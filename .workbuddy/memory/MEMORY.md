# 项目长期记忆 —— 25Efork（MaixCAM2 靶心识别）

> 只记"跨会话必须知道、且不在 AGENTS.md 里"的结论；流水见 `YYYY-MM-DD.md`。
> 协作契约 / 目录归属 / 门禁 / git 见仓库根 **`AGENTS.md`（每轮自动注入）**。
> **本文件须精炼（超长会被注入截断）**，细节放各轨 README 与专题文档。

## 0 · 当前状态

| 轨 | 目录 | BUILD_ID | 状态 |
|---|---|---|---|
| A（DeepSeek） | 根 | `2026-10-05-r7` | 性能治理 + 抗抖跟踪已完成 |
| B（名义 GPT） | `maixcam2_rect_center/` | `2026-10-12-motion-predict-v0.5.2` | v0.4.x 修轴映射/按钮自检/显示线程/掐断图传 → v0.4.6 入口模块化（main 923→296 行）→ v0.5.0 里程碑 → v0.5.1 恢复单帧穷尽搜索 → **v0.5.2 运动预测 + 搜索单帧完成（治"速度快捕获不到/重捕获慢"）** |

**B 轨 v0.5.2（10-12，当前）** —— **运动预测 + 搜索单帧完成**（用户实报 v0.5.1 之后仍"重捕获很慢、速度快捕获不到"）：
- **根因（PC 量化实证）**：① **无运动预测**——`track` 只按 `TRACK_RADIUS=12`px 窄带精修，单帧位移 >12px 即掉出，
  每帧退化成搜全图（实测 track 命中：18px/帧 `0/10`、24px `0/7`、36px `0/5`）；板端搜索远慢于 PC ⇒ 帧率塌陷、
  窗口跟不上 ⇒ "速度快捕获不到"。② v0.5.1 的 `SEARCH_BUDGET_MS=25ms` **软预算在板端把一次搜索拆到十几帧** ⇒ "重捕获很慢"。
- **改法**：① 新增 `_motion_delta()`：用**上两帧四角逐角位移**（含平移/缩放/旋转）外推本帧四角，在预测位置窄带精修
  （`ENABLE_MOTION_PREDICT=True`、`PREDICT_MAX_PX=60`；失锁 `_lost>0` 禁用预测；`track` 与 `roi_search` 窗口都叠加预测位移）；
  ② `config.SEARCH_SPLIT_FRAMES=False`（默认）⇒ `_search` **单帧跑完整搜索，重捕获恒 1 帧**（旧行为）。
- **验证**：track 命中 36px/帧 **0/5→4/5**、18px **0/10→9/10**、12px `8/15→15/15`；重捕获恒 1 帧；
  `test_performance` motion **0.637→0.418px**、稳态 `tracked=60` 不变、29/30·0.803px·0误检；`images_e2e` 16/16；
  `runtime` 8/8；`target_api`/`laser`/`gimbal_aim`(11)/`main_self_test`(4) 全绿；门禁 OK=96/FAIL=0。
- ⚠ **别靠加大 `TRACK_RADIUS` 治快移**：实测半径 80 时连 6px/帧 都开始掉（大步长会误锁印刷线/另一侧黑边）。

**B 轨 v0.5.1（10-12）** —— 恢复「单帧穷尽搜索」：v0.4.3(`26eea20`) 把 `_search` 改"合作式"（每帧只轮换 1 阈值、验 1 候选，
旧完整搜索被改名 `_legacy_search` 成死代码）⇒ 重捕获 1→11 帧。改回一帧内穷尽全部阈值；**重捕获 11→1**，
`images_e2e` 13/16→16/16。搜索**不参与稳态**（锁定走 `track`）⇒ 稳态帧率不变。

**B 轨模块化（v0.4.6，10-12）** —— 入口 `main.py` 按功能模块化，零行为变更：
- 新增 `vision/{rate_meter,camera_source,vision_worker,preview,aim_control,telemetry}.py` 六模块；
  `main.py` 只留 `BUILD_ID`+装配+主循环+模块地图 docstring；实现下沉到从未被覆写的 `vision/`（覆写损失面 -70%）。
- ★ **时钟注入**（唯一行为性调整）：组件构造可传 `clock`（默认标准库 `time`），`main()` 传自身 `time`
  ⇒ `test_main_self_test` 假时钟能穿透子模块；不传则行为不变。
- **旧私有名以别名保留** ⇒ 两个入口测试零改动通过；日志逐字不变。
- 构建 `MODULES` 9→15（`vision.aim_control` 须排 `gimbal_self_test` 之后）；包内 **12→18 文件**。
- `AGENTS §6.9` 已改写：**默认 `SHOW_ASYNC=True`（主进程内显示线程）**，v18「默认必须 False」被推翻；
  ⚠ `Display()` 仍**不可**放子进程（v16 坑，与显示线程无关）。

**B 轨关键历史结论（v0.4.3~v0.4.5）**：
- 板上恒 ~2fps 根因 = **MaixCDK 图传未就绪时 `send_image()` 忙等 500ms 空转 CPU**（show 三值挤 505~509ms）。
- 治本：`Config.DISABLE_MAIXVISION_STREAM=True`（默认）⇒ `main()` 建 Display 前置 `MAIXVISION=0` ⇒ 不建图传 ⇒ show 1~5ms 全帧；
  代价 = MaixVision **画面预览**无图（终端日志照常）；要看画面则设 `False`。
- ⚠ 证伪：降 `SHOW_FPS` 无效；「关掉预览丢帧」也无效（丢帧是 show 慢的**结果**）；`touchscreen.read()` 非阻塞。

## 1 · 项目一句话
2025 电赛 E 题视觉模块：靶心 = A4 靶纸四周 1.8cm 黑胶带矩形**对角线交点**（射影不变量），
输出 `(u,v)`（A 轨另给 `px_per_cm`）。详见 `AGENTS.md §0`。光斑口径（10-07 定死）：`LASER_MODE="fixed"`
全程不检测，每帧仅"黑框中心−常量"两次减法；实测光斑 ≈ (294,202)。旧瞄准代码备份在
`电赛/25Efork_rollback_backup_2026-10-07_2100/`。

## 2 · 改动纪律（AGENTS §3/§3′ 之外）
- 改 `vision/*`/`config.py`/`main.py`/`target_api.py` 后：**改 BUILD_ID → `build_single.py` 重生成 → 跑门禁**，顺序不能反。
  B 轨新增/改名模块必须同步 `build_single.py` 的 `MODULES` 清单（单文件版按序 exec，被依赖者须在前）。
- `tools/*.jpg`、`screenshot_*.png` 是调试产物：别当源文件改，别进包。

## 3 · 双轨并发 / git
严禁 `git add -A`（B 轨目录随时被别的进程写盘），**按路径精确 add**，提交前缀 `deepseek:`/`gpt:`/`repo:`/`docs:`。
- ⚠ **B 轨 main.py 被外部进程多次整份覆盖**（实测 v6/v9）：改完入口**立刻** `build_single.py` + `check_app.py --manifest`；
  **C8（dist 与 main.py 的 BUILD_ID 一致）是覆盖探针**。
- ★ **B 轨每轮交 git**：改码→BUILD_ID→build_single→门禁→**立即提交**。
  一条命令 `"D:/Python311/python.exe" -B tools/git_round.py -m "gpt: 摘要"`（`--check` 只核对；`--rollback <文件>` 看历史）。
  ⚠ `git_round --check` 第 3 步（`apply_gimbal_track_patch --check`）**长期红**：该工具是 v20 血统、锚点对当前
  v0.4.x 代码结构性误报 → **勿去"修"它、勿改它的 `LATEST_BUILD`**；此红存在时 `git_round -m` 会拒绝提交，
  改**手工精确 add+commit**（只 add 本轨目录 / 根文档，绝不 `-A`）。
- **B 轨另有 `maixcam2_rect_center/AGENTS.md`**（B 轨契约：边界/轴映射定案表/调参/改完三件事/每轮交 git/常见坑）；
  根 `AGENTS.md` 是跨轨契约，**改 B 轨先读 B 轨 AGENTS.md**。B 轨 `.gitignore`：忽略 `tools/e2e_report/`，
  保留 `tools/negative_e*.png`（测试固件）。
- 根 `.gitignore` 原则：**源码/标定/文档/单文件版进库**；`dist/main_single.py` 不忽略（有 `build_single --check` 守着）。
  仓库根 = A 轨（其 `main.py`/`config.py`/`dist`/`tools` 都在根）。

## 4 · 门禁（两轨通用）
`"D:/Python311/python.exe" -B tools/check_app.py --manifest`（C1..C10 + 包内清单）。B 轨包内 **18 文件**
（app.yaml/main/config/target_api/assets/vision×13），dist 已排除。改源码必更新 BUILD_ID，板端日志与本地逐字一致才算新代码。
**WorkBuddy 托管 venv 装不上 numpy/cv2 —— 一律用 `"D:/Python311/python.exe"`。**

## 5 · A 轨性能（README §三′）
- 症状入口 = `[fps]` 的 `掩膜=N`：稳态个位数（锁定=1）；长期 12/24 = 每帧全扫。
- 提速只动性能组；动 `MIN_AREA_RATIO`/`SCORE_*`/`MORPH_*` 必跑 `test_hardscene --ab`；性能结论必须同进程交错 A/B。
  `findContours` 代价∝轮廓数（2 万条≈190ms）→ `boundary_estimate()` 预筛（阈值 13000 实测交叉点）；
  `_masks` 必须生成器；锁定值排扫描表最前。

## 6 · A 轨抗抖（README §三″）
- 症状入口 = `保持=Y/N`；长期 Y 查**检测**不是滤波器。抖动位移是**必须跟踪的真实运动**；重低通"滤掉"= 照旧位置打（已证伪）。
- 改 `TRACK_*`/`KIND_BONUS` 必跑 `test_jitter.py --unit` 看**四项指标**；`TRACK_JUMP_*` 别往小调；机动判据扣 `meas_var`；q 映射按对数。

## 7 · B 轨要点（详见 `B轨代码技术解析与功能分析.md` 与 B 轨 README）
- 差异化 = 像素↔毫米（`rect_to_image` mm→px 单应已写好未用；不输出标量 px_per_cm）。
- **12 个死参数**（改了无效）：`SEARCH_BUDGET_MS` `SEARCH_MAX_PROPOSALS` `SEARCH_GEOMETRY_STEPS` `MAX_CONTOURS`
  `LINE_MAX_PROPOSALS` `LINE_SEARCH_INTERVAL` `ACQUIRE_CONFIRM_FRAMES` `CONFIRM_CORNER_PX` `RECHECK_INTERVAL`
  `MAX_LINE_RESIDUAL_PX` `MIN_TRANSITION_COVERAGE` `MAX_BOUNDARY_WIDTH_ERROR`。
- 基线：30 场景检出 29（fixture#2 强反光属安全失败）、max 0.803px、0 误检、稳态 ~1ms。
- 两轨互补：A 轨跟踪/性能成熟，B 轨几何/毫米完整 —— 补短板去另一轨抄已验证结论。
- ★ **"换 Canny 会不会更稳/更准"定论（10-12 只读核查，结论已固化，别再重复调研）**：
  A 轨文档里的"Canny 路线"(`reference/upstream_2025E` + `exp_alt_pipeline.py` 头注释的"另一条技术路线")
  **就是 B 轨现行路线**（Canny兜底/8×8单应/72点圆逐条对应）；A 轨自己从无 Canny 版本。
  复跑 `exp_alt_pipeline.py`：**检测算子对中心贡献同量级（阈值 0.8285px vs Canny 0.7496px），
  真正到 0.061px（13.6×）的是 `fitLine` 亚像素精修**。B 轨同图 A/B（只换候选生成器）：**29/30 对 29/30、
  0 误检、track 命中都 12/13** ⇒ **换纯 Canny 是等价搬迁**。
  两条防弯路：① **追踪不走 Canny/二值化**（走 `_refine` 窄带灰度梯度+fitLine，与捕获算子解耦）；
  ② A 轨"全分辨率多 3~4MB"的内存顾虑**在 B 轨不成立**（B 的 Canny 跑在 `SEARCH_WIDTH=320` 缩小图上，≈0.46MB）。
  详见根目录 `A轨黑框方案对比与Canny可行性.html`。
- ⚠ 参数有效性更新（v0.5.1/v0.5.2 后）：`SEARCH_BUDGET_MS` 仅在 `SEARCH_SPLIT_FRAMES=True` 时生效；
  `SEARCH_MAX_PROPOSALS` 现为单帧候选硬上限（128）；搜索相关新增 `ENABLE_MOTION_PREDICT`/`PREDICT_MAX_PX`/`SEARCH_SPLIT_FRAMES`。
- 对外 API `target_api.py`：`detect_black_frame`/`CenterTracker`/`AimLoop`，返回 `Result` 是 dict 子类。

## 8 · B 轨版本演进速查（细节在各提交信息 / B 轨 README）
- **v3 基线（`main`，`b32cd08`）**：⚠ 轴映射/方向是**旧值**、`ACC_DEFAULT=0`、无自检优先/LENS_CORR —— **上板前必须重核标定**。
  10-10 定案：**git 里不存在 v9 快照**（`track-curve-v9` 是外部覆写的未入库中间态）；安全分支 `backup/b-track-v20-pre-v9-rollback`（`2a08703`）。
- **v0.4.0**（`a06f81d`）：修轴映射 —— config `U1_*/U2_*`→`YAW_*/PITCH_*`；加 `PINMAP_STRICT=False`/`FLUSH_RX_ON_START=True`、
  `DIR_SIGN=-1/-1`、正方向 CCW、`ACC_DEFAULT 0→80`、`LENS_CORR=False`。
- **v0.4.1**（`6093827`）：取消上电自检自动跑 → **屏幕右上角点击 `SELF-TEST` 按钮**（`ENABLE_TOUCH_BUTTON`），
  固定脉冲 `SELF_TEST_PULSES=400`；画面流转到 MaixVision。
- **历史教训（别重踩）**：
  · v16 `Display()` 进 spawn 子进程 → 子进程起不来时 `can_submit()` 恒 False → **主循环一帧都不画**（无报错）。
    排查：sync 仍无画面且卡 `SAMPLE_VO_StartChn` ⇒ 设备 VO/fb0 卡死，**重启设备**。
  · v20 屏幕 `FPS` 曾画主循环速率（被跳帧稀释→虚高）；日志口径本就分开（`[perf] FPS=`=主循环 vs `[preview] 实测显示 FPS=`）。
  · **v9 更跟手**：v9 追踪猛（GAIN 0.5/COARSE 0.9、RPM 120·400、MAX_STEP 360、死区 2.0、无低通/限速/等待）；
    v12+ 逐版放缓（GAIN 0.3、RPM 60·150、MAX_STEP 80），v15 加低通 τ=0.25+限速 120+`AIM_WAIT_MOTION`。
    **"跟手 ↔ 柔和"是跷跷板不是 bug**；**改回需用户确认**。
- **v11 定案**：上电顺序 = **人工标定 → 自检 → auto_cal 延迟创建 → 自动标定 → 追踪解锁**；自检只用链路+时间（无黑框照走），
  但 `aim.state=="FAULT"` 时**不发任何指令**（门控勿丢）。改主循环控制流**必跑** `tools/test_main_self_test.py`（端到端跑真实 main()）。
- **v9 定案**：闭环发帧带 **ZDT 曲线加减速 `ACC_TRACK=200`**；`main.py` 发送点必须用 `Config.ACC_TRACK`
  （`test_track_uses_curve_config` 静态锁死，同时是 main.py 被覆盖的探针）。

## 9 · 云台/ZDT 速查（详见各专题文档 + 10-09 日志）
- **实测轴映射（v8 定案，v0.4.0 已落码）**：`ttyS0`(U0,ID=1)=**pitch**、`ttyS2`(U2,ID=2)=**yaw**。
  代码侧：`YAW_DEV=ttyS2`+`ADDR_YAW=2`、`PITCH_DEV=ttyS0`+`ADDR_PITCH=1`；两轴正方向都是 CCW（yaw CCW=左、pitch CCW=上）。
- 引脚：U0(A28/A29) **必须显式 pinmap**（空表静默不通）、是系统日志口（开机 `FLUSH_RX_ON_START` 清 RX）；
  U2(B0/B1) 需 pinmap；UART1 无外接引脚；备用 U3(B2/B3)/U4(A21/A22)。`PINMAP_STRICT=False`+引脚回读日志是"某轴不通"第一手证据。
- ZDT 协议：Checksum=0x6B 固定；多字节大端；F3 使能/F6 速度/FD 位置(mode0 相对 mode1 绝对)/FE 停止；
  dir 0x00=CW 0x01=CCW；16 细分 3200 脉冲=1 圈；返回 02 成功/E2 未使能堵转。
- **acc 档位语义（v5 实测）**：位置/速度模式 `acc=0` = 不用曲线、**直接以设定速度启停**（=卡顿根源）；
  ≠0 时每 `(256-acc)*50us` 变 1RPM，**acc 越小越柔和**（常用 20-80）。`estimate_move_seconds()`（stepper_link）按官方公式建模。
- 驱动器菜单必设（否则串口无效）：`P_Serial=UART_FUN`（出厂 RxTx_OFF！）、`UartBaud=115200`、`ID_Addr`、
  `Checksum=0x6B`、`Response=Receive`、`MotType=1.8°`、`MStep=16`、`P_Pul=PUL_OFF`；未 `Cal` 的板子只有 6 项菜单。
- `_ingest()` 无故障返回 `fault=False`：判断必须走 `stepper_link.fault_reason()`（None=无故障）。
- 符号：`dx=u靶心−u光斑` → 误差为正发 CCW；实测反了只改 `DIR_SIGN_*`。
- 五条回头路：① 闭环=定点跟踪；② `PULSE_PER_PX` 必须实测；③ 主用相对位置+增量比例律+单帧限幅；
  ④ 死区 ≥1.5×回差（3D 打印齿轮回差 1°≈9.7px）；⑤ 闭环用 raw_u/raw_v 并前馈底盘角速度（`CENTER_ALPHA` 平滑有 ≈28ms 滞后）。
- 未决：FAULT 需手动重启恢复（有意保守）；`ENABLE_SCAN` 默认关；光斑极性（fixed 模式已绕过）。

## 10 · 下一步（板端）
① 先解显示链路：跑新版看开机 `[preview] show 探针 ?ms` —— ≈505ms 则改**从设备本地启动**（不经 MaixVision）；
  探针回 1~5ms 后 `[perf]` 的 `read/loop/capture` 应一并恢复。② 验证屏幕右上角 `SELF-TEST` 按钮命中；③ 标定成功后打印"闭环已解锁"；
  追踪停下若晃 → `ACC_TRACK` 调小或 `RPM_FINE` 再降；④ P4 依实测回差调 `DEADBAND_PX`；P7 移动瞄准需 3507 经 SPI 前馈（接口预留）。
