# 项目长期记忆 —— 25Efork（MaixCAM2 靶心识别）

> 只记"跨会话必须知道、且不在 AGENTS.md 里"的结论；流水见同目录 `YYYY-MM-DD.md`。
> 协作契约 / 目录归属 / 门禁 / git 见仓库根 **`AGENTS.md`（每轮自动注入）**。
> **本文件必须保持精炼（超长会在注入时被截断）**，细节放各轨 README 与专题文档。

## 0 · 当前状态

| 轨 | 目录 | BUILD_ID | 状态 |
|---|---|---|---|
| A（DeepSeek） | 根 | `2026-10-05-r7` | 性能治理 + 抗抖跟踪已完成 |
| B（名义 GPT） | `maixcam2_rect_center/` | `2026-10-11-show-probe-v0.4.4` | v0.4.0 修轴映射 → v0.4.1 按钮自检+画面流转 → v0.4.3 显示移出主循环 → v0.4.4 锁定「板上 ~2fps」= MaixCDK 图传 500ms 忙等 + 开机探针 |

**B 轨 v0.4.4（10-11，当前）** —— 板上恒 ~2.5fps 的**根因已源码级锁定**：
- **症状**：`capture=2.5 disp=1.9 detect_fps=0 | read=106.9 loop=321.2ms | show avg=505.8 p95=508.4 max=509.1ms`。
- **根因**：show 的 avg/p95/max 挤在 505~509ms = **固定超时**，不是算力。MaixCDK `vision/src/maix_image_trans.cpp`
  的 `send_image()`：`if(!handle->init){ while(!init){ if(ticks-t>500) return NOT_READY; } }` ——
  **MaixVision 模式下图传 websocket 未连上时，每次 show() 忙等 500ms 且空转 CPU**；
  而 `Display::show()` 首件事就是 `if(img_trans) img_trans->send_image(img)`，`img_trans` 仅在
  `maixvision_mode()`（用 MaixVision 运行代码）时才创建。⇒ 1.9 次/秒 × 500ms ≈ **烧掉 ~95% 一个核** →
  相机 read 被抢到 107~175ms、识别线程饿死。**本地屏本身官方只要 1~5ms**。
- **改法**：`_Preview` 构造时做 `show()` 计时探针（`_probe_show()`，≥300ms 打根因 + 对策）；
  周期性 `show slow` 改 `warn_slow_once()`（只打一次、点名 MaixVision 图传）。
- **给用户的三条真解**：① **从设备本地启动应用**（不经 MaixVision 运行）⇒ `maixvision_mode()` 假 ⇒
  show ≈1~5ms ⇒ 跑满帧率（日志 `/maixapp/tmp/last_run.log`）；② 让 MaixVision 画面预览真正连上（异步入队）；③ `SHOW=False`。
- ⚠ **两条被证伪的直觉（别再走）**：降 `SHOW_FPS` 无效；「关掉预览丢帧」也无效 —— 丢帧是 show 慢的**结果**。
- **已排除**：`touchscreen.read()` 非阻塞（FIONBIO + 直读 input event）；`Camera.skip_frames(n)` 是一次性预热（非每帧跳帧）。

**B 轨 v0.4.3（10-11）** —— 修「画面只有 1.5 帧」：
- **根因**：`Display.show()` 仍在主循环同步调用，`cam.read→submit→绘制→show` 串行 ⇒ show 慢多少整条链就慢多少，
  `capture`/`disp`/`detect_fps` **三路一起**掉到同一低位而 `read=` 仍很小。
  ⚠ **降 `SHOW_FPS` 无效**（show 阻塞后 `now>=_t_show` 恒真，`due()` 固定时间轴被架空）—— 别再靠调它治。
- **改法**：① `SHOW_ASYNC=True`（默认）显示移入 daemon 线程 `_Preview._show_loop`，画布池 2 张、最新优先、池满丢帧不排队；
  `False` 一键退回旧行为。② `SHOW_WIDTH 480→0` = 用屏幕原始宽度（**MaixCAM2 屏 640×480 与相机同比**，
  画布==屏幕 ⇒ show 零缩放 + 触摸 1:1；旧值是"缩 480×360 再让驱动放回"的来回折腾）。
  ③ 去冗：按钮几何/文本预计算、仅黑边刷黑、`snapshot`/`rate.values` 每轮只取一次。
  ④ 恢复分相计时：`read/draw/loop`=主循环、`show avg/p95/max n`=显示线程、`drop(show)`/`drop(vision)` 分开。
- **回归**：新增 `test_slow_show_does_not_block_main_loop`（show 故意 300ms → render 最坏 0.27ms）；
  `test_main_self_test` 补 `[perf]` 日志分支覆盖（该 print 板上每 2s 跑，格式错会打死主循环）。
- ⚠ **上板实测否掉了当时的预期**：`loop` 未回到 34ms 而是 321ms —— 原因见上面的 v0.4.4（图传忙等抢 CPU）。
  若 MaixVision 看不到画面（=历史 v16 的坑）→ `SHOW_ASYNC=False` 退回主进程内同步 show。

**B 轨 main.py 模块化重构（方案已出 · 未执行）** —— 用户要求把识别/追踪等功能像 `target_api.py` 一样各自成独立文件，
便于按模块审查、降低入口复杂度。方案页 `25Efork/B轨main模块化重构方案.html`。
- 定案：新增 `vision/{rate_meter,camera_source,vision_worker,preview,aim_control,telemetry}.py` 六模块，
  入口 **843 行 → 约 230 行**；`main.py` **保留旧私有名别名** ⇒ 两个入口测试**零改动**。
- **开工前置**：① 确认并发写入者停手（≥5min `sha1sum main.py` 不变）；② 把已 staged 的 v0.4.3 **真正 commit**
  （10-11 23:03 实测 `HEAD` 仍 `72a011d`、文件 `M `=已 add 未 commit ⇒ **回滚点尚未落地**，别看日志写"已提交"就信）；
  ③ 拍板 `SHOW_ASYNC` 上板默认并同步 `AGENTS §6.9`（该条 v18 结论已被 v0.4.3 推翻）。
- 硬约束：`build_single.py` 的 `MODULES` 新增 6 项且 **`vision.aim_control` 须排在 `vision.gimbal_self_test` 之后**；
  包内清单 **12 → 18 文件**；`[ui] 上电自检已改为屏幕按钮` 等日志串逐字冻结。

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
- ⚠ **B 轨 main.py 被外部进程多次整份覆盖**（实测 v6/v9；vision/config 不受影响，只有入口丢）：
  改完入口**立刻** `build_single.py` + `check_app.py --manifest`；**C8（dist 与 main.py 的 BUILD_ID 一致）是覆盖探针**。
- ★ **B 轨每轮交 git**：改码→BUILD_ID→build_single→门禁→**立即提交**。
  一条命令 `"D:/Python311/python.exe" -B tools/git_round.py -m "gpt: 摘要"`（`--check` 只核对；`--rollback <文件>` 看历史）。
  ⚠ `git_round --check` 第 3 步（`apply_gimbal_track_patch --check`）**长期红**：该工具是 v20 血统、锚点对当前
  v0.4.x 代码结构性误报 → **勿去"修"它、勿改它的 `LATEST_BUILD`**；此红存在时 `git_round -m` 会拒绝提交，
  改**手工精确 add+commit**（只 add 本轨目录 / 根文档，绝不 `-A`）。
- **B 轨另有 `maixcam2_rect_center/AGENTS.md`**（B 轨工作契约：边界 / 轴映射定案表 / 调参 / 改完三件事 / 每轮交 git / 常见坑）；
  根 `AGENTS.md` 是跨轨契约，**改 B 轨先读 B 轨 AGENTS.md**。B 轨子树 `.gitignore`：忽略 `tools/e2e_report/`，
  保留 `tools/negative_e*.png`（测试固件）。
- 根 `.gitignore` 原则：**源码/标定/文档/单文件版进库**；`dist/main_single.py` 不忽略（有 `build_single --check` 守着）。
  仓库根 = A 轨（其 `main.py`/`config.py`/`dist`/`tools` 都在根）。

## 4 · 门禁（两轨通用）
`"D:/Python311/python.exe" -B tools/check_app.py --manifest`（C1..C10 + 包内清单）。B 轨包内 **12 文件**
（app.yaml/main/config/target_api/assets/vision×7），dist 已排除。改源码必更新 BUILD_ID，板端日志与本地逐字一致才算新代码。
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
- 对外 API `target_api.py`：`detect_black_frame`/`CenterTracker`/`AimLoop`，返回 `Result` 是 dict 子类。

## 8 · B 轨版本演进速查（细节在各提交信息 / B 轨 README）
- **v3 基线（`main` 所在，`b32cd08`）**：⚠ 其轴映射/方向是**旧值**（`ttyS1`=yaw 无引脚、`ttyS0` 未用、`DIR_SIGN=+1/+1`，未经 v8 实测），
  且 `ACC_DEFAULT=0`、无上电自检优先/LENS_CORR/帧率计量修复 —— **上板前必须重核标定**。
  10-10 定案：**git 里不存在 v9 快照**（`track-curve-v9` 是外部进程覆写的未入库中间态），已回退到 `b32cd08`；
  安全分支 `backup/b-track-v20-pre-v9-rollback`（=`2a08703`）可一键还原。
- **v0.4.0**（`a06f81d`）：修 v3 电机轴映射错误 —— config 改名 `U1_*/U2_*`→`YAW_*/PITCH_*`，落定案见 §9；
  加 `PINMAP_STRICT=False`/`FLUSH_RX_ON_START=True`（含引脚回读）、`DIR_SIGN=-1/-1`、正方向 CCW、`ACC_DEFAULT 0→80`、`LENS_CORR=False`。
- **v0.4.1**（`6093827`）：取消"上电自检自动跑"，改**屏幕右上角可点击 `SELF-TEST` 按钮**（`ENABLE_TOUCH_BUTTON`，
  `main._Preview/_Touch`，`touchscreen.read()` 按下沿触发），自检用固定脉冲 `SELF_TEST_PULSES=400`（免标定）；并把画面流转到 MaixVision。
- **历史教训（别重踩）**：
  · v16 把 `Display()` 放进 **spawn 子进程** → 子进程起不来时 `can_submit()` 恒 False → **主循环一帧都不画**（空白无报错）。
    排查：sync 仍无画面且卡在 `SAMPLE_VO_StartChn` ⇒ 设备 VO/fb0 卡死，**重启设备**，与本工程无关。
  · v20 屏幕 `FPS` 曾画**主循环速率**（被大量跳过帧稀释 → 虚高上百，眼睛只见 ≈1fps）；**日志口径本就分开**
    （`[perf] FPS=`=主循环 vs `[preview] 实测显示 FPS=`=shown/窗口），错的是屏幕文字没分口径。
  · **v9 更跟手**：v9 追踪又猛又快（GAIN 0.5/COARSE 0.9、RPM 120·400、MAX_STEP 360、死区 2.0、无低通/无限速/不等待运动）；
    v12 起因抱怨"过激"逐版放缓（GAIN 0.3、RPM 60·150、MAX_STEP 80），v15 加输出低通 τ=0.25+限速 120+`AIM_WAIT_MOTION`（走走停停）。
    **"跟手 ↔ 柔和"是跷跷板，不是 bug**（回退对照见 B 轨 README「v9→v17 追踪手感对照」）；**改回需用户确认**。
- **v11 定案**：上电顺序 = **人工标定 → 自检 → auto_cal 延迟创建 → 自动标定 → 追踪解锁**；自检只用链路+时间（无黑框照走），
  但 `aim.state=="FAULT"` 时**不发任何指令**（门控勿丢）。改主循环控制流**必跑** `tools/test_main_self_test.py`（端到端跑真实 main()）。
- **v9 定案**：闭环发帧带 **ZDT 曲线加减速 `ACC_TRACK=200`**（acc=0 是瞬时启停，停下瞬间晃开），短行程自动退化三角曲线；
  `main.py` 发送点必须用 `Config.ACC_TRACK`（`test_track_uses_curve_config` 静态锁死，同时是 main.py 被覆盖的探针）。

## 9 · 云台/ZDT 速查（详见各专题文档 + 10-09 日志）
- **实测轴映射（v8 定案，v0.4.0 已落码）**：`ttyS0`(U0,ID=1)=**pitch**、`ttyS2`(U2,ID=2)=**yaw**（与最初设想相反）。
  代码侧：`YAW_DEV=ttyS2`+`ADDR_YAW=2`、`PITCH_DEV=ttyS0`+`ADDR_PITCH=1`；两轴正方向都是 CCW（yaw CCW=左、pitch CCW=上）。
  等效方案：对调串口线 + ID 设回 1/2。
- 引脚：U0(A28/A29) **必须显式 pinmap**（走空表静默不通）、是系统日志口（开机 `FLUSH_RX_ON_START` 清 RX）；
  U2(B0/B1) 需 pinmap；UART1 无外接引脚（勿再踩）；备用 U3(B2/B3)/U4(A21/A22)。
  `PINMAP_STRICT=False` + 引脚回读日志是"某轴不通"第一手证据。
- ZDT 协议：Checksum=0x6B 固定字节；多字节大端；F3 使能/F6 速度/FD 位置(FD mode0 相对 mode1 绝对)/FE 停止；
  dir 0x00=CW 0x01=CCW；16 细分 3200 脉冲=1 圈；返回 02 成功/E2 未使能堵转。
- **acc 档位语义（v5 实测）**：位置/速度模式 `acc=0` = 不用曲线、**直接以设定速度启停**（=卡顿根源）；
  ≠0 时每 `(256-acc)*50us` 变 1RPM，**acc 越小越柔和**（常用 20-80）。`estimate_move_seconds()`（stepper_link）按官方公式建模。
- 驱动器菜单必设（否则串口完全无效）：`P_Serial=UART_FUN`（出厂 RxTx_OFF！）、`UartBaud=115200`、`ID_Addr`、
  `Checksum=0x6B`、`Response=Receive`、`MotType=1.8°`、`MStep=16`、`P_Pul=PUL_OFF`；未做 `Cal` 的板子只有 6 项菜单，
  先 Cal（空载）解锁；`En` 不接默认一直使能，"有劲"≠"串口通"。
- `_ingest()` 无故障返回 `fault=False`：判断必须走 `stepper_link.fault_reason()`（None=无故障），写 `is not None` 会让开机第 1 帧误判 FAULT。
- 符号：`dx=u靶心−u光斑` → 误差为正发 CCW；实测反了只改 `DIR_SIGN_*`。
- 五条回头路：① 闭环=定点跟踪（光斑几乎不动，不需小车位置/视差标定）；② `PULSE_PER_PX` 必须实测（走 2000 脉冲看 Δpx）；
  ③ 主用相对位置+增量比例律+单帧限幅；④ 死区 ≥1.5×回差（3D 打印齿轮回差 1°≈9.7px）；⑤ 闭环必须用 raw_u/raw_v 并前馈底盘角速度
  （`CENTER_ALPHA` 平滑有 ≈28ms 滞后）。
- 未决：FAULT 需手动重启恢复（有意保守）；`ENABLE_SCAN` 默认关（防上电甩头）；光斑极性（405nm 打紫外感光纸可能是暗斑，fixed 模式已绕过）。

## 10 · 下一步（板端）
① **先解决显示链路**：跑 v0.4.4，看开机那行 `[preview] show 探针 ?ms` —— 若 ≈505ms，改**从设备本地启动应用**
   （不经 MaixVision 运行）；探针回到 1~5ms 后，`[perf]` 的 `read/loop/capture` 应一并恢复正常。
② 验证屏幕右上角 `SELF-TEST` 按钮点击命中；③ 标定成功后打印"闭环已解锁"；追踪停下瞬间若晃 → `ACC_TRACK` 调小或 `RPM_FINE` 再降；
④ P4 依实测回差调 `DEADBAND_PX`；P7 移动中瞄准需 3507 经 SPI 给角速度前馈（接口预留）。
