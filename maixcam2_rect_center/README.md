# MaixCAM2 黑框中心：四边精修、快速跟踪与双轴云台瞄准

构建号 `2026-10-08-target-api-v3`，应用版本 `1.4.0`。在 MaixVision 中打开本文件夹，选择“运行项目”。
必须一起下发应用包内的 **12 个文件**（`app.yaml` / `main.py` / `config.py` / `target_api.py` /
`assets/icon.png` / `vision/` 下 7 个 `.py`），不能只运行当前 `main.py`。

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

tools/                        离线验证 / 调参 / 打包（**不进应用包**）
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

**⑥ 其余** —— `vision/__init__.py` 统一导出 9 个符号；`app.yaml` 版本 1.3.0 → **1.4.0**；
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
[rect-center] 2026-10-08-target-api-v3 search_width=... display_width=... display_fps=...
[aim] 闭环口径=raw（raw_u/raw_v）离群拒收=400.0px 死区=2.0px 限幅=360p
[perf] 2026-10-08-target-api-v3 valid=True FPS=... stage=track frames=... searches=0 read=... lens=... bridge=... detect=... laser=... uart=... rx=... aim=... show=... loop=... ms | laser_call=...ms n=... hit=...
[aim] state=FINE dx=+3.2 dy=-1.1 yaw=CCW/12p pit=CW/4p lost=0
```

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
| `DIR_SIGN_YAW/PITCH` | +1 | **必须实测**（`CALIBRATE_AXIS`），反了会正反馈甩头 |
| `PULSE_PER_PX_YAW/PITCH` | 3.67 | **占位值，必须实测替换** |
| `GAIN_YAW/PITCH`、`GAIN_COARSE` | 0.5 / 0.9 | 精/粗两段增益，必须 ≤1 才收敛 |
| `DEADBAND_PX` | 2.0 | 应 ≥ 1.5 × 实测回差，否则在齿隙里极限环 |
| `MAX_STEP_PULSES` | 360 | 单帧脉冲限幅（≈10° 云台角），防误检猛甩 |
| `JUMP_REJECT_PX` | 400 | 单帧离群拒收门限（连续 3 帧才采信） |
| `CALIBRATE_AXIS` | "" | 置 `"yaw"`/`"pitch"` 开机做一次开环标定；**标完置回 `""`** |
| `ENABLE_SCAN` | False | 失锁自动慢扫重捕获；确认机械行程后再开 |

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
python -B tools/test_performance.py          # 黑框检测：30 合成场景 + 5 反例 + 运动/失锁
python -B tools/test_performance.py --legacy 改动前的rectangle_center.py路径
python -B tools/test_laser.py                # 激光：默认 fixed 口径 + detect 精度/限频/反光拒绝/耗时
python -B tools/test_gimbal_aim.py           # 云台：协议帧 / 闭环收敛 / 符号反发散 / ACK·E2 / 标定器
python -B tools/test_target_api.py           # 对外 API：识别/追踪/偏差符号/参数覆盖
python -B tools/example_api_usage.py         # 三种小问场景的可运行调用示例
```

测试包含 30 个已知真值的合成场景（透视、42°/45°/90°/135°旋转、暗背景、低曝光、局部反光、噪声），5 个反例，连续移动、突然平移、目标消失及重新出现。真值由独立 OpenCV 投影生成；测试检查未经时间平滑的原始中心，不能用旧坐标混过测试。

本版针对失锁卡顿的关键点是：默认不进入 Hough 恢复；失锁时只在缩小图上做有限阈值/轮廓搜索，搜索耗时在 PC 上约 5～17 ms，不再出现秒级单帧。已加入黑带全宽采样，避免笔记本屏幕等细黑边被锁定。PC 的绝对耗时受系统负载影响，板上以 `[perf]` 日志为准。
