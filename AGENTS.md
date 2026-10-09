# AGENTS.md · 25Efork 多智能体协作与工程规范

> 面向在本仓库工作的 **AI 编码代理**（DeepSeek / GPT / 其它）。人类可读的工程说明在
> [`README.md`](README.md)（算法、上板、内存、版本核对等），本文件是**协作契约**：
> 谁负责哪些目录、改完必须做什么、怎么提交、怎么打包。
> 两者冲突时，**本文件对"协作与流程"生效，README 对"算法与上板"生效**，改一处请顺手同步另一处。

---

## 0. 项目一句话

2025 全国大学生电子设计竞赛本科组 **E 题《简易自行瞄准装置》** 的视觉模块：
由 **A4 靶纸四周 1.8cm 黑胶带矩形的两条对角线交点** 反算 **靶心像素坐标 `(u, v)`** 与
**局部尺度 `px_per_cm`**，通过串口交给云台主控解算激光指向。工程按 MaixPy 应用规范组织
（`main.py` + `app.yaml` + `config.py` + `vision/`）。

> 靶心是红色记号笔点（≤0.1cm），165cm 处只有约 0.67px，物理不可见 → **只能由黑框矩形几何反算**。

---

## 1. ★ 双轨并行：谁负责什么（最重要的约定）

同一道题、同一个视觉子任务，本仓库里放了**两份并行实现**，分别由两个 AI 代理负责，**目前都在调试中**：

| 轨道 | 目录 | 负责方 | 应用入口 | `app.yaml` 的 id |
|---|---|---|---|---|
| **A（主轨 / 根）** | `25Efork/`（除下方子目录外的全部） | **DeepSeek** | `main.py` | `target_center_25e` |
| **B（并行第二实现）** | `25Efork/maixcam2_rect_center/` | **GPT** | `maixcam2_rect_center/main.py` | `maixcam2_rect_center` |

两条轨道的**功能需求完全相同**，差别只在技术路线：

- **轨道 A（DeepSeek）**：灰度二值化 + **环 / 亮面双族**候选轮廓 + 结构打分选优 + 角点亚像素精修；
  靶心 = 极值法取四角、**对角线交点**（射影不变量，不解单应）；缩小图粗定位失败则回退全分辨率；
  输出 `(u,v)` + `px_per_cm`，带 One Euro 滤波。
- **轨道 B（GPT）**：**Canny + `approxPolyDP` 凸四边形**配对内外框 + 四边灰度剖面校验；
  **显式 8×8 单应**解算；在实际正视面以中心为圆心取 **72 点圆**逆映射回原图求中心；Canny 中断时用
  Otsu/多档暗阈值掩膜兜底。

### 硬规则（违反会真出事故）

1. **只改自己轨道的目录**，不要"顺手"改对方文件。确需跨轨改动（公共文档、根 `config.py` 等），
   先说明意图，改完在提交信息里点明。
2. **两条轨道不能同时装到板子上、也不能同时跑**（`app_id` 与入口都不同）；同一时刻只调试一条，
   避免日志/串口互相干扰、误判"改了没用"。
3. **一个实现绝不能打进另一个的应用包**：根 `app.yaml` 的 `exclude` 必须含 `maixcam2_rect_center`
   （已配），并注意 B 侧打包时排除上层工程。
4. **双方共用一个工作树、并发写盘** → **严禁 `git add -A` / `git add .`**，
   一律**按路径精确 add**（见 §5）。历史上就发生过"把别人写到一半的文件一起提交"。

---

## 2. 目录地图（含归属）

```
25Efork/
├── AGENTS.md                  ★ 本文件（协作契约，跨轨公共）
├── README.md                   人类可读工程说明（跨轨公共）
├── app.yaml                    轨道 A 应用描述（id=target_center_25e）
├── main.py                     轨道 A 入口（顶部有 BUILD_ID）
├── config.py                   轨道 A 现场唯一需要改的文件
├── vision/{geometry,tracker,detector,overlay}.py   轨道 A 算法（tracker=抗抖跟踪）
├── assets/icon.png             轨道 A 图标
├── dist/main_single.py         轨道 A 单文件版（自动生成，"运行当前文件"用）
├── tools/                      轨道 A 离线验证 / 调参 / 打包（含 maix_stub/ 桩模块）
├── reference/                  upstream 参考资料（他人开源代码，非本工程）
├── 视觉项目代码分析报告.md       轨道 A 深度分析报告
│
└── maixcam2_rect_center/       ★ 轨道 B（GPT）—— 独立应用，自带 app.yaml
    ├── AGENTS.md             ★ 轨道 B 工作契约（工作边界 / 实测电机轴映射 / 调参 / **每轮交 git**）
    ├── app.yaml                轨道 B 应用描述（id=maixcam2_rect_center，v1.4.0）
    ├── main.py                 轨道 B 入口（顶部 BUILD_ID）
    ├── config.py               轨道 B 现场调参（CANNY_* / MIN_EDGE_CONTRAST / TARGET_*_MM / 云台组）
    ├── target_api.py           ★ 轨道 B 对外 API（跨小问复用）：detect_black_frame / CenterTracker / AimLoop
    ├── vision/rectangle_center.py   轨道 B 算法（Canny + 单应 + 72 点圆）
    ├── vision/laser_spot.py         光斑读数（fixed/detect）+ deviation(dx,dy)
    ├── vision/{stepper_link,gimbal_aim,gimbal_calibrate,gimbal_self_test}.py   双轴云台链路/控制/标定/自检
    ├── tools/test_*.py / example_api_usage.py / check_app.py   轨道 B 测试与门禁
    └── assets/icon.png         轨道 B 图标
```

`maixcam2_rect_center/` **不是本工程的模块**，是**另一个自带 `app.yaml` 的独立应用**——
不要当成普通文件 import，也不要让它进 A 的应用包。

---

## 3. 轨道 A（DeepSeek）改完**必须**做的三件事

1. **更新 `main.py` 顶部 `BUILD_ID`**（当前 `2026-10-05-r7`，格式 `YYYY-MM-DD-rN`）——
   用来确认板子跑的是新代码，**不更新就等于无法判断是否生效**。
2. **重新生成单文件版**：`python tools/build_single.py`（生成 `dist/main_single.py`）。
   `smoke_main.py` 的"预检 0"会核对源码与单文件版 `BUILD_ID` 一致。
3. **跑门禁**（§4）。改了 `main.py` / `config.py` / `vision/*` 任一都要做。

> `config.py` 是**唯一**需要现场改的文件。改算法参数前先想清楚是否真该动 `SCORE_*` 类阈值
> （它们承担抗误检职责）；优先调阈值/曝光/形态学，别一上来就放宽打分。

---

## 3′. 轨道 B（GPT）改完**必须**做的三件事

与轨道 A 同构。B 轨构建号格式 `YYYY-MM-DD-<slug>-vN`（当前 `2026-10-08-target-api-v3`），
版本核对依据是 `main.py` 的 `BUILD_ID` 启动日志串。**以下命令一律在 `maixcam2_rect_center/`
目录下执行**（脚本按自身位置定位工程根），本机 Python 用 `"D:/Python311/python.exe"`。

1. **更新 `main.py` 顶部 `BUILD_ID`** —— 板端会打印 `[rect-center] <BUILD_ID> ...`，
   **与本地逐字一致才算跑的是新代码**；不更新则无法判断是否生效。
2. **重新生成单文件版**：
   ```bash
   "D:/Python311/python.exe" -B tools/build_single.py           # 生成 dist/main_single.py
   "D:/Python311/python.exe" -B tools/build_single.py --check   # 单文件是否已过期
   "D:/Python311/python.exe" -B tools/build_single.py --smoke   # PC 自检：嵌入模块是否全部就位
   ```
   `smoke_main.py` 的"预检 0"（A 轨）在这里由 `--smoke` 承担：它核对单文件与源码的
   `BUILD_ID` 是否一致、8 个嵌入模块能否就位。**新增/改名 `vision/*.py` 或根级模块后，
   必须同步 `tools/build_single.py` 里的 `MODULES` 清单**，否则「运行当前文件」会漏模块。
3. **跑门禁**（改了 `main.py` / `config.py` / `vision/*` / `target_api.py` 任一都要做）：
   ```bash
   "D:/Python311/python.exe" -B tools/check_app.py --manifest   # C1..C10 机检 + 打印包内清单
   "D:/Python311/python.exe" -B tools/test_performance.py       # 视觉回归（29/30、0.803px、0 误检）
   "D:/Python311/python.exe" -B tools/test_target_api.py        # 对外 API 回归
   "D:/Python311/python.exe" -B tools/test_gimbal_aim.py        # 云台协议/闭环/看门狗/标定
   "D:/Python311/python.exe" -B tools/test_main_self_test.py    # 端到端跑 main()：上电自检优先/故障阻断
   ```
   改了光斑相关（`laser_spot.py` / `config.LASER_*`）还要跑 `tools/test_laser.py`；
   改了视觉算法阈值（`MIN_*` / `MAX_*` / `SCORE_*` 类）必须对着 `test_performance` 确认无回归。
   改了主循环控制流（上电自检 / 标定 / 追踪的顺序）必须跑 `tools/test_main_self_test.py`。

> B 轨同样只有 `config.py` 需要现场改。`dist/` 在 `app.yaml` 的 `exclude` 里，**不进应用包**；
> 应用包 = **12 文件**（`app.yaml` / `main.py` / `config.py` / `target_api.py` / `assets/icon.png` /
> `vision/` 下 7 个 `.py`）—— 新增文件后**必须核 `--manifest` 打印的包内清单**。


---

## 4. 门禁（提交前 / 上板前同一套，**顺序不能反**）

```bash
python tools/build_single.py --check     # 单文件版是否与源码同步（改源码后没重生成会报错）
python tools/smoke_main.py               # 主循环 + 形状矩阵预检（必要时加 --single 测单文件版）
python tools/test_pc.py                  # 精度回归
python tools/test_hardscene.py --ab      # 硬场景鲁棒性 + 误检反向验证（新旧策略对照）
python tools/profile_hotspots.py         # 性能回归（改了 config 的性能组 / detector 帧循环时加跑）
python tools/test_jitter.py              # 抖动稳定性回归（改了 tracker / 滤波 / 角点提取时加跑）
```

- **性能组参数别乱动**：`MAX_MASKS_FULL` / `FRAG_BOUNDARY_EST` / `MAX_SCAN_CONTOURS` /
  `MAX_SCAN_COMPONENTS` / `ROI_TRACK*` 决定"最坏一帧有多慢"，`MIN_AREA_RATIO` /
  `SCORE_*` / `MORPH_*` 决定"多准、多不容易误检"。**后者动了一定要跑
  `test_hardscene.py --ab`**（18 个硬场景必须全检出、5 个"画面里没靶纸"必须全拒检）。
- **抗抖组参数（`TRACK_*` / `KIND_BONUS`）改完必须跑 `test_jitter.py`**（含 `--unit`），
  看四项指标：**跟手（误差不能比原始测量大）、抑噪、静止平滑、拒离群**。
  这几项互相拉扯，只看一项很容易把另一项调坏。
- 帧率问题先看板端 `[fps]` 行里的 **`掩膜=N`**：稳态应为个位数（锁定后 = 1）；
  长期 12/24 说明每帧都在全扫 —— 机理与排查见 [`README.md` §三′](README.md)。
- "车一晃中心就偏"先看 **`保持=Y/N`**：长期 `Y` 说明该查**检测**（阈值/曝光/干扰物），
  不是滤波器 —— 机理与设计见 [`README.md` §三″](README.md)。
  ⚠ 抖动引起的图像位移是**必须跟踪的真实运动**，不要试图用更重的低通去"滤掉"它
  （这正是 0.1fps 之后暴露出来、且已实测证伪的旧做法）。

- **本机跑法**：`"D:/Python311/python.exe" tools/xxx.py`
  （WorkBuddy 托管 venv 装不上 numpy/cv2，别在那里浪费时间；系统 Python 已带 numpy 2.3.5 + cv2 5.0.0）。
- 板端日志出现 `[app] 靶心识别 <BUILD_ID>`，**与本地 `main.py` 的 `BUILD_ID` 一致才算新代码**。

---

## 5. git 协作规范

- **仓库根 = `25Efork/`**（不是上层 `电赛/`，那里有 800MB 无关内容）。单 `main` 分支，两轨同库不同目录。
- **提交信息带轨道前缀**，一眼看清是谁的改动：
  - `deepseek: <摘要>` —— 根目录 / 跨轨公共文件（AGENTS.md、README.md、app.yaml、tools/ …）
  - `gpt: <摘要>` —— `maixcam2_rect_center/` 下的一切
  - `repo:` / `docs:` / `tools:` —— 纯公共或纯工具改动
- **按路径精确提交**（并发写盘，**绝不 `-A`**）：
  ```bash
  # DeepSeek 侧
  git add main.py config.py vision/ tools/ app.yaml AGENTS.md README.md .workbuddy/memory/
  git commit -m "deepseek: ..."
  # GPT 侧
  git add maixcam2_rect_center/
  git commit -m "gpt: ..."
  ```
  提交前用 `git status --short` 确认暂存区只含**你自己**的路径。
- **两条已配好的约定，别改**：
  - `.gitattributes` 强制 `eol=lf` —— 源码要拷到 MaixCAM2（Ubuntu）上跑，仓库里绝不能有 CRLF；
  - 仓库本地 `core.quotepath=false`（中文文件名正常显示）、`core.autocrlf=false`。
- **进库 / 不进库**：见 [`README.md` §9](README.md)。要点——
  ✅ 应用本体 / `tools/` / `dist/main_single.py` / `.workbuddy/memory/*.md` / `reference/`；
  ❌ `__pycache__/` `*.pyc` / `tools/*.jpg` / **调试截图（`screenshot_*.png` 等）** / `*.zip` /
  `.workbuddy/` 除 `memory/` 外的一切。
- **提交身份**：`CloudyRain <cloudyrain@example.com>`（**占位邮箱**，推远程前先改）。
- **尚无远程仓库**；要推时 `git remote add origin <url> && git push -u origin main`。
- ★ **轨道 B 额外要求「每轮交 git」**：`maixcam2_rect_center/main.py` 被外部进程
  （MaixVision 烧录 / 并行会话）**整份覆写**过多次，`git` 是唯一可靠的回滚点。
  B 轨每完成一轮（改码 → 提升 `BUILD_ID` → `build_single` → 门禁 → **立即提交**）即提交一次；
  细则与一条命令的助手脚本（`tools/git_round.py`）见 [`maixcam2_rect_center/AGENTS.md §5`](maixcam2_rect_center/AGENTS.md)。

---

## 6. 打包口径

- 根侧（轨道 A）：`python tools/pack_zip.py --app`（应用包）/ `--src`（完整源码包）/ `-o <目录>`。
  `--app` 打完后脚本会**自检**：只有一个根 `app.yaml`、无隐藏文件、有 `main.py`。
- **打完必须核一遍包内清单**：应用包 = **8 个文件**
  （`app.yaml` / `main.py` / `config.py` / `vision/`×4 / `assets/icon.png`）。
  本项目就是靠核清单才发现 `reference/`、`maixcam2_rect_center/`、文档曾混进应用包。
- 轨道 B 自带 `app.yaml`（`exclude` 黑名单里已含 `tools` / `dist` / `__pycache__` / `*.md` /
  调试截图），打包口径同理；**应用包 = 12 文件**（`app.yaml` / `main.py` / `config.py` /
  `target_api.py` / `assets/icon.png` / `vision/`×7）。核对方式：在 `maixcam2_rect_center/` 下跑
  `"D:/Python311/python.exe" -B tools/check_app.py --manifest`，它直接打印包内清单。

---

## 7. 常见坑（都真实踩过，别再踩）

1. **灰度形状**：MaixCAM2 的 `image2cv()` 对灰度图返回 **`(H,W,1)` 三维哑轴**，不是 `(H,W)`；
   绝不能按 `ndim == 2` 判灰度，否则单通道喂给 `COLOR_BGR2GRAY` → `cv2.error ... 'scn' is 1`。
   按**实际 shape** 判断。
2. **「运行当前文件」只下发一个文件** → `ModuleNotFoundError: No module named 'config'`。
   改用「运行项目」，或先 `build_single.py` 再跑 `dist/main_single.py`。
3. **`main.py` 被 MaixVision 里的旧内容覆盖** → 看起来"改了没用"，其实板子在跑旧代码。
   上板前跑 `smoke_main.py`（预检 0 会当场报出来）；改完在 MaixVision 里关掉再重开该文件。
4. **`app.yaml` 的 `exclude` 漏目录** → 嵌套应用 / 文档 / 参考资料混进应用包。
5. **远距离锁不住**（轨道 A 的历史根因）：`MIN_AREA_RATIO` / `MASK_FG_*` 按"靶纸占满画面"定死，
   超过约 2m 产生不出候选；形态学核把细环腐蚀掉；失败时锁阈值把兜底路径一起锁死。
   现在这些阈值已按远距离重定，改动务必跑 `test_hardscene.py --ab` 反向验证。

---

## 8. 完成定义（DoD）

- **改完源码**：`BUILD_ID` 已更新 → `dist/main_single.py` 已重生成 → 门禁全绿 → **按路径**提交。
- **上板**：日志 `BUILD_ID` 与本地一致，`[fps]` 正常，锁定效果符合预期。
- **留痕**：往 `.workbuddy/memory/YYYY-MM-DD.md` **追加**一行本轮做了什么、怎么验证的
  （只记有长期价值的结论与坑，不记流水）。跨会话必须知道的约定 → `.workbuddy/memory/MEMORY.md`。
