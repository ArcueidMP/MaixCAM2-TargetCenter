# 项目长期记忆 —— 25Efork（MaixCAM2 靶心识别）

> 只记"跨会话必须知道"的约定与结论。日常流水见同目录 `YYYY-MM-DD.md`。

## 一、项目定位

2025 电赛 E 题《简易自行瞄准装置》的视觉模块。靶心 = A4 靶纸四周 1.8cm 黑胶带
矩形**两条对角线的交点**（射影不变量，不需要解单应）。输出 `(u, v)` + `px_per_cm`。
工程按 MaixPy 应用规范：`main.py` + `app.yaml` + `config.py` + `vision/`。

## 二、必须遵守的工程约定

1. **`config.py` 是唯一需要现场改的文件。** 改算法参数前先想清楚是否真的该动
   `SCORE_*`（它承担抗误检职责）；优先调阈值/曝光。
2. **改了 `main.py` / `config.py` / `vision/*` 之后必须**
   - 更新 `main.py` 顶部的 `BUILD_ID`（形如 `2026-10-05-r6`）—— 用来确认板子跑的是新代码；
   - 跑 `python tools/build_single.py` 重新生成 `dist/main_single.py`
     （`tools/smoke_main.py` 里的"预检 0"会核对两入口 BUILD_ID 一致）。
3. **交付前门禁顺序**（少一步都可能下发到旧代码）：
   `build_single.py --check` → `smoke_main.py`（必要时 `--single`）→ `test_pc.py`
   → `test_hardscene.py --ab` → 再下发。
4. **`app.yaml` 的 `exclude` 必须列出所有"不该进应用包"的东西**，包括
   `reference/`（upstream 参考资料）、**嵌套的独立应用目录**（如
   `maixcam2_rect_center/`，它自带 app.yaml，不排除就会被当普通文件打进包）、
   `tools/` `dist/` `.workbuddy/`以及各类文档。打完包**必须核一遍包内文件清单**。
5. **`tools/*.jpg` 是脚本产物**，随时可重新生成，不要当成源文件改。

## 三、git 约定（2026-10-05 建立）

- **仓库根 = `25Efork/` 本身**，不是上一层 `电赛/`（那里有 800MB 无关内容）。
- **`.gitattributes` 强制 `eol=lf`**：源码要拷到 MaixCAM2（Linux），仓库里不能有 CRLF；
  同时避免 `core.autocrlf=true` 那种"一检出就变脏"。
- 仓库本地配置 `core.quotepath=false`（中文文件名正常显示）、`core.autocrlf=false`。
- **跟踪**：应用本体、`tools/`（含 `maix_stub/` 桩模块）、`dist/main_single.py`
  （「运行当前文件」直接用，且 `--check` 守着它同步）、`.workbuddy/memory/`（设计记录）、
  `reference/`（upstream 对照资料，非本工程代码）。
- **不跟踪**：`__pycache__/`、`*.pyc`、`tools/*.jpg`、`*.zip`、
  `.workbuddy/` 除 `memory/` 以外的一切。要强加某张图用 `git add -f`。
- 提交身份用全局 `CloudyRain <cloudyrain@example.com>`（邮箱是占位的，要推远程前先改）。
- 尚未配置远程仓库。

## 四、★ 双轨并行：根 = DeepSeek，`maixcam2_rect_center/` = GPT

**用户已明确定死归属**：`25Efork/` 根目录及其下（除子目录外）的一切 = **DeepSeek 负责**；
`25Efork/maixcam2_rect_center/` = **GPT 负责**。两轨**功能需求完全相同**（同一道题同一视觉子任务），
**目前都在调试**；只是技术路线不同（A：二值化+环/亮面双族+对角交点；B：Canny+四边形逼近+单应+72 点圆）。
→ 处置方式已定：**同库不同目录、双轨并行、都跟踪**（此前"待用户确认处置"已作废）。

**协作契约见仓库根 `AGENTS.md`**（2026-10-05 新增，面向 AI 代理；README 面向人）。
必须遵守的硬规则：
1. **只改自己轨道的目录**，别顺手改对方文件；
2. **两轨不能同时装/同时跑**（app_id 与入口都不同），同一时刻只调试一条；
3. **一个实现绝不能进另一个的应用包**（根 `app.yaml` exclude 已含 `maixcam2_rect_center`）；
4. **双方共用一个工作树、并发写盘 → 严禁 `git add -A`**，一律按路径精确 add；
   提交信息带轨道前缀：`deepseek: ...` / `gpt: ...` / `repo:`（跨轨公共）。

该目录（B 轨）**会被其它进程随时并发写入**，改动期间做全量 add 会把半成品一起提交进去。

## 五、★ 性能预算（2026-10-05 建立，改帧相关代码前必读）

**现场症状入口**：板端 `[fps]` 行末尾的 **`掩膜=N`**。稳态应为个位数（锁定后 = 1）；
**长期 12/24 就等于"每帧都在全分辨率全扫"，那是掉帧的头号原因**。
机理与排查表见 `README.md §三′`。

**两条铁律**
1. **提速不许动 `MIN_AREA_RATIO` / `SCORE_*` / `MORPH_*`。** 它们是精度与抗误检的旋钮，
   动了必须重跑 `test_hardscene.py --ab`（18 硬场景全检出 + 5 个"无靶纸"场景全拒检）。
   提速只动性能组：`MAX_MASKS_FULL` / `FRAG_BOUNDARY_EST` / `MAX_SCAN_CONTOURS` /
   `MAX_SCAN_COMPONENTS` / `ROI_TRACK*`，改完跑 `tools/profile_hotspots.py`。
2. **性能结论必须用同进程交错 A/B 得出。** 用 `git show HEAD:<file>` 把旧实现另存成临时包，
   同进程交替测；跨进程/跨时间对比会被机器负载污染（实测同一个 morphology 两次差 7 倍）。

**关键实现事实（改动时别踩回去）**
- `findContours` 的代价主要由**要创建的轮廓对象数**决定（2 万条 ≈ 190ms PC/x86），
  单靠事后截断救不了 → 必须先"不生成"。判据用 `boundary_estimate()`（0.3ms，
  与 findContours 耗时严格单调），超过 `FRAG_BOUNDARY_EST=13000` 才走连通域预筛。
  **13000 是两条路径的实测交叉点** —— 无条件走连通域会给每张掩膜固定加 17~20ms。
- `MORPH_SEARCH=True` 时 **lvl=1 精细档不做开运算**，噪点原样保留，是最容易爆的一档，
  且每轮扫描都会跑到 → 它是性能与远距离鲁棒性的共同关键点，改形态学务必两头都测。
- `_masks` 必须是**生成器**：旧实现一次性把 24 个掩膜全算完，`break` 只省描边、
  省不掉阈值化 + 形态学。
- "全分辨率升级"那一趟 `force_sweep=True` 会无视锁定 → 只要缩小图长期失败，
  它就是"每帧 24 掩膜"的来源；因此**锁定值必须排到扫描表最前面**。
- ROI 跟踪（`_try_track`）有**跳变门限**与裁剪检测，跟丢即回落全局搜索 ——
  它只会更快、不会更准，所以关掉它是安全的降级选项。

**新增工具**：`tools/profile_hotspots.py`（端到端 / 最坏全扫 / 病态掩膜回归）。
当前 `BUILD_ID = 2026-10-05-r7`。
