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

## 四、注意：`maixcam2_rect_center/` 是**并行的第二份实现**

同一个任务的另一种写法（`vision/rectangle_center.py`，Canny+多边形逼近+单应路线），
自带 `app.yaml` / `main.py` / `config.py`。它是**独立应用**，不要和本工程混跑，
也不要让它进本工程的应用包（已在 `app.yaml` exclude 与 `pack_zip.py` 兜底里排除）。
该目录曾被其它进程并发写入 —— 在这个目录被改动期间做 `git add -A` 会把
别人半成品一起提交进去，必要时按路径精确 add。
