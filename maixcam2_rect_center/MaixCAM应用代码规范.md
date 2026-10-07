# MaixCAM / MaixPy 应用代码规范（v1.0）

> 适用范围：`25Efork/` 下所有要**下发到 MaixCAM2 运行 / 打包成应用**的代码。
> 每条规则都有编号（`R-x`），与 `tools/check_app.py` 一一对应，可机检。
> 标记含义：**必须**（违反 = 上板会出事故，checker 报 FAIL）、**禁止**（同上）、
> **建议**（违反只报 WARN）、**须知**（无法机检，靠人/AI 自觉）。
>
> 本文档只约束"怎么写成能跑、能打包的 MaixCAM 应用"，
> **算法怎么设计、参数怎么调**请看同目录 `B轨代码技术解析与功能分析.md` 与仓库根 `README.md`。

---

## 0. 一句话原则

> **能在 PC 上跑通的代码 ≠ 能上板的代码。**
> 上板会额外引入三类风险：① 设备 API 的形状/语义与 PC 不同；② 设备内存/算力比 PC 小 1~2 个数量级；
> ③ **下发方式**（"运行项目" / "运行当前文件" / 装成应用）决定哪些文件真的到了板子上。
> 本规范就是把这三点固化成可检查的条目。

---

## 1. 目录与文件（R-1）

**必须**按下面的骨架组织。`<app_id>` 用自己应用的 id，`<name>` 用模块名。

```
<app_root>/                      ← 这个目录整体就是"应用"，也是 app.yaml 所在层
├── app.yaml                     ★ 必须存在，必须在应用包根目录（不能再套一层文件夹）
├── main.py                      ★ 必须存在，入口名固定
├── config.py                    现场标定参数（唯一需要现场改的文件）
├── vision/<name>.py             算法包（建议，非强制）
├── assets/icon.png              由 app.yaml 的 icon 指向
├── dist/main_single.py          [可选] 单文件版，只用于"运行当前文件"调试，必须 exclude
├── tools/*.py                   仅 PC 用的测试/校验脚本，必须 exclude
└── README.md / *.md             给人看的文档，必须 exclude
```

- **R-1.1 必须**：`main.py` 与 `app.yaml` 位于**同一个**目录，且该目录就是下发/打包的根。
  - ❌ 反例：`app_root/src/main.py` —— launcher 找不到入口。
- **R-1.2 禁止**：在应用根下放**嵌套的独立应用目录**（自带 `app.yaml` 的子目录）。
  一个应用包里**有且只有一个** `app.yaml`（在根）。嵌套应用必须写进 `exclude`
  （本项目真实踩过：`reference/`、`maixcam2_rect_center/`、文档曾混进应用包，靠核清单才发现）。
- **R-1.3 必须**：应用包内**只能有**这些内容：`app.yaml` + `.py` 源码 + `assets/`。
  调试截图（`*.png` 落盘图）、`*.zip`、`.pyc`、日志、文档、测试脚本**一律不得进包**。
- **R-1.4 建议**：算法代码放 `vision/` 子包（`vision/__init__.py` + 模块），
  `main.py` 只负责"设备 IO + 主循环 + 串口"，几何/算法**不依赖 maix**，以便在 PC 上直接单测。
- **R-1.5 必须有且仅有**一个模块级别名入口：`if __name__ == "__main__": main()`。

### 文件名与命名

- Python 文件用**小写 + 下划线**（`rectangle_center.py`），**禁止**中文文件名（板上 shell / zip 都可能出问题）。
- 类用 `CamelCase`，函数/变量 `snake_case`，常量 `UPPER_SNAKE`。
- 文档（`.md`）可以用中文名，但**必须进 `exclude`**。

---

## 2. 编码与换行（R-2）

- **R-2.1 必须**：全部源码 **UTF-8 无 BOM**。
- **R-2.2 必须**：全部源码 **LF 换行**（不得有 CRLF）。
  - 理由：源码要塞进 `'''...'''` 式的单文件打包、要拷到 Ubuntu 板子上跑，
    CRLF 会破坏多行字符串/正则，并且 `core.autocrlf` 会导致"一检出就变脏"。
  - 本仓库由根 `.gitattributes` 的 `* text=auto eol=lf` 兜底，但**写入时就要写 LF**。
- **R-2.3 建议**：文件头不加 BOM、不加 shebang（MaixPy 由 launcher 调起，不是 `./main.py` 执行）。
- **R-2.4 必须**：源码中**不得出现绝对 Windows 路径**（`C:\...`、`D:/...`）、
  也不得写死 `/tmp/maixpy_run` 作为唯一根路径（用 §5 的根探测）。

---

## 3. `app.yaml`（R-3）

```yaml
id: my_app_id                 # 必须与目录名一致；小写字母+数字+下划线
name: My App                   # 英文名
name[zh]: 我的应用              # 中文名
version: 1.1.0                 # x.y.z
icon: assets/icon.png          # 相对应用根
author: 25Efork
desc: One line English description.
desc[zh]: 一行中文说明。
exclude:                       # ★ 见 R-3.4，默认"全包含"，这里写"不打包的东西"
  - __pycache__
  - tools
  - dist
  - README.md
```

- **R-3.1 必须**：字段齐全 = `id` / `name` / `version` / `icon` / `author` / `desc`；
  建议再加 `name[zh]` / `desc[zh]`（MaixVision 界面显示中文）。
- **R-3.2 必须**：`id` **等于所在目录名**。多轨并行时 id 必须唯一
  （本项目：根 = `target_center_25e`，`maixcam2_rect_center/` = `maixcam2_rect_center`），
  否则两个应用在设备上会互相覆盖，且无法判断到底跑的是谁。
- **R-3.3 必须**：`version` 用 `x.y.z`；**改了行为就要升版本**。
- **R-3.4 必须**：`exclude` 里至少包含：`__pycache__`、`tools`、`dist`、
  **所有 `.md` 文档**、**所有调试落盘图**（`*.png` 落盘截图 / `screenshot_*.png`）、`build`、
  `.workbuddy`、`.vscode`。默认是"包含全部"，**漏一项就直接进包**。
- **R-3.5 禁止**：`exclude` 里写**当前不存在的路径**（如 `screenshot_optimized.png` 这种历史残留）。
  它不是错误，但会掩盖"真正的截图没被排除"这个事实 —— checker 会报 WARN 并要求清理。
- **R-3.6 建议**：不要写 `**/*.py` 这类反向白名单规则；只做黑名单排除，规则越简单越好核。
- **R-3.7 须知**：`id` / 文件名改动会改变设备上的安装目录与升级关系，
  现场调试期间**不要改 `id`**。

---

## 4. `main.py`（R-4）

**必须**遵守下面这个骨架（本项目两轨都等价于此结构）：

```python
"""一句话说明这个应用做什么。"""

import os
import sys
import time

import cv2
import numpy as np

BUILD_ID = "2026-10-06-r1"          # ★ R-4.3


def _root():
    """板上落地位置各不相同，逐个试探取第一个含 config.py 的目录。"""
    cands = []
    try:
        cands.append(os.path.dirname(os.path.abspath(__file__)))
    except NameError:
        pass
    cands += [os.getcwd(), "/tmp/maixpy_run", "/root/apps"]
    for c in cands:
        if c and os.path.exists(os.path.join(c, "config.py")):
            return c
    return cands[0] if cands else os.getcwd()


ROOT = _root()
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from config import Config                       # noqa: E402
from vision.my_module import Detector           # noqa: E402


def main():
    from maix import app, camera, display, image   # ★ R-4.5：设备 API 延迟到函数内导入
    ...
    print("[app] my_app %s" % BUILD_ID)            # ★ R-4.3
    while not app.need_exit():
        ...
    try:
        cam.close()
    except Exception:
        pass


if __name__ == "__main__":
    main()
```

- **R-4.1 必须**：只导入 `cv2` / `numpy` / 标准库 与**本应用内的**模块。
  第三方依赖要显式列在应用说明里（MaixCAM 系列是 Ubuntu，`opencv-python` / `numpy` 需已安装）。
- **R-4.2 禁止**：导入任何**桌面 GUI / 图像库的交互 API**
  （见 R-6 的清单）。
- **R-4.3 必须**：定义 `BUILD_ID`（`YYYY-MM-DD-rN` 或 `YYYY-MM-DD-<短描述>`）并**在启动时打印一次**。
  这是判断"板子到底跑没跑新代码"的唯一可靠手段。
  单文件版 / 打包版 / 源码版的 `BUILD_ID` **必须一致**。
- **R-4.4 必须**：`maix` 相关模块**只在 `main()` 内或 `try/except` 内导入**。
  理由：只有这样才能在 PC 上用桩模块（或直接 import `vision/`）做离线单测；
  顶层 `from maix import ...` 会让 PC 上一 import 就 ModuleNotFoundError。
- **R-4.5 必须**：`main()` 结束时把相机/显示/串口**关干净**（至少 `cam.close()` 包在 `try` 里），
  避免"退出后重新运行已占用设备"。
- **R-4.6 必须**：主循环提供**优雅退出**：`while not app.need_exit():`。
  禁止 `while True:`（板上只能强杀）。
- **R-4.7 必须**：主循环内**不得有无限阻塞调用**（`time.sleep(>0.1)`、`input()`、
  同步网络请求、大文件写 flash）。需要节流用"时间戳比较"而不是 sleep。
- **R-4.8 建议**：启动时打印一行机器可读的能力行，例如
  `[app] <BUILD_ID> cam=<W>x<H>@<fps> fmt=GRAY mode=<...>`，便于现场核对档位。

### 串口输出（对接云台主控时）

- **R-4.9 必须**：串口发送**按帧发送且只发一帧一包**，无效结果也要发（明确告诉下游"这帧没结果"），
  禁止"没检出就不发"（下游会拿着旧值继续打）。
- **R-4.10 建议**：包格式带帧头 + 长度/校验，例如本项目：
  `@[u,v]%\r\n` / `@[invalid]%\r\n`；并新增 `valid` 字节位以与其它数据帧区分。
- **R-4.11 必须**：串口对象创建失败**不得**导致进程退出（`try/except` + 打印原因，继续跑视觉）。

---

## 5. 设备 IO 桥接（R-5）—— 头号上板事故区

### 5.1 灰度图获取

- **R-5.1 必须**：相机优先用**灰度直出**：`camera.Camera(w, h, image.Format.FMT_GRAYSCALE, ...)`，
  帧缓冲只占 1B/px（RGB888 是 3B/px）。
- **R-5.2 必须**：`image2cv` 的返回值**按实际 `shape` 判断通道数**，
  **禁止**用 `ndim == 2` 判断"这是灰度图"。
  已知事实：**MaixCAM2 的 `image2cv(img, ensure_bgr=False, copy=False)` 对灰度图返回 `(H, W, 1)` 三维哑轴**，
  不是 `(H, W)`。靠 `ndim == 2` 判 → 判不出 → 落到 `ensure_bgr=True` 兜底 →
  对灰度图 `ensure_bgr` 是**空操作** → 单通道喂给 `cv2.COLOR_BGR2GRAY` →
  `cv2.error: ... 'scn' is 1` 崩在第一帧。
- **R-5.3 必须**：兜底分支**不得**直接 `cvtColor(..., COLOR_BGR2GRAY)`，
  必须再过一次通道数判断（同 R-5.2 的理由）。
- **R-5.4 必须**：算法入口做一次自我保护：`(H,W,1)` 先 `[:, :, 0]` 或 `np.squeeze`，
  并断言 `ndim == 2 and dtype == np.uint8`，否则抛**可读的**异常。
- **R-5.5 禁止**：把 `image2cv(copy=False)` 得到的数组**跨帧持有**（它是当帧缓冲的视图）。
  `copy=False` 只在"本帧内用完即弃"时使用。

标准写法（可直接复制）：

```python
def _to_gray(img, image_module):
    """把 maix image 转成 (H,W) uint8。按实际 shape 判断，绝不靠 ndim==2 认灰度。"""
    a = image_module.image2cv(img, ensure_bgr=False, copy=False)
    if a.ndim == 2:
        return a
    if a.ndim == 3 and a.shape[2] == 1:
        return a[:, :, 0]
    if a.ndim == 3 and a.shape[2] == 3:
        return cv2.cvtColor(a, cv2.COLOR_RGB2GRAY)
    if a.ndim == 3 and a.shape[2] == 4:
        return cv2.cvtColor(a, cv2.COLOR_RGBA2GRAY)
    b = image_module.image2cv(img, ensure_bgr=True, copy=True)   # 仅对彩色打包格式有效
    if b.ndim == 2:
        return b
    if b.ndim == 3 and b.shape[2] == 1:
        return b[:, :, 0]
    return cv2.cvtColor(b, cv2.COLOR_BGR2GRAY)
```

### 5.2 显示

- **R-5.6 建议**：显示**独立限频**（本项目 `SHOW_FPS=15`），检测与串口**每帧都跑**；
  显示不得拖慢检测节奏。
- **R-5.7 必须**：送显前先把图**缩小**（本项目缩到宽 480）再转 BGR，
  并 `image.cv2image(view, bgr=True, copy=True)`（`copy=True` 避免与异步显示抢同一块缓冲）。
- **R-5.8 必须**：`SHOW` 做成配置开关；调试完现场用 `SHOW=False` 跑纯检测，作为"显示开销"的归因实验。

### 5.3 内存与性能

- **R-5.9 必须**：相机缓冲 `buff_num=2`（默认 3），启动后 `cam.skip_frames(>=10)` 丢掉预热帧。
- **R-5.10 必须**：**每帧分配的对象数要有上界**。禁止在帧循环里 `np.zeros(大图)`、
  禁止每帧重新建标准点集/查表（用缓存）。
- **R-5.11 必须**：给单帧耗时一个**天花板**：任何"可能爆炸"的路径（轮廓数、阈值档数、
  连通域数、候选数）都要有上限参数，且参数必须在 `config.py` 里可见。
  踩过的坑：噪声掩膜下 `findContours` 会创建上万条轮廓对象，单帧从 1ms 涨到几十~几百 ms。
- **R-5.12 必须**：板端性能以**板端日志**为准，PC 耗时不等于板端耗时（差一个数量级都很正常）。
  每 N 秒打印一行 `[perf]`，至少含：`FPS` / 各阶段 ms / **本帧搜索次数** / 是否锁定。
- **R-5.13 建议**：`cv2.setNumThreads(1)`。小图上多线程调度比运算本身更贵，还会和相机线程抢核。

---

## 6. 禁止清单（R-6）

**禁止**在会进应用包的 `.py` 里出现（checker 用 AST 扫描）：

| 类别 | 具体 API | 为什么 |
|---|---|---|
| 桌面窗口 | `cv2.imshow` `cv2.namedWindow` `cv2.waitKey` `cv2.destroyAllWindows` `cv2.destroyWindow` `cv2.moveWindow` `cv2.resizeWindow` | 板上无 X11/窗口系统，直接崩 |
| 桌面绘图库 | `matplotlib` / `pyplot` / `PIL.Image.show` / `tkinter` / `PyQt5` / `PySide` | 同上，且增大体积 |
| 交互输入 | `input()` / `getpass` | 板上无 stdin |
| 进程控制 | 顶层 `sys.exit()` / `os._exit()` / `raise SystemExit` 于 import 期 | 会让 launcher 认为应用启动失败 |
| 设备侧调用 | 帧循环里的 `os.system` / `subprocess.*` | 引入不可控阻塞 |
| 落盘调试 | 帧循环里无条件 `cv2.imwrite` / `imencode`+写 flash | 写 flash 是**秒级**开销，必须"单独节流 + 限量 + 可按开关关闭" |

允许但**必须加保护**的：

- **R-6.1** `img.lens_corr(...)`：MaixPy 版本间可能不存在 → 必须 `try/except`，
  失败后**本次会话永久停用**（打一行日志），不要每帧刷错误日志。
- **R-6.2** 相机档位降级：`camera.Camera(...)` 失败时按"更省内存"的顺序重试，并打印最终档位。
- **R-6.3** `from maix import sys` 会**遮蔽标准库 `sys`** → 同时要用两者时用别名（`m_sys`）。

---

## 7. 打包与下发（R-7）

- **R-7.1 必须**：用「**运行项目**」下发整个文件夹（或先安装成应用）。
  「**运行当前文件**」**只发当前打开的那一个 .py**，其它文件不会过去 →
  典型症状就是 `ModuleNotFoundError: No module named 'config'`。
  **这是使用方式问题，不是代码错误**，不要为此改 import 写法。
- **R-7.2 建议**：同时维护 `dist/main_single.py` 单文件版（把 `config` / `vision/*` 源码原样嵌入、
  运行时造虚拟模块注册进 `sys.modules`、最后 `exec` 主源码），让「运行当前文件」也能跑。
  生成端要带 **sha1 指纹 + `--check`**，防止"单文件版停在旧代码"。
- **R-7.3 必须**：打包后**核一遍包内文件清单**（用 `tools/check_app.py --manifest`），
  与预期清单逐项比对，不要凭信任。
- **R-7.4 必须**：上板前**先关掉 MaixVision 里打开的旧文件再重新打开**。
  真实事故：MaixVision 里开着旧内容的 `main.py`，保存/运行时把 stale buffer 写回磁盘，
  于是"改了源码 → 板子上报一模一样的错、行号都不变"。
  判据：板端日志的 `BUILD_ID` 必须等于本地 `main.py` 的 `BUILD_ID`。
- **R-7.5 须知**：设备上的落地路径 —— 「运行项目」→ `/tmp/maixpy_run/`；
  装成应用 → `/root/apps/<app_id>/`。根路径探测要覆盖这两处（见 R-4 的 `_root()`）。

---

## 8. 交付门禁（R-8）

**必须**按顺序跑，任一失败就**不要下发**：

```bash
cd <app_root>
python -B tools/check_app.py --manifest     # ① 本规范机检 + 包内清单（新增，见 §9）
python -B tools/check_app.py                # ② 同上，只看结论/退出码
python -B tools/<你自己的回归测试>.py        # ③ 精度/性能回归（本项目 tools/test_performance.py）
python -B tools/build_single.py --check     # ④ 有单文件版时：核对是否与源码同步
```

- **R-8.1 必须**：只要改了 `main.py` / `config.py` / `vision/*` 任一，
  §8 全部条目都要重跑。
- **R-8.2 必须**：改了**精度类**参数（阈值、覆盖率、面积门、对比度门）后，
  必须同时跑"正样本检出"和"**负样本拒检**"两类测试 —— 只测前者会把手感调成"什么都敢锁"。
- **R-8.3 必须**：把"本轮做了什么、怎么验证的"追加到 `.workbuddy/memory/YYYY-MM-DD.md`（只记有长期价值的结论）。

---

## 9. `tools/check_app.py`（本规范的可执行版本）

放在应用目录的 `tools/` 下（`tools/` 必须 exclude，不影响应用包）。

| 检查 | 内容 |
|---|---|
| C1 | `app.yaml` 存在、字段齐全、`id` == 目录名、`version` 形如 `x.y.z` |
| C2 | `icon` 指向的文件存在、是 PNG、尺寸/体积在建议范围内 |
| C3 | `main.py` 存在、`config.py` 存在、`vision/` 有 `__init__.py` |
| C4 | 所有 `.py` **语法编译通过**（`compile()`，不执行） |
| C5 | AST 扫描：禁止清单（§6）、顶层 `maix` 导入、帧循环里无条件落盘 |
| C6 | 所有将进包的文件 = **UTF-8 且纯 LF**（无 CRLF、无 BOM） |
| C7 | 源码里无绝对 Windows 路径 |
| C8 | `BUILD_ID` 存在且被打印；有 `dist/main_single.py` 时核对两者一致 |
| C9 | `exclude` 覆盖了所有 `.md`、`tools`、`dist`、`__pycache__`、调试截图；报告**失效的 exclude 项** |
| C10 | 输出最终**包内清单**（路径 + 体积 + 合计），并检查是否有禁入类型（`.pyc`/`.png` 截图/`.zip`/`.md`） |

退出码：`0` = 通过（可有 WARN）；`1` = 有 FAIL。

---

## 10. 上板前的 5 分钟自检（人 / AI 都照这个顺序）

1. `python -B tools/check_app.py` → 必须 exit 0。
2. 打开板端日志，核对第一行 `BUILD_ID` 与本地 `main.py` **逐字一致**。
3. 看 `[perf]` 行：`FPS` 是否正常、`stage` 稳态是否为 `track`（锁定后不该每帧搜索）。
4. 把靶纸拿走后应输出 `invalid`（**不得**继续输出旧坐标）。
5. 关掉 MaixVision 里的旧文件，重新打开工程 →「运行项目」。

---

## 附：与仓库既有约定的关系

- 仓库根 `AGENTS.md` 是**协作契约**（谁改哪些目录、怎么提交）；本文档是**代码格式契约**。
  两者冲突时：`AGENTS.md` 管"协作与流程"，本文档管"代码长什么样"。
- 双轨并行时（根 = 轨道 A，`maixcam2_rect_center/` = 轨道 B）：
  **只改自己轨道的目录**；两轨的 `app.yaml` 的 `id` 不同、不能同时装、不能同时跑。
- 本规范一旦被证明好用，建议提升到仓库根（与 `AGENTS.md` 并列），成为两轨公共契约。
