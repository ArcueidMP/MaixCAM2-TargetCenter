# B 轨代码技术解析与功能分析

> **分析对象**：`25Efork/maixcam2_rect_center/`（轨道 B，由 GPT 实现）
> **分析版本**：`BUILD_ID = 2026-10-05-fast-edges-v2`（`main.py` 第 25 行）
> **分析日期**：2026-10-06
> **本文档用途**：让后续 Agent **不必重读全部源码**就能接手开发新功能。
> 所有"实测"数据均在 **PC（Windows / `D:/Python311`，numpy 2.3.5 + cv2 5.0.0，`OPENCV_THREADS=1`）**
> 上取得，不是板端数据。行号对应 `2026-10-05-fast-edges-v2`。

---

## 一、这份代码要解决什么问题

2025 电赛 E 题《简易自行瞄准装置》。靶心是 A4 靶纸中央的红色记号笔点（≤0.1cm），
在 165cm 处只占约 **0.67px，物理不可见** → 只能由靶纸四周 **1.8cm 黑胶带矩形**的几何反算。

本轨道（B）的技术主张与轨道 A 不同：

| | 做法 |
|---|---|
| **A（根目录）** | 二值化 + ring/paper 双族候选 + 结构打分 + 极值法取角 → **对角线交点**（不需要单应） |
| **B（本目录）** | Canny/阈值 INV + **轮廓树孔环关系**取候选 + **四边窄带梯度精修** → **显式 8×8 单应** → 物理正视面圆心逆映射 |

**关键取舍**：A 的"对角线交点"是**射影不变量**，求中心不需要单应；B 选择**建单应**，
代价是多一层数值计算，收益是 **`px_per_cm` 这类毫米度量变成正确**的
（标量 `px_per_cm` 在透视下有 36% 的方向性误差，见仓库根 `README.md` / 技能包六·五）。
所以 **B 轨真正的差异化能力不是"中心更准"，而是"像素↔毫米可度量"** —— 这一点是理解全部代码的钥匙。

---

## 二、文件职责地图

| 文件 | 行数 | 职责 | 谁需要改 |
|---|---|---|---|
| `app.yaml` | 15 | 应用描述（`id=maixcam2_rect_center`）+ `exclude` | 加/删文件时要改 |
| `main.py` | 143 | 设备 IO、主循环、分段计时、串口、显示限频 | 加新阶段/新输出时 |
| `config.py` | 69 | **现场唯一需要改的文件**（相机档位 + 全部算法阈值） | 现场调参 |
| `vision/rectangle_center.py` | 484 | 全部算法：几何工具 + 检测器 + 绘制 | 改算法 |
| `vision/__init__.py` | 3 | 只导出 `RectangleCenterDetector` | 基本不动 |
| `tools/test_performance.py` | 142 | PC 回归：30 合成场景 + 5 负样本 + 运动/失锁/重捕 + 稳态基准 | 加测试用例 |
| `assets/icon.png` | — | 应用图标 | 不动 |
| `screen_check.png` | 151KB | **调试截图，当前不在 `exclude` 里（缺陷）** | 见 §七.5 |

**依赖面很窄**：运行期只用 `cv2` / `numpy` / `maix` / 标准库（`os sys time`）；
`vision/rectangle_center.py` **不 import maix**，因此可以在 PC 上直接单测 —— 这是本工程最好的一条设计。

---

## 三、运行时数据流（`main.py`）

```
cam.read()                     ← FMT_GRAYSCALE 640x480 @30fps, buff_num=2, skip_frames(10)
   │  [计时: read]
   ├─ img.lens_corr(strength=1.5)      ← try/except，失败则本次会话永久停用（lens_enabled=False）
   │  [计时: lens]
   ├─ _to_gray(img, image)             ← image2cv 桥接，按实际 shape 判通道（见 §四.1）
   │  [计时: bridge]
   ├─ detector.process(gray)           ← 全部算法（见 §四.3 状态机）
   │  [计时: detect]
   ├─ _send_uart(uart, result)         ← "@[u,v]%\r\n" 或 "@[invalid]%\r\n"（ENABLE_UART 默认 False）
   │  [计时: uart]
   └─ if 距上次显示 >= 1/SHOW_FPS:      ← 显示独立限频，不跳检测帧
        draw_result(gray, result, SHOW_WIDTH=480) → image.cv2image(view, bgr=True, copy=True) → disp.show
      [计时: show / loop]
```

主循环每 `LOG_INTERVAL=2.0s` 输出一行 **分段平均耗时**：

```text
[perf] <BUILD_ID> valid=<bool> FPS=<窗口帧率> stage=<track|roi_search|full_search|no_black_frame>
       frames=<n> searches=<n> read=.. lens=.. bridge=.. detect=.. uart=.. show=.. loop=.. ms
```

> **这是本轨道的性能锚点**。`stage` 稳态应该是 `track`、`searches=0`；
> 若长期 `full_search`，说明检测一直在失锁（去查阈值/曝光/靶纸），**不要去调性能参数**。

---

## 四、算法逐模块解析

### 4.1 设备桥接：`_to_gray`（`main.py:28`）

按 **实际 `shape`** 分派，而不是按 `ndim == 2` 判灰度。

```python
a = image_module.image2cv(img, ensure_bgr=False, copy=False)
if a.ndim == 2:            return a                      # 某些固件灰度直出是 (H,W)
if a.ndim == 3 and a.shape[2] == 1: return a[:, :, 0]    # ★ MaixCAM2 是 (H,W,1)
if a.ndim == 3 and a.shape[2] == 3: return cv2.cvtColor(a, cv2.COLOR_RGB2GRAY)
if a.ndim == 3 and a.shape[2] == 4: return cv2.cvtColor(a, cv2.COLOR_RGBA2GRAY)
b = image_module.image2cv(img, ensure_bgr=True, copy=True)   # 兜底
if b.ndim == 2: return b
return cv2.cvtColor(b, cv2.COLOR_BGR2GRAY)               # ⚠ 兜底没再审通道数，见 §七.7
```

**要点**：`(H,W,1)` 这一支是**必须**的 —— MaixCAM2 的灰度 `image2cv` 返回三维哑轴，
靠 `ndim==2` 认灰度会掉进 `ensure_bgr=True` 兜底，而它对灰度图是**空操作** →
单通道喂 `COLOR_BGR2GRAY` → `cv2.error: 'scn' is 1`。
**注意兜底分支的最后一行少了通道数检查**（相对仓库规范 R-5.3 是个缺口）。

另外 `process()` 里还有**第二道**保护（`rectangle_center.py:401-405`）：
`(H,W,1)` → `[:, :, 0]`，然后断言 `ndim==2 and dtype==uint8`，否则抛 `ValueError`。
→ **几何模块可以脱离 maix 独立测试**，这是本工程的核心可测性设计。

### 4.2 几何工具层（`rectangle_center.py:12-99`）

| 函数 | 行 | 做什么 | 关键点 |
|---|---|---|---|
| `_polygon_area(p)` | 12 | 有向面积 | `contourArea(..., oriented=True)`，用符号判环绕方向 |
| `_order_quad(points)` | 16 | 四角排序 | **质心极角排序**（`arctan2`）→ 符号修正 → `roll` 使 index0 为 `x+y` 最小者（左上）。**不用 `x±y` 极值排序，因此没有 45° 奇点**（见技能包六·六） |
| `_side_lengths(q)` | 25 | 四边长 | `\|roll(q,-1)-q\|` |
| `_gaussian_solve(a,b)` | 29 | 高斯-约当消元 | 部分主元 + **消去行向量化**（`m -= factors[:,None]*m[col]`），退化解抛 `ValueError` |
| `_homography(src,dst)` | 45 | **显式 8×8 单应** | ★ **先做 Hartley 归一化**（减均值 + 除以最大绝对偏差），解完再用 `ts`/`td_inv` 反归一化。原始像素坐标下条件数 4.55e5 → 归一化后 3.08（技能包六·五） |
| `homography_8x8(...)` | 66 | 语义化包装 | **未被调用**（仅公共 API） |
| `project_points(m,pts)` | 71 | 批量投影 | 齐次除法；`\|z\|<1e-9` 抛 `ValueError` |
| `inverse_map_points(h,pts)` | 79 | 反投影 | `project_points(inv(h))`，**未被调用** |
| `standard_circle(w,h,r,n)` | 83 | 物理正视面标准圆 | 圆心 = `(w/2,h/2)`，半径 `CIRCLE_RADIUS_MM`，默认 72 点 |
| `_rectangle(w,h,inset)` | 88 | 物理矩形四角 | `inset` 用于表达"内沿"这个物理坐标 |
| `_sample(gray,pts)` | 93 | **一次 `cv2.remap` 批量双线性采样** | 把 N 个点排成 `(1,N)` 的 `float32` map 一次采完，`BORDER_REPLICATE`。这是本工程最有效的性能技巧：把"逐点 Python 循环读灰度"变成一次 C 调用 |

> **新功能提示**：`standard_circle` / `project_points` / `_homography` 已经足够支撑
> "在靶纸上做毫米级几何"这类需求（打环、圆轨迹、偏差量）。`inverse_map_points`
> 已经写好但没接线 —— 要做毫米度量时**直接用它**，不要重写。

### 4.3 检测器状态机：`process()`（`rectangle_center.py:399`）

```
                    ┌──────────────────────────────────────────────┐
每帧进来 ─────────► │ 1) _track 非空？                             │
                    │    _refine(g, 上次quad, kind, TRACK_RADIUS)  │  ← 只在四边 ±12px 窄带里找跳变
                    │    + _verify  →  stage = "track"             │     不跑全图 Canny / 不扫阈值
                    └───────────────┬──────────────────────────────┘
                                    │ 失败
                    ┌───────────────▼──────────────────────────────┐
                    │ 2) ROI 搜索（_track["outer_quad"] 外扩        │
                    │    ROI_PADDING=0.20、至少 24px）→ stage="roi_search"
                    └───────────────┬──────────────────────────────┘
                                    │ 失败
                    ┌───────────────▼──────────────────────────────┐
                    │ 3) 全图搜索 _search(g)  → stage="full_search" │
                    └───────────────┬──────────────────────────────┘
                                    │ 失败
                    ┌───────────────▼──────────────────────────────┐
                    │ 4) 清空 _track/_smooth/_locked_th            │
                    │    valid=False, stage="no_black_frame"       │
                    │    ★ 不输出旧坐标（安全失败）                 │
                    └──────────────────────────────────────────────┘
```

中心解算（成功后，`436-443`）：

```python
raw = project_points(matrix, [[w/2, h/2]])[0]      # ★ 直接映射物理圆心，而不是对圆周点求均值
if _smooth is None or CENTER_ALPHA <= 0 or |raw - _smooth| > FILTER_RESET_PX(25):
    center = raw                                    # 大跳变 → 重置，不做平滑
else:
    center = _smooth + CENTER_ALPHA(0.70) * (raw - _smooth)
```

> README 已明确：**不能对投影后的 72 个圆周点求均值**（投影不是线性运算，均值 ≠ 中心的投影）。
> `circle_points` 是**留给下游用的**（打环/显示），不参与中心计算。

### 4.4 几何合法性：`_valid_quad()`（`116`）

三道门（全部在**图像坐标**下）：

1. 四角必须在画面内（`margin=1`）；
2. `cv2.isContourConvex`；
3. 最短边 ≥ **`12`（硬编码）**；
4. 四个角的**正弦值** `|e_i × e_{i+1}| / (L_i·L_{i+1}) > 0.20`（≈11.5°），
   排除退化尖角与过扁四边形。

> **注意**：`12` 与 `0.20` 是硬编码，**不在 `config.py`**。远距离小目标（靶纸只有 40px 宽、边长约 20px）
> 会贴着这个下限 —— 想支持更远距离，应先把这两个值提成配置项。

### 4.5 物理朝向推断：`_layout()`（`132`）

```python
horizontal = lengths[0] + lengths[2] > lengths[1] + lengths[3]
w, h = (TARGET_LONG_MM, TARGET_SHORT_MM) if horizontal else (TARGET_SHORT_MM, TARGET_LONG_MM)
inset = BLACK_FRAME_MM if kind == "inner" else 0.0
return w, h, _rectangle(w, h, inset)
```

- 像素上更"长"的那一对边 ↔ 物理 `TARGET_LONG_MM=297`。**旋转不变**（绕光轴任意角都成立）。
- `kind`（来自轮廓树深度）决定 `q` 是**胶带外沿**（≈纸边，物理 0..w）还是**胶带内沿**（物理 tape..w-tape）。
- ⚠ 这把物理尺度**写死成 A4 竖/横两态**。若要支持其它靶纸，必须同时改
  `TARGET_SHORT_MM` / `TARGET_LONG_MM` / `BLACK_FRAME_MM`，**三者必须与实际靶纸/胶带一致**
  （内沿推算外沿完全依赖 `BLACK_FRAME_MM`）。

### 4.6 ★ 核心一：`_verify()` 稀疏透视结构验证（`140-195`）

这是**本工程最有价值的模块**，替代了"靠亮中心猜框"的旧回退（README 说旧版会误检靶纸边缘/桌面阴影）。

**做法**：把候选四边形当作物理正视面 → 建 `rect_to_image` 单应 →
在物理面上**只挑 4×(24×5) 个稀疏点** → 一次 `remap` 投影回当前帧采样。
**不生成整张 297×210 的正视图**（这是关键的省时设计：72 点变换的成本 ≈ 0）。

采样几何（`156-165`）：每条边沿胶带**厚度方向**取 5 个偏移
`offsets = tape * [0.20, 0.40, 0.60, 1.22, 1.45]`：

```
外侧 ┃◀ 黑胶带 tape ┃  纸面（亮）
  ·  ·  ·        ·  ·
  ↑  ↑  ↑        ↑  ↑
 0.2t 0.4t 0.6t 1.22t 1.45t
 └── 暗带 3 点 ──┘ └ 亮面 2 点 ┘
```

判据（`174-184`）：

```python
dark   = max(samples[:, :, :3])          # 3 个带内点取 max → 反光是"最坏情况"，从严
bright = median(samples[:, :, 3:])       # 2 个纸面点取中位
delta  = bright - dark
coverage = mean( (delta >= MIN_EDGE_CONTRAST(20)) & (dark <= bright * MAX_DARK_BRIGHT_RATIO(0.72)) )
# 接受条件：
#   ① 4 条边里至少 3 条 coverage >= MIN_EDGE_COVERAGE(0.55)
#   ② 最差那条边 coverage >= WEAK_EDGE_COVERAGE(0.25)
#   ③ 4 条边 contrast 的中位数 >= MIN_EDGE_CONTRAST(20)
```

**设计要点（改代码前必须理解）**：

1. **允许局部反光**：要求"3 条边好 + 第 4 条至少有点证据"，而不是四条边都必须完美。
2. **允许暗背景**：**没有**"外侧必须是白色"这一条。实测 fixture#1 背景灰度仅 20，照样检出。
3. **实心黑块 / 普通纸边会被拒**：因为它们没有"暗带 + 内侧亮面"的厚度结构。
4. **`score = median(edge_contrast) + 60 * mean(coverage)`（`195`）** —— ⚠
   **在主搜索路径里 `score` 根本没被使用**：`_search` 一旦有候选通过就 `return`（首中即停）。
   `score` 只在 `_line_search`（Hough 路径，默认关闭）里用于择优。
   → **想提高鲁棒性，"让候选按 score 竞争"是一个明确的、尚未实现的扩展点**（见 §八）。

返回结构（`190-195`）：`quad`(检测到的那个四边) / `outer_quad` / `inner_quad` /
`kind` / `rect_to_image` / `width_mm` / `height_mm` / `edge_profile`(四边统计) /
`contrast` / `min_contrast` / `coverage` / `score`。

### 4.7 ★ 核心二：`_refine()` 四边窄带亚像素精修（`197-243`）

`approxPolyDP` 给的四角只有 ±1px 量级的精度，这一层把它压到亚像素。

1. 每边取 `EDGE_SAMPLES=24` 个基点（`t = linspace(0.12, 0.88)`，**两端各 trim 12% 避开角点**）；
2. 沿边法向取 `offset ∈ [-radius, radius]`（跟踪 `TRACK_RADIUS=12`，初次搜索 `REFINE_RADIUS=6`）；
3. 一次 `_sample` 采完整批点，算 `offset` 方向的中心差分梯度；`kind=="outer"` 时**翻转符号**统一极性；
4. **罚项** `penalty = |offset| * 0.45` → **优先取靠近预测位置的那条边**，
   避免抓到靶纸中央的印刷圆或胶带另一侧的边缘（`216-218`，注释明说）；
5. 只保留 `strength >= 0.7 * MIN_EDGE_CONTRAST` 的样本，再按 **MAD×3** 剔除离群，要求 ≥5 个；
6. 每边 `cv2.fitLine(..., DIST_HUBER, 0, 0.01, 0.01)` → 直线 `ax+by+c=0`；
7. `np.cross(roll(lines,1), lines)` 一次算四个交点（齐次叉积 = 交点）；
8. 门限：角点位移 > `radius*2.8` 或精修后四边形不合法 → 返回 `None`。

> ⚠ **改动注意**：`gradient = values[:, :, 2:] - values[:, :, :-2]` 用 `values[:, :, :-2]` 作为左邻
> —— 这是**非中心差分**（步长不对称）。它对结果影响小，但改公式时别以为它是中心差分。
> 另外 `kind` 翻转梯度的极性是**硬约定**，新增候选族（如"内沿/外沿之外的第三种结构"）时必须同步给极性。

### 4.8 候选生成：`_candidate_quads()` 用轮廓树替代两两配对（`245-281`）

旧版是"最多 64 个四边形两两配对"（README 明确说是被删掉的旧做法）。现在：

1. `findContours(mask, RETR_TREE, CHAIN_APPROX_SIMPLE)`；
2. 过滤：`min(boundingRect)` ≥ 15 且 `cw*ch >= w*h*MIN_AREA_RATIO(0.025)`；
   `eligible` = 面积落在 `[MIN_AREA_RATIO, MAX_AREA_RATIO(0.92)]` 的轮廓；
3. 按面积降序取前 `MAX_CANDIDATES=8`，`approxPolyDP(eps = APPROX_EPSILON(0.025) × 周长)`，
   **必须刚好 4 点且凸**（注释：**不用凸包修补任意背景**，因为凸包能把桌面阴影拼成错误大四边形）；
4. `_order_quad` → `_valid_quad`；
5. **`kind` 由轮廓树深度决定**：`depth` 偶数 → `"outer"`（暗区边界 = 胶带外沿），
   奇数 → `"inner"`（亮孔边界 = 胶带内沿）；
   `kind=="outer"` 且 `hierarchy[i][2] < 0`（没有子轮廓）→ **跳过**（实心黑块没有亮孔）；
6. 返回时把 `"inner"` 排在前面（注释：亮面孔比与背景粘连的黑环更稳定）。

> **为什么这样能省时间**：一次 `findContours` 就同时拿到"外轮廓"和"内孔"，
> 不需要"枚举四边形再配对"，候选数从 O(n²) 降到 O(n)。
> 代价是**强依赖 `mask` 的极性**：`mask` 必须是 `THRESH_BINARY_INV`（**暗 → 255**），
> 否则"偶数深度 = 暗区"这个前提不成立。**新增任何生成 mask 的路径都要守住这个极性。**

### 4.9 搜索：`_search()` 阈值阶梯 + 首中即停（`283-335`）

```
roi(全分辨率坐标) → crop → 缩到 SEARCH_WIDTH=320 宽(INTER_AREA) → small
thresholds = [_locked_th (上次成功的那档), otsu, *DARK_THRESHOLDS(8 档)]  # 去重：|Δ| < 6 视为同档
for th in thresholds:                                   # ← 每档 = 1 张掩膜
    mask = threshold(small, th, THRESH_BINARY_INV) + morphologyEx(CLOSE, 3x3)
    for q, kind in _candidate_quads(mask):               # ← inner 优先、大轮廓优先
        q 映射回原图坐标
        verified = _verify(gray, q, kind)   ← 在【原分辨率】验证
        if verified: _refine(REFINE_RADIUS=6) → _verify 再验一次
             ★ 成功 → self._locked_th = th; return  ← 首中即停，不做跨档竞争
# ② Canny 兜底（"失锁后的最后辅助"）：小图 Canny → RETR_LIST → 取面积前 8 → 近似四边形 → 验两种 kind
# ③ ENABLE_LINE_RECOVERY=True 时再走 _line_search（默认关闭）
```

三个必须记住的点：

- **`_locked_th` 会被排到扫描表最前面** —— 稳态失锁重搜时第一档就命中，这是"锁定值优先"的正确做法。
- **搜索与验证的分辨率是分离的**：搜索在 320 宽小图上（便宜），`_verify` / `_refine` 在**当前帧原分辨率**上（准）。
  → 所以输出坐标**永远是原图像素**，不需要任何缩放还原。
- **`_search_count` / `_threshold_count` 每帧重置**，由 `process()` 汇总进 `result`（`407`），
  最终打到 `[perf]` 的 `searches=` 上 —— **这就是本轨道的"掩膜数"等价指标**。

### 4.10 `_line_search()` 反光断环恢复（`337-397`，默认关闭）

`HoughLinesP` → 取最长 64 段 → 以最长段为种子、最多试 3 组主方向（±28° 归组）→
每组取最多 8 条"间距 ≥5px"的线 → **对边配对**（间距 ≥ `0.24*min(w,h)`）→ 4 交点成四边形 →
`_order_quad` → `_verify` 两种 `kind` → 取 `score` 最高者。

- **为什么默认关**：README 明说"避免失锁时一次计算阻塞数秒"。
- ⚠ `LINE_MAX_PROPOSALS` / `LINE_SEARCH_INTERVAL` / `MAX_LINE_RESIDUAL_PX`
  **都是死参数**（§七.3），打开这条路之前先看 `_line_search` 里**硬编码的 `64` / `8` / `6` / `3`**。

### 4.11 输出契约：`result` 字典

下游（串口 / 显示 / 其它 Agent）只应依赖这张表：

| 字段 | 类型 | 含义 | 无效时 |
|---|---|---|---|
| `valid` | bool | 本帧是否可信 | `False` |
| `stage` | str | `track` / `roi_search` / `full_search` / `no_black_frame` | 后者 |
| `u`, `v` | float | **平滑后**的靶心（原图像素坐标） | — |
| `raw_u`, `raw_v` | float | 未平滑的靶心（**测精度/做控制时用这一对**） | — |
| `quad` | (4,2) float32 | 外沿四角（**绘制用**，= `outer_quad`） | — |
| `outer_quad` / `inner_quad` | (4,2) float32 | 胶带外沿 / 内沿四角 | — |
| `detected_quad` | (4,2) float32 | 实际被检测到的那个四边（可能是内沿） | — |
| `kind` | str | `"inner"` / `"outer"`：检测到的是哪一族 | — |
| `outer_inferred` | bool | `True` = 外沿是**由内沿 + `BLACK_FRAME_MM` 推算**出来的 | — |
| `rect_to_image` | (3,3) float64 | **物理毫米正视面 → 图像像素** 的单应矩阵 ★ | — |
| `width_mm` / `height_mm` | float | 该单应对应的物理尺寸 | — |
| `circle_points` | (72,2) | 物理圆（半径 `CIRCLE_RADIUS_MM`）投影到图像的 72 点 | — |
| `edge_profile` | list[4] dict | 每边 `dark` / `inside` / `contrast` / `coverage` | — |
| `contrast` / `min_contrast` / `coverage` / `score` | float / ndarray | 结构验证统计量 | — |
| `threshold` | int\|None | 本帧锁定的阈值 | — |
| `search_count` / `thresholds` | int | 本帧走了几次搜索 / 试了几档阈值 | — |
| `detector_ms` | float | `process()` 内部耗时 | 有 |

> **坐标语义（最重要的一条）**：`u,v` / `raw_u,raw_v` / `quad` / `circle_points`
> **全部是"当前处理帧"的像素坐标**。
> 若 `LENS_CORR=True`，那是**校正后图像**的坐标；若为 `False`，是相机原帧坐标。
> 下游主控必须知道当前用的是哪一套，否则云台会有系统性偏差。

### 4.12 绘制：`draw_result()`（`458`）

先把灰度图缩到 `show_width`（默认 480）**再**转 BGR 和画线（省 RGB 拷贝与 MaixVision 传输）。
绿色 = 外沿、黄色 = 内沿、洋红点 + `P1..P4 (x,y)` 标签、蓝色十字 = 靶心、顶部一行 `(u,v) stage ms`。
标签位置做了夹取，不会画出画面外。**输出坐标仍是原图坐标**（画布缩放只影响绘制）。

---

## 五、实测基线（本次分析实测，非作者声明）

命令：`"D:/Python311/python.exe" -B tools/test_performance.py`

```json
{
  "fixtures": 30, "detected": 29, "failed": [2],
  "max_center_error_px": 0.803,
  "negative_cases": 5, "false_positives": 0,
  "motion_max_error_px": 0.637,
  "motion_stages": ["track"×10, "roi_search", "track", "track"],
  "lost_valid": false, "reacquired": true,
  "optimized": {"median_ms": 1.012, "p95_ms": 1.549, "tracked": 60}
}
```

**结论**：30 个合成场景检出 29（唯一失败的是 #2）、最大中心误差 **0.803px**、
5 个负样本 **0 误检**、连续位移最大误差 0.637px、拿走靶纸后正确失效并成功重捕、
稳态 60/60 帧走 `track` 路径（**不扫阈值**）、PC 上稳态 **1.01ms/帧**（p95 1.55ms）。

各阶段单帧耗时（640×480，同机实测）：

| 场景 | median | `stage` | 本帧阈值档数 |
|---|---|---|---|
| 已锁定稳态 | **0.84 ms** | `track` | 0 |
| 强反光失锁（每帧全扫） | 1.51 ms | `no_black_frame` | 9 |
| 纯色无靶纸 | 0.46 ms | `no_black_frame` | 9 |
| **噪声无靶纸** | **7.49 ms** | `no_black_frame` | 8 |

> ⚠ **最坏帧是"噪声画面"而不是"反光画面"**：噪点让掩膜碎成大量小轮廓，
> `_candidate_quads` 里那句"对**所有**轮廓算 `boundingRect` + `contourArea`"没有上界。
> 640×480 就 7.5ms，换成板上 1280×720 且 ARM 单核，线性外推就是**几十~上百 ms/帧**。
> 这是本轨道**最需要补的性能缺口**（A 轨的做法见 §八.3）。

---

## 六、`config.py` 参数全表（含"是不是活的"）

> 下面标 **死** 的项在当前代码里**没有任何引用**（除被测试的 `LegacyConfig`），
> 改了**不会有任何效果** —— 这是接手时最容易踩的坑。

### 6.1 相机 / 主循环（`main.py` 用）

| 参数 | 值 | 说明 |
|---|---|---|
| `CAMERA_W/H` | 640/480 | 灰度直出省 3× 帧缓冲 |
| `CAMERA_FPS` / `CAMERA_BUFFERS` | 30 / 2 | 默认 3 缓冲 → 改 2 |
| `SKIP_FRAMES` | 10 | 丢预热帧 |
| `LENS_CORR` / `LENS_STRENGTH` | True / 1.5 | ⚠ `1.5` 是旧工程经验值，**不是本机标定参数**；启用时坐标属于校正后图像 |
| `SHOW` / `SHOW_WIDTH` / `SHOW_FPS` | True / 480 / 15 | 显示独立限频；`SHOW=False` 可做纯检测对照 |
| `OPENCV_THREADS` | 1 | 小图上多线程比运算本身贵 |
| `LOG_INTERVAL` | 2.0 | `[perf]` 行间隔 |
| `ENABLE_UART` / `UART_PORT` / `UART_BAUD` | False / `/dev/ttyS0` / 115200 | 输出 `@[u,v]%` / `@[invalid]%` |

### 6.2 检测 / 验证（`vision/rectangle_center.py` 用）

| 参数 | 值 | 活 | 影响面 |
|---|---|---|---|
| `CANNY_LOW/HIGH` | 35 / 120 | ✔ | 仅 Canny 兜底 + 可选 Hough |
| `DARK_THRESHOLDS` | (25,40,55,70,85,100,120,145) | ✔ | 失锁时扫的 8 档（外加 Otsu 与上次锁定档） |
| `SEARCH_WIDTH` | 320 | ✔ | 失锁搜索的工作宽度（越小越快、远距越易漏） |
| `ENABLE_LINE_RECOVERY` | False | ✔ | 打开会显著增加最坏帧耗时 |
| `MAX_CANDIDATES` | 8 | ✔ | 每张掩膜最多拟合的轮廓数 |
| `EDGE_SAMPLES` | 24 | ✔ | `_verify` / `_refine` 的每条边采样数 |
| `REFINE_RADIUS` / `TRACK_RADIUS` | 6 / 12 | ✔ | 精修窗口 / 跟踪窗口（像素） |
| `ROI_PADDING` | 0.20 | ✔ | ROI 搜索外扩比例（下限 24px 硬编码） |
| `MIN_EDGE_COVERAGE` / `WEAK_EDGE_COVERAGE` | 0.55 / 0.25 | ✔ | ★ 结构门：3 边达标 + 最差边下限 |
| `MIN_EDGE_CONTRAST` | 20.0 | ✔ | ★ 绝对灰度差门（暗场可降到 15） |
| `MAX_DARK_BRIGHT_RATIO` | 0.72 | ✔ | ★ 黑/白亮度比门（越小越严） |
| `MIN_AREA_RATIO` / `MAX_AREA_RATIO` | 0.025 / 0.92 | ✔ | ★ 候选面积门（**远距离是否锁得住的关键**） |
| `APPROX_EPSILON` | 0.025 | ✔ | `approxPolyDP` 的 `eps/周长`（0.005~0.08 都能给 4 点） |
| `BLACK_FRAME_MM` | 18.0 | ✔ | ★ 胶带宽度，**内沿推算外沿**依赖它 |
| `TARGET_SHORT_MM` / `TARGET_LONG_MM` | 210 / 297 | ✔ | ★ 物理靶纸尺寸，必须匹配实际靶纸 |
| `CIRCLE_SAMPLES` / `CIRCLE_RADIUS_MM` | 72 / 2.0 | ✔ | 输出 `circle_points`（不参与中心计算） |
| `FILTER_RESET_PX` | 25.0 | ✔ | 跳变超过它就**放弃平滑**直接取 `raw` |
| `CENTER_ALPHA` | 0.70 | ✔ | EMA 系数；0 = 关闭平滑（等价用 `raw_u/raw_v`） |
| `SEARCH_BUDGET_MS` | 25.0 | **死** | 设计过"时间预算"，从未接线 |
| `SEARCH_MAX_PROPOSALS` | 1 | **死** | 同上 |
| `SEARCH_GEOMETRY_STEPS` | 8 | **死** | 同上 |
| `MAX_CONTOURS` | 128 | **死** | ⚠ 唯一一个能防"轮廓爆炸"的参数，却没接线 |
| `LINE_MAX_PROPOSALS` | 36 | **死** | Hough 路径用硬编码 64 |
| `LINE_SEARCH_INTERVAL` | 15 | **死** | 节流未接线 |
| `ACQUIRE_CONFIRM_FRAMES` | 3 | **死** | 没有"连续 N 帧确认"机制 |
| `CONFIRM_CORNER_PX` | 12.0 | **死** | 同上 |
| `RECHECK_INTERVAL` | 20 | **死** | 无周期性复检 |
| `MAX_LINE_RESIDUAL_PX` | 2.0 | **死** | 无残差校验 |
| `MIN_TRANSITION_COVERAGE` | 0.50 | **死** | 被 `MIN_EDGE_COVERAGE` 取代 |
| `MAX_BOUNDARY_WIDTH_ERROR` | 0.30 | **死** | 被 `MAX_DARK_BRIGHT_RATIO` 取代 |
| `MIN_ANGLE_DEG` / `MIN_INNER_RATIO` / `MAX_INNER_RATIO` / `DARK_CENTER_MIN` | — | **死** | 只在 `test_performance.py` 的 `LegacyConfig` 里被覆盖 |

**合计 12 个参数在当前实现里完全无效**（另有 4 个只被测试用到）。
→ **建议**：接手第一件事就是把死参数要么接线、要么删掉并在文档标注，
否则"改了没反应"会被误判成"算法不work"。

---

## 七、已知问题与坑（按优先级）

### 7.1 ★ 最坏帧不可控（性能缺口）

`_candidate_quads` 对所有轮廓做 `boundingRect`+`contourArea`，无上限；
`_search` 一次最长要试 **9 档阈值 + 1 次 Canny**（+ 可选 Hough）。
实测噪声画面 7.49ms@640×480。缺的三件东西：
① 轮廓数/候选数上限（`MAX_CONTOURS` 未接线）；
② "掩膜碎成渣"的**廉价前置检测**（A 轨的 `boundary_estimate`，0.3ms，与 `findContours` 耗时严格单调）；
③ 单帧**时间预算**（`SEARCH_BUDGET_MS` 未接线）。

### 7.2 ★ 强反光下永久失锁（实测复现）

`test_performance.py` 的 fixture#2（`glare=True`，两个亮椭圆压在胶带上）**永远不检出**。
实测其最佳候选的 `coverage = [0, 1, 0, 1]`、四边 `contrast = [44, 199, 44, 198]`：
两条长边的 `delta ≥ 20` 满足，但 **`dark <= bright*0.72` 整条不成立**（反光把胶带照亮）。
→ **这是"安全失败"：拒检而不是误检**（负样本 0 误检）。
想救它有两个方向：放宽 `MAX_DARK_BRIGHT_RATIO` 到 ~0.9；
或把"暗带取 max"改成**取分位数**（允许一定比例的采样点被反光覆盖）。

### 7.3 死参数 12 个

见 §六。`MAX_CONTOURS` 尤其需要注意（它是性能缺口的一半）。

### 7.4 README 与代码漂移

`README.md` 写构建号 `2026-10-05-fast-edges-v1`，而 `main.py` 是 `...-v2`。
`README` 也还没有覆盖 `[perf]` 里的 `lens` / `bridge` / `uart` / `thresholds` 等新字段。
→ 改代码时**顺手同步 README**，否则"板子上跑的到底是哪版"又变成猜谜。

### 7.5 ★ `app.yaml` 的 `exclude` 有两处缺陷（会被打进应用包）

```yaml
exclude: [__pycache__, README.md, test_pc.py, tools, screenshot_optimized.png]
```

- **漏了 `screen_check.png`（151KB 调试截图）** → 实测模拟打包：包内会出现
  `app.yaml / assets/icon.png / config.py / main.py / screen_check.png / vision/__init__.py / vision/rectangle_center.py`
  —— 共 7 项，其中**那张截图不该在**（照 §五的规范应为 6 项）。
- **`screenshot_optimized.png` 是不存在的文件**，`test_pc.py` 也不存在（本轨道叫 `test_performance.py`）。
  stale 条目本身无害，但**会掩盖"真正的截图没被排除"**。
- 另外新增的文档 `*.md` 也必须补进 `exclude`。

### 7.6 单帧即输出，无多帧确认

`ACQUIRE_CONFIRM_FRAMES` 没接线：一次通过 `_verify` 就会输出（并进入跟踪）。
目前靠 `_verify` 的结构门兜底（实测负样本 0 误检），但**没有时间维度的确认**。
下游云台如果在锁定的第一帧就开始动作，会有被单帧误检带偏的风险。

### 7.7 `_to_gray` 兜底分支少一次通道数检查

最后一行 `cv2.cvtColor(b, cv2.COLOR_BGR2GRAY)` 没有 `if b.shape[2] == 1: return b[:, :, 0]`。
在"灰度直出但形状判定全落空"的固件上，会退化成 A 轨踩过的 `'scn' is 1` 崩。
（当前 MaixCAM2 上走不到这一支，属于稳健性缺口而非线上 bug。）

### 7.8 硬编码常量未进配置

- `_valid_quad`：最短边 `12`、角正弦门 `0.20`；
- `_candidate_quads`：`boundingRect` 最小边 `15`；
- `process` ROI：外扩下限 `24`；
- `_line_search`：`64` / `8` / `6` / `3` / `5`px / `0.24`；
- `_refine`：罚项系数 `0.45`、强度门 `0.7×`、MAD `3`、最少 `5` 点。

→ 远距离/小目标相关的（`12` / `15`）应该提成配置项。

### 7.9 时间平滑只有 EMA，无速度项

`CENTER_ALPHA=0.70` 的 EMA 在快速运动时**必然滞后**；`FILTER_RESET_PX=25` 又会在
"抖一下"时直接放弃平滑（等价于不平滑）。
A 轨在这件事上做过完整实验并得到结论（"抖动引起的位移是必须跟踪的真实运动，低通只会造成滞后"），
B 轨若要提升动态性能，**应直接借鉴 A 轨的常数速度卡尔曼 + q 自适应，不要加大 EMA 权重**。

### 7.10 `process()` 的 `timestamp` 参数未使用

`process(self, gray, timestamp=None)` —— 签名留了口子，但内部没用（时间都从 `time.perf_counter()` 取）。
做多帧确认/预测时可以直接用这个参数，不用改签名。

---

## 八、面向"新功能开发"的扩展点

> 每条给出：**在哪加 → 加什么 → 会破坏什么 → 怎么验**。
> 通用前提：改完必须跑 `tools/test_performance.py`（正样本 + **负样本拒检**都要看），
> 并遵守同目录 `MaixCAM应用代码规范.md`（改 `main.py`/`config.py`/`vision/*` 后要更新 `BUILD_ID`）。

### 8.1 让候选"按 score 竞争"（提高鲁棒性，改动小，收益明确）

- **在哪**：`_search` 的阈值循环里（`296-311`）。
- **改什么**：不要 `return verified` 就结束，而是把 `verified` 存进 `best`（比 `score`）；
  可以只在"同一档阈值内的多个候选"之间竞争，也可以跨档竞争（成本 = 多验几个候选，不增加掩膜数）。
- **风险**：首中即停改择优 → 最坏帧耗时上升（每档多验几个候选）。建议先只在**档内**竞争。
- **怎么验**：`test_performance.py` 的 `max_center_error_px` 不升、负样本仍 0 误检；
  新增"同一帧里同时存在真靶纸与干扰黑框"的用例。

### 8.2 提高强反光下的检出率（救 fixture#2）

- **在哪**：`_verify` 的 `dark = np.max(samples[:, :, :3], axis=2)`（`174`）。
- **改什么**：改成 **分位数**（如 `np.percentile(..., 60)`）+ 允许一定比例的采样点被反光覆盖
  （即把 `coverage` 门从"比例"改成"去掉最差的 k% 再算"）；
  或给 `MAX_DARK_BRIGHT_RATIO` 增加"至少 k 条边满足"的软化形式。
- **风险**：放宽暗带门 = 更容易把"亮块贴在亮墙上"当成靶纸 → **必须**看负样本。
- **怎么验**：fixture#2 从"永久失败"变为可检出（`failed` 列表应清空），
  且 5 个负样本仍 0 误检；再加 2~3 个"反光 + 无靶纸"的负样本。

### 8.3 补最坏帧耗时上界（性能，最关键）

三件可独立落地的事：

1. **轮廓数上限**：把 `MAX_CONTOURS` 接到 `_candidate_quads` 里 —— 先按面积取前 N 个轮廓再算 `contourArea`/`boundingRect`。
   注意：`findContours` 本身的成本由**创建的轮廓对象数**决定，事后截断省不掉它，
   所以还得配 ②。
2. **廉价碎裂检测**：照 A 轨的 `boundary_estimate()`（隔点采样统计 0↔255 跳变数，约 0.3ms，
   与 `findContours` 耗时严格单调），**超过阈值才**走连通域预筛 / 降低处理分辨率 / 直接跳过该档阈值。
   A 轨实测交叉点在 `est ≈ 13000` —— **无条件预筛会给每张掩膜固定加 17~20ms**，别无条件开。
3. **单帧时间预算**：把 `SEARCH_BUDGET_MS` 接成"软预算"：本次搜索累计超过预算就提前退出（本帧返回无效），
   下一帧从上次中断的阈值档继续（用轮转游标）。这样**单帧耗时有上界**，代价是失锁恢复慢一点。

- **怎么验**：① 用噪声画面 + 大目标画面测 `process()` 的 p95；
  ② **必须同进程交错 A/B**（跨进程/跨时间对比会被机器负载污染，实测同一 `morphologyEx` 隔十分钟差 7 倍）；
  ③ 确认稳态 `track` 耗时不变（改性能不许动精度旋钮：`MIN_AREA_RATIO` / `MIN_EDGE_*` / `MAX_DARK_BRIGHT_RATIO`）。

### 8.4 毫米度量类新功能（本轨道最大的差异化能力，几乎"已具备、只差接线"）

`result["rect_to_image"]` 是 **mm → px** 的单应，`inverse_map_points()` 已写好但未使用。
- **直接可做**：靶心到靶纸上任意 mark 的**真实毫米距离**、打环半径、激光落点偏差 cm。
- **若下游要"局部尺度"**：不要用标量 `px_per_cm`（透视下有 36% 方向性误差），
  而要输出单应在靶心处的**局部雅可比的两个方向尺度**：
  `h(x+1,y)-h(x,y)` 与 `h(x,y+1)-h(x,y)`（单位 px/mm）。
- **风险**：`TARGET_*_MM` / `BLACK_FRAME_MM` 与实际不符时，毫米量会整体偏。
- **怎么验**：用"已裁断四边"或已知 mm 的合成图，核对 mm→px→mm 往返误差
  （A 轨实测往返 8.6e-14mm，胶带宽度反算 18.09mm vs 真值 18.0mm）。

### 8.5 抗抖 / 动态性能（借鉴 A 轨，不要自己重造）

- **在哪**：`process()` 末尾的 `CENTER_ALPHA` EMA（`436-443`）。
- **改什么**：换成"常数速度卡尔曼（位置+速度）+ 过程噪声 q 自适应"，
  并把 `TRACK_RADIUS` 跟踪路径的测量作为观测。
- **必须照抄 A 轨的八条实测教训**（`README.md §三″`）：机动判据必须扣掉测量噪声、q 按对数映射、
  单帧速度修正量上限、形状门限永不自我放开、离群点不喂养机动尺度……
- **怎么验**：四项指标同时看 —— **跟手 / 抑噪 / 静止平滑 / 拒离群**，只看一项一定调坏。
- ⚠ **最重要的一条认知**：抖动引起的位移是**必须跟踪的真实运动**，加大低通只会造成滞后。
  真要减抖只能硬件层（**缩短曝光**，当前 `LENS_STRENGTH` / 曝光都是旧工程继承值）。

### 8.6 多帧确认（降低单帧误锁风险）

- **在哪**：`process()` 成功分支（`432`）。
- **改什么**：把 `ACQUIRE_CONFIRM_FRAMES=3` 接成"新目标需连续 N 帧通过且角点互相接近
  （`CONFIRM_CORNER_PX=12`）才转为 `valid`"，期间 `stage="confirm"`。
- **风险**：确认期会**延迟输出**（云台要多等几帧）；抖动/快速运动时可能永远凑不齐 N 帧 → 需要超时放行。
- **怎么验**：失锁→锁定的时延（`test_performance.py` 的 `acquire()` 返回第几帧）；
  误检场景下是否被拦住。

### 8.7 支持其它靶纸 / 目标尺寸自适应

- **在哪**：`_layout()`（`132`）+ `config.py` 的 `TARGET_*_MM`。
- **改什么**：加"候选物理尺寸候选集"（如 A4/A3/自定义），或从 `_verify` 的 `edge_profile`
  反推实际胶带宽度并据此重算 `BLACK_FRAME_MM`。
- **风险**：`_layout` 决定单应尺度，猜错会让 `_verify` 的采样点整体错位 →
  **本来能检出的变成检不出**。改这里必须重跑全部 30 个 fixture。
- **怎么验**：新增不同尺寸/长宽比的合成靶纸用例；确认旧用例不退化。

### 8.8 串口协议扩展

- **在哪**：`main.py:_send_uart`（`44`）+ `config.py` 的 `ENABLE_UART/UART_PORT/UART_BAUD`。
- **改什么**：加"毫米偏差 / 尺度 / 置信度 / `kind`"字段。**建议**升成带帧头+长度+校验的二进制包
  （A 轨用的是 `AA 55 | valid | u(2B) | v(2B) | scale*100(2B) | XOR`）。
- **必须**：无效帧也要发（明确告诉下游"这帧没结果"），不能靠"不发"表达失败。
- **怎么验**：抓包核对字节序/校验；板端用串口助手确认无粘包/断包。

### 8.9 加测试用例的规范位置

`tools/test_performance.py` 的 `fixtures()`（`38`）返回 `(image, truth)` 列表，
`run()` 里用 `acquire()` 逐图驱动并断言：

```python
assert len(failed) <= 8          # 允许少量失败（当前实际 1 个）
assert max(errors) < 3           # 中心误差上限（当前 0.803px）
assert not false_positives       # ★ 负样本必须 0 误检
assert max(movement_errors) < 3  # 连续位移不漂
assert not lost['valid'] and reacquired['valid']
assert report['optimized']['tracked'] == 60   # ★ 稳态必须 60/60 走 track
```

**加功能的正确姿势**：先加**负样本**（画面里没有靶纸的用例）再加正样本。
只加正样本会把算法调成"什么都敢锁"，而**误检比漏检更糟**（云台会照着错位置打）。

---

## 九、5 分钟上手（给接手的新 Agent）

1. 读 `config.py`（69 行）→ 全部可调项一目了然；
2. 读 `vision/rectangle_center.py` 的 `process()`（399-455）→ 拿到状态机与输出契约（§4.3 / §4.11）；
3. 读 `_verify()`（140-195）→ 理解"稀疏透视验证"这个核心思想（§4.6）；
4. 跑一次 `"D:/Python311/python.exe" -B tools/test_performance.py` → 拿到基线数字，确认环境可用；
5. 跑 `"D:/Python311/python.exe" -B tools/check_app.py --manifest` → 确认打包清单与规范（§七.5 的两处缺陷）；
6. 要动算法 → 先看 §七（坑）与 §八（扩展点），别重复踩。

---

## 十、与轨道 A 的对照速查

| 维度 | A（`25Efork/` 根，DeepSeek） | B（本目录，GPT） |
|---|---|---|
| 候选生成 | 多阈值二值化 + **ring / paper 双族** | 阈值/Canny + **轮廓树孔环关系**（`RETR_TREE` 深度判族） |
| 选优 | **结构打分**（fill/rim/ring/aspect/contrast 加权 + 硬性否决） | **阈值阶梯首中即停**（`score` 只在 Hough 路径用） |
| 角点 | 极值顶点 + 去重 + 质心极角排序 + `fitLine` 精修 | `approxPolyDP` + **四边法向窄带梯度极值**（带近邻罚项）+ `fitLine` + 交点 |
| 中心 | **对角线交点**（射影不变量，不建单应） | **显式 8×8 单应**（Hartley 归一化 + 向量化高斯消元）→ 物理圆心逆映射 |
| 尺度 | 标量 `px_per_cm`（**有 36% 方向性误差**） | 单应矩阵（各向异性正确）+ 72 点圆 |
| 动态 | 常数速度卡尔曼 + q 自适应 + 形状门限（成熟） | EMA(`CENTER_ALPHA`) + `FILTER_RESET_PX`（简单、有滞后） |
| 失锁 | 缩小图 → 全分辨率升级 + **掩膜预算 / 碎裂预筛** | 缩小图(320) 阈值阶梯 → Canny → 可选 Hough |
| 性能指标 | `掩膜=N` / `保持=Y/N` | `[perf]` 分段 ms / `searches=` / `thresholds=` |
| 性能缺口 | 已治理（有 `boundary_estimate` / 预算 / 游标） | **未治理**（§7.1） |
| 毫米度量 | 弱（标量） | **强（单应已就绪）** |
| 负样本拒检 | 18 硬场景 + 5 负样本（`test_hardscene.py --ab`） | 30 场景 + **5 负样本**（`test_performance.py`，实测 0 误检） |

> **互补关系很清楚**：A 轨的**动态跟踪与性能治理**成熟，B 轨的**几何/毫米度量**更完整。
> 任何一轨要补短板，优先去另一轨抄已经验证过的结论，**不要从零试**。
