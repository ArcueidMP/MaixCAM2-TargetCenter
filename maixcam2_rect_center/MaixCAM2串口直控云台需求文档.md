# MaixCAM2 串口直控步进云台 —— 需求与实现指南（B 轨）

> **文档性质**：需求 + 设计 + 给 GPT 的实现指南。**本轮只改文档，不改代码。**
> **适用目录**：`maixcam2_rect_center/`（B 轨：Canny + 四边形 + 单应 + 72 点圆 + 光斑/偏差）。
> **撰写日期**：2026-10-06 初版 / **2026-10-07 v2 修订**（按用户明确的硬件方案重写）
> **对照现状**：`main.py` 的 `BUILD_ID = 2026-10-06-laser-spot-v1`，`app.yaml version: 1.2.0`
> **协作提示**：写在 B 轨目录下（用户指定），但 §3 的外设事实对两轨通用。提交时按路径
> `git add maixcam2_rect_center/MaixCAM2串口直控云台需求文档.md`，前缀 `docs:`，信息里点明"由 A 轨侧撰写、内容对两轨通用"。

---

## ★ v2 修订记录（v1 里被推翻的假设，先读这一节）

v1 是按"执行器类型未知、UART1/UART2 里留一条给底盘"写的。**用户 2026-10-07 给出实际方案，
以下为硬事实，v1 的对应章节作废**：

| 项 | **v2 实际方案（以此为准）** | v1 的错误/待定假设 |
|---|---|---|
| UART1 | **控制 1 号步进电机 → 云台 yaw 角** | 以为留给底盘 MSPM0 |
| UART2 | **控制 2 号步进电机 → 云台 pitch 角** | 以为作云台主链路、UART1 待定 |
| 与 3507 通信 | **SPI**（不是 UART） | v1 未涉及 |
| 执行器 | **ZDT（张大头）闭环步进驱动器 ×2 + 齿轮减速** | 泛泛讨论舵机/驱动器选型 |
| 协议 | **ZDT 原生 Emm_V5.0 协议**（不再自造二进制帧） | 自造 15B 定长帧 + 10B 状态帧 |
| 闭环输入 | **`dx/dy`：视觉给出的"靶心 − 光斑"像素差**（GPT 已实现） | 假设要自己标定光斑与视差 |
| 视觉接口 | `vision/aiming.py` 的 `add_aiming()` → `aim_valid / dx / dy` | 无 |
| 机构 | 相机与激光笔**同装在俯仰平台上**、**齿轮减速**（2026-10-06 实拍照片确认） | 仅纸面假设"大致共轴" |

**v1 仍有效的部分**：§2 题目指标换算、§3 MaixCAM2 外设与电平事实、以及"闭环不能用视觉平滑值
（任何视觉滞后都直接变成瞄准误差）"这条分析 —— 它现在落在 §5.2。

---

## 0. 一页速览（GPT 从这里开始）

> 目录：[§0 速览](#0-一页速览gpt-从这里开始) ｜ [§1 目标](#1-目标与范围) ｜ [§2 题目指标](#2-题目要求--可执行指标) ｜
> [§3 MaixCAM2 外设](#3-maixcam2-外设事实) ｜ [§4 步进执行器](#4-被控对象zdt-闭环步进--齿轮云台) ｜
> [§5 视觉契约](#5-视觉契约gpt-已实现控制器只读这些字段) ｜ [§6 ZDT 协议](#6-zdtemm_v50串口协议) ｜
> **[§7 ★ 控制算法](#7--控制算法把光斑搬进黑框中心)** ｜ [§8 代码清单](#8-代码落地清单给-gpt) ｜
> [§9 调试路线](#9-上板调试路线) ｜ [§10 坑](#10-风险与坑清单) ｜ [§11 验收](#11-验收标准dod) ｜
> [§12 待确认](#12-待确认事项) ｜ [§13 参考](#13-参考资料)

### 0.1 谁连谁

```
                      MaixCAM2 (MaixPy / Ubuntu)
        ┌──────────────────────────────────────────────────────┐
        │  视觉: rectangle_center.py  → 靶心 (u,v)              │
        │        aiming.py            → 光斑(laser_u,v)、dx,dy  │
        │        gimbal_aim.py(新)    → 每轴 (方向, 脉冲数)      │
        ├──────────────────────────────────────────────────────┤
 左侧排针 │ B0  → UART2_TX ──► 2 号 ZDT 驱动器 RX   控制 pitch │
 2×6P    │ B1  ← UART2_RX ──  2 号 ZDT 驱动器 TX   /dev/ttyS2 │
        │ GND ──────────── 共地                                │
 右侧排针 │ A30 → UART1_TX ──► 1 号 ZDT 驱动器 RX   控制 yaw   │
 2×6P    │ A31 ← UART1_RX ──  1 号 ZDT 驱动器 TX   /dev/ttyS1 │
        │ GND ──────────── 共地                                │
 左侧排针 │ B20 → SPI2_SCK ─┐                                   │
        │ B18 → SPI2_MOSI │  与 MSPM0G3507（3507）通信          │
        │ B19 ← SPI2_MISO │  （本轮不实现，仅占位说明）          │
        │ B21 → SPI2_CS1 ─┘                                   │
        └──────────────────────────────────────────────────────┘
```

**要点**：UART1 与 UART2 **各自独占一个电机**（不是共总线），所以不存在寻址冲突；
但仍要把两个驱动器的 `ID_Addr` 设成不同值（1 / 2），便于区分返回帧、也便于将来改共总线。

### 0.2 电机编号与转向定义（用户给定，必须原样实现）

| 电机 | 串口 | 轴 | 面朝电机转轴看：**顺时针 (CW)** | **逆时针 (CCW)** |
|---|---|---|---|---|
| **1 号** | UART1 | yaw | 云台 yaw **向左** | 云台 yaw **向右** |
| **2 号** | UART2 | pitch | 云台 **仰角增大**（抬头） | 云台 **俯角增大**（低头） |

ZDT 协议里：**方向字节 `0x00` = CW（顺时针）、`0x01` = CCW（逆时针）**，手册的口径也是
"面朝电机轴"，与用户口径一致 → 可直接一一对应。

### 0.3 视觉给的误差定义（GPT 已实现，别改）

`vision/aiming.py → add_aiming()` 写入：

| 字段 | 含义 |
|---|---|
| `dx = u_靶心 − u_光斑` | **> 0 表示靶心在光斑右侧** |
| `dy = v_靶心 − v_光斑` | **> 0 表示靶心在光斑下方** |
| `aim_valid` | 靶心与光斑**都**有效才为 True；任一失效即 False，且 `dx/dy = None` |
| `laser_valid` / `laser_source` | 光斑是否测到 / `measured` 或 `fixed` |

**控制目标：把 `dx`、`dy` 都打到 0**（= 光斑压在黑框对角线交点上）。

### 0.4 ★ 符号结论（推导见 §7.2）

> **`dx > 0`（靶心在右）→ 1 号电机发 CCW(`0x01`)；`dx < 0` → CW(`0x00`)**
> **`dy > 0`（靶心在下）→ 2 号电机发 CCW(`0x01`)；`dy < 0` → CW(`0x00`)**
>
> 即两个轴统一成一句：**「误差为正 → 发 CCW」，不需要再乘任何负号。**

⚠ 这是**纸面推导**，必须用 §9 的 P2 开环试验实测确认（发一帧固定方向，看光斑在图像里往哪走）。
若实测反了，**只把 `DIR_SIGN_*` 配置置 `−1`**，不要改电机接线、不要在公式里再加一个负号。

### 0.5 下一步要写的代码（3 个文件）

| 文件 | 作用 |
|---|---|
| `vision/stepper_link.py` | ZDT Emm_V5.0 帧构造/解析 + 两条串口（pinmap、非阻塞收发、看门狗） |
| `vision/gimbal_aim.py` | `dx/dy` → 每轴 (方向, 脉冲数)：状态机、死区、限幅 |
| `tools/test_gimbal_aim.py` | 假链路 + 假电机模型，离线验证收敛/失锁/限幅（纯 PC，不依赖板子） |

`main.py` 只接三处：① 启动时建链路；② 每帧 `aim` 后 `publish`；③ `[perf]` 加 `rx=` / `aim=`。

---

## 1. 目标与范围

**要做成什么**：MaixCAM2 既是视觉模块也是瞄准模块的**主控**——`aiming.py` 算出"靶心 − 光斑"的
像素偏差 `(dx, dy)`，`gimbal_aim.py` 把它换算成两个步进电机的位移，通过 UART1/UART2 发给
ZDT 驱动器，闭环把 `(dx, dy)` 压到 0，使激光光斑落在黑框对角线交点（靶心）上。

一句话验收：**给定任意位置与姿态，启动后 2s（指定位置 4s）内光斑痕迹距靶心 ≤2cm。**

| 本轮做 | 本轮不做 |
|---|---|
| 收集 MaixCAM2 外设事实（UART×2 + SPI） | 改任何代码 |
| 摸清 ZDT 协议并钉到字段级、给参考实现 | 实现 SPI 与 3507 的协议 |
| 把 `dx/dy` → 步进脉冲 的转换推导清楚 | "沿 6cm 红圆画圆"（发挥部分，二期） |
| 给出控制律、死区/回差处理、状态机、调试阶梯 | 激光开关的 GPIO 控制（先按常亮） |
| 给出 config 参数表与文件清单 | 动 B 轨现有视觉算法 |

---

## 2. 题目要求 → 可执行指标

### 2.1 E 题关键条目（2025 本科组 E《简易自行瞄准装置》）

| 项 | 要求 |
|---|---|
| 装置 | 寻迹小车 + 瞄准模块；**小车行驶必须用 TI MSPM0**；瞄准模块以小车为载体，二维云台控制蓝紫激光笔（建议 405nm、≤10mW），靶上光斑直径 ≤0.5cm |
| 供电 | MSPM0 与瞄准模块**分别独立开关**控制、各带指示灯 |
| 轨迹 | 外沿 100cm×100cm，边线黑 1.8cm±0.2cm |
| 靶 | 距 AB 外 50cm；A4 紫外感光纸；**1.8cm 宽黑胶带**四边贴矩形勾出外轮廓；红笔记号笔标靶心（≤0.1cm）；以靶心为圆心画 2/4/6/8/10cm 红圆 |
| 基本(2) | 位置姿态自定，**2s 内**击中靶心，D1 ≤ 2cm |
| 基本(3) | 指定位置、**瞄准方向任意指定**，**4s 内**自动瞄准，D1 ≤ 2cm |
| 发挥 | 行驶中连续发光；N=1 圈 t≤20s，D1≤2cm；沿 6cm 红圆同步画圆 D2≤2cm、同步误差 <1/4 圈 |
| 评分 | Di ≤ 2cm 满分，每 +1cm 扣 1 分 |

### 2.2 推导出的设计输入

工作距离：靶在 AB 外 50cm，小车可在 1m×1m 内任意位置 → **设计 0.4 ~ 2.0m，典型 1.5m**。

| 指标 | 数值 | 来源 |
|---|---|---|
| 允许角误差（满分） | ≤ **0.76°** | atan(2cm / 150cm) |
| 允许像素误差预算 | ≈ **7.4px @1.5m** | fx(≈554px) × 0.02m / 1.5m |
| 视觉静态精度（实测） | **0.80px ≈ 0.22cm @1.5m** | B 轨 `test_performance.py` |
| 时间预算 | 60 帧(2s) / 120 帧(4s) @30fps | |

> **结论不变**：视觉有近 10 倍余量；瓶颈在 **链路可靠性 + 齿轮回差 + 光斑能否检出**。
> 现在闭环直接建在 `dx/dy` 上，**v1 里最头疼的"激光-相机视差标定"被彻底绕过了**——
> 光斑的实测坐标本身就包含视差。

---

## 3. MaixCAM2 外设事实

来源：Sipeed Wiki《UART / PINMAP》+ MaixCAM2 板卡引脚图 v1.0 + MaixCDK `maix_pinmap.cpp`（maixcam2 分册）交叉验证。

### 3.1 串口映射

| 外设 | 设备路径 | TX | RX | 默认引脚？ | 本工程用途 |
|---|---|---|---|---|---|
| UART0 | `/dev/ttyS0` | A28 | A29 | 是 | **系统终端/日志，禁用** |
| **UART1** | `/dev/ttyS1` | **A30** | **A31** | 否，**需 pinmap** | **1 号电机（yaw）** |
| **UART2** | `/dev/ttyS2` | **B0** | **B1** | 是（专用引脚） | **2 号电机（pitch）** |
| UART3 | `/dev/ttyS3` | B2 | B3 | 否 | 备用 |
| UART4 | `/dev/ttyS4` | A21 | A22 | 是（1.25mm 6P 座） | 备用 |

⚠ **`config.py` 现在的 `UART_PORT = "/dev/ttyS0"` 是错的**（那是系统日志口），必须改成 ttyS1 / ttyS2 两条。

### 3.2 引脚复用速查（MaixCAM2）

| 引脚 | 可选功能 | 本工程 |
|---|---|---|
| A30 | `GPIOA30` / `PWM6` / **`UART1_TX`** / `SPI2_CS3` | 用 UART1_TX → 放掉 PWM6、SPI2_CS3 |
| A31 | `GPIOA31` / `PWM7` / **`UART1_RX`** | 放掉 PWM7 |
| B0 / B1 | `GPIOB0/1` / **`UART2_TX/RX`** | 专用，无冲突 |
| **B18** | `GPIOB18` / **`SPI2_MOSI`** | ← 3507 |
| **B19** | `GPIOB19` / **`SPI2_MISO`** | ← 3507 |
| **B20** | `GPIOB20` / **`SPI2_SCK`** | ← 3507 |
| **B21** | `GPIOB21` / **`SPI2_CS1`** | ← 3507 片选 |
| A0 / A1 / A4 | SPI1_MOSI / SPI1_MISO / SPI1_SCK | SPI1 备选 |
| **A2** | `SPI1_CS0` | **出厂被防呆堵住 → SPI1 不可用** |
| A6 / B25 / B29 / D2 | 状态灯 / 照明 LED / LED / 用户按键 | 别占 |
| A21/A22、A0/A1 | UART4、I2C6（1.25mm 6P 座） | 备用 |

> **SPI 结论**：`SPI1` 因为 `CS0` 在 A2 上被物理堵住而不可用 → **与 3507 的 SPI 用 `SPI2`、
> 片选用 `B21`**，正好与 UART2 同在左侧 2×6P 排针上，接线顺手。MaixPy 侧用 `maix.spi`
> （API 细节以官方文档为准，本轮不实现）。

### 3.3 电气与接线（硬约束）

| 项 | 要求 | 后果 |
|---|---|---|
| 电平 | IO 一律 **3.3V** | 直接灌 5V 会损坏引脚 |
| 接线 | **TX ↔ 对方 RX，RX ↔ 对方 TX，GND 必须共地** | 不共地 = 乱码 |
| 板上 5V | Type-C 供电 3.7~5V；**仅电池时 3.7~4.2V** | 不能给电机供电 |
| 波特率 | **115200（8N1、无流控）**，必须与 ZDT 的 `UartBaud` 一致 | 不一致 = 全乱 |
| 电机电源 | **独立供电**，与 MaixCAM2 共地；电源入口并 470~1000µF | 否则电机启停拉低母线 → MaixCAM 复位 |
| UART0 禁忌 | TX 是启动模式脚，**上电被拉低会开不了机** | 别碰 UART0 |

### 3.4 pinmap（用 UART1 必须做）

```python
from maix import uart, pinmap, err

for pin, func in (("A30", "UART1_TX"), ("A31", "UART1_RX"),
                  ("B0", "UART2_TX"), ("B1", "UART2_RX")):
    err.check_raise(pinmap.set_pin_function(pin, func), "set %s failed" % pin)

print([pinmap.get_pin_function(p) for p in ("A30", "A31", "B0", "B1")])   # 板端可读真值
print(uart.list_devices())

yaw_link = uart.UART("/dev/ttyS1", 115200)
pit_link = uart.UART("/dev/ttyS2", 115200)
```

要点：**先 pinmap 再构造 `uart.UART`**；一个引脚同一时刻只能有一个功能；`set_pin_function`
失败返回 `err.Err`，用 `err.check_raise` 立刻暴露。建议程序退出时把引脚复位为 GPIO（`finally`，非必需）。

### 3.5 MaixPy UART API 与收发语义（决定帧循环怎么写）

```python
u.write(bytes)              # 发送字节流，返回长度，<0 = -err.Err
u.write_str("abc")
u.available(timeout=0)      # 非阻塞：有数据返回字节数
u.read(len=-1, timeout=0)   # 非阻塞：读走缓冲区现有全部数据，无数据返回 b''
u.set_received_callback(cb) # 收到数据回调；**与 read 不能混用同一串口**
```

| 语义 | 必须记住 |
|---|---|
| `read()` 默认 = `read(-1, 0)` | **非阻塞**，主循环里安全 |
| `read(len, timeout>0)` | **阻塞**到收满或超时 → **绝不能放进 30fps 主循环** |
| 回调 | 在**另一线程**执行：只入队，不解析、不落盘 |
| 两条串口 | 可同时 open；**各自算自己的超时**，一条断了不能让另一条停 |

### 3.6 上板自检三连（P0）

```bash
python -c "from maix import uart; print(uart.list_devices())"
python -c "from maix import pinmap; print([pinmap.get_pin_function(p) for p in ('A30','A31','B0','B1')])"
# 回环：把 UART2 的 B0 与 B1 用杜邦线短接，发使能帧看能否收到自己
python -c "from maix import uart,time; u=uart.UART('/dev/ttyS2',115200); u.write(b'\x01\xF3\xAB\x01\x00\x6B'); time.sleep_ms(50); print(u.read())"
```

---

## 4. 被控对象：ZDT 闭环步进 + 齿轮云台

### 4.1 机构事实（来自 2026-10-06 实拍四张照片）

| 观察 | 推论 | 对控制的影响 |
|---|---|---|
| 两个黑色闭环步进电机 + 白色 3D 打印齿轮减速（小齿轮驱动大齿轮） | 存在**减速比 i = 大齿轮齿数 / 小齿轮齿数** | 角度↔脉冲换算必须含 i；**但推荐直接实测"每像素脉冲"绕开它**（§7.3） |
| MaixCAM2 板与激光笔装在**同一块俯仰支架**上（图 2/3/4），随 pitch 一起转 | 激光与相机**刚性同转** → 光斑在图像中的位置几乎不随云台运动而变 | ★ **闭环退化成"定点跟踪"**：云台一动，动的是靶心，光斑几乎不动（§7.1） |
| 支架是 3D 打印件、螺栓固定 | 相机可能有几度**横滚误差** → 图 x/y 轴与云台轴不严格对齐 | 需要 §7.3 的**交叉项检查**（只转 yaw 时看 dy 是否跟着变） |
| 齿轮为 3D 打印件 | 有**齿隙回差** | 死区必须 ≥ 回差对应像素（§7.5） |
| 两个四线接插件（红/蓝/绿/黑） | 电机相线 / 编码器线 | 别与串口线混；串口在驱动器自己的 4P 座上 |
| 俯仰轴有轴承载荷且受重力 | 静止时可能自溜、掉电会低头 | 闭环能补；掉电后必须重新对准（本方案每次开机都重锁，无影响） |

> ⚠ 照片**不能**用来断定激光笔光轴与相机光轴的夹角。**P2 开环试验必须实测确认**：
> 只转 yaw，看画面里移动的是"靶心"还是"光斑"，还是两者都动。若光斑也在动，说明激光与相机
> 不是刚性同转（例如激光笔实际固定在底座侧）；这时闭环依然成立（我们闭合的就是 `dx/dy`），
> 但 §7.3 的标定要按实测位移重新做。

### 4.2 齿轮回差的两个换算事实

1. **减速比越大，负载侧回差越小**：负载回差 ≈ 电机轴回差 / i。所以照片里的齿轮减速对精度**有利**。
2. **回差在图像上的等价值** = 云台回差(°) × k(≈9.7px/deg)。例：云台回差 0.5° ≈ 4.9px；1° ≈ 9.7px。
   → **回差 1° 就吃掉整个 7.4px 误差预算**，所以 §7.5 必须实测并据此定死区。

### 4.3 驱动器上电前必须用 OLED 菜单/上位机设好的项

| 菜单项 | 设成 | 为什么 |
|---|---|---|
| `P_Serial` | **`UART_FUN`** | 把通信口复用为串口（默认可能关闭或为 CAN） |
| `UartBaud` | **`115200`** | 与 MaixCAM2 一致 |
| `ID_Addr` | 1 号机 = **1**、2 号机 = **2** | 返回帧带地址，便于区分；将来可改共总线 |
| `Checksum` | **`0x6B`（固定校验）** | 本文所有帧尾都按 `0x6B` 写；改成 XOR/CRC-8 则帧尾算法要换 |
| `MStep` | 例如 **16**（→ 3200 脉冲/圈） | 决定"每圈脉冲数"，参与角度换算 |
| `MotType` | **1.8°**（42 电机） | 决定整步是 200 还是 400 |
| `Response` | 调试期 **`Receive`** / 量产 **`None`** | `Receive` 能收到确认帧与 `E2` 错误，可发现"没使能/堵转" |
| **`Cal`（编码器校准）** | **必须做**（上电提示 `Not Cal` 时） | 不做闭环精度无从谈起；**必须在空载/轻载下做** |
| `P_Pul` | 完全用串口控制时可设 `PUL_OFF` | 避免脉冲口干扰（可选） |

> **上电自动使能**：若菜单里有该选项建议开启，但**不要依赖它** —— 程序设计成"启动后主动发一次
> `0xF3` 使能"（多发了也不会错），这样无论菜单怎么设都安全。

---

## 5. 视觉契约（GPT 已实现，控制器只读这些字段）

`vision/aiming.py`：

| 函数 | 作用 |
|---|---|
| `LaserSpotDetector.process(gray)` | 在参考点附近 **±`LASER_SEARCH_RADIUS`(默认 32)px 的 ROI** 内找"小而孤立的**亮斑**"，返回 `valid/u/v/contrast/stage/score/area` |
| `add_aiming(result, shape, cfg, spot)` | 原地补充 `laser_u/v`、`laser_valid`、`laser_source`、`aim_valid`、**`dx`、`dy`**、`distance_px` |
| `pack_aiming(result, protocol)` | ASCII：`@A,valid,tx,ty,lx,ly,dx,dy,calibrated,laser_valid,target_valid%\r\n` |
| `draw_aiming(canvas, result, shape)` | 预览叠加（只给显示用） |

控制器的**唯一输入**：`aim_valid`、`dx`、`dy`（另用 `laser_source`、`stage` 打日志）。

### 5.1 ★ 三个必须先解决的接口问题（否则闭环起不来）

| # | 问题 | 说明 | 建议 |
|---|---|---|---|
| 1 | **光斑检测假设"亮斑"** | `aiming.py:52` 用 `residual >= LASER_MIN_CONTRAST`、`:76-78` 用 `peak − ring` 判对比 → **只认亮斑**。而 405nm 蓝紫在硅基 CMOS 上量子效率低，打在**紫外感光纸**上留下的痕迹偏暗 → **可能是暗斑** | 实测**灰度极性**：把光斑打在纸上，看 `[aim]` 是否 `laser_valid=True`；若恒 `False`，给 `LaserSpotDetector` 加 `LASER_POLARITY = "dark"` 分支（把两处不等号反向）。**这是闭环能否启动的前提** |
| 2 | **ROI 只有 ±32px** | 大偏差时光斑压根不在 ROI 内 → `laser_valid=False` → `aim_valid=False` → 粗瞄阶段没有输入 | **粗瞄用 `LASER_MODE="fixed"`**（固定参考点），`|e|` 收到 <24px 后再切 `detect` 做精调（§7.6） |
| 3 | **固定参考点是截图估的初值** | `LASER_REFERENCE_SIZE=(453,333)`、`LASER_PIXEL=(209,144)`、`LASER_CALIBRATED=False`；`reference_point()` 会按比例缩放到当前图宽 → 要求截图与运行时**完全同视场** | **把 `LASER_REFERENCE_SIZE` 改成 `(640, 480)`，`LASER_PIXEL` 直接填处理图的像素坐标**，消掉一次缩放换算（少一个错误来源） |

### 5.2 ★ 一个容易被忽略的滞后面（B 轨内部，务必处理）

`dx/dy` 建立在 `result["u"]/["v"]` 上，而 B 轨的 `CENTER_ALPHA = 0.70` 一阶低通
**在 30fps 下等效滞后 ≈ 0.83 帧 ≈ 28ms**：

- 静止瞄准：无影响。
- **底盘运动时**（发挥部分）：28ms 滞后 × 90°/s ≈ 2.5° ≈ **6.6cm @1.5m**，直接超差。

→ **闭环用的靶心改用 `result["raw_u"]/["raw_v"]`**（B 轨已输出，未平滑），`CENTER_ALPHA` 只留给显示。
落地方式：给 `add_aiming()` 加一个 `use_raw` 开关，用 `raw_u/raw_v` 算 `dx/dy`。
**注意：换坐标后 `PULSE_PER_PX` 必须在同一套取值下重新标定**（见 §7.3 末尾）。

---

## 6. ZDT（Emm_V5.0）串口协议

来源：官方《Emm_V5.0 步进闭环驱动说明书 Rev1.3》示例 + 多个开源驱动实现交叉核对。

### 6.1 帧结构与校验

```
[地址 1B] [功能码 1B] [指令数据 N B] [校验字节 1B]
```

- **地址**：1~255；`0` = 广播
- **校验字节**：`Checksum` 菜单设为 `0x6B` 时，**就是固定值 `0x6B`**（不是算出来的）。
  另有 XOR / CRC-8 / Modbus-RTU 可选 —— 一旦改动，本文所有帧尾都要重算，**统一用 `0x6B`**。
- **多字节参数：大端（高位在前）** ★ 与常见小端习惯相反，最容易写错
- **返回帧**：`[地址] [功能码] [命令状态] [校验]`
  - `02` = 成功；`E2` = 条件不满足（**电机没使能 / 触发堵转保护**）；`00 EE` = 错误命令
  - 位置模式**到位帧**：`01 FD 9F 校验`（需 `Response` = `Reached` 或 `Both`）

### 6.2 控制类命令（本工程要用的全部）

| 码 | 命令 | 帧格式（大端） | 长度 |
|---|---|---|---|
| `0xF3` | **电机使能** | `addr F3 AB en sync CR`，`en`：1=使能 0=失能 | 6 |
| `0xF6` | **速度模式** | `addr F6 dir spd(2) acc sync CR` | 8 |
| `0xFD` | **位置模式** | `addr FD dir spd(2) acc pulses(4) mode sync CR`，`mode`：0=相对 1=绝对 | 13 |
| `0xFE` | **立即停止** | `addr FE 98 sync CR` | 5 |
| `0xFF` | **多机同步运动** | `addr FF 66 CR` | 4 |

参数语义：

| 参数 | 取值 | 说明 |
|---|---|---|
| `dir` | `0x00` = **CW 顺时针**，`0x01` = **CCW 逆时针** | 口径 = 面朝电机轴 |
| `spd` | 0~5000，单位 **RPM** | 大端 2 字节；`0x05DC` = 1500 |
| `acc` | 0~255 | ★ **`0` = 不使用曲线加减速，直接按设定速度运行**；非 0 时**数值越大加减速越快**（每 +1RPM 的间隔 = `(256−acc)×50µs`）。**增量闭环一律 `acc=0`**，否则加减速会拖到秒级 |
| `pulses` | 0~2³²−1 | 大端 4 字节；**16 细分下 3200 脉冲 = 电机转 1 圈** |
| `mode` | `0x00` = 相对位置，`0x01` = 绝对位置 | 本工程**用相对模式**（理由见 §7.4） |
| `sync` | `0x00` = 不启用，`0x01` = 启用多机同步 | 见 §7.8 |

官方示例（可直接当 P1 的测试向量）：

```
使能      01 F3 AB 01 00 6B            正确返回 01 F3 02 6B
速度模式  01 F6 01 05 DC 0A 00 6B      CCW, 1500RPM, acc=10, 不同步     → 01 F6 02 6B
位置模式  01 FD 01 05 DC 00 00 00 7D 00 00 00 6B
                                       CCW, 1500RPM, acc=0, 32000 脉冲(=10 圈), 相对, 不同步
立即停止  01 FE 98 00 6B
多机同步  01 FF 66 6B
条件不满足返回  01 F6 E2 6B            错误命令返回  01 00 EE 6B
```

### 6.3 查询类命令（二期用，帧 = `addr func 6B`，3 字节）

| 码 | 名称 | 用途 |
|---|---|---|
| `0x36` | `S_CPOS` **实时位置** | 用真实角度校正"积分累加"、发现丢帧/卡死 |
| `0x35` | `S_VEL` 实时转速 | 判断是否真的在动 |
| `0x33` | `S_TPOS` 目标位置 | — |
| `0x3A` | `S_FLAG` 电机状态标志位 | **堵转 / 到位 / 使能**状态 |
| `0x3B` | `S_ORG` 归零状态 | — |
| `0x31` | `S_ENCL` 编码器值（校准后） | — |
| `0x24` | `S_VBUS` 总线电压 | 供电诊断 |
| `0x1F` | `S_VER` 固件/硬件版本 | 上板识别 |
| `0x42` / `0x43` | 驱动参数 / 系统状态参数 | 需再加辅助码 `0x6C` / `0x7A` |

> 查询命令的**返回帧长度与字段排布以手册为准**（不同固件版本有差异）：上板先抓一次原始字节再写解析。
> **本工程不依赖角度回读**（闭环在图像上），所以这些属于**增强项**，不要卡在这一步。

### 6.4 Response 模式（决定要不要读串口）

| Response | 控制命令 | 到位 | 查询 |
|---|---|---|---|
| `None` | 不返回 | 不返回 | **始终返回** |
| `Receive`（默认） | 返回确认帧 | 不返回 | 始终返回 |
| `Reached` | 不返回 | 返回 `..FD 9F CR` | 始终返回 |
| `Both` | 返回确认 | 返回到位 | 始终返回 |

**建议**：调通期用 `Receive`（能发现 `E2` = 没使能/堵转）；量产切 `None` 省 CPU。
**无论哪种，读串口一律用 `available()` + `read()` 非阻塞**，并计入 `[perf] rx=`。

### 6.5 参考实现（`vision/stepper_link.py` 可直接照抄）

```python
ADDR_YAW, ADDR_PITCH = 1, 2
CR = 0x6B


def _frame(addr, func, data):
    return bytes([addr & 0xFF, func]) + bytes(data) + bytes([CR])


def cmd_enable(addr, enable=True, sync=False):
    return _frame(addr, 0xF3, [0xAB, 0x01 if enable else 0x00, 0x01 if sync else 0x00])


def cmd_speed(addr, ccw, rpm, acc=0, sync=False):
    rpm = max(0, min(5000, int(rpm)))
    return _frame(addr, 0xF6, [0x01 if ccw else 0x00,
                               (rpm >> 8) & 0xFF, rpm & 0xFF,
                               int(acc) & 0xFF, 0x01 if sync else 0x00])


def cmd_position(addr, ccw, rpm, pulses, acc=0, absolute=False, sync=False):
    n = max(0, min(0xFFFFFFFF, int(abs(pulses))))
    return _frame(addr, 0xFD, [0x01 if ccw else 0x00,
                               (rpm >> 8) & 0xFF, rpm & 0xFF,
                               int(acc) & 0xFF,
                               (n >> 24) & 0xFF, (n >> 16) & 0xFF,
                               (n >> 8) & 0xFF, n & 0xFF,
                               0x01 if absolute else 0x00,
                               0x01 if sync else 0x00])


def cmd_stop_now(addr, sync=False):
    return _frame(addr, 0xFE, [0x98, 0x01 if sync else 0x00])


def cmd_sync_motion(addr=0):          # addr=0 广播
    return _frame(addr, 0xFF, [0x66])


def cmd_read(addr, func):             # func 见 6.3
    return _frame(addr, func, [])
```

**自测**：写完先跑一遍，把上表 5 条官方示例逐字节比对，长度必须是 6 / 8 / 13 / 5 / 4。

---

## 7. ★ 控制算法：把光斑搬进黑框中心

### 7.1 为什么这是"定点跟踪"而不是一般视觉伺服

**前提**（照片已确认）：激光笔与相机**刚性同装在俯仰平台上，一起转**。于是：

> 云台转动时，**激光与相机的相对姿态不变** → 光斑投在图像里的位置几乎不动；
> 变的是**靶心**在图像里的位置。
> 所以"把 `dx/dy` 打到 0" = 把靶心搬到光斑上，而**目标点在图像坐标里是静止的**。

这带来三个简化：

1. 不需要知道靶的绝对位置、不需要小车位置、不需要解算云台-车体几何。
2. 不需要标定"像素→毫米"，也不再需要单独标定激光-相机视差（**光斑实测值已经把视差含进去了**）。
3. 底盘运动时闭环照样成立 —— 这是发挥部分"移动中瞄准"的实现基础。

**唯一的例外**：若激光与相机并非刚性同转（例如激光笔实际固定在 yaw 底座上），光斑会随轴运动。
→ **P2 开环试验第一件事就是判定它**：只转 yaw，看画面里动的是靶心、光斑，还是两者都动。

### 7.2 ★ 符号推导（`dx/dy` → `dir` 字节）

设定：图像 `u` 向右增、`v` 向下增。云台角记 `Δ右`（yaw 向右为正）、`Δ仰`（仰角增大为正）。

| 步骤 | 推导 |
|---|---|
| ① | 相机随云台转、靶在世界上不动 → 云台**向右**转，靶在图像里**左移**：`Δu_靶 = −k·Δ右` |
| ② | 云台**抬头**（仰角增大），光轴抬高 → 靶在图像里**下移**：`Δv_靶 = +k·Δ仰` |
| ③ | 目标 `dx→0, dy→0`，即需要 `Δu_靶 = −dx`、`Δv_靶 = −dy` |
| ④ | 由 ①：`−k·Δ右 = −dx` → **`Δ右 = dx / k`**（`dx>0` ⇒ yaw 必须向右） |
| ⑤ | 由 ②：`+k·Δ仰 = −dy` → **`Δ仰 = −dy / k`**（`dy>0` ⇒ 必须低头 / 仰角减小） |
| ⑥ | 代入 §0.2：1 号机 `CW→左、CCW→右` ⇒ `dx>0` 要"向右" ⇒ **CCW(`0x01`)** |
| ⑦ | 代入 §0.2：2 号机 `CW→仰角增、CCW→俯角增` ⇒ `dy>0` 要"低头" ⇒ **CCW(`0x01`)** |

**最终实现（不需要任何隐藏负号）**：

```python
ccw_yaw   = (dx > 0)      # dx>0 → CCW
ccw_pitch = (dy > 0)      # dy>0 → CCW
```

统一写成 `ccw = (e * DIR_SIGN) > 0`，`DIR_SIGN_*` 默认 `+1`；**实测反了就把对应项置 `−1`**。
`DIR_SIGN` 存在的唯一意义：现场纠方向时不必改公式、不必动接线。

> **易错提醒**：`dx/dy` 是"靶心 − 光斑"方向，不是"光斑 − 靶心"。若 GPT 在别处看到有人用
> `spot − target` 的定义，那里的符号是反的，**不要混用**。本文一律以 §0.3 为准。

### 7.3 幅值：每像素脉冲 `PULSE_PER_PX`

两种得到方式，**强烈推荐后者**：

| 方式 | 公式 | 评价 |
|---|---|---|
| 理论换算 | `P_PER_PX = (200 × MStep × i) / 360 / k`，`k ≈ 9.7 px/度` | 要数齿数、查细分、还得知道 fx 与安装角 —— 任一错就全错 |
| **实测标定（推荐）** | 让某轴走 `N` 个脉冲，测 `dx`（或 `dy`）改变了多少像素 → `P_PER_PX = N / |Δ|` | **一次性把齿轮比、细分、焦距、安装角全部吸收掉**；且对"光斑是否随轴动"两种安装都成立 |

**标定步骤（10 分钟，用现有 `[aim]` 日志就能做）**：

```
1) 靶纸固定，切 LASER_MODE="detect"，静置，读 [aim] 记 (dx0, dy0)；
2) 只让 yaw 轴发一帧位置模式：CCW、rpm=300、pulses=2000、acc=0，等 1~2s 停稳；
3) 记 (dx1, dy1)：
       PULSE_PER_PX_YAW = 2000 / |dx1 − dx0|
       C_YAW            = (dy1 − dy0) / (dx1 − dx0)        # 交叉项，正常应 ≈ 0
4) yaw 反向走 2000 脉冲回原位（同时验证正/反程是否对称 → 顺手量回差）；
5) 只让 pitch 走 2000 脉冲，同理得 PULSE_PER_PX_PITCH 与 C_PITCH；
6) 在 2~3 个不同 yaw/pitch 位姿重复，看 P_PER_PX 变化量：
       <15% → 用固定值；>15% → 按位姿分档（少见）。
```

**交叉项怎么判**：`|C| > 0.25` 说明相机横滚没装正或两轴不垂直 → 优先**机械上把相机横滚调正**；
不方便再带 2×2 修正（`Δ轴1 = M11·ex + M12·ey`）。`|C|` 在 0.1 量级**完全可以忽略**
（对角控制器照样收敛，只是每轮留一点残差）。

**初值（占位，必须被 P2 标定替换）**：`fx ≈ 554px`（640 宽 / 60° HFOV）、`k ≈ 9.7 px/度`、
16 细分 + 减速比 4 ⇒ `P_PER_PX ≈ 3.67 脉冲/像素`。

> ⚠ **标定与运行必须用同一套坐标**：若按 §5.2 改用 `raw_u/raw_v`，`P_PER_PX` 必须在 raw 下重标；
> 若 `HERTZ` 变（相机帧率）或改了镜头/镜头校正开关，也要重标。**换坐标系不重标 = 增益错一半。**

### 7.4 ★ 控制律：相对位置模式 + 增量式比例律（主方案）

每一帧、每一轴，用**一帧位置模式（相对）**表达"这一步该走多少"：

```
e = dx（yaw 轴）或 dy（pitch 轴）                         # px
if |e| < DEADBAND_PX:                                     # 死区内
        本轴不发指令（保持），return
ccw   = (e * DIR_SIGN) > 0                                # §7.2
gain  = GAIN_COARSE if |e| > COARSE_PX else GAIN_FINE      # 粗/精两段
mag   = round(gain * |e| * PULSE_PER_PX)                  # 脉冲
mag   = clamp(mag, 1, MAX_STEP_PULSES)                    # ★ 单帧限幅
send  cmd_position(addr, ccw, RPM, mag, acc=0, absolute=False)
```

**为什么选相对位置模式而不是绝对位置模式**

| | 相对位置（推荐） | 绝对位置 |
|---|---|---|
| 零点 | **不需要** | 需要开机对零 / 断电可能丢零 |
| 丢一帧指令 | 少走一点 → **下一帧视觉自动补上** | 目标错位 → 需重新对零 |
| 角度累积 | 主机不管，视觉闭环负责 | 主机要维护角度积分 |
| 脉冲范围 | 4 字节（±21 亿），够走几万圈 | 同样 |

**为什么 `acc = 0`**：ZDT 的 `acc` 语义是"**0 = 不做曲线加减速，直接用设定速度**"；非 0 时
`每 +1RPM 的间隔 = (256−acc)×50µs` —— 取 `acc=30` 时爬到 300RPM 要 **3.4 秒**，对 30Hz 的增量闭环
是灾难。**增量闭环一律 `acc=0`。**

**收敛性（给 GPT 的定心丸）**：只要满足三条 —— ① 方向符号对；② 每帧位移不超过误差对应量
（`gain ≤ 1`）；③ 指令确实被执行 —— 误差就按几何级数衰减，**与 ZDT 内部如何累计相对位置无关**。
按 `gain = 0.6`：200px → 2px 需 `ln(0.01)/ln(0.4) ≈ 5` 帧 ≈ 0.2s。**剩下 55 帧全是余量。**

**`MAX_STEP_PULSES` 必须写**：误检可能给出 `dx = 2000px` → `mag = 2000×3.67×0.6 ≈ 4400 脉冲 ≈ 12°`
的猛甩。建议限到 **≈10° 云台角对应的脉冲数**（按上面的初值 ≈ 360 脉冲）；粗瞄也不会慢
（10°/帧 × 30fps = 300°/s 等效）。

**一个必须在 P3 实测的细节**：连续两帧各发一次相对位置命令（第 1 帧还没走完就发第 2 帧），
看**总位移是否等于两者之和**。若"新命令覆盖未完成的目标"（不累加），则精调段改用 §7.7 的速度模式。
**注意：这只会影响收敛速率，不影响收敛性**（因为每帧都重新测量 `dx/dy`）。

### 7.5 ★ 死区与齿轮回差（3D 打印件的头号问题）

**回差怎么坏事**：换向时，前 `B` 个脉冲只是在"吃掉齿隙"，负载不动 → 每次换向都少走 `B`
→ 误差在零点附近来回超调，形成极限环（表现为云台"嗒嗒"抖、光斑在靶心附近左右跳）。

**处理三步**：

1. **量回差**：同一目标，分别"从左逼近"和"从右逼近"到稳定，记录两次的 `dx` →
   `BACKLASH_PX ≈ |dx_左 − dx_右| / 2`。（图像等价：`B_px = 云台回差(°) × 9.7`）
2. **死区必须 ≥ 回差**：`DEADBAND_PX = 1.5 × BACKLASH_PX`，下限也给 1.5px。
   **不要设 0.5px 之类的极小死区** —— 那一定会在回差带里抖。
3. **算清账**：死区 ≤ 7.4px 就不影响满分（2cm@1.5m）。若 `B_px` 已超过 ~5px：
   - 机械优先：齿轮中心距调紧 / 加弹簧预紧 / 换金属齿轮；
   - 或提高减速比（**负载侧回差 = 电机侧回差 / i**）；
   - 软件只能缓解，不能消除。

**可选增强（二期，主体跑通后再做）—— 单向逼近**：最终定位一律从固定方向（例如"总是从右/下"）
进入：当误差很小且需要反向时，先向反方向多走 `1.2 × BACKLASH_PULSES` 再正向逼近，使最后一次
啮合方向恒定 → 精度由**回差的重复性**而不是回差本身决定。数控机床的常规做法，但对 30Hz 闭环
要小心，别把简单问题复杂化。

### 7.6 粗瞄与精瞄（2s / 4s 时限的关键）

| 阶段 | 输入 | 触发 | `gain` | 说明 |
|---|---|---|---|---|
| **COARSE 粗瞄** | `LASER_MODE="fixed"` 的固定参考点算 `dx/dy`（**不依赖光斑检出**） | 初始 / 复位 | **0.8 ~ 1.0** | 一步跳到位；解决"光斑不在 ±32px ROI 内 → 无输入"的死结 |
| **FINE 精瞄** | `LASER_MODE="detect"` 的实测光斑 | `|e| < COARSE_PX`（建议 24px，给 `LASER_SEARCH_RADIUS=32` 留余量） | **0.4 ~ 0.6** | 小步逼近，抗抖 |
| **SETTLE** | — | `|e| < DEADBAND` 连续 5 帧 | 0（不发指令） | 保持，允许发射激光 |

> 粗瞄用固定参考点的**前提**是 §7.1 成立（激光与相机刚性同转）——此时"靶心落在固定参考点"
> 就等价于"光斑落在靶心"，残差量级就是同平台刚连的剩余视差（很小）。
> **若 P2 判定"光斑会随轴动"，粗瞄必须改用 `detect` 并把 `LASER_SEARCH_RADIUS` 放大到能覆盖初始偏差**
> （代价：ROI 变大、误检概率上升）。

### 7.7 备选：速度模式（`0xF6`）闭环

`rpm = clamp(KV × |e|, 0, RPM_MAX)`，`ccw = (e·DIR_SIGN) > 0`，`|e| < DEADBAND` 时发 `rpm = 0`。

| | 速度模式 | 相对位置模式（主方案） |
|---|---|---|
| 静态误差 | 天然为 0（e=0 → 速度 0） | 天然为 0（e=0 → 不发指令） |
| 大误差 | 自动快 | 自动大位移（受限幅） |
| 需要标定 | **速度增益**（rpm/px） | 每像素脉冲 |
| 静止 | 要持续"发 0"或发 `0xFE` | 天然静止 |
| 风险 | 低速爬行/共振；`rpm` 取整后可能不动 | 频繁小位移，可能"走走停停" |

**用它的时机**：P3 实测发现"连续相对位置命令不累加"（即新命令覆盖未完成的目标）时，精调段切速度模式；
否则**保持相对位置模式**（更省心）。

### 7.8 两轴调度与是否用多机同步

- 两条串口各自独立：每帧 yaw 一帧（13B）+ pitch 一帧（13B）= 26B @115200 ≈ **2.3ms**。
  两帧之间只差 ~1.1ms → **默认不需要多机同步**。
- 什么时候才需要 `0xFF 0x66 6B`：**大角度粗瞄**时两轴到位时间差会让光斑划弧（可能扫过靶外）。
  若实测发现是问题：把两轴命令的 `sync=1`（命令被缓存不执行），再分别/广播发 `0xFF 66 6B` 触发同时启动。
  **先不做，P5 测出来再说。**
- 频率：按"检测帧"发（30fps），不要另开定时器；若串口拥塞或报错，用 `TX_RATE_HZ` 限流。

### 7.9 状态机

```
BOOT    上电 → 等 300~500ms → 两轴各发一次 0xF3 使能 → 进 COARSE
        （Response=Receive 时读确认帧；收不到只记警告，不阻塞开机）
  │
COARSE  用固定参考点算 (dx,dy) → gain=0.9，大步长
  │    |e| < COARSE_PX 且 laser_valid
  ▼
FINE    用实测光斑算 (dx,dy) → gain=0.5 + 死区(≥1.5×回差) + 限幅
  │    |e| < DEADBAND 连续 5 帧
  ▼
SETTLE  停发指令；可发激光；每帧仍检查误差
  │    |e| > 2×DEADBAND 连续 2 帧 → 回 FINE
  │
HOLD    aim_valid=False（光斑或靶心缺失）→ **不发指令**，保持当前位置
  │    连续 LOST_FRAMES（15，约 0.5s）
  ▼
SCAN    ±SCAN_DEG 内单向慢扫（位置模式小步、acc=0、低 rpm），直到重新 aim_valid → 回 COARSE
  │
FAULT   连续 N 帧收不到确认帧 / 反复收到 E2 / 串口异常
        → 发 0xFE 立即停止 + 关激光（若有）+ 停止发送 + 日志告警
```

**异常处理表**：

| 现象 | 判据 | 动作 |
|---|---|---|
| 光斑丢失 | `laser_valid=False` | HOLD；连续 15 帧 → SCAN |
| 靶心丢失 | `result["valid"]=False` | 同上（B 轨已保证不输出旧中心） |
| 收到 `E2` | 返回帧第 3 字节 `0xE2` | ①补发 `0xF3` 使能；②仍 E2 → 查堵转保护（手册有清除命令）/ 查机械卡死 |
| `Response=Receive` 时超时 | 200ms 无确认帧 | 计数；连续 20 次 → FAULT |
| 误差突然巨大 | `|e| > JUMP_REJECT_PX`（如 400px） | **本帧不发指令**（当离群点丢弃）；连续 3 帧才采信 |
| 需要限位 | 累积角度超 `LIMIT_*` | 该方向不再发指令（相对模式没有硬限位，必须主机管） |
| 整帧 `aim_valid=False` 且持续 | — | 绝不用"上一次的 `dx/dy`"续发（那等于按旧坐标继续推进） |

### 7.10 参数初值与整定顺序

| 参数 | 初值 | 整定阶段 |
|---|---|---|
| `DIR_SIGN_YAW/PITCH` | `+1` | **P2 先定**（方向不对一切白搭） |
| `PULSE_PER_PX_YAW/PITCH` | `3.67`（占位） | **P2 标定**（§7.3） |
| `RPM_FINE` | `300`（电机轴） | P3：太小会"发不出步"、太大会抖 |
| `RPM_COARSE` | `1200` | P3 |
| `GAIN_FINE` | `0.5` | P3：从 0.3 起往上加 |
| `GAIN_COARSE` | `0.9` | P3 |
| `COARSE_PX` | `24` | P4 |
| `DEADBAND_PX` | `2.0`（先）→ 实测回差后改 `1.5×BACKLASH_PX` | **P4 必做** |
| `MAX_STEP_PULSES` | `360`（≈10°） | P4 |
| `JUMP_REJECT_PX` | `400` | P4 |
| `LOST_FRAMES` / `SCAN_DEG` | `15` / `15` | P6 |

### 7.11 时间与带宽预算

| 环节 | 耗时 | 备注 |
|---|---|---|
| 视觉（检测 + 光斑） | 见 `[perf] detect`（稳态 ~1ms 量级） | 光斑只在 64×64 ROI 内，很轻 |
| 控制律 | <0.1ms | 几次乘加 |
| 两轴发送 26B | **2.3ms** | 115200；占带宽 6.8% |
| 下行读取（非阻塞） | <0.1ms | `available()` + `read()` |
| **新增合计** | **≈2.4ms/帧** | 对 30fps（33ms）无影响 |

**必须监控**：B 轨 `[perf]` 行已有 `uart=` 字段，把新增的 `rx=`（下行解析）与 `aim=`（控制律）
也统计进去。`uart` 突增 = 写阻塞或链路重传，要立刻查。

### 7.12 安全（写在最前面）

1. **激光安全**：405nm ≤10mW 直射眼睛有永久损伤风险。调试期激光只在确认指向靶面时开启；
   任何 `FAULT` **立即关激光**；上电默认激光关。
2. **上电默认不动**：BOOT 只发使能，不发位置命令，等第一帧合法 `dx/dy` 再动（避免上电满舵）。
3. **失联即停**：链路超时 → `0xFE` 立即停止，而不是继续执行最后一条。
4. **软限位**：`LIMIT_PAN_*` / `LIMIT_TILT_*` 必须由主机维护（相对位置模式下驱动器不知道绝对角）。
5. **断电顺序**：先断电机电源，再拔信号线；调试时把云台垫稳，防止掉电低头砸到镜头。

---

## 8. 代码落地清单（给 GPT）

### 8.1 新增文件与建议接口

**`vision/stepper_link.py`**（协议 + 双串口；**禁止顶层 `import maix`**，板端才导入）

```python
class StepperLink:
    def __init__(self, cfg): ...          # 内部做 pinmap + 建两条 uart；失败只降级该条
    def enable(self, axis=None): ...      # 发 0xF3；axis None = 两轴都发
    def move(self, axis, ccw, pulses, rpm, acc=0): ...   # 发 0xFD 相对位置
    def speed(self, axis, ccw, rpm, acc=0): ...          # 发 0xF6（备选方案用）
    def stop(self, axis=None): ...        # 发 0xFE
    def poll(self): ...                   # 非阻塞读两条串口 → 解析返回帧 → 计数
    def stats(self): ...                  # 返回 tx/rx/err/timeout 计数，给 [perf] 用
```

**`vision/gimbal_aim.py`**（控制律；**保持纯 Python、不含 maix**，便于离线仿真）

```python
STATE = ("BOOT", "COARSE", "FINE", "SETTLE", "HOLD", "SCAN", "FAULT")

class GimbalAim:
    def __init__(self, cfg): ...
    def update(self, result):             # 输入：result（含 aim_valid/dx/dy/laser_valid）
                                          # 输出：dict(state=..., cmds={"yaw": (ccw,pulses) or None, "pitch": ...})
    def status_line(self): ...            # 给日志：[aim] state=FINE dx=+3.2 dy=-1.1 yaw=CCW/12p pit=CW/4p
```

**`tools/test_gimbal_aim.py`**（离线仿真，纯 PC）
用一个假电机模型：`角位移 = 脉冲 × pulse_per_px_truth`（含可选回差与噪声），
把 `dx/dy` 的真值按几何反推每帧的测量值，验证：收敛帧数、是否振荡、失锁时是否保持、
限幅是否生效、`DIR_SIGN` 反了会不会发散（**必须把这个失败模式也写进测试**，它是最危险的坑）。

### 8.2 `config.py` 新增参数（建议默认值）

```python
# ---- 链路（替换现有的 ENABLE_UART/UART_PORT/UART_BAUD 单串口写法）----
ENABLE_GIMBAL  = True            # False = 退回纯视觉（保留 pack_aiming 的 ASCII 输出）
TX_RATE_HZ     = 30
U1_DEV, U1_BAUD = "/dev/ttyS1", 115200      # 1 号机 = yaw
U2_DEV, U2_BAUD = "/dev/ttyS2", 115200      # 2 号机 = pitch
U1_PINS = {"A30": "UART1_TX", "A31": "UART1_RX"}
U2_PINS = {"B0": "UART2_TX", "B1": "UART2_RX"}
ADDR_YAW, ADDR_PITCH = 1, 2
RX_TIMEOUT_MS  = 200             # Receive 模式下多久没确认帧算一次超时

# ---- 轴向与换算（P2 标定后替换）----
DIR_SIGN_YAW    = 1              # +1 → dx>0 发 CCW（§7.2 推导）；实测反了置 -1
DIR_SIGN_PITCH  = 1              # +1 → dy>0 发 CCW
PULSE_PER_PX_YAW   = 3.67        # 占位，必须实测
PULSE_PER_PX_PITCH = 3.67        # 占位，必须实测
BACKLASH_PX        = 0.0         # P4 实测（§7.5）

# ---- 控制律 ----
AIM_USE_RAW      = True          # 用 raw_u/raw_v 算 dx/dy（§5.2）
GAIN_COARSE      = 0.9
GAIN_FINE        = 0.5
COARSE_PX        = 24.0
DEADBAND_PX      = 2.0           # P4 后改为 1.5 × BACKLASH_PX
MAX_STEP_PULSES  = 360           # ≈10° 云台角
JUMP_REJECT_PX   = 400.0
RPM_COARSE       = 1200
RPM_FINE         = 300
ACC_DEFAULT      = 0             # ★ ZDT 语义：0 = 不做曲线加减速

# ---- 状态机 ----
LOST_FRAMES      = 15
SCAN_DEG         = 15.0
SCAN_RPM         = 60
LIMIT_PAN_DEG    = (-60.0, 60.0)
LIMIT_TILT_DEG   = (-10.0, 45.0)
```

### 8.3 `main.py` 的接线位置

| 位置 | 现状 | 改成 |
|---|---|---|
| `main.py:45-48` `_send_uart()` | 单串口发 `pack_aiming` ASCII | 保留（作为 `ENABLE_GIMBAL=False` 的降级路径）；新增 `link.move(...)` 调用 |
| `main.py:64-70` uart 初始化 | 单口、`UART_PORT=/dev/ttyS0` | 先 pinmap，再按 §8.2 建两条；任一失败只降级该轴并打日志 |
| `main.py:104-108` 主循环 | `detector.process` → `laser_detector.process` → `add_aiming` → `_send_uart` | 末尾追加 `cmds = aim.update(result)` → `link.move(...)` → `link.poll()` |
| `main.py:123-126` `[perf]` 相位 | `read/lens/bridge/detect/uart/show/loop` | 追加 `rx` / `aim` 两相位 |
| `main.py:137-146` `[aim]` 日志 | 只打 `dx/dy` | 追加状态机字段：`state / yaw=CCW/12p / pit=CW/4p` |
| `main.py:26` `BUILD_ID` | `2026-10-06-laser-spot-v1` | 更新为新构建号（**板端日志必须逐字一致**才算新代码） |

### 8.4 与现有门禁 / 打包的关系

```bash
# B 轨门禁（新增文件后必须跑）
"D:/Python311/python.exe" -B tools/check_app.py --manifest    # 静态机检 + 打印包内清单
"D:/Python311/python.exe" -B tools/test_performance.py        # 视觉回归（29/30 检出、0.803px、1.012ms）
"D:/Python311/python.exe" -B tools/test_aiming.py             # 光斑/偏差回归（GPT 已建）
"D:/Python311/python.exe" -B tools/test_gimbal_aim.py         # 新增：闭环离线仿真
```

1. 新增两个 `.py` 后 **B 轨应用包 = 8 个文件**（`app.yaml` / `main.py` / `config.py` /
   `vision/`×3 / `assets/icon.png`）——**打完必须核清单**（B 轨曾因漏排除 `screen_check.png` 踩坑）。
   本文档是 `.md`，被 `*.md` 规则排除，不会进包。
2. **新模块禁止顶层 `import maix`**（否则 PC 离线测试挂掉、`check_app.py` 的 C5 判 FAIL）：
   `stepper_link.py` 里把 `from maix import uart, pinmap` 放进方法内；`gimbal_aim.py` 完全不碰 maix。
3. 文件必须 **UTF-8 无 BOM + 纯 LF**；源码里不得出现绝对 Windows 路径。
4. **不要动**视觉的 `MIN_*` / `MAX_*` / `SCORE_*` / `SEARCH_WIDTH` / `TRACK_RADIUS` —— 本任务是
   加一层控制，视觉算法一行都不用改。

### 8.5 分阶段落地顺序

| 阶段 | 内容 | 验收 |
|---|---|---|
| **P0** | §3.6 自检 + 回环 | `list_devices` 有 ttyS1/ttyS2；回环能收到自己发的字节 |
| **P1** | `stepper_link.py` 帧构造自测 + 发官方 5 条示例 | 长度 6/8/13/5/4 全对；电机有响应（听声/看轴） |
| **P2** | **开环标定**：方向符号 + `PULSE_PER_PX` + 交叉项 + 回差 | §7.3 的表填出来；`DIR_SIGN` 确定 |
| **P3** | 闭环只 P（`DEADBAND=0` 先不提），`gain=0.3` 起步 | 静止靶能收进 5px 且不振荡；离线仿真通过 |
| **P4** | 加死区（依回差）、限幅、离群拒收 | 无极限环；人为给 2000px 离群不甩头 |
| **P5** | 加粗/精两段 + 状态机 + 看门狗 | 遮镜头 → HOLD 不甩头；2s 内重锁 |
| **P6** | 视差/距离无关性复核 + 多距离验证 | 0.8 / 1.5 / 2.0m 三点光斑落地误差 ≤1cm |
| **P7** | 移动中瞄准（发挥部分，需 3507 经 SPI 给角速度做前馈） | 底盘 1 圈 20s 时 D1 ≤2cm |

> **P7 依赖 SPI 链路**：底盘转动时，仅靠视觉反馈的滞后约 `ω×L`（90°/s × 33ms ≈ 3° ≈ 7.9cm @1.5m）
> → **必须前馈**。这就是"SPI 接 3507"的价值所在：从底盘拿 yaw 角速度，直接叠到云台指令上。
> 本期不实现，但**接口要留**（`gimbal_aim.update(result, feed_forward=None)`）。

---

## 9. 上板调试路线

| # | 动作 | 观测 / 判据 | 常见失败原因 |
|---|---|---|---|
| 1 | 只接 GND+TX，看 MaixCAM2 发的使能帧 | 逻辑分析仪/串口助手见 `01 F3 AB 01 00 6B` | pinmap 没设 → 无波形；TX/RX 没交叉 |
| 2 | 回环（TX-RX 短接） | 发什么收什么 | 波特率不匹配；用了阻塞 `read` |
| 3 | **只发一帧固定方向的位置命令** | 看轴往哪转、云台往哪动 | 符号反 → 闭环正反馈（最危险） |
| 4 | 只转 yaw，看画面里**动的是靶心还是光斑** | 判定 §7.1 前提 | 若光斑也动 → 粗瞄方案要改 |
| 5 | 走 2000 脉冲量 `Δdx` | 得到 `PULSE_PER_PX` 与交叉项 | 忘了先切 `LASER_MODE` |
| 6 | 正反各走一次逼近同一目标 | `BACKLASH_PX` | 回差太大 → 需机械处理 |
| 7 | 闭环只 P，`gain=0.3` 起 | 阶跃无超调、无振荡 | 增益过大 → 抖/啸叫；用了平滑值而非 raw |
| 8 | 加死区 + 限幅 | 静止时纹丝不动 | 死区 < 回差 → 极限环 |
| 9 | 遮挡 / 拔线 / 断电 异常测试 | HOLD、不甩头、不丢锁 | 缺看门狗 |

**板端必看日志**：

```text
[rect-center] <BUILD_ID> ...                     # 必须与本地 main.py 逐字一致
[perf] <BUILD_ID> valid=True FPS=.. stage=track ... uart=.. rx=.. aim=.. ms
[aim]  L=(x,y) T=(x,y) dx=+3.2 dy=-1.1 state=FINE yaw=CCW/12p pit=CW/4p
```

---

## 10. 风险与坑清单

### 10.1 MaixCAM2 侧

| # | 坑 | 规避 |
|---|---|---|
| 1 | **UART1 引脚未 pinmap** → `/dev/ttyS1` 打不开或收不到 | §3.4：先 `set_pin_function` 再构造 `uart.UART` |
| 2 | 误用 **UART0**（系统日志占用；TX 被拉低还会**无法开机**） | 只用 ttyS1 / ttyS2（▶ 当前 `UART_PORT` 就是错的，必须改） |
| 3 | 阻塞式 `read(len, timeout>0)` 进主循环 → 帧率崩 | 用 `available()` + `read()` 非阻塞；或回调（二者不可混用同一串口） |
| 4 | 在 `set_received_callback` 里做重活 | 回调只入队，解析在主循环 |
| 5 | 顶层 `import maix` 进新模块 → PC 测试挂 + `check_app.py` C5 FAIL | 函数内 import；`gimbal_aim.py` 不碰 maix |
| 6 | 用 `CENTER_ALPHA` 平滑值算 `dx/dy` → 附加 ~28ms 滞后 | `AIM_USE_RAW = True`（§5.2） |
| 7 | 丢亚像素精度（只用整数像素） | 闭环内部用 float `dx/dy`，只在发 ASCII 时才取整 |
| 8 | `lens_corr` 开关/镜头变化后没重标 `PULSE_PER_PX` | 标定与运行必须同设置（§7.3 末） |
| 9 | 新增 `.py` 后忘核包内清单 | `check_app.py --manifest`，目标 8 文件 |

### 10.2 电气 / 机械侧

| # | 坑 | 规避 |
|---|---|---|
| 10 | 电机与 MaixCAM2 共用电源 → 电机启动瞬间 MaixCAM 复位 | 独立供电 + 大电容 + 共地 |
| 11 | 不共地 → 乱码 | 三路 GND 全连一起 |
| 12 | 驱动器 `UartBaud` ≠ 115200 / `P_Serial` 没开成 `UART_FUN` | 上电先用 OLED 菜单核对（§4.3） |
| 13 | `Checksum` 被改成 XOR/CRC-8 → 所有帧被拒 | 统一用 `0x6B`（§6.1） |
| 14 | 大端写成了小端 | 用官方示例逐字节自测（§6.5） |
| 15 | **编码器没校准（`Not Cal`）** | 空载执行 `Cal` |
| 16 | 3D 打印齿轮回差 | `DEADBAND ≥ 1.5×BACKLASH_PX`；必要时调中心距/预紧 |
| 17 | 云台线缆缠绕 / 撞限位 | 软限位 + 上电先不动（BOOT 只使能） |

### 10.3 控制侧（最危险的两个）

| # | 坑 | 规避 |
|---|---|---|
| 18 | **符号搞反 → 正反馈甩头** | P2 必须先做开环方向标定；首次闭环用 `gain=0.3` 试；`test_gimbal_aim.py` 里专门加"符号反了会发散"的用例 |
| 19 | 用"更重的低通"去滤抖动 | 抖动引起的图像位移是**必须跟踪的真实运动**（A/B 轨都已实测证伪"用低通滤掉"的做法）；要减小抖动只能缩短曝光/加机械阻尼 |

---

## 11. 验收标准（DoD）

**文档阶段（本轮）**
- [x] MaixCAM2 外设事实（UART×2、SPI、pinmap、电平、波特率）成表且注明来源
- [x] ZDT 协议钉到字段级（帧格式、大端、`0x6B`、`acc=0` 语义、返回码）+ 参考实现
- [x] `dx/dy → (方向, 脉冲)` 的符号推导与结论（§7.2）
- [x] 控制律、死区/回差处理、粗精两段、状态机、参数表、调试阶梯
- [x] 给 GPT 的文件/接口/config 清单与分阶段顺序

**代码阶段（下一轮）**
- [ ] pinmap 正确设置；`uart.list_devices()` / `get_pin_function` 实测通过
- [ ] `stepper_link.py` 5 条官方示例逐字节自测通过；两条串口可同时收发
- [ ] 两轴 `DIR_SIGN` 实测确认；`PULSE_PER_PX` 标定完成并写入 config
- [ ] `test_gimbal_aim.py`：阶跃 200px→2px ≤8 帧、无超调、失锁不甩头、**符号反了能报错**
- [ ] 在线：静止靶 2s 内锁定；死区内不动；人为离群不甩头
- [ ] 0.8 / 1.5 / 2.0m 三点光斑落地误差 ≤1cm
- [ ] 遮镜头 → HOLD；恢复 → 2s 内重锁
- [ ] `check_app.py --manifest` 通过、包内 8 文件；`test_performance.py` / `test_aiming.py` 无回归
- [ ] `BUILD_ID` 更新，板端日志逐字一致

---

## 12. 待确认事项

| # | 问题 | 影响 | 我的默认假设 |
|---|---|---|---|
| **Q1** | **光斑在灰度图里是亮斑还是暗斑？**（405nm 打在紫外感光纸上） | 决定 `LaserSpotDetector` 能不能用、要不要加 `LASER_POLARITY="dark"` | 先按亮斑试；不行就加暗斑分支 |
| **Q2** | 两台 ZDT 驱动器的**菜单配置现状**？（`P_Serial` / `UartBaud` / `ID_Addr` / `Checksum` / `MStep` / `Response` / `Cal`） | 直接决定 P0/P1 能否通过 | 按 §4.3 全设成推荐值 |
| **Q3** | 减速比 **i = 大齿轮齿数 / 小齿轮齿数**？ | 只影响理论换算；实测法可绕开 | 实测 `PULSE_PER_PX`，不数齿数 |
| **Q4** | 电机轴径/联轴器是否有弹性或间隙？pitch 轴掉电会不会低头砸到镜头？ | 机械与安全 | 假设有轴承载荷、掉电会低头 → 上电不动作 + 垫稳 |
| **Q5** | 与 3507 的 SPI 协议是否已有定义？底盘能否提供 yaw 角速度/里程？ | 发挥部分"移动中瞄准"的前馈 | 本轮不实现，只留 `feed_forward` 接口 |
| **Q6** | 激光开关是否要 MaixCAM2 控制（GPIO）？ | 协议/安全 | 先按常亮；若要控，用空闲 GPIO + MOS，不直挂 |
| **Q7** | 是否允许改 B 轨 `main.py` / `config.py`（会动 `BUILD_ID`、包内清单 6→8）？ | 交付形态 | 允许，且跑完整门禁 |
| **Q8** | 与 GPT 轨的改动如何编排（谁先写 `stepper_link.py`）？ | 协作 | 我只出文档，实现由 GPT 做；需要我写则明确说 |

---

## 13. 参考资料

| 内容 | 来源 |
|---|---|
| MaixPy UART（引脚表、接线、FAQ） | https://wiki.sipeed.com/maixpy/doc/zh/peripheral/uart.html |
| MaixPy UART API（`read`/`write`/`available`/回调） | https://wiki.sipeed.com/maixpy/api/maix/peripheral/uart.html |
| MaixPy PINMAP（复用、`set_pin_function`、默认功能与注意引脚） | https://wiki.sipeed.com/maixpy/doc/zh/peripheral/pinmap.html |
| MaixCAM2 板卡引脚图 v1.0（丝印与复用） | https://wiki.sipeed.com/hardware/assets/maixcam/maixcam2_pins.jpg |
| MaixCAM2 硬件规格（IO 3.3V、20 IO、6pin 扩展、供电） | https://wiki.sipeed.com/hardware/zh/maixcam/maixcam2.html |
| MaixCDK `maix_pinmap.cpp`（MaixCAM2 分册，引脚→功能） | https://github.com/sipeed/MaixCDK |
| **ZDT Emm_V5.0 步进闭环驱动说明书 Rev1.3**（帧格式、示例、`0x6B`、`acc=0`、大端） | https://blog.csdn.net/zhangdatou666/article/details/132644047 |
| ZDT 驱动菜单项（`P_Serial` / `Checksum` / `MStep` / `Response` / `Cal`）与配置要点 | https://wenku.csdn.net/answer/o4xa6bsy0nq8 |
| ZDT Response 模式（None/Receive/Reached/Both）行为 | https://wenku.csdn.net/answer/5bxrzsgmx10e |
| 查询类功能码与返回解析（0x31/0x35/0x36/0x3A/0x3B） | https://blog.csdn.net/hope816/article/details/150551149 ・ https://deepwiki.com/3amson3902/UART_Command_Sender/3.2-emm42-v5.0-plugin |
| MaixCAM 作上位机控张大头 42 步进的实例 | https://blog.csdn.net/hope816/article/details/150551149 |
| 2025 电赛 E 题题目原文 | https://nuedc.org/problems/2025_E题_简易自行瞄准装置.pdf |
| 本项目 B 轨现状（输出契约、实测基线） | `maixcam2_rect_center/README.md`、`B轨代码技术解析与功能分析.md`、`tools/test_performance.py` |
| 双轨协作与门禁口径 | `AGENTS.md`、`MaixCAM应用代码规范.md`、`tools/check_app.py` |

---

> **下一步**：先回答 §12 的 **Q1（光斑极性）与 Q2（驱动器菜单现状）** —— 这两条决定 P0/P1 能否通过。
> 之后按 §8.5 的 P0→P3 顺序做：`stepper_link.py`（帧自测）→ 开环标定方向与每像素脉冲 → 单轴闭环只 P。
> 每阶段跑 `check_app.py` + `test_performance.py` + `test_aiming.py` + `test_gimbal_aim.py` 四门禁，最后再上板。
