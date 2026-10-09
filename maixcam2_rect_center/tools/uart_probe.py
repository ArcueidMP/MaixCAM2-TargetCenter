"""板端双串口诊断：确认两个 ZDT 电机接口是否真的打开、能否驱动电机。

在 **MaixCAM2 上**运行（MaixVision「运行当前文件」，或 ssh 进板子跑）：:

    python uart_probe.py                  # 只读诊断：设备表 / pinmap 真值 / 打开结果 / 使能回执
    python uart_probe.py --listen 3       # 额外盯 3 秒原始 RX（看驱动器到底回不回帧）
    python uart_probe.py --move           # 两轴各小幅 CW/CCW 一次（⚠ 先把云台架空！）
    python uart_probe.py --move pitch     # 只动某一轴
    python uart_probe.py --loop yaw       # 先把该轴 TX-RX 短接，再跑回环自测

本脚本直接读 `config.py` 并复用 `vision.stepper_link` 的**同一套打开逻辑**，
所以它显示 OK 就代表 `main.py` 也一定能开上。

本工程映射（2026-10-09 核对官方 UART 表 + 引脚图 v1.0）：

    轴    串口   丝印      引脚        设备路径        备注
    yaw   UART2  U2T/U2R   B0  / B1    /dev/ttyS2      显式 pinmap（★ 实测挂此口，ID=2）
    pitch UART0  U0T/U0R   A28 / A29   /dev/ttyS0      显式 pinmap；系统终端/日志口（ID=1）

判读要点（按现场出现频率排序）：
  · 两轴都 OK、电机不转、收不到回帧 -> ★ **驱动器 `P_Serial` 没设成 `UART_FUN`**（出厂默认
    `RxTx_OFF`，串口指令被完全忽略）。用驱动器 OLED 菜单改；顺带核 `Checksum=0x6B`、
    `ID_Addr`、`UartBaud=115200`、`Response=Receive`，未校准时先跑 `Cal`。
  · 电机通电锁轴但不动       -> `En` 不接线默认 Hold（一直使能），"有劲"≠"串口通"，别被误导。
  · 某轴 `open ... -> FAIL`  -> 引脚被别的功能占用（SPI/PWM）或设备名不对。
  · 只有一轴转               -> 另一轴多半是接线（TX/RX 没交叉、GND 没共地、驱动器地址不对）。
  · 确证 MaixCAM2 侧 UART    -> `--loop <轴>`（先把该轴 TX-RX 短接）：能收回自己发的字节即硬件正常。
  · UART0(丝印 U0) 偶尔乱动   -> 那是系统日志口，内核日志会串进电机 RX；优先改用 UART1/UART3。
"""
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def _probe_pins(pinmap, pins, axis):
    """打印该轴引脚的**实测**功能。

    config 里 pins 为空时回退到标称引脚（A28/A29 等）—— 仍能看到 UART0 当前被复用成了什么。
    这正是"某轴明明打开了却不通"最需要的第一手证据。
    """
    try:
        from vision.stepper_link import NOMINAL_PINS
    except Exception:
        NOMINAL_PINS = {}
    watch = dict(pins) or dict(NOMINAL_PINS.get(axis, {}))
    if not watch:
        print("         %-5s (无引脚信息)" % axis)
        return
    for pin, want in watch.items():
        try:
            cur = pinmap.get_pin_function(pin)
            print("         %-5s 期望 %-12s 实测 %-12s %s" %
                  (pin, want, cur, "一致" if str(cur) == str(want) else "<-- 不一致!"))
        except Exception as exc:
            print("         %-5s 期望 %-12s 查询失败: %s" % (pin, want, exc))
    if not pins:
        print("         （config 里该轴 pins 为空，上面按标称引脚回读）")


def main():
    argv = sys.argv[1:]
    do_move = "--move" in argv
    move_axis = None
    if "--move" in argv:
        i = argv.index("--move")
        if i + 1 < len(argv) and argv[i + 1] in ("yaw", "pitch"):
            move_axis = argv[i + 1]
    loop_axis = None
    if "--loop" in argv:
        i = argv.index("--loop")
        loop_axis = argv[i + 1] if i + 1 < len(argv) else "yaw"
    listen_s = 0.0
    if "--listen" in argv:
        i = argv.index("--listen")
        if i + 1 < len(argv):
            try:
                listen_s = float(argv[i + 1])
            except ValueError:
                listen_s = 3.0
        else:
            listen_s = 3.0

    try:
        from maix import uart, pinmap
    except Exception as exc:
        print("需要 MaixPy 环境（请在 MaixCAM2 板上运行）：%s" % exc)
        return 1

    from config import Config
    from vision.stepper_link import StepperLink, axis_cfg, cmd_enable, cmd_position, frame_status

    print("=" * 74)
    print("MaixCAM2 双串口诊断 %s" % os.path.basename(ROOT))
    print("=" * 74)
    try:
        print("[devices] uart.list_devices() = %s" % (uart.list_devices(),))
    except Exception as exc:
        print("[devices] list_devices 失败: %s" % exc)

    for axis in ("yaw", "pitch"):
        dev, baud, pins = axis_cfg(Config, axis)
        print("[%s] dev=%s baud=%d" % (axis, dev, baud))
    print("[pinmap] 当前引脚功能（和配置里的期望值对照）：")
    for axis in ("yaw", "pitch"):
        _dev, _baud, pins = axis_cfg(Config, axis)
        _probe_pins(pinmap, pins, axis)

    link = StepperLink(Config)
    print("-" * 74)
    print("[链路] StepperLink.health_lines()：")
    for line in link.health_lines():
        print("  " + line)
    dropped = link.flush()
    print("[链路] 开机清空陈旧 RX 字节 = %d（UART0 上常有内核日志/终端回显）" % dropped)

    for axis in ("yaw", "pitch"):
        if not link.axis_info[axis]["opened"]:
            print("[%s] 跳过（串口未打开）" % axis)
            continue
        frame = cmd_enable(link._addr(axis))
        n = link._write(axis, frame)
        print("[%s] TX 使能 %s -> write=%s" % (axis, frame.hex(" "), n))
    time.sleep_ms(120)
    fb = link.poll()
    got_frames = bool(fb["frames"])
    for axis, frame in fb["frames"]:
        print("[rx] %s <- %s  状态=%s" % (axis, frame.hex(" "), frame_status(frame)))
    if not fb["frames"]:
        print("[rx] 无回执")
        print("     · 若驱动器 `Response` 已设为 Receive/Reached/Both，收不到回帧 = 驱动器**没在解析串口指令**，")
        print("       最常见原因就是 `P_Serial` 没设成 `UART_FUN`（出厂默认 RxTx_OFF）—— 请先用 OLED 菜单改。")
        print("     · 若 `Response=None`，本就不回帧，属正常；确证请用 `--loop <轴>`（先把该轴 TX-RX 短接）。")

    if listen_s > 0:
        print("-" * 74)
        print("[listen] 盯 %.1fs 原始 RX ..." % listen_s)
        end = time.perf_counter() + listen_s
        total = 0
        while time.perf_counter() < end:
            for axis in ("yaw", "pitch"):
                info = link.axis_info[axis]
                port = link._ports.get(axis)
                if port is None:
                    continue
                try:
                    avail = int(port.available(0))
                    data = bytes(port.read(avail)) if avail > 0 else b""
                except Exception:
                    data = b""
                if data:
                    total += len(data)
                    print("[listen] %s <- %s" % (axis, data.hex(" ")))
            time.sleep_ms(20)
        if total == 0:
            print("[listen] %.1fs 内没有任何字节（驱动器默认不主动上报，属正常）" % listen_s)

    if loop_axis in ("yaw", "pitch"):
        port = link._ports.get(loop_axis)
        print("-" * 74)
        if port is None:
            print("[loop-%s] 串口未打开，跳过" % loop_axis)
        else:
            probe = bytes([0xAA, 0x55, 0x01, 0x02, 0x6B])
            port.write(probe)
            time.sleep_ms(60)
            try:
                avail = int(port.available(0))
                back = bytes(port.read(avail)) if avail > 0 else b""
            except Exception:
                back = b""
            print("[loop-%s] 发 %s 收 %s -> %s" % (
                loop_axis, probe.hex(" "), back.hex(" "),
                "回环通过（UART 硬件链路 OK）" if probe in back
                else "未收到 —— 先把该轴 TX-RX 短接再试"))

    if do_move:
        print("-" * 74)
        targets = [move_axis] if move_axis else ["yaw", "pitch"]
        print("[move] %s 各小幅 CW/CCW 400 脉冲（确认云台已架空）..." % "+".join(targets))
        for name in targets:
            if not link.axis_info[name]["opened"]:
                print("[%s] 串口未打开，跳过" % name)
                continue
            move_rpm = int(getattr(Config, "SELF_TEST_RPM", 60))
            move_acc = int(getattr(Config, "SELF_TEST_ACC", 60))
            for ccw in (False, True):
                frame = cmd_position(link._addr(name), ccw, move_rpm, 400, acc=move_acc)
                link._write(name, frame)
                print("[%s] move %s rpm=%d acc=%d  %s" %
                      (name, "CCW" if ccw else "CW", move_rpm, move_acc, frame.hex(" ")))
                time.sleep_ms(1200)
            print("[%s] 若电机没动：查接线(TX/RX 交叉、GND 共地)、驱动器地址(%d)、"
                  "波特率(115200)、Checksum 是否固定 0x6B" % (name, link._addr(name)))

    print("=" * 74)
    print("判读：")
    print("  1) 两轴都应 `opened`（yaw=U2/ttyS2、pitch=U0/ttyS0 —— 按实机接线）。")
    axes_open = link.available_axes()
    if len(axes_open) < 2:
        print("     ⚠ 有串口没打开 -> 查该引脚是否被 SPI/PWM 占用、设备名是否正确。")
    if axes_open and not got_frames and listen_s > 0:
        print("  2) 发使能帧后收不到任何回帧 —— 按优先级查：")
        print("     ① 驱动器 `P_Serial` 未设 `UART_FUN`（出厂默认 RxTx_OFF，串口指令被完全忽略）；")
        print("     ② `Checksum` 不是 `0x6B`；③ `ID_Addr` 与 config 的 ADDR_* 不一致；")
        print("     ④ `UartBaud` 不是 115200；⑤ 未做 `Cal` 编码器校准（此时菜单只有 6 项）。")
        print("     先用 OLED 菜单逐项核对；确证 MaixCAM 侧 UART 是否正常请跑 `--loop <轴>`（短接 TX-RX）。")
    if "yaw" in axes_open:
        print("  3) yaw 的 U0 是系统日志口：若偶发乱动是内核日志串入，改用 UART3(B2/B3) 即可")
        print("     （只改 config 的 YAW_DEV=\"/dev/ttyS3\" 与 YAW_PINS={\"B2\":\"UART3_TX\",\"B3\":\"UART3_RX\"}）。")
    print("  4) 只有一轴能动时，先看上面 `引脚实测`：期望≠实测 -> 该脚根本没切到串口功能，")
    print("     这就是那条轴『打开了却不通』的直接原因。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
