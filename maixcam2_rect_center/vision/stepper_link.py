"""ZDT Emm_V5.0 双串口链路（协议帧构造 + 非阻塞收发 + 链路健康）。

本模块可以在 PC 上导入；只有创建 StepperLink 时才会导入 MaixPy。
协议使用固定 0x6B 校验、大端参数、相对位置命令（`MaixCAM2串口直控云台需求文档.md` §6）。

⚠ **MaixCAM2 可用串口（2026-10-09 核对官方 UART 表 + 引脚图 v1.0）**：

    串口   丝印      引脚        设备路径        pinmap
    UART0  U0T/U0R   A28 / A29   /dev/ttyS0     免（默认已复用）；系统终端/日志口
    UART1  U1T/U1R   A30 / A31   /dev/ttyS1     需（官方 UART 表标"无引脚"）
    UART2  U2T/U2R   B0  / B1    /dev/ttyS2     需
    UART3  U3T/U3R   B2  / B3    /dev/ttyS3     需
    UART4  U4T/U4R   A21 / A22   /dev/ttyS4     需；底部 6P 扩展座默认串口

本工程：轴 1 (yaw) = UART0(U0T/U0R)，轴 2 (pitch) = UART2(B0/B1)。
配置项 `YAW_DEV/YAW_BAUD/YAW_PINS` 与 `PITCH_DEV/PITCH_BAUD/PITCH_PINS`（旧名
`U1_*`/`U2_*` 仍兼容）。**pins 传空表 = 该轴不做 pinmap**；两轴默认都显式声明引脚功能，
且 `PINMAP_STRICT=False`（默认）时 pinmap 失败只警告、**仍打开 UART**，
避免因功能名不匹配把整条轴变成"打不开"（那会把真正的故障点藏起来）。

打开后还会用 `pinmap.get_pin_function()` **回读实测功能**（MaixCAM2 支持从硬件读），
启动日志以 `└ 引脚实测:` 一行打印 —— 这是判断"某一轴为什么不通"的第一手证据。


帧构造是纯函数（``cmd_*``），返回帧解析也做成纯函数 + 可注入的 ``_ingest``，
这样即使没有 MaixPy 也能在 PC 上逐字节回归。

返回帧：[地址][功能码][状态][校验]，状态 `0x02`=成功、`0xE2`=未使能/堵转、
`0x00 0xEE`=错误命令（见 §6.1）。
"""

import time

CR = 0x6B
STATUS_OK = 0x02
STATUS_E2 = 0xE2

# 各轴的"标称引脚"（仅用于诊断回读）：即使 config 里 pins 为空，也去查这些脚**实测**是什么功能。
NOMINAL_PINS = {
    "yaw": {"A28": "UART0_TX", "A29": "UART0_RX"},
    "pitch": {"B0": "UART2_TX", "B1": "UART2_RX"},
}


def _frame(addr, func, data=()):
    return bytes([int(addr) & 0xFF, int(func) & 0xFF] +
                 [int(x) & 0xFF for x in data] + [CR])


def cmd_enable(addr, enable=True, sync=False):
    return _frame(addr, 0xF3, (0xAB, 1 if enable else 0, 1 if sync else 0))


def cmd_speed(addr, ccw, rpm, acc=0, sync=False):
    rpm = max(0, min(5000, int(rpm)))
    return _frame(addr, 0xF6, (1 if ccw else 0, rpm >> 8, rpm,
                               int(acc), 1 if sync else 0))


def cmd_position(addr, ccw, rpm, pulses, acc=0, absolute=False, sync=False):
    rpm = max(0, min(5000, int(rpm)))
    pulses = max(0, min(0xFFFFFFFF, int(abs(pulses))))
    return _frame(addr, 0xFD, (
        1 if ccw else 0, rpm >> 8, rpm, int(acc),
        pulses >> 24, pulses >> 16, pulses >> 8, pulses,
        1 if absolute else 0, 1 if sync else 0))


def cmd_stop_now(addr, sync=False):
    return _frame(addr, 0xFE, (0x98, 1 if sync else 0))


def cmd_sync_motion(addr=0):
    return _frame(addr, 0xFF, (0x66,))


def cmd_read(addr, func):
    return _frame(addr, func, ())


def frame_status(frame):
    """返回帧第 3 字节 = 命令状态；帧太短返回 None。"""
    return int(frame[2]) if frame is not None and len(frame) >= 3 else None


def fault_reason(fb):
    """从 ``poll()``/``_ingest()`` 的结果里取出故障原因；**无故障返回 None**。

    ★ 必须走这个函数，不要直接写 ``fb.get("fault") is not None``：
    ``_ingest()`` 的约定是 **"fault: False 表示无故障"**（见其返回值），而
    ``False is not None`` 恒为 True —— 2026-10-09 现场 bug 就是 main.py 用
    ``is not None`` 判断，导致**开机第 1 帧就误判 FAULT 并停机**，
    云台从此不接收任何指令（表现："电机上电有劲、但永不自检、永不追踪"）。
    """
    if not fb:
        return None
    return fb.get("reason") if fb.get("fault") else None


def axis_cfg(cfg, axis):
    """读取某轴的 (device, baud, pins)。新命名 `YAW_*`/`PITCH_*` 优先，兼容旧 `U1_*`/`U2_*`。

    pins 为空 dict 表示**该轴不做 pinmap**（设备默认已复用为串口，如 UART0/U0T-U0R）。
    """
    if axis == "yaw":
        dev = getattr(cfg, "YAW_DEV", getattr(cfg, "U1_DEV", "/dev/ttyS0"))
        baud = getattr(cfg, "YAW_BAUD", getattr(cfg, "U1_BAUD", 115200))
        pins = getattr(cfg, "YAW_PINS", getattr(cfg, "U1_PINS", None))
        if pins is None:
            pins = {}
    else:
        dev = getattr(cfg, "PITCH_DEV", getattr(cfg, "U2_DEV", "/dev/ttyS2"))
        baud = getattr(cfg, "PITCH_BAUD", getattr(cfg, "U2_BAUD", 115200))
        pins = getattr(cfg, "PITCH_PINS", getattr(cfg, "U2_PINS", None))
        if pins is None:
            pins = {"B0": "UART2_TX", "B1": "UART2_RX"}
    return str(dev), int(baud), dict(pins)


def _pins_text(pins):
    """把引脚映射打印成可读串；空表表示"免 pinmap（设备默认已复用为串口）"。"""
    if not pins:
        return "(默认复用,免pinmap)"
    return ",".join("%s=%s" % kv for kv in pins.items())


class StepperLink:
    """两条独立 UART 链路；任一轴打开失败不会阻塞另一轴。"""

    def __init__(self, cfg):
        self.cfg = cfg
        self._ports = {}
        self._rxbuf = {"yaw": bytearray(), "pitch": bytearray()}
        self._stats = {"tx": 0, "rx": 0, "ok": 0, "e2": 0, "err": 0,
                       "timeout": 0, "yaw": True, "pitch": True}
        self._last_status = {"yaw": None, "pitch": None}
        self._last_cmd = {"yaw": 0.0, "pitch": 0.0}
        self._last_ack = {"yaw": 0.0, "pitch": 0.0}
        self._timeout_count = {"yaw": 0, "pitch": 0}
        self._e2_times = []
        self.fault_reason = None
        self._init_error = None
        # 逐轴链路诊断（启动打印 / uart_probe 用）：设备、引脚、pinmap 结果、是否打开、失败原因。
        self.axis_info = {}
        for axis in ("yaw", "pitch"):
            dev, baud, pins = axis_cfg(cfg, axis)
            self.axis_info[axis] = {
                "device": dev, "baud": baud, "pins": pins,
                "pinmap_ok": None, "pinmap_error": None, "pin_funcs": {},
                "opened": False, "error": None,
            }
        try:
            from maix import uart, pinmap
            try:
                from maix import err
            except Exception:
                err = None
        except Exception as exc:
            self._stats["err"] += 1
            self._stats["yaw"] = self._stats["pitch"] = False
            self._init_error = exc
            for info in self.axis_info.values():
                info["error"] = "maix import failed: %s" % exc
            return
        strict = bool(getattr(cfg, "PINMAP_STRICT", False))
        for axis in ("yaw", "pitch"):
            info = self.axis_info[axis]
            # ---- 1) pinmap（失败是否致命由 PINMAP_STRICT 决定）----
            try:
                for pin, func in info["pins"].items():
                    result = pinmap.set_pin_function(pin, func)
                    # 严格判定：成功返回 None/0；失败返回 err.Err（err.check_raise 会抛）。
                    # 旧写法只在"返回对象带 .value 且非 0"时报错，遇到未知返回类型会静默吞掉失败。
                    if err is not None:
                        err.check_raise(result, "pinmap set %s=%s failed" % (pin, func))
                    elif result is not None and int(getattr(result, "value", 0) or 0) != 0:
                        raise RuntimeError("pinmap set %s=%s failed: %s" % (pin, func, result))
                info["pinmap_ok"] = True
            except Exception as exc:
                info["pinmap_ok"] = False
                info["pinmap_error"] = str(exc)
                if strict:
                    info["error"] = "pinmap: %s" % exc
                    self._stats[axis] = False
                    self._stats["err"] += 1
                    print("[gimbal] %s 链路不可用（PINMAP_STRICT=True）| dev=%s pins=%s | %s" %
                          (axis, info["device"], _pins_text(info["pins"]), exc))
                    continue
                print("[gimbal] %s pinmap 设置失败（按 PINMAP_STRICT=False 继续打开）：%s"
                      % (axis, exc))
            # ---- 2) 回读实测引脚功能（诊断；MaixCAM2 支持直接从硬件读）----
            watch = dict(info["pins"]) or dict(NOMINAL_PINS.get(axis, {}))
            funcs = {}
            for pin in watch:
                try:
                    funcs[pin] = pinmap.get_pin_function(pin)
                except Exception as exc:
                    funcs[pin] = "?err:%s" % exc
            info["pin_funcs"] = funcs
            # ---- 3) 打开 UART ----
            try:
                self._ports[axis] = uart.UART(info["device"], info["baud"])
                info["opened"] = True
            except Exception as exc:
                info["error"] = str(exc)
                self._stats[axis] = False
                self._stats["err"] += 1
                print("[gimbal] %s 串口打开失败 | dev=%s | %s" % (axis, info["device"], exc))
        # 开机清空缓冲：UART0(U0) 上常有内核启动日志/终端回显，必须丢掉再发指令。
        if bool(getattr(cfg, "FLUSH_RX_ON_START", True)):
            self.flush()

    def flush(self, axis=None):
        """丢弃串口缓冲区里的陈旧字节（U0 上可能有内核日志/终端回显），返回丢弃字节数。"""
        axes = tuple(self._ports) if axis is None else (axis,)
        dropped = 0
        for name in axes:
            port = self._ports.get(name)
            self._rxbuf[name] = bytearray()
            if port is None:
                continue
            try:
                avail = int(port.available(0))
                if avail > 0:
                    data = port.read(min(avail, 256), 0)
                    dropped += len(bytes(data)) if data else 0
            except Exception:
                pass
        return dropped

    def health_lines(self):
        """返回两轴链路自检的可读行（启动时打印，用于现场确认串口是否真的开上了）。"""
        lines = []
        for axis in ("yaw", "pitch"):
            info = self.axis_info[axis]
            if info["opened"]:
                state = "OK"
            elif self._init_error is not None:
                state = "NO-MAIX"
            else:
                state = "FAIL"
            lines.append("[gimbal] %-5s %-4s dev=%-12s baud=%d pins=%-30s pinmap=%s -> %s%s" % (
                axis, state, info["device"], info["baud"], _pins_text(info["pins"]),
                info["pinmap_ok"], "opened" if info["opened"] else "NOT opened",
                ("  err=%s" % info["error"]) if info["error"] else ""))
            # 实测引脚功能：判断"该轴明明打开了却不通"的关键证据（例如 A28 实测不是 UART0_TX）。
            funcs = info.get("pin_funcs") or {}
            if funcs:
                lines.append("[gimbal]   └ 引脚实测: %s" %
                             "  ".join("%s=%s" % kv for kv in funcs.items()))
        return lines

    def available_axes(self):
        return [axis for axis in ("yaw", "pitch") if self.axis_info[axis]["opened"]]

    # ------------------------------------------------------------- 发送
    def _write(self, axis, payload, now=None):
        port = self._ports.get(axis)
        if port is None:
            return False
        try:
            n = port.write(payload)
            if n is not None and int(n) < 0:
                raise IOError("write returned %s" % n)
            self._stats["tx"] += 1
            self._last_cmd[axis] = time.perf_counter() if now is None else float(now)
            return True
        except Exception as exc:
            self._stats["err"] += 1
            print("[gimbal] %s TX failed: %s" % (axis, exc))
            return False

    def enable(self, axis=None, now=None):
        axes = ("yaw", "pitch") if axis is None else (axis,)
        ok = True
        for name in axes:
            ok = self._write(name, cmd_enable(self._addr(name)), now=now) and ok
        return ok

    def move(self, axis, ccw, pulses, rpm, acc=0, now=None):
        if axis not in ("yaw", "pitch"):
            raise ValueError("axis must be yaw or pitch")
        if int(pulses) <= 0:
            return False
        return self._write(axis, cmd_position(self._addr(axis), ccw, rpm, pulses, acc=acc),
                           now=now)

    def speed(self, axis, ccw, rpm, acc=0, now=None):
        return self._write(axis, cmd_speed(self._addr(axis), ccw, rpm, acc=acc), now=now)

    def stop(self, axis=None, now=None):
        axes = ("yaw", "pitch") if axis is None else (axis,)
        ok = True
        for name in axes:
            ok = self._write(name, cmd_stop_now(self._addr(name)), now=now) and ok
        return ok

    def _addr(self, axis):
        return int(getattr(self.cfg, "ADDR_YAW" if axis == "yaw" else "ADDR_PITCH",
                           1 if axis == "yaw" else 2))

    # ------------------------------------------------------------- 接收
    def _drain(self):
        """非阻塞读走两条串口现有字节，切出完整返回帧。"""
        frames = []
        for axis, port in tuple(self._ports.items()):
            try:
                available = int(port.available(0))
                if available > 0:
                    # 明确 timeout=0；每轴每帧最多 256 字节，避免 UART0 日志洪水拖长循环。
                    data = port.read(min(available, 256), 0)
                    if data:
                        self._rxbuf[axis].extend(bytes(data))
                buf = self._rxbuf[axis]
                while len(buf) >= 4:
                    if buf[0] != self._addr(axis):
                        del buf[0]
                        continue
                    end = buf.find(bytes([CR]), 1)
                    if end < 3:
                        break
                    frames.append((axis, bytes(buf[:end + 1])))
                    del buf[:end + 1]
                    self._stats["rx"] += 1
            except Exception:
                self._stats["err"] += 1
        return frames

    def _ingest(self, frames, now):
        """解析返回帧：统计成功/E2、E2 自动补使能、E2 洪水与 ACK 超时判障。

        返回 dict(ok, e2, reenabled, fault, reason)。本方法不碰 MaixPy，可直接单测。
        """
        now = float(now)
        out = {"frames": list(frames), "ok": 0, "e2": 0,
               "reenabled": [], "fault": False, "reason": None}
        e2_axes = []
        for axis, frame in frames:
            status = frame_status(frame)
            self._last_status[axis] = status
            if status is None:
                continue
            # 凡是能解析出状态的帧都证明链路活着 -> 刷新该轴 ACK 时刻。
            self._last_ack[axis] = now
            if status == STATUS_OK:
                out["ok"] += 1
                self._stats["ok"] += 1
            elif status == STATUS_E2:
                out["e2"] += 1
                self._stats["e2"] += 1
                e2_axes.append(axis)
                self._e2_times.append(now)
        if not bool(getattr(self.cfg, "ENABLE_ACK_WATCHDOG", True)):
            return out
        if e2_axes and bool(getattr(self.cfg, "E2_REENABLE", True)):
            for axis in sorted(set(e2_axes)):
                if self.enable(axis, now=now):
                    out["reenabled"].append(axis)
        window = float(getattr(self.cfg, "E2_WINDOW_S", 2.0))
        self._e2_times = [t for t in self._e2_times if now - t <= window]
        limit = int(getattr(self.cfg, "E2_FAULT_COUNT", 10))
        if limit > 0 and len(self._e2_times) > limit:
            out["fault"] = True
            out["reason"] = "e2_flood(%d in %.1fs)" % (len(self._e2_times), window)
            self.fault_reason = out["reason"]
            return out
        # ACK 硬超时：只在收到过确认帧（Response=Receive/Reached/Both）之后才生效，
        # Response=None 时永远收不到确认帧，该判据自动不启用，不会误判 FAULT。
        timeout_ms = float(getattr(self.cfg, "RX_TIMEOUT_MS", 0) or 0)
        if timeout_ms > 0 and self._stats["ok"] > 0:
            for axis in self._ports:
                if now - self._last_cmd[axis] <= timeout_ms / 1000.0:
                    self._timeout_count[axis] = 0
                    continue
                if self._last_ack[axis] >= self._last_cmd[axis]:
                    self._timeout_count[axis] = 0
                    continue
                self._timeout_count[axis] += 1
                self._stats["timeout"] += 1
                if self._timeout_count[axis] > int(getattr(self.cfg, "RX_TIMEOUT_LIMIT", 20)):
                    out["fault"] = True
                    out["reason"] = "ack_timeout:%s" % axis
                    self.fault_reason = out["reason"]
                    return out
        return out

    def poll(self, now=None):
        """非阻塞读两条串口并处理反馈；返回 `_ingest` 的 dict（含 frames）。"""
        now = time.perf_counter() if now is None else float(now)
        return self._ingest(self._drain(), now)

    def stats(self):
        return dict(self._stats)

    def status_line(self):
        """单行链路心跳（周期日志用）：证明"是否真的发过帧 / 是否收到过回帧"。

        判读：tx=0 -> 一条指令都没发出（链路没打开或被上层拦住）；
              tx>0 rx=0 -> 发出去了但驱动器一个字节都没回（查 P_Serial/地址/波特率/接线）；
              ok>0      -> 驱动器确实在应答，链路通。
        """
        s = self._stats
        axes = []
        for axis in ("yaw", "pitch"):
            info = self.axis_info[axis]
            if not info["opened"]:
                axes.append("%s:FAIL" % axis)
            elif self._last_status[axis] is None:
                axes.append("%s:open" % axis)
            else:
                axes.append("%s:open,0x%02X" % (axis, self._last_status[axis]))
        return "[gimbal] 链路 tx=%d rx=%d ok=%d e2=%d to=%d drop=%d | %s" % (
            s["tx"], s["rx"], s["ok"], s["e2"], s["timeout"], s["err"],
            " ".join(axes))
