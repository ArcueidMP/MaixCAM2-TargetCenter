"""ZDT Emm_V5.0 双串口链路（协议帧构造 + 非阻塞收发 + 链路健康）。

本模块可以在 PC 上导入；只有创建 StepperLink 时才会导入 MaixPy。
协议使用固定 0x6B 校验、大端参数、相对位置命令（`MaixCAM2串口直控云台需求文档.md` §6）。

帧构造是纯函数（``cmd_*``），返回帧解析也做成纯函数 + 可注入的 ``_ingest``，
这样即使没有 MaixPy 也能在 PC 上逐字节回归。

返回帧：[地址][功能码][状态][校验]，状态 `0x02`=成功、`0xE2`=未使能/堵转、
`0x00 0xEE`=错误命令（见 §6.1）。
"""

import math
import time

CR = 0x6B
STATUS_OK = 0x02
STATUS_E2 = 0xE2


def estimate_move_seconds(pulses, rpm, acc=0, pulses_per_rev=3200):
    """估算从静止到静止的位置运动时长（秒），含驱动器的加减速曲线。

    非零 acc 每 ``(256-acc)*50us`` 改变 1 RPM；短行程使用三角曲线，
    长行程使用梯形曲线。这里只做排程估算，不代表收到实际到位反馈。
    """
    pulses, rpm, pulses_per_rev = float(pulses), float(rpm), float(pulses_per_rev)
    if not all(math.isfinite(v) for v in (pulses, rpm, pulses_per_rev)):
        raise ValueError("motion parameters must be finite")
    if pulses <= 0:
        return 0.0
    if rpm <= 0 or pulses_per_rev <= 0:
        raise ValueError("rpm and pulses_per_rev must be positive")
    rpm = min(5000.0, rpm)
    speed = rpm * pulses_per_rev / 60.0
    acc = max(0, min(255, int(acc)))
    if acc == 0:
        return pulses / speed
    ramp = (256 - acc) * 50e-6
    acceleration = pulses_per_rev / (60.0 * ramp)
    ramp_seconds = rpm * ramp
    ramp_distance = speed * ramp_seconds
    if pulses <= ramp_distance:
        return 2.0 * math.sqrt(pulses / acceleration)
    return 2.0 * ramp_seconds + (pulses - ramp_distance) / speed


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


def _readback_pin(pinmap, pin):
    """回读引脚当前实际功能（AGENTS.md §6.4：'某轴不通'的第一手证据）。失败返回 '?'。"""
    try:
        return pinmap.get_pin_function(pin)
    except Exception:
        return "?"


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
        # 保留最早一条尚未确认的命令时间；持续发新命令不能推迟断链检测。
        self._pending_ack_since = {"yaw": None, "pitch": None}
        self._timeout_count = {"yaw": 0, "pitch": 0}
        self._e2_times = []
        self.fault_reason = None
        try:
            from maix import uart, pinmap
        except Exception as exc:
            self._stats["err"] += 1
            self._stats["yaw"] = self._stats["pitch"] = False
            self._init_error = exc
            return
        # ★ 实测轴映射定案（AGENTS.md §2）：yaw=UART2/B0B1/ttyS2/ID2，pitch=UART0/A28A29/ttyS0/ID1。
        #   两轴都要显式 pinmap（U0 走空表会静默不通）；ID 跟着"电机"走，不跟着端口。
        pin_sets = {
            "yaw": getattr(cfg, "YAW_PINS", {"B0": "UART2_TX", "B1": "UART2_RX"}),
            "pitch": getattr(cfg, "PITCH_PINS", {"A28": "UART0_TX", "A29": "UART0_RX"}),
        }
        devices = {"yaw": (getattr(cfg, "YAW_DEV", "/dev/ttyS2"),
                             getattr(cfg, "YAW_BAUD", 115200)),
                   "pitch": (getattr(cfg, "PITCH_DEV", "/dev/ttyS0"),
                               getattr(cfg, "PITCH_BAUD", 115200))}
        strict = bool(getattr(cfg, "PINMAP_STRICT", False))
        flush_rx = bool(getattr(cfg, "FLUSH_RX_ON_START", False))
        for axis in ("yaw", "pitch"):
            try:
                for pin, func in pin_sets[axis].items():
                    try:
                        result = pinmap.set_pin_function(pin, func)
                        failed = (result is not None and hasattr(result, "value")
                                  and result.value != 0)
                    except Exception as exc:              # 无该引脚 / 固件不支持
                        result, failed = exc, True
                    if failed:
                        message = "pinmap %s=%s failed: %s" % (pin, func, result)
                        if strict:
                            raise RuntimeError(message)
                        # 非严格模式：只告警，仍尝试打开 UART —— 否则真故障会被藏成"打不开"。
                        print("[gimbal] WARN " + message)
                    else:
                        print("[gimbal] pinmap %s=%s ok (readback=%s)" %
                              (pin, func, _readback_pin(pinmap, pin)))
                self._ports[axis] = uart.UART(*devices[axis])
                if flush_rx:
                    # U0(pitch) 是系统日志口：开机先丢掉内核启动日志/终端回显等 RX 残留。
                    self._flush_rx(axis)
            except Exception as exc:
                self._stats[axis] = False
                self._stats["err"] += 1
                print("[gimbal] %s UART open failed: %s" % (axis, exc))

    # ------------------------------------------------------------- 发送
    def _flush_rx(self, axis):
        """清空该轴串口开机前的 RX 残留（U0=系统日志口，会混内核启动日志/终端回显）。"""
        port = self._ports.get(axis)
        if port is None:
            return
        try:
            available = int(port.available(0))
            if available > 0:
                port.read(min(available, int(getattr(self.cfg, "UART_RX_MAX_BYTES", 256))), timeout=0)
        except Exception:
            pass
        self._rxbuf[axis] = bytearray()

    def _write(self, axis, payload, now=None):
        port = self._ports.get(axis)
        if port is None:
            return False
        try:
            n = port.write(payload)
            if n is not None and int(n) != len(payload):
                raise IOError("write returned %s" % n)
            self._stats["tx"] += 1
            self._last_cmd[axis] = time.perf_counter() if now is None else float(now)
            if self._pending_ack_since[axis] is None:
                self._pending_ack_since[axis] = self._last_cmd[axis]
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
                    data = port.read(min(available, int(getattr(self.cfg, "UART_RX_MAX_BYTES", 256))), timeout=0)
                    if data:
                        self._rxbuf[axis].extend(bytes(data))
                buf = self._rxbuf[axis]
                limit = max(4, int(getattr(self.cfg, "UART_RX_BUFFER_MAX_BYTES", 1024)))
                if len(buf) > limit:
                    del buf[:-limit]
                while len(buf) >= 4:
                    if buf[0] != self._addr(axis):
                        del buf[0]
                        continue
                    end = buf.find(bytes([CR]), 1)
                    if end < 0:
                        break
                    if end < 3:
                        del buf[0]
                        continue
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
            self._pending_ack_since[axis] = None
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
                pending = self._pending_ack_since[axis]
                if pending is None:
                    # 兼容只注入时间戳的离线链路，也排除已经得到 ACK 的命令。
                    if self._last_ack[axis] >= self._last_cmd[axis]:
                        self._timeout_count[axis] = 0
                        continue
                    pending = self._last_cmd[axis]
                if now - pending <= timeout_ms / 1000.0:
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

    def close(self):
        """控制线程停止后释放 UART；无 close 方法的 SDK 对象通过清引用析构。"""
        ports, self._ports = self._ports, {}
        self._stats["yaw"] = self._stats["pitch"] = False
        error = None
        for port in ports.values():
            close = getattr(port, "close", None)
            if close is not None:
                try:
                    close()
                except Exception as exc:
                    error = error or exc
        if error is not None:
            raise error
