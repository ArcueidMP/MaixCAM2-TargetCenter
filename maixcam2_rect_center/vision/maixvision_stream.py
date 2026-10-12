"""MaixVision protocol-0 sender, isolated from camera/display/UART and their GIL.

Protocol reference: Sipeed/MaixCDK 30f4b8b7, vision/src/maix_image_trans.cpp.
The board already runs the WebSocket bridge on localhost:7899; we are its CLIENT.
Only one bounded raw frame lives in an mmap. The producer takes a nonblocking
file lock and drops a frame if busy. JPEG, WebSocket and reconnect run in a fresh
Python process which never imports maix (that import opens a default UART).
The source string also works when tools/build_single embeds this module.
"""
try:
    import fcntl
except ImportError:  # PC/single-file smoke on Windows; stream needs the Linux board.
    fcntl = None
import mmap
import os
import struct
import subprocess
import sys
import tempfile
import time

import numpy as np


_CHILD_SOURCE = r'''
import base64
import fcntl
import hashlib
import mmap
import os
import select
import socket
import struct
import sys
import time
import cv2
import numpy as np

HEADER = 128
FRAME = struct.Struct("<QIII")
STATUS = struct.Struct("<iddQQ")
STATUS_OFFSET = 24
MAGIC = bytes((0xAC, 0xBE, 0xCB, 0xCA))
MAX_CONTROL = 65536


class ProtocolError(Exception):
    pass


def protocol_frame(message, payload=b""):
    body = bytes((0, message)) + payload
    frame = MAGIC + struct.pack("<I", len(body) + 1) + body
    return frame + bytes((sum(frame) & 255,))


def protocol_parse(frame):
    if (len(frame) < 11 or len(frame) > MAX_CONTROL or frame[:4] != MAGIC
            or struct.unpack_from("<I", frame, 4)[0] != len(frame) - 8
            or frame[8] != 0 or sum(frame[:-1]) & 255 != frame[-1]):
        raise ProtocolError("unsupported/corrupt MaixVision protocol")
    return frame[9], frame[10:-1]


class WebSocketClient:
    """Bounded RFC6455 client. No proxy: this is the local board bridge socket."""
    def __init__(self, host, port, timeout=.5):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.buffer = bytearray()
        try:
            key = base64.b64encode(os.urandom(16)).decode("ascii")
            request = ("GET / HTTP/1.1\r\nHost: %s:%d\r\nUpgrade: websocket\r\n"
                       "Connection: Upgrade\r\nSec-WebSocket-Key: %s\r\n"
                       "Sec-WebSocket-Version: 13\r\n\r\n" % (host, port, key))
            self.sock.sendall(request.encode("ascii"))
            deadline = time.monotonic() + timeout
            while b"\r\n\r\n" not in self.buffer:
                if len(self.buffer) > 4096 or time.monotonic() > deadline:
                    raise ProtocolError("WebSocket handshake exceeded bound")
                chunk = self.sock.recv(4096)
                if not chunk:
                    raise ConnectionError("bridge closed during handshake")
                self.buffer.extend(chunk)
            raw, remaining = self.buffer.split(b"\r\n\r\n", 1)
            if len(raw) > 4096:
                raise ProtocolError("WebSocket handshake exceeded bound")
            self.buffer = bytearray(remaining)
            lines = raw.decode("ascii").split("\r\n")
            headers = {}
            for line in lines[1:]:
                name, value = line.split(":", 1)
                headers[name.lower()] = value.strip()
            accept = base64.b64encode(hashlib.sha1(
                (key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")).digest()).decode("ascii")
            if (lines[0].split()[1] != "101"
                    or headers.get("upgrade", "").lower() != "websocket"
                    or "upgrade" not in [x.strip() for x in headers.get("connection", "").lower().split(",")]
                    or headers.get("sec-websocket-accept") != accept
                    or "sec-websocket-extensions" in headers):
                raise ProtocolError("invalid WebSocket handshake")
        except Exception:
            self.sock.close()
            raise

    def send(self, payload, opcode=2):
        # All CLIENT frames must be masked, including ping replies.
        n = len(payload)
        if n > 4 * 1024 * 1024 or (opcode >= 8 and n > 125):
            raise ProtocolError("outbound WebSocket payload exceeded bound")
        head = bytes((0x80 | opcode, 0x80 | (n if n < 126 else 126 if n < 65536 else 127)))
        if n >= 126:
            head += struct.pack("!H" if n < 65536 else "!Q", n)
        mask = os.urandom(4)
        data = np.frombuffer(payload, np.uint8).copy()
        for i in range(4):
            data[i::4] ^= mask[i]
        self.sock.sendall(head + mask + data.tobytes())

    def _read(self, n):
        deadline = time.monotonic() + .5
        while len(self.buffer) < n:
            if time.monotonic() > deadline:
                raise TimeoutError("partial WebSocket frame timed out")
            chunk = self.sock.recv(min(65536, max(1, n - len(self.buffer))))
            if not chunk:
                raise ConnectionError("bridge disconnected")
            self.buffer.extend(chunk)
        value = bytes(self.buffer[:n])
        del self.buffer[:n]
        return value

    def receive(self):
        if not self.buffer and not select.select([self.sock], [], [], 0)[0]:
            return None
        a, b = self._read(2)
        if not a & 0x80 or a & 0x70 or b & 0x80:
            raise ProtocolError("unsupported fragmented/extended/masked server frame")
        opcode, n = a & 15, b & 127
        if n in (126, 127):
            n = struct.unpack("!H" if n == 126 else "!Q", self._read(2 if n == 126 else 8))[0]
        if n > MAX_CONTROL or (opcode >= 8 and n > 125):
            raise ProtocolError("bridge control payload exceeded bound")
        payload = self._read(n)
        if opcode == 8:
            raise ConnectionError("bridge closed WebSocket")
        if opcode == 9:
            self.send(payload, opcode=10)
            return None
        if opcode == 10:
            return None
        if opcode != 2:
            raise ProtocolError("expected binary MaixVision protocol")
        return payload

    def close(self):
        self.sock.close()


def _status(mm, fd, state, sent=0, sequence=0, duration=0.0):
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return
    try:
        STATUS.pack_into(mm, STATUS_OFFSET, state, time.monotonic(), duration, sent, sequence)
    finally:
        fcntl.flock(fd, fcntl.LOCK_UN)


def _latest(mm, fd, capacity, previous):
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return None
    try:
        sequence, width, height, n = FRAME.unpack_from(mm)
        if not sequence or sequence == previous:
            return None
        if (width <= 0 or height <= 0 or n != width * height * 3 or n > capacity):
            raise ProtocolError("invalid shared frame dimensions")
        return sequence, np.frombuffer(mm[HEADER:HEADER + n], np.uint8).reshape(height, width, 3)
    finally:
        fcntl.flock(fd, fcntl.LOCK_UN)


def _run_sender(fd, capacity, parent, host, port, fps, quality, retry, retry_max):
    cv2.setNumThreads(1)
    # flock locks belong to an open file description. pass_fds inherits the
    # parent's description, whose locks would otherwise NOT exclude this child.
    inherited = fd
    fd = os.open("/proc/self/fd/%d" % inherited, os.O_RDWR)
    os.close(inherited)
    mm = mmap.mmap(fd, HEADER + capacity)
    client = None
    sent = previous = 0
    retry_max = max(retry, retry_max)
    delay, next_retry, next_image = retry, 0.0, 0.0
    duration = 0.0
    state, fmt = 0, 1
    ready, handshake_deadline = False, 0.0
    try:
        while os.getppid() == parent and not mm[124]:
            now = time.monotonic()
            if client is None:
                _status(mm, fd, state, sent, previous, duration)
                if now < next_retry:
                    time.sleep(.02)
                    continue
                try:
                    client = WebSocketClient(host, port)
                    client.send(protocol_frame(1, b"code"))
                    ready, fmt = False, 1
                    handshake_deadline = time.monotonic() + 2.0
                    state = 0
                except Exception as exc:
                    client = None
                    state, next_retry = (3 if isinstance(exc, ProtocolError) else 2), time.monotonic() + delay
                    delay = min(retry_max, delay * 2)
                    continue
            try:
                # Bounded control work: an abusive peer cannot starve newest images.
                for _ in range(8):
                    raw = client.receive()
                    if raw is None:
                        break
                    message, payload = protocol_parse(raw)
                    if message == 2:
                        if payload != b"\x01":
                            raise ProtocolError("MaixVision rejected sender")
                        ready, state, delay = True, 1, retry
                    elif message == 14 and len(payload) == 1:
                        requested = payload[0]
                        if requested <= 2:
                            fmt = requested
                        client.send(protocol_frame(15, bytes((int(requested <= 2), requested))))
                        state = 5 if ready and fmt == 0 else 1 if ready else 0
                    elif message == 21 and len(payload) == 1:
                        quality = min(100, payload[0])
                        client.send(protocol_frame(22, bytes((1, quality))))
                    else:
                        raise ProtocolError("unsupported MaixVision control message")
                if not ready and now > handshake_deadline:
                    raise TimeoutError("MaixVision sender acknowledgment timed out")
                if ready and fmt and now >= next_image:
                    frame = _latest(mm, fd, capacity, previous)
                    if frame is not None:
                        sequence, array = frame
                        started = time.monotonic()
                        extension = ".jpg" if fmt == 1 else ".png"
                        args = [cv2.IMWRITE_JPEG_QUALITY, quality] if fmt == 1 else [cv2.IMWRITE_PNG_COMPRESSION, 1]
                        ok, encoded = cv2.imencode(extension, array, args)
                        if not ok:
                            raise RuntimeError("preview encoding failed")
                        client.send(protocol_frame(6, bytes((fmt,)) + encoded.tobytes()))
                        previous, sent = sequence, sent + 1
                        next_image = time.monotonic() + 1.0 / max(1.0, fps)
                        duration = time.monotonic() - started
                        _status(mm, fd, state, sent, previous, duration)
                _status(mm, fd, state, sent, previous, duration)
                time.sleep(.005 if ready else .02)
            except Exception as exc:
                state = 3 if isinstance(exc, ProtocolError) else 2
                client.close()
                client, ready = None, False
                next_retry = time.monotonic() + delay
                delay = min(retry_max, delay * 2)
                _status(mm, fd, state, sent, previous)
    finally:
        if client is not None:
            client.close()
        _status(mm, fd, 4, sent, previous)
        mm.close()
        os.close(fd)
'''

# Executing only defines the protocol helpers. No connection, worker or SDK import.
exec(_CHILD_SOURCE.replace("import fcntl\n", ""), globals())


class MaixVisionStream:
    """One frame slot + one sender process. submit never waits for encoding/network."""
    STATES = {0: "connecting", 1: "ready", 2: "retrying", 3: "unsupported",
              4: "closed", 5: "paused"}

    def __init__(self, cfg, width, height):
        if fcntl is None:
            raise RuntimeError("MaixVision isolated stream requires the Linux board")
        self.cfg = cfg
        self.capacity = max(1, int(width) * int(height) * 3)
        self.file = tempfile.TemporaryFile(prefix="maixvision-frame-")
        self.file.truncate(HEADER + self.capacity)
        self.mm = mmap.mmap(self.file.fileno(), HEADER + self.capacity)
        self.process = None
        self.sequence = self.dropped = 0
        self.closed = False
        self.failed = False
        self._last_status = {"state": "connecting", "sent": 0, "sequence": 0,
                             "duration_ms": 0.0, "heartbeat": 0.0}

    def _start(self):
        fd = self.file.fileno()
        args = [fd, self.capacity, os.getpid(), getattr(self.cfg, "SHOW_STREAM_HOST", "127.0.0.1"),
                int(getattr(self.cfg, "SHOW_STREAM_PORT", 7899)),
                max(1, int(getattr(self.cfg, "SHOW_FPS", 30))),
                max(0, min(100, int(getattr(self.cfg, "SHOW_JPEG_QUALITY", 70)))),
                max(.05, float(getattr(self.cfg, "SHOW_STREAM_RETRY_S", 2.0))),
                max(.05, float(getattr(self.cfg, "SHOW_STREAM_RETRY_MAX_S", 15.0)))]
        script = _CHILD_SOURCE + '\n_run_sender(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), sys.argv[4], int(sys.argv[5]), float(sys.argv[6]), int(sys.argv[7]), float(sys.argv[8]), float(sys.argv[9]))\n'
        self.process = subprocess.Popen([sys.executable, "-c", script] + [str(x) for x in args],
                                        pass_fds=(fd,), stdin=subprocess.DEVNULL,
                                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def submit(self, frame):
        if self.closed or self.failed:
            return False
        array = np.asarray(frame)
        if (array.dtype != np.uint8 or array.ndim != 3 or array.shape[2] != 3
                or not array.shape[0] or not array.shape[1]
                or array.nbytes > self.capacity):
            self.dropped += 1
            return False
        if self.process is None:
            try:
                self._start()
            except Exception as exc:
                self.failed = True
                print("[preview] 图传进程不可用，保留本地显示: %s" % exc)
                return False
        if self.process.poll() is not None:
            self.failed = True
            print("[preview] 图传进程已退出，保留本地显示（请重新启动应用）")
            return False
        try:
            fcntl.flock(self.file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.dropped += 1
            return False
        try:
            self.sequence += 1
            self.mm[HEADER:HEADER + array.nbytes] = array.tobytes()
            FRAME.pack_into(self.mm, 0, self.sequence, array.shape[1], array.shape[0], array.nbytes)
        finally:
            fcntl.flock(self.file.fileno(), fcntl.LOCK_UN)
        return True

    def status(self):
        if self.closed:
            return {"state": "closed", "sent": 0, "dropped": self.dropped}
        if self.failed:
            return {"state": "unavailable", "sent": 0, "dropped": self.dropped}
        try:
            fcntl.flock(self.file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return dict(self._last_status, dropped=self.dropped)
        try:
            state, heartbeat, duration, sent, sequence = STATUS.unpack_from(self.mm, STATUS_OFFSET)
            self._last_status = {"state": self.STATES.get(state, "unsupported"), "sent": sent,
                                 "sequence": sequence, "duration_ms": duration * 1000.0,
                                 "heartbeat": heartbeat}
        finally:
            fcntl.flock(self.file.fileno(), fcntl.LOCK_UN)
        return dict(self._last_status, dropped=self.dropped)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.mm[124] = 1
        try:
            if self.process is not None:
                try:
                    self.process.wait(timeout=.8)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=.5)
        finally:
            self.mm.close()
            self.file.close()
