"""Real subprocess / loopback bridge regression; no Maix SDK or board hardware.

Wire expectations follow Sipeed/MaixCDK 30f4b8b7 maix_image_trans.cpp,
including little-endian lengths, protocol-0/checksum, 'code' sender handshake.
"""
import base64
import fcntl
import hashlib
import os
import socket
import struct
import sys
import threading
import time
import types
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from vision.maixvision_stream import (MaixVisionStream, ProtocolError, protocol_frame,
                                    protocol_parse, HEADER, FRAME, _CHILD_SOURCE)
from vision.preview import Preview, apply_maixvision_stream_policy


def wait_for(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError("timed out waiting for bridge/stream")


def wire(message, payload=b"", version=0):
    # Separate literal fixture builder, intentionally independent of production.
    value = b"\xac\xbe\xcb\xca" + struct.pack("<I", len(payload) + 3) + bytes((version, message)) + payload
    return value + bytes((sum(value) % 256,))


class Bridge:
    """Tiny independent RFC6455 server implementing official image bridge messages."""
    def __init__(self, ack_delay=0, wrong_version=False, bad_upgrade=False):
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(4)
        self.listener.settimeout(.05)
        self.port = self.listener.getsockname()[1]
        self.ack_delay = ack_delay
        self.wrong_version = wrong_version
        self.bad_upgrade = bad_upgrade
        self.stopping = False
        self.connections, self.images, self.controls, self.clients, self.errors = [], [], [], [], []
        self.next_control = []
        self.next_raw = []
        self.drop_next = False
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    @staticmethod
    def read(sock, n):
        data = bytearray()
        while len(data) < n:
            part = sock.recv(n - len(data))
            if not part:
                raise ConnectionError("client closed")
            data.extend(part)
        return bytes(data)

    def receive(self, sock):
        a, b = self.read(sock, 2)
        assert a == 0x82 and b & 0x80, "client messages must be FIN binary + masked"
        n = b & 127
        if n in (126, 127):
            n = struct.unpack("!H" if n == 126 else "!Q", self.read(sock, 2 if n == 126 else 8))[0]
        assert n <= 2 * 1024 * 1024
        mask, body = self.read(sock, 4), self.read(sock, n)
        return bytes(value ^ mask[i % 4] for i, value in enumerate(body))

    @staticmethod
    def send(sock, payload, opcode=2):
        n = len(payload)
        head = bytes((0x80 | opcode, n if n < 126 else 126))
        if n >= 126:
            head += struct.pack("!H", n)
        sock.sendall(head + payload)

    def handle(self, sock):
        try:
            sock.settimeout(2)
            request = bytearray()
            while b"\r\n\r\n" not in request:
                request.extend(sock.recv(1024))
                assert len(request) < 4096
            headers = dict(line.split(": ", 1) for line in request.decode("ascii").split("\r\n")[1:] if ": " in line)
            key = headers["Sec-WebSocket-Key"]
            accept = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
            if self.bad_upgrade:
                sock.sendall(b"HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n")
                return
            sock.sendall(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: %s\r\n\r\n" % accept).encode())
            assert self.receive(sock) == wire(1, b"code"), "official code sender handshake mismatch"
            if self.ack_delay:
                time.sleep(self.ack_delay)
            self.send(sock, wire(2, b"\x01", version=1 if self.wrong_version else 0))
            if self.wrong_version:
                return
            sock.settimeout(.05)
            while not self.stopping:
                if self.drop_next:
                    self.drop_next = False
                    return
                if self.next_control:
                    self.send(sock, self.next_control.pop(0))
                if self.next_raw:
                    sock.sendall(self.next_raw.pop(0))
                try:
                    raw = self.receive(sock)
                except socket.timeout:
                    continue
                assert raw[:4] == b"\xac\xbe\xcb\xca" and raw[8] == 0
                assert struct.unpack_from("<I", raw, 4)[0] == len(raw) - 8
                assert raw[-1] == sum(raw[:-1]) % 256
                if raw[9] == 6:
                    assert raw[10] in (1, 2)
                    decoded = cv2.imdecode(np.frombuffer(raw[11:-1], np.uint8), cv2.IMREAD_COLOR)
                    assert decoded is not None
                    self.images.append((time.monotonic(), raw[10], decoded))
                else:
                    self.controls.append(raw)
        except (ConnectionError, BrokenPipeError, ConnectionResetError, OSError):
            pass
        except Exception as exc:
            self.errors.append(str(exc))
        finally:
            sock.close()

    def loop(self):
        while not self.stopping:
            try:
                sock, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self.connections.append(time.monotonic())
            self.clients.append(sock)
            threading.Thread(target=self.handle, args=(sock,), daemon=True).start()

    def close(self):
        self.stopping = True
        self.listener.close()
        for sock in self.clients:
            try:
                sock.close()
            except OSError:
                pass
        self.thread.join(1)
        assert not self.errors, self.errors


def config(port, **values):
    defaults = dict(SHOW=True, SHOW_ASYNC=True, SHOW_FPS=30, SHOW_JPEG_QUALITY=70,
                    SHOW_STREAM_HOST="127.0.0.1", SHOW_STREAM_PORT=port,
                    SHOW_STREAM_RETRY_S=.08, SHOW_STREAM_RETRY_MAX_S=.2,
                    SHOW_STREAM_WIDTH=64, DISABLE_MAIXVISION_STREAM=False)
    defaults.update(values)
    return types.SimpleNamespace(**defaults)


def frame(value=90):
    return np.full((48, 64, 3), value, np.uint8)


def test_wire_contract_and_invalid_messages():
    assert protocol_frame(1, b"code") == wire(1, b"code")
    assert protocol_frame(6, b"\x01\xff\xd8\xff\xd9") == wire(6, b"\x01\xff\xd8\xff\xd9")
    assert protocol_parse(wire(2, b"\x01")) == (2, b"\x01")
    for invalid in (wire(2, b"\x01", version=1), wire(2, b"\x01")[:-1] + b"\x00",
                    b"other", wire(2, b"\x01") + b"extra"):
        try:
            protocol_parse(invalid)
        except ProtocolError:
            continue
        raise AssertionError("accepted unsupported/corrupt message")
    assert "import maix" not in _CHILD_SOURCE


def test_connected_protocol_controls_and_cleanup():
    bridge = Bridge()
    sender = MaixVisionStream(config(bridge.port), 64, 48)
    try:
        assert sender.submit(frame())
        wait_for(lambda: len(bridge.images) >= 1)
        assert sender.process.pid != os.getpid() and sender.status()["state"] == "ready"
        assert bridge.images[0][1] == 1
        assert bridge.images[0][2].shape == (48, 64, 3)
        bridge.next_control.append(wire(14, b"\x00"))
        wait_for(lambda: wire(15, b"\x01\x00") in bridge.controls)
        wait_for(lambda: sender.status()["state"] == "paused")
        previous = len(bridge.images)
        sender.submit(frame(180))
        time.sleep(.09)
        assert len(bridge.images) == previous, "pause control must stop image sends"
        bridge.next_control.extend([wire(21, b"\x64"), wire(14, b"\x02")])
        wait_for(lambda: wire(22, b"\x01\x64") in bridge.controls)
        wait_for(lambda: len(bridge.images) > previous)
        assert bridge.images[-1][1] == 2 and int(bridge.images[-1][2].mean()) == 180
    finally:
        sender.close()
        bridge.close()
    assert sender.process.poll() is not None and sender.file.closed and sender.mm.closed
    sender.close()


def test_delayed_500ms_startup_latest_slot_nonblocking():
    bridge = Bridge(ack_delay=.5)
    sender = MaixVisionStream(config(bridge.port), 64, 48)
    try:
        sender.submit(frame(10))
        wait_for(lambda: bridge.connections)
        started = time.monotonic()
        for value in range(20, 221):
            assert sender.submit(frame(value))
        elapsed = time.monotonic() - started
        assert elapsed < .15, "500ms bridge handshake blocked producer: %.3fs" % elapsed
        assert sender.file.seek(0, os.SEEK_END) == HEADER + sender.capacity
        wait_for(lambda: bridge.images)
        assert abs(float(bridge.images[0][2].mean()) - 220) <= 2, "must send newest image, never backlog"
        assert sender.sequence == 202
    finally:
        sender.close()
        bridge.close()


def test_offline_backoff_disconnect_and_recovery():
    bridge = Bridge(wrong_version=True)
    sender = MaixVisionStream(config(bridge.port), 64, 48)
    try:
        sender.submit(frame())
        wait_for(lambda: sender.status()["state"] == "unsupported")
        wait_for(lambda: len(bridge.connections) >= 3)
        intervals = np.diff(bridge.connections[:3])
        assert intervals[0] >= .07 and intervals[1] >= .15, intervals
        bridge.wrong_version = False
        wait_for(lambda: bridge.images)
        bridge.drop_next = True
        count = len(bridge.connections)
        wait_for(lambda: sender.status()["state"] == "retrying")
        sender.submit(frame(140))
        wait_for(lambda: len(bridge.connections) > count)
        wait_for(lambda: len(bridge.images) >= 2)
        assert abs(float(bridge.images[-1][2].mean()) - 140) <= 2
    finally:
        sender.close()
        bridge.close()


def test_unavailable_bridge_and_lock_contention_bounded():
    reserved = socket.socket()
    reserved.bind(("127.0.0.1", 0))
    port = reserved.getsockname()[1]  # Bound but not listening: deterministic offline.
    sender = MaixVisionStream(config(port), 64, 48)
    other = None
    try:
        sender.submit(frame())
        wait_for(lambda: sender.status()["state"] == "retrying")
        other = os.open("/proc/self/fd/%d" % sender.file.fileno(), os.O_RDWR)
        fcntl.flock(other, fcntl.LOCK_EX | fcntl.LOCK_NB)
        started = time.monotonic()
        assert not sender.submit(frame(5))
        assert time.monotonic() - started < .03 and sender.dropped == 1
        fcntl.flock(other, fcntl.LOCK_UN)
        assert not sender.submit(np.zeros((100, 100, 3), np.uint8))
        assert sender.dropped == 2
        assert sender.submit(frame(200))
        sequence, width, height, n = FRAME.unpack_from(sender.mm)
        assert (sequence, width, height, n) == (2, 64, 48, 64 * 48 * 3)
    finally:
        if other is not None:
            os.close(other)
        sender.close()
        reserved.close()


def test_preview_session_policy_local_screen_and_async():
    bridge = Bridge(ack_delay=.5)
    env_seen, shows = [], []
    class Screen:
        def __init__(self): env_seen.append(os.environ.get("MAIXVISION"))
        def width(self): return 64
        def height(self): return 48
        def show(self, image): shows.append(image.copy())
        def close(self): pass
    maix = types.ModuleType("maix")
    maix.display = types.SimpleNamespace(Display=Screen)
    maix.image = types.SimpleNamespace(cv2image=lambda image, **kwargs: image.copy())
    preview = None
    try:
        with patch.dict(sys.modules, maix=maix), patch.dict(os.environ, MAIXVISION="1"):
            cfg = config(bridge.port)
            assert not apply_maixvision_stream_policy(cfg)
            assert os.environ["MAIXVISION"] == "1"
            preview = Preview(cfg)
            assert env_seen == ["0"] and os.environ["MAIXVISION"] == "1"
            assert preview.async_show and preview.stream is not None and preview.probe_ms < 0
            started = time.monotonic()
            for i in range(30):
                now = time.monotonic() + i / 30
                if preview.due(now):
                    preview.render(frame(i), now)
            assert time.monotonic() - started < .15 and preview._pool <= 2
            wait_for(lambda: shows)
            wait_for(lambda: bridge.images)
            preview.close()
            local = config(bridge.port, DISABLE_MAIXVISION_STREAM=True)
            assert apply_maixvision_stream_policy(local)
            assert os.environ["MAIXVISION"] == "0"
            only_screen = Preview(local)
            assert only_screen.stream is None
            only_screen.close()
    finally:
        if preview is not None:
            preview.close()
        bridge.close()


def test_failed_display_constructor_restores_environment():
    def broken():
        assert os.environ["MAIXVISION"] == "0"
        raise RuntimeError("no screen")
    maix = types.ModuleType("maix")
    maix.display = types.SimpleNamespace(Display=broken)
    with patch.dict(sys.modules, maix=maix), patch.dict(os.environ, MAIXVISION="1"):
        preview = Preview(config(7899))
        assert not preview.enabled and preview.stream is None
        assert os.environ["MAIXVISION"] == "1"
        preview.close()


def test_child_lock_is_independent_of_inherited_file_description():
    bridge = Bridge()
    sender = MaixVisionStream(config(bridge.port), 64, 48)
    fd = sender.file.fileno()
    locked = False
    try:
        # Hold the producer's own FD lock while a genuine child starts. Inherited
        # descriptions share locks: without reopening, the child incorrectly
        # reads a frame which the producer is still writing.
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        locked = True
        value = frame(170)
        sender.mm[HEADER:HEADER + value.nbytes] = value.tobytes()
        FRAME.pack_into(sender.mm, 0, 1, 64, 48, value.nbytes)
        sender._start()
        wait_for(lambda: bridge.connections)
        time.sleep(.15)
        assert not bridge.images, "child read frame while producer owned its lock"
        fcntl.flock(fd, fcntl.LOCK_UN)
        locked = False
        wait_for(lambda: bridge.images)
        assert abs(float(bridge.images[0][2].mean()) - 170) <= 2
    finally:
        if locked:
            fcntl.flock(fd, fcntl.LOCK_UN)
        sender.close()
        bridge.close()


def test_corrupt_controls_and_oversized_ws_frames_reconnect():
    bridge = Bridge()
    sender = MaixVisionStream(config(bridge.port), 64, 48)
    try:
        sender.submit(frame())
        wait_for(lambda: bridge.images)
        corrupt = wire(21, b"\x50")[:-1] + b"\x00"
        count = len(bridge.connections)
        bridge.next_control.append(corrupt)
        wait_for(lambda: sender.status()["state"] == "unsupported")
        wait_for(lambda: len(bridge.connections) > count)
        wait_for(lambda: sender.status()["state"] == "ready")
        count = len(bridge.connections)
        # Announce 65537 bytes but send no body. The child must reject the
        # declared length immediately without allocating/waiting for that body.
        bridge.next_raw.append(b"\x82\x7f" + struct.pack("!Q", 65537))
        wait_for(lambda: sender.status()["state"] == "unsupported")
        wait_for(lambda: len(bridge.connections) > count)
        wait_for(lambda: sender.status()["state"] == "ready")
        assert sender.submit(frame(155))
        wait_for(lambda: len(bridge.images) >= 2)
        assert abs(float(bridge.images[-1][2].mean()) - 155) <= 2
    finally:
        sender.close()
        bridge.close()


def test_normal_show_preserves_session_and_closed_screen_reopens_locally():
    observed = []

    class Screen:
        def __init__(self):
            self.opened = True
            observed.append(("construct", os.environ.get("MAIXVISION")))

        def width(self): return 64
        def height(self): return 48
        def is_opened(self): return self.opened

        def open(self):
            observed.append(("open", os.environ.get("MAIXVISION")))
            self.opened = True

        def show(self, image):
            assert self.opened
            observed.append(("show", os.environ.get("MAIXVISION")))

        def close(self): self.opened = False

    maix = types.ModuleType("maix")
    maix.display = types.SimpleNamespace(Display=Screen)
    maix.image = types.SimpleNamespace(cv2image=lambda image, **kwargs: image.copy())
    # Disable the separate transport constructor only to examine native calls.
    with patch.dict(sys.modules, maix=maix), patch.dict(os.environ, MAIXVISION="1"), \
            patch("vision.maixvision_stream.MaixVisionStream", side_effect=RuntimeError("test isolation")):
        preview = Preview(config(7899, SHOW_ASYNC=False))
        try:
            preview.render(frame(), 1.0)
            preview.display.opened = False
            preview.render(frame(), 2.0)
            assert observed == [("construct", "0"), ("show", "1"),
                                ("open", "0"), ("show", "1")]
            assert os.environ["MAIXVISION"] == "1"
        finally:
            preview.close()


def test_failed_local_reopen_never_autoopens_under_maixvision_session():
    observed = []

    class Screen:
        def __init__(self): self.opened = True
        def width(self): return 64
        def height(self): return 48
        def is_opened(self): return self.opened

        def open(self):
            observed.append(("open", os.environ.get("MAIXVISION")))
            return 1  # SDK-style error return; stays closed, does not throw.

        def show(self, image):
            raise AssertionError("failed local open must not reach native show")

        def close(self): pass

    maix = types.ModuleType("maix")
    maix.display = types.SimpleNamespace(Display=Screen)
    maix.image = types.SimpleNamespace(cv2image=lambda image, **kwargs: image.copy())
    with patch.dict(sys.modules, maix=maix), patch.dict(os.environ, MAIXVISION="1"), \
            patch("vision.maixvision_stream.MaixVisionStream", side_effect=RuntimeError("test isolation")):
        preview = Preview(config(7899, SHOW_ASYNC=False))
        try:
            preview.display.opened = False
            preview.render(frame(), 1.0)
            assert observed == [("open", "0")]
            assert not preview.enabled and preview.shown == 0
            assert os.environ["MAIXVISION"] == "1"
        finally:
            preview.close()


def test_canvas_pool_bounded_even_when_caller_skips_due():
    release, entered = threading.Event(), threading.Event()

    class Screen:
        def width(self): return 64
        def height(self): return 48

        def show(self, image):
            entered.set()
            assert release.wait(2)

        def close(self): pass

    maix = types.ModuleType("maix")
    maix.display = types.SimpleNamespace(Display=Screen)
    maix.image = types.SimpleNamespace(cv2image=lambda image, **kwargs: image.copy())
    with patch.dict(sys.modules, maix=maix), patch.dict(os.environ, MAIXVISION="0"):
        preview = Preview(config(7899))
        try:
            preview.render(frame(), time.monotonic())
            assert entered.wait(1)
            for i in range(100):
                preview.render(frame(i), time.monotonic())
            assert preview._pool == 2 and preview.dropped >= 99
        finally:
            release.set()
            preview.close()
    assert not preview.thread.is_alive()


if __name__ == "__main__":
    tests = [value for name, value in list(globals().items()) if name.startswith("test_")]
    for test in tests:
        test()
        print("PASS", test.__name__)
    print("PASS %d stream/preview regressions" % len(tests))
