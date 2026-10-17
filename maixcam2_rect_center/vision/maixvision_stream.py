"""MaixVision protocol-0 CLIENT; JPEG/WebSocket run in a fresh process, no maix.

Protocol source: Sipeed/MaixCDK components/vision/src/maix_image_trans.cpp.
The parent only copies into one mmap slot under a nonblocking flock. The child
reopens the inherited fd: flock on a shared open-file-description cannot exclude
its parent, even though it is in another process.
"""
import base64
import hashlib
import mmap
import os
import select
import socket
import struct
import subprocess
import sys
import tempfile
import time
try:
    import fcntl
except ImportError:  # Windows PC tools can import the application without POSIX I/O.
    fcntl = None

HEADER = 128
MAGIC = b'\xac\xbe\xcb\xca'
MAX_MESSAGE = 4*1024*1024


def packet(message, payload=b''):
    frame = MAGIC+struct.pack('<I', len(payload)+3)+bytes((0, message))+payload
    return frame+bytes((sum(frame)&255,))


def unpack_packet(frame):
    if len(frame) < 11 or frame[:4] != MAGIC or struct.unpack_from('<I', frame, 4)[0] != len(frame)-8 or frame[8] != 0 or sum(frame[:-1])&255 != frame[-1]:
        raise ValueError('invalid protocol-0 packet')
    return frame[9], frame[10:-1]


class WebSocketClient:
    def __init__(self, host, port):
        self.socket = socket.create_connection((host, port), timeout=.5)
        self.buffer = bytearray()
        self.fragments = bytearray()
        self.fragment_opcode = None
        try:
            key = base64.b64encode(os.urandom(16)).decode('ascii')
            request = ('GET / HTTP/1.1\r\nHost: %s:%d\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n' % (host, port, key))
            self.socket.sendall(request.encode('ascii'))
            while b'\r\n\r\n' not in self.buffer:
                data = self.socket.recv(4096)
                if not data or len(self.buffer) > 16384:
                    raise OSError('invalid WebSocket handshake')
                self.buffer.extend(data)
            headers, rest = bytes(self.buffer).split(b'\r\n\r\n', 1)
            self.buffer = bytearray(rest)
            lines = headers.decode('ascii').split('\r\n')
            fields = {line.split(':',1)[0].lower(): line.split(':',1)[1] for line in lines[1:] if ':' in line}
            expected = base64.b64encode(hashlib.sha1((key+'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()).decode()
            if ' 101 ' not in lines[0] or fields.get('sec-websocket-accept', '').strip() != expected:
                raise OSError('WebSocket upgrade rejected')
        except BaseException:
            self.close()
            raise

    def send(self, payload, opcode=2):
        mask = os.urandom(4)
        size = len(payload)
        head = bytes((0x80|opcode, 0x80|size)) if size < 126 else (bytes((0x80|opcode, 0x80|126))+struct.pack('>H', size) if size < 65536 else bytes((0x80|opcode, 0x80|127))+struct.pack('>Q', size))
        encoded = bytearray(payload)
        for i in range(4):
            encoded[i::4] = bytes(v^mask[i] for v in encoded[i::4])
        self.socket.sendall(head+mask+encoded)

    def receive(self, timeout=.01):
        if select.select([self.socket], [], [], timeout)[0]:
            data = self.socket.recv(65536)
            if not data:
                raise OSError('WebSocket closed')
            self.buffer.extend(data)
        messages = []
        while len(self.buffer) >= 2:
            b0, b1 = self.buffer[:2]
            opcode, size, offset = b0&15, b1&127, 2
            if b0 & 0x70 or b1 & 0x80:
                raise ValueError('unsupported WebSocket frame')
            if size in (126, 127):
                count = 2 if size == 126 else 8
                if len(self.buffer) < offset+count:
                    break
                size = int.from_bytes(self.buffer[offset:offset+count], 'big')
                offset += count
            if size > MAX_MESSAGE:
                raise ValueError('WebSocket message too large')
            if len(self.buffer) < offset+size:
                break
            payload = bytes(self.buffer[offset:offset+size])
            del self.buffer[:offset+size]
            if opcode == 8:
                raise OSError('WebSocket close')
            if opcode == 9:
                self.send(payload, 10)
                continue
            if opcode == 10:
                continue
            if opcode in (1, 2):
                if self.fragment_opcode is not None:
                    raise ValueError('interleaved WebSocket message')
                self.fragment_opcode = opcode
            elif opcode != 0 or self.fragment_opcode is None:
                raise ValueError('invalid continuation')
            self.fragments.extend(payload)
            if len(self.fragments) > MAX_MESSAGE:
                raise ValueError('WebSocket fragments too large')
            if b0 & 0x80:
                if self.fragment_opcode == 2:
                    messages.append(bytes(self.fragments))
                self.fragments.clear()
                self.fragment_opcode = None
        return messages

    def close(self):
        self.socket.close()


class MaixVisionStream:
    def __init__(self, cfg, width, height, clock=None):
        self.cfg, self.width, self.height = cfg, int(width), int(height)
        self.clock = clock if clock is not None else time
        self.process = self.mapping = self.file = None
        self.sequence = self.dropped = 0
        self.enabled = False
        if fcntl is None:
            print('[stream] POSIX flock unavailable; local display only')
            return
        try:
            self.file = tempfile.TemporaryFile()
            size = HEADER+self.width*self.height*3
            self.file.truncate(size)
            self.mapping = mmap.mmap(self.file.fileno(), size)
            # __source__ is supplied by build_single for virtual modules.
            source = getattr(sys.modules.get(__name__), '__source__', None)
            if source is None:
                with open(__file__, encoding='utf-8') as handle:
                    source = handle.read()
            command = source+'\n_child(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), sys.argv[4], int(sys.argv[5]), int(sys.argv[6]), int(sys.argv[7]), float(sys.argv[8]), float(sys.argv[9]))\n'
            self.process = subprocess.Popen([sys.executable, '-c', command, str(self.file.fileno()), str(self.width), str(self.height),
                getattr(cfg, 'SHOW_STREAM_HOST', '127.0.0.1'), str(getattr(cfg, 'SHOW_STREAM_PORT', 7899)),
                str(getattr(cfg, 'SHOW_STREAM_WIDTH', 480)), str(getattr(cfg, 'SHOW_JPEG_QUALITY', 70)),
                str(getattr(cfg, 'SHOW_STREAM_RETRY_S', 2.)), str(getattr(cfg, 'SHOW_STREAM_RETRY_MAX_S', 15.))],
                pass_fds=(self.file.fileno(),))
            self.enabled = True
        except Exception as exc:
            print('[stream] disabled: %s' % exc)
            self.close()

    def submit(self, canvas):
        if not self.enabled or self.process.poll() is not None:
            return False
        if canvas.shape != (self.height, self.width, 3) or str(canvas.dtype) != 'uint8':
            raise ValueError('stream frame shape/dtype mismatch')
        try:
            fcntl.flock(self.file.fileno(), fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            self.dropped += 1
            return False
        try:
            self.sequence += 1
            self.mapping[HEADER:] = canvas.tobytes()
            struct.pack_into('<QII', self.mapping, 0, self.sequence, self.width, self.height)
        finally:
            fcntl.flock(self.file.fileno(), fcntl.LOCK_UN)
        return True

    def close(self):
        self.enabled = False
        if self.process is not None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1.)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1.)
            self.process = None
        if self.mapping is not None:
            self.mapping.close()
            self.mapping = None
        if self.file is not None:
            self.file.close()
            self.file = None


def _child(inherited_fd, width, height, host, port, stream_width, quality, retry, retry_max):
    import cv2
    import numpy as np
    cv2.setNumThreads(1)
    fd = os.open('/proc/self/fd/%d' % inherited_fd, os.O_RDWR)
    os.close(inherited_fd)
    memory = mmap.mmap(fd, HEADER+width*height*3)
    last = 0
    delay = retry
    while True:
        client = None
        try:
            client = WebSocketClient(host, port)
            client.send(packet(1, b'code'))
            accepted, fmt, deadline = False, 1, time.monotonic()+10
            while True:
                for message in client.receive(.01):
                    ident, payload = unpack_packet(message)
                    if ident == 2:
                        if payload != b'\x01':
                            raise OSError('MaixVision registration rejected')
                        accepted, delay = True, retry
                    elif ident == 14 and len(payload) == 1:
                        value = payload[0]
                        if value <= 2:
                            fmt = value
                        client.send(packet(15, bytes((int(value <= 2), value))))
                    elif ident == 21 and len(payload) == 1:
                        quality = max(1, min(100, payload[0]))
                        client.send(packet(22, bytes((1, quality))))
                if not accepted:
                    if time.monotonic() > deadline:
                        raise OSError('MaixVision registration timeout')
                    continue
                if fmt == 0:
                    continue
                try:
                    fcntl.flock(fd, fcntl.LOCK_EX|fcntl.LOCK_NB)
                except BlockingIOError:
                    continue
                try:
                    sequence, w, h = struct.unpack_from('<QII', memory, 0)
                    raw = bytes(memory[HEADER:]) if sequence != last and sequence and (w,h)==(width,height) else None
                finally:
                    fcntl.flock(fd, fcntl.LOCK_UN)
                if raw is None:
                    continue
                last = sequence
                frame = np.frombuffer(raw, np.uint8).reshape(height, width, 3)
                if 0 < stream_width < width:
                    frame = cv2.resize(frame, (stream_width, max(1, round(height*stream_width/width))), interpolation=cv2.INTER_AREA)
                ok, encoded = cv2.imencode('.jpg' if fmt == 1 else '.png', frame,
                                          [cv2.IMWRITE_JPEG_QUALITY, quality] if fmt == 1 else [])
                if ok:
                    client.send(packet(6, bytes((fmt,))+encoded.tobytes()))
        except (OSError, ValueError) as exc:
            print('[stream] reconnect: %s' % exc, flush=True)
        finally:
            if client is not None:
                client.close()
        time.sleep(delay)
        delay = min(retry_max, max(.05, delay*2))
