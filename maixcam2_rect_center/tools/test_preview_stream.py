"""Real child process + independent local WebSocket bridge; no board SDK required."""
import base64
import hashlib
import os
import select
import socket
import struct
import threading
import time
import unittest
from unittest.mock import patch
from test_support import load_config
import cv2
import numpy as np
from vision.maixvision_stream import MaixVisionStream, packet, unpack_packet, fcntl


def exact(sock,n):
    data=bytearray()
    while len(data)<n:
        part=sock.recv(n-len(data))
        if not part: raise OSError('eof')
        data.extend(part)
    return bytes(data)


def receive(sock):
    a,b=exact(sock,2)
    n=b&127
    if n==126: n=struct.unpack('>H',exact(sock,2))[0]
    elif n==127: n=struct.unpack('>Q',exact(sock,8))[0]
    if not b&128: raise AssertionError('client frame must be masked')
    mask=exact(sock,4)
    data=exact(sock,n)
    return bytes(v^mask[i%4] for i,v in enumerate(data))


def send(sock,message):
    n=len(message)
    header=bytes((0x82,n)) if n<126 else b'\x82\x7e'+struct.pack('>H',n)
    sock.sendall(header+message)


def protocol(ident,payload=b''):
    # Independent encoder using the official C++ layout.
    data=bytearray([172,190,203,202])+bytearray((len(payload)+3).to_bytes(4,'little'))+bytearray([0,ident])+bytearray(payload)
    data.append(sum(data)%256)
    return bytes(data)


class Bridge:
    def __init__(self):
        self.server=socket.socket()
        self.server.bind(('127.0.0.1',0))
        self.server.listen()
        self.server.settimeout(.1)
        self.port=self.server.getsockname()[1]
        self.ready=threading.Event()
        self.stop=threading.Event()
        self.frames=[]
        self.acks=[]
        self.connections=0
        self.error=None
        self.thread=threading.Thread(target=self.run,daemon=True)
        self.thread.start()
    def run(self):
        try:
            while not self.stop.is_set():
                try: conn,_=self.server.accept()
                except socket.timeout: continue
                with conn:
                    conn.settimeout(2)
                    request=b''
                    while b'\r\n\r\n' not in request: request+=conn.recv(4096)
                    key=next(line.split(b':',1)[1].strip() for line in request.split(b'\r\n') if line.lower().startswith(b'sec-websocket-key:'))
                    accept=base64.b64encode(hashlib.sha1(key+b'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').digest())
                    conn.sendall(b'HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: '+accept+b'\r\n\r\n')
                    assert receive(conn)==protocol(1,b'code')
                    self.connections+=1
                    send(conn,protocol(2,b'\x01'))
                    send(conn,protocol(21,b'\x37'))
                    send(conn,protocol(14,b'\x02'))
                    self.ready.set()
                    while not self.stop.is_set():
                        if not select.select([conn],[],[],.05)[0]: continue
                        data=receive(conn)
                        assert data[:4]==b'\xac\xbe\xcb\xca' and data[-1]==sum(data[:-1])%256
                        if data[9] in (15,22): self.acks.append(data)
                        elif data[9]==6:
                            image=cv2.imdecode(np.frombuffer(data[11:-1],np.uint8),cv2.IMREAD_COLOR)
                            assert image is not None
                            self.frames.append(image)
                            if self.connections==1: break  # force reconnect after a real encoded frame
        except (OSError,AssertionError) as exc:
            if not self.stop.is_set(): self.error=exc
    def close(self):
        self.stop.set()
        self.thread.join(3)
        self.server.close()


class StreamTests(unittest.TestCase):
    def test_protocol_reference(self):
        self.assertEqual(packet(1,b'code'),protocol(1,b'code'))
        self.assertEqual(unpack_packet(protocol(2,b'\x01')),(2,b'\x01'))
        with self.assertRaises(ValueError): unpack_packet(packet(6,b'test')[:-1]+b'\x00')

    @unittest.skipIf(fcntl is None,'POSIX process/flock unavailable')
    def test_process_bridge_reconnect_and_negotiation(self):
        bridge=Bridge()
        cfg=load_config(SHOW_STREAM_PORT=bridge.port,SHOW_STREAM_WIDTH=40,SHOW_STREAM_RETRY_S=.02,SHOW_STREAM_RETRY_MAX_S=.05)
        stream=MaixVisionStream(cfg,80,60)
        try:
            self.assertTrue(stream.enabled)
            self.assertTrue(bridge.ready.wait(5))
            frame=np.full((60,80,3),(20,90,180),np.uint8)
            deadline=time.monotonic()+5
            while (len(bridge.frames)<2 or bridge.connections<2 or protocol(15,b'\x01\x02') not in bridge.acks) and time.monotonic()<deadline:
                started=time.perf_counter()
                stream.submit(frame)
                self.assertLess(time.perf_counter()-started,.10)
                time.sleep(.02)
            self.assertIsNone(bridge.error)
            self.assertGreaterEqual(bridge.connections,2)
            self.assertGreaterEqual(len(bridge.frames),2)
            self.assertEqual(bridge.frames[-1].shape,(30,40,3))
            self.assertIn(protocol(22,b'\x01\x37'),bridge.acks)
            self.assertIn(protocol(15,b'\x01\x02'),bridge.acks)
            process=stream.process
        finally:
            stream.close(); bridge.close()
        self.assertIsNotNone(process.poll())

    @unittest.skipIf(fcntl is None,'POSIX process/flock unavailable')
    def test_congested_slot_drops_and_child_has_no_maix(self):
        stream=MaixVisionStream(load_config(SHOW_STREAM_PORT=1,SHOW_STREAM_RETRY_S=.05),80,60)
        try:
            independent=os.open('/proc/self/fd/%d'%stream.file.fileno(),os.O_RDWR)
            fcntl.flock(independent,fcntl.LOCK_EX)
            self.assertFalse(stream.submit(np.zeros((60,80,3),np.uint8)))
            fcntl.flock(independent,fcntl.LOCK_UN); os.close(independent)
            self.assertEqual(stream.dropped,1)
            self.assertNotIn('from maix ',stream.process.args[2])
            self.assertNotIn('import maix\n',stream.process.args[2])
        finally: stream.close()

    @unittest.skipIf(fcntl is None,'POSIX process/flock unavailable')
    def test_single_file_process_uses_embedded_source(self):
        import runpy
        from test_support import ROOT
        runpy.run_path(str(ROOT/'dist'/'main_single.py'),run_name='_bundle_stream_test_')
        from vision.maixvision_stream import MaixVisionStream as EmbeddedStream
        stream=EmbeddedStream(load_config(SHOW_STREAM_PORT=1),80,60)
        try:
            self.assertTrue(stream.enabled)
            self.assertIn('def _child(',stream.process.args[2])
            self.assertTrue(stream.submit(np.zeros((60,80,3),np.uint8)))
        finally: stream.close()


if __name__ == '__main__': unittest.main()
