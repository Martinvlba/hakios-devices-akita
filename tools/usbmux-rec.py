#!/usr/bin/env python3
"""The Pixel 8a's screen recording, fetched over the USB cable while HakiOS runs (usbmux=1, boot-arg rec=N): the
phone's port 7, where the kernel's usbmux (drivers/usb/hakios.akita_usbmux.c) sends the ring of frames the framebuffer
keeps (xnu/files/iokit/Kernel/MiniIOMobileFramebuffer.cpp: one pixel of every N, each frame as the panel began to read
it, with its time) -- the recording pauses meanwhile and goes on after. Then, with OUT.mp4, tools/recording.py makes the
video (every frame held as long as it was on the panel).

usage: usbmux-rec.py OUT.rec [OUT.mp4 [recording.py's options]]
"""
import os
import struct
import subprocess
import sys


def mux_connect(port):
    # usbmux-log.py's, without running that script's main part
    import plistlib
    import socket
    s = socket.socket(socket.AF_UNIX)
    s.connect('/var/run/usbmuxd')
    b = plistlib.dumps({'MessageType': 'ListDevices'})
    s.sendall(struct.pack('<IIII', 16 + len(b), 1, 8, 1) + b)
    n = struct.unpack('<I', s.recv(16)[:4])[0]
    data = b''
    while len(data) < n - 16:
        data += s.recv(n - 16 - len(data))
    devices = plistlib.loads(data).get('DeviceList', [])
    if not devices:
        sys.exit('usbmux-rec: no device (is usbmuxd running, the phone attached?)')
    s.close()
    s = socket.socket(socket.AF_UNIX)
    s.connect('/var/run/usbmuxd')
    body = plistlib.dumps({'MessageType': 'Connect', 'DeviceID': devices[0]['DeviceID'], 'PortNumber': socket.htons(port)})
    s.sendall(struct.pack('<IIII', 16 + len(body), 1, 8, 2) + body)
    n = struct.unpack('<I', s.recv(16)[:4])[0]
    result = plistlib.loads(s.recv(n - 16))
    if result.get('Number') != 0:
        sys.exit(f'usbmux-rec: the phone refused port {port}: {result}')
    return s



if len(sys.argv) < 2:
    sys.exit(__doc__)
s = mux_connect(7)
data = bytearray()
while chunk_ := s.recv(1 << 20):
    data += chunk_
if not data.startswith(b'RWREC'):
    sys.exit('usbmux-rec: ' + data[:200].decode('latin1', 'replace').strip())
magic, width, height, frames, oldest = struct.unpack_from('<8sIIII', data, 0)
began = struct.unpack_from('<Q', data, 24)[0] / 1e9
with open(sys.argv[1], 'wb') as f:
    f.write(data)
print(f'{sys.argv[1]}: {frames} frames of {width} x {height}, the oldest at {oldest}, begun at {began:.3f} s of uptime, '
      f'{len(data) >> 20} MB')
if len(sys.argv) > 2:
    here = os.path.dirname(os.path.abspath(__file__))
    subprocess.run([sys.executable, os.path.join(here, '..', '..', '..', 'tools', 'recording.py'), sys.argv[1]] +
                   sys.argv[2:], check=False)
