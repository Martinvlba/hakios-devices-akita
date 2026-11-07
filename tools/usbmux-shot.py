#!/usr/bin/env python3
"""A picture of what the Pixel 8a's panel shows while HakiOS runs, over the USB cable (usbmux=1): the phone's port 6,
where the kernel's usbmux (drivers/usb/hakios.akita_usbmux.c) sends what the display's DMA reads -- its own base,
stride and size, not what any driver meant to show -- as a binary PPM.

usage: usbmux-shot.py OUTPUT.png [SCALE]     (SCALE: keep one pixel of every SCALE across and down, default 1)
       the PPM is kept beside it (OUTPUT.ppm); the DMA's base, stride and size are printed
"""
import os
import struct
import sys
import zlib


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
        sys.exit('usbmux-shot: no device (is usbmuxd running, the phone attached?)')
    s.close()
    s = socket.socket(socket.AF_UNIX)
    s.connect('/var/run/usbmuxd')
    body = plistlib.dumps({'MessageType': 'Connect', 'DeviceID': devices[0]['DeviceID'], 'PortNumber': socket.htons(port)})
    s.sendall(struct.pack('<IIII', 16 + len(body), 1, 8, 2) + body)
    n = struct.unpack('<I', s.recv(16)[:4])[0]
    result = plistlib.loads(s.recv(n - 16))
    if result.get('Number') != 0:
        sys.exit(f'usbmux-shot: the phone refused port {port}: {result}')
    return s


def png(path, w, h, rgb):
    raw = b''.join(b'\0' + rgb[y * w * 3:(y + 1) * w * 3] for y in range(h))
    chunk = lambda t, d: struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)) +
                chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))


if len(sys.argv) < 2:
    sys.exit(__doc__)
out = sys.argv[1]
scale = int(sys.argv[2]) if len(sys.argv) > 2 else 1
s = mux_connect(6)
data = bytearray()
while chunk_ := s.recv(1 << 20):
    data += chunk_
if not data.startswith(b'P6'):
    sys.exit('usbmux-shot: ' + data.decode('latin1', 'replace').strip())
lines, at = [], 0
while len(lines) < 4:                       # P6, the comment, width height, 255
    end = data.index(b'\n', at)
    lines.append(data[at:end].decode())
    at = end + 1
print(lines[1].lstrip('# '))
w, h = map(int, lines[2].split())
pixels = bytes(data[at:at + w * h * 3])
if len(pixels) < w * h * 3:
    sys.exit(f'usbmux-shot: {len(pixels)} of {w * h * 3} bytes arrived')
with open(os.path.splitext(out)[0] + '.ppm', 'wb') as f:
    f.write(data)
if scale > 1:
    sw, sh = w // scale, h // scale
    pixels = b''.join(pixels[(y * scale * w + x * scale) * 3:(y * scale * w + x * scale) * 3 + 3]
                      for y in range(sh) for x in range(sw))
    w, h = sw, sh
png(out, w, h, pixels)
print(f'{out}: {w} x {h}')
