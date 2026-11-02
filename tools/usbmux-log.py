#!/usr/bin/env python3
"""The Pixel 8a's kernel log while HakiOS runs, over the USB cable (usbmux=1): through usbmuxd's socket to the phone's
port 1, where the kernel's usbmux (drivers/usb/hakios.akita_usbmux.c) serves its message buffer -- all of it, then
what is added, until this stops.

usage: usbmux-log.py [-k] [OUTPUT]      (stdout, or OUTPUT as well; needs usbmuxd running, and access to its socket)
       -k: the kernel message buffer (port 5: printf and IOLog, which without -v are not on the console)
"""
import plistlib
import socket
import struct
import sys


def mux_connect(port):
    s = socket.socket(socket.AF_UNIX)
    s.connect('/var/run/usbmuxd')
    s.sendall(struct.pack('<IIII', 16 + len(b := plistlib.dumps({'MessageType': 'ListDevices'})), 1, 8, 1) + b)
    n = struct.unpack('<I', s.recv(16)[:4])[0]
    data = b''
    while len(data) < n - 16:
        data += s.recv(n - 16 - len(data))
    devices = plistlib.loads(data).get('DeviceList', [])
    if not devices:
        sys.exit('usbmux-log: no device (is usbmuxd running, the phone attached?)')
    device = devices[0]['DeviceID']
    s.close()
    s = socket.socket(socket.AF_UNIX)
    s.connect('/var/run/usbmuxd')
    body = plistlib.dumps({'MessageType': 'Connect', 'DeviceID': device, 'PortNumber': socket.htons(port)})
    s.sendall(struct.pack('<IIII', 16 + len(body), 1, 8, 2) + body)
    n = struct.unpack('<I', s.recv(16)[:4])[0]
    result = plistlib.loads(s.recv(n - 16))
    if result.get('Number') != 0:
        sys.exit(f'usbmux-log: the phone refused port {port}: {result}')
    return s


args = sys.argv[1:]
port = 5 if args[:1] == ['-k'] else 1
args = args[1:] if port == 5 else args
out = open(args[0], 'ab') if args else None
s = mux_connect(port)
try:
    while data := s.recv(65536):
        sys.stdout.buffer.write(data)
        sys.stdout.buffer.flush()
        if out:
            out.write(data)
            out.flush()
except KeyboardInterrupt:
    pass
