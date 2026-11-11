#!/usr/bin/env python3
"""The phone's Mali shader cache fetched over the USB cable, for the next image's seed (TODO 21, 11-11).

usage: usbmux-pull-cache.py [--hours H] [OUTDIR]

While HakiOS runs userland/src/cache_send.c (a job: /redwood/bin/cache_send 120), connects through usbmuxd to the
phone's 127.0.0.1:31415 and writes every compiled program it is given (NNNN.prog) into OUTDIR (default
userland/hakios-metal-cache-mali, which git ignores: compiled code is not kept in the repository). userland/build.sh puts
that directory into the image at /redwood/share/hakios-metal-cache-mali; c_api_mali.cpp's cacheLoad looks there first.
Programs filed under another build of the compiler are simply never asked for: a stale seed costs room, nothing else.
Retries for a minute while the phone is not serving yet.

--hours H: only the programs the phone used (read or wrote) in the last H hours of its clock (default 6). The phone's
cache keeps programs of every earlier build of the compiler until its 64 MB trim, and those are never asked for again;
the image has room for a few MB. OUTDIR is emptied first.
"""
import os
import plistlib
import socket
import struct
import sys
import time

PORT = 31415


def mux_connect(port):
    s = socket.socket(socket.AF_UNIX)
    s.connect('/var/run/usbmuxd')
    b = plistlib.dumps({'MessageType': 'ListDevices'})
    s.sendall(struct.pack('<IIII', 16 + len(b), 1, 8, 1) + b)
    n = struct.unpack('<I', s.recv(16)[:4])[0]
    data = b''
    while len(data) < n - 16:
        data += s.recv(n - 16 - len(data))
    devices = plistlib.loads(data).get('DeviceList', [])
    s.close()
    if not devices:
        return None
    s = socket.socket(socket.AF_UNIX)
    s.connect('/var/run/usbmuxd')
    body = plistlib.dumps({'MessageType': 'Connect', 'DeviceID': devices[0]['DeviceID'], 'PortNumber': socket.htons(port)})
    s.sendall(struct.pack('<IIII', 16 + len(body), 1, 8, 2) + body)
    n = struct.unpack('<I', s.recv(16)[:4])[0]
    result = plistlib.loads(s.recv(n - 16))
    if result.get('Number') != 0:
        s.close()
        return None
    return s


def read_exact(s, n):
    data = bytearray()
    while len(data) < n:
        chunk = s.recv(min(1 << 20, n - len(data)))
        if not chunk:
            raise EOFError('the phone closed the connection early')
        data += chunk
    return bytes(data)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    args = sys.argv[1:]
    hours = 6.0
    if len(args) >= 2 and args[0] == '--hours':
        hours = float(args[1])
        args = args[2:]
    out = args[0] if args else os.path.join(here, '..', '..', '..', 'userland', 'hakios-metal-cache-mali')
    out = os.path.normpath(out)
    s = None
    for _ in range(60):
        s = mux_connect(PORT)
        if s is not None:
            break
        time.sleep(1)
    if s is None:
        sys.exit('usbmux-pull-cache: no answer on the phone\'s port %d (is cache_send running?)' % PORT)
    s.settimeout(60)
    if read_exact(s, 8) != b'HKMC0002':
        sys.exit('usbmux-pull-cache: not cache_send\'s greeting (an older cache_send?)')
    entries = []
    while True:
        (length,) = struct.unpack('<I', read_exact(s, 4))
        if length == 0:
            break
        name = read_exact(s, length).decode()
        size, mtime = struct.unpack('<QQ', read_exact(s, 16))
        data = read_exact(s, size)
        if '/' not in name and name.endswith('.prog'):
            entries.append((mtime, name, data))
    s.close()
    newest = max((e[0] for e in entries), default=0)
    keep = [e for e in entries if e[0] >= newest - hours * 3600]
    print('usbmux-pull-cache: %d programs on the phone (%.1f MB); %d used in the %.1f hours before its newest' %
          (len(entries), sum(len(e[2]) for e in entries) / 1e6, len(keep), hours))
    if os.path.isdir(out):
        for f in os.listdir(out):
            if f.endswith('.prog') or f.endswith('.tmp'):
                os.unlink(os.path.join(out, f))
    os.makedirs(out, exist_ok=True)
    files = total = 0
    for mtime, name, data in keep:
        size = len(data)
        tmp = os.path.join(out, name + '.tmp')
        with open(tmp, 'wb') as f:
            f.write(data)
        os.replace(tmp, os.path.join(out, name))
        files += 1
        total += size
    print('usbmux-pull-cache: %d programs, %.1f MB into %s' % (files, total / 1e6, out))


if __name__ == '__main__':
    main()
