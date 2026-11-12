#!/usr/bin/env python3
"""The Pixel 8a's sound recording, fetched over the USB cable while HakiOS runs (usbmux=1, boot-arg rec_sound=N; with
rec_sound_mic=1 what the phone's own two microphones heard -- the AoC's aocmic=S, mic 0 left, mic 1 right): the
phone's port 8, where the kernel's usbmux (drivers/usb/hakios.akita_usbmux.c) sends drivers/audio/hakios.audio.c's
recording as it lies ("RWSND001" and its blocks). Then, with OUT.wav, tools/sound.py makes the WAV.
usage: usbmux-sound.py OUT.snd [OUT.wav]
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
        sys.exit('usbmux-sound: no device (is usbmuxd running, the phone attached?)')
    s.close()
    s = socket.socket(socket.AF_UNIX)
    s.connect('/var/run/usbmuxd')
    body = plistlib.dumps({'MessageType': 'Connect', 'DeviceID': devices[0]['DeviceID'], 'PortNumber': socket.htons(port)})
    s.sendall(struct.pack('<IIII', 16 + len(body), 1, 8, 2) + body)
    n = struct.unpack('<I', s.recv(16)[:4])[0]
    result = plistlib.loads(s.recv(n - 16))
    if result.get('Number') != 0:
        sys.exit(f'usbmux-sound: the phone refused port {port}: {result}')
    return s



if len(sys.argv) < 2:
    sys.exit(__doc__)
s = mux_connect(8)
data = bytearray()
while chunk_ := s.recv(1 << 20):
    data += chunk_
if not data.startswith(b'RWSND'):
    sys.exit('usbmux-sound: ' + data[:200].decode('latin1', 'replace').strip())
magic, rate, channels, blocks = struct.unpack_from('<8sIII', data, 0)
with open(sys.argv[1], 'wb') as f:
    f.write(data)
print(f'{sys.argv[1]}: {blocks} blocks at {rate} Hz, {channels} channels, {len(data) >> 10} KB')
if len(sys.argv) > 2:
    here = os.path.dirname(os.path.abspath(__file__))
    subprocess.run([sys.executable, os.path.join(here, '..', '..', '..', 'tools', 'sound.py'), sys.argv[1], sys.argv[2]],
                   check=False)
