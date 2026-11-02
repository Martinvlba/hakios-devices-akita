#!/usr/bin/env python3
"""Restart the Pixel 8a while HakiOS runs (usbmux=1), over the USB cable: a connection to the phone's port 2 (into the
bootloader, as `adb reboot bootloader`) or port 3 (a normal start into the active slot). The kernel's usbmux
(drivers/usb/hakios.akita_usbmux.c) syncs the data volume, sets the bootloader's reboot command and resets.

usage: usbmux-reboot.py [bootloader | bootloader-cold]      (needs usbmuxd running, and access to its socket)
(bootloader: the command, then a warm reset; bootloader-cold: the same with a cold reset, port 4)
"""
import plistlib
import socket
import struct
import sys


def mux_request(message):
    s = socket.socket(socket.AF_UNIX)
    s.connect('/var/run/usbmuxd')
    body = plistlib.dumps(message)
    s.sendall(struct.pack('<IIII', 16 + len(body), 1, 8, 1) + body)
    n = struct.unpack('<I', s.recv(16)[:4])[0]
    data = b''
    while len(data) < n - 16:
        data += s.recv(n - 16 - len(data))
    return s, plistlib.loads(data)


_, listing = mux_request({'MessageType': 'ListDevices'})
devices = listing.get('DeviceList', [])
if not devices:
    sys.exit('usbmux-reboot: no device (is usbmuxd running, the phone attached?)')
port = {'bootloader': 2, 'bootloader-cold': 4}.get(sys.argv[1] if sys.argv[1:] else '', 3)
mux_request({'MessageType': 'Connect', 'DeviceID': devices[0]['DeviceID'], 'PortNumber': socket.htons(port)})
print('usbmux-reboot: asked for', {2: 'the bootloader', 4: 'the bootloader (cold reset)'}.get(port, 'a restart'))
