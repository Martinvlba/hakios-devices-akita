#!/usr/bin/env python3
"""wcallers.py NAME|ADDR: every Thumb BL/B.W in the firmware's RAM text whose target is the function."""
import sys, os, struct, bisect
FW = os.path.expanduser('~/akita/hackios-kernel/fw-wifi/'); BASE = 0x6e0000
d = open(FW + 'fw_bcmdhd.bin_4383_a3', 'rb').read()
syms = sorted((int(p[0], 16), p[2]) for p in (l.split() for l in open(FW + 'fw_bcmdhd.map_4383_a3')) if len(p) == 3 and not p[2].endswith('__bcmromfn'))
addrs = [a for a, _ in syms]
def name(a):
    i = bisect.bisect_right(addrs, a) - 1; b, n = syms[i]; return '%s+0x%x' % (n, a - b)
arg = sys.argv[1]
try: t = int(arg, 16)
except ValueError: t = [a for a, n in syms if n == arg or n.endswith('$' + arg)][0]
t &= ~1
for off in range(0, 0x78a888 - BASE - 4, 2):
    h, l = struct.unpack_from('<HH', d, off)
    if h & 0xf800 == 0xf000 and l & 0x8000:
        s = (h >> 10) & 1; j1 = (l >> 13) & 1; j2 = (l >> 11) & 1
        if l & 0x1000 or True:
            i1 = 1 - (j1 ^ s); i2 = 1 - (j2 ^ s)
            imm = (s << 24) | (i1 << 23) | (i2 << 22) | ((h & 0x3ff) << 12) | ((l & 0x7ff) << 1)
            if s: imm -= 1 << 25
            if l & 0x5000 in (0x5000, 0x1000) and (BASE + off + 4 + imm) == t:
                print('%x %s %s' % (BASE + off, 'bl' if l & 0x4000 else 'b.w', name(BASE + off)))
# pointers to it in data (function tables, ramfnptr)
for off in range(0, len(d) - 4, 4):
    if struct.unpack_from('<I', d, off)[0] == t | 1: print('ptr at %x %s' % (BASE + off, name(BASE + off)))
