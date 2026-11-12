#!/usr/bin/env python3
"""wdis.py NAME|ADDR [LEN]: disassemble (Thumb) a function of the BCM4383 firmware, with the map's names for branch targets."""
import sys, os, struct, bisect
from capstone import Cs, CS_ARCH_ARM, CS_MODE_THUMB
FW = os.path.expanduser('~/akita/hackios-kernel/fw-wifi/')
BASE = 0x6e0000
d = open(FW + 'fw_bcmdhd.bin_4383_a3', 'rb').read()
syms = []
for l in open(FW + 'fw_bcmdhd.map_4383_a3'):
    p = l.split()
    if len(p) == 3 and not p[2].endswith('__bcmromfn'):
        syms.append((int(p[0], 16), p[2]))
syms.sort()
addrs = [a for a, _ in syms]
def name(a):
    a &= ~1
    i = bisect.bisect_right(addrs, a) - 1
    if i < 0: return hex(a)
    b, n = syms[i]
    return n if a == b else '%s+0x%x' % (n, a - b)
arg = sys.argv[1]
try: start = int(arg, 16)
except ValueError:
    c = [a for a, n in syms if n == arg or n.endswith('$' + arg)]
    start = c[0]
start &= ~1
i = bisect.bisect_right(addrs, start)
while i < len(addrs) and addrs[i] & ~1 == start: i += 1
ln = int(sys.argv[2], 0) if len(sys.argv) > 2 else min((addrs[i] & ~1) - start, 0x600)
cs = Cs(CS_ARCH_ARM, CS_MODE_THUMB); cs.skipdata = True
print('%s at 0x%x, %d bytes' % (name(start), start, ln))
for ins in cs.disasm(d[start - BASE:start - BASE + ln], start):
    s = ins.op_str; extra = ''
    if ins.mnemonic.startswith('b') and '#0x' in s:
        extra = '   ; ' + name(int(s.split('#')[-1], 16))
    elif 'pc' in s and ins.mnemonic.startswith('ldr') and '#' in s:
        try:
            off = int(s.split('#')[-1].rstrip(']'), 0); a = ((ins.address + 4) & ~3) + off
            v = struct.unpack_from('<I', d, a - BASE)[0]; extra = '   ; =0x%x %s' % (v, name(v) if v > 0x400000 else '')
        except Exception: pass
    print('%x: %-7s %s%s' % (ins.address, ins.mnemonic, s, extra))
