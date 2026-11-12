#!/usr/bin/env python3
"""wsites.py: every event_logN call in the firmware's RAM text: FORMAT TAG FUNCTION+OFF (format = the literal in r1, tag = r0),
written to evsites.txt; `wsites.py FMT ...` looks formats up."""
import sys, os, struct, bisect, re
from capstone import Cs, CS_ARCH_ARM, CS_MODE_THUMB
FW = os.path.expanduser('~/akita/hackios-kernel/fw-wifi/'); BASE = 0x6e0000; END = 0x78a888
d = open(FW + 'fw_bcmdhd.bin_4383_a3', 'rb').read()
allsyms = [(int(p[0], 16) & ~1, p[2]) for p in (l.split() for l in open(FW + 'fw_bcmdhd.map_4383_a3')) if len(p) == 3 and not p[2].endswith('__bcmromfn')]
ev = {a: n for a, n in allsyms if re.match(r'event_log\d$|event_logn$|event_logv$', n)}
syms = sorted(set((a, n) for a, n in allsyms if BASE <= a < END))
cs = Cs(CS_ARCH_ARM, CS_MODE_THUMB); cs.skipdata = True
out = []
for i, (a, n) in enumerate(syms):
    e = syms[i + 1][0] if i + 1 < len(syms) else END
    if e <= a: continue
    r0 = r1 = None
    for ins in cs.disasm(d[a - BASE:e - BASE], a):
        s = ins.op_str
        if ins.mnemonic.startswith('ldr') and s.startswith('r1, [pc'):
            try:
                off = int(s.split('#')[-1].rstrip(']'), 0); r1 = struct.unpack_from('<I', d, ((ins.address + 4) & ~3) + off - BASE)[0]
            except Exception: r1 = None
        elif ins.mnemonic in ('movs', 'mov', 'movw', 'mov.w') and s.startswith('r1, #'): r1 = int(s.split('#')[1], 0)
        elif ins.mnemonic in ('movs', 'mov', 'movw', 'mov.w') and s.startswith('r0, #'): r0 = int(s.split('#')[1], 0)
        elif ins.mnemonic in ('bl', 'b.w', 'b') and '#0x' in s:
            t = int(s.split('#')[1], 16)
            if t in ev and r1 is not None:
                out.append((r1, r0, '%s+0x%x' % (n, ins.address - a)))
            if ins.mnemonic == 'bl': r0 = None
open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'evsites.txt'), 'w').write(''.join('%04x %s %s\n' % (f, '%x' % t if t is not None else '?', w) for f, t, w in sorted(out, key=lambda x: (x[0], x[2]))))
print(len(out), 'sites')
