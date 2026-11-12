#!/usr/bin/env python3
"""wgrep.py REGEX: disassemble the firmware's whole RAM text (Thumb, per function of the map) and print matching lines with the function."""
import sys, os, re, bisect
from capstone import Cs, CS_ARCH_ARM, CS_MODE_THUMB
FW = os.path.expanduser('~/akita/hackios-kernel/fw-wifi/'); BASE = 0x6e0000; END = 0x78a888
d = open(FW + 'fw_bcmdhd.bin_4383_a3', 'rb').read()
syms = sorted(set((int(p[0], 16) & ~1, p[2]) for p in (l.split() for l in open(FW + 'fw_bcmdhd.map_4383_a3')) if len(p) == 3 and p[1] in 'TtWw' and BASE <= int(p[0], 16) < END))
rx = re.compile(sys.argv[1]); cs = Cs(CS_ARCH_ARM, CS_MODE_THUMB); cs.skipdata = True
for i, (a, n) in enumerate(syms):
    e = syms[i + 1][0] if i + 1 < len(syms) else END
    if e <= a: continue
    for ins in cs.disasm(d[a - BASE:e - BASE], a):
        t = '%s %s' % (ins.mnemonic, ins.op_str)
        if rx.search(t): print('%x %s+0x%x: %s' % (ins.address, n, ins.address - a, t))
