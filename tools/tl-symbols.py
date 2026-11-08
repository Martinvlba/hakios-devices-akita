#!/usr/bin/env python3
"""Name the frames of SpringBoard's long run-loop turns (touch_lat=1: "tl: t MS frame frame ...", userland/src/inject_sb.c),
where the shared cache hid the name and the line has the frame's cache address instead ("c:0x1a2b3c4d").

usage: tl-symbols.py SYMS LOG [LOG ...]
       SYMS: every symbol of the build's shared cache, one a line ("0xADDR:  (SECTION)  NAME  IMAGE"), as
             `ipsw dyld symaddr DSC --no-color > SYMS` writes it (the cache's .symbols file beside DSC for the
             hidden names); made once, looked up here by address (the nearest symbol at or below it)

Prints each stall with its frames named, then the frames by the milliseconds of the stalls they were in (each frame
counted once a stall), the top of the stack first in each line.
"""
import bisect
import collections
import sys

if len(sys.argv) < 3:
    sys.exit(__doc__)
syms_path, logs = sys.argv[1], sys.argv[2:]
addrs, labels = [], []
with open(syms_path, errors='replace') as f:
    for line in f:
        parts = line.rstrip('\n').split('\t')
        if len(parts) < 3 or not parts[0].startswith('0x'):
            continue
        try:
            a = int(parts[0].rstrip(':'), 16)
        except ValueError:
            continue
        addrs.append(a)
        labels.append((parts[2], parts[3] if len(parts) > 3 else ''))
order = sorted(range(len(addrs)), key=addrs.__getitem__)
addrs = [addrs[i] for i in order]
labels = [labels[i] for i in order]


def name(addr):
    a = int(addr, 16)
    i = bisect.bisect_right(addrs, a) - 1
    if i < 0 or a - addrs[i] > 0x20000:
        return addr
    sym, image = labels[i]
    return f'{image}: {sym} + {a - addrs[i]:#x}'


stalls = []
for log in logs:
    with open(log, 'rb') as f:
        for raw in f:
            line = raw.decode('latin1')
            i = line.find('tl: t ')
            if i < 0:
                continue
            parts = line[i + 6:].split()
            if not parts or not parts[0].isdigit():
                continue
            stalls.append((int(parts[0]), parts[1:]))

by_frame = collections.Counter()
for ms, frames in stalls:
    named = [name(f[2:]) if f.startswith('c:') else f for f in frames]
    print(f'{ms:5d} ms: ' + '  <  '.join(named[:8]))
    for n in set(named):
        by_frame[n] += ms
print('\nby frame (ms of the stalls it was in):')
for n, ms in by_frame.most_common(40):
    print(f'{ms:6d}  {n}')
