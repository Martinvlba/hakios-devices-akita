#!/usr/bin/env python3
"""Fetch the kernel logs the phone kept while it was off the cable (userland/src/klogsave.c: one file per boot in
/var/mobile/Media/hakios-klog, written twice a second) over AFC, with the PC's pairing as usbmux-install.py makes the
connection. Reads only.

usage: usbmux-pull-klog.py OUTDIR [--all]      the newest three files (--all: every one), whole
       usbmux-pull-klog.py --summary FILE...   battery and CPU-idle lines of fetched files, condensed

(the phone's kernel is started with battery_say=6 -- "akita battery say: N mA ..." every 30 s -- and cpuidle=1 --
"akita mct: idle so far: ..." every 30 s -- for these to be in the files.)
"""
import importlib.util
import os
import re
import struct
import sys

here = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location('usbmux_install', os.path.join(here, 'usbmux-install.py'))
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)

AFC_READ_DIR, AFC_FILE_READ = 3, 15
DIR = 'hakios-klog'


def summary(paths):
    for path in paths:
        text = open(path, errors='replace').read()
        if os.path.basename(path).startswith('wired_'):
            lines = [l for l in text.splitlines() if l.startswith('memsay: at')]
            print('%s: %d lines; the first, one in the middle and the last:' % (os.path.basename(path), len(lines)))
            for l in ([lines[0], lines[len(lines) // 2], lines[-1]] if lines else []):
                print('  ' + l[:400])
            continue
        # (klogsave before 11-13 10:30 wrote the ring anew when it lost its place: each line once, by its time)
        seen = set()
        text = '\n'.join(l for l in text.splitlines() if not (l in seen or seen.add(l)))
        stamp = lambda line: float(re.search(r'\[\s*([0-9.]+)\]', line).group(1)) if re.search(r'\[\s*([0-9.]+)\]', line) else None
        battery = [(stamp(l), int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4)), m.group(5)) for l in text.splitlines()
                   for m in [re.search(r'akita battery say: (-?\d+) mA \(mean (-?\d+) mA\), (\d+) mV, charge (-?\d+) uAh, (.*)', l)] if m]
        idle = [(stamp(l), [int(x) for x in m.groups()]) for l in text.splitlines()
                for m in [re.search(r'idle so far: (\d+) tries -- (\d+) powered down, (\d+) back at once, (\d+) refused', l)] if m]
        print('%s: %d bytes, %d battery lines, %d idle lines' % (os.path.basename(path), len(text), len(battery), len(idle)))
        if battery:
            first, last = battery[0], battery[-1]
            hours = ((last[0] or 0) - (first[0] or 0)) / 3600
            print('  battery: %.2f h of lines; charge %d -> %d mAh, %d -> %d mV' % (hours, first[4] // 1000, last[4] // 1000,
                                                                                 first[3], last[3]))
            # ten stretches: the mean current of each, and how it ended (on the cable or not)
            n = max(1, len(battery) // 10)
            for i in range(0, len(battery), n):
                part = battery[i:i + n]
                print('    at %7.0f s: %5d mA mean of %d lines, %d mV, %s' % (part[0][0] or 0, sum(b[1] for b in part) // len(part),
                                                                           len(part), part[-1][3], part[-1][5][:60]))
        if idle:
            (t0, a), (t1, b) = idle[0], idle[-1]
            d = [y - x for x, y in zip(a, b)]
            print('  idle over %.0f s: %d tries, %d powered down, %d back at once, %d refused' % ((t1 or 0) - (t0 or 0), *d))
        for l in text.splitlines():
            if re.search(r'panic|ring went round|GPU fault|watchdog', l, re.I) and 'panic_trace' not in l:
                print('  ! ' + l[:160])


def main():
    if sys.argv[1:2] == ['--summary']:
        return summary(sys.argv[2:])
    if not sys.argv[1:]:
        sys.exit(__doc__)
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    afc = base.Afc(base.Phone().service('com.apple.afc'))
    _, data = afc.call(AFC_READ_DIR, DIR.encode() + b'\0')
    every = [n.decode() for n in data.split(b'\0')]
    names = sorted(n for n in every if n.startswith('kernel_'))
    wired = sorted(n for n in every if n.startswith('wired_'))      # (memsay's: where the kernel's memory is)
    if '--all' not in sys.argv:
        names, wired = names[-3:], wired[-3:]
    names += wired
    got = []
    for name in names:
        _, data = afc.call(base.AFC_OPEN, struct.pack('<Q', 1) + ('%s/%s' % (DIR, name)).encode() + b'\0')   # 1: read
        handle = data[:8]
        size = 0
        with open(os.path.join(out, name), 'wb') as f:
            while True:
                _, part = afc.call(AFC_FILE_READ, handle + struct.pack('<Q', 1 << 20))
                if not part:
                    break
                f.write(part)
                size += len(part)
        afc.call(base.AFC_CLOSE, handle)
        print('usbmux-pull-klog: %s, %d bytes' % (name, size))
        got.append(os.path.join(out, name))
    afc.s.close()
    summary(got)


if __name__ == '__main__':
    try:
        main()
    except (ConnectionError, RuntimeError, OSError, EOFError) as e:
        sys.exit('usbmux-pull-klog: %s' % e)
