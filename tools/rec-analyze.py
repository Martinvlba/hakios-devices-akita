#!/usr/bin/env python3
"""rec-analyze.py REC [SHEET.png]: the frames of a screen recording (boot-arg rec=N, devices/akita/tools/usbmux-rec.py)
checked one by one with sb_debug=1's strip: its 12 squares of Gray code read where they are in the frame (the strip's
own window, userland/src/inject_cube.m: 16-point squares 19 apart from (4.5, 37) points) -- every recorded frame is
what the panel began to read, so a frame whose counter is LOWER than the one before was an older picture shown again,
one whose squares are neither white nor black (or whose references are wrong) was shown unfinished, and a counter that
jumps is refreshes CoreAnimation drew nothing new for. SHEET: the suspects, each beside the frames before and after.
"""
import struct
import sys

from PIL import Image, ImageDraw

data = open(sys.argv[1], 'rb').read()
magic, w, h, n, oldest = struct.unpack_from('<8sIIII', data, 0)
HDR, FH = 32, 8
fs = FH + w * h * 3
step = round(1080 / w)                          # (rec=N: one pixel of every N; the panel's 1080 points x 3)


def frame(i):
    off = HDR + ((oldest + i) % n) * fs
    return struct.unpack_from('<Q', data, off)[0], data[off + FH:off + fs]


def px(buf, x, y):
    x, y = int(x), int(y)
    vals = []
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            o = ((y + dy) * w + x + dx) * 3
            vals.append(sum(buf[o:o + 3]) / 3)
    return sum(vals) / len(vals)


def gray_to_bin(g):
    b = g
    for s in (1, 2, 4, 8):
        b ^= b >> s
    return b


rows = []
for i in range(n):
    ns, buf = frame(i)
    lum = [px(buf, ((4.5 + 19 * k + 8) * 3 - 1) / step, ((37 + 8) * 3 - 1) / step) for k in range(14)]
    white, black = lum[0], lum[1]
    span = white - black
    code, grey = 0, 0
    for k in range(12):
        v = (lum[2 + k] - black) / span if span > 60 else 0.5
        grey += 0.25 < v < 0.75
        code = code << 1 | (v >= 0.5)
    ok = span > 60 and grey == 0
    rows.append((i, ns / 1e9, gray_to_bin(code) if ok else None, grey, span, ns & 1))

sus = []
prev = None
for i, t, c, grey, span, busy in rows:
    why = None
    if c is None:
        why = 'strip unreadable (%d squares grey, references %d apart)' % (grey, span)
    elif prev is not None:
        d = (c - prev) % 4096
        if d > 2048:
            why = 'counter %d after %d: an older picture' % (c, prev)
        elif d == 0:
            why = 'counter %d again: the same picture recorded twice' % c
        elif d > 3:
            why = 'counter %d after %d: %d refreshes without a new picture' % (c, prev, d - 1)
    if why:
        sus.append((i, t, why + (' [GPU busy]' if busy else '')))
    if c is not None:
        prev = c
gaps = [b[1] - a[1] for a, b in zip(rows, rows[1:])]
allbusy = sum(r[5] for r in rows)
print('frames recorded with the GPU still busy: %d of %d' % (allbusy, n))
for kind in ('unreadable', 'older', 'twice', 'refreshes'):
    k = [x for x in sus if kind in x[2]]
    if k:
        print('  %-11s %3d suspects, %3d of them with the GPU busy' % (kind, len(k), sum('GPU busy' in x[2] for x in k)))
print('%s: %d frames over %.2f s; the longest gap between recorded frames %.1f ms; suspects %d'
      % (sys.argv[1], n, rows[-1][1] - rows[0][1], 1000 * max(gaps or [0]), len(sus)))
kinds = {}
for i, t, why in sus:
    kinds[why.split(':')[0].split(' (')[0].rsplit(' ', 1)[0] if 'counter' not in why else why.split(': ')[-1]] = 1 + \
        kinds.get(why.split(':')[0].split(' (')[0].rsplit(' ', 1)[0] if 'counter' not in why else why.split(': ')[-1], 0)
for k, v in sorted(kinds.items(), key=lambda kv: -kv[1]):
    print('  %4d  %s' % (v, k))
for i, t, why in sus[:40]:
    print('  frame %4d at %7.3f s: %s' % (i, t, why))
if len(sys.argv) > 2 and sus:
    pick = [s for s in sus if 'unreadable' in s[2] or 'older' in s[2]][:6] or sus[:6]
    sheet = Image.new('RGB', (3 * (w + 4), len(pick) * (h + 4)), 'white')
    for r, (i, t, why) in enumerate(pick):
        for c, j in enumerate((i - 1, i, i + 1)):
            if 0 <= j < n:
                im = Image.frombytes('RGB', (w, h), frame(j)[1])
                ImageDraw.Draw(im).text((2, h - 12), '%d' % j, fill=(255, 255, 0))
                sheet.paste(im, (c * (w + 4), r * (h + 4)))
    sheet.save(sys.argv[2])
    print('sheet: %s' % sys.argv[2])
