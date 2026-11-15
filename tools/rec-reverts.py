#!/usr/bin/env python3
"""Frames of a screen recording that go BACK to an earlier picture: the flicker of a frame shown, replaced, then shown
again (Maps' tiles while zooming, 11-15: the compositor's mirror of a drawable one frame behind).

usage: rec-reverts.py FRAMES [FROM_MS TO_MS] [--quiet]

FRAMES is the folder tools/recording.py --frames wrote (one PPM a frame, named by its time in milliseconds of the
kernel's clock). For every frame after the first: how much of the picture changed from the frame before, where, and
whether it is the frame BEFORE THAT one again (a revert: fewer than a quarter as many pixels differ from it as from
the frame between). Prints every frame unless --quiet; always the count of reverts, and exits 1 if there were any,
so that a test can fail on them.
"""
import os
import sys


def load(path):
    data = open(path, 'rb').read()
    header, pixels = data.split(b'\n', 1)
    _, width, height, _ = header.split()
    return int(width), int(height), pixels


def diff(a, b, width, height):
    """pixels that differ, and the box they are in"""
    n = 0
    x0 = y0 = 1 << 30
    x1 = y1 = -1
    row = width * 3
    for y in range(height):
        ra = a[y * row:(y + 1) * row]
        rb = b[y * row:(y + 1) * row]
        if ra == rb:
            continue
        for x in range(width):
            i = x * 3
            if ra[i:i + 3] != rb[i:i + 3]:
                n += 1
                if x < x0:
                    x0 = x
                if x > x1:
                    x1 = x
                if y < y0:
                    y0 = y
                if y > y1:
                    y1 = y
    return n, (x0, y0, x1, y1) if n else None


def main():
    args = [a for a in sys.argv[1:] if a != '--quiet']
    quiet = '--quiet' in sys.argv
    if not args:
        sys.exit(__doc__)
    folder = args[0]
    lo = int(args[1]) if len(args) > 2 else 0
    hi = int(args[2]) if len(args) > 2 else 1 << 62
    names = sorted(n for n in os.listdir(folder) if n.endswith('.ppm'))
    frames = [(int(n.split('.')[0]), n) for n in names]
    frames = [(t, n) for t, n in frames if lo <= t <= hi]
    if len(frames) < 3:
        sys.exit(f'rec-reverts: {len(frames)} frames in {folder} between {lo} and {hi} ms: nothing to compare')
    loaded = []
    for t, n in frames:
        width, height, pixels = load(os.path.join(folder, n))
        loaded.append((t, pixels))
    total = width * height
    reverts = 0
    big = 0
    for i in range(1, len(loaded)):
        t, pb = loaded[i]
        t0, pa = loaded[i - 1]
        n, box = diff(pa, pb, width, height)
        note = ''
        if n * 50 >= total:
            big += 1
        if i >= 2 and n > 0 and n * 20 >= total:
            n2, _ = diff(loaded[i - 2][1], pb, width, height)
            if n2 * 4 < n:
                reverts += 1
                note = f'  <-- REVERT to the frame before last ({n2} px differ from it)'
        if not quiet:
            print(f'{t} (+{t - t0:4d} ms): {100 * n / total:5.1f}% changed{" box " + str(box) if box else ""}{note}')
    print(f'rec-reverts: {len(loaded)} frames from {loaded[0][0]} to {loaded[-1][0]} ms: {big} with 2% or more of the picture '
          f'changed, {reverts} REVERTS to the frame before last')
    sys.exit(1 if reverts else 0)


if __name__ == '__main__':
    main()
