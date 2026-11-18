#!/usr/bin/env python3
"""Fetch the front camera's frame the kernel saved (camera=9: /var/mobile/Media/hakios-klog/front.raw, the packed RAW10
the front end wrote: 3072 lines of 5312 bytes, 4208 pixels each) over AFC and make pictures of it.

usage: usbmux-pull-frame.py OUTDIR            fetch front.raw, write front.raw, front-gray.png and front.png
       usbmux-pull-frame.py --convert RAW OUTDIR    pictures of a fetched file only

front-gray.png: every pixel as it is (the Bayer mosaic, 8 bits). front.png: half size, one colour pixel from each 2x2
cell (the green pair averaged), white balance from the unclipped mid-tones, gamma 2.2, turned upright (--rotate DEGREES,
default 90: the front sensor's lines run up the phone). --bayer RGGB|BGGR|GRBG|GBRG picks the cell's order (default BGGR:
with RGGB a wooden table came out blue). Needs numpy and Pillow for the pictures.
"""
import importlib.util
import os
import struct
import sys

STRIDE, HEIGHT, WIDTH = 5312, 3072, 4208
AFC_FILE_READ = 15
REMOTE = 'hakios-klog/front.raw'


def fetch(out):
    here = os.path.dirname(os.path.abspath(__file__))
    spec = importlib.util.spec_from_file_location('usbmux_install', os.path.join(here, 'usbmux-install.py'))
    base = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(base)
    afc = base.Afc(base.Phone().service('com.apple.afc'))
    _, data = afc.call(base.AFC_OPEN, struct.pack('<Q', 1) + REMOTE.encode() + b'\0')     # 1: read
    handle = data[:8]
    size = 0
    path = os.path.join(out, 'front.raw')
    with open(path, 'wb') as f:
        while True:
            _, part = afc.call(AFC_FILE_READ, handle + struct.pack('<Q', 1 << 20))
            if not part:
                break
            f.write(part)
            size += len(part)
    afc.call(base.AFC_CLOSE, handle)
    afc.s.close()
    print('usbmux-pull-frame: %s, %d bytes' % (path, size))
    return path


def convert(raw, out, order, rot=90, sat=1.35):
    import numpy as np
    from PIL import Image

    d = np.fromfile(raw, dtype=np.uint8)
    if d.size < STRIDE * HEIGHT:
        sys.exit('%s: %d bytes, a frame is %d' % (raw, d.size, STRIDE * HEIGHT))
    d = d[:STRIDE * HEIGHT].reshape(HEIGHT, STRIDE)[:, :WIDTH // 4 * 5].reshape(HEIGHT, WIDTH // 4, 5).astype(np.uint16)
    low = d[:, :, 4]
    px = np.stack([(d[:, :, i] << 2) | ((low >> (2 * i)) & 3) for i in range(4)], axis=2).reshape(HEIGHT, WIDTH)
    print('usbmux-pull-frame: pixels min %d max %d mean %.1f; rows that are all zero: %d' %
          (px.min(), px.max(), px.mean(), int((px.max(axis=1) == 0).sum())))
    Image.fromarray((px >> 2).astype(np.uint8)).save(os.path.join(out, 'front-gray.png'))
    cell = {c: px[y::2, x::2].astype(np.float64) for c, (y, x) in zip('abcd', ((0, 0), (0, 1), (1, 0), (1, 1)))}
    names = dict(zip('abcd', order))
    r = next(cell[k] for k in 'abcd' if names[k] == 'R')
    b = next(cell[k] for k in 'abcd' if names[k] == 'B')
    g = sum(cell[k] for k in 'abcd' if names[k] == 'G') / 2
    # black level 64 of 1023 (the sensor's pedestal); white balance from the pixels no channel clips in (grey world over
    # the mid-tones); a cell with any channel at the sensor's ceiling is white (a clipped channel scaled by a gain is what
    # turned windows pink); a little saturation in place of the colour matrix nobody has measured; gamma 2.2
    black, ceil = 64.0, 1000.0
    clipped = (r >= ceil) | (g >= ceil) | (b >= ceil)
    rgb = [np.clip(c - black, 0, None) for c in (r, g, b)]
    lum = (rgb[0] + 2 * rgb[1] + rgb[2]) / 4
    mid = (~clipped) & (lum > np.percentile(lum, 20)) & (lum < np.percentile(lum, 95))
    gm = rgb[1][mid].mean() or 1.0
    gains = [gm / (c[mid].mean() or 1.0) for c in rgb]
    gains = [min(max(k, 0.5), 3.0) for k in gains]
    rgb = [c * k for c, k in zip(rgb, gains)]
    top = float(np.percentile(np.stack(rgb)[:, ~clipped], 99.7)) if (~clipped).any() else 959.0
    rgb = [np.clip(c / top, 0, 1) for c in rgb]
    grey = (rgb[0] + rgb[1] + rgb[2]) / 3
    rgb = [np.clip(grey + (c - grey) * sat, 0, 1) for c in rgb]
    for c in rgb:
        c[clipped] = 1.0
    img = np.stack([c ** (1 / 2.2) * 255 for c in rgb], axis=2).astype(np.uint8)
    out_img = Image.fromarray(img, 'RGB')
    if rot:
        out_img = out_img.rotate(rot, expand=True)
    out_img.save(os.path.join(out, 'front.png'))
    print('usbmux-pull-frame: white balance gains R %.2f G %.2f B %.2f, %.1f %% of the cells clipped' %
          (gains[0], gains[1], gains[2], 100.0 * clipped.mean()))
    img = np.asarray(out_img)
    print('usbmux-pull-frame: %s (%dx%d) and front-gray.png' % (os.path.join(out, 'front.png'), img.shape[1], img.shape[0]))


def main():
    args = [a for a in sys.argv[1:]]
    order, rot = 'BGGR', 90
    if '--bayer' in args:
        i = args.index('--bayer')
        order = args[i + 1].upper()
        del args[i:i + 2]
    if '--rotate' in args:
        i = args.index('--rotate')
        rot = int(args[i + 1])
        del args[i:i + 2]
    if args[:1] == ['--convert'] and len(args) == 3:
        os.makedirs(args[2], exist_ok=True)
        return convert(args[1], args[2], order, rot)
    if len(args) != 1:
        sys.exit(__doc__)
    os.makedirs(args[0], exist_ok=True)
    convert(fetch(args[0]), args[0], order, rot)


if __name__ == '__main__':
    main()
