#!/usr/bin/env python3
"""The Pixel 8a's cameras' colour tuning for libhakios.media's processor (userland/src/camera_isp.c, isp_load_tuning: the
"RWCISP02" file tools/camtuning.py describes), out of what Android says of each camera -- its static characteristics, as
`adb shell dumpsys media.camera` prints them:

  android.sensor.forwardMatrix1/2      white-balanced camera RGB -> CIE XYZ (D50) under referenceIlluminant1/2
                                       (standard illuminant A, 2856 K, and D65)
  android.sensor.colorTransform1/2 and calibrationTransform1/2
                                       XYZ -> this sensor's RGB: what a grey looks like to it under those two lights

A colour matrix for each light (XYZ D50 -> linear sRGB after the forward matrix; rows sum to 1), between them by colour
temperature; the two lights, and a warm LED lamp measured here, as the white balance's reference lights; no lens shading (the grids all 1: Android has it
per frame only).

usage: camtuning-akita.py DUMPSYS.txt OUTDIR   -> OUTDIR/camera-akita-main.isp, -front.isp, -uw.isp
(the main camera is HAL device 0, the front one device 1, the ultra-wide device 0's physical camera 3)
"""
import os
import re
import struct
import sys

XYZ_D50_TO_SRGB = ((3.1338561, -1.6168667, -0.4906146), (-0.9787684, 1.9161415, 0.0334540), (0.0719453, -0.2289914, 1.4052427))
WHITE = {'A': (1.09850, 1.0, 0.35585), 'D65': (0.95047, 1.0, 1.08883)}
GRID = 17 * 13


def mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def rationals(lines, key):
    for i, l in enumerate(lines):
        if key + ' (' in l:
            v = []
            for m in lines[i + 1:i + 8]:
                if 'android.' in m or 'com.google' in m:
                    break
                v += [int(a) / int(b) for a, b in re.findall(r'\((-?\d+) / (\d+)\)', m)]
            if len(v) == 9:
                return [v[0:3], v[3:6], v[6:9]]
    sys.exit('no %s' % key)


def main():
    lines = open(sys.argv[1]).read().split('\n')
    out = sys.argv[2]
    os.makedirs(out, exist_ok=True)

    def start(text):
        return next(i for i, l in enumerate(lines) if text in l)

    cuts = {'main': start('internal/0 (v1.3) static information'), 'uw': start('Physical camera 3 characteristics'),
            'front': start('internal/1 (v1.3) static information')}
    ends = sorted([start('Physical camera 2 characteristics'), start('Physical camera 4 characteristics'), len(lines)] +
                  list(cuts.values()))
    for name in ('main', 'front', 'uw'):
        a = cuts[name]
        part = lines[a:next(e for e in ends if e > a)]
        ccm, refs = {}, {}
        for n, light, cct in ((1, 'A', 2856.0), (2, 'D65', 6504.0)):
            m = mul(XYZ_D50_TO_SRGB, rationals(part, 'android.sensor.forwardMatrix%d' % n))
            ccm[cct] = [[x / sum(row) for x in row] for row in m]          # (the rows' sums are 1 to a thousandth already)
            cam = mul(rationals(part, 'android.sensor.calibrationTransform%d' % n), rationals(part, 'android.sensor.colorTransform%d' % n))
            grey = [sum(cam[i][k] * WHITE[light][k] for k in range(3)) for i in range(3)]
            refs[cct] = (grey[0] / grey[1], grey[2] / grey[1])
        if name == 'front':
            front_refs = dict(refs)
        if name == 'uw':
            # The ultra-wide's greys as Android gives them are NOT what its frames show here: white things in a frame
            # of 11-19 (a radiator, the ceiling, a wall) want gains of 1.55-1.73 in red and 1.5-1.9 in blue, where
            # Android's two lights give 2.73/1.78 and 1.59/3.03 -- a quarter less green than it says, everywhere; kept
            # to that line the picture went magenta. They lie on the FRONT camera's line (the same sensor, an IMX712;
            # the main and front cameras' own white things lie on theirs): its reference lights, the ultra-wide's matrices.
            refs = front_refs
        b = bytearray(b'RWCISP02')
        b += struct.pack('<I2fI', 1, 0.0, 100000.0, 2)
        for cct, lo, hi in ((2856.0, 0.0, 2856.0), (6504.0, 6504.0, 100000.0)):
            b += struct.pack('<2f9f', lo, hi, *[x for row in ccm[cct] for x in row])
        b += struct.pack('<I2fI', 1, 0.0, 100000.0, 1) + struct.pack('<2f', 0.0, 100000.0) + struct.pack('<%df' % (4 * GRID), *([1.0] * 4 * GRID))
        # A third light, measured and not Android's: a warm white LED lamp (the tester's room, 11-19). White things in its
        # light want the red gain of illuminant A and a quarter more blue (the front camera: red x1.19-1.33, blue
        # x2.36-2.61, where A is 1.25 and 1.98) -- an LED has less blue than a filament of its colour. Without it the
        # white balance stopped at A and the lamp's light came out yellow-green. The same step for each camera.
        refs[2600.0] = (refs[2856.0][0], refs[2856.0][1] * 0.79)
        b += struct.pack('<I', 3)
        for cct in (6504.0, 2856.0, 2600.0):    # (from the bluest to the reddest)
            b += struct.pack('<3f', refs[cct][0], refs[cct][1], cct)
        path = os.path.join(out, 'camera-akita-%s.isp' % name)
        open(path, 'wb').write(b)
        print('%s: %d bytes' % (path, len(b)))
        for cct in (2856.0, 6504.0):
            print('  %4.0f K: a grey is R/G %.3f B/G %.3f; matrix %s' % (cct, refs[cct][0], refs[cct][1],
                  ' | '.join(' '.join('%6.3f' % x for x in row) for row in ccm[cct])))


if __name__ == '__main__':
    main()
