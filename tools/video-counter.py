#!/usr/bin/env python3
"""video-counter.py CLIP [EPOCH [FFMPEG]] [--binary]: reads sb_debug=1's frame counter (userland/src/inject_cube.m) out of every
frame of a slow-motion video of the Pixel 8a's screen, and says where the frames on the panel were not one after the
other.

The strip under the status bar: a white and a black reference, then 12 bits b11..b0 of Gray code (--binary: the
strip of 11-10 10:31, a plain binary count), red-edged squares. It is found in
each frame by itself (the phone may move in the hand): in the top third, the row that changes most along its length,
cut into runs of clearly white or clearly black pixels -- fourteen in a row at an even pitch, white then black first.
A frame where that fails (blur, two refreshes in one exposure) is read where the last good one found the squares.

EPOCH: the console's "hakios: debug: counter epoch S" of the run; with it every frame gets the uptime of the refresh it
was drawn for (epoch + counter / 120, the counter's wraps every 34.1 s followed), the kernel log's clock.
FFMPEG: an ffmpeg binary (default: ffmpeg on the PATH).

Prints a line per video frame (index, time in the clip, the counter or '?' when a square was neither white nor black --
two frames in one exposure or a torn one), then what came out of order: BACK (a counter lower than the one before: an
older frame on the panel again), JUMP (refreshes the render server drew for that never reached the panel), HOLD (one
counter over more video frames than a refresh lasts: a freeze), with the uptimes when EPOCH is given.

  e.g. video-counter.py /mnt/c/Users/Martin/Videos/20261110_103138.mp4 41.847489 $FF > counter.txt
"""
import subprocess
import sys

import numpy as np

if len(sys.argv) < 2:
    sys.exit(__doc__)
args = [a for a in sys.argv[1:] if a != '--binary']
binary = '--binary' in sys.argv                 # (the strip before it was Gray code: 11-10 10:31's clip)
clip = args[0]
epoch = float(args[1]) if len(args) > 1 and args[1] != '-' else None
ff = args[2] if len(args) > 2 else 'ffmpeg'


def from_gray(g):
    n = g
    for shift in (1, 2, 4, 8):
        n ^= n >> shift
    return n
HZ, BITS = 120.0, 12

probe = subprocess.run([ff, '-hide_banner', '-i', clip], capture_output=True, text=True).stderr
fps = 30.0
for part in probe.split(','):
    part = part.strip()
    if part.endswith(' fps'):
        try:
            fps = float(part.split()[0])
        except ValueError:
            pass
capture = 240.0
for line in probe.splitlines():
    if 'capture.fps' in line:
        capture = float(line.split(':')[-1])
first = subprocess.run([ff, '-v', 'error', '-i', clip, '-frames:v', '1', '-f', 'image2pipe', '-vcodec', 'ppm', '-'],
                       capture_output=True).stdout
fw, fh = (int(v) for v in first.split(b'\n')[1].split())
proc = subprocess.Popen([ff, '-v', 'error', '-i', clip, '-fps_mode', 'passthrough', '-f', 'rawvideo', '-pix_fmt', 'gray',
                         '-'], stdout=subprocess.PIPE)


def runs_of(row):
    """runs of clearly white (1) or clearly black (0) pixels, 8 or more long: (start, end, class)"""
    cls = np.where(row > 150, 1, np.where(row < 35, 0, -1))
    out, x, n = [], 0, len(row)
    while x < n:
        c, s = cls[x], x
        while x < n and cls[x] == c:
            x += 1
        if c >= 0 and x - s >= 8:
            out.append((s, x - 1, int(c)))
    return out


def find_strip(f, y_hint):
    """(row, [14 centres]) or None"""
    lo, hi = (max(0, y_hint - 40), min(fh // 3, y_hint + 40)) if y_hint else (fh // 20, fh // 3)
    if hi <= lo:
        return None
    # the rows that change most along the frame's width (the strip's alternating squares)
    band = f[lo:hi].astype(np.int16)
    score = np.abs(np.diff(band, axis=1)).sum(axis=1)
    for y in (lo + int(i) for i in np.argsort(score)[::-1][:6]):
        row = f[max(0, y - 2):y + 3].mean(axis=0)
        runs = [r for r in runs_of(row) if 12 <= r[1] - r[0] <= fw // 12]
        for k in range(len(runs) - 13):
            cand = runs[k:k + 14]
            centres = [(a + b) / 2 for a, b, c in cand]
            pitch = np.diff(centres)
            if cand[0][2] == 1 and cand[1][2] == 0 and pitch.min() > 0.8 * np.median(pitch) and \
                    pitch.max() < 1.2 * np.median(pitch):
                return y, centres, float(np.median([b - a for a, b, c in cand]))
    return None


values, cal, index = [], None, 0
while True:
    raw = proc.stdout.read(fw * fh)
    if len(raw) < fw * fh:
        break
    f = np.frombuffer(raw, dtype=np.uint8).reshape(fh, fw)
    found = find_strip(f, cal[0] if cal else None)
    if found:
        cal = found
    if cal is None:
        values.append((index, None, 0))
        index += 1
        continue
    y, centres, width = cal
    half = max(2, int(width * 0.2))
    lum = [float(f[y - half:y + half + 1, int(c) - half:int(c) + half + 1].mean()) for c in centres]
    white, black = lum[0], lum[1]
    span = white - black
    counter, grey = 0, 0
    for k in range(BITS):
        v = (lum[2 + k] - black) / span if span > 40 else 0.5
        grey += 0.3 < v < 0.7
        counter = counter << 1 | (1 if v >= 0.5 else 0)
    if not binary:
        counter = from_gray(counter)
        ok = grey <= 1                          # (one square between: a frame spanning two refreshes, read as one)
    else:
        ok = grey == 0
    values.append((index, counter if ok else None, grey))
    index += 1

slow = capture / fps                            # the camera's frames to one of the clip's second
print('# %s: %d frames, %.2f fps played, captured at %.0f (%.1f x slow): a refresh at %.0f Hz is %.1f video frames'
      % (clip, len(values), fps, capture, slow, HZ, capture / HZ))


def uptime(i, c, base):
    return epoch + (base + c) / HZ if epoch is not None else None


# the counter's wraps followed: each reading's full count (base + counter)
full, base, last = [], 0, None
for i, c, grey in values:
    if c is not None:
        if last is not None and c < last - 2048:
            base += 4096
        last = c
    full.append(base + c if c is not None else None)
for (i, c, grey), n in zip(values, full):
    t = i / capture                             # the time the camera took it, from the clip's start (real time)
    up = ' up %.4f' % (epoch + n / HZ) if (epoch is not None and n is not None) else ''
    print('%5d %8.4f %s%s%s' % (i, t, '%5d' % n if n is not None else '    ?', up,
                                '  (%d squares grey)' % grey if grey else ''))

print('#')
print('# out of order: BACK (an older frame again), JUMP (frames never shown), HOLD (one frame over 3+ refreshes):')
prev, prev_i, hold_from = None, None, None
events = {'BACK': 0, 'JUMP': 0, 'HOLD': 0}
per_refresh = capture / HZ
for (i, c, grey), n in zip(values, full):
    if n is None:
        continue
    up = ' (up %.3f)' % (epoch + n / HZ) if epoch is not None else ''
    if prev is not None:
        if n == prev:
            pass
        elif n < prev:
            print('  BACK  frame %5d: %d after %d%s -- an older frame on the panel' % (i, n, prev, up))
            events['BACK'] += 1
        elif n > prev + 1:
            print('  JUMP  frame %5d: %d after %d%s -- %d refreshes without a new frame' % (i, n, prev, up, n - prev - 1))
            events['JUMP'] += 1
        if n != prev:
            if hold_from is not None and (prev_i - hold_from + 1) > 3 * per_refresh:
                print('  HOLD  frames %d-%d: %d for %.1f refreshes%s' % (hold_from, prev_i, prev,
                                                                        (prev_i - hold_from + 1) / per_refresh, up))
                events['HOLD'] += 1
            hold_from = i
    else:
        hold_from = i
    prev, prev_i = n, i
print('# %d BACK, %d JUMP, %d HOLD' % (events['BACK'], events['JUMP'], events['HOLD']))
