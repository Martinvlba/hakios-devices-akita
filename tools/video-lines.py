#!/usr/bin/env python3
"""video-lines.py RAW T0 K [W H]: frames with a bright horizontal line (a row much brighter than K frames before and after).
RAW: gray frames cropped to the phone's screen, e.g. ffmpeg -ss 4.9 -t 3.3 -i clip.mp4 -vf "crop=900:1700:90:60,
scale=450:850,format=gray" -f rawvideo w.raw; T0 the -ss. Found the display's white lines of 11-10 (fixed rows, every flip)."""
import sys, numpy as np
w, h = (int(sys.argv[4]), int(sys.argv[5])) if len(sys.argv) > 5 else (450, 850)
f = np.fromfile(sys.argv[1], dtype=np.uint8); n = f.size // (w*h)
f = f[:n*w*h].reshape(n, h, w).astype(np.float32); t0 = float(sys.argv[2]); k = int(sys.argv[3]); fps = 29.22
res = []
for t in range(k, n-k):
    frac = ((f[t] - np.maximum(f[t-k], f[t+k])) > 18).mean(axis=1); y = int(frac.argmax()); res.append((frac[y], t, y))
for s, t, y in sorted(res, reverse=True)[:15]:
    print("%.3f s (frame %3d): row %3d, %.0f%% of it brighter" % (t0 + t/fps, t, y, 100*s))
