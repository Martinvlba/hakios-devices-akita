#!/usr/bin/env python3
"""video-flashes.py RAW FPS [W H]: one-frame flashes in a phone video (a frame unlike both neighbours while they agree).
A torn or half-drawn frame shows up as one; moving content does not. RAW: gray frames, e.g.
  ffmpeg -i clip.mp4 -vf "scale=180:320,format=gray" -f rawvideo clip.raw
(ffmpeg without sudo: python3 -m venv V; V/bin/pip install imageio-ffmpeg numpy; the binary is in
V/lib/python3*/site-packages/imageio_ffmpeg/binaries/). Used 11-09/10 on the tester's 60 fps and slow-motion clips."""
import sys, numpy as np
w, h = (int(sys.argv[3]), int(sys.argv[4])) if len(sys.argv) > 4 else (180, 320)
raw = np.fromfile(sys.argv[1], dtype=np.uint8); n = raw.size // (w*h)
f = raw[:n*w*h].reshape(n, h, w).astype(np.int16); fps = float(sys.argv[2])
d = lambda a, b: np.abs(f[a]-f[b]).mean()
res = sorted(((min(d(t, t-1), d(t, t+1)) - d(t-1, t+1), t) for t in range(1, n-1)), reverse=True)
print("frames", n)
for s, t in res[:15]:
    print("%7.2f s  frame %5d  score %.2f" % (t/fps, t, s))
