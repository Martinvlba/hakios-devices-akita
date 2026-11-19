#!/usr/bin/env python3
"""rec-boxes.py REC [SHEET.png]: the frames of a screen recording (boot-arg rec=N, usbmux-rec.py) in which an area is much
darker than at the same place in the frame before AND the frame after -- a one-frame black box (the cover sheet held two
thirds down and wiggled, rc.jobs.akitacover: 2 to 4 of 318 frames, 11-19). SHEET: those frames, half size."""
import struct, sys, numpy as np
from PIL import Image
d=np.memmap(sys.argv[1],dtype=np.uint8,mode='r')
magic,w,h,n,oldest=struct.unpack_from('<8sIIII',bytes(d[:32]),0)
fs=8+w*h*3
order=[(oldest+i)%n for i in range(n)]
def frame(i):
    o=32+order[i]*fs
    return np.asarray(d[o+8:o+fs]).reshape(h,w,3)
# a "box": a compact region much darker than the same place one frame before AND one frame after
hits=[]
for i in range(1,n-1):
    f=frame(i).astype(np.int16).sum(2); p=frame(i-1).astype(np.int16).sum(2); q=frame(i+1).astype(np.int16).sum(2)
    m=(p-f>45)&(q-f>45)
    ys,xs=np.nonzero(m)
    if len(ys)>200:
        hits.append((i,len(ys),int(xs.min()),int(xs.max()),int(ys.min()),int(ys.max())))
print('%d frames of %dx%d; %d with an area darker than in both neighbours:'%(n,w,h,len(hits)))
for x in hits[:12]: print('  frame %d: %d px, x %d-%d y %d-%d'%x)
# and persistently dark boxes: pixels nearly black where the frame's surroundings are not
if len(sys.argv)>2:
    ims=[Image.fromarray(frame(i)).resize((w//2,h//2)) for i in ([x[0] for x in hits[:5]] or [n//4,n//2,3*n//4])]
    W=Image.new('RGB',((w//2+6)*len(ims),h//2),'white')
    for k,im in enumerate(ims): W.paste(im,(k*(w//2+6),0))
    W.save(sys.argv[2])
