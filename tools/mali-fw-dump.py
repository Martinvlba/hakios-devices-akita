#!/usr/bin/env python3
"""mali-fw-dump.py FILE: the entries of a Mali CSF firmware image (mali_csffw-*.bin), as the kernel's loader must
place them: each interface entry's MCU virtual range, flags and file bytes. Written from the format as Google's kbase
reads it (header magic 0xC3F13A6E, version 0.3; entries: type in bits 7:0, size in bytes in 15:8, update bit 30,
optional bit 31)."""
import struct, sys

TYPES = {0: 'interface', 1: 'configuration', 3: 'trace buffer', 4: 'timeline metadata', 6: 'build info',
         7: 'func call list', 8: 'page fault count', 9: 'core dump'}

def flags_text(f):
    out = [c for b, c in ((0, 'R'), (1, 'W'), (2, 'X')) if f >> b & 1]
    out.append('cache%d' % (f >> 3 & 3))
    for b, n in ((5, 'protected'), (30, 'shared'), (31, 'zero')):
        if f >> b & 1:
            out.append(n)
    rest = f & ~((7) | (3 << 3) | (1 << 5) | (1 << 30) | (1 << 31))
    if rest:
        out.append('other 0x%x' % rest)
    return ' '.join(out)

def main(path):
    d = open(path, 'rb').read()
    magic, vmin, vmaj, _, vhash, _, end = struct.unpack_from('<IBBHIII', d, 0)
    print('%s: %d bytes, magic 0x%08x, header %d.%d, hash 0x%08x, entries end 0x%x' % (path, len(d), magic, vmaj, vmin,
          vhash, end))
    off = 0x14
    while off < end:
        h, = struct.unpack_from('<I', d, off)
        t, size = h & 0xff, h >> 8 & 0xff
        words = struct.unpack_from('<%dI' % (size // 4 - 1), d, off + 4)
        tag = '%s%s' % (' optional' if h >> 31 & 1 else '', ' update' if h >> 30 & 1 else '')
        name = TYPES.get(t, 'type %d' % t)
        if t == 0:
            f, vs, ve, ds, de = words[:5]
            nm = bytes(d[off + 4 + 0x14: off + size]).split(b'\0')[0].decode(errors='replace')
            print('  0x%04x %-13s va 0x%08x-0x%08x (%6d KiB)  file 0x%06x-0x%06x (%6d B)  %-30s %s%s' % (
                  off, name, vs, ve, (ve - vs) // 1024, ds, de, de - ds, flags_text(f), nm, tag))
        else:
            txt = bytes(d[off + 4: off + size])
            s = ''.join(chr(c) if 32 <= c < 127 else '.' for c in txt)
            print('  0x%04x %-13s %3d B: %s%s' % (off, name, size, ' '.join('%08x' % w for w in words[:6]),
                  tag) + ('  "%s"' % s.strip('.')[:60] if t in (1, 3, 6) else ''))
        if size == 0:
            break
        off += size

if __name__ == '__main__':
    for p in sys.argv[1:]:
        main(p)
