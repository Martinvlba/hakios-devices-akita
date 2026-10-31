#!/usr/bin/env python3
"""Read out what devices/akita/pull-boot-info.sh pulled: the boot image's kind, and where the splash screen is.

usage: devices/akita/pull-boot-info.py devices/akita/boot-info

What it says goes into devices/akita/u-boot/boot-image (header version, a compressed kernel or not) and
devices/akita/u-boot/files/dts/upstream/src/arm64/exynos/google/zuma-akita.dts (the framebuffer). It prints; it
changes nothing.
"""
import os
import re
import struct
import sys


def boot_header(path):
    """An Android boot image's header: its version, page size, the kernel's size and what its first bytes say."""
    d = open(path, 'rb').read()
    if d[:8] != b'ANDROID!':
        return f'not a boot image (starts {d[:8]!r})'
    version = struct.unpack_from('<I', d, 40)[0]
    if version >= 3:
        kernel_size, ramdisk_size, os_version, header_size = struct.unpack_from('<IIII', d, 8)
        page = 4096
    else:
        kernel_size, _, ramdisk_size = struct.unpack_from('<III', d, 8)
        page = struct.unpack_from('<I', d, 36)[0]
        os_version = struct.unpack_from('<I', d, 44)[0]
    k = d[page:page + 0x40]
    if len(k) < 0x40:
        kind = '(the kernel\'s first bytes were not pulled)'
    elif k[0x38:0x3c] == b'ARM\x64':
        kind = 'an uncompressed arm64 Image'
    elif k[:2] == b'\x1f\x8b':
        kind = 'gzip'
    elif k[:4] in (b'\x02\x21\x4c\x18', b'\x04\x22\x4d\x18'):
        kind = 'lz4'
    else:
        kind = f'unknown ({k[:4].hex()})'
    ver = os_version >> 11
    level = os_version & 0x7ff
    os_text = (f'Android {ver >> 14}.{(ver >> 7) & 0x7f}.{ver & 0x7f}, patch level '
               f'{2000 + (level >> 4)}-{level & 0xf:02d}') if os_version else 'not given'
    return (f'header version {version}, page {page}, kernel {kernel_size} bytes ({kind}), ramdisk {ramdisk_size} '
            f'bytes, os {os_text}')


def vendor_boot_header(path):
    d = open(path, 'rb').read()
    if d[:8] != b'VNDRBOOT':
        return f'not a vendor_boot image (starts {d[:8]!r})'
    version, page = struct.unpack_from('<II', d, 8)
    ramdisk_size = struct.unpack_from('<I', d, 24)[0]
    dtb_size = struct.unpack_from("<I", d, 2100)[0] if version >= 3 else 0
    return f'header version {version}, page {page}, vendor ramdisk {ramdisk_size} bytes, device tree {dtb_size} bytes'


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    d = sys.argv[1]
    p = lambda name: os.path.join(d, name)
    out = []
    if os.path.isfile(p('props.txt')):
        props = dict(re.findall(r'\[([^\]]+)\]: \[([^\]]*)\]', open(p('props.txt'), errors='replace').read()))
        for key in ('ro.product.device', 'ro.bootloader', 'ro.boot.slot_suffix', 'ro.build.version.release',
                    'ro.build.version.security_patch', 'ro.boot.flash.locked', 'ro.boot.verifiedbootstate'):
            out.append(f'{key}: {props.get(key, "(not set)")}')
        if props.get('ro.boot.flash.locked') == '1':
            out.append('  !! the bootloader says it is LOCKED: fastboot boot will be refused')
    for name, fn in (('boot', boot_header), ('vendor_boot', vendor_boot_header)):
        f = p(f'{name}.head')
        out.append(f'{name}: ' + (fn(f) if os.path.isfile(f) and os.path.getsize(f) else '(not pulled: needs root)'))
    for log in ('bldrlog.txt', 'last_bldrlog.txt'):
        f = p(log)
        if not (os.path.isfile(f) and os.path.getsize(f)):
            out.append(f'{log}: (not pulled)')
            continue
        lines = open(f, errors='replace').read().splitlines()
        hits = [l.strip() for l in lines if re.search(r'(?i)splash|framebuf|\bfb\b|fb_|fbaddr|fb addr|decon|display|lcd|panel', l)]
        out.append(f'{log}: {len(lines)} lines; the ones about the display:')
        out += [f'  {h[:200]}' for h in hits[:40]] or ['  (none)']
        addrs = sorted(set(re.findall(r'0x(?:f[0-9a-f]{7}|[89a-e][0-9a-f]{7})\b', ' '.join(hits).lower())))
        if addrs:
            out.append(f'  addresses in them (a framebuffer candidate for zuma-akita.dts): {", ".join(addrs)}')
    if os.path.isfile(p('iomem.txt')):
        ram = [l.strip() for l in open(p('iomem.txt'), errors='replace') if 'System RAM' in l and not l.startswith(' ')]
        out.append('System RAM (/proc/iomem): ' + ('; '.join(ram) if ram else '(addresses hidden: read as root)'))
    print('\n'.join(out))


if __name__ == '__main__':
    main()
