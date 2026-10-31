#!/usr/bin/env python3
"""Print the zuma (Google Tensor G3) block addresses of the Pixel 8a (akita), from its own device tree.

usage: devices/akita/soc/zuma-addrs.py DEVICE-TREE.dtb [> devices/akita/soc/zuma.addrs]

NOTES, NOT BOOT-ARGUMENTS. No driver of the kernel reads any of these names: the common repository's
drivers are Qualcomm's (tools/soc-args.py prints the soc_* they do read). These are the addresses a zuma
driver of this device's own will need, read out of the phone's tree so that the head of such a driver can
say where its numbers came from. Do not put them in devices/akita/soc.args: the run scripts add that file
to the kernel's command line, which has room for 607 characters (tools/soc-only-changes.sh).

Every address is read with the #address-cells of the node's parent, so a tree with 1 or 2 address cells
reads right. Reserved-memory regions are looked up under /reserved-memory only (the tree also has an
/aoc@17000000 that is not the AoC's memory). A name whose node is not in the tree is said so on stderr.
Only numbers are read out; no part of the tree is copied.
"""
import struct
import sys

FDT_MAGIC = 0xd00dfeed


def nodes_of(blob):
    """Every node of a flattened device tree: {path: {property: bytes}}."""
    magic, _, off_struct, off_strings = struct.unpack('>IIII', blob[:16])
    if magic != FDT_MAGIC:
        sys.exit('zuma-addrs.py: not a flattened device tree')
    out, path, at = {}, [], off_struct
    while True:
        token = struct.unpack('>I', blob[at:at + 4])[0]
        at += 4
        if token == 1:                                  # FDT_BEGIN_NODE
            end = blob.index(b'\0', at)
            path.append(blob[at:end].decode())
            at = (end + 4) & ~3
            out['/'.join(path) or '/'] = {}
        elif token == 2:                                # FDT_END_NODE
            path.pop()
        elif token == 3:                                # FDT_PROP
            length, name_off = struct.unpack('>II', blob[at:at + 8])
            at += 8
            name = blob[off_strings + name_off:blob.index(b'\0', off_strings + name_off)].decode()
            out['/'.join(path) or '/'][name] = blob[at:at + length]
            at = (at + length + 3) & ~3
        elif token == 4:                                # FDT_NOP
            continue
        elif token == 9:                                # FDT_END
            return out
        else:
            sys.exit(f'zuma-addrs.py: unexpected token {token}')


def parent(path):
    return path.rsplit('/', 1)[0] or '/'


def address_cells(nodes, path):
    """The #address-cells that applies to `path`'s reg: its parent's (2 when the parent does not say)."""
    value = nodes.get(parent(path), {}).get('#address-cells')
    return struct.unpack('>I', value)[0] if value else 2


def first_address(nodes, path):
    reg = nodes[path].get('reg', b'')
    cells = address_cells(nodes, path)
    if len(reg) < cells * 4:
        return None
    value = 0
    for i in range(cells):
        value = (value << 32) | struct.unpack_from('>I', reg, i * 4)[0]
    return value


def by_compatible(nodes, compatible):
    for path, props in nodes.items():
        if compatible.encode() in props.get('compatible', b'').split(b'\0') and 'reg' in props:
            return path
    return None


def by_name(nodes, leaf, under=None):
    for path in nodes:
        if under is not None and parent(path) != under:
            continue
        if path.rsplit('/', 1)[-1].split('@', 1)[0] == leaf and 'reg' in nodes[path]:
            return path
    return None


# name, how it is found, what
BLOCKS = [
    ('zuma_chipid',        ('compatible', 'google,zuma-chipid'),      'chip id'),
    ('zuma_gsa',           ('compatible', 'google,gs101-gsa-v1'),     'GSA (security processor) mailbox'),
    ('zuma_pinctrl',       ('compatible', 'google,zuma-pinctrl'),     'first of the 13 pin controllers'),
    ('zuma_cmu',           ('compatible', 'samsung,zuma-clock'),      'clock management unit'),
    ('zuma_pmu',           ('compatible', 'samsung,gs101-pmu'),       'power management unit'),
    ('zuma_gpu',           ('compatible', 'arm,malit6xx'),            'Mali G3D GPU'),
    ('zuma_tpu',           ('compatible', 'google,edgetpu-zuma'),     'Edge TPU'),
    ('zuma_decon',         ('name', 'drmdecon'),                      'first display controller (DECON0)'),
    ('zuma_dsim',          ('name', 'drmdsim'),                       'first DSI host (DSIM0)'),
    ('zuma_sysmmu_pcie',   ('compatible', 'samsung,pcie-sysmmu'),     'SysMMU of the PCIe block'),
    ('zuma_gsa_mem',       ('reserved', 'gsa'),                       'reserved memory: GSA'),
    ('zuma_tpu_fw_mem',    ('reserved', 'tpu_fw'),                    'reserved memory: TPU firmware'),
    ('zuma_aoc_mem',       ('reserved', 'aoc'),                       'reserved memory: always-on compute'),
    ('zuma_pkvm_fw_mem',   ('reserved', 'pkvm_guest_firmware'),       'reserved memory: pKVM guest firmware'),
]


def main():
    sys.stdout.reconfigure(newline='\n')        # LF on Windows too
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    nodes = nodes_of(open(sys.argv[1], 'rb').read())
    found = 0
    print(f'# zuma block addresses, by devices/akita/soc/zuma-addrs.py from {sys.argv[1]}: notes, not boot-arguments')
    for name, (how, key), what in BLOCKS:
        if how == 'compatible':
            path = by_compatible(nodes, key)
        elif how == 'name':
            path = by_name(nodes, key)
        else:
            path = by_name(nodes, key, under='/reserved-memory')
        address = first_address(nodes, path) if path else None
        if address is None:
            print(f'zuma-addrs.py: {name}: no node in the tree ({how} {key})', file=sys.stderr)
            continue
        print(f'{name}={address:#x}\t# {what}: {path}')
        found += 1
    if not found:
        sys.exit('zuma-addrs.py: found none of the expected nodes; is this the Pixel 8a\'s own device tree?')


if __name__ == '__main__':
    main()
