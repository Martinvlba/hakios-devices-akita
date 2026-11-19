#!/usr/bin/env python3
"""The Pixel 8a modem's carrier configuration for the HakiOS image: Android's /vendor/firmware/carrierconfig (the vendor's;
a copy in ../fw-modem-akita/carrierconfig, never in the repository) packed into one file the kernel's modem driver reads
(drivers/modem/hakios.akita_modem.c): which manifest a SIM's network takes, and the manifests themselves.

The modem tells the phone whose card it has (an indication with the IMSI); the phone answers with the directory and the
SHA-1 name of that carrier's manifest; the modem then reads /vendor/firmware/carrierconfig/manifests/<name> through its
file service. Which manifest: cfg.db -- carrier_info (the network's number -> a carrier), confmap (the carrier -> its
manifest), carrier 0 the one for everybody else.

usage: mkcarrierpack.py CARRIERCONFIG_DIR OUT.pack
OUT: "HKCC0001"; u32 networks, u32 files (manifests and configuration sequences); per network { char number[8] (zero padded), u8 name[20] }, the first being
"*" (the default); per manifest { u8 name[20], u32 offset, u32 size }; the manifests' bytes. Little-endian.
Only the rule for a whole network is taken (no IMSI range, SPN or GID variants yet)."""
import os
import sqlite3
import struct
import sys


def main():
    src, out = sys.argv[1], sys.argv[2]
    db = sqlite3.connect(os.path.join(src, 'cfg.db'))
    confman = {str(c): m for c, m in db.execute('select carrier_id, confman from confmap')}
    parent = {str(c): str(p) for c, p in db.execute('select carrier_id, parent_id from carrier_parent')}
    nets = {}
    for cid, mccmnc in db.execute("select carrier_id, mccmnc from carrier_info where imsi_prefix_xpattern = '%' and spn = '%' "
                                  "and gid1 = '%' and gid2 = '%' order by carrier_id"):
        c = str(cid)
        while c not in confman and c in parent:
            c = parent[c]
        if c in confman and mccmnc not in nets:
            nets[mccmnc] = confman[c]
    # the manifests and the configuration sequences they name (confseqs/): both are files named by their SHA-1, asked
    # for by path -- one table of names serves both directories
    files = {}
    for d in ('manifests', 'confseqs'):
        for n in os.listdir(os.path.join(src, d)):
            if len(n) == 40 and not os.path.islink(os.path.join(src, d, n)) or os.path.isfile(os.path.join(src, d, n)):
                files[n] = open(os.path.join(src, d, n), 'rb').read()
    names = sorted(n for n in files if len(n) == 40)
    blobs = [files[n] for n in names]
    table = [('*', confman['0'])] + sorted(nets.items())
    head = b'HKCC0001' + struct.pack('<II', len(table), len(names))
    for number, name in table:
        head += number.encode().ljust(8, b'\0') + bytes.fromhex(name)
    at = len(head) + 28 * len(names)
    for n, b in zip(names, blobs):
        head += bytes.fromhex(n) + struct.pack('<II', at, len(b))
        at += len(b)
    open(out, 'wb').write(head + b''.join(blobs))
    print('%s: %d networks, %d files, %d bytes' % (out, len(table), len(names), at))


if __name__ == '__main__':
    main()
