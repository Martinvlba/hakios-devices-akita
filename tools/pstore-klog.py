#!/usr/bin/env python3
"""The HakiOS run before, read back from Android: after a restart into Android (the USB watchdog, Restart, a panic's way
back) the ramoops pmsg zone holds U-Boot's console, the loader's log and -- with pstore_klog (hakios.akita_power.c, on
by default) -- the kernel's own log, a ring with a marker line where it wrapped. This pulls the zone over adb (su),
puts the kernel's ring back in order and writes it out.

usage: pstore-klog.py [OUTDIR]      (default ~/akita/logs/android-pstore; writes pmsg-raw.bin, uboot-loader.log,
                                     kernel.log; prints the kernel log's last lines)
"""
import os
import subprocess
import sys

HEAD = b"\n== HAKIOS KERNEL LOG (pstore_klog, every 200 ms) ==\n"
MARK = b"\n~~~ HAKIOS KLOG: the older part follows ~~~\n"


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/akita/logs/android-pstore")
    os.makedirs(out, exist_ok=True)
    raw = subprocess.run(["adb", "exec-out", "su", "-c", "cat /sys/fs/pstore/pmsg-ramoops-0"],
                         capture_output=True, timeout=60).stdout
    if not raw:
        sys.exit("pstore-klog: nothing read (is the phone in Android, adb authorised, su there?)")
    open(os.path.join(out, "pmsg-raw.bin"), "wb").write(raw)
    at = raw.find(HEAD)
    before = raw if at < 0 else raw[:at]
    open(os.path.join(out, "uboot-loader.log"), "wb").write(before)
    if at < 0:
        print("pstore-klog: no HakiOS kernel log in the zone (pstore_klog off, an older kernel, or a power cycle);"
              " U-Boot's and the loader's text in uboot-loader.log")
        return
    ring = raw[at + len(HEAD):]
    # (the loader's echo of a panic, appended after the ring, if any: it begins with its own header)
    tail = b""
    k = ring.find(b"\n== KERNEL LOG (")
    if k >= 0:
        ring, tail = ring[:k], ring[k:]
    m = ring.find(MARK)
    if m >= 0:
        ring = ring[m + len(MARK):] + b"\n~~~ (the newer part) ~~~\n" + ring[:m]
    log = ring.rstrip(b"\0") + tail
    open(os.path.join(out, "kernel.log"), "wb").write(log)
    text = log.decode("utf-8", "replace").splitlines()
    print("pstore-klog: %d lines of the HakiOS kernel's log in %s/kernel.log%s; its last ones:" %
          (len(text), out, " (wrapped)" if m >= 0 else ""))
    for line in text[-25:]:
        print("  " + line[:200])


if __name__ == "__main__":
    main()
