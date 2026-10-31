#!/bin/bash
# What the Pixel 8a's own Android says about how it boots: the facts the first HakiOS boot needs and that its device
# tree does not hold. Run it on a machine where adb reaches the phone (Windows' Git Bash with platform-tools,
# docs/windows.md; not WSL2, which has no USB), with the phone in Android, USB debugging on, and root answering
# `su -c` (Magisk, KernelSU) for the parts marked (root).
#
# It only reads. Into devices/akita/boot-info/:
#   props.txt            getprop: the bootloader's version, the slot, the boot image's patch level
#   fdt.dtb              /sys/firmware/fdt again (the tree soc/zuma-addrs.py and u-boot/ were made from)
#   bldrlog.txt          the bootloader's log of this boot (/proc/bldrlog, (root)): where it put the splash screen
#   last_bldrlog.txt     ... and of the boot before
#   iomem.txt            /proc/iomem (root): the memory map Linux was given
#   boot.head            the first 8 KiB of the active slot's boot partition (root): the header and the kernel's
#   vendor_boot.head     ... and of vendor_boot: page size, where the device tree and the vendor ramdisks are
#   summary.txt          what of all that matters, read out (pull-boot-info.py)
#
# Root reads go through /data/local/tmp/hakios-info, which is made world-readable, pulled and removed again, as
# devices/redwood/pull-firmware-root.sh does. Nothing else on the phone is changed: dd only ever READS a partition.
set -e -o pipefail
here=$(cd "$(dirname "$0")" && pwd)
cd "$here/../.."
PATH=$PWD/tools/macos-bin/platform-tools:$PWD/tools/win-bin/platform-tools:$PATH
out=$here/boot-info
S=/data/local/tmp/hakios-info
adb get-state >/dev/null
mkdir -p "$out"

adb shell getprop > "$out/props.txt"
slot=$(adb shell getprop ro.boot.slot_suffix | tr -d '\r')
echo "pull-boot-info.sh: slot ${slot:-(none)}, bootloader $(adb shell getprop ro.bootloader | tr -d '\r')"

# what root can read, staged where the shell can pull it; every partition opened read-only by dd's if=
adb shell "su -c 'sh -c \"
rm -rf $S; mkdir -p $S
cat /sys/firmware/fdt > $S/fdt.dtb
cat /proc/bldrlog > $S/bldrlog.txt 2>/dev/null
cat /proc/last_bldrlog > $S/last_bldrlog.txt 2>/dev/null
cat /proc/iomem > $S/iomem.txt
dd if=/dev/block/by-name/boot$slot of=$S/boot.head bs=4096 count=2 2>/dev/null
dd if=/dev/block/by-name/vendor_boot$slot of=$S/vendor_boot.head bs=4096 count=2 2>/dev/null
chmod -R a+rX $S\"'" || echo "pull-boot-info.sh: su did not answer: only what the shell can read is pulled"
for f in fdt.dtb bldrlog.txt last_bldrlog.txt iomem.txt boot.head vendor_boot.head; do
	adb pull "$S/$f" "$out/$f" >/dev/null 2>&1 || echo "pull-boot-info.sh: $f: not read"
done
adb shell "su -c 'rm -rf $S'" >/dev/null 2>&1 || true
[ -s "$out/fdt.dtb" ] || adb pull /sys/firmware/fdt "$out/fdt.dtb" >/dev/null 2>&1 || true

python3 "$here/pull-boot-info.py" "$out" | tee "$out/summary.txt"
echo "pull-boot-info.sh: wrote $out/ (send summary.txt and bldrlog.txt along with a question)"
