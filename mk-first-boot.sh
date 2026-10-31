#!/bin/bash
# The Pixel 8a's first HakiOS boot (docs/porting/index.md, step 2): one boot image for `fastboot boot`, nothing flashed.
#
#   devices/akita/mk-first-boot.sh            -> build/akita/hakios-akita.img
#
# In it: HakiOS's U-Boot for akita (devices/akita/u-boot), and in the ramdisk the HakiOS loader with a payload of the
# kernel as built (make kernel) and the Apple-free `hello` (prebuilt/userland/hello) as user space, with this
# configuration's boot-arguments (configs/akita.config: one processor, no storage).
#
# What to expect on the phone, in order, all on the screen (there is no USB or log partition under U-Boot on zuma yet):
#   U-Boot's banner; "[oosexgang] hakios: the container is N bytes into the ramdisk", "payload of ...", "loader of ...";
#   the loader's "== XNULOADER v3 ==", "CNTFRQ=...", "platform akita"; the kernel's lines; "Hello from user space on
#   Darwin!"; then the phone restarts by itself (into Android). A step that is not reached is the one to look at
#   (devices/akita/README.md, "First boot").
#
# Needs: make kernel (hakios-kernel); an AArch64 GCC for U-Boot (uefi/u-boot/build.sh; LLVM's linker does not link a
# working U-Boot); u-boot/ (uefi/u-boot/apply.sh, then this device's files copied in, which this does).
set -e
here=$(cd "$(dirname "$0")" && pwd)
cd "$here/../.."
out=build/akita
mkdir -p "$out"

[ -f hakios-kernel ] || { echo "mk-first-boot.sh: no hakios-kernel: make kernel first" >&2; exit 1; }
[ -f xnuloader/identity.h ] || PLACEHOLDER=1 xnuloader/mkidentity.sh
xnuloader/build.sh >/dev/null
python3 tools/hakios-config.py check >/dev/null

args=$(python3 tools/hakios-config.py args)
xnuloader/mkpayload.py -o "$out/payload.bin" kernel=hakios-kernel ramdisk=prebuilt/userland/hello --cmdline "-v $args"

[ -d u-boot ] || uefi/u-boot/apply.sh
cp -r "$here/u-boot/files/." u-boot/          # this device's U-Boot files, as apply.sh does at the start
cp uefi/u-boot/files/cmd/hakios.c u-boot/cmd/hakios.c
uefi/u-boot/build.sh akita

python3 tools/mkubootimg.py -d akita -p "$out/payload.bin" -o "$out/hakios-akita.img"
echo
echo "mk-first-boot.sh: $out/hakios-akita.img. On a machine where fastboot reaches the phone (Windows' platform-tools):"
echo "  adb reboot bootloader"
echo "  fastboot boot $out/hakios-akita.img"
echo "Nothing is flashed: a restart gives the phone back to Android."
