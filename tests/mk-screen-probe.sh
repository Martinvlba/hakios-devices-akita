#!/bin/bash
# Build devices/akita/tests/screen-probe.S into build/akita/screen-probe.img, a boot image for `fastboot boot` made the
# way the phone's own is (devices/akita/u-boot/boot-image): header version 4, the kernel LZ4-framed, no ramdisk.
set -e
here=$(cd "$(dirname "$0")" && pwd)
cd "$here/../../.."
out=build/akita
mkdir -p "$out"
aarch64-linux-gnu-gcc -nostdlib -static -Wl,-Ttext=0 -Wl,--build-id=none -o "$out/screen-probe.elf" "$here/screen-probe.S"
aarch64-linux-gnu-objcopy -O binary "$out/screen-probe.elf" "$out/screen-probe.bin"
python3 tools/mkubootimg.py -d akita -u "$out/screen-probe.bin" --no-ramdisk -o "$out/screen-probe.img"
for p in off hang; do
	P=$(echo $p | tr a-z A-Z)
	aarch64-linux-gnu-gcc -nostdlib -static -Wl,-Ttext=0 -Wl,--build-id=none -DPROBE_$P -o "$out/power-$p.elf" "$here/power-probe.S"
	aarch64-linux-gnu-objcopy -O binary "$out/power-$p.elf" "$out/power-$p.bin"
	python3 tools/mkubootimg.py -d akita -u "$out/power-$p.bin" --no-ramdisk -o "$out/power-$p.img"
done
