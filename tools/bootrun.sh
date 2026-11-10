#!/bin/bash
# build user space (JOBFILE= job file), make a boot image, boot it once with fastboot boot (nothing flashed), log for
# SECS seconds into LOGDIR (default ~/akita/logs/run: build-boot.log, test-console.log, test-k.log). Extra boot-args in
# BOOTARGS (usb_lost_reboot=90 after them: a phone that loses its USB link restarts into the active slot, Android;
# usb_lost_reboot=0 in BOOTARGS turns it off -- the first one counts; test_reboot_after=600 too: a test build restarts
# after ten minutes whatever happens, test_reboot_after=0 in BOOTARGS for a longer run); THERMAL= overrides "thermal=1 dvfs=1". Never set RC or JOBS (userland/build.sh reads them).
cd ~/akita/hackios-kernel; D=~/akita/darwin-flash; S=${LOGDIR:-$HOME/akita/logs/run}; mkdir -p $S
# GPU=1, the default unless the job file is mali_draw_test's own: CoreAnimation on the Mali (METAL_PLUGIN=ca with the
# iPhone 11's library from ~/ca-share), the panel at 120 Hz, zero-copy scan-out. GPU=0: iOS draws in software -- what the
# Wi-Fi/USB runs of 11-09 did by accident, 10-25 fps, the big cores at 75 C (the tester's video of 23:12 was one).
# (The GPU arguments take ~140 of the 495 characters: 453 with them alone; GPU=0 when a test needs the room.)
case "${JOBFILE:-rc.jobs.maldraw}" in rc.jobs.maldraw) GPU=${GPU-0};; *) GPU=${GPU-1};; esac
if [ "$GPU" = 1 ]; then
  export METAL_PLUGIN=${METAL_PLUGIN-ca} EXTRA_SHARE=${EXTRA_SHARE-$HOME/ca-share}
  case "$BOOTARGS" in *ca_metallib=*) ;; *) BOOTARGS="ca_metallib=/redwood/share/ca-share/QuartzCore-default.metallib display_hz=120 fbwidth=1080 fbheight=2340 fbscale=3 fbsubtype=2340 fb_zc=1 $BOOTARGS";; esac
fi
env -u RC -u JOBS BIG_IN_ROOT=1 RC_JOBS=${JOBFILE:-rc.jobs.maldraw} HAKIOS_METAL_OUT=ios-mali LOCKDOWND=1 CRYPTEXES=flash FLASHBOOT=1 REST=1 userland/build.sh > $S/build-boot.log 2>&1 || { echo build failed; exit 1; }
# the safety arguments first (a long command line loses its end: the loader keeps 607 characters, ~110 of them its own),
# unless BOOTARGS gives them itself
SAFE=""; case "$BOOTARGS" in *usb_lost_reboot=*) ;; *) SAFE="usb_lost_reboot=90";; esac
# the ten-minute restart always (the tester, 11-10 01:35: back on as a failsafe while nobody watches; test_reboot_after=0
# in BOOTARGS for a longer run by hand)
TRA=600
case "$BOOTARGS" in *test_reboot_after=*) ;; *) SAFE="$SAFE test_reboot_after=$TRA";; esac
CMDLINE="debug=0x8 $SAFE cpus=9 touchscreen=1 buttons=1 fuelgauge=1 usbmux=1 usb_serial=d6ca5f8da676e44a1ed9455d72d070481d33dd9e ${THERMAL-thermal=1 dvfs=1} root_apfs=ufs_darwin ufs=2 ufs_write=1 var_hfs=ufs_darwin_data mali_keep=1 metal=1 metal_plugin=../../../redwood/lib/HakiosMetal display_flip=1 $BOOTARGS"
[ ${#CMDLINE} -gt 495 ] && { echo "bootrun: the command line is ${#CMDLINE} characters: more than the loader keeps with its own (~495 of 607); shorten BOOTARGS"; exit 1; }
xnuloader/mkpayload.py -o $D/test-payload.bin kernel=hakios-kernel ramdisk=userland/root.apfs resident=userland/flashboot.apfs --cmdline "$CMDLINE" >/dev/null && tools/mkubootimg.py -d akita -p $D/test-payload.bin -o $D/test.img >/dev/null || exit 1
# to the bootloader from wherever the phone is -- and again if it turns up elsewhere meanwhile (a test build's own
# restart, usb_lost_reboot or test_reboot_after, lands in Android: a run started then waited for fastboot for ever)
for i in $(seq 1 90); do
  fastboot devices 2>/dev/null | grep -q . && break
  if adb devices 2>/dev/null | grep -q "device$"; then adb reboot bootloader >/dev/null 2>&1; sleep 8
  elif adb -P 5039 devices 2>/dev/null | grep -q "device$"; then adb -P 5039 reboot bootloader >/dev/null 2>&1; sleep 8   # (the second adb server: the port's log)
  elif lsusb | grep -q 05ac; then timeout 30 python3 devices/akita/tools/usbmux-reboot.py bootloader >/dev/null 2>&1; sleep 8
  else sleep 2; fi
done
fastboot devices 2>/dev/null | grep -q . || { echo "no fastboot device (adb: $(adb devices | grep -c 'device$'), usb: $(lsusb | grep -o '05ac:[0-9a-f]*\|18d1:[0-9a-f]*'))"; exit 1; }
timeout 120 fastboot boot $D/test.img >/dev/null 2>&1 || { echo boot failed; exit 1; }
for i in $(seq 1 60); do lsusb | grep -q 05ac && break; sleep 2; done
rm -f $S/test-console.log $S/test-k.log
for i in $(seq 1 20); do timeout 2 python3 devices/akita/tools/usbmux-log.py /dev/null >/dev/null 2>&1; [ $? -eq 124 ] && break; sleep 2; done
timeout ${SECS:-90} python3 devices/akita/tools/usbmux-log.py $S/test-console.log >/dev/null 2>&1 &
timeout ${SECS:-90} python3 devices/akita/tools/usbmux-log.py -k $S/test-k.log >/dev/null 2>&1
wait
echo "usb now: $(lsusb | grep -o '05ac:[0-9a-f]*\|18d1:[0-9a-f]*')"
