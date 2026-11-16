#!/bin/bash
# The Maps zoom flicker, tested without a hand on the phone: a RAM test boot (bootrun.sh, nothing flashed) with a job
# file that unlocks, opens Maps and pinches in and out six times (JOBFILE, default ~/akita/local-jobs/rc.jobs.mapszoom:
# local, it holds the passcode's taps), the ring recording (rec=4) fetched once the pinches are over, its frames
# compared (rec-reverts.py): a frame that goes back to the picture before last is the flicker. Exit 1 if any did.
# Logs and frames in LOGDIR (default ~/akita/logs/mapszoom). The phone is left in the test boot (it restarts into the
# bootloader by itself after ten minutes, QUIET=1's restart_fastboot=1), unless FLASHED_BACK=1: then it is rebooted
# into the installed image at the end.
cd ~/akita/hackios-kernel || exit 1
S=${LOGDIR:-$HOME/akita/logs/mapszoom}; mkdir -p "$S"
JOB=${JOBFILE:-$HOME/akita/local-jobs/rc.jobs.mapszoom}
[ -f "$JOB" ] || { echo "maps-flicker-test: no job file $JOB"; exit 1; }
# the pinches begin ~85 s after the boot (the job's waits: 50 unlock, 8, 20 for the tiles) and are over 20 s later;
# bootrun logs for SECS seconds from the first console line and then returns
# the gestures' end: UPS_NEEDED lines of UP_PATTERN on the console (default: six pinches' second finger lifted;
# a job of one-finger drags: UP_PATTERN="tl: p 3 0 " and its count of taps and swipes in all)
UP_PATTERN=${UP_PATTERN:-"tl: p 3 1 "}; UPS_NEEDED=${UPS_NEEDED:-6}
export JOBFILE="$JOB" QUIET=1 SECS=${SECS:-60} LOGDIR="$S"
export BOOTARGS="rec=4 touch_lat=1 torch=0 haptic=0 ${BOOTARGS:-}"
echo "maps-flicker-test: booting the test build (job $JOB)"
devices/akita/tools/bootrun.sh || { echo "maps-flicker-test: the test boot failed"; exit 1; }
# The computer's usbmuxd often loses the phone as it re-enumerates (restart it: memory of 11-14); the log is then
# empty. Either way the console is watched from here on for the pinches' samples (tl: p TYPE FINGER ...: finger 1's
# UP, type 3, ends each pinch) -- the unlock takes a while and the pinches come ~120 s after the kernel's start.
# (Only usbmuxd is restarted. Never de-authorize the USB device here: if the link is not back within 90 s the
# phone's usb_lost_reboot restarts it, and that killed a run on 11-15. If the phone left the bus entirely,
# usbipd.exe attach --wsl --busid 2-2 --auto-attach on the Windows side brings it back.)
usbmuxd_back() {
	echo "$SUDO_PASSWORD" | sudo -S sh -c 'pkill -x usbmuxd; sleep 1; usbmuxd -U usbmux' 2>/dev/null; sleep 6
}
[ -s "$S/test-console.log" ] || { echo "maps-flicker-test: no console log came: restarting usbmuxd"; [ -n "$SUDO_PASSWORD" ] && usbmuxd_back; }
timeout 200 python3 devices/akita/tools/usbmux-log.py "$S/test-console-2.log" >/dev/null 2>&1 &
WATCH=$!
for i in $(seq 1 40); do
	sleep 5
	UPS=$(cat "$S/test-console.log" "$S/test-console-2.log" 2>/dev/null | grep -a -c "$UP_PATTERN")
	[ "$UPS" -ge "$UPS_NEEDED" ] && break
done
sleep 4; kill $WATCH 2>/dev/null
cat "$S/test-console.log" "$S/test-console-2.log" 2>/dev/null | grep -a "tl: " > "$S/tl.log"
echo "maps-flicker-test: $UPS gesture ends seen in the log"
# the recording, now: the ring holds the last ~560 frames that changed the screen, the pinches among them
# (no rm -rf: last run's frames and recording are removed by name)
[ -d "$S/frames" ] && find "$S/frames" -maxdepth 1 -name '*.ppm' -delete
rm -f "$S/zoom.rec"
timeout 300 python3 devices/akita/tools/usbmux-rec.py "$S/zoom.rec" > "$S/rec-fetch.log" 2>&1 || {
	echo "maps-flicker-test: the recording could not be fetched:"; tail -3 "$S/rec-fetch.log"; exit 1; }
python3 tools/recording.py "$S/zoom.rec" "$S/zoom.mp4" --frames "$S/frames" > "$S/recording.log" 2>&1
# the pinches' window: from the first second-finger sample to the last's end, in ms of the kernel's clock
read -r FROM TO <<< "$(awk '/tl: p [0-9]+ 1 /{t=$0; sub(/.* at /,"",t); t=int(t/1000); if(!f) f=t; l=t} END{print f+0, l+2500}' "$S/tl.log")"
if [ "$FROM" = 0 ]; then
	echo "maps-flicker-test: no second finger was seen in the log (did rwtouch's pinch run?); every frame compared"
	FROM=0; TO=9999999999
fi
echo "maps-flicker-test: the pinches between $FROM and $TO ms of the kernel's time"
python3 devices/akita/tools/rec-reverts.py "$S/frames" "$FROM" "$TO" --quiet | tee "$S/reverts.txt"
RESULT=${PIPESTATUS[0]}
python3 tools/frames.py "$S/test-console.log" 2>/dev/null | grep -E "touch at (4[0-9]|5[0-9]|6[0-9])%,4[0-9]%" | head -8
if [ -n "$FLASHED_BACK" ]; then
	timeout 30 python3 devices/akita/tools/usbmux-reboot.py >/dev/null 2>&1
fi
[ "$RESULT" = 0 ] && echo "maps-flicker-test: PASS (no frame went back)" || echo "maps-flicker-test: FAIL (frames went back: $S/reverts.txt, $S/zoom.mp4)"
exit "$RESULT"
