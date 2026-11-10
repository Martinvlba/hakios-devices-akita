#!/bin/bash
# rec-trans.sh LOGDIR OUTDIR: while a run with rc.jobs.trans (and boot-arg rec=N) goes, fetch the screen recording each
# time the job starts one of its "wait 20" pauses -- the ring then holds the action just done: OUTDIR/rec-N.txt (rec-analyze.py's
# report) and sus-N.png, the recording itself kept only with KEEP=1.
log=$1/test-console.log; out=$2; n=0; mkdir -p "$out"
for i in $(seq 1 400); do
  c=$(grep -a -c "rwtouch wait 20" "$log" 2>/dev/null); c=${c:-0}
  if [ "$c" -gt "$n" ]; then
    n=$c; sleep 1
    timeout 120 python3 "$(dirname "$0")/usbmux-rec.py" "$out/rec-$n.rec" | tail -1
    # (each recording is 256 MB: read, its suspects kept as a sheet, then let go of unless KEEP=1)
    python3 "$(dirname "$0")/rec-analyze.py" "$out/rec-$n.rec" "$out/sus-$n.png" > "$out/rec-$n.txt" 2>&1
    [ -n "$KEEP" ] || rm -f "$out/rec-$n.rec"
    [ "$n" -ge 4 ] && exit 0
  fi
  sleep 1
done
