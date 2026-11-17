#!/usr/bin/env bash
#
# akita-usb.sh -- robust WSL <-> akita USB/usbmux helper.
#
# Encapsulates the connection dance that bites on WSL+usbipd: the busid changes on every replug, iTunes /
# Apple Mobile Device Service on Windows grabs the device ("used by Windows"), and usbmuxd drops ("No device
# found" / Mux error). SE-SAFE: if more than one Apple device is connected it refuses to act and asks for an
# explicit busid, so it can never grab the tester's real iPhone.
#
# Usage:
#   akita-usb.sh status            -- what's connected, attached, mux/pairing state, device mode
#   akita-usb.sh attach [BUSID]    -- bind + attach the akita Apple device to WSL (force past iTunes)
#   akita-usb.sh mux               -- clean usbmuxd restart, wait for lockdownd pairing
#   akita-usb.sh recover [BUSID]   -- attach + mux (the usual "get me talking to the phone")
#   akita-usb.sh klog [DIR]        -- pull the newest klogsave kernel log via afcclient (DIR default ~/akita/logs)
#   akita-usb.sh android           -- stop usbmuxd so adb/fastboot can own the device (no more usbmux-vs-adb fight)
#   akita-usb.sh ios               -- bring usbmux back when the phone is in iOS again
# Needs: usbipd.exe (Windows), sudo for usbmuxd (no password is stored here), afcclient/idevicepair.
# usbmuxd here is the HakiOS fork (~/akita/usbmuxd-hakios): bounded disconnect (no hangs), Apple
# mode-switch disabled, built --without-preflight. Reverts to stock via the systemd drop-in.
set -uo pipefail

usbipd() { usbipd.exe "$@" 2>&1; }

apple_busids() { usbipd list 2>/dev/null | grep -E '^[0-9]+-[0-9]+' | grep -i '05ac' | awk '{print $1}'; }

device_mode() {
	local u; u=$(lsusb 2>/dev/null | grep -oE '05ac:[0-9a-f]+|18d1:4ee[0-9a-f]')
	case "$u" in
		05ac:*) echo "iOS" ;;
		18d1:4ee0) echo "fastboot" ;;
		18d1:*) echo "android" ;;
		*) echo "none-in-wsl" ;;
	esac
}

cmd_status() {
	echo "== usbipd (Apple devices) =="
	usbipd list 2>/dev/null | grep -iE 'BUSID|05ac' | grep -vE '^$'
	echo "== WSL =="
	echo "lsusb   : $(lsusb 2>/dev/null | grep -oE '05ac:[0-9a-f]+|18d1:[0-9a-f]+' | paste -sd' ' -)"
	echo "mode    : $(device_mode)"
	echo "usbmuxd : $(systemctl is-active usbmuxd 2>/dev/null)"
	echo -n "pairing : "; timeout 8 idevicepair validate 2>&1 | tail -1
}

pick_busid() {
	local want="${1:-}"
	if [ -n "$want" ]; then echo "$want"; return 0; fi
	local ids; ids=$(apple_busids); local n; n=$(echo "$ids" | grep -c .)
	if [ "$n" -eq 0 ]; then echo "akita-usb: no Apple device in usbipd list" >&2; return 1; fi
	if [ "$n" -gt 1 ]; then
		echo "akita-usb: $n Apple devices connected -- REFUSING to guess (SE safety). Pass the busid:" >&2
		usbipd list 2>/dev/null | grep -i '05ac' >&2
		return 1
	fi
	echo "$ids"
}

cmd_attach() {
	local b; b=$(pick_busid "${1:-}") || return 1
	echo "akita-usb: busid $b"
	usbipd bind --busid "$b" | grep -viE 'already|^$' || true
	local out; out=$(usbipd attach --wsl --busid "$b")
	echo "$out" | tail -1
	if echo "$out" | grep -qi 'used by Windows'; then
		echo "akita-usb: held by Windows (iTunes) -- forcing (safe: single Apple device = akita)"
		usbipd bind --force --busid "$b" | tail -1
		usbipd attach --wsl --busid "$b" | tail -1
	fi
	local i; for i in $(seq 1 15); do lsusb | grep -q 05ac && { echo "akita-usb: attached ($(device_mode))"; return 0; }; sleep 1; done
	echo "akita-usb: attach did not show in lsusb (device may be off/rebooting)"; return 1
}

cmd_mux() {
	echo "akita-usb: resetting usbmuxd"
	sudo sh -c 'pkill -f usbmuxd 2>/dev/null; systemctl reset-failed usbmuxd 2>/dev/null; systemctl restart usbmuxd' || return 1
	local i; for i in $(seq 1 20); do
		[ "$(systemctl is-active usbmuxd)" = active ] && idevicepair validate >/dev/null 2>&1 && { echo "akita-usb: paired, usbmux ready"; return 0; }
		sleep 2
	done
	echo "akita-usb: usbmux up but not paired (tap Trust on first pairing of a fresh /var, or replug)"; return 1
}

cmd_klog() {
	local dir="${1:-$HOME/akita/logs}"; mkdir -p "$dir"
	local f; f=$(afcclient ls hakios-klog 2>/dev/null | grep -E '^kernel_' | tail -1)
	[ -n "$f" ] || { echo "akita-usb: no klogsave found (is the phone up in iOS + paired? run 'mux')"; return 1; }
	local out="$dir/$f"
	timeout 40 afcclient get "hakios-klog/$f" "$out" >/dev/null 2>&1 && echo "akita-usb: $out ($(wc -l <"$out") lines)" || echo "akita-usb: pull failed"
}

# Hand the device to adb/fastboot: fully stop usbmuxd so it cannot claim the Pixel while it is in
# Android/fastboot mode (the old "usbmux breaks adb" conflict). Safe to run any time.
cmd_android() {
	echo "akita-usb: stopping usbmuxd so adb/fastboot can own the device"
	sudo systemctl stop usbmuxd.socket usbmuxd.service 2>/dev/null
	sudo pkill -x usbmuxd 2>/dev/null
	sleep 1
	echo "akita-usb: usbmuxd stopped ($(pgrep -xc usbmuxd) running); mode now: $(device_mode)"
	echo "akita-usb: run 'akita-usb.sh ios' to bring usbmux back when the phone returns to iOS"
}

# Bring usbmux back (HakiOS fork) once the phone is in iOS again.
cmd_ios() {
	case "$(device_mode)" in
		iOS|none-in-wsl) ;;
		*) echo "akita-usb: device is in $(device_mode) mode, not iOS -- refusing to start usbmux (SE safety)"; return 1;;
	esac
	sudo systemctl start usbmuxd.socket 2>/dev/null
	cmd_mux
}

case "${1:-status}" in
	status) cmd_status ;;
	attach) cmd_attach "${2:-}" ;;
	mux) cmd_mux ;;
	recover) cmd_attach "${2:-}" && cmd_mux ;;
	klog) cmd_klog "${2:-}" ;;
	android|adb|fastboot) cmd_android ;;
	ios) cmd_ios ;;
	*) sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
