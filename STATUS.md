# Pixel 8a (akita): what works

HakiOS on the Google Pixel 8a (Tensor G3, "zuma"): XNU and the Darwin 16.5 (iOS) user space, started from RAM over
USB. Nothing is flashed; a restart gives the phone back to Android. Last updated 2026-11-01.

How to run it: `the port's log`. What is next and how: `the port's log`. What was done when: `the port's log`.

| | |
|---|---|
| ✅ | works, seen on the phone |
| 🟡 | partly, or works but not yet seen in full |
| ❌ | not done |

## Summary

| Part | State | Notes |
|---|---|---|
| Boot (U-Boot → loader → XNU) | ✅ | `fastboot boot` of a RAM image; Google's bootloader is untouched |
| iOS user space | ✅ | Darwin 16.5: launchd, backboardd, SpringBoard to the home screen; apps open |
| Display | 🟡 | the bootloader's framebuffer, drawn in software: ~10 frames a second |
| Touch | ✅ | taps, swipes, the app switcher |
| Buttons (power, volume) | 🟡 | driver found the keys, no false presses; a real press not yet seen |
| CPU | 🟡 | 8 of 9 cores; no frequency scaling (runs at the bootloader's clocks) |
| USB | ✅ | the host side for development (`xl`): 44-46 MB/s uploads, screenshots, logs |
| Watchdog | ✅ | fed by the kernel; a hung phone restarts itself in ~2.5 min |
| Clock (time of day) | 🟡 | correct, but set from the computer at each run (`xl`), not read from the phone |
| Storage (UFS) | ❌ | the system runs from RAM; nothing is kept between runs |
| GPU (Mali-G715) | ❌ | software drawing only |
| Battery / charging info | ❌ | the battery icon is empty; the phone does not charge while in HakiOS |
| Audio | ❌ | behind Google's AoC (always-on processor) |
| Wi-Fi / Bluetooth | ❌ | Broadcom on PCIe |
| Modem / cellular | ❌ | Samsung's modem: last on the list |
| Camera, haptics, fingerprint, sensors | ❌ | |
| Thermal | ❌ | no throttling either: keep runs short-ish |

## In more detail

**Boot.** Google's bootloader (`fastboot boot`) starts HakiOS's U-Boot from an LZ4 boot image with an unsigned AVB
footer. U-Boot starts the HakiOS loader, which enters XNU at EL1 from the EL2 the bootloader leaves. About 35 s from
`xl enter` to the loader answering.

**iOS user space.** The images come from your own iPhone X 16.5 IPSW (`iPhone10,3,iPhone10,6_16.5_20F66_Restore.ipsw`;
`sysimage/unpack.sh`). The 2.4 GB system image is uploaded to RAM each session (~53 s). SpringBoard is up about a
minute after the kernel starts; ~160 processes run. iOS thinks it is an iPhone X (1125x2436 at 3x). The picture is
fitted to the 1080x2400 panel with nothing visibly cut off.

**Display.** The bootloader's framebuffer at 0xfac00000. The panel stays in command mode, with the trigger unmasked
by U-Boot. Every frame is drawn by the CPU, ~10 a second, so it works but is slow. Kernel text over the picture with
`-v`.

**Touch.** Goodix GT9895 on SPI (zuma HSI0 USI2), polled 125 times a second (`touchscreen=1`). U-Boot powers the chip;
the kernel driver sets its clock and reads it.

**Buttons.** Power gpa4-0, volume down gpa4-1, volume up gpa6-2 (`buttons=1`), mapped like the iPhone X's side and
volume buttons. Read only.

**CPU.** 8 cores (4 small, then the middle ones). The 9th, the Cortex-X3, is not used (the kernel's limit is 8).
No frequency or voltage control: that is ACPM's job (Google's power firmware), and the kernel has no ACPM client yet
(U-Boot has one, `acpm`).

**USB.** The development link only: `xl` (enter, run, ramdisk, screenshot, log, reboot) from Windows/WSL with usbipd.
No USB for iOS itself (no data, no charging).

**Storage (UFS).** Not started. The system runs from RAM, so settings and app data are gone at every restart. Next
on the plan: zuma's UFS host, read only first.

**GPU.** Mali-G715 with its CSF firmware: the largest single job.

**Battery, audio.** Both go through Google's AoC and power firmware, not plain registers.

**Modem, Wi-Fi, Bluetooth, camera.** Not started.

## Known oddities

- A second kernel run in one loader session dies at once: restart the phone between runs (`./xl reboot`), as on the
  reference phone.
- `-v` puts the kernel's text over the iOS screen; leave it off for clean screenshots.
- The phone does not charge while in the loader or in HakiOS: send it back to Android between sessions.
