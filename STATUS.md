# Pixel 8a (akita): what works

HakiOS on the Google Pixel 8a (Tensor G3, "zuma"): XNU and the Darwin 16.5 (iOS) user space, started from RAM over
USB. Nothing is flashed; a restart gives the phone back to Android. Last updated 2026-11-02.

How to run it: `the port's log`. What is next and how: `the port's log`. What was done when: `the port's log`.

| | |
|---|---|
| ✅ | works, seen on the phone |
| 🟡 | partly, or works but not yet seen in full |
| ❌ | not done |

## Summary

| Part | State | Notes |
|---|---|---|
| Boot (U-Boot → loader → XNU) | ✅ | installed in boot_b (slot b = iOS, slot a = Android), or `fastboot boot` of a RAM image |
| iOS user space | ✅ | Darwin 16.5: launchd, backboardd, SpringBoard to the home screen; apps open |
| Display | 🟡 | the bootloader's framebuffer, drawn in software: ~10 frames a second |
| Touch | ✅ | taps, swipes, the app switcher |
| Buttons (power, volume) | ✅ | volume HUD, power locks and wakes (`buttons=1`) |
| CPU | ✅ | all 9 cores (`cpus=9`: 4 A510, 4 A715, the X3); clocks set by a governor through ACPM (`dvfs=1`), held down when hot (the X3 from 75 C) |
| USB | ✅ | as an iPhone's in iOS (`usbmux=1`): pairs with a computer (Trust This Computer?), ideviceinfo, idevicesyslog, files over AFC (~7 MB/s); and the development link (`xl`) under the loader |
| Watchdog | ✅ | fed by the kernel; a hung phone restarts itself in ~2.5 min |
| Clock (time of day) | 🟡 | correct, but set from the computer at each run (`xl`), not read from the phone |
| Storage (UFS) | ✅ | iOS runs from the phone's own flash (a 77 GiB darwin partition); /private/var kept between starts (71 GB; iPhone Storage shows 80 GB) |
| GPU (Mali-G715) | ❌ | software drawing only |
| Battery / charging info | ✅ | level, charging or not, in the status bar (`fuelgauge=1`: the MAX77759 read over I2C); the phone charges in iOS (63 → 64 % seen) |
| Audio | ❌ | behind Google's AoC (always-on processor) |
| Wi-Fi / Bluetooth | ❌ | Broadcom on PCIe |
| Modem / cellular | ❌ | Samsung's modem: last on the list |
| Camera, haptics, fingerprint, sensors | ❌ | |
| Thermal | ✅ | the SoC's 7 sensors read (`thermal=1`), iOS's thermal state from them, clocks held down from 90 C, the phone powered off at 100 C; ~45 C at rest |

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
volume buttons. Read only. In iOS: the volume HUD, power locks and wakes the screen.

**CPU.** All 9 cores: 4 Cortex-A510, 4 Cortex-A715 and the Cortex-X3 (the kernel's MAX_CPUS is 16 now). Their clocks
are set through ACPM (Google's power firmware) by the kernel's governor, `dvfs=1`: LITTLE 324-1704 MHz, MID 402-2367, BIG
500-2914; at rest all near their lowest, about 15 % of one core busy in all and 45 C.

**USB.** In iOS the port is an iPhone's: the kernel drives zuma's DWC3 as a device (05ac:12a8) and speaks the phone's
side of usbmux, lockdownd runs, and a computer pairs with it ("Trust This Computer?" on the phone) and uses its
services with libimobiledevice (usbmuxd, ideviceinfo, idevicesyslog, afcclient). The pairing and what is written over
AFC stay on the flash (the data volume is synced every 10 s). Not yet: screenshots, app install, Finder/iTunes on
Windows (Apple's driver there; usbipd takes the device for WSL). Under the loader the port is `xl`'s (enter, run,
ramdisk, screenshot, log, reboot). Charging works either way.

**Storage (UFS).** iOS is installed on the flash: a 77 GiB `darwin` partition (taken from userdata; Android keeps
32 GiB) holds the root volume and cryptexes, and Darwin boots from it with `/private/var` kept on the flash between starts
(the port's log 4). Starting without the PC is next.

**GPU.** Mali-G715 with its CSF firmware: the largest single job.

**Battery.** The MAX77759's fuel gauge (0x36) and charger (0x69) on zuma's hsi2c@10cb0000, read every 5 s by
`drivers/power/hakios.akita_battery.c` and given to Darwin's power source (HakiosBattery): the status bar's level and
bolt, "Charging" on the lock screen. Read only: the charger runs as the bootloader left it (charging on), and it does
charge, though slowly from a computer's port while the screen is on and nothing throttles.

**Audio.** Through Google's AoC firmware, not plain registers.

**Modem, Wi-Fi, Bluetooth, camera.** Not started.

## Known oddities

- Shut Down or Restart from iOS never completes: the spinner stays (the port's log 4). Use Power + Volume Down to leave.

- A second kernel run in one loader session dies at once: restart the phone between runs (`./xl reboot`), as on the
  reference phone.
- `-v` puts the kernel's text over the iOS screen; leave it off for clean screenshots.
- On a computer's USB port the phone charges in iOS only slowly; left on overnight with the screen on, the battery once
  ran flat. Keep it in Android between sessions, or on a wall charger.
