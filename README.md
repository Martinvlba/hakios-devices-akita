# Device: ZUMA AKITA MP board based on ZUMA (akita)

Made by `tools/new-device.py` on 2026-10-30, from the file device-tree.dtb. Edit it as the port goes.

| What | Value |
|---|---|
| Board (the tree's `model`) | ZUMA AKITA MP board based on ZUMA |
| SoC | Google Tensor G3 (zuma): 4 x Cortex-A510, 4 x Cortex-A715, 1 x Cortex-X3; Mali GPU; Exynos-derived blocks |
| Compatible | `google,ZUMA AKITA google,ZUMA` |

## The SoC addresses

The `soc_*` boot-arguments of `tools/soc-args.py` are the Qualcomm modem's and IPA's; zuma has none of them, so
`soc.args` is empty and nothing is added to the kernel's command line. zuma's own block addresses are in
`soc/zuma.addrs` (made by `soc/zuma-addrs.py` from the phone's tree): notes for the drivers still to be written.

## First boot

Where the port is (docs/porting/index.md, step 2), as of 2026-10-31. Everything below is written from the phone's own
device tree (`prebuilt/device-tree.dtb`); nothing of it has run on the phone yet.

| Part | What | Where |
|---|---|---|
| boot path | Google's bootloader `fastboot boot`s HakiOS's U-Boot from RAM (header version 4, U-Boot as an arm64 Image); U-Boot finds the loader's container behind the vendor_boot ramdisks and starts it | `u-boot/`, `u-boot/boot-image`, cmd/hakios.c (`find_container`) |
| U-Boot | exynos-mobile with only what needs no zuma driver: RAM (4 x 2 GiB), the 34 static carve-outs of the Android tree that are in DRAM (of 37), the bootloader's framebuffer, the timer (24.576 MHz), PSCI reset | `u-boot/files/` |
| loader | `platform akita`, known by the `google,zuma` of U-Boot's tree: GICv3 at 0x10400000 / 0x10440000, 8 of the 9 cores (MPIDR's second field), PSCI by SMC, the timer's rate taken from the platform | xnuloader/loader.c |
| EL2 | the bootloader enters its payload at EL2 (the phone runs protected KVM), XNU runs at EL1: the loader starts it there and takes it back through HVC; checked in QEMU (`virtualization=on`), with and without VHE | xnuloader/loader.c, `jump_to_kernel_from_el2` |
| configuration | one core, no storage (`configs/akita.config`) | |

Run, on a machine where fastboot reaches the phone (not WSL2):

```sh
devices/akita/mk-first-boot.sh        # build/akita/hakios-akita.img: U-Boot, the loader, the kernel and hello
adb reboot bootloader
fastboot boot build/akita/hakios-akita.img
```

What the screen should show, and what it means when it stops:

| Seen | Not seen: look at |
|---|---|
| U-Boot's banner | nothing at all: the framebuffer's address or format (`zuma-akita.dts`; `pull-boot-info.sh` reads the bootloader's log for it), or the boot image's kind (`u-boot/boot-image`; pull-boot-info.sh reads the phone's own boot header) |
| `[oosexgang] hakios: payload of ...`, `loader of ...` | `the boot image has no ramdisk` or `nothing known at`: the ramdisk the bootloader handed over (cmd/hakios.c) |
| `== XNULOADER v3 ==`, `CurrentEL=2`, `platform akita` | `platform qemu-virt`: U-Boot's tree did not reach the loader as the EFI device tree table |
| `jumping to ...`, then the kernel's lines | a reset or a hang right after `jumping`: the drop to EL1 (U-Boot's EL2 is without VHE, the branch QEMU's firmware does not take) |
| `Hello from user space on Darwin!`, then a restart into Android | the kernel's log comes back to the loader and is shown before the restart |

**First boot on the phone, 2026-10-31** (`boot-info/first-boot-2026-10-31.txt`, read back from pstore): U-Boot found its
container behind itself (loaded at 0x80000000), the loader came up at EL2 as `platform akita` (CNTFRQ 24576000, display
1080 x 2400 at 0xfac00000), the kernel ran at EL1 on one core (GICv3 and its redistributor found, the virtual timer),
`hello` printed `Hello from user space on Darwin!`, the kernel returned to the loader. What it took: an unsigned AVB
footer (the bootloader refuses a boot image without one, unlocked or not), the container in the kernel (the bootloader
loads init_boot's ramdisk, never boot's), DECON0's trigger unmasked, U-Boot's load address noted before it relocates.
The phone restarted about two minutes after the bootloader handed over, whatever ran (117 and 121 s): the bootloader
leaves both cluster watchdogs counting with reset on (cl0 WTCON 0x1af39, cl1 0x18021). U-Boot's preboot now stops them
(`hakios wdt`); the next run went on to the end of the kernel's log and the loader's own reset, 324 s, most of it the
log drawn on the screen (`boot-info/first-boot-2026-10-31-watchdog-off.txt`). A hang now stays a hang (power key).

**2026-11-01:** eight cores (`cpus=8`): the kernel starts the other seven itself, the firmware brings them up at EL2,
and the loader's trampoline (`/chosen/el2-trampoline`) takes each one down to EL1; the big cores join at once, the
four A510s 28 s later, by design (`boot-info/first-boot-2026-11-01-eight-cores.txt`). zuma's redistributors are 0x20000
apart (no VLPIS). U-Boot ran with the instruction cache off (Google's bootloader hands over so, and Exynos'
enable_caches() turned on the data cache only): `hakios load` took 1 min 45 s, now 0.24 s; a whole run, from
`fastboot boot` to the phone back in Android, takes 50 s (`boot-info/first-boot-2026-11-01-icache.txt`). From the loader
on, the console goes to pstore only (`hakios_screen=1` keeps the screen too).

Reading a run back: U-Boot keeps its console (with the loader's and the kernel's lines) in the pmsg zone of ramoops
(`hakios_pstore`); after the restart, in Android as root, `cat /sys/fs/pstore/pmsg-ramoops-0`.

## What is this phone's own

The kernel, loader and user space know nothing about a phone: addresses, interrupts, rails and firmware
come from its tree and its Android. What is written for one phone is the list below. The head of each file
says where its numbers came from; that is the map of what to rewrite.

- [ ] **UEFI**: the firmware that starts the loader: devices/redwood/uefi/ is a port of Mu-Silicium; the vendor's drivers are extracted from your own phone (uefi/README.md)
- [ ] **Display, buttons, touch**: drivers/input/hakios.input.c, hakios.buttons.c, hakios.goodix.c (the reference phone's controller; another phone has another)
- [ ] **Storage**: hakios.ufs.c and hakios.ufshc.c (UFS); a phone with eMMC needs another driver
- [ ] **CPU clocks**: hakios.cpufreq.c
- [ ] **GPU**: hakios.adreno.c (Adreno 642L here: another generation needs its own firmware and registers)
- [ ] **Wi-Fi**: hakios.wifi.c, hakios.lx.c and wifi/lx/ (the WCN6750 radio of the reference phone)
- [ ] **Audio and battery**: hakios.adsp.c, hakios.audio.c, hakios.sound.c, hakios.speaker.c, hakios.mic.c, hakios.battery.c
- [ ] **Cellular**: hakios.mpss.c, hakios.qmi.c, hakios.nas.c, hakios.ipa.c, hakios.cell.c; soc.args carries the addresses, the IPA layout is for IPA 4.11 (docs/cellular.md)
- [ ] **Haptics and cameras**: hakios.haptic.c, hakios.camera.c

## Configuration

The kernel's settings are layered the way an Android kernel's are: the common kernel, the SoC's, this phone's
(`docs/configs.md`). Two short files make this phone a configuration:

    configs/akita.config          CONFIG_DEVICE="akita"   and whatever is true of this phone only
    configs/akita_defconfig       #include common_defconfig
                                   #include <the SoC's fragment: sc7280.config for the SC7280 family>
                                   #include akita.config

Then `make akita_defconfig`. Turn on only what this phone has been shown to do; `tools/hakios-config.py check` says what the
choice needs.

The drivers are part of the configuration too (`docs/drivers-config.md`): every one is built unless a configuration leaves it out,
and the addresses, pins and rails in each are the reference phone's. Leave out in `configs/akita.config` what has not been
ported to this phone yet (`# CONFIG_HAKIOS_GOODIX is not set`, ...; `configs/qemu_defconfig` leaves out all of them), and put
each one back when it has been made to work here. `tools/hakios-config.py show` lists what is built in.

## Firmware

Pulled from your own phone, never committed: `devices/redwood/pull-firmware.sh`, or
`pull-firmware-root.sh` where the firmware is readable by root only. Check with `make doctor`.

## Before you open a pull request

`tools/provenance-check.py` must pass: no vendor firmware, no UEFI drivers, no Apple files in the tree.

## The repository

A HakiOS device repository: what is true of this device and of no other. The kernel, the loader, the drivers and the tools are
the common repository's; the zuma family's facts are in `soc/`, carried with the device. How the three are put together:
[docs/porting/repositories.md](https://github.com/oosexgang/hakios-xnu-kernel/blob/main/docs/porting/repositories.md).

| File | What |
|---|---|
| `device.conf` | what the device is, and how far each part of it works: its page in the documentation is made from it |
| `configs/akita_defconfig` | the common kernel, the SoC's fragment, this device's: `make akita_defconfig` |
| `configs/akita.config` | what is this device's own: the drivers built in, the arguments it needs |
| `soc/` | its SoC family's layer (zuma): `soc.conf`, `configs/zuma.config`, the SoC's drivers; carried here so that this one repository is all the device needs |
| `drivers/` | drivers of its own, if it needs any (`drivers/<subsystem>/conf`, as in the common repository) |

Every phone driver of the common repository is left out to begin with: their addresses and pins are the phone they were written
for. Put one back in `configs/akita.config` when it has been made to work here, and say so in `device.conf`.

Check with `tools/hakios-device.py check akita`.
