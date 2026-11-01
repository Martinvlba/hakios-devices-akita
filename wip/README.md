# Work in progress: UFS in U-Boot (the Pixel 8a's flash, read)

Not applied anywhere yet: U-Boot as committed has no UFS. To try it, onto a `u-boot/` made by `uefi/u-boot/apply.sh`:

    (cd u-boot && git apply ../devices/akita/wip/ufs.patch)
    sed -i -f devices/akita/wip/ufs-memmap.sed u-boot/board/samsung/exynos-mobile/exynos-mobile.c
    patch -p0 < devices/akita/wip/ufs-defconfig.diff     # (the device's defconfig -> u-boot's)
    patch -p0 < devices/akita/wip/ufs-dts.diff           # (the device's tree -> u-boot's)

**Status (2026-11-01, evening): reads work.** `scsi scan` finds the Micron MT128GBCAV2U31 (119 GB, LUNs 0-3), `part list
scsi 0` shows Android's 34 partitions, raw reads are right ("EFI PART"). Slow: the link stays in PWM-G1 (0.6 MB/s);
next is the power-mode change to HS gear 4 (the CAL's pre_pmc/post_pmc around U-Boot's change_power_mode).

What is in it:
- `drivers/ufs/ufs-exynos-gs.c` (compatible `hakios,zuma-ufs`): Google's Linux bring-up order: the device's supply and
  pins back on (the bootloader switches them off), IO coherency, host reset and configuration, the device's reset,
  Samsung's CAL (`drivers/ufs/exynos-gs/`, Google's zuma files) around link startup.
- U-Boot UFS core: a link startup that completes but fails (UIC result not 0) is retried, as Linux does (a bug in
  U-Boot: it took it for success with no device); Exynos nexus types per request; PRDT entries of 4 KB; a DMA window
  with a bounce buffer (kept from the take-over attempt; may not be needed); diagnostics printed.
- `drivers/ufs/ufs-hakios.c`: the first attempt, taking over the bootloader's link: dead (the device has no power).
