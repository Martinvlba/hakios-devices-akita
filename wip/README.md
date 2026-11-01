# Work in progress: UFS by taking over the bootloader's controller

Not applied anywhere: U-Boot as committed has no UFS. To try it, apply onto a `u-boot/` made by `uefi/u-boot/apply.sh`:

    (cd u-boot && git apply ../devices/akita/wip/ufs-handover.patch)
    patch -p0 < devices/akita/wip/ufs-handover-defconfig.diff     # (paths inside: the device's file -> u-boot's)
    patch -p0 < devices/akita/wip/ufs-handover-dts.diff

What it is (2026-11-01): `drivers/ufs/ufs-hakios.c` (compatible `hakios,ufs-handover`) and two quirks in U-Boot's UFS
core: KEEP_LINK (no reset, no link startup, no power mode change; the request lists kept or moved) and PRDT_4K;
`hakios,dma-window` (descriptors and a bounce buffer in a given memory range), `hakios,smu-init`
(SMC_CMD_SMU(SMU_INIT)). It has diagnostics printed in it (the vendor HCI block, the link's UniPro state, the
registers after a timeout). See the port's log 4 for what it found and where it is stuck.
