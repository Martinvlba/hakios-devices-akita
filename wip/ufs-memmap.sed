# u-boot/board/samsung/exynos-mobile/exynos-mobile.c: the peripheral window to 0x30000000 (zuma's CMU_TOP is at 0x26040000)
/Peripheral MMIO block/,/\.size/ s/\.size = 0x10000000UL,/.size = 0x20000000UL,/
