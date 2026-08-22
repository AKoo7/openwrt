# SPDX-License-Identifier: GPL-2.0-only

ARCH:=aarch64
SUBTARGET:=rtl9607f
BOARD:=realtek-elnath
BOARDNAME:=Realtek RTL9607F Cortex-A55
# aarch64 boots a raw Image (self-relocating); build Image + DTBs and let
# OpenWrt copy them to KDIR (as Image / Image-initramfs) for the FIT.
KERNELNAME:=Image dtbs

define Target/Description
	Build firmware images for the Realtek RTL9607F GPON ONU SoC ("Elnath"):
	a dual-core ARM Cortex-A55 (aarch64) Cortina-Access "Venus" core with a
	GICv3, DDR3 and SPI-NAND. Bring-up is run-from-RAM: an initramfs FIT is
	TFTP'd into RAM and booted by the vendor U-Boot 2022.10 ("Elnath-SoC",
	autoboot stop key Ctrl+A), leaving NAND untouched.
endef
