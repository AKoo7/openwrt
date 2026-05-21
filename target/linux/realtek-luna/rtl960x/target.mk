# SPDX-License-Identifier: GPL-2.0-only

ARCH:=mips
SUBTARGET:=rtl960x
BOARD:=realtek-luna
BOARDNAME:=Realtek RTL960xC / RTL8672 "Europa" (RLX/Taroko)
CPU_TYPE:=mips32

define Target/Description
	Build firmware images for Realtek RTL960xC GPON ONU boards based on
	the RLX "Taroko" core (RTL9602C, RTL9603C, ...). Big-endian MIPS,
	16 MB SPI-NOR, run-from-RAM bring-up via TFTP/initramfs.
endef
