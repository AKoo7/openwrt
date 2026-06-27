# SPDX-License-Identifier: GPL-2.0-only

ARCH:=mips
SUBTARGET:=rtl9607x
BOARD:=realtek-luna
BOARDNAME:=Realtek RTL9607C interAptiv
# interAptiv is a standard MIPS32 Release 2 core: build userspace as r2 (24kc:
# -mips32r2) so the C library uses the standard rdhwr/UserLocal thread pointer
# and lwl/lwr unaligned access, not the RLX/Lexra-only encodings the r1 (mips32)
# rtl960x subtarget needs. This also gives this subtarget its own toolchain so
# the two cores never share an ISA-incompatible libc.
CPU_TYPE:=24kc

define Target/Description
	Build firmware images for the Realtek RTL9607C GPON ONU SoC, built on a
	MIPS interAptiv core (MIPS32 R2, big-endian) in a single-core Coherent
	Processing System (GIC + CM + CPC). DDR3, SPI-NAND; run-from-RAM
	bring-up via TFTP/initramfs. Kernel debugging (ftrace/kprobes) is on by
	default here -- this subtarget is the instrumented reference for the
	shared GPON datapath.
endef
