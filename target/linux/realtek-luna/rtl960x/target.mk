# SPDX-License-Identifier: GPL-2.0-only

ARCH:=mips
SUBTARGET:=rtl960x
BOARD:=realtek-luna
BOARDNAME:=Realtek RTL960xC / RTL8672 Europa (RLX/Taroko)
CPU_TYPE:=mips32

# Lexra MDU erratum mitigation (RLX/Taroko cores only): a memory load issued in
# the div/mult -> mflo/mfhi shadow silently corrupts HI/LO. -fno-schedule-insns2
# stops gcc's sched2 pass from filling that shadow with loads (measured: musl and
# the -O2 datapath drop to 0 hazard windows). CFLAGS here becomes the dumped
# Target-Optimization, i.e. the CONFIG_TARGET_OPTIMIZATION default - pinned in
# the subtarget so a `make defconfig` cannot silently drop the flag (it did once).
# Complete fix = a `-mfix-lexra` gcc div-fusion (TODO). Do NOT add this to the
# rtl9607x subtarget (interAptiv core, unaffected).
CFLAGS:=-Os -pipe -mno-branch-likely -mips32 -mtune=mips32 -fno-schedule-insns2

define Target/Description
	Build firmware images for Realtek RTL960xC GPON ONU boards based on
	the RLX "Taroko" core (RTL9602C, RTL9603C, ...). Big-endian MIPS,
	16 MB SPI-NOR, run-from-RAM bring-up via TFTP/initramfs.
endef
