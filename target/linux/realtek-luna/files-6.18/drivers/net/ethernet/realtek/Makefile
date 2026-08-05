# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for the Realtek network device drivers.
#
#
# OpenWrt/realtek-luna -- WHOLE-FILE OVERLAY, migrated out of a patch.
#
# This is Linux 6.18.31's own file plus the realtek-luna GPON/ETH wiring
# that used to live in patches-6.18/103-realtek-luna-gpon.patch.  The patch
# and this file are mutually exclusive: files- overlays are copied into the
# kernel tree BEFORE patches are applied (include/quilt.mk:93-107), so with
# both present patch 103 applies to this file, its context no longer matches
# and the build FAILS LOUDLY.  That is the intended direction -- but it means
# patch 103 must be deleted in the SAME commit that adds this file.
#
# WHY the migration is a prerequisite and not a cleanup: the shared GPON
# core (target/linux/gpon-common) is reached by `select GPON_CORE` from the
# vendor Kconfig, and every later move changes luna's obj-y lines.  Both
# edits land in files that only existed inside a forbidden patch.
# Operator hard rule: driver wiring is a browsable source file, never a
# patches-<ver>/*.patch -- not even for Kconfig/Makefile.
#
# Content is byte-identical to what patch 103 produced (verified 2026-08-05
# against build_dir/target-mips_mips32_musl/.../linux-6.18.31).
# Nothing else changed: the object list is untouched by this migration.
#

obj-$(CONFIG_RTL9602C_ETH) += rtl9602c_eth.o
obj-$(CONFIG_RTL9602C_GPON) += gpon-rtl9602c.o rtl960x_ponmac.o
obj-$(CONFIG_RTL9607C_GPON) += rtl9607c_gpon.o rtl960x_ponmac.o
obj-$(CONFIG_RTL9607C_ETH) += rtl9607c_eth.o
# The datapath hot path keeps -O2 even under the size-optimized (-Os) kernel:
# the NIC RX/TX + GEM/PON-MAC forwarding must not lose per-packet performance.
CFLAGS_rtl9602c_eth.o += -O2
CFLAGS_gpon-rtl9602c.o += -O2
CFLAGS_rtl9607c_eth.o += -O2
CFLAGS_rtl9607c_gpon.o += -O2
CFLAGS_rtl960x_ponmac.o += -O2
obj-$(CONFIG_8139CP) += 8139cp.o
obj-$(CONFIG_8139TOO) += 8139too.o
obj-$(CONFIG_ATP) += atp.o
r8169-y += r8169_main.o r8169_firmware.o r8169_phy_config.o
r8169-$(CONFIG_R8169_LEDS) += r8169_leds.o
obj-$(CONFIG_R8169) += r8169.o
obj-$(CONFIG_RTASE) += rtase/
