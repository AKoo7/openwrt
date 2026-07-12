# SPDX-License-Identifier: GPL-2.0-only

include $(TOPDIR)/rules.mk

ARCH:=aarch64
BOARD:=realtek-elnath
BOARDNAME:=Realtek Elnath GPON ONU
DEVICE_TYPE:=basic
FEATURES:=ramdisk squashfs gpio fpu
SUBTARGETS:=rtl9607f

KERNEL_PATCHVER:=6.18

define Target/Description
	Build firmware images for Realtek "Elnath" GPON ONU SoCs (RTL9607F and
	family): a Cortina-Access "Venus" ARM Cortex-A55 core (aarch64) that
	Realtek productized after acquiring Cortina Access. Distinct silicon and
	CPU architecture from the MIPS "Luna" family (realtek-luna target).
	DDR3, SPI-NAND; run-from-RAM bring-up via TFTP + U-Boot bootm.
endef

include $(INCLUDE_DIR)/target.mk

# Match realtek-luna: skip urngd (the crng self-seeds from interrupt entropy;
# the jitter daemon just burns boot CPU on these low-clock ONU cores).
DEFAULT_PACKAGES := $(filter-out urngd,$(DEFAULT_PACKAGES))

DEFAULT_PACKAGES += \
	kmod-gpio-button-hotplug \
	uboot-envtools \
	iw

$(eval $(call BuildTarget))
