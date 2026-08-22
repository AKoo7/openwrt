# SPDX-License-Identifier: GPL-2.0-only

include $(TOPDIR)/rules.mk

ARCH:=aarch64
BOARD:=realtek-elnath
BOARDNAME:=Realtek Elnath GPON ONU
DEVICE_TYPE:=basic
FEATURES:=separate_ramdisk squashfs gpio fpu
SUBTARGETS:=rtl9607f

KERNEL_PATCHVER:=6.18

define Target/Description
	Build firmware images for Realtek "Elnath" GPON ONU SoCs (RTL9607F and
	family): a Cortina-Access "Venus" ARM Cortex-A55 core (aarch64) that
	Realtek productized after acquiring Cortina Access. DDR3, SPI-NAND;
	run-from-RAM bring-up via TFTP + U-Boot bootm.
endef

include $(INCLUDE_DIR)/target.mk

# Skip urngd on these low-clock ONU cores (the crng self-seeds from interrupt
# entropy; the jitter daemon just burns boot CPU).
DEFAULT_PACKAGES := $(filter-out urngd,$(DEFAULT_PACKAGES))

DEFAULT_PACKAGES += \
	kmod-gpio-button-hotplug \
	uboot-envtools \
	iw \
	ethtool

$(eval $(call BuildTarget))
