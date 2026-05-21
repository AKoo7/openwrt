# SPDX-License-Identifier: GPL-2.0-only

include $(TOPDIR)/rules.mk

ARCH:=mips
BOARD:=realtek-luna
BOARDNAME:=Realtek Luna GPON ONU
DEVICE_TYPE:=basic
FEATURES:=ramdisk squashfs gpio
SUBTARGETS:=rtl960x

KERNEL_PATCHVER:=6.18

define Target/Description
	Build firmware images for Realtek "Luna" GPON ONU SoCs
	(RTL960xC / RTL8672 "Europa" family, e.g. RTL9602C).
endef

include $(INCLUDE_DIR)/target.mk

DEFAULT_PACKAGES += \
	kmod-gpio-button-hotplug \
	uboot-envtools

$(eval $(call BuildTarget))
