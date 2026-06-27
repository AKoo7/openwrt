# SPDX-License-Identifier: GPL-2.0-only

include $(TOPDIR)/rules.mk

ARCH:=mips
BOARD:=realtek-luna
BOARDNAME:=Realtek Luna GPON ONU
DEVICE_TYPE:=basic
FEATURES:=ramdisk squashfs gpio
SUBTARGETS:=rtl960x rtl9607x

KERNEL_PATCHVER:=6.18

define Target/Description
	Build firmware images for Realtek "Luna" GPON ONU SoCs: the RTL960xC /
	RTL8672 "Europa" family on the RLX "Taroko" core (rtl960x subtarget)
	and the RTL9607C on a MIPS interAptiv core (rtl9607x subtarget).
endef

include $(INCLUDE_DIR)/target.mk

DEFAULT_PACKAGES += \
	kmod-gpio-button-hotplug \
	uboot-envtools \
	iw

$(eval $(call BuildTarget))
