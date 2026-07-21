#
# maple-firmware: Broadcom Maple BCM68620 GPON MAC bootloader + application
# firmware, recovered from the stock kernel's embedded initramfs. Required by
# kmod-maple to bring up the MAC (fld boot path).
#
# These are the SAME blobs shipped in the vendor kernel (work/initramfs/broadcom/);
# redistributed here so the open driver has them. Source: the stock kernel image
# (system.img), which embeds them as a cpio initramfs at offset 0x3a2e70.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=maple-firmware
PKG_VERSION:=1
PKG_RELEASE:=1
PKG_LICENSE:=Proprietary
PKG_MAINTAINER:=hg08-olt-re

include $(INCLUDE_DIR)/package.mk

define Package/maple-firmware
  SECTION:=firmware
  CATEGORY:=Firmware
  TITLE:=Broadcom Maple BCM68620 GPON MAC firmware
  DEPENDS:=+kmod-maple
endef

define Package/maple-firmware/description
  Bootloader (bcm68620_boot.bin, 43 KB) + application firmware
  (bcm68620_appl.bin, 11.8 MB) for the Broadcom Maple BCM6862x GPON OLT MAC.
  Recovered from the stock kernel's embedded initramfs. The open maple
  driver loads these via request_firmware() into SRAM/DDR during bring-up.
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
endef

define Build/Compile
	true
endef

define Package/maple-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware/bcm68620
	$(INSTALL_DATA) ./files/bcm68620_boot.bin $(1)/lib/firmware/bcm68620/
	$(INSTALL_DATA) ./files/bcm68620_appl.bin $(1)/lib/firmware/bcm68620/
endef

$(eval $(call BuildPackage,maple-firmware))
