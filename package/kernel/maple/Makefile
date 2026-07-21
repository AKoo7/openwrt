#
# Open Maple GPON MAC driver (kmod-maple).
# Open-driver replacement for the vendor ll_pcie/os_linux/fld/trmux/bcm_dev_ctrl
# .ko stack, for the Broadcom Maple BCM6862x GPON MAC on the HSGQ HG08 OLT.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=maple
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0
PKG_MAINTAINER:=hg08-olt-re

include $(INCLUDE_DIR)/package.mk

define KernelPackage/maple-epld
  SUBMENU:=Other modules
  TITLE:=Maple EPLD reset controller (releases Maple MAC from reset)
  DEPENDS:=@TARGET_bcm53xx
  FILES:=$(PKG_BUILD_DIR)/maple-epld.ko
  AUTOLOAD:=$(call AutoLoad,70,maple-epld,1)
endef

define KernelPackage/maple-epld/description
  Board EPLD control via I2C — releases the Maple GPON MAC from reset
  (EPLD reg @0x40 offset 2, via PCA9548A mux 0x70 ch1) then triggers
  a PCIe bus rescan so the maple.ko PCI driver can bind.
endef

define KernelPackage/maple
  SUBMENU:=Other modules
  TITLE:=Open Maple GPON MAC driver (BCM6862x)
  DEPENDS:=@TARGET_bcm53xx +kmod-maple-epld
  FILES:=$(PKG_BUILD_DIR)/maple.ko
  AUTOLOAD:=$(call AutoLoad,80,maple,1)
endef

define KernelPackage/maple/description
  Open driver for the Broadcom Maple BCM6862x GPON OLT MAC. Binds PCI
  14E4:6862/6863/5554, maps BAR0/2/4, pushes the boot+appl firmware,
  runs the bcmtr_pcie DMA ring transport, exposes a genetlink "maple"
  family for ONU get-cfg/get-stat/set-state (block/unblock/activate).
  Reverse-engineered from the vendor ll_pcie/os_linux/fld/trmux
  kernel modules (DWARF-verified against the binary, 67/67 oracle).
endef

define Build/Compile
	$(KERNEL_MAKE) M="$(PKG_BUILD_DIR)" modules
endef

$(eval $(call KernelPackage,maple-epld))
$(eval $(call KernelPackage,maple))
