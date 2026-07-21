#
# maple-tools: userspace CLIs for the open Maple GPON driver.
#   - maplectl : list/info/block/unblock ONUs, per-ONU bandwidth
#   - maple_snmp: SNMP pass_persist subagent for per-ONU OIDs
#

include $(TOPDIR)/rules.mk

PKG_NAME:=maple-tools
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0
PKG_MAINTAINER:=hg08-olt-re

PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_NAME)

include $(INCLUDE_DIR)/package.mk

define Package/maple-tools
  SECTION:=net
  CATEGORY:=Network
  SUBMENU:=GPON
  TITLE:=Open Maple GPON ONU management tools
  DEPENDS:=+kmod-maple
endef

define Package/maple-tools/description
  Userspace tools for the open Maple GPON driver:
   - maplectl  : list/info/block/unblock ONUs, per-ONU bandwidth
   - maple_snmp: SNMP pass_persist subagent (per-ONU state/SN/distance/counters)
  Uses raw NETLINK_GENERIC — no libnl dependency.
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(TARGET_CC) $(TARGET_CFLAGS) -Wall -Wextra \
		-o $(PKG_BUILD_DIR)/maplectl \
		$(PKG_BUILD_DIR)/maplectl.c $(PKG_BUILD_DIR)/maple_lib.c
	$(TARGET_CC) $(TARGET_CFLAGS) -Wall -Wextra \
		-o $(PKG_BUILD_DIR)/maple_snmp \
		$(PKG_BUILD_DIR)/maple_snmp.c $(PKG_BUILD_DIR)/maple_lib.c
endef

define Package/maple-tools/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/maplectl $(1)/usr/sbin/
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/maple_snmp $(1)/usr/sbin/
	$(INSTALL_DIR) $(1)/etc/config
	$(INSTALL_DATA) ./files/etc/config/maple $(1)/etc/config/
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) ./files/etc/init.d/maple $(1)/etc/init.d/
endef

$(eval $(call BuildPackage,maple-tools))
