#
# Clean-room OpenWrt package: one-shot remote ONU diagnostic.
# SPDX-License-Identifier: GPL-2.0-or-later
#
include $(TOPDIR)/rules.mk

PKG_NAME:=onu-diag
PKG_VERSION:=1
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0-or-later
PKG_MAINTAINER:=Confiared <brule.herman@confiared.com>

include $(INCLUDE_DIR)/package.mk

define Package/onu-diag
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=One-command remote GPON ONU diagnostic
  DEPENDS:=
endef

define Package/onu-diag/description
  A single self-contained command (onu-diag) that prints the full state of this
  GPON ONU - identity (MAC/SN/LOID), fiber O1-O5 state and RX/TX dBm, WAN lease,
  LAN clients, Wi-Fi and Ethernet link, system health - so a technician can
  diagnose the unit remotely (e.g. over AnyDesk) even when the GPON WAN is down.
  Use "onu-diag" for a report or "onu-diag -j" for JSON.
endef

define Build/Compile
endef

define Package/onu-diag/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) ./files/onu-diag $(1)/usr/sbin/onu-diag
endef

$(eval $(call BuildPackage,onu-diag))
