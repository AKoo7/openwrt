#
# Clean-room LuCI app: GPON ONU provisioning UI.
# SPDX-License-Identifier: GPL-2.0-or-later
#
include $(TOPDIR)/rules.mk

PKG_NAME:=luci-app-gpon
PKG_VERSION:=1
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0-or-later
PKG_MAINTAINER:=Confiared <brule.herman@confiared.com>

include $(INCLUDE_DIR)/package.mk

define Package/luci-app-gpon
  SECTION:=luci
  CATEGORY:=LuCI
  SUBMENU:=3. Applications
  TITLE:=GPON ONU provisioning UI
  DEPENDS:=+luci-base +gpon-provision
  PKGARCH:=all
endef

define Package/luci-app-gpon/description
  LuCI page to view/edit this unit's GPON identity (/etc/config/gpon): SN, LOID,
  LOID password, PLOAM password, MAC. Save & Apply re-provisions live via the
  gpon_provision reload trigger (no reboot). Factory MAC/SN are shown read-only
  unless source is set to manual.
endef

define Build/Compile
endef

define Package/luci-app-gpon/install
	$(INSTALL_DIR) $(1)/www/luci-static/resources/view/gpon
	$(INSTALL_DATA) ./htdocs/luci-static/resources/view/gpon/identity.js \
		$(1)/www/luci-static/resources/view/gpon/identity.js
	$(INSTALL_DIR) $(1)/usr/share/luci/menu.d
	$(INSTALL_DATA) ./root/usr/share/luci/menu.d/luci-app-gpon.json \
		$(1)/usr/share/luci/menu.d/luci-app-gpon.json
	$(INSTALL_DIR) $(1)/usr/share/rpcd/acl.d
	$(INSTALL_DATA) ./root/usr/share/rpcd/acl.d/luci-app-gpon.json \
		$(1)/usr/share/rpcd/acl.d/luci-app-gpon.json
endef

$(eval $(call BuildPackage,luci-app-gpon))
