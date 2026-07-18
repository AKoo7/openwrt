include $(TOPDIR)/rules.mk

PKG_NAME:=wireless-regdb
PKG_VERSION:=2026.03.18
PKG_RELEASE:=1
PKG_LICENSE:=ISC
PKG_LICENSE_FILES:=LICENSE

PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.xz
PKG_SOURCE_URL:=@KERNEL/software/network/wireless-regdb/
PKG_HASH:=5fc0000475d8c5368ccc5222827c16aef98b1eb6a69c9b5a3e7b7e98528945ac

PKG_MAINTAINER:=Felix Fietkau <nbd@nbd.name>

include $(INCLUDE_DIR)/package.mk

define Package/wireless-regdb
  PKGARCH:=all
  SECTION:=firmware
  CATEGORY:=Firmware
  URL:=https://git.kernel.org/pub/scm/linux/kernel/git/wens/wireless-regdb.git/
  TITLE:=Wireless Regulatory Database
endef

# Ship the PRISTINE maintainer-signed pair from the tarball instead of
# regenerating regulatory.db with db2fw.py: an in-kernel cfg80211 (built-in,
# not the backports kmod - e.g. realtek-elnath) has
# CFG80211_REQUIRE_SIGNED_REGDB forced =y (promptless "default y" without
# CERTIFICATION_ONUS), so it only loads regulatory.db when the matching
# regulatory.db.p7s signature (made by the wireless-regdb maintainer key the
# kernel embeds via CFG80211_USE_KERNEL_REGDB_KEYS) verifies against the
# EXACT file content - a regenerated db no longer matches the signature.
# Backports-based targets load the pristine db just the same (no sig check).
define Build/Compile
endef

define Package/wireless-regdb/install
	$(INSTALL_DIR) $(1)/lib/firmware
	$(CP) $(PKG_BUILD_DIR)/regulatory.db $(1)/lib/firmware/
	$(CP) $(PKG_BUILD_DIR)/regulatory.db.p7s $(1)/lib/firmware/
endef

$(eval $(call BuildPackage,wireless-regdb))
