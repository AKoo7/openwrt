# SPDX-License-Identifier: GPL-2.0-only
#
# Register-level diagnostic tools for the RTL9607F "Elnath" (X400AXF) board.
#
# Built static on purpose: the same binary has to run on the vendor firmware
# as well as on this one, because a measurement taken only on our own image
# cannot tell you whether the thing you are measuring even works on hardware
# that is known good.  A counter reading zero on a working vendor image is a
# phantom, not a fault, and that distinction has cost this port more debugging
# time than any other single mistake.

include $(TOPDIR)/rules.mk

PKG_NAME:=x400axf-tools
PKG_RELEASE:=1

PKG_LICENSE:=GPL-2.0-only
PKG_MAINTAINER:=Confiared <contact@confiared.com>

include $(INCLUDE_DIR)/package.mk

define Package/x400axf-tools
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=Cortina Access register diagnostics (X400AXF / RTL9607F)
  DEPENDS:=@TARGET_realtek_elnath
endef

define Package/x400axf-tools/description
 caregt       read, write and poll-trace Cortina Access registers via /dev/mem.
              The poll-trace samples a set of registers and reports which one
              moved and in what order, which a static snapshot cannot show.
 forge_inject craft and inject frames for datapath forwarding tests.

 Both link static so they also run on the vendor image, making a stock-versus-
 this-firmware differential possible with one tool.
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR) $(TARGET_CONFIGURE_OPTS) CFLAGS="$(TARGET_CFLAGS) -Wall -Wextra"
endef

define Package/x400axf-tools/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/caregt $(1)/usr/bin/
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/forge_inject $(1)/usr/bin/
endef

$(eval $(call BuildPackage,x400axf-tools))
