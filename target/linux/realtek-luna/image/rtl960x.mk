# SPDX-License-Identifier: GPL-2.0-only
#
# Per-board image definitions for the rtl960x subtarget.
# Boards are brought up run-from-RAM first: the initramfs uImage is TFTP'd
# into RAM and bootm'd by the vendor U-Boot (no flash write during bring-up).

define Device/hsgq_x111w
  DEVICE_VENDOR := HSGQ
  DEVICE_MODEL := X111W
  DEVICE_DTS := rtl9602c_x111w
  DEVICE_DTS_DIR := $(DTS_DIR)/realtek-luna
  SOC := rtl9602c
  # NAND persistent image (raw MTD; the vendor fwu.sh flashes a uImage -> k0 and a
  # squashfs -> r0). Partition sizes from the live device /proc/mtd:
  #   k0 = 0x300000 (3072k) kernel, r0 = 0x4c0000 (4864k) rootfs.
  # SIZE GUARD: KERNEL_SIZE auto-fires check-size on the kernel (image.mk), and the
  # explicit check-size on kernel.bin/rootfs.bin makes the BUILD fail if either
  # overflows its partition -- an oversize image is never written to the device.
  KERNEL_SIZE := 3072k
  IMAGE_SIZE := 4864k
  IMAGES := kernel.bin rootfs.bin sysupgrade.bin
  IMAGE/kernel.bin := append-kernel | check-size $$(KERNEL_SIZE)
  IMAGE/rootfs.bin := append-rootfs | pad-rootfs | check-size
  IMAGE/sysupgrade.bin := sysupgrade-tar | append-metadata
  # luci-app-gpon builds (validated) but its luci-base dep needs cgi-io from the
  # 'packages' feed; add "luci-app-gpon luci" here once that feed is installed.
  # Router stack so the ONU actually routes/NATs LAN<->WAN and serves DHCP:
  #   dnsmasq           - LAN DHCPv4 + DNS server
  #   firewall4(+nft)   - lan/wan zones, lan->wan forward + masquerade (NAT)
  #   odhcpd-ipv6only   - LAN IPv6 RA/DHCPv6 server (distributes the WAN PD)
  #   odhcp6c           - WAN DHCPv6 client (PD request toward the OLT)
  #   ppp ppp-mod-pppoe - PPPoE WAN client (alternative WAN proto to IPoE/DHCP)
  #   wpad-basic-mbedtls - WiFi AP/STA daemon (hostapd+wpa_supplicant) for the
  #     rtl8192fe AP: WPA2-PSK(psk2) + WPA3-SAE personal, no enterprise/EAP, no
  #     mesh; mbedtls TLS (libmbedtls is already pulled by libustream-mbedtls, so
  #     this adds only the ~1MB wpad binary, ~350k squashfs-compressed). This is
  #     the smallest wpad variant that still does psk2 AP + PMF; wpad-mini lacks
  #     SAE/802.11w. Listed here so 'make defconfig' keeps it tied to the device
  #     profile (a bare .config regen would otherwise drop a manual selection).
  # (urngd is excluded target-wide in target/linux/realtek-luna/Makefile: it
  #  burned ~60s of boot CPU on this core for nothing - crng seeds itself.)
  DEVICE_PACKAGES := gpon-provision luci-app-gpon luci-base luci-mod-admin-full luci-theme-bootstrap uhttpd uhttpd-mod-ubus rpcd rpcd-mod-file \
	dnsmasq firewall4 odhcpd-ipv6only odhcp6c ppp ppp-mod-pppoe wpad-basic-mbedtls
endef
TARGET_DEVICES += hsgq_x111w
