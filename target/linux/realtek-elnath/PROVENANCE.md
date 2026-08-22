# PROVENANCE — RTL9607F / AOT-5221Zy OpenWrt target

This file records the origin of every source file in this target, for GPL-2.0
compliance and honest attribution.

## License chain (all GPL-2.0)

1. **Cortina-Access Limited** released the GPON/NE datapath sources
   (`aal_*.c`, `(c) Cortina-Access Limited 2015`) and the SoC peripheral kernel
   drivers (`(c) Cortina Access, Inc.`) under **GPL-2.0**.
2. **BRULE Herman** (`alpha_one_x86@first-world.info`,
   https://github.com/alphaonex86/openwrt) ported the Cortina NI/GPON/L3FE MAC
   and the SoC peripherals to a kernel-6.18 `realtek-elnath` OpenWrt target,
   under **GPL-2.0**.
3. **This port** (AK Sharma, `@AKoo7`) adds AOT-5221Zy board support, a GPON
   OMCI/PLOAM/GEM stack, a stock-daemon bridge and DSA support, under
   **GPL-2.0**.

All files retain their SPDX identifiers and upstream copyright notices.

---

## Original to THIS port — (c) AK Sharma, GPL-2.0

- `base-files/etc/board.d/02_network`
- `base-files/etc/config/omci_stock`
- `base-files/etc/init.d/omci_stock`
- `base-files/etc/uci-defaults/99-uhttpd-tune`
- `base-files/lib/preinit/00_no_failsafe`
- `base-files/lib/preinit/30_failsafe_wait`
- `files-6.18/arch/arm64/boot/dts/realtek-elnath/rtl9607f_aot5221zy.dts`
- `files-6.18/drivers/net/Kconfig`
- `files-6.18/drivers/net/Makefile`
- `files-6.18/drivers/net/ethernet/cortina/cortina-ni-dsa.c`
- `files-6.18/drivers/net/ethernet/cortina/cortina-ni-ethtool.c`
- `files-6.18/drivers/net/ethernet/cortina/cortina-omci-bridge.c`
- `files-6.18/drivers/net/ethernet/cortina/cortina-omci-bridge.h`
- `files-6.18/drivers/net/gpon/Kconfig`
- `files-6.18/drivers/net/gpon/Makefile`
- `files-6.18/drivers/net/gpon/gpon_common.h`
- `files-6.18/drivers/net/gpon/gpon_gem_us.c`
- `files-6.18/drivers/net/gpon/gpon_gem_us.h`
- `files-6.18/drivers/net/gpon/gpon_omci_core.c`
- `files-6.18/drivers/net/gpon/gpon_omci_core.h`
- `files-6.18/drivers/net/gpon/gpon_omci_me.c`
- `files-6.18/drivers/net/gpon/gpon_omci_me.h`
- `files-6.18/drivers/net/gpon/gpon_ploam.c`
- `files-6.18/drivers/net/gpon/gpon_ploam.h`
- `files-6.18/net/dsa/tag_cortina.c`
- `patches-6.18/900-net-dsa-add-cortina-tag.patch`

> **Note:** `files-6.18/drivers/net/Kconfig` and `files-6.18/drivers/net/Makefile` are unmodified mainline-kernel files plus a one-line registration of the `gpon` subdirectory; they appear here only because they are absent from the alphaonex86 base, not as original works.

## Derived from alphaonex86/openwrt `realtek-elnath` — (c) BRULE Herman / Cortina-Access, GPL-2.0

The files below originate in Herman Brule's `realtek-elnath` target (itself
built on Cortina-Access GPL-2.0 sources). Some are unchanged; others carry
substantial additions made for this port (notably `cortina-gpon.c`,
`cortina-ni-rx.c`, `cortina-ni-tx.c`). The SoC peripheral drivers
(`pcie-cortina`, `spi-cortina-qspi`, `cortina-uart` ← `serial_cortina`,
`gpio-cortina-ca77xx` ← `gpio-ca77xx`) trace directly to Cortina-Access GPL-2.0
kernel drivers.

- `Makefile`
- `base-files/etc/config/network`
- `base-files/etc/hotplug.d/iface/99-pppoe-hw-session`
- `base-files/etc/init.d/gpon-identity`
- `base-files/etc/uci-defaults/05_factory_mac`
- `base-files/etc/uci-defaults/25_flow_offload`
- `base-files/etc/uci-defaults/30-upnp-secure`
- `base-files/etc/uci-defaults/90-dropbear-lan-only`
- `config-6.18`
- `files-6.18/drivers/gpio/Kconfig`
- `files-6.18/drivers/gpio/Makefile`
- `files-6.18/drivers/gpio/gpio-cortina-ca77xx.c`
- `files-6.18/drivers/net/ethernet/cortina/Kconfig`
- `files-6.18/drivers/net/ethernet/cortina/Makefile`
- `files-6.18/drivers/net/ethernet/cortina/cortina-gpon-bosa-cal.h`
- `files-6.18/drivers/net/ethernet/cortina/cortina-gpon-bosa-seq.h`
- `files-6.18/drivers/net/ethernet/cortina/cortina-gpon-bosa.c`
- `files-6.18/drivers/net/ethernet/cortina/cortina-gpon-bosa.h`
- `files-6.18/drivers/net/ethernet/cortina/cortina-gpon-ddm.h`
- `files-6.18/drivers/net/ethernet/cortina/cortina-gpon-serdes.h`
- `files-6.18/drivers/net/ethernet/cortina/cortina-gpon.c`
- `files-6.18/drivers/net/ethernet/cortina/cortina-i2c.c`
- `files-6.18/drivers/net/ethernet/cortina/cortina-i2c.h`
- `files-6.18/drivers/net/ethernet/cortina/cortina-l3fe-aging.c`
- `files-6.18/drivers/net/ethernet/cortina/cortina-l3fe.c`
- `files-6.18/drivers/net/ethernet/cortina/cortina-l3fe.h`
- `files-6.18/drivers/net/ethernet/cortina/cortina-ni-flowoffload.c`
- `files-6.18/drivers/net/ethernet/cortina/cortina-ni-regs.h`
- `files-6.18/drivers/net/ethernet/cortina/cortina-ni-rx.c`
- `files-6.18/drivers/net/ethernet/cortina/cortina-ni-tx.c`
- `files-6.18/drivers/net/ethernet/cortina/cortina-ni.c`
- `files-6.18/drivers/net/ethernet/cortina/cortina-ni.h`
- `files-6.18/drivers/pci/controller/dwc/Kconfig`
- `files-6.18/drivers/pci/controller/dwc/Makefile`
- `files-6.18/drivers/pci/controller/dwc/pcie-cortina.c`
- `files-6.18/drivers/spi/Kconfig`
- `files-6.18/drivers/spi/Makefile`
- `files-6.18/drivers/spi/spi-cortina-qspi.c`
- `files-6.18/drivers/tty/serial/Kconfig`
- `files-6.18/drivers/tty/serial/Makefile`
- `files-6.18/drivers/tty/serial/cortina-uart.c`
- `image/Makefile`
- `image/rtl9607f.mk`
- `rtl9607f/config-default`
- `rtl9607f/target.mk`

---

A full file-by-file provenance and de-vendoring plan (which files are byte-
identical vs. modified, and how each could be re-originated from the Cortina
GPL sources) is maintained by the port author outside this tree.
