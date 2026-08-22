// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * cortina-ni-dsa.c — expose the four RJ45 jacks of the RTL9607F Cortina NI as
 * Linux DSA user ports lan1..lan4.
 *
 * The NI is an integrated MAC+switch: one CPU-side netdev ("eth0", the DSA
 * conduit) fronts a switch whose four LAN jacks are internal GPHY NI ports.
 * The per-jack identity is carried OUT OF BAND, not on the wire:
 *   RX: HEADER_A.lspid in the DMA metadata (jack N == lspid N, 0..3).
 *   TX: DESC1_DEST in the DMA descriptor (identity ARB map -> jack N).
 *
 * ★ FORWARDING MODEL (M1) — SOFTWARE bridging, not autonomous HW switching.
 * The NI bring-up traps every LAN ingress frame to the CPU (fix#95 CLE_IGR);
 * the hardware does NOT forward LAN<->LAN on its own.  So the Linux bridge owns
 * LAN<->LAN forwarding, exactly as the pre-DSA single-netdev driver did.  Two
 * consequences drive the RX/bridge design here:
 *   - RX delivers each LAN frame DIRECTLY to its jack's user netdev (lan_ndev[])
 *     rather than via a METADATA_HW_PORT_MUX md_dst.  The md_dst path would make
 *     net/dsa/tag.c set skb->offload_fwd_mark = 1, which tells the bridge the
 *     hardware already forwarded within the switch's hwdom and SUPPRESSES the
 *     software forward to sibling jacks (nbp_switchdev_allowed_egress) — i.e. it
 *     would blackhole LAN<->LAN.  Direct delivery leaves offload_fwd_mark = 0.
 *   - .port_bridge_join returns 0 (no HW to program) purely so the kernel ALLOWS
 *     lan1..4 to join br-lan (dsa_switch_bridge_join returns -EOPNOTSUPP if the
 *     op is absent); *tx_fwd_offload stays false.
 * Real hardware LAN<->LAN bridge/VLAN offload over the L2FE is M3; at that point
 * this flips to md_dst + a programming .port_bridge_join and offload_fwd_mark=1
 * becomes truthful.
 *
 * The GPHY bring-up (fix#141), per-port MAC enable and the ARB identity map are
 * already established by the NI probe; DSA rides on top, so .setup and the
 * phylink MAC ops are near-empty for M1 (fixed-link 1G/full per port; real
 * per-port phylink over the GPHYs is M2).
 *
 * See ~/rtl9607f_port/DSA_PLANS/DSA_M1_VERIFIED_DESIGN_2026-08-19.md.
 */

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <net/dsa.h>

#include "cortina-ni.h"
#include "cortina-ni-regs.h"

static enum dsa_tag_protocol
cortina_dsa_get_tag_protocol(struct dsa_switch *ds, int port,
			     enum dsa_tag_protocol mprot)
{
	return DSA_TAG_PROTO_CORTINA;
}

static int cortina_dsa_setup(struct dsa_switch *ds)
{
	/* Nothing to program: the NI probe already brought up the GPHYs, the
	 * per-port MACs, the ARB identity map and the ingress-to-CPU classifier
	 * (so the HW never forwards LAN<->LAN autonomously — the bridge does). */
	return 0;
}

static void cortina_dsa_phylink_get_caps(struct dsa_switch *ds, int port,
					 struct phylink_config *config)
{
	config->mac_capabilities = MAC_SYM_PAUSE | MAC_ASYM_PAUSE |
				   MAC_10 | MAC_100 | MAC_1000FD;

	/* Every port is internal silicon: the four jacks are internal GPHYs and
	 * the CPU port is an internal MAC.  GMII must also be advertised or
	 * phylib's default interface makes phylink reject the internal PHYs. */
	__set_bit(PHY_INTERFACE_MODE_INTERNAL, config->supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_GMII, config->supported_interfaces);
}

/* Allow lan1..4 to join a Linux bridge.  No HW to program in M1 (the bridge
 * software-forwards through the CPU); tx_fwd_offload stays false so DSA does not
 * claim hardware TX forwarding and RX stays offload_fwd_mark=0. */
static int cortina_dsa_port_bridge_join(struct dsa_switch *ds, int port,
					struct dsa_bridge bridge,
					bool *tx_fwd_offload,
					struct netlink_ext_ack *extack)
{
	return 0;
}

static void cortina_dsa_port_bridge_leave(struct dsa_switch *ds, int port,
					  struct dsa_bridge bridge)
{
}

static const struct dsa_switch_ops cortina_dsa_switch_ops = {
	.get_tag_protocol	= cortina_dsa_get_tag_protocol,
	.setup			= cortina_dsa_setup,
	.phylink_get_caps	= cortina_dsa_phylink_get_caps,
	.port_bridge_join	= cortina_dsa_port_bridge_join,
	.port_bridge_leave	= cortina_dsa_port_bridge_leave,
};

/* phylink MAC ops — mandatory (phylink dereferences them for every port).  For
 * M1 the per-port MACs are already enabled/forced by the NI bring-up, so these
 * only need to exist; M2 pokes per-port speed/duplex here. */
static void cortina_dsa_mac_config(struct phylink_config *config,
				   unsigned int mode,
				   const struct phylink_link_state *state)
{
}

static void cortina_dsa_mac_link_up(struct phylink_config *config,
				    struct phy_device *phy, unsigned int mode,
				    phy_interface_t interface, int speed,
				    int duplex, bool tx_pause, bool rx_pause)
{
}

static void cortina_dsa_mac_link_down(struct phylink_config *config,
				      unsigned int mode,
				      phy_interface_t interface)
{
}

static const struct phylink_mac_ops cortina_dsa_phylink_mac_ops = {
	.mac_config	= cortina_dsa_mac_config,
	.mac_link_up	= cortina_dsa_mac_link_up,
	.mac_link_down	= cortina_dsa_mac_link_down,
};

int cortina_ni_dsa_register(struct cortina_ni *ni)
{
	struct dsa_switch *ds;
	int i, ret;

	ds = devm_kzalloc(ni->dev, sizeof(*ds), GFP_KERNEL);
	if (!ds)
		return -ENOMEM;
	ds->dev = ni->dev;
	ds->num_ports = CA_NI_DSA_NUM_PORTS;
	ds->ops = &cortina_dsa_switch_ops;
	ds->phylink_mac_ops = &cortina_dsa_phylink_mac_ops;
	ds->priv = ni;
	ni->ds = ds;

	/* The conduit netdev (eth0) is already registered with its dev.of_node
	 * set to the "ethernet@0" child, which port@CA_NI_DSA_CPU_PORT references
	 * via `ethernet = <&conduit>`, so this resolves without deferring. */
	ret = dsa_register_switch(ds);
	if (ret) {
		dev_err(ni->dev, "DSA register failed: %d\n", ret);
		ni->ds = NULL;
		return ret;
	}

	/* Cache the user netdevs (created synchronously by dsa_register_switch)
	 * so the RX hot path resolves lspid -> lanN without a list walk. */
	for (i = 0; i < CA_NI_DSA_USER_PORTS; i++)
		WRITE_ONCE(ni->lan_ndev[i], dsa_to_port(ds, i)->user);

	dev_info(ni->dev,
		 "DSA registered: %d user ports (lan1..lan%d) + CPU port %d\n",
		 CA_NI_DSA_USER_PORTS, CA_NI_DSA_USER_PORTS, CA_NI_DSA_CPU_PORT);
	return 0;
}

void cortina_ni_dsa_unregister(struct cortina_ni *ni)
{
	int i;

	if (ni->ds) {
		dsa_unregister_switch(ni->ds);
		ni->ds = NULL;
	}
	for (i = 0; i < CA_NI_DSA_USER_PORTS; i++)
		WRITE_ONCE(ni->lan_ndev[i], NULL);
}
