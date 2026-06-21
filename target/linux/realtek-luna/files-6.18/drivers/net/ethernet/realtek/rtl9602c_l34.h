/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * RTL9602C "Luna" hardware L3/L4 forwarding + NAPT engine.
 *
 * The SoC switch core carries an L3/L4 (routing + NAPT) accelerator reached
 * through a single indirect table-access block. This header describes that
 * block and the per-flow NAPT table model the flow-offload path programs so
 * established WAN<->LAN connections are NAT'd/forwarded in hardware instead of
 * on the CPU. Clean-room: register offsets, field positions and table geometry
 * are hardware facts, re-expressed here as new idiomatic definitions.
 *
 * Copyright (C) 2026 Confiared <contact@confiared.com>
 */
#ifndef _RTL9602C_L34_H
#define _RTL9602C_L34_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/io.h>

/*
 * Indirect table-access block (byte offsets within the switch-core MMIO).
 * A table op writes the data bank, issues a command with the table type and
 * entry index, then polls the matching EXE bit until the engine clears it.
 */
#define L34_CMD			0x800100	/* command/trigger register */
#define  L34_CMD_RD_EXE		BIT(25)		/* start read  (self-clears when done) */
#define  L34_CMD_WR_EXE		BIT(24)		/* start write (self-clears when done) */
#define  L34_CMD_TYPE_SHIFT	16		/* [19:16] table type */
#define  L34_CMD_TYPE_MASK	0xf
#define  L34_CMD_IDX_MASK	0xffff		/* [15:0] entry index */

#define L34_CLR			0x800104	/* per-table-type reset, self-clearing */
#define L34_RDATA		0x800108	/* read-data bank base  (word0..) */
#define L34_WDATA		0x80011c	/* write-data bank base (word0..) */
#define L34_SWTCR0		0x800010	/* engine control (NAT mode / lookup / route) */
#define  L34_SWTCR0_V6RT_EN	BIT(31)
#define  L34_SWTCR0_V4RT_EN	BIT(30)
#define  L34_SWTCR0_NATMODE_SH	10		/* [11:10] b0=L3 NAT en, b1=L4 NAT en */
#define  L34_SWTCR0_LOOKUP_SH	8		/* [9:8] 0=VLAN-base, 2=MAC-base */
#define L34_GLB_CFG		0x01106c	/* master L34 routing enable */
#define L34_NAPT_HIT		0x800400	/* outbound NAPT hit/age bitmap (idx/32 words) */

/*
 * Table types (L34_CMD_TYPE). The NAPT path is a hashed pair: an outbound slot
 * table (key hash -> inbound index) and an inbound rewrite table (the actual
 * 5-tuple + post-NAT addresses/ports). EXTIP/NEXTHOP/NETIF/ARP/L3ROUTE support
 * the routing the NAT entries reference.
 */
enum l34_tbl {
	L34_TBL_L3ROUTE		= 0,	/* 16 entries, 2 words: LPM route -> nexthop */
	L34_TBL_NEXTHOP		= 2,	/* 16 entries, 1 word:  egress intf + L2 (ARP) index */
	L34_TBL_NETIF		= 3,	/* 16 entries, 4 words: per-interface MAC/VLAN/MTU/IP */
	L34_TBL_EXTIP		= 4,	/* 8 entries,  3 words: external (WAN) IP -> nexthop */
	L34_TBL_NAPTR_IN	= 9,	/* 4096 entries, 3 words: inbound rewrite (NAT tuple) */
	L34_TBL_NAPT_OUT	= 10,	/* 4096 entries, 1 word:  outbound hash slot */
	L34_TBL_ARP		= 13,	/* 128 entries, 2 words: IP -> L2 (next-hop MAC) index */
};

/* Word counts per table type (data-bank words moved per op). */
#define L34_WORDS_L3ROUTE	2
#define L34_WORDS_NEXTHOP	1
#define L34_WORDS_NETIF		4
#define L34_WORDS_EXTIP		3
#define L34_WORDS_NAPTR_IN	3
#define L34_WORDS_NAPT_OUT	1
#define L34_WORDS_ARP		2

#define L34_NAPT_ENTRIES	4096
#define L34_NAPT_WAYS		4		/* 4-way bucket: index = (hash << 2) + way */

/*
 * Entry field layouts: {LSP, W} = LSB bit position within the packed entry and
 * field width in bits. The data bank is significance-ordered (w[i] holds entry
 * bits [32*i+31 : 32*i]); a field's word index is LSP/32. Hardware facts; the
 * naming/expression is original.
 */

/* NAPT_OUT (type 10, 1 word): outbound hash slot. The slot's table index is
 * itself the outbound hash; the word only points at the rewrite entry. */
#define L34_NAPT_HASHIN_IDX_LSP	0	/* -> NAPTR_IN entry index */
#define L34_NAPT_HASHIN_IDX_W	12
#define L34_NAPT_VALID_LSP	12
#define L34_NAPT_VALID_W	1
#define L34_NAPT_PRIVALID_LSP	13
#define L34_NAPT_PRIVALID_W	1
#define L34_NAPT_PRIORITY_LSP	14
#define L34_NAPT_PRIORITY_W	3

/* NAPTR_IN (type 9, 3 words): the inbound / rewrite entry. */
#define L34_NAPTR_INTIP_LSP	0	/* internal (LAN) host IP */
#define L34_NAPTR_INTIP_W	32
#define L34_NAPTR_INTPORT_LSP	32	/* internal host L4 port */
#define L34_NAPTR_INTPORT_W	16
#define L34_NAPTR_REMHASH_LSP	48	/* remote-peer confirm hash (type 2/3) */
#define L34_NAPTR_REMHASH_W	16
#define L34_NAPTR_EXTIPIDX_LSP	64	/* -> EXTIP slot (WAN src IP + next-hop) */
#define L34_NAPTR_EXTIPIDX_W	3
#define L34_NAPTR_EXTPORT_LSP	67	/* post-NAT (WAN) L4 port */
#define L34_NAPTR_EXTPORT_W	16
#define L34_NAPTR_TCP_LSP	83	/* 1 = TCP, 0 = UDP */
#define L34_NAPTR_TCP_W		1
#define L34_NAPTR_VALID_LSP	84	/* 2-bit valid / NAT-type (modes below) */
#define L34_NAPTR_VALID_W	2
#define L34_NAPTR_PRIVALID_LSP	86
#define L34_NAPTR_PRIVALID_W	1
#define L34_NAPTR_PRIORITY_LSP	87
#define L34_NAPTR_PRIORITY_W	3
/* NAPTR VALID/type: 0 invalid; 1 full-cone (remHash ignored); 2 port-restricted
 * (remHash = hash(remIP,remPort)); 3 restricted-cone (remHash = hash(remIP)). */
#define L34_NAPTR_TYPE_INVALID	0
#define L34_NAPTR_TYPE_FULLCONE	1
#define L34_NAPTR_TYPE_IPPORT	2
#define L34_NAPTR_TYPE_IP	3

/* EXTIP (type 4, 3 words, 8 slots): a NAPTR's EXTIP_IDX points here for the WAN
 * source address + the next-hop the rewritten frame egresses through. */
#define L34_EXTIP_INTIP_LSP	0
#define L34_EXTIP_INTIP_W	32
#define L34_EXTIP_EXTIP_LSP	32	/* WAN external IP */
#define L34_EXTIP_EXTIP_W	32
#define L34_EXTIP_VALID_LSP	64
#define L34_EXTIP_VALID_W	1
#define L34_EXTIP_TYPE_LSP	65	/* 0 = NAPT */
#define L34_EXTIP_TYPE_W	2
#define L34_EXTIP_NHIDX_LSP	67	/* -> NEXTHOP slot */
#define L34_EXTIP_NHIDX_W	4

/* NETIF (type 3, 4 words, 16 slots): per-interface MAC/VLAN/MTU/IP. */
#define L34_NETIF_VALID_LSP	0
#define L34_NETIF_VALID_W	1
#define L34_NETIF_VLANID_LSP	1
#define L34_NETIF_VLANID_W	12
#define L34_NETIF_GMAC_LSP	13	/* 48-bit gateway/source MAC, spans w0..w1 */
#define L34_NETIF_GMAC_W	48
#define L34_NETIF_MACMASK_LSP	61
#define L34_NETIF_MACMASK_W	3
#define L34_NETIF_ENRTR_LSP	64	/* enable routing */
#define L34_NETIF_ENRTR_W	1
#define L34_NETIF_MTU_LSP	65
#define L34_NETIF_MTU_W		14
#define L34_NETIF_L34_LSP	82	/* classify this interface into the L34 NAT domain */
#define L34_NETIF_L34_W		1
#define L34_NETIF_IP_LSP	83	/* interface IP, spans w2..w3 */
#define L34_NETIF_IP_W		32

#define L34_EXTIP_SLOTS		8	/* EXTIP/EXTIP_IDX is 3-bit: netif idx must be < 8 */
#define L34_NETIF_DEF_VLAN	1
#define L34_NETIF_DEF_MTU	1500
#define L34_NETIF_DEF_MACMASK	0x7

/* NEXTHOP (type 2, 1 word, 16 slots). nhIdx is an L2-unicast-table index. */
#define L34_NH_TYPE_LSP		0	/* 0 = ETHER, 1 = PPPOE */
#define L34_NH_TYPE_W		1
#define L34_NH_IFIDX_LSP	1	/* -> NETIF slot */
#define L34_NH_IFIDX_W		4
#define L34_NH_NHIDX_LSP	8	/* -> L2 unicast entry holding the dst MAC */
#define L34_NH_NHIDX_W		11

/* ARP_CAM (type 13, 2 words, 128 slots). The MAC lives in the L2 table; this
 * holds the gateway IP and the L2 index where its MAC is resolved. */
#define L34_ARP_IP_LSP		0
#define L34_ARP_IP_W		32
#define L34_ARP_VALID_LSP	32
#define L34_ARP_VALID_W		1
#define L34_ARP_NHIDX_LSP	33	/* -> L2 unicast entry */
#define L34_ARP_NHIDX_W		11

/* L3 ROUTE (type 0, 2 words, 16 slots). The per-netif local route (process=ARP)
 * classifies an ingress frame into the L34 NAT domain and sets US/DS direction;
 * it is what an offloaded NAPT flow needs in order to match. Slot = netif idx. */
#define L34_RT_IP_LSP		0
#define L34_RT_IP_W		32
#define L34_RT_MASK_LSP		32	/* prefix code 0..31 (0 => /1 anchor) */
#define L34_RT_MASK_W		5
#define L34_RT_VALID_LSP	37
#define L34_RT_VALID_W		1
#define L34_RT_PROCESS_LSP	38	/* 0=CPU 1=DROP 2=ARP(local) 3=NH(global) */
#define L34_RT_PROCESS_W	2
#define L34_RT_INT_LSP		40	/* 1 = LAN, 0 = WAN */
#define L34_RT_INT_W		1
#define L34_RT_DENTIF_LSP	41	/* netif index (local-route view of [44:41]) */
#define L34_RT_DENTIF_W		4
#define L34_RT_RT2WANINF_LSP	45	/* route to WAN interface */
#define L34_RT_RT2WANINF_W	1
#define L34_RT_PROCESS_CPU	0	/* terminate locally (to the CPU) */
#define L34_RT_PROCESS_ARP	2

/*
 * L2 unicast table (the gateway/peer destination MAC). Reached through a
 * SEPARATE indirect block from the L34 NAT block: a MAC-method insert lets the
 * engine hash the MAC, place it in a free way, and report the assigned index
 * that NEXTHOP/ARP nhIdx then reference.
 */
#define L2_CMD			0x12000
#define  L2_CMD_TYPE_SH		0	/* [2:0] table type (0 = L2_UC) */
#define  L2_CMD_WR		BIT(3)	/* 0 = read, 1 = write */
#define  L2_CMD_METHOD_SH	4	/* [6:4] 0 = MAC-hash, 1 = direct address */
#define L2_STS			0x12004
#define  L2_STS_ADDR_MASK	0x3ff	/* [9:0] hardware-assigned entry address */
#define  L2_STS_CAM		BIT(10)	/* hash(0)/CAM(1): index = (cam << 10) | addr */
#define  L2_STS_HIT		BIT(12)
#define  L2_STS_BUSY		BIT(13)
#define L2_WDATA		0x12008	/* word0..2 @ +4 */
#define L2_RDATA		0x1201c	/* word0..2 @ +4 */
#define L34_WORDS_L2UC		3
#define L2_METHOD_MAC		0

/* L2_UC entry (3 words). The 48-bit MAC is packed octet[0] at the field MSB. */
#define L2UC_MAC_LSP		0
#define L2UC_STATIC_LSP		62	/* no-source-learn => static (not aged out) */
#define L2UC_STATIC_W		1
#define L2UC_SPA_LSP		65	/* [66:65] egress port */
#define L2UC_SPA_W		2
#define L2UC_AGE_LSP		67	/* static forces this non-zero */
#define L2UC_AGE_W		3
#define L2UC_ARPUSED_LSP	73
#define L2UC_ARPUSED_W		1
#define L2UC_VALID_LSP		77
#define L2UC_VALID_W		1

/* Per-flow programming request, filled by the flow-offload glue (endian-safe:
 * addresses/ports are kept in host order here and packed with explicit math). */
struct l34_flow {
	u8	l4proto;		/* IPPROTO_TCP / IPPROTO_UDP */
	u32	orig_sip, orig_dip;	/* ingress (original-direction) 5-tuple */
	u16	orig_sport, orig_dport;
	u32	nat_sip, nat_dip;	/* post-NAT addresses (0 = unchanged) */
	u16	nat_sport, nat_dport;	/* post-NAT ports (0 = unchanged) */
	u8	egress_netif;		/* L34_TBL_NETIF index of the output interface */
	u8	nexthop;		/* L34_TBL_NEXTHOP index (gateway L2) */
	u16	hw_index;		/* assigned NAPT slot, valid after add (for del/stats) */
};

struct rtl9602c_l34 {
	void __iomem	*sw;		/* switch-core MMIO base (offsets above are relative) */
	struct mutex	lock;		/* serialises table ops */
	bool		ready;		/* table-access plumbing usable */
	bool		engine_on;	/* NAT engine enabled (deferred, lazy) */
};

/* Public API (mainline flow-offload glue calls these). */
int  rtl9602c_l34_init(struct rtl9602c_l34 *l, void __iomem *sw);
int  rtl9602c_l34_wan_setup(struct rtl9602c_l34 *l, u8 idx, u32 wan_ip,
			    const u8 *wan_mac, u32 gw_ip, const u8 *gw_mac,
			    u8 wan_port, u16 vlan);
int  rtl9602c_l34_lan_setup(struct rtl9602c_l34 *l, u8 idx, u32 lan_ip,
			    const u8 *lan_mac, u32 lan_net, u8 prefix, u16 vlan);
int  rtl9602c_l34_flow_add(struct rtl9602c_l34 *l, struct l34_flow *f);
int  rtl9602c_l34_flow_del(struct rtl9602c_l34 *l, struct l34_flow *f);
int  rtl9602c_l34_flow_hit(struct rtl9602c_l34 *l, u16 hw_index, bool *active);
void rtl9602c_l34_proc_init(struct rtl9602c_l34 *l);	/* bring-up test harness */

#endif /* _RTL9602C_L34_H */
