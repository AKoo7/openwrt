// SPDX-License-Identifier: GPL-2.0
/*
 * cortina-omci-bridge.c -- kernel seam for the stock Zyxel/EcoNet omci_app daemon
 *
 * Presents the two planes the unmodified stock OMCI daemon uses, so it can run
 * in a chroot on this OpenWrt port and drive the REAL provisioned MIB, bridged
 * to the clean-room cortina-gpon MAC driver.  Contract recovered from
 * libomci_mib.so / libpr.so and verified by the M1 dry run (see kshim.c):
 *
 *   PLANE A -- OMCI messages over NETLINK
 *     socket(AF_NETLINK, SOCK_RAW, proto=2)  OMCI PDUs   -> omci_msg_sk
 *     socket(AF_NETLINK, SOCK_RAW, proto=1)  async events -> omci_evt_sk
 *     REGISTER  (nlmsg_len == 28): nlmsghdr(16) + {u16 ver; u16 type; u32 pid;
 *                u8 action(1=REG/2=DEREG); u8 pad; u16 appId(1988)}.  appId @+26,
 *                daemon portid taken from NETLINK_CB(skb).portid.
 *     US-TX     (nlmsg_len == 2016): nlmsghdr(16) + redirect-hdr(16) +
 *                payload(1984); OMCI PDU @ +32 -> cg_omci_tx().
 *     DS-RX     kernel -> daemon: nlmsghdr(16) + {u32 class=0; u8 pdu @+4;
 *                u32 reallen @+1984}  (total 2004) -> netlink_unicast().
 *
 *   PLANE B -- control over getsockopt(rawINETfd, level=0, optname=5400, 264)
 *     req = {u32 cmd; u32 len; u8 data[256]}.  cmds: 0=resetMib 2=setDevMode
 *     3=getDevCapabilities 4=getDevIdVersion 13=getOnuState 14=setSerialNum
 *     17=activateGpon (+~90 datapath wrappers).  getOnuState MUST return 5 (O5)
 *     when the driver is at O5 -- that unlocks the daemon's TX gate.  Every other
 *     cmd is stubbed to success (zeros) for M2; the datapath wrappers land in M5.
 *
 *   /proc/omci/devMode -- the daemon echoes "router"/"bridge" into it.
 *
 * WIRE-BYTE NOTE: the wrapper fields (nlmsg_len, class, appId, reallen) are
 * host-order (aarch64 == little-endian == this kernel); only the OMCI PDU is
 * big-endian, and it is copied through opaquely.
 *
 * NETLINK PROTO-2 NOTE: proto 2 is NETLINK_USERSOCK, which the core registers at
 * boot but WITHOUT a portid-0 kernel socket.  __netlink_kernel_create() stores
 * our .input per-socket (nlk->netlink_rcv) before the nl_table[].registered
 * check and inserts our kernel socket at portid 0 in proto 2's own hash; for an
 * already-registered proto it merely bumps a refcount.  So co-registering here
 * is fine -- the daemon's messages (sent to nl_pid 0) dispatch to omci_msg_input
 * -- and NO core-kernel patch is needed.  proto 1 (NETLINK_UNUSED) is free too.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/capability.h>
#include <linux/rcupdate.h>
#include <linux/netlink.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <net/sock.h>
#include <net/net_namespace.h>
#include <net/netlink.h>
#include <linux/unaligned.h>

#include "cortina-omci-bridge.h"

/* ---- contract constants ---- */
#define OMCI_NL_PROTO_MSG	2	/* NETLINK proto for OMCI PDUs (co-registers)   */
#define OMCI_NL_PROTO_EVT	1	/* NETLINK proto for async events (UNUSED)      */
#define OMCI_NL_GROUPS		32	/* daemon binds nl_groups up to bit 21          */

#define OMCI_APPID		1988	/* 0x7C4 */

#define OMCI_MSG_REGISTER_LEN	28	/* nlmsghdr(16) + 12B register body            */
#define OMCI_MSG_USTX_LEN	2016	/* nlmsghdr(16) + rdh(16) + payload(1984)      */
#define OMCI_USTX_PDU_OFF	16	/* OMCI PDU inside the message body (after rdh) */
#define OMCI_REG_ACTION_OFF	8	/* action byte inside body                      */
#define OMCI_REG_APPID_OFF	10	/* appId u16 inside body                        */
#define OMCI_REG_ACTION_REG	1
#define OMCI_REG_ACTION_DEREG	2

/* DS-RX frame layout, relative to nlmsg_data() (the body) */
#define OMCI_DS_BODY_LEN	1988	/* u32 class + 1980 pdu region + u32 reallen    */
#define OMCI_DS_PDU_OFF		4
#define OMCI_DS_PDU_MAX		1980
#define OMCI_DS_REALLEN_OFF	1984

/* Plane B */
#define OMCI_SOCKOPT_CMD	5400	/* 0x1518 */
#define OMCI_CMD_RESET_MIB	0
#define OMCI_CMD_SET_DEVMODE	2
#define OMCI_CMD_GET_DEVCAPS	3
#define OMCI_CMD_GET_DEVID_VER	4
#define OMCI_CMD_GET_ONU_STATE	13
#define OMCI_CMD_SET_SERIALNUM	14
#define OMCI_CMD_ACTIVATE_GPON	17

struct omci_ctrl_req {
	u32 cmd;
	u32 len;
	u8  data[256];
};	/* 264 bytes, matches the daemon's optlen */

/* ---- runtime arm switch ---- */
static bool cg_omci_userspace;
module_param_named(omci_userspace, cg_omci_userspace, bool, 0644);
MODULE_PARM_DESC(omci_userspace,
	"divert downstream OMCI to the userspace stock daemon (default 0)");

/* ---- bridge state ---- */
static struct sock *omci_msg_sk;		/* proto 2 kernel netlink */
static struct sock *omci_evt_sk;		/* proto 1 kernel netlink */
static u32 omci_daemon_portid;			/* set on REGISTER; 0 = no daemon */
static u16 omci_appid;
static bool omci_sockopt_reg;			/* nf_register_sockopt succeeded */
static struct proc_dir_entry *omci_proc_dir;
static struct cortina_gpon *bridge_cg;

/* Is the bridge live -- armed AND a daemon registered?  Used both here and by
 * cortina-gpon.c to suppress the in-kernel autonomous US-OMCI emitters so
 * exactly one OMCI stack transmits on the OMCC. */
bool cg_omci_bridge_armed(void)
{
	return cg_omci_userspace && READ_ONCE(omci_daemon_portid) != 0;
}

/* =====================================================================
 * PLANE A -- netlink
 * ===================================================================== */

/* daemon -> kernel, OMCI message socket (proto 2).  Runs in the daemon's
 * process context (netlink_unicast_kernel calls us inline on its sendmsg). */
static void omci_msg_input(struct sk_buff *skb)
{
	struct cortina_gpon *cg;
	struct nlmsghdr *nlh;
	u32 portid = NETLINK_CB(skb).portid;
	const u8 *body;

	/* The stock daemon runs as root; refuse an unprivileged process trying
	 * to hijack the downstream stream or inject upstream OMCI onto the PON. */
	if (!netlink_capable(skb, CAP_NET_ADMIN))
		return;
	if (skb->len < NLMSG_HDRLEN)
		return;
	nlh = nlmsg_hdr(skb);
	if (nlh->nlmsg_len < NLMSG_HDRLEN || nlh->nlmsg_len > skb->len)
		return;
	body = nlmsg_data(nlh);

	if (nlh->nlmsg_len == OMCI_MSG_REGISTER_LEN) {
		u8  action = body[OMCI_REG_ACTION_OFF];
		u16 appid  = get_unaligned_le16(body + OMCI_REG_APPID_OFF);

		if (action == OMCI_REG_ACTION_REG) {
			omci_appid = appid;
			WRITE_ONCE(omci_daemon_portid, portid);
			pr_info("cortina-omci-bridge: daemon REGISTER appId=%u portid=%u\n",
				appid, portid);
		} else if (action == OMCI_REG_ACTION_DEREG) {
			WRITE_ONCE(omci_daemon_portid, 0);
			pr_info("cortina-omci-bridge: daemon DEREGISTER appId=%u\n", appid);
		}
		return;
	}

	if (nlh->nlmsg_len >= OMCI_MSG_USTX_LEN) {
		/* redirect-hdr(16) then the 1984B payload; OMCI PDU is the first
		 * 48 bytes of the payload.  cg_omci_tx clamps to OMCI_LEN.  Only
		 * inject upstream while the bridge is armed (matches the DS/AVC
		 * gates so exactly one OMCI stack transmits). */
		const u8 *pdu48 = body + OMCI_USTX_PDU_OFF;

		if (!cg_omci_bridge_armed())
			return;
		/* RCU-read the cg so cg_omci_bridge_exit()'s synchronize_net()
		 * drains this process-context caller before devm frees cg. */
		rcu_read_lock();
		cg = READ_ONCE(bridge_cg);
		if (cg)
			cg_omci_tx(cg, pdu48);
		rcu_read_unlock();
		return;
	}
	/* anything else: ignore (the daemon sends only REGISTER + US-TX here) */
}

/* daemon -> kernel, events socket (proto 1).  The daemon only *receives* async
 * events here; nothing is expected inbound, so this is a sink. */
static void omci_evt_input(struct sk_buff *skb)
{
}

/* kernel -> daemon: hand a downstream OMCI PDU to the registered daemon. */
bool cg_omci_bridge_ds(const u8 *pdu, unsigned int len)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	struct sock *sk;
	u8 *data;
	u32 portid;
	int rc;

	if (!cg_omci_userspace)
		return false;
	portid = READ_ONCE(omci_daemon_portid);
	sk = READ_ONCE(omci_msg_sk);
	if (!portid || !sk)
		return false;

	if (len > OMCI_DS_PDU_MAX)
		len = OMCI_DS_PDU_MAX;

	/* Called from the NI CPU-RX softirq path: atomic alloc + nonblock unicast. */
	skb = nlmsg_new(OMCI_DS_BODY_LEN, GFP_ATOMIC);
	if (!skb)
		return true;	/* armed: consumed (dropped), do not run in-kernel path */
	nlh = nlmsg_put(skb, 0, 0, 0, OMCI_DS_BODY_LEN, 0);
	if (!nlh) {
		kfree_skb(skb);
		return true;
	}
	data = nlmsg_data(nlh);
	memset(data, 0, OMCI_DS_BODY_LEN);
	/* {u32 class=0 @+0; u8 pdu @+4; u32 reallen @+1984} -- class stays 0 */
	memcpy(data + OMCI_DS_PDU_OFF, pdu, len);
	put_unaligned_le32(len, data + OMCI_DS_REALLEN_OFF);

	NETLINK_CB(skb).dst_group = 0;
	rc = netlink_unicast(sk, skb, portid, MSG_DONTWAIT);
	if (rc == -ECONNREFUSED || rc == -EPIPE) {
		/* Daemon socket gone (e.g. it exited without DEREGISTER): forget
		 * it so the in-kernel responder resumes instead of black-holing
		 * every DS OMCI PDU, and fall back for this PDU too. */
		WRITE_ONCE(omci_daemon_portid, 0);
		return false;
	}
	/* A transient error (e.g. -EAGAIN, the daemon's rcvbuf filling during a
	 * MIB-upload burst) leaves the daemon registered and still owning the
	 * OMCC: DROP the PDU so the OLT retransmits.  Never let the in-kernel
	 * responder answer from a MIB that has diverged from the daemon's. */
	return true;
}

/* =====================================================================
 * PLANE B -- control over nf getsockopt(level=0, optname=5400)
 * ===================================================================== */

static int omci_sockopt_get(struct sock *sk, int optval, void __user *user, int *len)
{
	struct cortina_gpon *cg;
	struct omci_ctrl_req req;
	int rlen = *len;

	if (optval != OMCI_SOCKOPT_CMD)
		return -ENOPROTOOPT;
	if (rlen < (int)(2 * sizeof(u32)))
		return -EINVAL;
	if (rlen > (int)sizeof(req))
		rlen = sizeof(req);

	memset(&req, 0, sizeof(req));
	if (copy_from_user(&req, user, rlen))
		return -EFAULT;

	switch (req.cmd) {
	case OMCI_CMD_GET_ONU_STATE:
		memset(req.data, 0, sizeof(req.data));
		cg = READ_ONCE(bridge_cg);
		if (cg)
			put_unaligned_le32(cg_omci_bridge_onu_state(cg), req.data);
		break;
	default:
		/* stub success: zeroed data -- see contract / M1 kshim. */
		memset(req.data, 0, sizeof(req.data));
		break;
	}

	if (copy_to_user(user, &req, rlen))
		return -EFAULT;
	return 0;
}

static int omci_sockopt_set(struct sock *sk, int optval, sockptr_t arg, unsigned int len)
{
	/* The stock daemon drives control bidirectionally over getsockopt; a
	 * setsockopt path is accepted as success for completeness. */
	if (optval != OMCI_SOCKOPT_CMD)
		return -ENOPROTOOPT;
	return 0;
}

static struct nf_sockopt_ops omci_sockopts = {
	.pf		= PF_INET,
	.set_optmin	= OMCI_SOCKOPT_CMD,
	.set_optmax	= OMCI_SOCKOPT_CMD + 1,	/* ranges are non-inclusive */
	.set		= omci_sockopt_set,
	.get_optmin	= OMCI_SOCKOPT_CMD,
	.get_optmax	= OMCI_SOCKOPT_CMD + 1,
	.get		= omci_sockopt_get,
	.owner		= THIS_MODULE,
};

/* =====================================================================
 * /proc/omci/devMode
 * ===================================================================== */

static char omci_devmode[16] = "router";

static int omci_devmode_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", omci_devmode);
	return 0;
}

static int omci_devmode_open(struct inode *inode, struct file *file)
{
	return single_open(file, omci_devmode_show, NULL);
}

static ssize_t omci_devmode_write(struct file *file, const char __user *ubuf,
				  size_t count, loff_t *ppos)
{
	char buf[16];
	char *p;
	size_t n = min(count, sizeof(buf) - 1);

	if (copy_from_user(buf, ubuf, n))
		return -EFAULT;
	buf[n] = '\0';
	p = strim(buf);			/* strim() reports the leading trim via its return */
	if (strcmp(p, "router") && strcmp(p, "bridge"))
		return -EINVAL;
	strscpy(omci_devmode, p, sizeof(omci_devmode));
	return count;
}

static const struct proc_ops omci_devmode_ops = {
	.proc_open	= omci_devmode_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
	.proc_write	= omci_devmode_write,
};

/* =====================================================================
 * lifecycle
 * ===================================================================== */

int cg_omci_bridge_init(struct cortina_gpon *cg)
{
	struct netlink_kernel_cfg mcfg = {
		.groups	= OMCI_NL_GROUPS,
		.input	= omci_msg_input,
	};
	struct netlink_kernel_cfg ecfg = {
		.groups	= OMCI_NL_GROUPS,
		.input	= omci_evt_input,
	};
	bool proc_ok = false;
	int ret;

	/* Start from a clean slate: a prior unbind may have left a stale
	 * registration if a REGISTER raced teardown. */
	WRITE_ONCE(omci_daemon_portid, 0);
	omci_appid = 0;
	WRITE_ONCE(bridge_cg, cg);

	omci_msg_sk = netlink_kernel_create(&init_net, OMCI_NL_PROTO_MSG, &mcfg);
	if (!omci_msg_sk)
		pr_warn("cortina-omci-bridge: netlink proto %d create failed\n",
			OMCI_NL_PROTO_MSG);

	omci_evt_sk = netlink_kernel_create(&init_net, OMCI_NL_PROTO_EVT, &ecfg);
	if (!omci_evt_sk)
		pr_warn("cortina-omci-bridge: netlink proto %d (events) create failed\n",
			OMCI_NL_PROTO_EVT);

	ret = nf_register_sockopt(&omci_sockopts);
	omci_sockopt_reg = (ret == 0);
	if (ret)
		pr_warn("cortina-omci-bridge: nf_register_sockopt failed (%d)\n", ret);

	omci_proc_dir = proc_mkdir("omci", NULL);
	if (omci_proc_dir)
		proc_ok = proc_create("devMode", 0644, omci_proc_dir,
				      &omci_devmode_ops) != NULL;

	pr_info("cortina-omci-bridge: init (msg=%d evt=%d sockopt=%d proc=%d); arm with cortina_gpon.omci_userspace=1\n",
		!!omci_msg_sk, !!omci_evt_sk, omci_sockopt_reg, proc_ok);
	return 0;
}

void cg_omci_bridge_exit(void)
{
	struct sock *msg_sk, *evt_sk;

	/* Stop publishing FIRST: any concurrent cg_rx_omci() softirq that is
	 * mid-flight in cg_omci_bridge_ds() has already loaded the socket, so
	 * NULL the publishers and drain in-flight readers before releasing. */
	WRITE_ONCE(bridge_cg, NULL);
	WRITE_ONCE(omci_daemon_portid, 0);
	msg_sk = omci_msg_sk;
	evt_sk = omci_evt_sk;
	WRITE_ONCE(omci_msg_sk, NULL);
	WRITE_ONCE(omci_evt_sk, NULL);
	synchronize_net();		/* wait out in-flight NAPI/softirq ds() callers */

	if (omci_proc_dir) {
		proc_remove(omci_proc_dir);	/* drops the whole /proc/omci subtree */
		omci_proc_dir = NULL;
	}
	if (omci_sockopt_reg) {
		nf_unregister_sockopt(&omci_sockopts);
		omci_sockopt_reg = false;
	}
	if (msg_sk)
		netlink_kernel_release(msg_sk);
	if (evt_sk)
		netlink_kernel_release(evt_sk);
}
