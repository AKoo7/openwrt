// SPDX-License-Identifier: GPL-2.0
/* Genetlink client for the open Maple "maple" family. Shared by maplectl/maple_snmp. */
#define _GNU_SOURCE
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <linux/netlink.h>
#include <linux/genetlink.h>

#include "maple_lib.h"
#include "maple_codec.h"	/* unpack the raw BE wire bytes */

#define MAPLE_GENL_NAME	"maple"
enum { A_PON_NI, A_ONU_ID, A_OP, A_CFG, A_STAT };
enum { C_GET_CFG, C_GET_STAT, C_SET_STATE, C_NI_GET_CFG, C_NI_GET_STAT };

struct nl_msg {
	struct nlmsghdr  nh;
	struct genlmsghdr gh;
	char payload[384];
};

static int g_fd = -1;
static uint16_t g_family;
static uint32_t g_seq = 1;

#ifndef NLA_ALIGNTO
#define NLA_ALIGNTO 4
#define NLA_ALIGN(n) (((n) + NLA_ALIGNTO - 1) & ~(NLA_ALIGNTO - 1))
#define NLA_HDRLEN ((int)NLA_ALIGN(sizeof(struct nlattr)))
#endif
static inline int nla_ok_(struct nlattr *a, int rem) {
	return rem >= (int)sizeof(struct nlattr) && a->nla_len >= (int)sizeof(struct nlattr) && (int)a->nla_len <= rem;
}
static inline struct nlattr *nla_next_(struct nlattr *a, int *rem) {
	int n = NLA_ALIGN(a->nla_len); *rem -= n; return (struct nlattr *)((char *)a + n);
}

static void *put_attr(struct nl_msg *m, int len, uint16_t type) {
	struct nlattr *a = (struct nlattr *)((char *)&m->nh + NLMSG_ALIGN(m->nh.nlmsg_len));
	a->nla_len = NLA_HDRLEN + len; a->nla_type = type;
	m->nh.nlmsg_len = NLMSG_ALIGN(m->nh.nlmsg_len) + NLA_ALIGN(NLA_HDRLEN + len);
	return a + 1;
}

static int xchg(struct nl_msg *m, struct nl_msg *r) {
	m->nh.nlmsg_type = g_family; m->nh.nlmsg_flags = NLM_F_REQUEST; m->nh.nlmsg_seq = g_seq++;
	m->gh.version = 1; m->gh.reserved = 0;
	if (send(g_fd, m, m->nh.nlmsg_len, 0) < 0) return -errno;
	ssize_t n = recv(g_fd, r, sizeof(*r), 0);
	if (n < 0) return -errno;
	if (r->nh.nlmsg_type == NLMSG_ERROR) return -(((struct nlmsgerr *)&r->gh)->error);
	return 0;
}

static struct nlattr *attr_find(struct nl_msg *r, uint16_t type) {
	int rem = r->nh.nlmsg_len - NLMSG_LENGTH(GENL_HDRLEN);
	struct nlattr *a = (struct nlattr *)((char *)&r->gh + NLMSG_ALIGN(GENL_HDRLEN));
	for (; nla_ok_(a, rem); a = nla_next_(a, &rem))
		if (a->nla_type == type) return a;
	return NULL;
}

int maple_gnl_open(void) {
	struct nl_msg m, r;
	int *p;

	g_fd = socket(AF_NETLINK, SOCK_RAW, NETLINK_GENERIC);
	if (g_fd < 0) return -errno;
	struct timeval tv = { .tv_sec = 2 };
	setsockopt(g_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	struct sockaddr_nl sa = { .nl_family = AF_NETLINK };
	bind(g_fd, (struct sockaddr *)&sa, sizeof(sa));

	memset(&m, 0, sizeof(m));
	m.nh.nlmsg_len = NLMSG_LENGTH(sizeof(struct genlmsghdr));
	m.nh.nlmsg_type = GENL_ID_CTRL;
	m.gh.cmd = CTRL_CMD_GETFAMILY;
	p = put_attr(&m, strlen(MAPLE_GENL_NAME) + 1, CTRL_ATTR_FAMILY_NAME);
	strcpy((char *)p, MAPLE_GENL_NAME);
	if (xchg(&m, &r)) return -ENOTSUP;
	/* find CTRL_ATTR_FAMILY_ID in the reply */
	int rem = r.nh.nlmsg_len - NLMSG_LENGTH(GENL_HDRLEN);
	struct nlattr *ca = (struct nlattr *)((char *)&r.gh + NLMSG_ALIGN(GENL_HDRLEN));
	for (; nla_ok_(ca, rem); ca = nla_next_(ca, &rem))
		if (ca->nla_type == CTRL_ATTR_FAMILY_ID) { g_family = *(uint16_t *)(ca + 1); return 0; }
	return -ENOTSUP;
}

int maple_gnl_onu_get_cfg(uint8_t pon, uint16_t onu, struct maple_onu_cfg *out) {
	struct nl_msg m, r; struct nlattr *a;
	memset(&m, 0, sizeof(m)); m.nh.nlmsg_len = NLMSG_LENGTH(sizeof(struct genlmsghdr));
	m.gh.cmd = C_GET_CFG;
	*(uint8_t *)put_attr(&m, 1, A_PON_NI) = pon;
	*(uint16_t *)put_attr(&m, 2, A_ONU_ID) = onu;
	if (xchg(&m, &r)) return -errno;
	a = attr_find(&r, A_CFG); if (!a) return -ENODATA;
	/* ABI carries raw BE wire bytes -> unpack on the host (endian-independent) */
	return maple_onu_cfg_unpack(out, a + 1, a->nla_len - NLA_HDRLEN) ? -EBADMSG : 0;
}

int maple_gnl_onu_get_stat(uint8_t pon, uint16_t onu, struct maple_onu_stat *out) {
	struct nl_msg m, r; struct nlattr *a;
	memset(&m, 0, sizeof(m)); m.nh.nlmsg_len = NLMSG_LENGTH(sizeof(struct genlmsghdr));
	m.gh.cmd = C_GET_STAT;
	*(uint8_t *)put_attr(&m, 1, A_PON_NI) = pon;
	*(uint16_t *)put_attr(&m, 2, A_ONU_ID) = onu;
	if (xchg(&m, &r)) return -errno;
	a = attr_find(&r, A_STAT); if (!a) return -ENODATA;
	return maple_onu_stat_unpack(out, a + 1, a->nla_len - NLA_HDRLEN) ? -EBADMSG : 0;
}

int maple_gnl_onu_set_state(uint8_t pon, uint16_t onu, uint32_t op) {
	struct nl_msg m, r;
	memset(&m, 0, sizeof(m)); m.nh.nlmsg_len = NLMSG_LENGTH(sizeof(struct genlmsghdr));
	m.gh.cmd = C_SET_STATE;
	*(uint8_t *)put_attr(&m, 1, A_PON_NI) = pon;
	*(uint16_t *)put_attr(&m, 2, A_ONU_ID) = onu;
	*(uint32_t *)put_attr(&m, 4, A_OP) = op;
	return xchg(&m, &r) ? -errno : 0;
}

int maple_gnl_ni_get_cfg(uint8_t pon, struct maple_ni_cfg *out) {
	struct nl_msg m, r; struct nlattr *a;
	memset(&m, 0, sizeof(m)); m.nh.nlmsg_len = NLMSG_LENGTH(sizeof(struct genlmsghdr));
	m.gh.cmd = C_NI_GET_CFG;
	*(uint8_t *)put_attr(&m, 1, A_PON_NI) = pon;
	if (xchg(&m, &r)) return -errno;
	a = attr_find(&r, A_CFG); if (!a) return -ENODATA;
	return maple_ni_cfg_unpack(out, a + 1, a->nla_len - NLA_HDRLEN) ? -EBADMSG : 0;
}

int maple_gnl_ni_get_stat(uint8_t pon, struct maple_ni_stat *out) {
	struct nl_msg m, r; struct nlattr *a;
	memset(&m, 0, sizeof(m)); m.nh.nlmsg_len = NLMSG_LENGTH(sizeof(struct genlmsghdr));
	m.gh.cmd = C_NI_GET_STAT;
	*(uint8_t *)put_attr(&m, 1, A_PON_NI) = pon;
	if (xchg(&m, &r)) return -errno;
	a = attr_find(&r, A_STAT); if (!a) return -ENODATA;
	return maple_ni_stat_unpack(out, a + 1, a->nla_len - NLA_HDRLEN) ? -EBADMSG : 0;
}

const char *maple_state_name(uint32_t s) {
	static const char *n[] = {"NOT_CONFIGURED","UNAWARE","PROCESSING","ACTIVE","INACTIVE",
		"DISABLED","ACTIVE_STANDBY","LOW_POWER_DOZE","LOW_POWER_SLEEP","LOW_POWER_WATCH","AWAKE_FREE"};
	return s < sizeof(n)/sizeof(n[0]) ? n[s] : "?";
}
const char *maple_deact_reason_name(uint32_t r) {
	static const char *n[] = {"NONE","LOS","DEACTIVATION","FORCE_DEACTIVATION",
		"PASSWORD_AUTHENTICATION","LOKI","ACK_TIMEOUT","ONU_ALARM"};
	return r < sizeof(n)/sizeof(n[0]) ? n[r] : "?";
}
