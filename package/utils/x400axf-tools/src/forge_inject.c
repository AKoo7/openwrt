/* forge_inject - static aarch64 raw AF_PACKET injector for the RTL9607F "Elnath"
 * HW-flow-offload proof.  Forges a foreign-src LAN-client frame (TCP / UDP /
 * ICMP, over IPv4 or IPv6) and transmits it out a chosen netdev via
 * AF_PACKET/SOCK_RAW.  Combined with an internal MAC/PHY loopback on that port
 * the frame loops back and physically ingresses the L3FE LAN ingress as if from
 * a real LAN client - so a matching pre-installed HW flow can HW-forward it US
 * to the WAN (CPU bypassed).  No scapy/tcpdump/python needed.  Root only.
 *
 * usage: forge_inject <iface> <smac> <dmac> <sip> <dip> <proto> <sport> <dport> <count> [paylen]
 *   proto = udp | tcp | icmp   (icmp: sport=id, dport=seq)
 *   IP version is inferred from <sip> (dotted=v4, colon=v6); sip/dip must match.
 * SPDX-License-Identifier: GPL-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <net/if.h>
#include <netpacket/packet.h>
#include <net/ethernet.h>

static int pmac(const char *s, uint8_t *m)
{
	return sscanf(s, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
		      &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) == 6 ? 0 : -1;
}
/* one's-complement sum accumulator (returns folded final complement) */
static uint32_t sum_add(const void *d, int len, uint32_t sum)
{
	const uint16_t *p = d;
	while (len > 1) { sum += *p++; len -= 2; }
	if (len) sum += *(const uint8_t *)p;
	return sum;
}
static uint16_t sum_fin(uint32_t sum)
{
	while (sum >> 16) sum = (sum & 0xffff) + (sum >> 16);
	return (uint16_t)~sum;
}

int main(int argc, char **argv)
{
	if (argc < 10) {
		fprintf(stderr, "usage: %s <iface> <smac> <dmac> <sip> <dip> <proto:udp|tcp|icmp> <sport> <dport> <count> [paylen]\n", argv[0]);
		return 2;
	}
	const char *iface = argv[1], *pr = argv[6];
	uint8_t smac[6], dmac[6];
	if (pmac(argv[2], smac) || pmac(argv[3], dmac)) { fprintf(stderr, "bad mac\n"); return 2; }
	int v6 = strchr(argv[4], ':') != NULL;
	uint8_t sa[16], da[16];
	int alen = v6 ? 16 : 4;
	if (inet_pton(v6 ? AF_INET6 : AF_INET, argv[4], sa) != 1 ||
	    inet_pton(v6 ? AF_INET6 : AF_INET, argv[5], da) != 1) { fprintf(stderr, "bad ip\n"); return 2; }
	uint16_t sport = atoi(argv[7]), dport = atoi(argv[8]);
	int count = atoi(argv[9]);
	int paylen = argc > 10 ? atoi(argv[10]) : 32;
	if (paylen < 0) paylen = 0; if (paylen > 1400) paylen = 1400;

	int is_udp = !strcmp(pr, "udp"), is_tcp = !strcmp(pr, "tcp"), is_icmp = !strcmp(pr, "icmp");
	if (!is_udp && !is_tcp && !is_icmp) { fprintf(stderr, "proto must be udp|tcp|icmp\n"); return 2; }
	uint8_t l4proto = is_udp ? 17 : is_tcp ? 6 : v6 ? 58 : 1;	/* 58=icmpv6 1=icmpv4 */

	uint8_t frame[1600];
	memset(frame, 0, sizeof(frame));
	memcpy(frame + 0, dmac, 6);
	memcpy(frame + 6, smac, 6);
	frame[12] = v6 ? 0x86 : 0x08; frame[13] = v6 ? 0xdd : 0x00;

	uint8_t *l3 = frame + 14;
	int l4hdr = is_udp ? 8 : is_tcp ? 20 : 8;
	int l4len = l4hdr + paylen;
	uint8_t *l4;

	if (!v6) {			/* IPv4 */
		int iptot = 20 + l4len;
		l3[0] = 0x45; l3[2] = iptot >> 8; l3[3] = iptot & 0xff;
		l3[4] = 0x13; l3[5] = 0x37; l3[6] = 0x40;	/* id, DF */
		l3[8] = 64; l3[9] = l4proto;
		memcpy(l3 + 12, sa, 4); memcpy(l3 + 16, da, 4);
		*(uint16_t *)(l3 + 10) = 0;
		*(uint16_t *)(l3 + 10) = sum_fin(sum_add(l3, 20, 0));
		l4 = l3 + 20;
	} else {			/* IPv6 */
		l3[0] = 0x60;
		l3[4] = l4len >> 8; l3[5] = l4len & 0xff;	/* payload length */
		l3[6] = l4proto; l3[7] = 64;			/* next-hdr, hop-limit */
		memcpy(l3 + 8, sa, 16); memcpy(l3 + 24, da, 16);
		l4 = l3 + 40;
	}

	/* L4 pseudo-header sum (TCP/UDP always; ICMPv6 yes; ICMPv4 no) */
	uint32_t ps = 0;
	int need_pseudo = is_udp || is_tcp || (is_icmp && v6);
	if (need_pseudo) {
		ps = sum_add(sa, alen, ps);
		ps = sum_add(da, alen, ps);
		uint16_t l4len_be = htons(l4len);
		ps += l4len_be;
		ps += htons(l4proto);
	}

	memset(l4 + l4hdr, 0x5a, paylen);
	if (is_udp) {
		*(uint16_t *)(l4 + 0) = htons(sport);
		*(uint16_t *)(l4 + 2) = htons(dport);
		*(uint16_t *)(l4 + 4) = htons(l4len);
		*(uint16_t *)(l4 + 6) = 0;
		uint16_t c = sum_fin(sum_add(l4, l4len, ps));
		*(uint16_t *)(l4 + 6) = c ? c : 0xffff;		/* v6 must be nonzero */
	} else if (is_tcp) {
		*(uint16_t *)(l4 + 0) = htons(sport);
		*(uint16_t *)(l4 + 2) = htons(dport);
		*(uint32_t *)(l4 + 4) = htonl(0x13370001);	/* seq */
		l4[12] = 0x50;					/* data off = 5 */
		l4[13] = 0x02;					/* SYN */
		*(uint16_t *)(l4 + 14) = htons(0xffff);		/* window */
		*(uint16_t *)(l4 + 16) = 0;
		*(uint16_t *)(l4 + 16) = sum_fin(sum_add(l4, l4len, ps));
	} else {			/* ICMP echo request */
		l4[0] = v6 ? 128 : 8; l4[1] = 0;
		*(uint16_t *)(l4 + 4) = htons(sport);		/* id */
		*(uint16_t *)(l4 + 6) = htons(dport);		/* seq */
		*(uint16_t *)(l4 + 2) = 0;
		*(uint16_t *)(l4 + 2) = sum_fin(sum_add(l4, l4len, ps));
	}

	int flen = (l4 - frame) + l4len;
	if (flen < 60) flen = 60;

	int fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
	if (fd < 0) { perror("socket(AF_PACKET)"); return 1; }
	int ifidx = if_nametoindex(iface);
	if (!ifidx) { perror("if_nametoindex"); return 1; }
	struct sockaddr_ll sll;
	memset(&sll, 0, sizeof(sll));
	sll.sll_family = AF_PACKET; sll.sll_ifindex = ifidx; sll.sll_halen = 6;
	memcpy(sll.sll_addr, dmac, 6);

	int sent = 0;
	for (int i = 0; i < count; i++) {
		if (sendto(fd, frame, flen, 0, (struct sockaddr *)&sll, sizeof(sll)) == flen)
			sent++;
		else if (i == 0) { perror("sendto"); return 1; }
	}
	printf("forge_inject: %s/%s sent %d/%d (%dB) %s->%s :%u->%u out %s\n",
	       v6 ? "v6" : "v4", pr, sent, count, flen, argv[4], argv[5], sport, dport, iface);
	close(fd);
	return 0;
}
