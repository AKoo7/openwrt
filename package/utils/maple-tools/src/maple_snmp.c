// SPDX-License-Identifier: GPL-2.0
/*
 * maple_snmp — NET-SNMP `pass_persist` agent exposing per-ONU management data
 * over SNMP. Dependency-free (links maple_lib genetlink client).
 *
 * snmpd.conf:
 *   pass_persist .1.3.6.1.4.1.99999.1 /usr/sbin/maple_snmp
 *
 * MIB subtree (private enterprise .99999.1, <pon> 1..8, <onu> 1..64):
 *   .1.<pon>.<onu>  onuState            INTEGER
 *   .2.<pon>.<onu>  onuSerialNumber     OCTETSTR (8 bytes)
 *   .3.<pon>.<onu>  onuDistanceMeters   INTEGER (ranging_time)
 *   .4.<pon>.<onu>  onuDeactReason      INTEGER
 *   .5.<pon>.<onu>  onuRxBytes          COUNTER64  (downstream)
 *   .6.<pon>.<onu>  onuTxBytes          COUNTER64  (upstream)
 *
 * Protocol (NET-SNMP pass_persist): one request per stdin line — "PING" →
 * "PONG"; "get"/"getnext"/"set" → next line is the OID; respond with
 * "<oid>\n<type>\n<value>\n" or "NONE\n".
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "maple_lib.h"

#define BASE_OID  ".1.3.6.1.4.1.99999.1"
#define PON_MIN 1
#define PON_MAX 8
#define ONU_MIN 1
#define ONU_MAX 64
#define FLD_MIN 1
#define FLD_MAX 6

/* parse ".1.3.6.1.4.1.99999.1.<field>.<pon>.<onu>[...]" -> field,pon,onu.
 * returns 0 on match (suffix found after BASE_OID). */
static int parse_oid(const char *oid, int *field, int *pon, int *onu)
{
	const char *base = BASE_OID;
	size_t blen = strlen(base);
	if (strncmp(oid, base, blen)) return -1;
	const char *p = oid + blen;
	if (*p == '\0') return -1;		/* exact base — no leaf */
	if (*p++ != '.') return -1;
	*field = strtol(p, (char **)&p, 10); if (*p++ != '.') return -1;
	*pon   = strtol(p, (char **)&p, 10); if (*p++ != '.') return -1;
	*onu   = strtol(p, (char **)&p, 10);
	return 0;
}

/* produce the value for (field,pon,onu); returns 0 ok (out filled), -1 err */
static int get_value(int field, int pon, int onu, char *type, char *val, size_t vlen)
{
	struct maple_onu_cfg c;
	struct maple_onu_stat s;

	if (field < FLD_MIN || field > FLD_MAX) return -1;
	if (field <= 4) {
		if (maple_gnl_onu_get_cfg(pon, onu, &c)) return -1;
		switch (field) {
		case 1: strcpy(type, "integer"); snprintf(val, vlen, "%u", c.onu_state); return 0;
		case 2: strcpy(type, "octetstr");
			for (int i = 0; i < 8; i++) snprintf(val + i*2, vlen - i*2, "%02x", c.serial_number[i]);
			return 0;
		case 3: strcpy(type, "integer"); snprintf(val, vlen, "%u", c.ranging_time); return 0;
		case 4: strcpy(type, "integer"); snprintf(val, vlen, "%u", c.deactivation_reason); return 0;
		}
	} else {
		if (maple_gnl_onu_get_stat(pon, onu, &s)) return -1;
		if (field == 5) { strcpy(type, "counter64"); snprintf(val, vlen, "%llu", (unsigned long long)s.rx_bytes); return 0; }
		if (field == 6) { strcpy(type, "counter64"); snprintf(val, vlen, "%llu", (unsigned long long)s.tx_bytes); return 0; }
	}
	return -1;
}

/* GETNEXT: walk field/pon/onu to the first existing leaf after (field,pon,onu). */
static int getnext(int *field, int *pon, int *onu, char *type, char *val, size_t vlen)
{
	int f, p, o;

	for (f = *field; f <= FLD_MAX; f++)
		for (p = (f == *field ? *pon : PON_MIN); p <= PON_MAX; p++)
			for (o = (f == *field && p == *pon ? *onu + 1 : ONU_MIN); o <= ONU_MAX; o++) {
				if (!get_value(f, p, o, type, val, vlen)) {
					*field = f; *pon = p; *onu = o;
					return 0;
				}
			}
	return -1;
}

static void reply_oid(const char *oid, const char *type, const char *val) {
	printf("%s\n%s\n%s\n", oid, type, val);
	fflush(stdout);
}
static void reply_none(void) { printf("NONE\n"); fflush(stdout); }
static void reply_notwritable(void) { printf("not-writable\n"); fflush(stdout); }

int main(void)
{
	char line[512], oid[256], type[16], val[64];
	int field, pon, onu;

	if (maple_gnl_open()) {
		/* no driver — still answer PING so snmpd keeps us alive */
	}

	while (fgets(line, sizeof(line), stdin)) {
		line[strcspn(line, "\r\n")] = 0;
		if (!strcmp(line, "PING")) { printf("PONG\n"); fflush(stdout); continue; }
		if (!strcmp(line, "get") || !strcmp(line, "getnext") || !strcmp(line, "set")) {
			const char *op = line;
			if (!fgets(oid, sizeof(oid), stdin)) break;
			oid[strcspn(oid, "\r\n")] = 0;
			if (!strcmp(op, "set")) { reply_notwritable(); continue; }
			if (parse_oid(oid, &field, &pon, &onu)) { reply_none(); continue; }
			if (!strcmp(op, "get")) {
				if (!get_value(field, pon, onu, type, val, sizeof(val)))
					reply_oid(oid, type, val);
				else
					reply_none();
			} else { /* getnext */
				if (!getnext(&field, &pon, &onu, type, val, sizeof(val)))
					snprintf(oid, sizeof(oid), "%s.%d.%d.%d", BASE_OID, field, pon, onu),
					reply_oid(oid, type, val);
				else
					reply_none();
			}
			continue;
		}
		/* bare OID (older snmpd): treat as get */
		if (parse_oid(line, &field, &pon, &onu) == 0) {
			if (!get_value(field, pon, onu, type, val, sizeof(val)))
				reply_oid(line, type, val);
			else
				reply_none();
		}
	}
	return 0;
}
