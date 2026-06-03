// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * rtk_factory - clean-room per-board identity reader for the RTL9602C ONU.
 *
 * The vendor stores this unit's identity as Realtek config-XML files inside the
 * JFFS2 "config" flash partition. This tool parses the JFFS2 image directly from
 * the raw (read-only) MTD - no kernel JFFS2 and no writable flash needed -
 * reconstructs the CURRENT version of the relevant file, and prints one
 * <Value Name="KEY" Value="..."/> value. One identical firmware image can thus
 * self-provision an entire fleet; the factory partition is only ever read.
 *
 *   HW (lastgood_hs.xml): ELAN_MAC_ADDR WLAN_MAC_ADDR GPON_SN OUI
 *                         PON_VENDOR_ID GPON_ONU_MODEL MAC_KEY
 *   SW (lastgood.xml):    LOID LOID_PASSWD GPON_PLOAM_PASSWD OMCI_OLT_MODE
 * The *_bak and *_mp_hs2 (manufacturing-default) variants are never read.
 *
 * JFFS2 facts (big-endian on this MIPS target): node header = {u16 magic 0x1985,
 * u16 nodetype, u32 totlen, u32 hdr_crc}. ACCURATE bit 0x2000 set = live, clear =
 * obsolete. DIRENT (nodetype&0xff==1): ino@20, ver@16, nsize@28, name@40.
 * INODE (nodetype&0xff==2): ino@12, ver@16, off@44, csize@48, dsize@52,
 * compr@56 (0=none, 6=zlib), data@68.
 */
#define _GNU_SOURCE 1
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <zlib.h>

#define PART_MAX (512 * 1024)
#define FILE_MAX (256 * 1024)

static uint8_t  partbuf[PART_MAX];
static uint8_t  filebuf[FILE_MAX];

static uint16_t be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static uint32_t be32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
	       ((uint32_t)p[2] << 8) | p[3];
}

struct vnode { uint32_t ver, pos; };
static struct vnode vn[16384];

static int cmp_ver(const void *a, const void *b)
{
	uint32_t x = ((const struct vnode *)a)->ver, y = ((const struct vnode *)b)->ver;
	return (x > y) - (x < y);
}

/* Reconstruct current contents of `name` into filebuf; return length or -1. */
static long jffs2_read_file(const uint8_t *d, uint32_t n, const char *name)
{
	size_t nl = strlen(name);
	uint32_t ino = 0, best = 0, i;
	int nv = 0;

	for (i = 0; i + 12 <= n; ) {                       /* pass 1: name -> ino */
		uint32_t tot;
		uint16_t nt;
		if (be16(d + i) != 0x1985) { i += 4; continue; }
		nt = be16(d + i + 2);
		tot = be32(d + i + 4);
		if (tot < 12 || (uint64_t)i + tot > n) { i += 4; continue; }
		if ((nt & 0x2000) && (nt & 0xff) == 1) {       /* live DIRENT */
			uint8_t nsize = d[i + 28];
			uint32_t ver = be32(d + i + 16);
			if (nsize == nl && i + 40 + nsize <= n &&
			    !memcmp(d + i + 40, name, nsize) && ver >= best) {
				best = ver;
				ino = be32(d + i + 20);
			}
		}
		i += (tot + 3) & ~3u;
	}
	if (!ino)
		return -1;

	for (i = 0; i + 12 <= n; ) {                       /* pass 2: collect INODE nodes */
		uint32_t tot;
		uint16_t nt;
		if (be16(d + i) != 0x1985) { i += 4; continue; }
		nt = be16(d + i + 2);
		tot = be32(d + i + 4);
		if (tot < 12 || (uint64_t)i + tot > n) { i += 4; continue; }
		if ((nt & 0x2000) && (nt & 0xff) == 2 && be32(d + i + 12) == ino &&
		    nv < (int)(sizeof vn / sizeof vn[0])) {
			vn[nv].ver = be32(d + i + 16);
			vn[nv].pos = i;
			nv++;
		}
		i += (tot + 3) & ~3u;
	}
	qsort(vn, nv, sizeof vn[0], cmp_ver);

	long flen = 0;
	int k;
	for (k = 0; k < nv; k++) {
		const uint8_t *p = d + vn[k].pos;
		uint32_t off = be32(p + 44), csize = be32(p + 48), dsize = be32(p + 52);
		uint8_t compr = p[56];
		const uint8_t *src = p + 68;
		if ((uint64_t)off + dsize > FILE_MAX)
			continue;
		if (compr == 0) {                              /* none */
			if (dsize > csize) dsize = csize;
			memcpy(filebuf + off, src, dsize);
		} else if (compr == 6) {                       /* zlib */
			uLongf dl = dsize;
			if (uncompress(filebuf + off, &dl, src, csize) != Z_OK)
				continue;
			dsize = dl;
		} else {
			continue;                              /* rtime/lzo not used by these files */
		}
		if ((long)(off + dsize) > flen)
			flen = off + dsize;
	}
	return flen;
}

/* Extract Value of <... Name="key" ... Value="val" ...> from a buffer. */
static const char *xml_value(const uint8_t *buf, long len, const char *key)
{
	static char out[256];
	char pat[96];
	int pl = snprintf(pat, sizeof pat, "Name=\"%s\"", key);
	long i;
	for (i = 0; i + pl <= len; i++) {
		if (memcmp(buf + i, pat, pl))
			continue;
		const uint8_t *v = (const uint8_t *)memmem(buf + i, len - i, "Value=\"", 7);
		if (!v)
			return NULL;
		v += 7;
		const uint8_t *e = (const uint8_t *)memchr(v, '"', buf + len - v);
		if (!e || e - v >= (long)sizeof out)
			return NULL;
		memcpy(out, v, e - v);
		out[e - v] = 0;
		return out;
	}
	return NULL;
}

/* verb -> (filename, xml key). HW verbs read lastgood_hs.xml, SW read lastgood.xml. */
struct map { const char *verb, *file, *key; int is_mac; };
static const struct map MAP[] = {
	{ "mac",          "lastgood_hs.xml", "ELAN_MAC_ADDR",     1 },
	{ "wlan_mac",     "lastgood_hs.xml", "WLAN_MAC_ADDR",     1 },
	{ "sn",           "lastgood_hs.xml", "GPON_SN",           0 },
	{ "oui",          "lastgood_hs.xml", "OUI",               0 },
	{ "vendor_id",    "lastgood_hs.xml", "PON_VENDOR_ID",     0 },
	{ "model",        "lastgood_hs.xml", "GPON_ONU_MODEL",    0 },
	{ "mac_key",      "lastgood_hs.xml", "MAC_KEY",           0 },
	{ "loid",         "lastgood.xml",    "LOID",              0 },
	{ "loid_passwd",  "lastgood.xml",    "LOID_PASSWD",       0 },
	{ "ploam_passwd", "lastgood.xml",    "GPON_PLOAM_PASSWD", 0 },
	{ "olt_mode",     "lastgood.xml",    "OMCI_OLT_MODE",     0 },
	{ NULL, NULL, NULL, 0 }
};

static int mtd_path_by_name(const char *name, char *out, size_t n);

/*
 * optical_cal: parse the rtl8290b Europa transceiver calibration blob
 * (rtl8290b.data in the config JFFS2 partition, 4096 bytes factory-written).
 *
 * Calibration byte offsets (big-endian, from vendor SDK facts):
 *   1350 int32  rx_a     - RX polynomial coefficient A
 *   1354 int32  rx_b     - RX polynomial coefficient B
 *   1358 int32  rx_c     - RX polynomial coefficient C (intercept ~= launch dBm*1000)
 *   1362 uint32 rssi_v0  - RX RSSI reference ADC count at factory calibration
 *   1366 uint32 mpd0     - TX MPD reference photo-diode count
 *   1372 int32  tx_a     - TX polynomial coefficient A
 *   1376 int32  tx_b     - TX polynomial coefficient B
 *   1380 int32  tx_c     - TX polynomial coefficient C (intercept ~= RX sensitivity dBm*1000)
 *   1384 int8   temp_off - Temperature offset (°C)
 *   1386 uint8  rx_th    - RX LOS assert threshold (raw)
 *   1387 uint8  rx_deth  - RX LOS de-assert threshold (raw)
 *
 * Prints: key=value pairs, one per line.
 */
static int do_optical_cal(const char *part)
{
	uint8_t blob[4096];
	char devpath[64];
	int fd;
	ssize_t got = 0, r;
	long flen;

	if (part[0] == '/')
		snprintf(devpath, sizeof devpath, "%s", part);
	else if (mtd_path_by_name(part, devpath, sizeof devpath) < 0) {
		fprintf(stderr, "rtk_factory: partition '%s' not in /proc/mtd\n", part);
		return 1;
	}

	fd = open(devpath, O_RDONLY);
	if (fd < 0) { perror(devpath); return 1; }
	while (got < (ssize_t)sizeof partbuf &&
	       (r = read(fd, partbuf + got, sizeof partbuf - got)) > 0)
		got += r;
	close(fd);
	flen = jffs2_read_file(partbuf, (uint32_t)got, "rtl8290b.data");

	if (flen < 1388) {
		fprintf(stderr, "rtk_factory: rtl8290b.data absent or too short (%ld)\n", flen);
		return 1;
	}
	memcpy(blob, filebuf, sizeof blob < (size_t)flen ? sizeof blob : (size_t)flen);

	int32_t  rx_a    = (int32_t)be32(blob + 1350);
	int32_t  rx_b    = (int32_t)be32(blob + 1354);
	int32_t  rx_c    = (int32_t)be32(blob + 1358);
	uint32_t rssi_v0 = be32(blob + 1362);
	uint32_t mpd0    = be32(blob + 1366);
	int32_t  tx_a    = (int32_t)be32(blob + 1372);
	int32_t  tx_b    = (int32_t)be32(blob + 1376);
	int32_t  tx_c    = (int32_t)be32(blob + 1380);
	int8_t   t_off   = (int8_t)blob[1384];
	uint8_t  rx_th   = blob[1386];
	uint8_t  rx_deth = blob[1387];

	printf("rx_a=%d\n",    rx_a);
	printf("rx_b=%d\n",    rx_b);
	printf("rx_c=%d\n",    rx_c);
	printf("rssi_v0=%u\n", rssi_v0);
	printf("mpd0=%u\n",    mpd0);
	printf("tx_a=%d\n",    tx_a);
	printf("tx_b=%d\n",    tx_b);
	printf("tx_c=%d\n",    tx_c);
	printf("temp_off=%d\n",(int)t_off);
	printf("rx_los_th=%u\n",   rx_th);
	printf("rx_delos_th=%u\n", rx_deth);
	return 0;
}

static int mtd_path_by_name(const char *name, char *out, size_t n)
{
	FILE *f = fopen("/proc/mtd", "r");
	char line[256], nm[128];
	int idx, found = -1;
	if (!f)
		return -1;
	if (fgets(line, sizeof line, f)) {                 /* header */
		while (fgets(line, sizeof line, f))
			if (sscanf(line, "mtd%d: %*x %*x \"%127[^\"]\"", &idx, nm) == 2 &&
			    !strcmp(nm, name)) {
				snprintf(out, n, "/dev/mtd%d", idx);
				found = 0;
				break;
			}
	}
	fclose(f);
	return found;
}

int main(int argc, char **argv)
{
	const char *part = "config", *verb;
	const struct map *m;
	char path[64];
	int a = 1, fd;
	ssize_t got = 0, r;
	long flen;
	const char *val;

	if (argc >= 3 && !strcmp(argv[1], "-p")) { part = argv[2]; a = 3; }
	if (a >= argc) {
		fprintf(stderr, "usage: rtk_factory [-p part] <mac|wlan_mac|sn|oui|"
			"vendor_id|model|mac_key|loid|loid_passwd|ploam_passwd|olt_mode|"
			"optical_cal>\n");
		return 2;
	}
	verb = argv[a];

	if (!strcmp(verb, "optical_cal"))
		return do_optical_cal(part);

	for (m = MAP; m->verb && strcmp(m->verb, verb); m++)
		;
	if (!m->verb) {
		fprintf(stderr, "rtk_factory: unknown field '%s'\n", verb);
		return 2;
	}

	if (part[0] == '/')
		snprintf(path, sizeof path, "%s", part);
	else if (mtd_path_by_name(part, path, sizeof path) < 0) {
		fprintf(stderr, "rtk_factory: partition '%s' not in /proc/mtd\n", part);
		return 1;
	}

	fd = open(path, O_RDONLY);
	if (fd < 0) { perror(path); return 1; }
	while (got < (ssize_t)sizeof partbuf &&
	       (r = read(fd, partbuf + got, sizeof partbuf - got)) > 0)
		got += r;
	close(fd);

	flen = jffs2_read_file(partbuf, (uint32_t)got, m->file);
	if (flen < 0) {
		fprintf(stderr, "rtk_factory: %s not found in %s\n", m->file, path);
		return 1;
	}
	val = xml_value(filebuf, flen, m->key);
	if (!val) {
		fprintf(stderr, "rtk_factory: %s not set in %s\n", m->key, m->file);
		return 1;
	}

	if (m->is_mac && strlen(val) == 12) {              /* 001122334455 -> 00:11:22:... */
		int j;
		for (j = 0; j < 12; j += 2)
			printf("%c%c%s", val[j], val[j + 1], j < 10 ? ":" : "\n");
	} else {
		printf("%s\n", val);
	}
	return 0;
}
