/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Semtech GN25L95 BOSA boot initialization (RTL9607F "Elnath").
 * See cortina-gpon-bosa.c.
 */

#ifndef _CORTINA_GPON_BOSA_H_
#define _CORTINA_GPON_BOSA_H_

struct device;
struct seq_file;
struct cg_bosa_ddm;

int cg_bosa_init(struct device *dev);
int cg_bosa_dump(struct device *dev);
void cg_bosa_proc_show(struct device *dev, struct seq_file *m);

/* Sample the SFF-8472 A2h optical diagnostics.  PROCESS CONTEXT ONLY (the i2c
 * master sleeps).  Returns an enum cg_ddm_status; CG_DDM_OK == 0 means the
 * sample is usable, anything else means NO value may be reported. */
int cg_bosa_ddm_read(struct device *dev, struct cg_bosa_ddm *d);

#endif /* _CORTINA_GPON_BOSA_H_ */
