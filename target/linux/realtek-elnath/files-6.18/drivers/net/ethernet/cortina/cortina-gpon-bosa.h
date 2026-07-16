/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Semtech GN25L95 BOSA boot initialization (RTL9607F "Elnath").
 * See cortina-gpon-bosa.c.
 */

#ifndef _CORTINA_GPON_BOSA_H_
#define _CORTINA_GPON_BOSA_H_

struct device;
struct seq_file;

int cg_bosa_init(struct device *dev);
int cg_bosa_dump(struct device *dev);
void cg_bosa_proc_show(struct device *dev, struct seq_file *m);

#endif /* _CORTINA_GPON_BOSA_H_ */
