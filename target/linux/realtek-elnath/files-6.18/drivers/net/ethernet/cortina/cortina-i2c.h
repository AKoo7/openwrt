/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Minimal MMIO I2C master for the Cortina-Access ca77xx "per_i2c" controller
 * (Realtek RTL9607F "Elnath").  See cortina-i2c.c.
 */

#ifndef _CORTINA_I2C_H_
#define _CORTINA_I2C_H_

#include <linux/types.h>

struct device;

int cg_i2c_init(struct device *dev);
int cg_i2c_write_byte(u8 addr, u8 reg, u8 val);
int cg_i2c_read_byte(u8 addr, u8 reg, u8 *val);

#endif /* _CORTINA_I2C_H_ */
