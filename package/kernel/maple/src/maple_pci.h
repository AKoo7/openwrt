/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Per-device state shared by the PCI glue, firmware loader and ring transport.
 */
#ifndef MAPLE_PCI_H
#define MAPLE_PCI_H

#include <linux/io.h>
#include <linux/pci.h>
#include <linux/types.h>

#include "maple_ring.h"
#include "maple_fw.h"
#include "maple_bal.h"

#define DRV_NAME "maple"

struct maple_dev {
	struct pci_dev	*pdev;
	void __iomem	*bar0;		/* SoC control / registers        */
	void __iomem	*bar2;		/* secondary memory window        */
	void __iomem	*bar4;		/* DDR window (rings + comm area) */
	size_t		bar2_sz;
	int		irq;
	u8		index;

	struct maple_ring	ring;
	struct maple_fw		fw;
	struct maple_bal		bal;

	bool			running;
};

#endif /* MAPLE_PCI_H */
