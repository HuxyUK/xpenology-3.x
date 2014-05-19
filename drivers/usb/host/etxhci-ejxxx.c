/*
 * xHCI host controller driver PCI Bus Glue.
 *
 * Copyright (C) 2008 Intel Corp.
 *
 * Author: Sarah Sharp
 * Some code borrowed from the Linux EHCI driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/pci.h>

#include "etxhci.h"
#include "etxhci-ej168v0.0660.c"
#include "etxhci-ej188v0.01.00.900.c"

void xhci_init_ejxxx(struct xhci_hcd *xhci)
{
	struct usb_hcd *hcd = xhci_to_hcd(xhci);
	u32 reg32 = 0;

	switch (xhci->hcc_params1 & 0xff) {
	case 0x30:
		xhci_init_ej168_v00660(xhci);

		reg32 = xhci_readl(xhci, hcd->regs + 0x40c0);
		reg32 = (reg32 & 0xffff00ff) | 0x0100;
		xhci_writel(xhci, reg32, hcd->regs + 0x40c0);
		reg32 = xhci_readl(xhci, hcd->regs + 0x40d4);
		reg32 = (reg32 & 0xfffffffe) | 0x01;
		xhci_writel(xhci, reg32, hcd->regs + 0x40d4);
		break;
	case 0x40:
		xhci_init_ej188_v00100900(xhci);

		reg32 = xhci_readl(xhci, hcd->regs + 0x4294);
		reg32 = (reg32 & 0xfffffffe) | 0x01;
		xhci_writel(xhci, reg32, hcd->regs + 0x4294);
		reg32 = xhci_readl(xhci, hcd->regs + 0x42d4);
		reg32 = (reg32 & 0xfffffffe) | 0x01;
		xhci_writel(xhci, reg32, hcd->regs + 0x42d4);
		break;
	default:
		break;
	}
}

