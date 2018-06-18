/*
 * Copyright(c) 2018 Intel Corporation.
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * BSD LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <linux/export.h>
#include <linux/thread_info.h>
#include <linux/ctype.h>
#include <linux/errno.h>
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/bug.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/pci.h>
#include "../hfi1/hfi.h"
#include "compat.h"

DEFINE_SRCU(debugfs_srcu);

const char *get_unit_name(int unit)
{
	static char iname[16];

	snprintf(iname, sizeof(iname), DRIVER_NAME "_%u", unit);
	return iname;
}
EXPORT_SYMBOL(get_unit_name);

void hfi1_vnic_setup(struct hfi1_devdata *dd)
{
}
EXPORT_SYMBOL(hfi1_vnic_setup);

int hfi1_vnic_send_dma(struct hfi1_devdata *dd, u8 q_idx,
		       struct hfi1_vnic_vport_info *vinfo,
		       struct sk_buff *skb, u64 pbc, u8 plen)
{
	return -ECOMM;
}
EXPORT_SYMBOL(hfi1_vnic_send_dma);

void hfi1_vnic_bypass_rcv(struct hfi1_packet *packet)
{
}
EXPORT_SYMBOL(hfi1_vnic_bypass_rcv);

void hfi1_vnic_cleanup(struct hfi1_devdata *dd)
{
}
EXPORT_SYMBOL(hfi1_vnic_cleanup);

/*
 * pci_request_irq - allocate an interrupt line for a PCI device
 * @dev:       PCI device to operate on
 * @nr:                device-relative interrupt vector index (0-based).
 * @handler:   Function to be called when the IRQ occurs.
 *             Primary handler for threaded interrupts.
 *             If NULL and thread_fn != NULL the default primary handler is
 *             installed.
 * @thread_fn: Function called from the IRQ handler thread
 *             If NULL, no IRQ thread is created
 * @dev_id:    Cookie passed back to the handler function
 * @fmt:       Printf-like format string naming the handler
 *
 * This call allocates interrupt resources and enables the interrupt line and
 * IRQ handling. From the point this call is made @handler and @thread_fn may
 * be invoked.  All interrupts requested using this function might be shared.
 *
 * @dev_id must not be NULL and must be globally unique.
 */
int pci_request_irq(struct pci_dev *dev, unsigned int nr, irq_handler_t handler,
		    irq_handler_t thread_fn, void *dev_id, const char *fmt, ...)
{
	va_list ap;
	int ret;
	char *devname;

	va_start(ap, fmt);
	devname = kvasprintf(GFP_KERNEL, fmt, ap);
	va_end(ap);

	ret = request_threaded_irq(pci_irq_vector(dev, nr), handler, thread_fn,
				   IRQF_SHARED, devname, dev_id);
	if (ret)
		kfree(devname);
	return ret;
}
EXPORT_SYMBOL(pci_request_irq);

/*
 * pci_free_irq - free an interrupt allocated with pci_request_irq
 * @dev:       PCI device to operate on
 * @nr:                device-relative interrupt vector index (0-based).
 * @dev_id:    Device identity to free
 *
 * Remove an interrupt handler. The handler is removed and if the interrupt
 * line is no longer in use by any driver it is disabled.  The caller must
 * ensure the interrupt is disabled on the device before calling this function.
 * The function does not return until any executing interrupts for this IRQ
 * have completed.
 *
 * This function must not be called from interrupt context.
 */
void pci_free_irq(struct pci_dev *dev, unsigned int nr, void *dev_id)
{
	free_irq(pci_irq_vector(dev, nr), dev_id);
}
EXPORT_SYMBOL(pci_free_irq);

/**
 * cdev_set_parent() - set the parent kobject for a char device
 * @p: the cdev structure
 * @kobj: the kobject to take a reference to
 *
 * cdev_set_parent() sets a parent kobject which will be referenced
 * appropriately so the parent is not freed before the cdev. This
 * should be called before cdev_add.
 */
void cdev_set_parent(struct cdev *p, struct kobject *kobj)
{
	WARN_ON(!kobj->state_initialized);
	p->kobj.parent = kobj;
}
EXPORT_SYMBOL(cdev_set_parent);

/*
 * We should only need to wait 100ms after FLR, but some devices take longer.
 * Wait for up to 1000ms for config space to return something other than -1.
 * Intel IGD requires this when an LCD panel is attached.  We read the 2nd
 * dword because VFs don't implement the 1st dword.
 */
static void pci_flr_wait(struct pci_dev *dev)
{
	int i = 0;
	u32 id;

	do {
		msleep(100);
		pci_read_config_dword(dev, PCI_COMMAND, &id);
	} while (i++ < 10 && id == ~0);

	if (id == ~0)
		dev_warn(&dev->dev, "Failed to return from FLR\n");
	else if (i > 1)
		dev_info(&dev->dev, "Required additional %dms to return from FLR\n",
			 (i - 1) * 100);
}

/**
 * pcie_flr - initiate a PCIe function level reset
 *  @dev:        device to reset
 *
 * Initiate a function level reset on @dev.  The caller should ensure the
 * device supports FLR before calling this function, e.g. by using the
 * pcie_has_flr() helper.
 */
void pcie_flr(struct pci_dev *dev)
{
	if (!pci_wait_for_pending_transaction(dev))
		dev_err(&dev->dev,
			"timed out waiting for pending transaction; performing function level reset anyway\n");

		pcie_capability_set_word(dev,
					 PCI_EXP_DEVCTL,
					 PCI_EXP_DEVCTL_BCR_FLR);
	pci_flr_wait(dev);
}
EXPORT_SYMBOL_GPL(pcie_flr);

