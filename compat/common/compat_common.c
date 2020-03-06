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
#if !defined (IFS_SLES15) && !defined (IFS_SLES15SP1) && !defined (IFS_SLES12SP4)
#include "compat.h"
#endif

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
#if !defined(IFS_SLES15) && !defined(IFS_SLES15SP1) && !defined(IFS_SLES12SP4) && !defined(IFS_SLES12SP5) && !defined(IFS_RH76) && !defined(IFS_RH77) && !defined(IFS_RH80) && !defined(IFS_RH81)
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
#endif
#if !defined(IFS_RH75) && !defined(IFS_RH76) && !defined(IFS_RH77) && !defined(IFS_RH80) && !defined(IFS_RH81) && !defined(IFS_SLES15) && !defined(IFS_SLES15SP1) && !defined(IFS_SLES12SP4) && !defined(IFS_SLES12SP5)
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

#endif

#ifndef HAVE_IB_SET_DEVICE_OPS
void ib_set_device_ops(struct ib_device *dev, const struct ib_device_ops *ops)
{
#ifdef IB_DEV_HAS_EMBEDDED_OPS
	struct ib_device_ops *dev_ops = &dev->ops;
#else
	struct ib_device *dev_ops = dev;
#endif
#define SET_DEVICE_OP(ptr, name)                                               \
	do {                                                                   \
		if (ops->name)                                                 \
			if (!((ptr)->name))				       \
				(ptr)->name = ops->name;                       \
	} while (0)

#define SET_OBJ_SIZE(ptr, name) SET_DEVICE_OP(ptr, size_##name)

	SET_DEVICE_OP(dev_ops, add_gid);
#ifdef HAVE_ADVICE_MR
	SET_DEVICE_OP(dev_ops, advise_mr);
#endif
#ifdef HAVE_ALLOC_DM
	SET_DEVICE_OP(dev_ops, alloc_dm);
#endif
	SET_DEVICE_OP(dev_ops, alloc_fmr);
	SET_DEVICE_OP(dev_ops, alloc_hw_stats);
	SET_DEVICE_OP(dev_ops, alloc_mr);
	SET_DEVICE_OP(dev_ops, alloc_mw);
	SET_DEVICE_OP(dev_ops, alloc_pd);
#ifdef HAVE_ALLOC_RDMA_NETDEV
	SET_DEVICE_OP(dev_ops, alloc_rdma_netdev);
#endif
	SET_DEVICE_OP(dev_ops, alloc_ucontext);
#ifdef HAVE_ALLOC_XRCD
	SET_DEVICE_OP(dev_ops, alloc_xrcd);
#endif
	SET_DEVICE_OP(dev_ops, attach_mcast);
	SET_DEVICE_OP(dev_ops, check_mr_status);
	SET_DEVICE_OP(dev_ops, create_ah);
#ifdef HAVE_CREATE_COUNTERS
	SET_DEVICE_OP(dev_ops, create_counters);
#endif
	SET_DEVICE_OP(dev_ops, create_cq);
	SET_DEVICE_OP(dev_ops, create_flow);
#ifdef HAVE_CREATE_FLOW_ACTION_ESP
	SET_DEVICE_OP(dev_ops, create_flow_action_esp);
#endif
	SET_DEVICE_OP(dev_ops, create_qp);
	SET_DEVICE_OP(dev_ops, create_rwq_ind_table);
	SET_DEVICE_OP(dev_ops, create_srq);
	SET_DEVICE_OP(dev_ops, create_wq);
#ifdef HAVE_ALLOC_DM
	SET_DEVICE_OP(dev_ops, dealloc_dm);
#endif
#ifdef HAVE_DEALLOC_DRIVER
	SET_DEVICE_OP(dev_ops, dealloc_driver);
#endif
	SET_DEVICE_OP(dev_ops, dealloc_fmr);
	SET_DEVICE_OP(dev_ops, dealloc_mw);
	SET_DEVICE_OP(dev_ops, dealloc_pd);
	SET_DEVICE_OP(dev_ops, dealloc_ucontext);
#ifdef HAVE_ALLOC_XRCD
	SET_DEVICE_OP(dev_ops, dealloc_xrcd);
#endif
	SET_DEVICE_OP(dev_ops, del_gid);
	SET_DEVICE_OP(dev_ops, dereg_mr);
	SET_DEVICE_OP(dev_ops, destroy_ah);
#ifdef HAVE_CREATE_COUNTERS
	SET_DEVICE_OP(dev_ops, destroy_counters);
#endif
	SET_DEVICE_OP(dev_ops, destroy_cq);
	SET_DEVICE_OP(dev_ops, destroy_flow);
#ifdef HAVE_DESTROY_FLOW_ACTION
	SET_DEVICE_OP(dev_ops, destroy_flow_action);
#endif
	SET_DEVICE_OP(dev_ops, destroy_qp);
	SET_DEVICE_OP(dev_ops, destroy_rwq_ind_table);
	SET_DEVICE_OP(dev_ops, destroy_srq);
	SET_DEVICE_OP(dev_ops, destroy_wq);
	SET_DEVICE_OP(dev_ops, detach_mcast);
	SET_DEVICE_OP(dev_ops, disassociate_ucontext);
	SET_DEVICE_OP(dev_ops, drain_rq);
	SET_DEVICE_OP(dev_ops, drain_sq);
#ifdef HAVE_ENABLE_DRIVER
	SET_DEVICE_OP(dev_ops, enable_driver);
#endif
#ifdef HAVE_FILL_RES_ENTRY
	SET_DEVICE_OP(dev_ops, fill_res_entry);
#endif
	SET_DEVICE_OP(dev_ops, get_dev_fw_str);
	SET_DEVICE_OP(dev_ops, get_dma_mr);
	SET_DEVICE_OP(dev_ops, get_hw_stats);
	SET_DEVICE_OP(dev_ops, get_link_layer);
	SET_DEVICE_OP(dev_ops, get_netdev);
	SET_DEVICE_OP(dev_ops, get_port_immutable);
#ifdef HAVE_GET_VECTOR_AFFINITY
	SET_DEVICE_OP(dev_ops, get_vector_affinity);
#endif
	SET_DEVICE_OP(dev_ops, get_vf_config);
	SET_DEVICE_OP(dev_ops, get_vf_stats);
#ifdef HAVE_INIT_PORT
	SET_DEVICE_OP(dev_ops, init_port);
#endif
#ifdef HAVE_IW_ACCEPT
	SET_DEVICE_OP(dev_ops, iw_accept);
	SET_DEVICE_OP(dev_ops, iw_add_ref);
	SET_DEVICE_OP(dev_ops, iw_connect);
	SET_DEVICE_OP(dev_ops, iw_create_listen);
	SET_DEVICE_OP(dev_ops, iw_destroy_listen);
	SET_DEVICE_OP(dev_ops, iw_get_qp);
	SET_DEVICE_OP(dev_ops, iw_reject);
	SET_DEVICE_OP(dev_ops, iw_rem_ref);
#endif
	SET_DEVICE_OP(dev_ops, map_mr_sg);
	SET_DEVICE_OP(dev_ops, map_phys_fmr);
	SET_DEVICE_OP(dev_ops, mmap);
	SET_DEVICE_OP(dev_ops, modify_ah);
	SET_DEVICE_OP(dev_ops, modify_cq);
	SET_DEVICE_OP(dev_ops, modify_device);
#ifdef HAVE_CREATE_FLOW_ACTION_ESP
	SET_DEVICE_OP(dev_ops, modify_flow_action_esp);
#endif
	SET_DEVICE_OP(dev_ops, modify_port);
	SET_DEVICE_OP(dev_ops, modify_qp);
	SET_DEVICE_OP(dev_ops, modify_srq);
	SET_DEVICE_OP(dev_ops, modify_wq);
	SET_DEVICE_OP(dev_ops, peek_cq);
	SET_DEVICE_OP(dev_ops, poll_cq);
	SET_DEVICE_OP(dev_ops, post_recv);
	SET_DEVICE_OP(dev_ops, post_send);
	SET_DEVICE_OP(dev_ops, post_srq_recv);
	SET_DEVICE_OP(dev_ops, process_mad);
	SET_DEVICE_OP(dev_ops, query_ah);
	SET_DEVICE_OP(dev_ops, query_device);
	SET_DEVICE_OP(dev_ops, query_gid);
	SET_DEVICE_OP(dev_ops, query_pkey);
	SET_DEVICE_OP(dev_ops, query_port);
	SET_DEVICE_OP(dev_ops, query_qp);
	SET_DEVICE_OP(dev_ops, query_srq);
#ifdef HAVE_RDMA_NETDEV_GET_PARAMS
	SET_DEVICE_OP(dev_ops, rdma_netdev_get_params);
#endif
#ifdef HAVE_CREATE_COUNTERS
	SET_DEVICE_OP(dev_ops, read_counters);
#endif
#ifdef HAVE_REG_DM_MR
	SET_DEVICE_OP(dev_ops, reg_dm_mr);
#endif
	SET_DEVICE_OP(dev_ops, reg_user_mr);
	SET_DEVICE_OP(dev_ops, req_ncomp_notif);
	SET_DEVICE_OP(dev_ops, req_notify_cq);
	SET_DEVICE_OP(dev_ops, rereg_user_mr);
	SET_DEVICE_OP(dev_ops, resize_cq);
	SET_DEVICE_OP(dev_ops, set_vf_guid);
	SET_DEVICE_OP(dev_ops, set_vf_link_state);
	SET_DEVICE_OP(dev_ops, unmap_fmr);

#if 0
	SET_OBJ_SIZE(dev_ops, ib_ah);
	SET_OBJ_SIZE(dev_ops, ib_pd);
	SET_OBJ_SIZE(dev_ops, ib_srq);
	SET_OBJ_SIZE(dev_ops, ib_ucontext);
#endif
}
EXPORT_SYMBOL(ib_set_device_ops);
#endif
