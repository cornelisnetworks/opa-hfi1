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
#if !defined(RH76_COMPAT_H)
#define RH76_COMPAT_H

#define CREATE_AH_HAS_UDATA
#define HAVE_ALLOC_RDMA_NETDEV
#define NEED_MM_HELPER_FUNCTIONS
#define HAVE_NOSPEC_H

#include "compat_common.h"

#define __GFP_RECLAIM	(__GFP_WAIT)

#define IB_QP_CREATE_USE_GFP_NOIO (1 << 7)


#define IB_FW_VERSION_NAME_MAX			  ETHTOOL_FWVERS_LEN
#define OPA_CLASS_PORT_INFO_PR_SUPPORT BIT(26)

#define NET_NAME_UNKNOWN 0

#define rdma_create_ah(a, b, c) rdma_create_ah(a, b)
#define rdma_destroy_ah(a, b) rdma_destroy_ah(a)

#define ib_register_device(a, b, c)  ib_register_device((a), (c))
#define rdma_set_device_sysfs_group(a, b)
#define alloc_netdev_mqs(size, name, name_assign_type, setup, sdma, ctxts) \
	alloc_netdev_mqs((size), (name), (setup), (sdma), (ctxts))
#undef access_ok
#define access_ok(addr, size) \
	(likely(__range_not_ok(addr, size, user_addr_max()) == 0))
#define _ib_alloc_device ib_alloc_device

struct hfi1_msix_entry;
struct hfi1_devdata;

void pcie_flr(struct pci_dev *dev);

int bitmap_print_to_pagebuf(bool list, char *buf,
			    const unsigned long *maskp, int nmaskbits);
int debugfs_use_file_start(struct dentry *dentry, int *srcu_idx)
__acquires(&debugfs_srcu);

void debugfs_use_file_finish(int srcu_idx) __releases(&debugfs_srcu);
struct ib_umem *ib_umem_get_hfi(struct ib_ucontext *context, unsigned long addr,
				size_t size, int access, int dmasync);

static inline long compat_get_user_pages(unsigned long start,
					 unsigned long nr_pages,
					 unsigned int gup_flags,
					 struct page **pages,
					 struct vm_area_struct **vmas)
{
	return get_user_pages(current, current->mm, start,
			      nr_pages, 1, 1, pages, vmas);
}

#define get_user_pages(start, nr_pages, gup_flags, pages, vmas) \
	compat_get_user_pages(start, nr_pages, gup_flags, pages, vmas)

static inline int simple_positive(struct dentry *dentry)
{
	return !d_unhashed(dentry) && dentry->d_inode;
}

static inline void hfi1_enable_intx(struct pci_dev *pdev)
{
	/* first, turn on INTx */
	pci_intx(pdev, 1);
	/* then turn off MSI-X */
	pci_disable_msix(pdev);
}

/* Helpers to hide struct msi_desc implementation details */
#define msi_desc_to_dev(desc)           ((desc)->dev)
#define dev_to_msi_list(dev)            (&(dev)->msi_list)
#define first_msi_entry(dev)            \
	list_first_entry(dev_to_msi_list((dev)), struct msi_desc, list)
#define for_each_msi_entry(desc, dev)   \
	list_for_each_entry((desc), dev_to_msi_list((dev)), list)

#endif //RH76_COMPAT
