/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Copyright(c) 2020 Intel Corporation.
 */
#if !defined(RH78_COMPAT_H)
#define RH78_COMPAT_H

#define HAVE_IB_GID_ATTR
#define POST_HAS_CONST
#define CREATE_FLOW_HAS_UDATA
#define CREATE_AH_HAS_UDATA
#define HAVE_ALLOC_RDMA_NETDEV
#define HAVE_RDMA_NETDEV_GET_PARAMS
#define HAVE_NET_DEVICE_EXTENDED
#define HAVE_ARRAY_SIZE
#define HAVE_NOSPEC_H
#define VM_OPS_FAULT_HAVE_VMA
#define HAVE_MAX_SEND_SGE
#define HAVE_KMALLOC_ARRAY_NODE
#define HAVE_IBDEV_DRIVER_ID
#define HAVE_IB_GET_CACHED_SUBNET_PREFIX
#define HAVE_SECURITY_H
#define HAVE_AIO_WRITE
#define HAVE_DEVICE_RH
#define NEED_KTHREAD_HELPER_FUNCTIONS
#define NEED_CURRENT_TIME
#define HAVE_RDMA_SET_DEVICE_SYSFS_GROUP
#define NEED_POLL_T
#define HAVE_RDMA_COPY_AH_ATTR
#define NEED_FILE_FINISH
#define NEED_PCI_BRIDGE_SECONDARY_BUS_RESET
#define HAS_PORT_IMMUTABLE

#include "compat_common.h"

#define __GFP_RECLAIM	(__GFP_WAIT)

#define IB_FW_VERSION_NAME_MAX			  ETHTOOL_FWVERS_LEN
#define OPA_CLASS_PORT_INFO_PR_SUPPORT BIT(26)

#define NET_NAME_UNKNOWN 0

#define ib_register_device(a, b) \
	ib_register_device((a), (b), (rdi->driver_f.port_callback))
#define alloc_netdev_mqs(size, name, name_assign_type, setup, sdma, ctxts) \
	alloc_netdev_mqs((size), (name), (setup), (sdma), (ctxts))

#define rdma_create_ah(a, b, c) rdma_create_ah(a, b)
#define rdma_destroy_ah(a, b) rdma_destroy_ah(a)
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

#endif //RH78_COMPAT
