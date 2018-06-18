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
#if !defined(RH74_COMPAT_H)
#define RH74_COMPAT_H

#include "compat_common.h"

#define __GFP_RECLAIM	(__GFP_WAIT)
#define OPA_CAP_MASK3_IsEthOnFabricSupported      BIT(13)

#define IB_FW_VERSION_NAME_MAX			  ETHTOOL_FWVERS_LEN
#define OPA_CLASS_PORT_INFO_PR_SUPPORT BIT(26)

#define PCI_IRQ_LEGACY          BIT(0) /* allow legacy interrupts */
#define PCI_IRQ_MSI             BIT(1) /* allow MSI interrupts */
#define PCI_IRQ_MSIX            BIT(2) /* allow MSI-X interrupts */
#define PCI_IRQ_AFFINITY        BIT(3) /* auto-assign affinity */
#define PCI_IRQ_ALL_TYPES \
(PCI_IRQ_LEGACY | PCI_IRQ_MSI | PCI_IRQ_MSIX)

struct hfi1_msix_entry;
struct hfi1_devdata;

/**
 * struct irq_affinity - Description for automatic irq affinity assignements
 * @pre_vectors:        Don't apply affinity to @pre_vectors at beginning of
 *                      the MSI(-X) vector space
 * @post_vectors:       Don't apply affinity to @post_vectors at end of
 *                      the MSI(-X) vector space
 */
struct irq_affinity {
	int     pre_vectors;
	int     post_vectors;
};

void pcie_flr(struct pci_dev *dev);
int pci_alloc_irq_vectors(struct pci_dev *dev, unsigned int min_vecs,
			  unsigned int max_vecs, unsigned int flags);
void msix_setup(struct pci_dev *pcidev, int pos, u32 *msixcnt,
		struct hfi1_msix_entry *hfi1_msix_entry);
int bitmap_print_to_pagebuf(bool list, char *buf,
			    const unsigned long *maskp, int nmaskbits);
struct ib_ah *rdma_create_ah(struct ib_pd *pd, struct rdma_ah_attr *ah_attr);

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

static inline void pci_free_irq_vectors(struct pci_dev *dev)
{
	pci_disable_msix(dev);
	pci_disable_msi(dev);
}

/* Helpers to hide struct msi_desc implementation details */
#define msi_desc_to_dev(desc)           ((desc)->dev)
#define dev_to_msi_list(dev)            (&(dev)->msi_list)
#define first_msi_entry(dev)            \
	list_first_entry(dev_to_msi_list((dev)), struct msi_desc, list)
#define for_each_msi_entry(desc, dev)   \
	list_for_each_entry((desc), dev_to_msi_list((dev)), list)

#ifdef CONFIG_PCI_MSI
/**
 * pci_irq_vector - return Linux IRQ number of a device vector
 * @dev: PCI device to operate on
 * @nr: device-relative interrupt vector index (0-based).
 */
static inline int pci_irq_vector(struct pci_dev *dev, unsigned int nr)
{
	if (dev->msix_enabled) {
		struct msi_desc *entry;
		int i = 0;

		for_each_msi_entry(entry, dev) {
			if (i == nr)
				return entry->irq;
			i++;
		}
		WARN_ON_ONCE(1);
		return -EINVAL;
	}

	if (dev->msi_enabled) {
		struct msi_desc *entry = first_msi_entry(dev);

		if (WARN_ON_ONCE(nr >= entry->nvec_used))
			return -EINVAL;
	} else {
		if (WARN_ON_ONCE(nr > 0))
			return -EINVAL;
	}

	return dev->irq + nr;
}
#else
static inline int pci_irq_vector(struct pci_dev *dev, unsigned int nr)
{
	if (WARN_ON_ONCE(nr > 0))
		return -EINVAL;
	return dev->irq;
}
#endif

#endif //RH74_COMPAT
