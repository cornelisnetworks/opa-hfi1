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

/* Address handles */
struct ib_ah *rdma_create_ah(struct ib_pd *pd, struct rdma_ah_attr *ah_attr)
{
	struct ib_ah *ah;

	ah = pd->device->create_ah(pd, ah_attr);

	if (!IS_ERR(ah)) {
		ah->device  = pd->device;
		ah->pd      = pd;
		ah->uobject = NULL;
		atomic_inc(&pd->usecnt);
	}

	return ah;
}
EXPORT_SYMBOL(rdma_create_ah);

#if !HAVE_MEMDUP_USER_NUL
/**
 * memdup_user_nul - duplicate memory region from user space and NUL-terminate
 *
 * @src: source address in user space
 * @len: number of bytes to copy
 *
 * Returns an ERR_PTR() on failure.
 */
void *memdup_user_nul(const void __user *src, size_t len)
{
	char *p;

	/*
	 * Always use GFP_KERNEL, since copy_from_user() can sleep and
	 * cause pagefault, which makes it pointless to use GFP_NOFS
	 * or GFP_ATOMIC.
	 */
	p = kmalloc_track_caller(len + 1, GFP_KERNEL);
	if (!p)
		return ERR_PTR(-ENOMEM);

	if (copy_from_user(p, src, len)) {
		kfree(p);
		return ERR_PTR(-EFAULT);
	}
	p[len] = '\0';

	return p;
}
EXPORT_SYMBOL(memdup_user_nul);
#endif

int pci_alloc_irq_vectors(struct pci_dev *pcidev, unsigned int min_vecs,
			  unsigned int max_vecs, unsigned int flags)
{
	int i, nvec;
	struct hfi1_msix_entry *entries;

	if (min_vecs != 1 || max_vecs < min_vecs)
		return -ERANGE;

	nvec = max_vecs;

	if (flags & PCI_IRQ_MSIX) {
		entries = kcalloc(nvec, sizeof(*entries), GFP_KERNEL);
		if (!entries)
			return -ENOMEM;

		/* 1-1 MSI-X entry assignment */
		for (i = 0; i < max_vecs; i++)
			entries[i].msix.entry = i;

		msix_setup(pcidev, pcidev->msix_cap, &nvec, entries);
		return nvec;
	}

	if (flags & PCI_IRQ_LEGACY) {
		hfi1_enable_intx(pcidev);
		return 1;
	}

	return -ENOSPC;
}
EXPORT_SYMBOL(pci_alloc_irq_vectors);

void msix_setup(struct pci_dev *pcidev, int pos, u32 *msixcnt,
		struct hfi1_msix_entry *hfi1_msix_entry)
{
	int ret;
	int nvec = *msixcnt;
	struct msix_entry *msix_entry;
	int i;

	/*
	 * We can't pass hfi1_msix_entry array to msix_setup
	 * so use a dummy msix_entry array and copy the allocated
	 * irq back to the hfi1_msix_entry array.
	 */
	msix_entry = kmalloc_array(nvec, sizeof(*msix_entry), GFP_KERNEL);
	if (!msix_entry) {
		ret = -ENOMEM;
		goto do_intx;
	}

	for (i = 0; i < nvec; i++)
		msix_entry[i] = hfi1_msix_entry[i].msix;

	ret = pci_enable_msix_range(pcidev, msix_entry, 1, nvec);
	if (ret < 0)
		goto free_msix_entry;
	nvec = ret;

	for (i = 0; i < nvec; i++)
		hfi1_msix_entry[i].msix = msix_entry[i];

	kfree(msix_entry);
	*msixcnt = nvec;
	return;

free_msix_entry:
	kfree(msix_entry);

do_intx:
	*msixcnt = 0;
	hfi1_enable_intx(pcidev);
}
EXPORT_SYMBOL(msix_setup);

/**
 * debugfs_use_file_start - mark the beginning of file data access
 * @dentry: the dentry object whose data is being accessed.
 * @srcu_idx: a pointer to some memory to store a SRCU index in.
 *
 * Up to a matching call to debugfs_use_file_finish(), any
 * successive call into the file removing functions debugfs_remove()
 * and debugfs_remove_recursive() will block. Since associated private
 * file data may only get freed after a successful return of any of
 * the removal functions, you may safely access it after a successful
 * call to debugfs_use_file_start() without worrying about
 * lifetime issues.
 *
 * If -%EIO is returned, the file has already been removed and thus,
 * it is not safe to access any of its data. If, on the other hand,
 * it is allowed to access the file data, zero is returned.
 *
 * Regardless of the return code, any call to
 * debugfs_use_file_start() must be followed by a matching call
 * to debugfs_use_file_finish().
 */
int debugfs_use_file_start(struct dentry *dentry, int *srcu_idx)
	__acquires(&debugfs_srcu)
{
	*srcu_idx = srcu_read_lock(&debugfs_srcu);
	barrier();
	if (d_unlinked(dentry))
		return -EIO;
	return 0;
}
EXPORT_SYMBOL(debugfs_use_file_start);

/**
 * debugfs_use_file_finish - mark the end of file data access
 * @srcu_idx: the SRCU index "created" by a former call to
 *            debugfs_use_file_start().
 *
 * Allow any ongoing concurrent call into debugfs_remove() or
 * debugfs_remove_recursive() blocked by a former call to
 * debugfs_use_file_start() to proceed and return to its caller.
 */
void debugfs_use_file_finish(int srcu_idx) __releases(&debugfs_srcu)
{
	srcu_read_unlock(&debugfs_srcu, srcu_idx);
}
EXPORT_SYMBOL(debugfs_use_file_finish);

int rvt_mapping_error(struct ib_device *dev, u64 dma_addr)
{
	return dma_addr == BAD_DMA_ADDRESS;
}
EXPORT_SYMBOL(rvt_mapping_error);

u64 rvt_dma_map_single(struct ib_device *dev, void *cpu_addr,
		       size_t size, enum dma_data_direction direction)
{
	if (WARN_ON(!valid_dma_direction(direction)))
		return BAD_DMA_ADDRESS;

	return (u64)cpu_addr;
}
EXPORT_SYMBOL(rvt_dma_map_single);

void rvt_dma_unmap_single(struct ib_device *dev, u64 addr, size_t size,
			  enum dma_data_direction direction)
{
	/* This is a stub, nothing to be done here */
}
EXPORT_SYMBOL(rvt_dma_unmap_single);

u64 rvt_dma_map_page(struct ib_device *dev, struct page *page,
		     unsigned long offset, size_t size,
		     enum dma_data_direction direction)
{
	u64 addr;

	if (WARN_ON(!valid_dma_direction(direction)))
		return BAD_DMA_ADDRESS;

	addr = (u64)page_address(page);
	if (addr)
		addr += offset;

	return addr;
}
EXPORT_SYMBOL(rvt_dma_map_page);

void rvt_dma_unmap_page(struct ib_device *dev, u64 addr, size_t size,
			enum dma_data_direction direction)
{
	/* This is a stub, nothing to be done here */
}
EXPORT_SYMBOL(rvt_dma_unmap_page);

int rvt_map_sg(struct ib_device *dev, struct scatterlist *sgl,
	       int nents, enum dma_data_direction direction)
{
	struct scatterlist *sg;
	u64 addr;
	int i;
	int ret = nents;

	if (WARN_ON(!valid_dma_direction(direction)))
		return 0;

	for_each_sg(sgl, sg, nents, i) {
		addr = (u64)page_address(sg_page(sg));
		if (!addr) {
			ret = 0;
			break;
		}
		sg->dma_address = addr + sg->offset;
#ifdef CONFIG_NEED_SG_DMA_LENGTH
		sg->dma_length = sg->length;
#endif
	}
	return ret;
}
EXPORT_SYMBOL(rvt_map_sg);

void rvt_unmap_sg(struct ib_device *dev,
		  struct scatterlist *sg, int nents,
		  enum dma_data_direction direction)
{
	/* This is a stub, nothing to be done here */
}
EXPORT_SYMBOL(rvt_unmap_sg);

void rvt_sync_single_for_cpu(struct ib_device *dev, u64 addr,
			     size_t size, enum dma_data_direction dir)
{
}
EXPORT_SYMBOL(rvt_sync_single_for_cpu);

void rvt_sync_single_for_device(struct ib_device *dev, u64 addr,
				size_t size,
				enum dma_data_direction dir)
{
}
EXPORT_SYMBOL(rvt_sync_single_for_device);

void *rvt_dma_alloc_coherent(struct ib_device *dev, size_t size,
			     u64 *dma_handle, gfp_t flag)
{
	struct page *p;
	void *addr = NULL;

	p = alloc_pages(flag, get_order(size));
	if (p)
		addr = page_address(p);
	if (dma_handle)
		*dma_handle = (u64)addr;
	return addr;
}
EXPORT_SYMBOL(rvt_dma_alloc_coherent);

void rvt_dma_free_coherent(struct ib_device *dev, size_t size,
			   void *cpu_addr, u64 dma_handle)
{
	free_pages((unsigned long)cpu_addr, get_order(size));
}
EXPORT_SYMBOL(rvt_dma_free_coherent);

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

struct ib_dma_mapping_ops rvt_default_dma_mapping_ops = {
	.mapping_error = rvt_mapping_error,
	.map_single = rvt_dma_map_single,
	.unmap_single = rvt_dma_unmap_single,
	.map_page = rvt_dma_map_page,
	.unmap_page = rvt_dma_unmap_page,
	.map_sg = rvt_map_sg,
	.unmap_sg = rvt_unmap_sg,
	.sync_single_for_cpu = rvt_sync_single_for_cpu,
	.sync_single_for_device = rvt_sync_single_for_device,
	.alloc_coherent = rvt_dma_alloc_coherent,
	.free_coherent = rvt_dma_free_coherent
};
