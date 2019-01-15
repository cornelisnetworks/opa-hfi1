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

	ah = pd->device->create_ah(pd, ah_attr, NULL);

	if (!IS_ERR(ah)) {
		ah->device  = pd->device;
		ah->pd      = pd;
		ah->uobject = NULL;
		atomic_inc(&pd->usecnt);
	}

	return ah;
}
EXPORT_SYMBOL(rdma_create_ah);

/*
 * The following functions implement driver specific replacements
 * for the ib_dma_*() functions.
 *
 * These functions return kernel virtual addresses instead of
 * device bus addresses since the driver uses the CPU to copy
 * data instead of using hardware DMA.
 */

static int rvt_mapping_error(struct ib_device *dev, u64 dma_addr)
{
	return dma_addr == BAD_DMA_ADDRESS;
}

static u64 rvt_dma_map_single(struct ib_device *dev, void *cpu_addr,
			      size_t size, enum dma_data_direction direction)
{
	if (WARN_ON(!valid_dma_direction(direction)))
		return BAD_DMA_ADDRESS;

	return (u64)cpu_addr;
}

static void rvt_dma_unmap_single(struct ib_device *dev, u64 addr, size_t size,
				 enum dma_data_direction direction)
{
	/* This is a stub, nothing to be done here */
}

static u64 rvt_dma_map_page(struct ib_device *dev, struct page *page,
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

static void rvt_dma_unmap_page(struct ib_device *dev, u64 addr, size_t size,
			       enum dma_data_direction direction)
{
	/* This is a stub, nothing to be done here */
}

static int rvt_map_sg(struct ib_device *dev, struct scatterlist *sgl,
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

static void rvt_unmap_sg(struct ib_device *dev,
			 struct scatterlist *sg, int nents,
			 enum dma_data_direction direction)
{
	/* This is a stub, nothing to be done here */
}

static int rvt_map_sg_attrs(struct ib_device *dev, struct scatterlist *sgl,
			    int nents, enum dma_data_direction direction,
			    struct dma_attrs *attrs)
{
	return rvt_map_sg(dev, sgl, nents, direction);
}

static void rvt_unmap_sg_attrs(struct ib_device *dev,
			       struct scatterlist *sg, int nents,
			       enum dma_data_direction direction,
			       struct dma_attrs *attrs)
{
	return rvt_unmap_sg(dev, sg, nents, direction);
}

static void rvt_sync_single_for_cpu(struct ib_device *dev, u64 addr,
				    size_t size, enum dma_data_direction dir)
{
}

static void rvt_sync_single_for_device(struct ib_device *dev, u64 addr,
				       size_t size,
				       enum dma_data_direction dir)
{
}

static void *rvt_dma_alloc_coherent(struct ib_device *dev, size_t size,
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

static void rvt_dma_free_coherent(struct ib_device *dev, size_t size,
				  void *cpu_addr, u64 dma_handle)
{
	free_pages((unsigned long)cpu_addr, get_order(size));
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,4,126))
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
#endif

struct ib_dma_mapping_ops rvt_default_dma_mapping_ops = {
	.mapping_error = rvt_mapping_error,
	.map_single = rvt_dma_map_single,
	.unmap_single = rvt_dma_unmap_single,
	.map_page = rvt_dma_map_page,
	.unmap_page = rvt_dma_unmap_page,
	.map_sg = rvt_map_sg,
	.unmap_sg = rvt_unmap_sg,
	.map_sg_attrs = rvt_map_sg_attrs,
	.unmap_sg_attrs = rvt_unmap_sg_attrs,
	.sync_single_for_cpu = rvt_sync_single_for_cpu,
	.sync_single_for_device = rvt_sync_single_for_device,
	.alloc_coherent = rvt_dma_alloc_coherent,
	.free_coherent = rvt_dma_free_coherent
};

