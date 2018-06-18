/*
 * Copyright(c) 2017 Intel Corporation.
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

#include "trace.h"
#include "mmu_rb.h"
#include "user_exp_rcv.h"
#include "user_exp_rcv_gpu.h"

static void unpin_rcv_gpu_pages_callback(void *data)
{
	struct tid_user_buf *tidbuf = (struct tid_user_buf *)data;

	if (tidbuf->pages.gpu) {
		free_gpu_page_table(tidbuf->pages.gpu);
		tidbuf->pages.gpu = NULL;
		if (tidbuf->handler) {
			trace_unpin_rcv_gpu_pages_callback(tidbuf->vaddr,
							   tidbuf->length);
			hfi1_gpu_cache_invalidate(tidbuf->handler,
						  tidbuf->vaddr,
						  (tidbuf->vaddr + tidbuf->length));
		}
	}
}

/**
 * Release pinned receive buffer GPU memory pages.
 */
void unpin_rcv_pages_gpu(struct hfi1_filedata *fd, struct tid_rb_node *node)
{
	if (atomic_dec_and_test(&node->tidbuf->refcount)) {
		/*
		 * If the refcount decrements to 0, there aren't any
		 * nodes that refer to a tidbuf. So the tidbuf can be
		 * freed and the GPU memory pages that have been
		 * pinned for the tidbuf can be unpinned.
		 */
		if (node->tidbuf->pages.gpu) {
			trace_free_recv_gpu_pages(node->tidbuf->vaddr);
			put_gpu_pages(node->tidbuf->vaddr,
				      node->tidbuf->pages.gpu);
		}
		fd->tid_n_gpu_pinned -= node->tidbuf->npages;
		kfree(node->tidbuf);
	}
}

/**
 * Pin receive buffer pages.
 */
int pin_rcv_pages_gpu(struct hfi1_filedata *fd, struct tid_user_buf *tidbuf)
{
	int ret, pinned;
	unsigned int max_cache_pages;
	unsigned long size, unaligned_vaddr;
	struct hfi1_devdata *dd = fd->uctxt->dd;

	/* Get the number of pages the user buffer spans */
	tidbuf->npages = num_user_pages_gpu(tidbuf->vaddr, tidbuf->length);
	if (!tidbuf->npages)
		return -EINVAL;

	if (tidbuf->npages > fd->uctxt->expected_count) {
		dd_dev_err(dd, "Expected buffer too big\n");
		return -EINVAL;
	}
	/*
	 * Convert gpu_cache_size in MB to bytes and calculate the
	 * maximum pages allowed in the cache. If the sum of the
	 * no of pages in the cache and the no of pages that need to
	 * be pinned exceede the maximum pages allowed in the cache,
	 * return -ENOMEM to user space. User space can request the
	 * driver to free up some buffers in the cache and then
	 * pin the new buffers.
	 */
	max_cache_pages = ((gpu_cache_size << 20) >> NV_GPU_PAGE_SHIFT);
	if (fd->tid_n_gpu_pinned + tidbuf->npages > max_cache_pages)
		return -ENOMEM;
	/*
	 * refcount is used to count the no of nodes (tid_rb_node) that
	 * refer to a tidbuf (tid_user_buf). There is one node for each
	 * TID RcvArray mapping of a tidbuf. This refcount is needed
	 * only for GPU memory pinned buffers as all the pages that have
	 * been pinned by nvidia_p2p_get_pages should be unpinned in
	 * one single call to nvidia_p2p_put_pages. We can not unpin a
	 * subset of the pages that have been pinned with
	 * nvidia_p2p_get_pages.
	 */
	atomic_set(&tidbuf->refcount, 0);
	/* Starting virtual address has to be aligned to 64K */
	unaligned_vaddr = tidbuf->vaddr;
	tidbuf->vaddr = unaligned_vaddr & NV_GPU_PAGE_MASK;
	size = unaligned_vaddr + tidbuf->length - tidbuf->vaddr;

	trace_pin_rcv_pages_gpu(unaligned_vaddr, tidbuf->vaddr, tidbuf->length,
				size, tidbuf->npages);
	ret = pin_gpu_pages(tidbuf->vaddr, size, &tidbuf->pages.gpu,
			    unpin_rcv_gpu_pages_callback, tidbuf);
	if (ret != 0) {
		trace_recv_pin_gpu_pages_fail(ret, tidbuf->vaddr, size);
		return ret;
	}
	tidbuf->length = size;
	pinned = tidbuf->pages.gpu->entries;
	fd->tid_n_gpu_pinned += pinned;
	trace_recv_gpu_page_table_info(pinned, tidbuf->pages.gpu->page_size);
	return pinned;
}

u32 find_phys_blocks_gpu(struct tid_user_buf *tidbuf)
{
	struct tid_pageset *list = tidbuf->psets;
	struct nvidia_p2p_page_table *page_table = tidbuf->pages.gpu;
	unsigned int pagecount, pageidx, setcount = 0, i;
	unsigned long pfn, this_pfn;
	unsigned int npages = page_table->entries;

	if (!npages)
		return 0;

	/*
	 * Look for sets of physically contiguous pages in the user buffer.
	 * This will allow us to optimize Expected RcvArray entry usage by
	 * using the bigger supported sizes.
	 */
	pfn = GPU_PAGE_TO_PFN(page_table->pages[0]);
	for (pageidx = 0, pagecount = 1, i = 1; i <= npages; i++) {
		this_pfn = i < npages ?
				GPU_PAGE_TO_PFN(page_table->pages[i]) : 0;

		/*
		 * If the pfn's are not sequential, pages are not physically
		 * contiguous.
		 */
		if (this_pfn != ++pfn) {
			/*
			 * At this point we have to loop over the set of
			 * physically contiguous pages and break them down it
			 * sizes supported by the HW.
			 * There are two main constraints:
			 *     1. The max buffer size is MAX_EXPECTED_BUFFER.
			 *        If the total set size is bigger than that
			 *        program only a MAX_EXPECTED_BUFFER chunk.
			 *     2. The buffer size has to be a power of two. If
			 *        it is not, round down to the closes power of
			 *        2 and program that size.
			 */
			while (pagecount) {
				int maxpages = pagecount;
				u32 bufsize = pagecount * NV_GPU_PAGE_SIZE;

				if (bufsize > MAX_EXPECTED_BUFFER)
					maxpages =
						MAX_EXPECTED_BUFFER >>
						NV_GPU_PAGE_SHIFT;
				else if (!is_power_of_2(bufsize))
					maxpages =
						rounddown_pow_of_two(bufsize) >>
						NV_GPU_PAGE_SHIFT;

				list[setcount].idx = pageidx;
				list[setcount].count = maxpages;
				pagecount -= maxpages;
				pageidx += maxpages;
				setcount++;
			}
			pageidx = i;
			pagecount = 1;
			pfn = this_pfn;
		} else {
			pagecount++;
		}
	}
	return setcount;
}

int set_rcvarray_entry_gpu(struct hfi1_filedata *fd, struct tid_user_buf *tbuf,
			   u32 rcventry, struct tid_group *grp,
			   u16 pageidx, unsigned int npages)
{
	int ret;
	struct hfi1_ctxtdata *uctxt = fd->uctxt;
	struct tid_rb_node *node;
	struct hfi1_devdata *dd = uctxt->dd;
	dma_addr_t phys;

	/*
	 * Allocate the node first so we can handle a potential
	 * failure before we've programmed anything.
	 */
	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node)
		return -ENOMEM;

	node->mmu.len = npages * NV_GPU_PAGE_SIZE;
	node->mmu.addr = tbuf->vaddr + (pageidx * NV_GPU_PAGE_SIZE);
	phys = tbuf->pages.gpu->pages[pageidx]->physical_address;
	node->phys = phys;
	node->tidbuf = tbuf;
	node->npages = npages;
	node->rcventry = rcventry;
	/*
	 * As per GPU Direct documentation, IOMMU must be disabled or configured
	 * for pass-through translation in order for GPU Direct RDMA to work.
	 * So physical address and DMA address would be the same.
	 */
	node->dma_addr = phys;
	node->grp = grp;
	node->freed = false;
	node->ongpu = tbuf->ongpu;

	if (!fd->handler_gpu)
		ret = tid_rb_insert(fd, &node->mmu);
	else
		ret = hfi1_mmu_rb_insert(fd->handler_gpu, &node->mmu);

	if (ret) {
		hfi1_cdbg(TID, "Failed to insert RB node %u 0x%lx, 0x%lx %d",
			  node->rcventry, node->mmu.addr, node->phys, ret);
		kfree(node);
		return -EFAULT;
	}

	/*
	 * In case of GPU memory, buffer size starts at 64K (GPU mem page size)
	 */
	hfi1_put_tid(dd, rcventry, PT_EXPECTED, phys, ilog2(npages) + 5);
	atomic_inc(&node->tidbuf->refcount);
	trace_hfi1_exp_tid_reg(uctxt->ctxt, fd->subctxt, rcventry, npages,
			       node->mmu.addr, node->phys, phys);
	return 0;
}

