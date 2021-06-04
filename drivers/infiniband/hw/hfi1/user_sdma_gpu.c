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

#include "sdma.h"
#include "trace.h"
#include "mmu_rb.h"
#include "user_sdma.h"
#include "user_sdma_gpu.h"

int user_sdma_txadd_gpu(struct user_sdma_request *req,
			struct user_sdma_txreq *tx,
			struct user_sdma_iovec *iovec, u32 datalen,
			u32 *queued_ptr, u32 *data_sent_ptr,
			u64 *iov_offset_ptr)
{
	int ret;
	unsigned int pageidx, len;
	unsigned long base, offset;
	u64 iov_offset = *iov_offset_ptr;
	u32 queued = *queued_ptr, data_sent = *data_sent_ptr;
	struct hfi1_user_sdma_pkt_q *pq = req->pq;
	struct nvidia_p2p_page_table *gpu;

	base = (unsigned long)iovec->iov.iov_base;
	offset = ((base + iovec->offset + iov_offset) & ~NV_GPU_PAGE_MASK);
	pageidx = (((iovec->offset + iov_offset + base) -
		   (base & NV_GPU_PAGE_MASK)) >> NV_GPU_PAGE_SHIFT);
	len = offset + req->info.fragsize > NV_GPU_PAGE_SIZE ?
				NV_GPU_PAGE_SIZE - offset : req->info.fragsize;
	len = min((datalen - queued), len);
	gpu = iovec->pages.gpu;
	ret = sdma_txadd_daddr(pq->dd, &tx->txreq,
			       gpu->pages[pageidx]->physical_address + offset,
			       len);
	if (ret) {
		SDMA_DBG(req, "NV: base 0x%lx, offset: 0x%lx, idx: %u, len: %u",
			 base, offset, pageidx, len);
		SDMA_DBG(req, "SDMA txreq add page failed %d\n", ret);
		return ret;
	}
	iov_offset += len;
	queued += len;
	data_sent += len;
	if (unlikely(queued < datalen && pageidx == iovec->npages &&
		     req->iov_idx < req->data_iovs - 1)) {
		iovec->offset += iov_offset;
		iovec = &req->iovs[++req->iov_idx];
		iov_offset = 0;
	}

	*queued_ptr = queued;
	*data_sent_ptr = data_sent;
	*iov_offset_ptr = iov_offset;
	return ret;
}

static u32 sdma_cache_evict_gpu(struct hfi1_user_sdma_pkt_q *pq, u32 npages)
{
	struct evict_data evict_data;

	evict_data.cleared = 0;
	evict_data.target = npages;
	hfi1_mmu_rb_evict(pq->handler_gpu, &evict_data);
	return evict_data.cleared;
}

static void unpin_gpu_pages_callback(void *data)
{
	struct sdma_mmu_node *node = (struct sdma_mmu_node *)data;

	if (node->pages.gpu) {
		/*
		 * Any previous SDMA sends from the node buffer should have
		 * completed by now and the node refcount should be zero.
		 */
		WARN_ON(atomic_read(&node->refcount));
		free_gpu_page_table(node->pages.gpu);
		node->pages.gpu = NULL;
		trace_unpin_gpu_pages_callback(node->rb.addr, node->rb.len);
		hfi1_gpu_cache_invalidate(node->pq->handler_gpu,
					  node->rb.addr,
					  (node->rb.addr + node->rb.len));
	}
}

static inline void release_gpu_mem_pages(unsigned long addr,
					 nvidia_p2p_page_table_t *page_table)
{
	trace_free_sdma_gpu_pages(addr);
	put_gpu_pages(addr, page_table);
}

static void unpin_vector_gpu_mem_pages(struct user_sdma_request *req,
				       struct user_sdma_iovec *iovec)
{
	unsigned long addr = (unsigned long)iovec->iov.iov_base &
							NV_GPU_PAGE_MASK;
	release_gpu_mem_pages(addr, iovec->pages.gpu);
	iovec->pages.gpu = NULL;
	iovec->npages = 0;
	iovec->offset = 0;
}

int pin_sdma_pages_gpu(struct user_sdma_iovec *iovec,
		       struct sdma_mmu_node *node,
		       struct user_sdma_request *req,
		       int npages, unsigned long size,
		       unsigned long addr)
{
	int ret, pinned, locked;
	unsigned int cleared, target;
	unsigned long max_cache_pages;
	struct hfi1_user_sdma_pkt_q *pq = req->pq;

	/*
	 * It was found that nvidia_p2p_get_pages() handles the cases where
	 * a buffer that needs to be pinned is already partially pinned.
	 * So, no additional handling is needed here for such cases.
	 */

	trace_pin_sdma_pages_gpu((unsigned long)iovec->iov.iov_base, addr,
				 iovec->iov.iov_len, size, npages);
	locked = atomic_read(&pq->n_gpu_locked);
	/*
	 * Convert gpu_cache_size in MB to bytes and divide it by
	 * GPU memory page size to calculate the maximum pages allowed
	 * in the GPU buffer cache.
	 */
	max_cache_pages = ((gpu_cache_size << 20) >> NV_GPU_PAGE_SHIFT);
	if (locked + npages > max_cache_pages) {
		if (npages > max_cache_pages)
			return -EINVAL;
		do {
			target = npages - (max_cache_pages - locked);
			cleared = sdma_cache_evict_gpu(pq, target);
			locked = atomic_read(&pq->n_gpu_locked);
		} while (cleared < target);
	}

retry_pin:
	ret = pin_gpu_pages(addr, size, &node->pages.gpu,
			    unpin_gpu_pages_callback, node);
	if (ret) {
		/*
		 * As per NVIDIA's documentation, nvidia_p2p_get_pages API
		 * returns -EINVAL if an invalid argument was supplied.
		 * While testing the code, it was found that the API may
		 * return -EINVAL and not -ENOMEM if there isn't enough GPU
		 * BAR memory to pin the required number of pages. So if the
		 * error return is -ENOMEM or -EINVAL, we make an attempt to
		 * evict npages from the SDMA pinned buffer cache and try
		 * pinning the buffer again. It could be that -EINVAL is
		 * returned from nvidia_p2p_get_pages when an argument is
		 * invalid. Even in that case, cache is evicted continuosly
		 * and once it is empty, we return -ENOMEM to the application
		 * without calling nvidia_p2p_get_pages again. NVIDIA document
		 * doesn't mention any driver API that returns free BAR memory
		 * which would have helped distinguish -EINVAL return when there
		 * isn't enough BAR memory to pin vs an invalid argument.
		 */
		if ((ret == -ENOMEM) || (ret == -EINVAL)) {
			target = npages;
			cleared = sdma_cache_evict_gpu(pq, target);
			/*
			 * If pinning GPU memory pages fails and there aren't
			 * any GPU memory pinned pages in the SDMA cache, then
			 * return -ENOMEM. This can happen when all the GPU
			 * memory pages are pinned to receive buffers. PSM
			 * needs to request the driver to unpin some receive
			 * buffers and re-attempt this failed SDMA transfer.
			 * If SDMA cache eviction unpins some GPU memory
			 * pinned buffers or detects some GPU buffers that are
			 * busy, re-attempt pinning the buffer for SDMA transfer
			 * The buffers in the cache that are busy can also be
			 * unpinned through the callback function invoked by the
			 * NVidia driver when the user space application frees
			 * the GPU buffers.
			 */
			if (cleared == 0 && (atomic_read(&pq->n_gpu_locked) == 0))
				ret = -ENOMEM;
			else
				goto retry_pin;
		}
		trace_sdma_pin_gpu_pages_fail(ret, addr, size);
		return ret;
	}
	trace_sdma_gpu_page_table_info(node->pages.gpu->entries,
				       node->pages.gpu->page_size);
	pinned = node->pages.gpu->entries;

	if (pinned != npages) {
		trace_sdma_pin_gpu_pages_fail(ret, addr, size);
		unpin_vector_gpu_mem_pages(req, iovec);
		return -EFAULT;
	}
	node->rb.len = size;
	atomic_add(pinned, &pq->n_gpu_locked);
	return pinned;
}

void unpin_sdma_pages_gpu(struct sdma_mmu_node *node)
{
	if (node->npages) {
		/*
		 * For GPU buffers that are being removed from the cache via
		 * the callback function, release_gpu_mem_pages which in turn
		 * calls nvidia_p2p_put_pages shouldn't be called as the
		 * callback function unpin_gpu_pages_callback already called
		 * nvidia_p2p_free_page_table. The total number of GPU pages
		 * pinned will still need to be adjusted.
		 */
		if (node->pages.gpu) {
			release_gpu_mem_pages(node->rb.addr, node->pages.gpu);
			node->pages.gpu = NULL;
		}
		atomic_sub(node->npages, &node->pq->n_gpu_locked);
	}
}

int user_sdma_gpu_cache_evict(struct hfi1_filedata *fd,
			      unsigned long arg, u32 len)
{
	unsigned cleared = 0, target;
	struct hfi1_user_sdma_pkt_q *pq = fd->pq;
	struct hfi1_sdma_gpu_cache_evict_params evict_params;

	if (sizeof(evict_params) != len)
		return -EINVAL;

	if (copy_from_user(&evict_params,
			   (struct hfi1_sdma_gpu_cache_evict_params __user *)arg,
			   sizeof(evict_params)))
		return -EFAULT;

	if (evict_params.evict_params_in.version != HFI1_GDR_VERSION)
		return -ENODEV;

	target = evict_params.evict_params_in.pages_to_evict;

	if (target > 0)
		cleared = sdma_cache_evict_gpu(pq, target);
	evict_params.evict_params_out.pages_evicted = cleared;
	evict_params.evict_params_out.pages_in_cache = atomic_read(&pq->n_gpu_locked);

	if (copy_to_user((struct hfi1_sdma_gpu_cache_evict_params __user *)arg,
			 &evict_params,
			 sizeof(evict_params)))
		return -EFAULT;
	return 0;
}
