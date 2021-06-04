#ifndef _HFI1_GPU_H
#define _HFI1_GPU_H
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
#include "nv-p2p.h"

#define NV_GPU_PAGE_SHIFT 16
#define NV_GPU_PAGE_SIZE BIT(NV_GPU_PAGE_SHIFT)
#define NV_GPU_PAGE_MASK (~(NV_GPU_PAGE_SIZE - 1))

#define GPU_PAGE_TO_PFN(page) (page->physical_address >> NV_GPU_PAGE_SHIFT)

extern unsigned long gpu_cache_size;

static inline void put_gpu_pages(unsigned long vaddr,
				 struct nvidia_p2p_page_table *page_table)
{
	nvidia_p2p_put_pages(0, 0, vaddr, page_table);
}

static inline int get_gpu_pages(unsigned long vaddr, unsigned long len,
				struct nvidia_p2p_page_table **page_table_ptr,
				void (*free_callback)(void *data), void *data)
{
	return nvidia_p2p_get_pages(0, 0, vaddr, len, page_table_ptr,
				    free_callback, data);
}

static inline void free_gpu_page_table(struct nvidia_p2p_page_table *page_table)
{
	nvidia_p2p_free_page_table(page_table);
}

static inline int num_user_pages_gpu(unsigned long addr,
				     unsigned long len)
{
	const unsigned long spage = addr & NV_GPU_PAGE_MASK;
	const unsigned long epage = (addr + len - 1) & NV_GPU_PAGE_MASK;

	return 1 + ((epage - spage) >> NV_GPU_PAGE_SHIFT);
}

int pin_gpu_pages(unsigned long vaddr, unsigned long len,
		  struct nvidia_p2p_page_table **page_table_ptr,
		  void (*free_callback)(void *data), void *data);
#endif /* _HFI1_GPU_H */
