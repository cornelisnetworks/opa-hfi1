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

#include <linux/pci.h>
#include <linux/types.h>
#include <linux/module.h>
#include "gpu.h"
#include "trace.h"

unsigned long gpu_cache_size = 256;
module_param(gpu_cache_size, ulong, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(gpu_cache_size, "Send and receive side GPU buffers cache size limit (in MB)");

int pin_gpu_pages(unsigned long vaddr, unsigned long len,
		  struct nvidia_p2p_page_table **page_table_ptr,
		  void (*free_callback)(void *data), void *data)
{
	int ret;
	struct nvidia_p2p_page_table *page_table;

	ret = get_gpu_pages(vaddr, len, page_table_ptr, free_callback, data);
	if (!ret) {
		page_table = *page_table_ptr;
		/* Current code supports only 64KB GPU memory page size */
		if (page_table->page_size != NVIDIA_P2P_PAGE_SIZE_64KB) {
			trace_gpu_page_size_check(page_table->page_size);
			put_gpu_pages(vaddr, page_table);
			return -EOPNOTSUPP;
		}
#ifdef NVIDIA_P2P_PAGE_TABLE_VERSION_COMPATIBLE
		if (!NVIDIA_P2P_PAGE_TABLE_VERSION_COMPATIBLE(page_table)) {
			trace_gpu_page_tbl_check(NVIDIA_P2P_PAGE_TABLE_VERSION,
						 page_table->version);
			put_gpu_pages(vaddr, page_table);
			return -EOPNOTSUPP;
		}
#endif
	}
	return ret;
}
