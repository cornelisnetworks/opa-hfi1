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
#if !defined(__HFI1_TRACE_GPU_H) || defined(TRACE_HEADER_MULTI_READ)
#define __HFI1_TRACE_GPU_H

#include <linux/tracepoint.h>
#include <linux/trace_seq.h>

#undef TRACE_SYSTEM
#define TRACE_SYSTEM hfi1_gpu

DECLARE_EVENT_CLASS(pin_gpu_pages,
		    TP_PROTO(unsigned long base, unsigned long addr,
			     unsigned long len, unsigned long size,
			     unsigned int npages),
		    TP_ARGS(base, addr, len, size, npages),
		    TP_STRUCT__entry(__field(unsigned long, base)
				     __field(unsigned long, addr)
				     __field(unsigned long, len)
				     __field(unsigned long, size)
				     __field(unsigned int, npages)
			    ),
		    TP_fast_assign(__entry->base = base;
				   __entry->addr = addr;
				   __entry->len = len;
				   __entry->size = size;
				   __entry->npages = npages;
			    ),
		    TP_printk("NV: base: 0x%lx, addr: 0x%lx, len: %lu, pin size: %lu, num pages: %u",
			      __entry->base,
			      __entry->addr,
			      __entry->len,
			      __entry->size,
			      __entry->npages
			    )
);

DECLARE_EVENT_CLASS(free_gpu_pages,
		    TP_PROTO(unsigned long addr),
		    TP_ARGS(addr),
		    TP_STRUCT__entry(__field(unsigned long, addr)
			    ),
		    TP_fast_assign(__entry->addr = addr;
			    ),
		    TP_printk("NV: Unpinning the buffer at address: 0x%lx",
			      __entry->addr
			    )
);

DECLARE_EVENT_CLASS(invalidate_gpu_pages,
		    TP_PROTO(unsigned long addr, unsigned long size),
		    TP_ARGS(addr, size),
		    TP_STRUCT__entry(__field(unsigned long, addr)
				     __field(unsigned long, size)
			    ),
		    TP_fast_assign(__entry->addr = addr;
				   __entry->size = size;
			    ),
		    TP_printk("NV: Invalidating address range: start 0x%lx, len %lu",
			      __entry->addr,
			      __entry->size
			    )
);

DECLARE_EVENT_CLASS(gpu_page_table_info,
		    TP_PROTO(unsigned int entries, unsigned int page_size),
		    TP_ARGS(entries, page_size),
		    TP_STRUCT__entry(__field(unsigned int, entries)
				     __field(unsigned int, page_size)
			    ),
		    TP_fast_assign(__entry->entries = entries;
				   __entry->page_size = page_size;
			    ),
		    TP_printk("NV: nvidia_p2p_get_pages: num entries: %u, page size: %u",
			      __entry->entries,
			      __entry->page_size
			    )
);

DECLARE_EVENT_CLASS(pin_gpu_pages_fail,
		    TP_PROTO(int ret, unsigned long addr, unsigned long size),
		    TP_ARGS(ret, addr, size),
		    TP_STRUCT__entry(__field(int, ret)
				     __field(unsigned long, addr)
				     __field(unsigned long, size)
			    ),
		    TP_fast_assign(__entry->ret = ret;
				   __entry->addr = addr;
				   __entry->size = size;
			    ),
		    TP_printk("NV: Failed to pin GPU mem pages. ret %d, addr 0x%lx, size %lu",
			      __entry->ret,
			      __entry->addr,
			      __entry->size
			    )
);

TRACE_EVENT(gpu_page_size_check,
	    TP_PROTO(unsigned int size_type),
	    TP_ARGS(size_type),
	    TP_STRUCT__entry(__field(unsigned int, size_type)),
	    TP_fast_assign(__entry->size_type = size_type;),
	    TP_printk("NV: GPU memory page size is not 64KB. Size type: %u",
		      __entry->size_type)
);

TRACE_EVENT(gpu_page_tbl_check,
	    TP_PROTO(unsigned int comp, unsigned int inst),
	    TP_ARGS(comp, inst),
	    TP_STRUCT__entry(__field(unsigned int, comp)
			     __field(unsigned int, inst)
			    ),
	    TP_fast_assign(__entry->comp = comp;
			   __entry->inst = inst;
			  ),
	    TP_printk("NV: page table version incompatible. Compiled: 0x%x, Installed: 0x%x",
		      __entry->comp,
		      __entry->inst)
);

DEFINE_EVENT(pin_gpu_pages, pin_rcv_pages_gpu,
	     TP_PROTO(unsigned long base, unsigned long addr,
		      unsigned long len, unsigned long size, unsigned int npages),
	     TP_ARGS(base, addr, len, size, npages));

DEFINE_EVENT(pin_gpu_pages, pin_sdma_pages_gpu,
	     TP_PROTO(unsigned long base, unsigned long addr,
		      unsigned long len, unsigned long size, unsigned int npages),
	     TP_ARGS(base, addr, len, size, npages));

DEFINE_EVENT(invalidate_gpu_pages, unpin_rcv_gpu_pages_callback,
	     TP_PROTO(unsigned long addr, unsigned long len),
	     TP_ARGS(addr, len));

DEFINE_EVENT(invalidate_gpu_pages, unpin_gpu_pages_callback,
	     TP_PROTO(unsigned long addr, unsigned long len),
	     TP_ARGS(addr, len));

DEFINE_EVENT(gpu_page_table_info, recv_gpu_page_table_info,
	     TP_PROTO(unsigned int entries, unsigned int page_size),
	     TP_ARGS(entries, page_size));

DEFINE_EVENT(gpu_page_table_info, sdma_gpu_page_table_info,
	     TP_PROTO(unsigned int entries, unsigned int page_size),
	     TP_ARGS(entries, page_size));

DEFINE_EVENT(pin_gpu_pages_fail, recv_pin_gpu_pages_fail,
	     TP_PROTO(int ret, unsigned long addr, unsigned long size),
	     TP_ARGS(ret, addr, size));

DEFINE_EVENT(pin_gpu_pages_fail, sdma_pin_gpu_pages_fail,
	     TP_PROTO(int ret, unsigned long addr, unsigned long size),
	     TP_ARGS(ret, addr, size));

DEFINE_EVENT(free_gpu_pages, free_recv_gpu_pages,
	     TP_PROTO(unsigned long addr),
	     TP_ARGS(addr));

DEFINE_EVENT(free_gpu_pages, free_sdma_gpu_pages,
	     TP_PROTO(unsigned long addr),
	     TP_ARGS(addr));

#endif /* __HFI1_TRACE_GPU_H */

#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE trace_gpu
#include <trace/define_trace.h>
