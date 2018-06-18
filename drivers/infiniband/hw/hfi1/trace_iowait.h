/*
 * Copyright(c) 2015, 2016 Intel Corporation.
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
#if !defined(__HFI1_TRACE_IOWAIT_H) || defined(TRACE_HEADER_MULTI_READ)
#define __HFI1_TRACE_IOWAIT_H

#include <linux/tracepoint.h>
#include "iowait.h"
#include "verbs.h"

#undef TRACE_SYSTEM
#define TRACE_SYSTEM hfi1_iowait

DECLARE_EVENT_CLASS(hfi1_iowait_template,
		    TP_PROTO(struct iowait *wait, u32 flag),
		    TP_ARGS(wait, flag),
		    TP_STRUCT__entry(
			    __field(unsigned long, addr)
			    __field(unsigned long, flags)
			    __field(u32, flag)
			    __field(u32, qpn)
			    ),
		    TP_fast_assign(
			    __entry->addr = (unsigned long)wait;
			    __entry->flags = wait->flags;
			    __entry->flag = (1 << flag);
			    __entry->qpn = iowait_to_qp(wait)->ibqp.qp_num;
			    ),
		    TP_printk(
			    "iowait 0x%lx qp %u flags 0x%lx flag 0x%x",
			    __entry->addr,
			    __entry->qpn,
			    __entry->flags,
			    __entry->flag
			    )
	);

DEFINE_EVENT(hfi1_iowait_template, hfi1_iowait_set,
	     TP_PROTO(struct iowait *wait, u32 flag),
	     TP_ARGS(wait, flag));

DEFINE_EVENT(hfi1_iowait_template, hfi1_iowait_clear,
	     TP_PROTO(struct iowait *wait, u32 flag),
	     TP_ARGS(wait, flag));

#endif /* __HFI1_TRACE_IOWAIT_H */

#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE trace_iowait
#include <trace/define_trace.h>
