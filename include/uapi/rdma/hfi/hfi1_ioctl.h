/*
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2015 Intel Corporation.
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
 * Copyright(c) 2015 Intel Corporation.
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

#ifndef _LINUX__HFI1_IOCTL_H
#define _LINUX__HFI1_IOCTL_H
#include <linux/types.h>

/*
 * This structure is passed to the driver to tell it where
 * user code buffers are, sizes, etc.   The offsets and sizes of the
 * fields must remain unchanged, for binary compatibility.  It can
 * be extended, if userversion is changed so user code can tell, if needed
 */
struct hfi1_user_info {
	/*
	 * version of user software, to detect compatibility issues.
	 * Should be set to HFI1_USER_SWVERSION.
	 */
	__u32 userversion;
	__u32 pad;
	/*
	 * If two or more processes wish to share a context, each process
	 * must set the subcontext_cnt and subcontext_id to the same
	 * values.  The only restriction on the subcontext_id is that
	 * it be unique for a given node.
	 */
	__u16 subctxt_cnt;
	__u16 subctxt_id;
	/* 128bit UUID passed in by PSM. */
	__u8 uuid[16];
};

struct hfi1_ctxt_info {
	__u64 runtime_flags;    /* chip/drv runtime flags (HFI1_CAP_*) */
	__u32 rcvegr_size;      /* size of each eager buffer */
	__u16 num_active;       /* number of active units */
	__u16 unit;             /* unit (chip) assigned to caller */
	__u16 ctxt;             /* ctxt on unit assigned to caller */
	__u16 subctxt;          /* subctxt on unit assigned to caller */
	__u16 rcvtids;          /* number of Rcv TIDs for this context */
	__u16 credits;          /* number of PIO credits for this context */
	__u16 numa_node;        /* NUMA node of the assigned device */
	__u16 rec_cpu;          /* cpu # for affinity (0xffff if none) */
	__u16 send_ctxt;        /* send context in use by this user context */
	__u16 egrtids;          /* number of RcvArray entries for Eager Rcvs */
	__u16 rcvhdrq_cnt;      /* number of RcvHdrQ entries */
	__u16 rcvhdrq_entsize;  /* size (in bytes) for each RcvHdrQ entry */
	__u16 sdma_ring_size;   /* number of entries in SDMA request ring */
};

struct hfi1_tid_info {
	/* virtual address of first page in transfer */
	__u64 vaddr;
	/* pointer to tid array. this array is big enough */
	__u64 tidlist;
	/* number of tids programmed by this request */
	__u32 tidcnt;
	/* length of transfer buffer programmed by this request */
	__u32 length;
};

#ifdef NVIDIA_GPU_DIRECT
/*
 * struct hfi1_tid_info_v2 is a copy of struct hfi1_tid_info plus a flags field
 * added at the end of the structure. A new structure is defined instead of
 * adding the flags field to struct hfi1_tid_info to prevent changing the IOCTL
 * command number and maintain backwards compatibility with older PSM versions.
 */
struct hfi1_tid_info_v2 {
	/* virtual address of first page in transfer */
	__u64 vaddr;
	/* pointer to tid array. this array is big enough */
	__u64 tidlist;
	/* number of tids programmed by this request */
	__u32 tidcnt;
	/* length of transfer buffer programmed by this request */
	__u32 length;
	/*  Buffer flags. See HFI1_BUF_* */
	__u16 flags;
};
#endif

/*
 * This structure is returned by the driver immediately after
 * open to get implementation-specific info, and info specific to this
 * instance.
 *
 * This struct must have explicit pad fields where type sizes
 * may result in different alignments between 32 and 64 bit
 * programs, since the 64 bit * bit kernel requires the user code
 * to have matching offsets
 */
struct hfi1_base_info {
	/* version of hardware, for feature checking. */
	__u32 hw_version;
	/* version of software, for feature checking. */
	__u32 sw_version;
	/* Job key */
	__u16 jkey;
	__u16 padding1;
	/*
	 * The special QP (queue pair) value that identifies PSM
	 * protocol packet from standard IB packets.
	 */
	__u32 bthqp;
	/* PIO credit return address, */
	__u64 sc_credits_addr;
	/*
	 * Base address of write-only pio buffers for this process.
	 * Each buffer has sendpio_credits*64 bytes.
	 */
	__u64 pio_bufbase_sop;
	/*
	 * Base address of write-only pio buffers for this process.
	 * Each buffer has sendpio_credits*64 bytes.
	 */
	__u64 pio_bufbase;
	/* address where receive buffer queue is mapped into */
	__u64 rcvhdr_bufbase;
	/* base address of Eager receive buffers. */
	__u64 rcvegr_bufbase;
	/* base address of SDMA completion ring */
	__u64 sdma_comp_bufbase;
	/*
	 * User register base for init code, not to be used directly by
	 * protocol or applications.  Always maps real chip register space.
	 * the register addresses are:
	 * ur_rcvhdrhead, ur_rcvhdrtail, ur_rcvegrhead, ur_rcvegrtail,
	 * ur_rcvtidflow
	 */
	__u64 user_regbase;
	/* notification events */
	__u64 events_bufbase;
	/* status page */
	__u64 status_bufbase;
	/* rcvhdrtail update */
	__u64 rcvhdrtail_base;
	/*
	 * shared memory pages for subctxts if ctxt is shared; these cover
	 * all the processes in the group sharing a single context.
	 * all have enough space for the num_subcontexts value on this job.
	 */
	__u64 subctxt_uregbase;
	__u64 subctxt_rcvegrbuf;
	__u64 subctxt_rcvhdrbuf;
};

#ifdef NVIDIA_GPU_DIRECT

/*
 * Use this for the version field in all the GDR related ioctl parameter
 * structures.  We are starting with version 1.
 */
#define HFI1_GDR_VERSION 0x1UL

/**
 * struct hfi1_sdma_gpu_cache_evict_params - arguments for sdma cache evict
 * @evict_params_in: Values passed into the ioctl
 * @version: The version number for this ioctl.
 * @pages_to_evict: The number of GPU pages we want evicted from this cache.
 * @evict_params_out: Values returned from the ioctl
 * @pages_evicted: The number of GPU pages that were actually evicted.
 * @pages_in_cache: The number of GPU pages resident in this cache.
 */
struct hfi1_sdma_gpu_cache_evict_params {
	union {
		struct {
			__u32 version;
			__u32 pages_to_evict;
		} evict_params_in;
		struct {
			__u32 pages_evicted;
			__u32 pages_in_cache;
		} evict_params_out;
	};
};

/**
 * struct hfi1_gdr_query_parms - argument for gdr driver ioctl command
 * @query_parms_in: Union member containing values passed into the ioctl()
 * @version: A way to pass in a version number for this interface.
 * @gpu_buf_addr: The starting address of a gpu buffer to be operated upon
 * @gpu_buf_size: The size of a gpu buffer to be operated upon
 * @query_params_out: Union member containig values pass back from ioctl()
 * @host_buf_addr: the host address of a pinned and mmaped gpu buffer.
 *
 * This structure is associated with the gdr_ops driver's ioctl commands;
 *
 *	HFI1_IOCTL_GDR_GPU_PIN_MMAP
 *	HFI1_IOCTL_GDR_GPU_MUNMAP_UNPIN
 *
 * It is used to pass in GPU buffer descriptors into the hfi_ops
 * driver.
 *
 * The driver will reject any gpu buffer address or gpu buffer size that
 * is NOT rounded to GPU buffer boundaries.  GPU buffer addresses must
 * start on a NV_GPU_PAGE_SIZE boundary, and a multiple of NV_GPU_PAGE_SIZE
 * in length.
 *
 */
struct hfi1_gdr_query_params {
	union {
		struct {
			__u32 version;
			__u32 gpu_buf_size;
			__u64 gpu_buf_addr;
		} query_params_in;
		struct {
			__u64 host_buf_addr;
		} query_params_out;
	};
};

/**
 * struct hfi1_gdr_cache_evict_params - arguments for GDR cache evict ioctl
 * @version: The version number for this ioctl.
 * @evict_params_in: Values passed into the ioctl
 * @pages_to_evict: The number of GPU pages we want evicted from this cache.
 * @evict_params_out: Values returned from the ioctl
 * @pages_evicted: The number of GPU pages that were actually evicted.
 * @pages_in_cache: The number of GPU pages resident in this cache.
 */
struct hfi1_gdr_cache_evict_params {
	union {
		struct {
			__u32 version;
			__u32 pages_to_evict;
		} evict_params_in;
		struct {
			__u32 pages_evicted;
			__u32 pages_in_cache;
		} evict_params_out;
	};
};
#endif
#endif /* _LINIUX__HFI1_IOCTL_H */
