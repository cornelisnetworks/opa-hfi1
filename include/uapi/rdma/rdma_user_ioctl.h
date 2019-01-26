/*
 * Copyright (c) 2016 Mellanox Technologies, LTD. All rights reserved.
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the
 * OpenIB.org BSD license below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef RDMA_USER_IOCTL_H
#define RDMA_USER_IOCTL_H

#include <linux/types.h>
#include <linux/ioctl.h>
#include <rdma/ib_user_mad.h>
#include <rdma/hfi/hfi1_ioctl.h>

/* Documentation/ioctl/ioctl-number.txt */
#define RDMA_IOCTL_MAGIC	0x1b
/* Legacy name, for user space application which already use it */
#ifndef IB_IOCTL_MAGIC
#define IB_IOCTL_MAGIC		RDMA_IOCTL_MAGIC
#endif

#define RDMA_VERBS_IOCTL \
	_IOWR(RDMA_IOCTL_MAGIC, 1, struct ib_uverbs_ioctl_hdr)

#define UVERBS_ID_NS_MASK 0xF000
#define UVERBS_ID_NS_SHIFT 12

enum {
	/* User input */
	UVERBS_ATTR_F_MANDATORY = 1U << 0,
	/*
	 * Valid output bit should be ignored and considered set in
	 * mandatory fields. This bit is kernel output.
	 */
	UVERBS_ATTR_F_VALID_OUTPUT = 1U << 1,
};

struct ib_uverbs_attr {
	__u16 attr_id;		/* command specific type attribute */
	__u16 len;		/* only for pointers */
	__u16 flags;		/* combination of UVERBS_ATTR_F_XXXX */
	__u16 reserved;
	__u64 data;		/* ptr to command, inline data or idr/fd */
};

struct ib_uverbs_ioctl_hdr {
	__u16 length;
	__u16 object_id;
	__u16 method_id;
	__u16 num_attrs;
	__u64 reserved;
	struct ib_uverbs_attr  attrs[0];
};

/*
 * General blocks assignments
 * It is closed on purpose do not expose it it user space
 * #define MAD_CMD_BASE		0x00
 * #define HFI1_CMD_BAS		0xE0
 */

/* MAD specific section */
#ifndef IB_USER_MAD_REGISTER_AGENT
#define IB_USER_MAD_REGISTER_AGENT	_IOWR(RDMA_IOCTL_MAGIC, 0x01, struct ib_user_mad_reg_req)
#endif
#ifndef IB_USER_MAD_UNREGISTER_AGENT
#define IB_USER_MAD_UNREGISTER_AGENT	_IOW(RDMA_IOCTL_MAGIC,  0x02, __u32)
#endif
#ifndef IB_USER_MAD_ENABLE_PKEY
#define IB_USER_MAD_ENABLE_PKEY		_IO(RDMA_IOCTL_MAGIC,   0x03)
#endif
#ifndef IB_USER_MAD_REGISTER_AGENT2
#define IB_USER_MAD_REGISTER_AGENT2	_IOWR(RDMA_IOCTL_MAGIC, 0x04, struct ib_user_mad_reg_req2)
#endif

/* HFI specific section */
/* allocate HFI and context */
#define HFI1_IOCTL_ASSIGN_CTXT		_IOWR(RDMA_IOCTL_MAGIC, 0xE1, struct hfi1_user_info)
/* find out what resources we got */
#define HFI1_IOCTL_CTXT_INFO		_IOW(RDMA_IOCTL_MAGIC,  0xE2, struct hfi1_ctxt_info)
/* set up userspace */
#define HFI1_IOCTL_USER_INFO		_IOW(RDMA_IOCTL_MAGIC,  0xE3, struct hfi1_base_info)
/* update expected TID entries */
#define HFI1_IOCTL_TID_UPDATE		_IOWR(RDMA_IOCTL_MAGIC, 0xE4, struct hfi1_tid_info)
/* free expected TID entries */
#define HFI1_IOCTL_TID_FREE		_IOWR(RDMA_IOCTL_MAGIC, 0xE5, struct hfi1_tid_info)
/* force an update of PIO credit */
#define HFI1_IOCTL_CREDIT_UPD		_IO(RDMA_IOCTL_MAGIC,   0xE6)
/* control receipt of packets */
#define HFI1_IOCTL_RECV_CTRL		_IOW(RDMA_IOCTL_MAGIC,  0xE8, int)
/* set the kind of polling we want */
#define HFI1_IOCTL_POLL_TYPE		_IOW(RDMA_IOCTL_MAGIC,  0xE9, int)
/* ack & clear user status bits */
#define HFI1_IOCTL_ACK_EVENT		_IOW(RDMA_IOCTL_MAGIC,  0xEA, unsigned long)
/* set context's pkey */
#define HFI1_IOCTL_SET_PKEY		_IOW(RDMA_IOCTL_MAGIC,  0xEB, __u16)
/* reset context's HW send context */
#define HFI1_IOCTL_CTXT_RESET		_IO(RDMA_IOCTL_MAGIC,   0xEC)
/* read TID cache invalidations */
#define HFI1_IOCTL_TID_INVAL_READ	_IOWR(RDMA_IOCTL_MAGIC, 0xED, struct hfi1_tid_info)
/* get the version of the user cdev */
#define HFI1_IOCTL_GET_VERS		_IOR(RDMA_IOCTL_MAGIC,  0xEE, int)

#ifdef NVIDIA_GPU_DIRECT
#define HFI1_IOCTL_SDMA_CACHE_EVICT     _IOWR(RDMA_IOCTL_MAGIC, 0xFD, struct hfi1_sdma_gpu_cache_evict_params)

#define HFI1_IOCTL_TID_UPDATE_V2        _IOWR(RDMA_IOCTL_MAGIC, 0xFE, struct hfi1_tid_info_v2)

/*
 * gdr_ops driver ioctl related declarations
 *	HFI1_IOCTL_GDR_GPU_PIN_MMAP
 *		return the host address of a gpu buffer that has been pinned
 *		and mmaped.
 *
 *	HFI1_IOCTL_GDR_GPU_MUNMAP_UNPIN
 *		unpin a gpu buffer and unmap it from the user address space.
 *
 *	HFI1_IOCTL_GDR_GPU_CACHE_EVICT
 *		Try to evict a number of pages from the GDR cache.
 *		Return the number of pages evicted, and the
 *		number of pages in that cache.
 */
#define GDR_IOCTL_MAGIC 0xDA /* See Documentation/ioctl/ioctl-number.txt */

#define HFI1_IOCTL_GDR_GPU_PIN_MMAP \
	_IOWR(GDR_IOCTL_MAGIC, 1, struct hfi1_gdr_query_params)

#define HFI1_IOCTL_GDR_GPU_MUNMAP_UNPIN \
	_IOWR(GDR_IOCTL_MAGIC, 2, struct hfi1_gdr_query_params)

#define HFI1_IOCTL_GDR_GPU_CACHE_EVICT \
	_IOWR(GDR_IOCTL_MAGIC, 3, struct hfi1_gdr_cache_evict_params)

#endif /* NVIDIA_GPU_DIRECT */

#endif /* RDMA_USER_IOCTL_H */
