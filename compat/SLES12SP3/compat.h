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
#if !defined(SLES12SP3_COMPAT_H)
#define SLES12SP3_COMPAT_H

#include <linux/device.h>
#include <linux/vmalloc.h>
#include <rdma/ib_mad.h>

#define GET_DEV_FW_STR_HAS_LEN
#define NEED_IB_HELPER_FUNCTIONS
#define NEED_MM_HELPER_FUNCTIONS
#define CREATE_AH_HAS_UDATA
#define IB_MODIFY_QP_IS_OK_HAS_LINK
#define VM_OPS_FAULT_HAVE_VMA
#define IB_PORT_ATTR_HAS_GRH_REQUIRED
#define NEED_RVT_DMA_MAPPINGS
#define NEED_CDEV_SET_PARENT
#define NEED_PCI_REQUEST_IRQ
#define NEED_CURRENT_TIME

#include "compat_common.h"

#undef CONFIG_FAULT_INJECTION

#define IB_OPCODE_MSP		0xe0

#define RDMA_HW_STATS_DEFAULT_LIFESPAN 10
#define OPA_CLASS_PORT_INFO_PR_SUPPORT BIT(26)

#define ib_register_device(a, b, c)  ib_register_device((a), (c))
#define rdma_set_device_sysfs_group(a, b)
#undef access_ok
#define access_ok(addr, size) \
	likely(!__range_not_ok(addr, size, user_addr_max()))
#define _ib_alloc_device ib_alloc_device

void pcie_flr(struct pci_dev *dev);

struct rdma_ah_attr;
struct ib_ah *rdma_create_ah(struct ib_pd *pd, struct rdma_ah_attr *ah_attr, u32 create_flags);
int rdma_destroy_ah(struct ib_ah *ah_attr, u32 flags);

static inline void *kvzalloc_node(size_t size, gfp_t flags, int node)
{
	void *ret;

	ret = kzalloc_node(size, flags, node);
	if (!ret)
		ret = vzalloc_node(size, node);
	return ret;
}

#ifndef HAVE_NOSPEC_H

#include <asm/barrier.h>

#ifndef array_index_nospec

/**
 * array_index_mask_nospec() - generate a ~0 mask when index < size, 0 otherwise
 * @index: array element index
 * @size: number of elements in array
 *
 * When @index is out of bounds (@index >= @size), the sign bit will be
 * set.  Extend the sign bit to all bits and invert, giving a result of
 * zero for an out of bounds index, or ~0 if within bounds [0, @size).
 */
#ifndef array_index_mask_nospec
static inline unsigned long array_index_mask_nospec(unsigned long index,
						    unsigned long size)
{
	/*
	 * Always calculate and emit the mask even if the compiler
	 * thinks the mask is not needed. The compiler does not take
	 * into account the value of @index under speculation.
	 */
	OPTIMIZER_HIDE_VAR(index);
	return ~(long)(index | (size - 1UL - index)) >> (BITS_PER_LONG - 1);
}
#endif

/*
 * array_index_nospec - sanitize an array index after a bounds check
 *
 * For a code sequence like:
 *
 *     if (index < size) {
 *         index = array_index_nospec(index, size);
 *         val = array[index];
 *     }
 *
 * ...if the CPU speculates past the bounds check then
 * array_index_nospec() will clamp the index within the range of [0,
 * size).
 */
#define array_index_nospec(index, size)					\
({									\
	typeof(index) _i = (index);					\
	typeof(size) _s = (size);					\
	unsigned long _mask = array_index_mask_nospec(_i, _s);		\
									\
	BUILD_BUG_ON(sizeof(_i) > sizeof(long));			\
	BUILD_BUG_ON(sizeof(_s) > sizeof(long));			\
									\
	(typeof(_i)) (_i & _mask);					\
})
#endif /* array_index_nospec  */

#endif /* !HAVE_NOSPEC_H */

#endif //SLES12SP3_COMPAT
