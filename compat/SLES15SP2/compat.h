/*
 * Copyright(c) 2020 Intel Corporation.
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
#ifndef SLES15SP2_COMPAT_H
#define SLES15SP2_COMPAT_H

#define CREATE_AH_HAS_UDATA
#define HAVE_ALLOC_RDMA_NETDEV
#define POST_HAS_CONST
#define HAVE_IB_GID_ATTR
#define CREATE_FLOW_HAS_UDATA
#define HAVE_RDMA_NETDEV_GET_PARAMS
#define HAVE_ARRAY_SIZE
#define HAVE_NOSPEC_H
#define HAVE_MAX_SEND_SGE
#define HAVE_IBDEV_DRIVER_ID
#define HAVE_IB_GET_CACHED_SUBNET_PREFIX
#define HAVE_SECURITY_H
#define HAVE_KMALLOC_ARRAY_NODE
#define HAVE_RDMA_SET_DEVICE_SYSFS_GROUP
#define HAVE_RDMA_COPY_AH_ATTR
#define HAVE_PUT_USER_PAGES
#define HAVE_ENUM_IB_UVERBS_ADVISE_MR_ADVICE
#define DESTROY_AH_HAS_FLAGS
#define CREATE_AH_HAS_FLAGS
#define HAVE_VM_FAULT_T
#define HAVE_IB_DEVICE_OPS
#define HAVE_RDMA_DEVICE_TO_IBDEV
#define HAVE_MMU_NOTIFIER_RANGE
#define HAVE_CPUS_PTR
#define HAVE_PCI_CORE_AER_CLEAR
#define HAVE_ATOMIC64_PINNED_VM
#define HAVE_PUT_USER_PAGES
#define HAVE_NEW_PUT_USER_PAGES_DIRTY_LOCK
#define IB_DEVICE_OPS_OWNER
#define HAVE_NEW_PROCESS_MAD_FUNCTION
#define HAVE_NETDEV_XMIT_MORE
#define HAVE_SKB_FRAG_OFF
#define KERNEL_PARAM_NEED_CONST
#define ALLOC_MR_HAS_UDATA
#define HAVE_CORE_ALLOC_PD
#define NO_IB_UCONTEXT
#define HAVE_CORE_ALLOC_UCONTEXT
#define HAVE_CORE_ALLOC_SRQ
#define HAVE_CORE_ALLOC_CQ
#define HAVE_CORE_ALLOC_AH
#define DEVICE_OPS_HAVE_ABI_VER
#define IB_DEVICE_OPS_DRIVER_ID
#define DEREG_MR_HAS_UDATA
#define DEALLOC_PD_HAS_UDATA
#define DESTROY_CQ_HAS_UDATA
#define DESTROY_QP_HAS_UDATA
#define HAVE_IB_DEVICE_OPS
#define HAVE_IB_SET_DEVICE_OPS
#define HAVE_IBDEV_INIT_PORT
#define HAVE_XARRAY
#define HAVE_DMASYNC_UMEM_GET
#define HAVE_ATOMIC_FETCH_ADD_UNLESS
#define MMU_NOTIFIER_RANGE_START_USES_MMU_NOTIFIER_RANGE

#include <linux/device.h>
#include <rdma/ib_mad.h>

#include "compat_common.h"

#undef CONFIG_FAULT_INJECTION

#endif //SLES15SP2_COMPAT
