/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Copyright(c) 2020 Cornelis Networks, Inc.
 * Copyright(c) 2019-2020 Intel Corporation.
 */
#if !defined(RH81_COMPAT_H)
#define RH81_COMPAT_H

#define CREATE_AH_HAS_UDATA
#define HAVE_ALLOC_RDMA_NETDEV
#define CREATE_FLOW_HAS_UDATA
#define HAVE_IB_GID_ATTR
#define ADD_GID_HAS_GID
#define HAVE_RDMA_NETDEV_GET_PARAMS
#define HAVE_ARRAY_SIZE
#define HAVE_NOSPEC_H
#define HAVE_ENUM_IB_UVERBS_ADVISE_MR_ADVICE
#define HAVE_IB_DEVICE_OPS
#define HAVE_IB_SET_DEVICE_OPS
#define POST_HAS_CONST
#define CREATE_AH_HAS_FLAGS
#define DESTROY_AH_HAS_FLAGS
#define HAVE_KMALLOC_ARRAY_NODE
#define HAVE_IBDEV_DRIVER_ID
#define HAVE_IB_GET_CACHED_SUBNET_PREFIX
#define HAVE_MAX_SEND_SGE
#define HAVE_SECURITY_H
#define HAVE_RDMA_SET_DEVICE_SYSFS_GROUP
#define HAVE_RDMA_DEVICE_TO_IBDEV
#define HAVE_VM_FAULT_T
#define HAVE_RDMA_COPY_AH_ATTR
#define HAVE_PCI_CORE_AER_CLEAR
#define HAS_PORT_IMMUTABLE
#define NO_MMU_NOTIFIER_MM

#include "compat_common.h"

#define ib_register_device(a, b) \
	ib_register_device((a), (b), (rdi->driver_f.port_callback))

#endif //RH81_COMPAT
