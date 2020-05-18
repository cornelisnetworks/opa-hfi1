/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Copyright(c) 2019 Intel Corporation.
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
#define HAVE_VM_FAULT_T
#define HAVE_KMALLOC_ARRAY_NODE
#define HAVE_IBDEV_DRIVER_ID
#define HAVE_IB_GET_CACHED_SUBNET_PREFIX
#define HAVE_MAX_SEND_SGE
#define HAVE_SECURITY_H
#define HAVE_RDMA_COPY_AH_ATTR

#include "compat_common.h"

#define rdma_set_device_sysfs_group(a, b)

#endif //RH81_COMPAT
