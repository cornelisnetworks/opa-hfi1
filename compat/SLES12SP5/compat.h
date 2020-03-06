/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Copyright(c) 2019 Intel Corporation.
 */
#if !defined(SLES12SP5_COMPAT_H)
#define SLES12SP5_COMPAT_H

#include <linux/device.h>
#include <rdma/ib_mad.h>

#define CREATE_AH_HAS_UDATA
#define HAVE_ALLOC_RDMA_NETDEV
#define HAVE_NOSPEC_H
#define HAVE_ARRAY_SIZE
#define POST_HAS_CONST
#define CREATE_FLOW_HAS_UDATA
#define HAVE_IB_GID_ATTR
#define HAVE_RDMA_NETDEV_GET_PARAMS

#include "compat_common.h"

#undef CONFIG_FAULT_INJECTION

#define RDMA_HW_STATS_DEFAULT_LIFESPAN 10
#define OPA_CLASS_PORT_INFO_PR_SUPPORT BIT(26)

#define rdma_create_ah(a, b, c) rdma_create_ah(a, b)
#define rdma_destroy_ah(a, b) rdma_destroy_ah(a)
#define rdma_set_device_sysfs_group(a, b)
#undef access_ok
#define access_ok(addr, size)                                     \
({                                                                      \
	WARN_ON_IN_IRQ();                                               \
	likely(!__range_not_ok(addr, size, user_addr_max()));           \
})
#define _ib_alloc_device ib_alloc_device

#endif //SLES12SP5_COMPAT
