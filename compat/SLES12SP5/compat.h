/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Copyright(c) 2020 Cornelis Networks, Inc.
 * Copyright(c) 2019-2020 Intel Corporation.
 */
#if !defined(SLES12SP5_COMPAT_H)
#define SLES12SP5_COMPAT_H

#define CREATE_AH_HAS_UDATA
#define HAVE_ALLOC_RDMA_NETDEV
#define HAVE_NOSPEC_H
#define HAVE_ARRAY_SIZE
#define POST_HAS_CONST
#define CREATE_FLOW_HAS_UDATA
#define HAVE_IB_GID_ATTR
#define HAVE_RDMA_NETDEV_GET_PARAMS
#define HAVE_MAX_SEND_SGE
#define HAVE_KMALLOC_ARRAY_NODE
#define HAVE_SECURITY_H
#define HAVE_IBDEV_DRIVER_ID
#define HAVE_RDMA_SET_DEVICE_SYSFS_GROUP
#define HAVE_RDMA_COPY_AH_ATTR
#define NO_RB_ROOT_CACHE
#define NEED_PCI_BRIDGE_SECONDARY_BUS_RESET
#define HAS_PORT_IMMUTABLE
#define IDR_INIT_HAVE_NONAME
#define NO_MMU_NOTIFIER_MM

#include <linux/device.h>
#include <rdma/ib_mad.h>

#include "compat_common.h"

#undef CONFIG_FAULT_INJECTION

#define RDMA_HW_STATS_DEFAULT_LIFESPAN 10
#define OPA_CLASS_PORT_INFO_PR_SUPPORT BIT(26)

#define ib_register_device(a, b) \
	ib_register_device((a), (b), (rdi->driver_f.port_callback))
#define rdma_create_ah(a, b, c) rdma_create_ah(a, b)
#define rdma_destroy_ah(a, b) rdma_destroy_ah(a)
#undef access_ok
#define access_ok(addr, size)                                     \
({                                                                      \
	WARN_ON_IN_IRQ();                                               \
	likely(!__range_not_ok(addr, size, user_addr_max()));           \
})
#define _ib_alloc_device ib_alloc_device

#endif //SLES12SP5_COMPAT
