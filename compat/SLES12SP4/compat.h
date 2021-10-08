/*
 * Copyright(c) 2020 Cornelis Networks, Inc.
 * Copyright(c) 2018-2020 Intel Corporation.
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
#if !defined(SLES12SP4_COMPAT_H)
#define SLES12SP4_COMPAT_H

#define CREATE_AH_HAS_UDATA
#define HAVE_ALLOC_RDMA_NETDEV
#define HAVE_NOSPEC_H
#define IB_MODIFY_QP_IS_OK_HAS_LINK
#define HAVE_SECURITY_H
#define IB_PORT_ATTR_HAS_GRH_REQUIRED
#define NEED_POLL_T
#define NEED_32_FORCE
#define NO_RB_ROOT_CACHE
#define NEED_FILE_FINISH
#define NEED_PCI_BRIDGE_SECONDARY_BUS_RESET
#define HAS_PORT_IMMUTABLE
#define IDR_INIT_HAVE_NONAME
#define NEED_RDMA_USER_IOCTL_CMDS_H
#define NO_MMU_NOTIFIER_MM

#include <linux/device.h>
#include <rdma/ib_mad.h>

#include "compat_common.h"

#undef CONFIG_FAULT_INJECTION

#define RDMA_HW_STATS_DEFAULT_LIFESPAN 10
#define OPA_CLASS_PORT_INFO_PR_SUPPORT BIT(26)

#define rdma_create_ah(a, b, c) rdma_create_ah(a, b)
#define rdma_destroy_ah(a, b) rdma_destroy_ah(a)
#define ib_register_device(a, b)  ib_register_device((a), (rdi->driver_f.port_callback))
#define rdma_set_device_sysfs_group(a, b)
#undef access_ok
#define access_ok(addr, size)                                     \
({                                                                      \
	WARN_ON_IN_IRQ();                                               \
	likely(!__range_not_ok(addr, size, user_addr_max()));           \
})
#define _ib_alloc_device ib_alloc_device

#endif //SLES12SP4_COMPAT
