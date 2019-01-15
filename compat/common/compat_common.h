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
#if !defined(COMPAT_COMMON_H)
#define COMPAT_COMMON_H

#include <linux/socket.h>
#include <net/ipv6.h>
#include <net/ip.h>
#include <linux/netdevice.h>
#include <linux/if_link.h>
#include <uapi/rdma/ib_user_verbs.h>
#include <rdma/ib_verbs.h>
#include <rdma/opa_addr.h>
#include <linux/pci.h>
#include <linux/msi.h>
#include <linux/kthread.h>

#define IB_QPN_MASK     0xFFFFFF
#include <rdma/ib_hdrs.h>

#if !defined(BAD_DMA_ADDRESS)
#define BAD_DMA_ADDRESS ((u64)0)
#endif

#define SHMLBA PAGE_SIZE         /* attach addr a multiple of this */

#define MAX_NICE        19
#define MIN_NICE        -20
#define NICE_WIDTH      (MAX_NICE - MIN_NICE + 1)
#define IB_DEVICE_RDMA_NETDEV_OPA_VNIC	0
#define IB_DEVICE_NODE_DESC_MAX 64
#undef __get_dynamic_array_len
#define __get_dynamic_array_len(field)	\
((__entry->__data_loc_##field >> 16) & 0xffff)

#define  IB_PORT_OPA_MASK_CHG			  BIT(4)

#if !defined(IFS_SLES15)
#define  current_time(inode)			  CURRENT_TIME
#endif

/* Address format                       0x000FF000 */
#if !defined RDMA_CORE_CAP_AF_IB
#define RDMA_CORE_CAP_AF_IB             0x00001000
#endif
#if !defined RDMA_CORE_CAP_ETH_AH
#define RDMA_CORE_CAP_ETH_AH            0x00002000
#endif
#if !defined RDMA_CORE_CAP_OPA_AH
#define RDMA_CORE_CAP_OPA_AH            0x00000000
#endif

#if !defined(kthread_init_work)
#define kthread_init_work(work, fn) init_kthread_work(work, fn)
#endif

#if !defined(RB_ROOT_CACHED)
#define rb_root_cached			rb_root
#define RB_ROOT_CACHED			RB_ROOT
#define rb_erase_cached(node, root)	rb_erase(node, root)
#define rb_first_cached(root)		rb_first(root)
#endif

extern struct srcu_struct debugfs_srcu;
extern struct ib_dma_mapping_ops rvt_default_dma_mapping_ops;

const char *get_unit_name(int unit);
void cdev_set_parent(struct cdev *p, struct kobject *kobj);
int pci_request_irq(struct pci_dev *dev, unsigned int nr,
		    irq_handler_t handler, irq_handler_t thread_fn,
		    void *dev_id, const char *fmt, ...);
void pci_free_irq(struct pci_dev *dev, unsigned int nr, void *dev_id);

#define NEED_MM_HELPER_FUNCTIONS (!defined(IFS_SLES15))

#if NEED_MM_HELPER_FUNCTIONS
/**
 * mmgrab() - Pin a &struct mm_struct.
 * @mm: The &struct mm_struct to pin.
 *
 * Make sure that @mm will not get freed even after the owning task
 * exits. This doesn't guarantee that the associated address space
 * will still exist later on and mmget_not_zero() has to be used before
 * accessing it.
 *
 * This is a preferred way to to pin @mm for a longer/unbounded amount
 * of time.
 *
 * Use mmdrop() to release the reference acquired by mmgrab().
 *
 * See also <Documentation/vm/active_mm.txt> for an in-depth explanation
 * of &mm_struct.mm_count vs &mm_struct.mm_users.
 */
static inline void mmgrab(struct mm_struct *mm)
{
	atomic_inc(&mm->mm_count);
}
#endif

#define NEED_IB_HELPER_FUNCTIONS (!defined(IFS_RH75) && !defined(IFS_SLES15))

#if NEED_IB_HELPER_FUNCTIONS
struct opa_class_port_info {
	u8 base_version;
	u8 class_version;
	__be16 cap_mask;
	__be32 cap_mask2_resp_time;

	u8 redirect_gid[16];
	__be32 redirect_tc_fl;
	__be32 redirect_lid;
	__be32 redirect_sl_qp;
	__be32 redirect_qkey;

	u8 trap_gid[16];
	__be32 trap_tc_fl;
	__be32 trap_lid;
	__be32 trap_hl_qp;
	__be32 trap_qkey;

	__be16 trap_pkey;
	__be16 redirect_pkey;

	u8 trap_sl_rsvd;
	u8 reserved[3];
} __packed;

/* rdma netdev type - specifies protocol type */
enum rdma_netdev_t {
	RDMA_NETDEV_OPA_VNIC,
	RDMA_NETDEV_IPOIB,
};

enum rdma_ah_attr_type {
	RDMA_AH_ATTR_TYPE_IB,
	RDMA_AH_ATTR_TYPE_ROCE,
	RDMA_AH_ATTR_TYPE_OPA,
};

/**
 * struct rdma_netdev - rdma netdev
 * For cases where netstack interfacing is required.
 */
struct rdma_netdev {
	void              *clnt_priv;
	struct ib_device  *hca;
	u8                 port_num;

	/* control functions */
	void (*set_id)(struct net_device *netdev, int id);
	/* send packet */
	int (*send)(struct net_device *dev, struct sk_buff *skb,
		    struct ib_ah *address, u32 dqpn);
	/* multicast */
	int (*attach_mcast)(struct net_device *dev, struct ib_device *hca,
			    union ib_gid *gid, u16 mlid,
		     int set_qkey, u32 qkey);
	int (*detach_mcast)(struct net_device *dev, struct ib_device *hca,
			    union ib_gid *gid, u16 mlid);
};

struct roce_ah_attr {
	u8			dmac[ETH_ALEN];
};

struct opa_ah_attr {
	u32			dlid;
	u8			src_path_bits;
};

#define rdma_ah_attr ib_ah_attr

static inline void rdma_ah_set_sl(struct rdma_ah_attr *attr, u8 sl)
{
	attr->sl = sl;
}

static inline u8 rdma_ah_get_sl(const struct rdma_ah_attr *attr)
{
	return attr->sl;
}

static inline void rdma_ah_set_port_num(struct rdma_ah_attr *attr, u8 port_num)
{
	attr->port_num = port_num;
}

static inline u8 rdma_ah_get_port_num(const struct rdma_ah_attr *attr)
{
	return attr->port_num;
}

static inline void rdma_ah_set_make_grd(struct rdma_ah_attr *attr,
					bool make_grd)
{
}

static inline bool rdma_ah_get_make_grd(const struct rdma_ah_attr *attr)
{
	return false;
}

static inline void rdma_ah_set_ah_flags(struct rdma_ah_attr *attr,
					enum ib_ah_flags flag)
{
	attr->ah_flags = flag;
}

/*To retrieve and modify the grh */
static inline struct ib_global_route
*rdma_ah_retrieve_grh(struct rdma_ah_attr *attr)
{
	return &attr->grh;
}

static inline void rdma_ah_set_subnet_prefix(struct rdma_ah_attr *attr,
					     __be64 prefix)
{
	struct ib_global_route *grh = rdma_ah_retrieve_grh(attr);

	grh->dgid.global.subnet_prefix = prefix;
}

static inline void rdma_ah_set_interface_id(struct rdma_ah_attr *attr,
					    __be64 if_id)
{
	struct ib_global_route *grh = rdma_ah_retrieve_grh(attr);

	grh->dgid.global.interface_id = if_id;
}

/*
 * rdma_modify_ah - Modifies the address vector associated with an address
 *     handle.
 * @ah: The address handle to modify.
 * @ah_attr: The new address vector attributes to associate with the
 *     address handle.
 */
static inline int rdma_modify_ah(struct ib_ah *ah, struct rdma_ah_attr *ah_attr)
{
	return (ah->device->modify_ah ?
	ah->device->modify_ah(ah, ah_attr) : -ENOSYS);
}

/*
 * ib_lid_cpu16 - Return lid in 16bit CPU encoding.
 *     In the current implementation the only way to get
 *     get the 32bit lid is from other sources for OPA.
 *     For IB, lids will always be 16bits so cast the
 *     value accordingly.
 *
 * @lid: A 32bit LID
 */
static inline u16 ib_lid_cpu16(u32 lid)
{
	WARN_ON_ONCE(lid & 0xFFFF0000);
	return (u16)lid;
}

/**
 * rdma_destroy_ah - Destroys an address handle.
 * @ah: The address handle to destroy.
 */
static inline int rdma_destroy_ah(struct ib_ah *ah)
{
	struct ib_pd *pd;
	int ret;

	pd = ah->pd;
	ret = ah->device->destroy_ah(ah);
	if (!ret)
		atomic_dec(&pd->usecnt);

	return ret;
}

static inline const struct ib_global_route
*rdma_ah_read_grh(const struct rdma_ah_attr *attr)
{
	return &attr->grh;
}

static inline void rdma_ah_set_dlid(struct rdma_ah_attr *attr, u32 dlid)
{
	attr->dlid = (u16)dlid;
}

static inline u32 rdma_ah_get_dlid(const struct rdma_ah_attr *attr)
{
	/* Different implementations for QIB and HFI1. */
#ifdef QIB_DRIVER
	return (u32)attr->dlid;
#else
	u32 dlid = attr->dlid;
	const struct ib_global_route *grh = rdma_ah_read_grh(attr);

	/* Modify ah_attr.dlid to be in the 32 bit LID space.
	 * This is how the address will be laid out:
	 * Assuming MCAST_NR to be 4,
	 * 32 bit permissive LID = 0xFFFFFFFF
	 * Multicast LID range = 0xFFFFFFFE to 0xF0000000
	 * Unicast LID range = 0xEFFFFFFF to 1
	 * Invalid LID = 0
	 */
	if (ib_is_opa_gid(&grh->dgid))
		dlid = opa_get_lid_from_gid(&grh->dgid);
	else if ((dlid >= be16_to_cpu(IB_MULTICAST_LID_BASE)) &&
		 (dlid != be16_to_cpu(IB_LID_PERMISSIVE)) &&
		 (dlid != be32_to_cpu(OPA_LID_PERMISSIVE)))
		dlid = dlid - be16_to_cpu(IB_MULTICAST_LID_BASE) +
		opa_get_mcast_base(OPA_MCAST_NR);
	else if (dlid == be16_to_cpu(IB_LID_PERMISSIVE))
		dlid = be32_to_cpu(OPA_LID_PERMISSIVE);

	return dlid;
#endif
}

static inline enum ib_ah_flags
rdma_ah_get_ah_flags(const struct rdma_ah_attr *attr)
{
	return attr->ah_flags;
}

static inline u8 rdma_ah_get_path_bits(const struct rdma_ah_attr *attr)
{
	return attr->src_path_bits;
}

static inline void rdma_ah_set_static_rate(struct rdma_ah_attr *attr,
					   u8 static_rate)
{
	attr->static_rate = static_rate;
}

static inline u8 rdma_ah_get_static_rate(const struct rdma_ah_attr *attr)
{
	return attr->static_rate;
}

/**
 * rdma_cap_opa_ah - Check if the port of device supports
 * OPA Address handles
 * @device: Device to check
 * @port_num: Port number to check
 *
 * Return: true if we are running on an OPA device which supports
 * the extended OPA addressing.
 */
static inline bool rdma_cap_opa_ah(struct ib_device *device, u8 port_num)
{
	return false;
}

/*Get AH type */
static inline enum rdma_ah_attr_type rdma_ah_find_type(struct ib_device *dev,
						       u32 port_num)
{
	return RDMA_AH_ATTR_TYPE_IB;
}

#endif /* NEED_IB_HELPER_FUNCTIONS */
#define NEED_KTHREAD_HELPER_FUNCTIONS (!defined(IFS_SLES15))

#if NEED_KTHREAD_HELPER_FUNCTIONS
static inline bool kthread_queue_work(struct kthread_worker *worker,
				      struct kthread_work *work)
{
	return queue_kthread_work(worker, work);
}

static inline void kthread_flush_work(struct kthread_work *work)
{
	flush_kthread_work(work);
}

/*
 * This API was added in commit f5eabf5e5 and is not yet in any of our distro
 * backports. Once it does show up we will get a compiler error and we can
 * add appropriate #ifdefs
 */
static inline struct kthread_worker *
kthread_create_worker_on_cpu(int cpu, unsigned int flags,
			     const char namefmt[], const char *cq_name)
{
	struct kthread_worker *worker;
	struct task_struct *task;
	int node = -1;

	worker = kzalloc(sizeof(*worker), GFP_KERNEL);
	if (!worker)
		return  NULL;
	init_kthread_worker(worker);
	if (cpu >= 0)
		node = cpu_to_node(cpu);
	task = kthread_create_on_node(
		kthread_worker_fn,
		worker,
		node,
		namefmt, cq_name);
	if (IS_ERR(task)) {
		kfree(worker);
		return NULL;
	}
	if (cpu >= 0)
		kthread_bind(task, cpu);

	wake_up_process(task);
	/*Flush to allow kthread_worker_fn properly set worker->task field*/
	flush_kthread_worker(worker);
	return worker;
}

static inline void kthread_destroy_worker(struct kthread_worker *worker)
{
	if (!worker)
		return;
	flush_kthread_worker(worker);
	kthread_stop(worker->task);
	kfree(worker);
}
#endif

/*
 *  For SELinux until our patches are accepted by the distro
 */

#if defined(IFS_RH75) || defined(IFS_SLES15)
int ib_get_cached_subnet_prefix(struct ib_device *device,
				u8                port_num,
				u64              *sn_pfx);
#endif
#endif
