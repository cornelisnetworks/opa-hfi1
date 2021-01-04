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
#include <rdma/opa_port_info.h>
#include <linux/timer.h>
#include <rdma/ib_umem.h>

#define IB_QPN_MASK     0xFFFFFF
#include <rdma/ib_hdrs.h>

#if !defined(BAD_DMA_ADDRESS)
#define BAD_DMA_ADDRESS ((u64)0)
#endif

#define SHMLBA PAGE_SIZE         /* attach addr a multiple of this */

#define MAX_NICE        19
#define MIN_NICE        -20
#define NICE_WIDTH      (MAX_NICE - MIN_NICE + 1)
#define IB_DEVICE_RDMA_NETDEV_OPA_VNIC	(1ULL << 35)
#define IB_DEVICE_NODE_DESC_MAX 64
#undef __get_dynamic_array_len
#define __get_dynamic_array_len(field)	\
((__entry->__data_loc_##field >> 16) & 0xffff)

#define  IB_PORT_OPA_MASK_CHG			  BIT(4)

#ifdef NEED_CURRENT_TIME
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

#ifndef smp_store_mb
#define smp_store_mb(var, value) \
	do { WRITE_ONCE(var, value); barrier(); } while (0)
#endif

extern struct srcu_struct debugfs_srcu;
extern struct ib_dma_mapping_ops rvt_default_dma_mapping_ops;

const char *get_unit_name(int unit);
#ifdef NEED_CDEV_SET_PARENT
void cdev_set_parent(struct cdev *p, struct kobject *kobj);
#endif
#ifdef NEED_PCI_REQUEST_IRQ
int pci_request_irq(struct pci_dev *dev, unsigned int nr,
		    irq_handler_t handler, irq_handler_t thread_fn,
		    void *dev_id, const char *fmt, ...);
void pci_free_irq(struct pci_dev *dev, unsigned int nr, void *dev_id);
#endif
#ifdef NEED_PCI_BRIDGE_SECONDARY_BUS_RESET
static inline int pci_bridge_secondary_bus_reset(struct pci_dev *dev)
{
	pci_reset_bridge_secondary_bus(dev);

	return 0;
}
#endif

#ifdef NEED_MM_HELPER_FUNCTIONS
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

#ifdef NEED_IB_HELPER_FUNCTIONS
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

#ifdef NEED_KTHREAD_HELPER_FUNCTIONS
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

#ifdef HAVE_IB_GET_CACHED_SUBNET_PREFIX
int ib_get_cached_subnet_prefix(struct ib_device *device,
				u8                port_num,
				u64              *sn_pfx);
#endif

#ifndef HAVE_KMALLOC_ARRAY_NODE
static inline void *kmalloc_array_node(size_t n, size_t size, gfp_t flags, int node)
{
	if (size != 0 && n > SIZE_MAX / size)
		return NULL;
	if (__builtin_constant_p(n) && __builtin_constant_p(size))
		return kmalloc_node(n * size, flags, node);
	return __kmalloc_node(n * size, flags, node);
}

static inline void *kcalloc_node(size_t n, size_t size, gfp_t flags, int node)
{
	return kmalloc_array_node(n, size, flags | __GFP_ZERO, node);
}
#endif

#define rdma_netdev ifs_aip_rdma_netdev

struct ifs_aip_rdma_netdev {
        void              *clnt_priv;
        struct ib_device  *hca;
        u8                 port_num;
        int                mtu;

        /*
         * cleanup function must be specified.
         * FIXME: This is only used for OPA_VNIC and that usage should be
         * removed too.
         */
        void (*free_rdma_netdev)(struct net_device *netdev);

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

#define IB_DEVICE_RDMA_NETDEV IB_DEVICE_RDMA_NETDEV_OPA_VNIC
#define IB_QP_CREATE_NETDEV_USE IB_QP_CREATE_RESERVED_START

#ifndef atomic_try_cmpxchg
#define __atomic_try_cmpxchg(type, _p, _po, _n)                         \
({                                                                      \
        typeof(_po) __po = (_po);                                       \
        typeof(*(_po)) __r, __o = *__po;                                \
        __r = atomic_cmpxchg##type((_p), __o, (_n));                    \
        if (unlikely(__r != __o))                                       \
                *__po = __r;                                            \
        likely(__r == __o);                                             \
})

#define atomic_try_cmpxchg(_p, _po, _n)         __atomic_try_cmpxchg(, _p, _po, _n)

#endif

#ifndef atomic_fetch_add_unless
static inline int atomic_fetch_add_unless(atomic_t *v, int a, int u)
{
        int c = atomic_read(v);

        do {
                if (unlikely(c == u))
                        break;
        } while (!atomic_try_cmpxchg(v, &c, c + a));

        return c;
}
#endif

static inline bool rdma_core_cap_opa_port(struct ib_device *device,
					  u32 port_num)
{
	if (!device)
		return false;

#ifdef HAS_PORT_IMMUTABLE
	return !!(device->port_immutable[port_num].core_cap_flags & RDMA_CORE_PORT_INTEL_OPA);
#else
	return !!(device->port_data[port_num].immutable.core_cap_flags & RDMA_CORE_PORT_INTEL_OPA);
#endif
}

static inline int opa_mtu_enum_to_int(int mtu)
{
	switch (mtu) {
	case OPA_MTU_8192:
		return 8192;
	case OPA_MTU_10240:
		return 10240;
	default:
		return(ib_mtu_enum_to_int(mtu));
	}
}

static inline int rdma_mtu_enum_to_int(struct ib_device *device, u8 port,
				       int mtu)
{
	if (rdma_core_cap_opa_port(device, port))
		return opa_mtu_enum_to_int(mtu);
	else
		return ib_mtu_enum_to_int((enum ib_mtu)mtu);
}

#ifndef timer_setup
#define timer_setup(timer, callback, flags)				\
	do {								\
		__init_timer((timer), (flags));				\
		(timer)->function = (void (*)(unsigned long))callback;	\
		(timer)->data = (unsigned long)(timer);			\
	} while (0)
#endif

#ifndef from_timer
#define from_timer(var, callback_timer, timer_fieldname) \
	container_of(callback_timer, typeof(*(var)), timer_fieldname)
#endif

#ifndef HAVE_ENUM_IB_UVERBS_ADVISE_MR_ADVICE
enum ib_uverbs_advise_mr_advice {
	IB_UVERBS_ADVISE_MR_ADVICE_PREFETCH,
	IB_UVERBS_ADVISE_MR_ADVICE_PREFETCH_WRITE,
};
#endif

#ifndef HAVE_IB_DEVICE_OPS

struct iw_cm_id;
struct iw_cm_conn_param;
struct uverbs_attr_bundle;
struct ib_flow_action_attrs_esp;
struct ib_dm_mr_attr;
struct ib_dm_alloc_attr;
struct ib_counters_read_attr;
struct rdma_netdev_alloc_params;
struct rdma_restrack_entry;

/**
 * struct ib_device_ops - InfiniBand device operations
 * This structure defines all the InfiniBand device operations, providers will
 * need to define the supported operations, otherwise they will be set to null.
 */
struct ib_device_ops {
#ifdef POST_HAS_CONST
	int (*post_send)(struct ib_qp *qp, const struct ib_send_wr *send_wr,
			 const struct ib_send_wr **bad_send_wr);
	int (*post_recv)(struct ib_qp *qp, const struct ib_recv_wr *recv_wr,
			 const struct ib_recv_wr **bad_recv_wr);
#else
	int (*post_send)(struct ib_qp *qp, struct ib_send_wr *send_wr,
			 struct ib_send_wr **bad_send_wr);
	int (*post_recv)(struct ib_qp *qp, struct ib_recv_wr *recv_wr,
			 struct ib_recv_wr **bad_recv_wr);
#endif
	void (*drain_rq)(struct ib_qp *qp);
	void (*drain_sq)(struct ib_qp *qp);
	int (*poll_cq)(struct ib_cq *cq, int num_entries, struct ib_wc *wc);
	int (*peek_cq)(struct ib_cq *cq, int wc_cnt);
	int (*req_notify_cq)(struct ib_cq *cq, enum ib_cq_notify_flags flags);
	int (*req_ncomp_notif)(struct ib_cq *cq, int wc_cnt);
#ifdef POST_HAS_CONST
	int (*post_srq_recv)(struct ib_srq *srq,
			     const struct ib_recv_wr *recv_wr,
			     const struct ib_recv_wr **bad_recv_wr);
#else
	int (*post_srq_recv)(struct ib_srq *srq,
			     struct ib_recv_wr *recv_wr,
			     struct ib_recv_wr **bad_recv_wr);
#endif
	int (*process_mad)(struct ib_device *device, int process_mad_flags,
			   u8 port_num, const struct ib_wc *in_wc,
			   const struct ib_grh *in_grh,
			   const struct ib_mad_hdr *in_mad, size_t in_mad_size,
			   struct ib_mad_hdr *out_mad, size_t *out_mad_size,
			   u16 *out_mad_pkey_index);
	int (*query_device)(struct ib_device *device,
			    struct ib_device_attr *device_attr,
			    struct ib_udata *udata);
	int (*modify_device)(struct ib_device *device, int device_modify_mask,
			     struct ib_device_modify *device_modify);
#ifdef GET_DEV_FW_STR_HAS_LEN
	void (*get_dev_fw_str)(struct ib_device *device, char *str,
			       size_t str_len);
#else
	void (*get_dev_fw_str)(struct ib_device *device, char *str);
#endif
#ifdef HAVE_GET_VECTOR_AFFINITY
	const struct cpumask *(*get_vector_affinity)(struct ib_device *ibdev,
						     int comp_vector);
#endif
	int (*query_port)(struct ib_device *device, u8 port_num,
			  struct ib_port_attr *port_attr);
	int (*modify_port)(struct ib_device *device, u8 port_num,
			   int port_modify_mask,
			   struct ib_port_modify *port_modify);
	/**
	 * The following mandatory functions are used only at device
	 * registration.  Keep functions such as these at the end of this
	 * structure to avoid cache line misses when accessing struct ib_device
	 * in fast paths.
	 */
	int (*get_port_immutable)(struct ib_device *device, u8 port_num,
				  struct ib_port_immutable *immutable);
	enum rdma_link_layer (*get_link_layer)(struct ib_device *device,
					       u8 port_num);
	/**
	 * When calling get_netdev, the HW vendor's driver should return the
	 * net device of device @device at port @port_num or NULL if such
	 * a net device doesn't exist. The vendor driver should call dev_hold
	 * on this net device. The HW vendor's device driver must guarantee
	 * that this function returns NULL before the net device has finished
	 * NETDEV_UNREGISTER state.
	 */
	struct net_device *(*get_netdev)(struct ib_device *device, u8 port_num);
	/**
	 * rdma netdev operation
	 *
	 * Driver implementing alloc_rdma_netdev or rdma_netdev_get_params
	 * must return -EOPNOTSUPP if it doesn't support the specified type.
	 */
#ifdef HAVE_ALLOC_RDMA_NETDEV
	struct net_device *(*alloc_rdma_netdev)(
		struct ib_device *device, u8 port_num, enum rdma_netdev_t type,
		const char *name, unsigned char name_assign_type,
		void (*setup)(struct net_device *));
#endif

#ifdef HAVE_RDMA_NETDEV_GET_PARAMS
	int (*rdma_netdev_get_params)(struct ib_device *device, u8 port_num,
				      enum rdma_netdev_t type,
				      struct rdma_netdev_alloc_params *params);
#endif
	/**
	 * query_gid should be return GID value for @device, when @port_num
	 * link layer is either IB or iWarp. It is no-op if @port_num port
	 * is RoCE link layer.
	 */
	int (*query_gid)(struct ib_device *device, u8 port_num, int index,
			 union ib_gid *gid);
	/**
	 * When calling add_gid, the HW vendor's driver should add the gid
	 * of device of port at gid index available at @attr. Meta-info of
	 * that gid (for example, the network device related to this gid) is
	 * available at @attr. @context allows the HW vendor driver to store
	 * extra information together with a GID entry. The HW vendor driver may
	 * allocate memory to contain this information and store it in @context
	 * when a new GID entry is written to. Params are consistent until the
	 * next call of add_gid or delete_gid. The function should return 0 on
	 * success or error otherwise. The function could be called
	 * concurrently for different ports. This function is only called when
	 * roce_gid_table is used.
	 */
#ifdef HAVE_IB_GID_ATTR
#ifdef ADD_GID_HAS_GID
	int (*add_gid)(const union ib_gid *gid,
		       const struct ib_gid_attr *attr,
		       void **context);
#else
	int (*add_gid)(const struct ib_gid_attr *attr, void **context);
#endif
#else
	int (*add_gid)(struct ib_device *device, u8 port_num,
		       unsigned int index, const union ib_gid *gid,
		       const struct ib_gid_attr *attr, void **context);
#endif
	/**
	 * When calling del_gid, the HW vendor's driver should delete the
	 * gid of device @device at gid index gid_index of port port_num
	 * available in @attr.
	 * Upon the deletion of a GID entry, the HW vendor must free any
	 * allocated memory. The caller will clear @context afterwards.
	 * This function is only called when roce_gid_table is used.
	 */
#ifdef HAVE_IB_GID_ATTR
	int (*del_gid)(const struct ib_gid_attr *attr, void **context);
#else
	int  (*del_gid)(struct ib_device *device, u8 port_num,
			unsigned int index,
			void **context);
#endif
	int (*query_pkey)(struct ib_device *device, u8 port_num, u16 index,
			  u16 *pkey);
#ifdef ALLOC_UCONTEXT_RETURNS_INT
	int (*alloc_ucontext)(struct ib_ucontext *context,
			      struct ib_udata *udata);
#else
	struct ib_ucontext *(*alloc_ucontext)(struct ib_device *ibdev,
					      struct ib_udata *udata);
#endif
#ifdef DEALLOC_UCONTEXT_RETURNS_VOID
	void (*dealloc_ucontext)(struct ib_ucontext *context);
#else
	int (*dealloc_ucontext)(struct ib_ucontext *context);
#endif
	int (*mmap)(struct ib_ucontext *context, struct vm_area_struct *vma);
	void (*disassociate_ucontext)(struct ib_ucontext *ibcontext);
#ifdef ALLOC_PD_RETURN_INT
	int (*alloc_pd)(struct ib_pd *pd, struct ib_udata *udata);
#else
	struct ib_pd *(*alloc_pd)(struct ib_device *ibdev,
				  struct ib_ucontext *context,
				  struct ib_udata *udata);
#endif
#ifdef DEALLOC_PD_HAS_UDATA
	void (*dealloc_pd)(struct ib_pd *pd, struct ib_udata *udata);
#else
	int (*dealloc_pd)(struct ib_pd *pd);
#endif
#ifdef HAVE_CORE_ALLOC_AH
	int (*create_ah)(struct ib_ah *ah, struct rdma_ah_attr *ah_attr,
			 u32 flags, struct ib_udata *udata);
#elif defined(CREATE_AH_HAS_FLAGS)
	struct ib_ah *(*create_ah)(struct ib_pd *pd,
				   struct rdma_ah_attr *ah_attr,
				   u32 create_flags,
				   struct ib_udata *udata);
#elif defined(CREATE_AH_HAS_UDATA)
	struct ib_ah *(*create_ah)(struct ib_pd *pd,
				   struct rdma_ah_attr *ah_attr,
				   struct ib_udata *udata);
#else
	struct ib_ah *(*create_ah)(struct ib_pd *pd,
				   struct rdma_ah_attr *ah_attr);
#endif
	int (*modify_ah)(struct ib_ah *ah, struct rdma_ah_attr *ah_attr);
	int (*query_ah)(struct ib_ah *ah, struct rdma_ah_attr *ah_attr);
#ifdef HAVE_CORE_ALLOC_AH
	void (*destroy_ah)(struct ib_ah *ah, u32 flags);
#elif defined(DESTROY_AH_HAS_FLAGS)
	int (*destroy_ah)(struct ib_ah *ah, u32 flags);
#else
	int (*destroy_ah)(struct ib_ah *ah);
#endif
#ifdef HAVE_CORE_ALLOC_SRQ
	int (*create_srq)(struct ib_srq *srq,
			  struct ib_srq_init_attr *srq_init_attr,
			  struct ib_udata *udata);
#else
	struct ib_srq *(*create_srq)(struct ib_pd *ibpd,
				     struct ib_srq_init_attr *srq_init_attr,
		 		     struct ib_udata *udata);
#endif
	int (*modify_srq)(struct ib_srq *srq, struct ib_srq_attr *srq_attr,
			  enum ib_srq_attr_mask srq_attr_mask,
			  struct ib_udata *udata);
	int (*query_srq)(struct ib_srq *srq, struct ib_srq_attr *srq_attr);
#ifdef HAVE_CORE_ALLOC_SRQ
	void (*destroy_srq)(struct ib_srq *srq, struct ib_udata *udata);
#else
	int (*destroy_srq)(struct ib_srq *srq);
#endif
	struct ib_qp *(*create_qp)(struct ib_pd *pd,
				   struct ib_qp_init_attr *qp_init_attr,
				   struct ib_udata *udata);
	int (*modify_qp)(struct ib_qp *qp, struct ib_qp_attr *qp_attr,
			 int qp_attr_mask, struct ib_udata *udata);
	int (*query_qp)(struct ib_qp *qp, struct ib_qp_attr *qp_attr,
			int qp_attr_mask, struct ib_qp_init_attr *qp_init_attr);
#ifdef DESTROY_QP_HAS_UDATA
	int (*destroy_qp)(struct ib_qp *qp, struct ib_udata *udata);
#else
	int (*destroy_qp)(struct ib_qp *qp);
#endif
#ifdef HAVE_CORE_ALLOC_CQ
	int *(*create_cq)(struct ib_device *device,
			  const struct ib_cq_init_attr *attr,
			  struct ib_udata *udata);
#elif defined(NO_IB_UCONTEXT)
	struct ib_cq *(*create_cq)(struct ib_device *device,
				   const struct ib_cq_init_attr *attr,
				   struct ib_udata *udata);
#else
	struct ib_cq *(*create_cq)(struct ib_device *device,
				   const struct ib_cq_init_attr *attr,
				   struct ib_ucontext *context,
				   struct ib_udata *udata);
#endif
	int (*modify_cq)(struct ib_cq *cq, u16 cq_count, u16 cq_period);
#ifdef HAVE_CORE_ALLOC_CQ
	 void (*destroy_cq)(struct ib_cq *cq, struct ib_udata *udata);
#elif defined(DESTROY_CQ_HAS_UDATA)
	int (*destroy_cq)(struct ib_cq *cq, struct ib_udata *udata);
#else
	int (*destroy_cq)(struct ib_cq *cq);
#endif
	int (*resize_cq)(struct ib_cq *cq, int cqe, struct ib_udata *udata);
	struct ib_mr *(*get_dma_mr)(struct ib_pd *pd, int mr_access_flags);
	struct ib_mr *(*reg_user_mr)(struct ib_pd *pd, u64 start, u64 length,
				     u64 virt_addr, int mr_access_flags,
				     struct ib_udata *udata);
	int (*rereg_user_mr)(struct ib_mr *mr, int flags, u64 start, u64 length,
			     u64 virt_addr, int mr_access_flags,
			     struct ib_pd *pd, struct ib_udata *udata);
#ifdef DEREG_MR_HAS_UDATA
	int (*dereg_mr)(struct ib_mr *mr, struct ib_udata *udata);
#else
	int (*dereg_mr)(struct ib_mr *mr);
#endif
#ifdef ALLOC_MR_HAS_UDATA
	struct ib_mr *(*alloc_mr)(struct ib_pd *pd, enum ib_mr_type mr_type,
				  u32 max_num_sg, struct ib_udata *udata);
#else
	struct ib_mr *(*alloc_mr)(struct ib_pd *pd, enum ib_mr_type mr_type,
				  u32 max_num_sg);
#endif
	int (*advise_mr)(struct ib_pd *pd,
			 enum ib_uverbs_advise_mr_advice advice, u32 flags,
			 struct ib_sge *sg_list, u32 num_sge,
			 struct uverbs_attr_bundle *attrs);
	int (*map_mr_sg)(struct ib_mr *mr, struct scatterlist *sg, int sg_nents,
			 unsigned int *sg_offset);
	int (*check_mr_status)(struct ib_mr *mr, u32 check_mask,
			       struct ib_mr_status *mr_status);
	struct ib_mw *(*alloc_mw)(struct ib_pd *pd, enum ib_mw_type type,
				  struct ib_udata *udata);
	int (*dealloc_mw)(struct ib_mw *mw);
	struct ib_fmr *(*alloc_fmr)(struct ib_pd *pd, int mr_access_flags,
				    struct ib_fmr_attr *fmr_attr);
	int (*map_phys_fmr)(struct ib_fmr *fmr, u64 *page_list, int list_len,
			    u64 iova);
	int (*unmap_fmr)(struct list_head *fmr_list);
	int (*dealloc_fmr)(struct ib_fmr *fmr);
	int (*attach_mcast)(struct ib_qp *qp, union ib_gid *gid, u16 lid);
	int (*detach_mcast)(struct ib_qp *qp, union ib_gid *gid, u16 lid);
#ifdef HAVE_ALLOC_XRCD
	struct ib_xrcd *(*alloc_xrcd)(struct ib_device *device,
				      struct ib_udata *udata);
#ifdef DEALLOC_XRCD_HAS_UDATA
	int (*dealloc_xrcd)(struct ib_xrcd *xrcd, struct ib_udata *udata);
#else
	int (*dealloc_xrcd)(struct ib_xrcd *xrcd);
#endif
#endif
#ifdef CREATE_FLOW_HAS_UDATA
	struct ib_flow *(*create_flow)(struct ib_qp *qp,
				       struct ib_flow_attr *flow_attr,
				       int domain, struct ib_udata *udata);
#else
	struct ib_flow *(*create_flow)(struct ib_qp *qp,
				       struct ib_flow_attr *flow_attr,
				       int domain);
#endif
	int (*destroy_flow)(struct ib_flow *flow_id);
	struct ib_flow_action *(*create_flow_action_esp)(
		struct ib_device *device,
		const struct ib_flow_action_attrs_esp *attr,
		struct uverbs_attr_bundle *attrs);
	int (*destroy_flow_action)(struct ib_flow_action *action);
	int (*modify_flow_action_esp)(
		struct ib_flow_action *action,
		const struct ib_flow_action_attrs_esp *attr,
		struct uverbs_attr_bundle *attrs);
	int (*set_vf_link_state)(struct ib_device *device, int vf, u8 port,
				 int state);
	int (*get_vf_config)(struct ib_device *device, int vf, u8 port,
			     struct ifla_vf_info *ivf);
	int (*get_vf_stats)(struct ib_device *device, int vf, u8 port,
			    struct ifla_vf_stats *stats);
	int (*set_vf_guid)(struct ib_device *device, int vf, u8 port, u64 guid,
			   int type);
	struct ib_wq *(*create_wq)(struct ib_pd *pd,
				   struct ib_wq_init_attr *init_attr,
				   struct ib_udata *udata);
#ifdef DESTROY_WQ_HAS_UDATA
	int (*destroy_wq)(struct ib_wq *wq, struct ib_udata *udata);
#else
	int (*destroy_wq)(struct ib_wq *wq);
#endif
	int (*modify_wq)(struct ib_wq *wq, struct ib_wq_attr *attr,
			 u32 wq_attr_mask, struct ib_udata *udata);
	struct ib_rwq_ind_table *(*create_rwq_ind_table)(
		struct ib_device *device,
		struct ib_rwq_ind_table_init_attr *init_attr,
		struct ib_udata *udata);
	int (*destroy_rwq_ind_table)(struct ib_rwq_ind_table *wq_ind_table);
	struct ib_dm *(*alloc_dm)(struct ib_device *device,
				  struct ib_ucontext *context,
				  struct ib_dm_alloc_attr *attr,
				  struct uverbs_attr_bundle *attrs);
	int (*dealloc_dm)(struct ib_dm *dm, struct uverbs_attr_bundle *attrs);
	struct ib_mr *(*reg_dm_mr)(struct ib_pd *pd, struct ib_dm *dm,
				   struct ib_dm_mr_attr *attr,
				   struct uverbs_attr_bundle *attrs);
	struct ib_counters *(*create_counters)(
		struct ib_device *device, struct uverbs_attr_bundle *attrs);
	int (*destroy_counters)(struct ib_counters *counters);
	int (*read_counters)(struct ib_counters *counters,
			     struct ib_counters_read_attr *counters_read_attr,
			     struct uverbs_attr_bundle *attrs);
	/**
	 * alloc_hw_stats - Allocate a struct rdma_hw_stats and fill in the
	 *   driver initialized data.  The struct is kfree()'ed by the sysfs
	 *   core when the device is removed.  A lifespan of -1 in the return
	 *   struct tells the core to set a default lifespan.
	 */
	struct rdma_hw_stats *(*alloc_hw_stats)(struct ib_device *device,
						u8 port_num);
	/**
	 * get_hw_stats - Fill in the counter value(s) in the stats struct.
	 * @index - The index in the value array we wish to have updated, or
	 *   num_counters if we want all stats updated
	 * Return codes -
	 *   < 0 - Error, no counters updated
	 *   index - Updated the single counter pointed to by index
	 *   num_counters - Updated all counters (will reset the timestamp
	 *     and prevent further calls for lifespan milliseconds)
	 * Drivers are allowed to update all counters in leiu of just the
	 *   one given in index at their option
	 */
	int (*get_hw_stats)(struct ib_device *device,
			    struct rdma_hw_stats *stats, u8 port, int index);
	/*
	 * This function is called once for each port when a ib device is
	 * registered.
	 */
	int (*init_port)(struct ib_device *device, u8 port_num,
			 struct kobject *port_sysfs);
	/**
	 * Allows rdma drivers to add their own restrack attributes.
	 */
	int (*fill_res_entry)(struct sk_buff *msg,
			      struct rdma_restrack_entry *entry);

	/* Device lifecycle callbacks */
	/*
	 * Called after the device becomes registered, before clients are
	 * attached
	 */
	int (*enable_driver)(struct ib_device *dev);
	/*
	 * This is called as part of ib_dealloc_device().
	 */
	void (*dealloc_driver)(struct ib_device *dev);

	/* iWarp CM callbacks */
	void (*iw_add_ref)(struct ib_qp *qp);
	void (*iw_rem_ref)(struct ib_qp *qp);
	struct ib_qp *(*iw_get_qp)(struct ib_device *device, int qpn);
	int (*iw_connect)(struct iw_cm_id *cm_id,
			  struct iw_cm_conn_param *conn_param);
	int (*iw_accept)(struct iw_cm_id *cm_id,
			 struct iw_cm_conn_param *conn_param);
	int (*iw_reject)(struct iw_cm_id *cm_id, const void *pdata,
			 u8 pdata_len);
	int (*iw_create_listen)(struct iw_cm_id *cm_id, int backlog);
	int (*iw_destroy_listen)(struct iw_cm_id *cm_id);

};
#endif

#ifndef HAVE_IB_SET_DEVICE_OPS
void ib_set_device_ops(struct ib_device *dev, const struct ib_device_ops *ops);
#endif

#if !defined(HAVE_ARRAY_SIZE) && !defined(check_mul_overflow)
#define is_signed_type(type)       (((type)(-1)) < (type)1)
#define __type_half_max(type) ((type)1 << (8*sizeof(type) - 1 - is_signed_type(type)))
#define type_max(T) ((T)((__type_half_max(T) - 1) + __type_half_max(T)))
#define type_min(T) ((T)((T)-type_max(T)-(T)1))

#define __unsigned_mul_overflow(a, b, d) ({		\
	typeof(a) __a = (a);				\
	typeof(b) __b = (b);				\
	typeof(d) __d = (d);				\
	(void) (&__a == &__b);				\
	(void) (&__a == __d);				\
	*__d = __a * __b;				\
	__builtin_constant_p(__b) ?			\
	  __b > 0 && __a > type_max(typeof(__a)) / __b : \
	  __a > 0 && __b > type_max(typeof(__b)) / __a;	 \
})

/* Checking for unsigned overflow is relatively easy without causing UB. */
#define __unsigned_add_overflow(a, b, d) ({     \
	typeof(a) __a = (a);                    \
	typeof(b) __b = (b);                    \
	typeof(d) __d = (d);                    \
	(void) (&__a == &__b);                  \
	(void) (&__a == __d);                   \
	*__d = __a + __b;                       \
	*__d < __a;                             \
})

#define check_mul_overflow(a, b, d) __unsigned_mul_overflow(a, b, d)
#define check_add_overflow(a, b, d) __unsigned_add_overflow(a, b, d)

static inline size_t array_size(size_t a, size_t b)
{
	size_t bytes;

	if(check_mul_overflow(a, b, &bytes))
		return SIZE_MAX;

	return bytes;
}

#ifndef __must_be_array
#define __must_be_array(a)      BUILD_BUG_ON_ZERO(__same_type((a), &(a)[0]))
#endif

/*
 * Compute a*b+c, returning SIZE_MAX on overflow. Internal helper for
 * struct_size() below.
 */
static inline __must_check size_t __ab_c_size(size_t a, size_t b, size_t c)
{
	size_t bytes;

	if (check_mul_overflow(a, b, &bytes))
		return SIZE_MAX;
	if (check_add_overflow(bytes, c, &bytes))
		return SIZE_MAX;

	return bytes;
}

/**
 * struct_size() - Calculate size of structure with trailing array.
 * @p: Pointer to the structure.
 * @member: Name of the array member.
 * @n: Number of elements in the array.
 *
 * Calculates size of memory needed for structure @p followed by an
 * array of @n @member elements.
 *
 * Return: number of bytes needed or SIZE_MAX on overflow.
 */
#define struct_size(p, member, n)					\
	__ab_c_size(n,							\
		    sizeof(*(p)->member) + __must_be_array((p)->member),\
		    sizeof(*(p)))

#endif

#ifndef PCI_EXP_LNKCAP_SLS_8_0GB
#define PCI_EXP_LNKCAP_SLS_8_0GB 0x00000003
#endif
#ifndef PCI_EXP_LNKCTL2_TLS_8_0GT
#define  PCI_EXP_LNKCTL2_TLS_8_0GT      0x0003 /* Supported Speed 8GT/s */
#endif
#ifndef PCI_EXP_LNKCTL2_TLS_2_5GT
#define PCI_EXP_LNKCTL2_TLS_2_5GT      0x0001 /* Supported Speed 2.5GT/s */
#endif
#ifndef PCI_EXP_LNKCTL2_TLS_5_0GT
#define PCI_EXP_LNKCTL2_TLS_5_0GT 0x0002
#endif
#ifndef PCI_EXP_LNKCTL2_TLS
#define PCI_EXP_LNKCTL2_TLS 0x000f
#endif

#ifndef u64_to_user_ptr
#define u64_to_user_ptr(x) (            \
{                                       \
	typecheck(u64, (x));            \
	(void __user *)(uintptr_t)(x);  \
}                                       \
)
#endif

#ifndef HAVE_VM_FAULT_T
typedef int vm_fault_t;
#endif

#ifndef CREATE_AH_HAS_FLAGS
enum rdma_create_ah_flags {
	/* In a sleepable context */
	RDMA_CREATE_AH_SLEEPABLE = BIT(0),
};
#endif

#if !defined(DESTROY_AH_HAS_FLAGS) && !defined(DESTROY_AH_RETURN_VOID)
enum rdma_destroy_ah_flags {
	/* In a sleepable context */
	RDMA_DESTROY_AH_SLEEPABLE = BIT(0),
};
#endif

#ifdef IB_MODIFY_QP_IS_OK_HAS_LINK
#define ib_modify_qp_is_ok(a,b,c,d) ib_modify_qp_is_ok(a,b,c,d,IB_LINK_LAYER_INFINIBAND)
#endif

#ifndef HAVE_RDMA_DEVICE_TO_IBDEV
/**
 * rdma_device_to_ibdev - Get ib_device pointer from device pointer
 *
 * @device:	device pointer for which ib_device pointer to retrieve
 *
 * rdma_device_to_ibdev() retrieves ib_device pointer from device.
 *
 */
static inline struct ib_device *rdma_device_to_ibdev(struct device *device)
{
	return container_of(device, struct ib_device, dev);
}

/**
 * rdma_device_to_drv_device - Helper macro to reach back to driver's
 *			       ib_device holder structure from device pointer.
 *
 * NOTE: New drivers should not make use of this API; This API is only for
 * existing drivers who have exposed sysfs entries using
 * rdma_set_device_sysfs_group().
 */
#define rdma_device_to_drv_device(dev, drv_dev_struct, ibdev_member)           \
	container_of(rdma_device_to_ibdev(dev), drv_dev_struct, ibdev_member)
#endif

#ifdef NEED_POLL_T
typedef unsigned __bitwise __poll_t;
#endif

#ifndef EPOLLERR
#define EPOLLERR POLLERR
#endif

#ifndef EPOLLRDNORM
#define EPOLLRDNORM POLLRDNORM
#endif

#ifndef EPOLLIN
#define EPOLLIN POLLIN
#endif

#ifndef HAVE_RDMA_COPY_AH_ATTR
static inline
void rdma_destroy_ah_attr(struct rdma_ah_attr *ah)
{
}

static inline
void rdma_copy_ah_attr(struct rdma_ah_attr *dest,
		       const struct rdma_ah_attr *src)
{
	*dest = *src;
}

static inline
void rdma_replace_ah_attr(struct rdma_ah_attr *old,
                          const struct rdma_ah_attr *new)
{
	*old = *new;
}

static inline
void rdma_move_ah_attr(struct rdma_ah_attr *dest, struct rdma_ah_attr *src)
{
	*dest = *src;
}
#endif

#ifndef HAVE_MMU_NOTIFIER_RANGE
struct mmu_notifier_range {
	struct mm_struct *mm;
	unsigned long start;
	unsigned long end;
};
#endif

#ifndef HAVE_PUT_USER_PAGES
static inline void put_user_page(struct page *p)
{
	put_page(p);
}

static inline void put_user_pages(struct page **p, unsigned long n)
{
	unsigned long i;

	for (i = 0; i < n; i++)
		put_user_page(p[i]);
}

static inline void put_user_pages_dirty_lock(struct page **p, unsigned long n)
{
	unsigned long i;

	for (i = 0; i < n; i++) {
		set_page_dirty_lock(p[i]);
		put_user_page(p[i]);
	}
}
#endif

#ifndef HAVE_UMEM_PTR_VALIDITY_CHECK
static inline void compat_ib_umem_release(struct ib_umem *umem)
{
	if (umem)
		ib_umem_release(umem);
}

#define ib_umem_release compat_ib_umem_release
#endif

#ifndef DESTROY_CQ_HAS_UDATA
#define rvt_destroy_cq(a,b) rvt_destroy_cq(a)
#define compat_rvt_destroy_cq(a,b) compat_rvt_destroy_cq(a)
#endif
#ifndef DEREG_MR_HAS_UDATA
#define rvt_dereg_mr(a,b) rvt_dereg_mr(a)
#endif
#ifndef DESTROY_QP_HAS_UDATA
#define rvt_destroy_qp(a,b) rvt_destroy_qp(a)
#endif
#ifndef HAVE_CORE_ALLOC_SRQ
#define compat_rvt_destroy_srq(a,b) compat_rvt_destroy_srq(a)
#define rvt_destroy_srq(a,b) rvt_destroy_srq(a)
#endif
#ifndef ALLOC_MR_HAS_UDATA
#define rvt_alloc_mr(a,b,c,d) rvt_alloc_mr(a,b,c)
#endif
#ifndef DEALLOC_PD_HAS_UDATA
#define rvt_dealloc_pd(a,b) rvt_dealloc_pd(a)
#endif

#ifndef IB_DEVICE_OPS_DRIVER_ID
#ifdef QIB_DRIVER
#define rvt_register_device(a) compat_rvt_register_device(a, RDMA_DRIVER_QIB)
#else
#define rvt_register_device(a) compat_rvt_register_device(a, RDMA_DRIVER_HFI1)
#endif
#endif

#ifndef HAVE_NETDEV_XMIT_MORE
#define netdev_xmit_more() (skb->xmit_more)
#endif

#ifndef HAVE_NEW_PUT_USER_PAGES_DIRTY_LOCK
#define put_user_pages_dirty_lock(p, npages, dirty) \
	if (dirty) \
		put_user_pages_dirty_lock(p, npages); \
	else \
		put_user_pages(p, npages)
#endif

#ifndef HAVE_SKB_FRAG_OFF
static inline unsigned int skb_frag_off(const skb_frag_t *frag)
{
	return frag->page_offset;
}
#endif

#endif
