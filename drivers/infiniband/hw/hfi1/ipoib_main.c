/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
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

/*
 * This file contains HFI1 support for IPOIB functionality
 */

#include <linux/module.h>
#include "ipoib.h"
#include "hfi.h"
#include "netdev.h"

#ifdef AIP
uint ipoib_accel = 1;
module_param(ipoib_accel, uint, 0644);
MODULE_PARM_DESC(ipoib_accel, "Accelerated ipoib mode");
#else
uint ipoib_accel = 0;
#endif
EXPORT_SYMBOL(ipoib_accel);

static u32 qpn_from_mac(u8 *mac_arr)
{
	return (u32)mac_arr[1] << 16 | mac_arr[2] << 8 | mac_arr[3];
}

static int hfi1_ipoib_dev_init(struct net_device *dev)
{
	struct hfi1_ipoib_dev_priv *priv = hfi1_ipoib_priv(dev);
	int ret;

	priv->netstats = netdev_alloc_pcpu_stats(struct pcpu_sw_netstats);

	ret = priv->netdev_ops->ndo_init(dev);
	if (ret)
		return ret;

	ret = hfi1_netdev_add_data(priv->dd,
				   qpn_from_mac(priv->netdev->dev_addr),
				   dev);
	if (ret < 0) {
		priv->netdev_ops->ndo_uninit(dev);
		return ret;
	}

	return 0;
}

static void hfi1_ipoib_dev_uninit(struct net_device *dev)
{
	struct hfi1_ipoib_dev_priv *priv = hfi1_ipoib_priv(dev);

	hfi1_netdev_remove_data(priv->dd, qpn_from_mac(priv->netdev->dev_addr));

	priv->netdev_ops->ndo_uninit(dev);
}

static int hfi1_ipoib_dev_open(struct net_device *dev)
{
	struct hfi1_ipoib_dev_priv *priv = hfi1_ipoib_priv(dev);
	int ret;

	ret = priv->netdev_ops->ndo_open(dev);
	if (!ret) {
		struct hfi1_ibport *ibp = to_iport(priv->device,
						   priv->port_num);
		struct rvt_qp *qp;
		u32 qpn = qpn_from_mac(priv->netdev->dev_addr);

		rcu_read_lock();
		qp = rvt_lookup_qpn(ib_to_rvt(priv->device), &ibp->rvp, qpn);
		rvt_get_qp(qp);
		priv->qp = qp;
		rcu_read_unlock();

		hfi1_netdev_enable_queues(priv->dd);
		hfi1_ipoib_napi_tx_enable(dev);
	}

	return ret;
}

static int hfi1_ipoib_dev_stop(struct net_device *dev)
{
	struct hfi1_ipoib_dev_priv *priv = hfi1_ipoib_priv(dev);

	if (!priv->qp)
		return 0;

	hfi1_ipoib_napi_tx_disable(dev);
	hfi1_netdev_disable_queues(priv->dd);

	rvt_put_qp(priv->qp);
	priv->qp = NULL;

	return priv->netdev_ops->ndo_stop(dev);
}

static void hfi1_ipoib_dev_get_stats64(struct net_device *dev,
				       struct rtnl_link_stats64 *storage)
{
	struct hfi1_ipoib_dev_priv *priv = hfi1_ipoib_priv(dev);
	u64 rx_packets = 0ull;
	u64 rx_bytes = 0ull;
	u64 tx_packets = 0ull;
	u64 tx_bytes = 0ull;
	int i;

	netdev_stats_to_stats64(storage, &dev->stats);

	for_each_possible_cpu(i) {
		const struct pcpu_sw_netstats *stats;
		unsigned int start;
		u64 trx_packets;
		u64 trx_bytes;
		u64 ttx_packets;
		u64 ttx_bytes;

		stats = per_cpu_ptr(priv->netstats, i);
		do {
			start = u64_stats_fetch_begin_irq(&stats->syncp);
			trx_packets = stats->rx_packets;
			trx_bytes = stats->rx_bytes;
			ttx_packets = stats->tx_packets;
			ttx_bytes = stats->tx_bytes;
		} while (u64_stats_fetch_retry_irq(&stats->syncp, start));

		rx_packets += trx_packets;
		rx_bytes += trx_bytes;
		tx_packets += ttx_packets;
		tx_bytes += ttx_bytes;
	}

	storage->rx_packets += rx_packets;
	storage->rx_bytes += rx_bytes;
	storage->tx_packets += tx_packets;
	storage->tx_bytes += tx_bytes;
}

static const struct net_device_ops hfi1_ipoib_netdev_ops = {
	.ndo_init         = hfi1_ipoib_dev_init,
	.ndo_uninit       = hfi1_ipoib_dev_uninit,
	.ndo_open         = hfi1_ipoib_dev_open,
	.ndo_stop         = hfi1_ipoib_dev_stop,
	.ndo_get_stats64  = hfi1_ipoib_dev_get_stats64,
};

static int hfi1_ipoib_mcast_attach(struct net_device *dev,
				   struct ib_device *device,
				   union ib_gid *mgid,
				   u16 mlid,
				   int set_qkey,
				   u32 qkey)
{
	struct hfi1_ipoib_dev_priv *priv = hfi1_ipoib_priv(dev);
	u32 qpn = (u32)qpn_from_mac(priv->netdev->dev_addr);
	struct hfi1_ibport *ibp = to_iport(priv->device, priv->port_num);
	struct rvt_qp *qp;
	int ret = -EINVAL;

	rcu_read_lock();

	qp = rvt_lookup_qpn(ib_to_rvt(priv->device), &ibp->rvp, qpn);
	if (qp) {
		rvt_get_qp(qp);
		rcu_read_unlock();
		if (set_qkey)
			priv->qkey = qkey;

		/* attach QP to multicast group */
		ret = ib_attach_mcast(&qp->ibqp, mgid, mlid);
		rvt_put_qp(qp);
	} else {
		rcu_read_unlock();
	}

	return ret;
}

static int hfi1_ipoib_mcast_detach(struct net_device *dev,
				   struct ib_device *device,
				   union ib_gid *mgid,
				   u16 mlid)
{
	struct hfi1_ipoib_dev_priv *priv = hfi1_ipoib_priv(dev);
	u32 qpn = (u32)qpn_from_mac(priv->netdev->dev_addr);
	struct hfi1_ibport *ibp = to_iport(priv->device, priv->port_num);
	struct rvt_qp *qp;
	int ret = -EINVAL;

	rcu_read_lock();

	qp = rvt_lookup_qpn(ib_to_rvt(priv->device), &ibp->rvp, qpn);
	if (qp) {
		rvt_get_qp(qp);
		rcu_read_unlock();
		ret = ib_detach_mcast(&qp->ibqp, mgid, mlid);
		rvt_put_qp(qp);
	} else {
		rcu_read_unlock();
	}
	return ret;
}

static void hfi1_ipoib_netdev_dtor(struct net_device *dev)
{
	struct hfi1_ipoib_dev_priv *priv = hfi1_ipoib_priv(dev);

	hfi1_ipoib_txreq_deinit(priv);
	hfi1_ipoib_rxq_deinit(priv->netdev);

	free_percpu(priv->netstats);
}

static void hfi1_ipoib_free_rdma_netdev(struct net_device *dev)
{
	hfi1_ipoib_netdev_dtor(dev);
	free_netdev(dev);
}

static void hfi1_ipoib_set_id(struct net_device *dev, int id)
{
	struct hfi1_ipoib_dev_priv *priv = hfi1_ipoib_priv(dev);

	priv->pkey_index = (u16)id;
	ib_query_pkey(priv->device,
		      priv->port_num,
		      priv->pkey_index,
		      &priv->pkey);
}

struct net_device *hfi1_ipoib_alloc_rn(struct ib_device *device,
				       u8 port_num,
				       enum rdma_netdev_t type,
				       const char *name,
				       unsigned char name_assign_type,
				       void (*setup)(struct net_device *))
{
	struct hfi1_devdata *dd = dd_from_ibdev(device);
	struct net_device *dev;
	struct rdma_netdev *rn;
	struct hfi1_ipoib_dev_priv *priv;
	int rc;

	if (!ipoib_accel)
		return ERR_PTR(-EOPNOTSUPP);

	if (!port_num || port_num > dd->num_pports)
		return ERR_PTR(-EINVAL);

	dev = alloc_netdev_mqs((int)sizeof(struct hfi1_ipoib_rdma_netdev),
			       name,
			       name_assign_type,
			       setup,
			       dd->num_sdma,
			       dd->num_netdev_contexts);
	if (!dev)
		return ERR_PTR(-ENOMEM);

	if (!dev->netdev_ops) {
		free_netdev(dev);
		return ERR_PTR(-EOPNOTSUPP);
	}

	if (!dev->netdev_ops->ndo_init || !dev->netdev_ops->ndo_uninit ||
	    !dev->netdev_ops->ndo_open || !dev->netdev_ops->ndo_stop) {
		free_netdev(dev);
		return ERR_PTR(-EOPNOTSUPP);
	}

	rn = netdev_priv(dev);

	rn->send = hfi1_ipoib_send;
	rn->attach_mcast = hfi1_ipoib_mcast_attach;
	rn->detach_mcast = hfi1_ipoib_mcast_detach;
	rn->set_id = hfi1_ipoib_set_id;
	rn->free_rdma_netdev = hfi1_ipoib_free_rdma_netdev;
	rn->hca = device;
	rn->port_num = port_num;
	rn->mtu = dev->mtu;

	priv = hfi1_ipoib_priv(dev);
	priv->dd = dd;
	priv->netdev = dev;
	priv->device = device;
	priv->port_num = port_num;
	priv->netdev_ops = dev->netdev_ops;

	dev->netdev_ops = &hfi1_ipoib_netdev_ops;

	ib_query_pkey(device, port_num, priv->pkey_index, &priv->pkey);

	rc = hfi1_ipoib_txreq_init(priv);
	if (rc) {
		dd_dev_err(dd, "IPoIB netdev TX init - failed(%d)\n", rc);
		hfi1_ipoib_free_rdma_netdev(dev);
		return ERR_PTR(rc);
	}

	rc = hfi1_ipoib_rxq_init(dev);
	if (rc) {
		dd_dev_err(dd, "IPoIB netdev RX init - failed(%d)\n", rc);
		hfi1_ipoib_free_rdma_netdev(dev);
		return ERR_PTR(rc);
	}
	return dev;
}

#ifdef HAVE_RDMA_NETDEV_GET_PARAMS
static int hfi1_ipoib_setup_rn(struct ib_device *device,
			       u8 port_num,
			       struct net_device *netdev,
			       void *param)
{
	struct hfi1_devdata *dd = dd_from_ibdev(device);
	struct rdma_netdev *rn = netdev_priv(netdev);
	struct hfi1_ipoib_dev_priv *priv;
	int rc;

	rn->send = hfi1_ipoib_send;
	rn->attach_mcast = hfi1_ipoib_mcast_attach;
	rn->detach_mcast = hfi1_ipoib_mcast_detach;
	rn->set_id = hfi1_ipoib_set_id;
	rn->hca = device;
	rn->port_num = port_num;
	rn->mtu = netdev->mtu;

	priv = hfi1_ipoib_priv(netdev);
	priv->dd = dd;
	priv->netdev = netdev;
	priv->device = device;
	priv->port_num = port_num;
	priv->netdev_ops = netdev->netdev_ops;

	netdev->netdev_ops = &hfi1_ipoib_netdev_ops;

	ib_query_pkey(device, port_num, priv->pkey_index, &priv->pkey);

	rc = hfi1_ipoib_txreq_init(priv);
	if (rc) {
		dd_dev_err(dd, "IPoIB netdev TX init - failed(%d)\n", rc);
		hfi1_ipoib_free_rdma_netdev(netdev);
		return rc;
	}

	rc = hfi1_ipoib_rxq_init(netdev);
	if (rc) {
		dd_dev_err(dd, "IPoIB netdev RX init - failed(%d)\n", rc);
		hfi1_ipoib_free_rdma_netdev(netdev);
		return rc;
	}

#ifdef HAVE_NET_DEVICE_EXTENDED
	netdev->extended->priv_destructor = hfi1_ipoib_netdev_dtor;
	netdev->extended->needs_free_netdev = true;
#else
	netdev->priv_destructor = hfi1_ipoib_netdev_dtor;
	netdev->needs_free_netdev = true;
#endif

	return 0;
}

int hfi1_ipoib_rn_get_params(struct ib_device *device,
			     u8 port_num,
			     enum rdma_netdev_t type,
			     struct rdma_netdev_alloc_params *params)
{
	struct hfi1_devdata *dd = dd_from_ibdev(device);

	if (type != RDMA_NETDEV_IPOIB)
		return -EOPNOTSUPP;

	if (!ipoib_accel || !dd->num_netdev_contexts)
		return -EOPNOTSUPP;

	if (!port_num || port_num > dd->num_pports)
		return -EINVAL;

	params->sizeof_priv = sizeof(struct hfi1_ipoib_rdma_netdev);
	params->txqs = dd->num_sdma;
	params->rxqs = dd->num_netdev_contexts;
	params->param = NULL;
	params->initialize_rdma_netdev = hfi1_ipoib_setup_rn;

	return 0;
}
#endif
