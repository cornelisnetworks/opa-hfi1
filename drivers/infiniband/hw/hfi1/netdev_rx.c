// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
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
 * This file contains HFI1 support for netdev RX functionality
 */

#include "sdma.h"
#include "verbs.h"
#include "netdev.h"
#include "hfi.h"
#include "compat_common.h"

#include <linux/atomic.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <rdma/ib_verbs.h>

static int hfi1_netdev_setup_ctxt(struct hfi1_netdev_priv *priv,
				  struct hfi1_ctxtdata *uctxt)
{
	unsigned int rcvctrl_ops = 0;
	struct hfi1_devdata *dd = priv->dd;
	int ret;

	uctxt->rhf_rcv_function_map = netdev_rhf_rcv_functions;
	uctxt->do_interrupt = &handle_receive_interrupt_napi_sp;

	/* Now allocate the RcvHdr queue and eager buffers. */
	ret = hfi1_create_rcvhdrq(dd, uctxt);
	if (ret)
		goto done;

	ret = hfi1_setup_eagerbufs(uctxt);
	if (ret)
		goto done;

	clear_rcvhdrtail(uctxt);

	if (!HFI1_CAP_KGET_MASK(uctxt->flags, MULTI_PKT_EGR))
		rcvctrl_ops |= HFI1_RCVCTRL_ONE_PKT_EGR_ENB;
	if (HFI1_CAP_KGET_MASK(uctxt->flags, NODROP_EGR_FULL))
		rcvctrl_ops |= HFI1_RCVCTRL_NO_EGR_DROP_ENB;
	if (HFI1_CAP_KGET_MASK(uctxt->flags, NODROP_RHQ_FULL))
		rcvctrl_ops |= HFI1_RCVCTRL_NO_RHQ_DROP_ENB;
	if (HFI1_CAP_KGET_MASK(uctxt->flags, DMA_RTAIL))
		rcvctrl_ops |= HFI1_RCVCTRL_TAILUPD_ENB;

	hfi1_rcvctrl(uctxt->dd, rcvctrl_ops, uctxt);
done:
	return ret;
}

static int hfi1_netdev_allocate_ctxt(struct hfi1_devdata *dd,
				     struct hfi1_ctxtdata **ctxt)
{
	struct hfi1_ctxtdata *uctxt;
	int ret;

	if (dd->flags & HFI1_FROZEN)
		return -EIO;

	ret = hfi1_create_ctxtdata(dd->pport, dd->node, &uctxt);
	if (ret < 0) {
		dd_dev_err(dd, "Unable to create ctxtdata, failing open\n");
		return -ENOMEM;
	}

	uctxt->flags = HFI1_CAP_KGET(MULTI_PKT_EGR) |
		HFI1_CAP_KGET(NODROP_RHQ_FULL) |
		HFI1_CAP_KGET(NODROP_EGR_FULL) |
		HFI1_CAP_KGET(DMA_RTAIL);
	/* Netdev contexts are always NO_RDMA_RTAIL */
	uctxt->fast_handler = handle_receive_interrupt_napi_fp;
	uctxt->slow_handler = handle_receive_interrupt_napi_sp;
	hfi1_set_seq_cnt(uctxt, 1);
	uctxt->is_vnic = true;

	msix_netdev_request_rcd_irq(uctxt);

	hfi1_stats.sps_ctxts++;

	dd_dev_info(dd, "created netdev context %d\n", uctxt->ctxt);
	*ctxt = uctxt;

	return 0;
}

static void hfi1_netdev_deallocate_ctxt(struct hfi1_devdata *dd,
					struct hfi1_ctxtdata *uctxt)
{
	flush_wc();

	/*
	 * Disable receive context and interrupt available, reset all
	 * RcvCtxtCtrl bits to default values.
	 */
	hfi1_rcvctrl(dd, HFI1_RCVCTRL_CTXT_DIS |
		     HFI1_RCVCTRL_TIDFLOW_DIS |
		     HFI1_RCVCTRL_INTRAVAIL_DIS |
		     HFI1_RCVCTRL_ONE_PKT_EGR_DIS |
		     HFI1_RCVCTRL_NO_RHQ_DROP_DIS |
		     HFI1_RCVCTRL_NO_EGR_DROP_DIS, uctxt);

	msix_free_irq(dd, uctxt->msix_intr);

	uctxt->msix_intr = CCE_NUM_MSIX_VECTORS;
	uctxt->event_flags = 0;

	hfi1_clear_tids(uctxt);
	hfi1_clear_ctxt_pkey(dd, uctxt);

	hfi1_stats.sps_ctxts--;

	hfi1_free_ctxt(uctxt);
}

static int hfi1_netdev_allot_ctxt(struct hfi1_netdev_priv *priv,
				  struct hfi1_ctxtdata **ctxt)
{
	int rc;
	struct hfi1_devdata *dd = priv->dd;

	rc = hfi1_netdev_allocate_ctxt(dd, ctxt);
	if (rc) {
		dd_dev_err(dd, "netdev ctxt alloc failed %d\n", rc);
		return rc;
	}

	rc = hfi1_netdev_setup_ctxt(priv, *ctxt);
	if (rc) {
		dd_dev_err(dd, "netdev ctxt setup failed %d\n", rc);
		hfi1_netdev_deallocate_ctxt(dd, *ctxt);
		*ctxt = NULL;
	}

	return rc;
}

static int hfi1_netdev_rxq_init(struct net_device *dev)
{
	int i;
	struct hfi1_netdev_priv *priv = hfi1_netdev_priv(dev);
	struct hfi1_devdata *dd = priv->dd;

	priv->num_rx_q = dd->num_netdev_contexts;
	priv->rxq = kcalloc_node(priv->num_rx_q, sizeof(struct hfi1_netdev_rxq),
				 GFP_KERNEL, dd->node);

	if (!priv->rxq) {
		dd_dev_err(dd, "Unable to allocate netdev queue data\n");
		return (-ENOMEM);
	}

	for (i = 0; i < priv->num_rx_q; i++) {
		struct hfi1_netdev_rxq *rxq = &priv->rxq[i];

		if (hfi1_netdev_allot_ctxt(priv, &rxq->rcd)) {
			int j;

			dd_dev_err(dd, "Unable to allot receive context\n");
			for (j = 0; j < i; j++) {
				rxq = &priv->rxq[j];
				hfi1_netdev_deallocate_ctxt(dd, rxq->rcd);
				hfi1_rcd_put(rxq->rcd);
				rxq->rcd = NULL;
			}
			kfree(priv->rxq);
			priv->rxq = NULL;
			return (-ENOMEM);
		}
		hfi1_rcd_get(rxq->rcd);
	}

	for (i = 0; i < priv->num_rx_q; i++) {
		struct hfi1_netdev_rxq *rxq = &priv->rxq[i];

		rxq->priv = priv;
		rxq->rcd->napi = &rxq->napi;
		dd_dev_info(dd, "Setting rcv queue %d napi to context %d\n",
			    i, rxq->rcd->ctxt);
		/*
		 * Disable BUSY_POLL on this NAPI as this is not supported
		 * right now.
		 */
		set_bit(NAPI_STATE_NO_BUSY_POLL, &rxq->napi.state);
		netif_napi_add(dev, &rxq->napi, hfi1_netdev_rx_napi, 64);
	}

	return 0;
}

static void hfi1_netdev_rxq_deinit(struct net_device *dev)
{
	int i;
	struct hfi1_netdev_priv *priv = hfi1_netdev_priv(dev);
	struct hfi1_devdata *dd = priv->dd;

	for (i = 0; i < priv->num_rx_q; i++) {
		struct hfi1_netdev_rxq *rxq = &priv->rxq[i];

		netif_napi_del(&rxq->napi);
		hfi1_netdev_deallocate_ctxt(dd, rxq->rcd);
		hfi1_rcd_put(rxq->rcd);
		rxq->rcd = NULL;
	}

	kfree(priv->rxq);
	priv->rxq = NULL;
	priv->num_rx_q = 0;
}

static void enable_queues(struct hfi1_netdev_priv *priv)
{
	int i;

	for (i = 0; i < priv->num_rx_q; i++) {
		struct hfi1_netdev_rxq *rxq = &priv->rxq[i];

		dd_dev_info(priv->dd, "enabling queue %d on context %d\n", i,
			    rxq->rcd->ctxt);
		napi_enable(&rxq->napi);
		hfi1_rcvctrl(priv->dd,
			     HFI1_RCVCTRL_CTXT_ENB | HFI1_RCVCTRL_INTRAVAIL_ENB,
			     rxq->rcd);
	}
}

static void disable_queues(struct hfi1_netdev_priv *priv)
{
	int i;

	msix_netdev_synchronize_irq(priv->dd);

	for (i = 0; i < priv->num_rx_q; i++) {
		struct hfi1_netdev_rxq *rxq = &priv->rxq[i];

		dd_dev_info(priv->dd, "disabling queue %d on context %d\n", i,
			    rxq->rcd->ctxt);

		/* wait for napi if it was scheduled */
		hfi1_rcvctrl(priv->dd,
			     HFI1_RCVCTRL_CTXT_DIS | HFI1_RCVCTRL_INTRAVAIL_DIS,
			     rxq->rcd);
		napi_synchronize(&rxq->napi);
		napi_disable(&rxq->napi);
	}
}

int hfi1_netdev_rx_init(struct hfi1_devdata *dd)
{
	struct hfi1_netdev_priv *priv = hfi1_netdev_priv(dd->dummy_netdev);
	int res;

#ifdef atomic_fetch_inc
	if (atomic_fetch_inc(&priv->netdevs))
#else
	if (atomic_inc_return(&priv->netdevs) - 1)
#endif
		return 0;

	mutex_lock(&hfi1_mutex);
	init_dummy_netdev(dd->dummy_netdev);
	res = hfi1_netdev_rxq_init(dd->dummy_netdev);
	mutex_unlock(&hfi1_mutex);
	return res;
}

int hfi1_netdev_rx_destroy(struct hfi1_devdata *dd)
{
	struct hfi1_netdev_priv *priv = hfi1_netdev_priv(dd->dummy_netdev);

	/* destroy the RX queues only if it is the last netdev going away */
	if (atomic_fetch_add_unless(&priv->netdevs, -1, 0) == 1) {
		mutex_lock(&hfi1_mutex);
		hfi1_netdev_rxq_deinit(dd->dummy_netdev);
		mutex_unlock(&hfi1_mutex);
	}

	return 0;
}

int hfi1_netdev_alloc(struct hfi1_devdata *dd)
{
	struct hfi1_netdev_priv *priv;
	const int netdev_size = sizeof(*dd->dummy_netdev) +
		sizeof(struct hfi1_netdev_priv);

	dd_dev_info(dd, "allocating netdev size %d\n", netdev_size);
	dd->dummy_netdev = kcalloc_node(1, netdev_size, GFP_KERNEL, dd->node);

	if (!dd->dummy_netdev)
		return -ENOMEM;

	priv = hfi1_netdev_priv(dd->dummy_netdev);
	priv->dd = dd;
	idr_init(&priv->idr);
	mutex_init(&priv->idr_lock);
	atomic_set(&priv->enabled, 0);
	atomic_set(&priv->netdevs, 0);

	return 0;
}

void hfi1_netdev_free(struct hfi1_devdata *dd)
{
	struct hfi1_netdev_priv *priv = hfi1_netdev_priv(dd->dummy_netdev);

	if (dd->dummy_netdev) {
		dd_dev_info(dd, "hfi1 netdev freed\n");
		idr_destroy(&priv->idr);
		mutex_destroy(&priv->idr_lock);
		kfree(dd->dummy_netdev);
		dd->dummy_netdev = NULL;
	}
}

void hfi1_netdev_enable_queues(struct hfi1_devdata *dd)
{
	struct hfi1_netdev_priv *priv;

	if (!dd->dummy_netdev)
		return;

	priv = hfi1_netdev_priv(dd->dummy_netdev);
#ifdef atomic_fetch_inc
	if (atomic_fetch_inc(&priv->enabled))
#else
	if (atomic_inc_return(&priv->enabled) - 1)
#endif
		return;

	mutex_lock(&hfi1_mutex);
	enable_queues(priv);
	mutex_unlock(&hfi1_mutex);
}

void hfi1_netdev_disable_queues(struct hfi1_devdata *dd)
{
	struct hfi1_netdev_priv *priv;

	if (!dd->dummy_netdev)
		return;

	priv = hfi1_netdev_priv(dd->dummy_netdev);
	if (atomic_dec_if_positive(&priv->enabled))
		return;

	mutex_lock(&hfi1_mutex);
	disable_queues(priv);
	mutex_unlock(&hfi1_mutex);
}

int hfi1_netdev_add_data(struct hfi1_devdata *dd, int id, void *data)
{
	struct hfi1_netdev_priv *priv = hfi1_netdev_priv(dd->dummy_netdev);
	int rc;

	mutex_lock(&priv->idr_lock);
	rc = idr_alloc(&priv->idr, data, id, id + 1, GFP_NOWAIT);
	mutex_unlock(&priv->idr_lock);
	return rc;
}

void hfi1_netdev_remove_data(struct hfi1_devdata *dd, int id)
{
	struct hfi1_netdev_priv *priv = hfi1_netdev_priv(dd->dummy_netdev);

	mutex_lock(&priv->idr_lock);
	idr_remove(&priv->idr, id);
	mutex_unlock(&priv->idr_lock);
}

void *hfi1_netdev_get_data(struct hfi1_devdata *dd, int id)
{
	struct hfi1_netdev_priv *priv = hfi1_netdev_priv(dd->dummy_netdev);
	void *p;

	rcu_read_lock();
	p = idr_find(&priv->idr, id);
	rcu_read_unlock();
	return p;
}

void *hfi1_netdev_get_first_data(struct hfi1_devdata *dd, int *start_id)
{
	struct hfi1_netdev_priv *priv = hfi1_netdev_priv(dd->dummy_netdev);

	return idr_get_next(&priv->idr, start_id);
}
