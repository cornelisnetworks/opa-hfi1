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
 * This file contains HFI1 support for IPOIB SDMA functionality
 */

#include "sdma.h"
#include "verbs.h"
#include "trace_ibhdrs.h"
#include "ipoib.h"

/**
 * struct ipoib_txreq - IPOIB transmit descriptor
 * @txreq: sdma transmit request
 * @sdma_hdr: 9b ib headers
 * @priv: ipoib netdev private data
 * @skb: skb to send
 */
struct ipoib_txreq {
	struct sdma_txreq           txreq;
	struct hfi1_sdma_header     sdma_hdr;
	struct hfi1_ipoib_dev_priv *priv;
	struct sk_buff             *skb;
};

struct ipoib_txparms {
	struct hfi1_devdata        *dd;
	struct rdma_ah_attr        *ah_attr;
	struct hfi1_ibport         *ibp;
	struct hfi1_ipoib_txq      *txq;
	union hfi1_ipoib_flow       flow;
	u32                         dqpn;
	u8                          hdr_dwords;
	u8                          entropy;
};

static void hfi1_ipoib_sdma_complete(struct sdma_txreq *txreq, int status)
{
	struct ipoib_txreq *tx = container_of(txreq, struct ipoib_txreq, txreq);
	struct sk_buff *skb = tx->skb;
	struct hfi1_ipoib_dev_priv *priv = tx->priv;
	struct net_device *netdev = priv->netdev;

	if (status == 0) {
		hfi1_ipoib_update_tx_netstats(priv, 1, skb->len);
	} else {
		++netdev->stats.tx_errors;
		dd_dev_warn(priv->dd, "%s: Status = 0x%x pbc 0x%llx\n",
			    __func__, status, le64_to_cpu(tx->sdma_hdr.pbc));
	}

	sdma_txclean(priv->dd, txreq);
	dev_kfree_skb_any(skb);
	kmem_cache_free(priv->txreq_cache, tx);
}

static int hfi1_ipoib_build_ulp_payload(struct ipoib_txreq *tx,
					struct ipoib_txparms *txp)
{
	struct hfi1_devdata *dd = txp->dd;
	struct sdma_txreq *txreq = &tx->txreq;
	struct sk_buff *skb = tx->skb;
	int ret = 0;
	int i;

	if (skb_headlen(skb)) {
		ret = sdma_txadd_kvaddr(dd, txreq, skb->data, skb_headlen(skb));
		if (unlikely(ret))
			return ret;
	}

	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		struct skb_frag_struct *frag = &skb_shinfo(skb)->frags[i];

		ret = sdma_txadd_page(dd,
				      txreq,
				      skb_frag_page(frag),
				      frag->page_offset,
				      skb_frag_size(frag));
		if (unlikely(ret))
			break;
	}

	return ret;
}

static int hfi1_ipoib_build_tx_desc(struct ipoib_txreq *tx,
				    struct ipoib_txparms *txp)
{
	struct hfi1_devdata *dd = txp->dd;
	struct sdma_txreq *txreq = &tx->txreq;
	struct hfi1_sdma_header *sdma_hdr = &tx->sdma_hdr;
	u16 pkt_bytes =
		sizeof(sdma_hdr->pbc) + (txp->hdr_dwords << 2) + tx->skb->len;
	int ret;

	ret = sdma_txinit(txreq, 0, pkt_bytes, hfi1_ipoib_sdma_complete);
	if (unlikely(ret))
		return ret;

	/* add pbc + headers */
	ret = sdma_txadd_kvaddr(dd,
				txreq,
				sdma_hdr,
				sizeof(sdma_hdr->pbc) + (txp->hdr_dwords << 2));
	if (unlikely(ret))
		return ret;

	/* add the ulp payload */
	return hfi1_ipoib_build_ulp_payload(tx, txp);
}

static void hfi1_ipoib_build_ib_tx_headers(struct ipoib_txreq *tx,
					   struct ipoib_txparms *txp)
{
	struct hfi1_ipoib_dev_priv *priv = tx->priv;
	struct hfi1_sdma_header *sdma_hdr = &tx->sdma_hdr;
	struct sk_buff *skb = tx->skb;
	struct hfi1_pportdata *ppd = ppd_from_ibp(txp->ibp);
	struct rdma_ah_attr *ah_attr = txp->ah_attr;
	struct ib_other_headers *ohdr;
	struct ib_grh *grh;
	u16 dwords;
	u16 slid;
	u16 dlid;
	u16 lrh0;
	u32 bth0;
	u32 sqpn = (u32)(priv->netdev->dev_addr[1] << 16 |
			 priv->netdev->dev_addr[2] << 8 |
			 priv->netdev->dev_addr[3]);
	u16 payload_dwords;
	u8 pad_cnt;

	pad_cnt = -skb->len & 3;

	/* Includes ICRC */
	payload_dwords = ((skb->len + pad_cnt) >> 2) + SIZE_OF_CRC;

	/* header size in dwords LRH+BTH+DETH = (8+12+8)/4. */
	txp->hdr_dwords = 7;

	if (rdma_ah_get_ah_flags(ah_attr) & IB_AH_GRH) {
		grh = &sdma_hdr->hdr.ibh.u.l.grh;
		txp->hdr_dwords +=
			hfi1_make_grh(txp->ibp,
				      grh,
				      rdma_ah_read_grh(ah_attr),
				      txp->hdr_dwords - LRH_9B_DWORDS,
				      payload_dwords);
		lrh0 = HFI1_LRH_GRH;
		ohdr = &sdma_hdr->hdr.ibh.u.l.oth;
	} else {
		lrh0 = HFI1_LRH_BTH;
		ohdr = &sdma_hdr->hdr.ibh.u.oth;
	}

	lrh0 |= (rdma_ah_get_sl(ah_attr) & 0xf) << 4;
	lrh0 |= (txp->flow.sc5 & 0xf) << 12;

	dlid = opa_get_lid(rdma_ah_get_dlid(ah_attr), 9B);
	if (dlid == be16_to_cpu(IB_LID_PERMISSIVE)) {
		slid = be16_to_cpu(IB_LID_PERMISSIVE);
	} else {
		u16 lid = (u16)ppd->lid;

		if (lid) {
			lid |= rdma_ah_get_path_bits(ah_attr) &
				((1 << ppd->lmc) - 1);
			slid = lid;
		} else {
			slid = be16_to_cpu(IB_LID_PERMISSIVE);
		}
	}

	/* Includes ICRC */
	dwords = txp->hdr_dwords + payload_dwords;

	/* Build the lrh */
	sdma_hdr->hdr.hdr_type = HFI1_PKT_TYPE_9B;
	hfi1_make_ib_hdr(&sdma_hdr->hdr.ibh, lrh0, dwords, dlid, slid);

	/* Build the bth */
	bth0 = (IB_OPCODE_UD_SEND_ONLY << 24) | (pad_cnt << 20) | priv->pkey;

	ohdr->bth[0] = cpu_to_be32(bth0);
	ohdr->bth[1] = cpu_to_be32(txp->dqpn);
	ohdr->bth[2] = cpu_to_be32(mask_psn(txp->txq->psn));

	/* Build the deth */
	ohdr->u.ud.deth[0] = cpu_to_be32(priv->qkey);
	ohdr->u.ud.deth[1] = cpu_to_be32((txp->entropy <<
					  RVT_AIP_ENTROPY_SHIFT) | sqpn);

	/* Construct the pbc. */
	sdma_hdr->pbc =
		cpu_to_le64(create_pbc(ppd,
				       ib_is_sc5(txp->flow.sc5) <<
							      PBC_DC_INFO_SHIFT,
				       0,
				       sc_to_vlt(priv->dd, txp->flow.sc5),
				       dwords - SIZE_OF_CRC +
						(sizeof(sdma_hdr->pbc) >> 2)));
}

static struct ipoib_txreq *hfi1_ipoib_send_dma_common(struct net_device *dev,
						      struct sk_buff *skb,
						      struct ipoib_txparms *txp)
{
	struct hfi1_ipoib_dev_priv *priv = hfi1_ipoib_priv(dev);
	struct ipoib_txreq *tx;
	int ret;

	tx = kmem_cache_alloc_node(priv->txreq_cache,
				   GFP_ATOMIC,
				   priv->dd->node);
	if (unlikely(!tx))
		return ERR_PTR(-ENOMEM);

	/* so that we can test if the sdma decriptors are there */
	tx->txreq.num_desc = 0;
	tx->priv = priv;
	tx->skb = skb;

	hfi1_ipoib_build_ib_tx_headers(tx, txp);

	ret = hfi1_ipoib_build_tx_desc(tx, txp);
	if (likely(!ret)) {
		if (txp->txq->flow.as_int != txp->flow.as_int) {
			txp->txq->flow.tx_queue = txp->flow.tx_queue;
			txp->txq->flow.sc5 = txp->flow.sc5;
			txp->txq->sde =
				sdma_select_engine_sc(priv->dd,
						      txp->flow.tx_queue,
						      txp->flow.sc5);
		}

		return tx;
	}

	sdma_txclean(priv->dd, &tx->txreq);
	kmem_cache_free(priv->txreq_cache, tx);

	return ERR_PTR(ret);
}

static int hfi1_ipoib_submit_tx_list(struct net_device *dev,
				     struct hfi1_ipoib_txq *txq)
{
	int ret;
	u16 count_out;

	ret = sdma_send_txlist(txq->sde,
			       iowait_get_ib_work(&txq->wait),
			       &txq->tx_list,
			       &count_out);
	if (likely(!ret) || ret == -EBUSY || ret == -ECOMM)
		return ret;

	dd_dev_warn(txq->priv->dd, "cannot send skb tx list, err %d.\n", ret);

	return ret;
}

static int hfi1_ipoib_flush_tx_list(struct net_device *dev,
				    struct hfi1_ipoib_txq *txq)
{
	int ret = 0;

	if (!list_empty(&txq->tx_list)) {
		/* Flush the current list */
		ret = hfi1_ipoib_submit_tx_list(dev, txq);

		if (unlikely(ret))
			if (ret != -EBUSY)
				++dev->stats.tx_carrier_errors;
	}

	return ret;
}

static int hfi1_ipoib_submit_tx(struct hfi1_ipoib_txq *txq,
				struct ipoib_txreq *tx)
{
	int ret;

	ret = sdma_send_txreq(txq->sde,
			      iowait_get_ib_work(&txq->wait),
			      &tx->txreq,
			      txq->pkts_sent);
	if (likely(!ret)) {
		txq->pkts_sent = true;
		iowait_starve_clear(txq->pkts_sent, &txq->wait);
	}

	return ret;
}

static int hfi1_ipoib_send_dma_single(struct net_device *dev,
				      struct sk_buff *skb,
				      struct ipoib_txparms *txp)
{
	struct hfi1_ipoib_dev_priv *priv = hfi1_ipoib_priv(dev);
	struct hfi1_ipoib_txq *txq = txp->txq;
	struct ipoib_txreq *tx;
	int ret;

	tx = hfi1_ipoib_send_dma_common(dev, skb, txp);
	if (unlikely(IS_ERR(tx))) {
		int ret = PTR_ERR(tx);

		dev_kfree_skb_any(skb);

		if (ret == -ENOMEM)
			++dev->stats.tx_errors;
		else
			++dev->stats.tx_carrier_errors;

		return NETDEV_TX_OK;
	}

	ret = hfi1_ipoib_submit_tx(txq, tx);
	if (likely(!ret)) {
		trace_sdma_output_ibhdr(tx->priv->dd,
					&tx->sdma_hdr.hdr,
					ib_is_sc5(txp->flow.sc5));
		return NETDEV_TX_OK;
	}

	txq->pkts_sent = false;

	if (ret == -EBUSY) {
		list_add_tail(&tx->txreq.list, &txq->tx_list);

		trace_sdma_output_ibhdr(tx->priv->dd,
					&tx->sdma_hdr.hdr,
					ib_is_sc5(txp->flow.sc5));
		return NETDEV_TX_OK;
	}

	if (ret == -ECOMM)
		return NETDEV_TX_OK;

	sdma_txclean(priv->dd, &tx->txreq);
	dev_kfree_skb_any(skb);
	kmem_cache_free(priv->txreq_cache, tx);
	++dev->stats.tx_carrier_errors;

	return NETDEV_TX_OK;
}

static int hfi1_ipoib_send_dma_list(struct net_device *dev,
				    struct sk_buff *skb,
				    struct ipoib_txparms *txp)
{
	struct hfi1_ipoib_txq *txq = txp->txq;
	struct ipoib_txreq *tx;

	/* Has the flow change ? */
	if (txq->flow.as_int != txp->flow.as_int)
		(void)hfi1_ipoib_flush_tx_list(dev, txq);

	tx = hfi1_ipoib_send_dma_common(dev, skb, txp);
	if (unlikely(IS_ERR(tx))) {
		int ret = PTR_ERR(tx);

		dev_kfree_skb_any(skb);

		if (ret == -ENOMEM)
			++dev->stats.tx_errors;
		else
			++dev->stats.tx_carrier_errors;

		return NETDEV_TX_OK;
	}

	list_add_tail(&tx->txreq.list, &txq->tx_list);

	trace_sdma_output_ibhdr(tx->priv->dd,
				&tx->sdma_hdr.hdr,
				ib_is_sc5(txp->flow.sc5));

	if (!skb->xmit_more)
		(void)hfi1_ipoib_flush_tx_list(dev, txq);

	return NETDEV_TX_OK;
}

static u8 hfi1_ipoib_calc_entropy(struct sk_buff *skb)
{
	if (unlikely(!skb))
		return 0;

	if (skb_transport_header_was_set(skb)) {
		u8 *hdr = (u8 *)skb_transport_header(skb);

		return (hdr[0] ^ hdr[1] ^ hdr[2] ^ hdr[3]);
	}

	return (u8)skb_get_queue_mapping(skb);
}

int hfi1_ipoib_send_dma(struct net_device *dev,
			struct sk_buff *skb,
			struct ib_ah *address,
			u32 dqpn)
{
	struct hfi1_ipoib_dev_priv *priv = hfi1_ipoib_priv(dev);
	struct ipoib_txparms txp;
	struct rdma_netdev *rn = netdev_priv(dev);

	if (unlikely(skb->len > rn->mtu + HFI1_IPOIB_ENCAP_LEN)) {
		dd_dev_warn(priv->dd, "packet len %d (> %d) too long to send, dropping\n",
			    skb->len,
			    rn->mtu + HFI1_IPOIB_ENCAP_LEN);
		++dev->stats.tx_dropped;
		++dev->stats.tx_errors;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	txp.dd = priv->dd;
	txp.ah_attr = &ibah_to_rvtah(address)->attr;
	txp.ibp = to_iport(priv->device, priv->port_num);
	txp.txq = &priv->txqs[skb_get_queue_mapping(skb)];
	txp.dqpn = dqpn;
	txp.txq->psn++;
	txp.flow.sc5 = txp.ibp->sl_to_sc[rdma_ah_get_sl(txp.ah_attr)];
	txp.flow.tx_queue = (u8)skb_get_queue_mapping(skb);
	txp.entropy = hfi1_ipoib_calc_entropy(skb);

	if (skb->xmit_more || !list_empty(&txp.txq->tx_list))
		return hfi1_ipoib_send_dma_list(dev, skb, &txp);

	return hfi1_ipoib_send_dma_single(dev, skb,  &txp);
}

/*
 * hfi1_ipoib_sdma_sleep - ipoib sdma sleep function
 *
 * This function gets called from sdma_send_txreq() when there are not enough
 * sdma descriptors available to send the packet. It adds Tx queue's wait
 * structure to sdma engine's dmawait list to be woken up when descriptors
 * become available.
 */
static int hfi1_ipoib_sdma_sleep(struct sdma_engine *sde,
				 struct iowait_work *wait,
				 struct sdma_txreq *txreq,
				 uint seq,
				 bool pkts_sent)
{
	struct hfi1_ipoib_txq *txq =
		container_of(wait->iow, struct hfi1_ipoib_txq, wait);
	struct hfi1_ibdev *dev = &txq->priv->dd->verbs_dev;

	write_seqlock(&dev->iowait_lock);
	if (likely(txq->priv->netdev->reg_state == NETREG_REGISTERED)) {
		if (sdma_progress(sde, seq, txreq)) {
			write_sequnlock(&dev->iowait_lock);
			return -EAGAIN;
		}

		netif_stop_subqueue(txq->priv->netdev, txq->q_idx);

		if (list_empty(&txq->wait.list))
			iowait_queue(pkts_sent, wait->iow, &sde->dmawait);

		write_sequnlock(&dev->iowait_lock);
		return -EBUSY;
	}

	write_sequnlock(&dev->iowait_lock);
	return -EINVAL;
}

/*
 * hfi1_ipoib_sdma_wakeup - ipoib sdma wakeup function
 *
 * This function gets called when SDMA descriptors becomes available and Tx
 * queue's wait structure was previously added to sdma engine's dmawait list.
 */
static void hfi1_ipoib_sdma_wakeup(struct iowait *wait, int reason)
{
	struct hfi1_ipoib_txq *txq =
		container_of(wait, struct hfi1_ipoib_txq, wait);

	if (likely(txq->priv->netdev->reg_state == NETREG_REGISTERED))
#if !defined(IFS_RH74)
		iowait_schedule(wait, system_highpri_wq, WORK_CPU_UNBOUND);
#else
		iowait_schedule(wait, system_wq, WORK_CPU_UNBOUND);
#endif
}

static void hfi1_ipoib_flush_txq(struct work_struct *work)
{
	struct iowait_work *ioww =
		container_of(work, struct iowait_work, iowork);
	struct iowait *wait;
	struct hfi1_ipoib_txq *txq;
	struct net_device *dev;

	wait = iowait_ioww_to_iow(ioww);
	if (!wait)
		return;

	txq = container_of(wait, struct hfi1_ipoib_txq, wait);
	dev = txq->priv->netdev;

	if (likely(dev->reg_state == NETREG_REGISTERED) &&
	    likely(__netif_subqueue_stopped(dev, txq->q_idx)) &&
	    likely(!hfi1_ipoib_flush_tx_list(dev, txq)))
		netif_wake_subqueue(dev, txq->q_idx);
}

int hfi1_ipoib_txreq_init(struct hfi1_ipoib_dev_priv *priv)
{
	struct net_device *dev = priv->netdev;
	char buf[HFI1_IPOIB_TXREQ_NAME_LEN];
	int i;

	snprintf(buf, sizeof(buf), "hfi1_%u_ipoib_txreq_cache", priv->dd->unit);
	priv->txreq_cache = kmem_cache_create(buf,
					      sizeof(struct ipoib_txreq),
					      0,
					      0,
					      NULL);
	if (!priv->txreq_cache)
		return -ENOMEM;

	priv->txqs = kcalloc_node(dev->num_tx_queues,
				  sizeof(struct hfi1_ipoib_txq),
				  GFP_ATOMIC,
				  priv->dd->node);
	if (!priv->txqs) {
		kmem_cache_destroy(priv->txreq_cache);
		priv->txreq_cache = NULL;
		return -ENOMEM;
	}

	for (i = 0; i < dev->num_tx_queues; i++) {
		struct hfi1_ipoib_txq *txq = &priv->txqs[i];

		iowait_init(&txq->wait,
			    0,
			    hfi1_ipoib_flush_txq,
			    NULL,
			    hfi1_ipoib_sdma_sleep,
			    hfi1_ipoib_sdma_wakeup,
			    NULL,
			    NULL);
		txq->priv = priv;
		txq->sde = NULL;
		INIT_LIST_HEAD(&txq->tx_list);
		txq->psn = 0;
		txq->q_idx = i;
		txq->flow.tx_queue = 0xff;
		txq->flow.sc5 = 0xff;
		txq->pkts_sent = false;

		netdev_queue_numa_node_write(netdev_get_tx_queue(dev, i),
					     priv->dd->node);
	}

	return 0;
}

static void hfi1_ipoib_drain_tx_list(struct hfi1_ipoib_txq *txq)
{
	struct sdma_txreq *txreq;
	struct sdma_txreq *txreq_tmp;

	list_for_each_entry_safe(txreq, txreq_tmp, &txq->tx_list, list) {
		struct ipoib_txreq *tx =
			container_of(txreq, struct ipoib_txreq, txreq);
		list_del(&txreq->list);
		sdma_txclean(txq->priv->dd, &tx->txreq);
		dev_kfree_skb_any(tx->skb);
		kmem_cache_free(txq->priv->txreq_cache, tx);
	}
}

void hfi1_ipoib_txreq_deinit(struct hfi1_ipoib_dev_priv *priv)
{
	int i;

	for (i = 0; i < priv->netdev->num_tx_queues; i++) {
		struct hfi1_ipoib_txq *txq = &priv->txqs[i];

		iowait_cancel_work(&txq->wait);
		iowait_sdma_drain(&txq->wait);
		hfi1_ipoib_drain_tx_list(txq);
	}

	kfree(priv->txqs);
	priv->txqs = NULL;

	kmem_cache_destroy(priv->txreq_cache);
	priv->txreq_cache = NULL;
}
