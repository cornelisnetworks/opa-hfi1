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

#include "netdev.h"
#include "ipoib.h"

#define HFI1_IPOIB_SKB_PAD NET_SKB_PAD + NET_IP_ALIGN

static void copy_ipoib_buf(struct sk_buff *skb, void *data, int size)
{
	void *dst_data;

	skb_checksum_none_assert(skb);
	skb->protocol = *((__be16 *)data);

	dst_data = skb_put(skb, size);
	memcpy(dst_data, data, size);
	skb->mac_header = HFI1_IPOIB_PSEUDO_LEN;
	skb_pull(skb, HFI1_IPOIB_ENCAP_LEN);
}

static struct sk_buff *prepare_frag_skb(struct napi_struct *napi, int size)
{
	struct sk_buff *skb;
	int skb_size = SKB_DATA_ALIGN(size + HFI1_IPOIB_SKB_PAD);
	void *frag;

	skb_size += SKB_DATA_ALIGN(sizeof(struct skb_shared_info));
	skb_size = SKB_DATA_ALIGN(skb_size);
	frag = napi_alloc_frag(skb_size);

	if (unlikely(!frag))
		return napi_alloc_skb(napi, size);

	skb = build_skb(frag, skb_size);

	if (unlikely(!skb)) {
		skb_free_frag(frag);
		return NULL;
	}

	skb_reserve(skb, HFI1_IPOIB_SKB_PAD);
	return skb;
}

struct sk_buff *hfi1_ipoib_prepare_skb(struct hfi1_netdev_rxq *rxq,
				       int size, void *data)
{
	struct napi_struct *napi = &rxq->napi;
	int skb_size = size + HFI1_IPOIB_ENCAP_LEN;
	struct sk_buff *skb;

	/*
	 * For smaller(4k + skb overhead) allocations we will go using
	 * napi cache. Otherwise we will try to use napi frag cache.
	 */
	if (size <= SKB_WITH_OVERHEAD(PAGE_SIZE))
		skb = napi_alloc_skb(napi, skb_size);
	else
		skb = prepare_frag_skb(napi, skb_size);

	if (unlikely(!skb))
		return NULL;

	copy_ipoib_buf(skb, data, size);

	return skb;
}

int hfi1_ipoib_rxq_init(struct net_device *netdev)
{
	struct hfi1_ipoib_dev_priv *ipoib_priv = hfi1_ipoib_priv(netdev);
	struct hfi1_devdata *dd = ipoib_priv->dd;
	int ret;

	ret = hfi1_netdev_rx_init(dd);
	if (ret)
		return ret;

	hfi1_init_aip_rsm(dd);

	return ret;
}

void hfi1_ipoib_rxq_deinit(struct net_device *netdev)
{
	struct hfi1_ipoib_dev_priv *ipoib_priv = hfi1_ipoib_priv(netdev);
	struct hfi1_devdata *dd = ipoib_priv->dd;

	hfi1_deinit_aip_rsm(dd);
	hfi1_netdev_rx_destroy(dd);
}
