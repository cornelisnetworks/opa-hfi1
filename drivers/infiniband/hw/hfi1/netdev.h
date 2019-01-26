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

#ifndef HFI1_NETDEV_H
#define HFI1_NETDEV_H

#include "hfi.h"

#include <linux/netdevice.h>
#include <linux/idr.h>

/**
 * struct hfi1_netdev_rxq - Receive Queue for HFI
 * dummy netdev. Both IPoIB and VNIC netdevices will be working on
 * top of this device.
 * @napi: napi object
 * @priv: ptr to netdev_priv
 * @rcd:  ptr to receive context data
 */
struct hfi1_netdev_rxq {
	struct napi_struct napi;
	struct hfi1_netdev_priv *priv;
	struct hfi1_ctxtdata *rcd;
};

#define HFI1_NETDEV_MAX 255
/*
 * Number of VNIC contexts used. Ensure it is less than or equal to
 * max queues supported by VNIC (HFI1_VNIC_MAX_QUEUE).
 */
#define HFI1_NUM_NETDEV_CTXT   8

/* Number of VNIC RSM entries */
#define NUM_NETDEV_MAP_ENTRIES 8

/**
 * struct hfi1_netdev_priv: data required to setup and run HFI netdev.
 * @dd:		hfi1_devdata
 * @rxq:	pointer to dummy netdev receive queues.
 * @num_rx_q:	number of receive queues
 * @rmt_index:	first free index in RMT Array
 * @msix_start: first free MSI-X interrupt vector.
 * @idr_lock:	lock for inserting/destroying IDR entries
 *		used int VLANs and VNIC implementations.
 * @idr:	IDR for unique identifier VNIC and IPoIb VLANs.
 * @enabled:	atomic counter of netdevs enabling receive queues.
 *		When 0 NAPI will be disabled.
 * @netdevs:	atomic counter of netdevs using dummy netdev.
 *		When 0 receive queues will be freed.
 */
struct hfi1_netdev_priv {
	struct hfi1_devdata *dd;
	struct hfi1_netdev_rxq *rxq;
	int num_rx_q;
	int rmt_start;
	/* mutex to protext idr */
	struct mutex idr_lock;
	struct idr idr;
	/* count of enabled napi polls */
	atomic_t enabled;
	/* count of netdevs on top */
	atomic_t netdevs;
};

static inline
struct hfi1_netdev_priv *hfi1_netdev_priv(struct net_device *dev)
{
	return (struct hfi1_netdev_priv *)&dev[1];
}

static inline
int hfi1_netdev_ctxt_count(struct hfi1_devdata *dd)
{
	struct hfi1_netdev_priv *priv = hfi1_netdev_priv(dd->dummy_netdev);

	return priv->num_rx_q;
}

static inline
struct hfi1_ctxtdata *hfi1_netdev_get_ctxt(struct hfi1_devdata *dd, int ctxt)
{
	struct hfi1_netdev_priv *priv = hfi1_netdev_priv(dd->dummy_netdev);

	return priv->rxq[ctxt].rcd;
}

static inline
int hfi1_netdev_get_free_rmt_idx(struct hfi1_devdata *dd)
{
	struct hfi1_netdev_priv *priv = hfi1_netdev_priv(dd->dummy_netdev);

	return priv->rmt_start;
}

static inline
void hfi1_netdev_set_free_rmt_idx(struct hfi1_devdata *dd, int rmt_idx)
{
	struct hfi1_netdev_priv *priv = hfi1_netdev_priv(dd->dummy_netdev);

	priv->rmt_start = rmt_idx;
}

/**
 * hfi1_netdev_enable_queues - This is napi enable function.
 * It enables napi objects associated with queues.
 * When at least one device has called it it increments atomic counter.
 * Disable function decrements counter and when it is 0,
 * calls napi_disable for every queue.
 *
 * @dd: hfi1 dev data
 */
void hfi1_netdev_enable_queues(struct hfi1_devdata *dd);
void hfi1_netdev_disable_queues(struct hfi1_devdata *dd);

/**
 * hfi1_netdev_rx_init - Incrememnts netdevs counter. When called first time,
 * it allocates receive queue data and calls netif_napi_add
 * for each queue.
 *
 * @dd: hfi1 dev data
 */
int hfi1_netdev_rx_init(struct hfi1_devdata *dd);

/*
 * hfi1_netdev_rx_destroy - Decrements netdevs counter, when it reaches 0
 * napi is deleted and receive queses memory is freed.
 *
 * @dd: hfi1 dev data
 */
int hfi1_netdev_rx_destroy(struct hfi1_devdata *dd);

/**
 * hfi1_netdev_alloc - Allocates netdev and private data. It is required
 * because RMT index and MSI-X interrupt can be set only
 * during driver initialization.
 *
 * @dd: hfi1 dev data
 */
int hfi1_netdev_alloc(struct hfi1_devdata *dd);
void hfi1_netdev_free(struct hfi1_devdata *dd);

/**
 * Rhfi1_netdev_add_data - egisters data with unique identifier
 * to be requested later this is needed for VNIC and IPoIB VLANs
 * implementations.
 * This call is protected by mutex idr_lock.
 *
 * @dd: hfi1 dev data
 * @id: requested integer id up to INT_MAX
 * @data: data to be associated with index
 */
int hfi1_netdev_add_data(struct hfi1_devdata *dd, int id, void *data);

/**
 * hfi1_netdev_remove_data - Removes data with previously given id.
 * Returns the reference to removed entry.
 *
 * @dd: hfi1 dev data
 * @id: requested integer id up to INT_MAX
 */
void hfi1_netdev_remove_data(struct hfi1_devdata *dd, int id);

/**
 * hfi1_netdev_get_data - Gets data with given id
 *
 * @dd: hfi1 dev data
 * @id: requested integer id up to INT_MAX
 */
void *hfi1_netdev_get_data(struct hfi1_devdata *dd, int id);

/**
 * hfi1_netdev_get_first_dat - aGets first entry with greater or equal id.
 *
 * @dd: hfi1 dev data
 * @id: requested integer id up to INT_MAX
 */
void *hfi1_netdev_get_first_data(struct hfi1_devdata *dd, int *start_id);

/* chip.c  */
/**
 * hfi1_netdev_rx_napi - NAPI poll function
 *
 * @napi: pointer to NAPI object
 * @budget: budget
 */
int hfi1_netdev_rx_napi(struct napi_struct *napi, int budget);

#endif /* HFI1_NETDEV_H */
