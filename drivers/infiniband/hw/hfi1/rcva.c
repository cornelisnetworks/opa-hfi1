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

#include <linux/err.h>
#include <linux/errno.h>

#include "rcva.h"
#include "hfi.h"

#define GROUP_SIZE RCV_INCREMENT
#define MAX_BUFFER_SIZE (256 * 1024)
#define MAX_PER_CONTEXT (2048 / GROUP_SIZE)
#define MIN_NEE_CONTEXTS 2
#define MIN_NEE_BUFFERS 1024
#define MIN_TEE_BUFFERS 2
#define MIN_EE_BUFFERS 1024

/**
 * rcva_size - partition the receive array
 * @r: the rcva to store size attributes
 * @attr: rcv_create_attr structure for parameters
 * @total_ee: the size of the eager entry pool
 * @total_ne: the size of the network eager pool
 * @total_te: the size of the tid entry pool
 *
 * All computations and variables reflect groups
 * to simplify the computations.
 *
 * Legend:
 * * ee - eager entries
 * * ne - network eager entries
 * * te - tid eager entries
 *
 * The routine assumes the the following dd variables have been
 * correctly determined:
 * * n_krcv_queues
 * * num_vnic_contexts
 * * num_user_contexts
 * * num_netdev_contexts
 *
 * The per context size is returned in r.
 *
 */
static void rcva_size(struct rcva *r,
		      struct rcva_create_attr *attr,
		      u16 *total_ee,
		      u16 *total_ne,
		      u16 *total_te)
{
	struct hfi1_devdata *dd = r->dd;
	u32 groups = chip_rcv_array_count(dd) / GROUP_SIZE;
	/* determine number of contexts */
	u32 ee_contexts = attr->ee_contexts;
	u32 ne_contexts = attr->ne_contexts;
	u32 te_contexts = attr->te_contexts;
	/* .e_groups - each context group size */
	/* total_.e_groups - total size across all contexts */
	u32 ee_groups, total_ee_groups;
	u32 te_groups, total_te_groups;
	u32 ne_groups, total_ne_groups;
	u32 extra_groups;
	bool multi_packet = attr->multi_packet;

	/* first size tid */
	te_groups = te_contexts ?
			min_t(u32, groups / te_contexts, MAX_PER_CONTEXT) : 0;
	total_te_groups = te_groups * te_contexts;
	dd_dev_dbg(dd, "ee_contexts %u te_contexts %u ne_contexts %u\n",
		   ee_contexts, te_contexts, ne_contexts);
	/* kernel/PSM with multi-packet */
	ee_groups = attr->max_buffers / GROUP_SIZE;
	ee_groups = min_t(u32, ee_groups, MAX_PER_CONTEXT);

	total_ee_groups = ee_groups * ee_contexts;

	ne_groups = MAX_PER_CONTEXT;
	total_ne_groups = ne_groups * ne_contexts;

	dd_dev_dbg(dd, "initial sizes: ee_groups %u total_ee_groups %u ne_groups %u total_ne_groups %u te_groups %u total_te_groups %u\n",
		   ee_groups, total_ee_groups,
		   ne_groups, total_ne_groups,
		   te_groups, total_te_groups);
	while (true) {
		if ((total_ee_groups + total_te_groups + total_ne_groups) <=
		    groups)
			break;
		/* we hit the floor for buffers per group  - cut contexts */
		if (ne_contexts > MIN_NEE_CONTEXTS) {
			ne_contexts /= 2;
			total_ne_groups = ne_groups * ne_contexts;
		}
		if ((total_ee_groups + total_te_groups + total_ne_groups) <=
		    groups)
			break;
		if (ne_groups > MIN_NEE_BUFFERS / GROUP_SIZE) {
			ne_groups /= 2;
			total_ne_groups = ne_groups * ne_contexts;
		}
		if ((total_ee_groups + total_te_groups + total_ne_groups) <=
		    groups)
			break;
		if  (!multi_packet) {
			if (ee_groups > MIN_EE_BUFFERS)
				ee_groups /= 2;
			total_ee_groups = ee_groups * ee_contexts;
		}
		if ((total_ee_groups + total_te_groups + total_ne_groups) <=
		    groups)
			break;
		if (te_groups > MIN_TEE_BUFFERS)
			total_te_groups = --te_groups * te_contexts;
		if ((total_ee_groups + total_te_groups + total_ne_groups) <=
		    groups)
			break;
		dd_dev_dbg(dd, "continue with ee_groups %u ne_groups %u te_groups %u ne_contexts %u\n",
			   ee_groups, ne_groups, te_groups, ne_contexts);
	}
	dd_dev_dbg(dd, "after reductions: ee_groups %u total_ee_groups %u ne_groups %u total_ne_groups %u te_groups %u total_te_groups %u\n",
		   ee_groups, total_ee_groups,
		   ne_groups, total_ne_groups,
		   te_groups, total_te_groups);
	extra_groups = groups -
			(total_ee_groups + total_te_groups + total_ne_groups);
	dd_dev_dbg(dd, "extra_groups %u\n", extra_groups);
	/* try to increase tid */
	if (te_contexts) {
		te_groups += extra_groups / te_contexts;
		te_groups = min_t(u32, MAX_PER_CONTEXT, te_groups);
		total_te_groups = te_groups * te_contexts;
	}
	extra_groups = groups -
			(total_ee_groups + total_te_groups + total_ne_groups);
	/* try to increase network */
	if (ne_contexts) {
		ne_groups += extra_groups / ne_contexts;
		ne_groups = min_t(u32, MAX_PER_CONTEXT, ne_groups);
		total_ne_groups = ne_groups * ne_contexts;
	}
	extra_groups = groups -
			(total_ee_groups + total_te_groups + total_ne_groups);
	ee_groups += extra_groups / ee_contexts;
	dd_dev_dbg(dd, "final sizing: ne_contexts %u ee_groups %u total_ee_groups %u ne_groups %u total_ne_groups %u te_groups %u total_te_groups %u extra_groups %u\n",
		   ne_contexts, ee_groups, total_ee_groups,
		   ne_groups, total_ne_groups,
		   te_groups, total_te_groups, extra_groups);
	/* return sizes of each pool */
	*total_ee = total_ee_groups;
	*total_ne = total_ne_groups;
	*total_te = total_te_groups;
	/* return size of each contexts group */
	r->ee_size = ee_groups;
	r->ne_size = ne_groups;
	r->te_size = te_groups;
	r->netdev_contexts = ne_contexts;
}

/**
 * rcva_create - returns a rcva structure
 * @dd: the device data
 * @attr: rcv_create_attr structure for parameters
 *
 * This routine inits and returns a rcva structure
 * for assigning within the dd structure.
 *
 * The routine sizes and establishes pools that support the allocation of
 * eager entries, tid entries, and network entries that are used by
 * the various types of contexts.
 *
 * The chunks are added as one relative to distinquish a failure
 * of 0 from a successful allocation.  The allocate functions and
 * deallocate functions make the necessary adjustments.
 *
 */
struct rcva *rcva_create(struct hfi1_devdata *dd,
			 struct rcva_create_attr *attr)
{
	struct rcva *r;
	u16 total_ee, total_ne, total_te;
	/* chunks are one relative */
	u16 start_ee = 0, start_ne, start_te;

	r = kzalloc_node(sizeof(*r), GFP_KERNEL, dd->node);
	if (!r)
		return ERR_PTR(-ENOMEM);
	r->dd = dd;
	rcva_size(r, attr, &total_ee, &total_ne, &total_te);
	start_ne = total_ee;
	start_te = total_ee + total_ne;

	r->ee_pool = gen_pool_create(0, dd->node);
	if (!r->ee_pool)
		goto bail;
	if (total_ee &&
	    gen_pool_add(r->ee_pool, start_ee + 1, total_ee, dd->node))
		goto bail;

	if (total_ne) {
		r->ne_pool = gen_pool_create(0, dd->node);
		if (!r->ne_pool)
			goto bail;
		if (gen_pool_add(r->ne_pool, start_ne + 1, total_ne, dd->node))
		goto bail;
	}

	if (total_te) {
		r->te_pool = gen_pool_create(0, dd->node);
		if (!r->te_pool)
			goto bail;
		if (gen_pool_add(r->te_pool, start_te + 1, total_te, dd->node))
			goto bail;
	}

	return r;
bail:
	rcva_destroy(r);
	return ERR_PTR(-ENOMEM);
}

/**
 * rcva_destroy - tears down a structure returned by rcva_create()
 * @r: the rcva structure to be torn down
 */
void rcva_destroy(struct rcva *r)
{
	if (!r)
		return;
	if (r->te_pool)
		gen_pool_destroy(r->te_pool);
	r->te_pool = NULL;
	if (r->ne_pool)
		gen_pool_destroy(r->ne_pool);
	r->ne_pool = NULL;
	if (r->ee_pool)
		gen_pool_destroy(r->ee_pool);
	r->ee_pool = NULL;
	kfree(r);
}

/**
 * rvca_alloc_ee_slice - allocate an eager entry slice
 * @r: the rcva structure that holds the slices
 * @s: the slice structure that will hold the slice
 */
int rvca_alloc_ee_slice(struct rcva *r, struct rcva_slice *s)
{
	unsigned long ret;

	ret = gen_pool_alloc(r->ee_pool, r->ee_size);
	if (!ret)
		return -ENOMEM;
	/* make zero relative */
	s->base = ret - 1;
	s->size = r->ee_size;
	return 0;
}

/**
 * rvca_alloc_ne_slice - allocate an network device slice
 * @r: the rcva structure that holds the slices
 * @s: the slice structure that will hold the slice
 */
int rvca_alloc_ne_slice(struct rcva *r, struct rcva_slice *s)
{
	unsigned long ret;

	ret = gen_pool_alloc(r->ne_pool, r->ne_size);
	if (!ret)
		return -ENOMEM;
	/* make zero relative */
	s->base = ret - 1;
	s->size = r->ne_size;
	return 0;
}

/**
 * rvca_alloc_te_slice - allocate an tid slice
 * @r: the rcva structure that holds the slices
 * @s: the slice structure that will hold the slice
 */
int rvca_alloc_te_slice(struct rcva *r, struct rcva_slice *s)
{
	unsigned long ret;

	ret = gen_pool_alloc(r->te_pool, r->te_size);
	if (!ret)
		return -ENOMEM;
	/* make zero relative */
	s->base = ret - 1;
	s->size = r->te_size;
	return 0;
}

/**
 * rvca_free_ee_slice - free an eager entry slice
 * @r: the rcva structure that holds the slices
 * @s: the slice structure that will be returned
 */
void rvca_free_ee_slice(struct rcva *r, struct rcva_slice *s)
{
	if (s->size)
		gen_pool_free(r->ee_pool, s->base + 1, s->size);
	s->size = 0;
}

/**
 * rvca_free_ne_slice - free an network entry slice
 * @r: the rcva structure that holds the slices
 * @s: the slice structure that will be returned
 */
void rvca_free_ne_slice(struct rcva *r, struct rcva_slice *s)
{
	if (s->size)
		gen_pool_free(r->ne_pool, s->base + 1, s->size);
	s->size = 0;
}

/**
 * rvca_free_te_slice - free an tid entry slice
 * @r: the rcva structure that holds the slices
 * @s: the slice structure that will be returned
 */
void rvca_free_te_slice(struct rcva *r, struct rcva_slice *s)
{
	if (s->size)
		gen_pool_free(r->te_pool, s->base + 1, s->size);
	s->size = 0;
}
