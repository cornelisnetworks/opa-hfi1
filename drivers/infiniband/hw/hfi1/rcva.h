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

#ifndef _RCVA_H
#define _RCVA_H

#include <linux/types.h>
#include <linux/spinlock_types.h>
#include <linux/genalloc.h>

/**
 * struct rcva - receive allocation structure
 *
 * @dd: pointer to devdata
 * @ee_pool: pool of receive array for eager entries
 * @ne_pool: pool of receive array for network entries
 * @te_pool: pool of receive array for tid entries
 * @ee_size: number of eager groups for each context
 * @ne_size: number of eager groups for netdev contexts
 * @te_size: number of eager groups for tid capable contexts
 * @netdev_contexts: return for number of netdev contexts
 */
struct hfi1_devdata;
struct rcva {
	struct hfi1_devdata *dd;
	struct gen_pool *ee_pool;
	struct gen_pool *ne_pool;
	struct gen_pool *te_pool;
	u16 ee_size;
	u16 ne_size;
	u16 te_size;
	u8 netdev_contexts;
};

/**
 * struct rcva_create_attr - input to sizing
 *
 * @max_buffers: max_number of eager buffers
 * @te_contexts: contexts with expected receive entries
 * @ne_contexts: number of netdev contexts
 * @ee_contexts: number of ee_contexts
 * @multi_packet: alter algorithm for multi-packet
 */
struct rcva_create_attr {
	u16 max_buffers;
	u8 te_contexts;
	u8 ne_contexts;
	u8 ee_contexts;
	bool multi_packet;
};

/**
 * struct rcva_slice - stores an allocation from one of the pools
 * @base: starting group for slice within receive array
 * @size: size of array slice
 */
struct rcva_slice {
	u16 base;
	u16 size;
};

struct rcva *rcva_create(struct hfi1_devdata *dd,
			 struct rcva_create_attr *attr);
void rcva_destroy(struct rcva *r);

int rvca_alloc_ee_slice(struct rcva *r, struct rcva_slice *s);
int rvca_alloc_ne_slice(struct rcva *r, struct rcva_slice *s);
int rvca_alloc_te_slice(struct rcva *r, struct rcva_slice *s);

void rvca_free_ee_slice(struct rcva *r, struct rcva_slice *s);
void rvca_free_ne_slice(struct rcva *r, struct rcva_slice *s);
void rvca_free_te_slice(struct rcva *r, struct rcva_slice *s);

#endif /* _RCVA_H */
