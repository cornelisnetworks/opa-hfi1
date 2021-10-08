#ifndef DEF_RVTAH_H
#define DEF_RVTAH_H

/*
 * Copyright(c) 2016 Intel Corporation.
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

#include <rdma/rdma_vt.h>

#ifdef HAVE_RDMA_AH_INIT_ATTR
int rvt_create_ah(struct ib_ah *ibah, struct rdma_ah_init_attr *init_attr,
		  struct ib_udata *udata);
#elif defined(HAVE_CORE_ALLOC_AH)
int compat_rvt_create_ah(struct ib_ah *ah, struct rdma_ah_attr *ah_attr,
			 u32 flags, struct ib_udata *udata);
#elif defined(CREATE_AH_HAS_FLAGS)
struct ib_ah *compat_rvt_create_ah(struct ib_pd *pd,
				   struct rdma_ah_attr *ah_attr,
				   u32 create_flags, struct ib_udata *udata);
#elif defined(CREATE_AH_HAS_UDATA)
struct ib_ah *compat_rvt_create_ah(struct ib_pd *pd, struct rdma_ah_attr *ah_attr,
			    struct ib_udata *udata);
#else
struct ib_ah *compat_rvt_create_ah(struct ib_pd *pd, struct rdma_ah_attr *ah_attr);
#endif

#ifdef HAVE_CORE_ALLOC_AH
void rvt_destroy_ah(struct ib_ah *ah, u32 destroy_flags);
#elif defined(DESTROY_AH_HAS_UDATA)
int compat_rvt_destroy_ah(struct ib_ah *ibah, u32 destroy_flags,
		   struct ib_udata *udata);
#elif defined(DESTROY_AH_HAS_FLAGS)
int compat_rvt_destroy_ah(struct ib_ah *ibah, u32 destroy_flags);
#else
int compat_rvt_destroy_ah(struct ib_ah *ibah);
#endif
int rvt_modify_ah(struct ib_ah *ibah, struct rdma_ah_attr *ah_attr);
int rvt_query_ah(struct ib_ah *ibah, struct rdma_ah_attr *ah_attr);

#endif          /* DEF_RVTAH_H */
