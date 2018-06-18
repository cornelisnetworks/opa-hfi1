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

#ifndef HFI1_RC_H
#define HFI1_RC_H

/* cut down ridiculously long IB macro names */
#define OP(x) IB_OPCODE_RC_##x

static inline void update_ack_queue(struct rvt_qp *qp, unsigned int n)
{
	unsigned int next;

	next = n + 1;
	if (next > rvt_size_atomic(ib_to_rvt(qp->ibqp.device)))
		next = 0;
	qp->s_tail_ack_queue = next;
	qp->s_acked_ack_queue = next;
	qp->s_ack_state = OP(ACKNOWLEDGE);
}

static inline void rc_defered_ack(struct hfi1_ctxtdata *rcd,
				  struct rvt_qp *qp)
{
	if (list_empty(&qp->rspwait)) {
		qp->r_flags |= RVT_R_RSP_NAK;
		rvt_get_qp(qp);
		list_add_tail(&qp->rspwait, &rcd->qp_wait_list);
	}
}

static inline struct rvt_ack_entry *find_prev_entry(struct rvt_qp *qp, u32 psn,
						    u8 *prev, u8 *prev_ack,
						    bool *scheduled)
	__must_hold(&qp->s_lock)
{
	struct rvt_ack_entry *e = NULL;
	u8 i, p;
	bool s = true;

	for (i = qp->r_head_ack_queue; ; i = p) {
		if (i == qp->s_tail_ack_queue)
			s = false;
		if (i)
			p = i - 1;
		else
			p = rvt_size_atomic(ib_to_rvt(qp->ibqp.device));
		if (p == qp->r_head_ack_queue) {
			e = NULL;
			break;
		}
		e = &qp->s_ack_queue[p];
		if (!e->opcode) {
			e = NULL;
			break;
		}
		if (cmp_psn(psn, e->psn) >= 0) {
			if (p == qp->s_tail_ack_queue &&
			    cmp_psn(psn, e->lpsn) <= 0)
				s = false;
			break;
		}
	}
	if (prev)
		*prev = p;
	if (prev_ack)
		*prev_ack = i;
	if (scheduled)
		*scheduled = s;
	return e;
}

static inline u32 restart_sge(struct rvt_sge_state *ss, struct rvt_swqe *wqe,
			      u32 psn, u32 pmtu)
{
	u32 len;

	len = delta_psn(psn, wqe->psn) * pmtu;
	ss->sge = wqe->sg_list[0];
	ss->sg_list = wqe->sg_list + 1;
	ss->num_sge = wqe->wr.num_sge;
	ss->total_len = wqe->length;
	rvt_skip_sge(ss, len, false);
	return wqe->length - len;
}

int do_rc_ack(struct rvt_qp *qp, u32 aeth, u32 psn, int opcode, u64 val,
	      struct hfi1_ctxtdata *rcd);
struct rvt_swqe *do_rc_completion(struct rvt_qp *qp, struct rvt_swqe *wqe,
				  struct hfi1_ibport *ibp);
void rdma_seq_err(struct rvt_qp *qp, struct hfi1_ibport *ibp, u32 psn,
		  struct hfi1_ctxtdata *rcd);

#endif /* HFI1_RC_H */
