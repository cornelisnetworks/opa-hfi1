/*
 * Copyright(c) 2015 - 2017 Intel Corporation.
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
#include "iowait.h"
#include "trace_iowait.h"

/* 1 priority == 16 starve_cnt */
#define IOWAIT_PRIORITY_STARVE_SHIFT 4

void iowait_set_flag(struct iowait *wait, u32 flag)
{
	trace_hfi1_iowait_set(wait, flag);
	set_bit(flag, &wait->flags);
}

bool iowait_flag_set(struct iowait *wait, u32 flag)
{
	return test_bit(flag, &wait->flags);
}

inline void iowait_clear_flag(struct iowait *wait, u32 flag)
{
	trace_hfi1_iowait_clear(wait, flag);
	clear_bit(flag, &wait->flags);
}

/**
 * iowait_init() - initialize wait structure
 * @wait: wait struct to initialize
 * @tx_limit: limit for overflow queuing
 * @func: restart function for workqueue
 * @sleep: sleep function for no space
 * @resume: wakeup function for no space
 *
 * This function initializes the iowait
 * structure embedded in the QP or PQ.
 *
 */
void iowait_init(struct iowait *wait, u32 tx_limit,
		 void (*func)(struct work_struct *work),
		 void (*tidfunc)(struct work_struct *work),
		 int (*sleep)(struct sdma_engine *sde,
			      struct iowait_work *wait,
			      struct sdma_txreq *tx,
			      uint seq,
			      bool pkts_sent),
		 void (*wakeup)(struct iowait *wait, int reason),
		 void (*sdma_drained)(struct iowait *wait),
		 void (*init_priority)(struct iowait *wait))
{
	int i;

	wait->count = 0;
	INIT_LIST_HEAD(&wait->list);
	init_waitqueue_head(&wait->wait_dma);
	init_waitqueue_head(&wait->wait_pio);
	atomic_set(&wait->sdma_busy, 0);
	atomic_set(&wait->pio_busy, 0);
	wait->tx_limit = tx_limit;
	wait->sleep = sleep;
	wait->wakeup = wakeup;
	wait->sdma_drained = sdma_drained;
	wait->init_priority = init_priority;
	wait->flags = 0;
	for (i = 0; i < IOWAIT_SES; i++) {
		wait->wait[i].iow = wait;
		INIT_LIST_HEAD(&wait->wait[i].tx_head);
		if (i == IOWAIT_IB_SE)
			INIT_WORK(&wait->wait[i].iowork, func);
		else
			INIT_WORK(&wait->wait[i].iowork, tidfunc);
	}
}

/**
 * iowait_cancel_work - cancel all work in iowait
 * @w: the iowait struct
 */
void iowait_cancel_work(struct iowait *w)
{
	cancel_work_sync(&iowait_get_ib_work(w)->iowork);
	cancel_work_sync(&iowait_get_tid_work(w)->iowork);
}

/**
 * iowait_set_work_flag - set work flag based on leg
 * @w - the iowait work struct
 */
int iowait_set_work_flag(struct iowait_work *w)
{
	if (w == &w->iow->wait[IOWAIT_IB_SE]) {
		iowait_set_flag(w->iow, IOWAIT_PENDING_IB);
		return IOWAIT_IB_SE;
	}
	iowait_set_flag(w->iow, IOWAIT_PENDING_TID);
	return IOWAIT_TID_SE;
}

/**
 * iowait_priority_update_top - update the top priority entry
 * @w: the iowait struct
 * @top: a pointer to the top priority entry
 * @idx: the index of the current iowait in an array
 * @top_idx: the array index for the iowait entry that has the top priority
 *
 * This function is called to compare the priority of a given
 * iowait with the given top priority entry. The top index will
 * be returned.
 */
uint iowait_priority_update_top(struct iowait *w,
				struct iowait *top,
				uint idx, uint top_idx)
{
	u8 cnt, tcnt;

	/* Convert priority into starve_cnt and compare the total.*/
	cnt = (w->priority << IOWAIT_PRIORITY_STARVE_SHIFT) + w->starved_cnt;
	tcnt = (top->priority << IOWAIT_PRIORITY_STARVE_SHIFT) +
		top->starved_cnt;
	if (cnt > tcnt)
		return idx;
	else
		return top_idx;
}
