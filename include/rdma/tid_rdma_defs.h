/*
 * Copyright(c) 2015 - 2018 Intel Corporation.
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

#ifndef TID_RDMA_DEFS_H
#define TID_RDMA_DEFS_H

#include <rdma/ib_pack.h>

struct tid_rdma_write_req {
	__le32 kdeth0;
	__le32 kdeth1;
	struct ib_reth reth;
	__be32 reserved[2];
	__be32 verbs_qp;
} __packed;

struct tid_rdma_write_resp {
	__le32 kdeth0;
	__le32 kdeth1;
	__be32 aeth;
	__be32 reserved[3];
	__be32 tid_flow_psn;
	__be32 tid_flow_qp;
	__be32 verbs_qp;
} __packed;

struct tid_rdma_write_data {
	__le32 kdeth0;
	__le32 kdeth1;
	__be32 reserved[6];
	__be32 verbs_qp;
} __packed;

struct tid_rdma_read_req {
	__le32 kdeth0;
	__le32 kdeth1;
	struct ib_reth reth;
	__be32 tid_flow_psn;
	__be32 tid_flow_qp;
	__be32 verbs_qp;
} __packed;

struct tid_rdma_read_resp {
	__le32 kdeth0;
	__le32 kdeth1;
	__be32 aeth;
	__be32 reserved[4];
	__be32 verbs_psn;
	__be32 verbs_qp;
} __packed;

struct tid_rdma_resync {
	__le32 kdeth0;
	__le32 kdeth1;
	__be32 reserved[6];
	__be32 verbs_qp;
} __packed;

struct tid_rdma_ack {
	__le32 kdeth0;
	__le32 kdeth1;
	__be32 aeth;
	__be32 reserved[2];
	__be32 tid_flow_psn;
	__be32 verbs_psn;
	__be32 tid_flow_qp;
	__be32 verbs_qp;
} __packed;

/*
 * TID RDMA Opcodes
 */
#define IB_OPCODE_TID_RDMA 0xe0
enum {
	IB_OPCODE_WRITE_REQ       = 0x0,
	IB_OPCODE_WRITE_RESP      = 0x1,
	IB_OPCODE_WRITE_DATA      = 0x2,
	IB_OPCODE_WRITE_DATA_LAST = 0x3,
	IB_OPCODE_READ_REQ        = 0x4,
	IB_OPCODE_READ_RESP       = 0x5,
	IB_OPCODE_RESYNC          = 0x6,
	IB_OPCODE_ACK             = 0x7,

	IB_OPCODE(TID_RDMA, WRITE_REQ),
	IB_OPCODE(TID_RDMA, WRITE_RESP),
	IB_OPCODE(TID_RDMA, WRITE_DATA),
	IB_OPCODE(TID_RDMA, WRITE_DATA_LAST),
	IB_OPCODE(TID_RDMA, READ_REQ),
	IB_OPCODE(TID_RDMA, READ_RESP),
	IB_OPCODE(TID_RDMA, RESYNC),
	IB_OPCODE(TID_RDMA, ACK),
};

#define TID_OP(x) IB_OPCODE_TID_RDMA_##x

/*
 * Define TID RDMA specific WR opcodes. The ib_wr_opcode
 * enum already provides some reserved values for use by
 * low level drivers. Two of those are used but renamed
 * to be more descriptive.
 */
#define IB_WR_TID_RDMA_WRITE IB_WR_RESERVED1
#define IB_WR_TID_RDMA_READ  IB_WR_RESERVED2

#endif /* TID_RDMA_DEFS_H */
