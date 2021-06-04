#ifndef _OPA_VNIC_DEBUGFS_H
#define _OPA_VNIC_DEBUGFS_H
/*
 * Copyright(c) 2017 Intel Corporation.
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
 * This file contains OPA VNIC Debug interface declarations
 */

#ifdef CONFIG_DEBUG_FS
void opa_vnic_dbg_ctrl_init(struct opa_vnic_ctrl_port *cport);
void opa_vnic_dbg_ctrl_exit(struct opa_vnic_ctrl_port *cport);
void opa_vnic_dbg_vport_init(struct opa_vnic_adapter *adapter);
void opa_vnic_dbg_vport_exit(struct opa_vnic_adapter *adapter);
void opa_vnic_dbg_init(void);
void opa_vnic_dbg_exit(void);
#else
static void opa_vnic_dbg_ctrl_init(struct opa_vnic_ctrl_port *cport) {}
static void opa_vnic_dbg_ctrl_exit(struct opa_vnic_ctrl_port *cport) {}
static inline void opa_vnic_dbg_vport_init(struct opa_vnic_adapter *adapter) {}
static inline void opa_vnic_dbg_vport_exit(struct opa_vnic_adapter *adapter) {}
static inline void opa_vnic_dbg_init(void) {}
static inline void opa_vnic_dbg_exit(void) {}
#endif

#endif /* _OPA_VNIC_DEBUGFS_H */
