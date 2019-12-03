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
 * This file contains HFI VNIC debug interface
 */

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/list_sort.h>
#include <linux/module.h>

#include "opa_vnic_internal.h"
#include "opa_vnic_debugfs.h"

/* MAC table string size; 64K is enough for the whole table */
#define OPA_VNIC_MACTBL_STR_SIZE  SZ_64K

/* EEPH string size */
#define OPA_VNIC_EEPH_STR_SIZE    SZ_1K

/* Vnic stats string size */
#define OPA_VNIC_STATS_STR_SIZE   SZ_1K

enum {
	OPA_VNIC_DBG_FABRIC_ID,
	OPA_VNIC_DBG_VESW_ID,
	OPA_VNIC_DBG_PKEY,
	OPA_VNIC_DBG_ETH_LINK_STATUS,
	OPA_VNIC_DBG_BASE_MAC_ADDR,
	OPA_VNIC_DBG_CFG_STATE,
	OPA_VNIC_DBG_OPER_STATE,
	OPA_VNIC_DBG_ENCAP_SLID,
	OPA_VNIC_DBG_DEF_PORT_MASK,
	OPA_VNIC_DBG_UC_DLID,
	OPA_VNIC_DBG_MC_DLID,
	OPA_VNIC_DBG_SC_UC,
	OPA_VNIC_DBG_SC_MC,
	OPA_VNIC_DBG_PCP_SC_UC,
	OPA_VNIC_DBG_PCP_SC_MC,
	OPA_VNIC_DBG_VL_UC,
	OPA_VNIC_DBG_VL_MC,
	OPA_VNIC_DBG_PCP_VL_UC,
	OPA_VNIC_DBG_PCP_VL_MC,
	OPA_VNIC_DBG_ETH_MTU,
	OPA_VNIC_DBG_ENCAP_RC,
	OPA_VNIC_DBG_RESET,
	OPA_VNIC_DBG_MACTBL_DIGEST,
	OPA_VNIC_DBG_NUM_ATTR,
};

static struct dentry *opa_vnic_dbg_root;

#define DEBUGFS_SEQ_FILE_OPS(name) \
static const struct seq_operations _##name##_seq_ops = { \
	.start = _##name##_seq_start, \
	.next  = _##name##_seq_next, \
	.stop  = _##name##_seq_stop, \
	.show  = _##name##_seq_show \
}

#define DEBUGFS_SEQ_FILE_OPEN(name) \
static int _##name##_open(struct inode *inode, struct file *file) \
{ \
	struct seq_file *seq; \
	int ret; \
	ret =  seq_open(file, &_##name##_seq_ops); \
	if (ret) \
		return ret; \
	seq = file->private_data; \
	seq->private = inode->i_private; \
	return 0; \
}

#define DEBUGFS_FILE_OPS(name) \
static const struct file_operations _##name##_file_ops = { \
	.owner   = THIS_MODULE, \
	.open    = _##name##_open, \
	.write   = _##name##_write, \
	.read    = seq_read, \
	.llseek  = seq_lseek, \
	.release = seq_release \
}

#define DEBUGFS_FILE_CREATE(name, parent, data, ops, mode)	\
do { \
	struct dentry *ent; \
	ent = debugfs_create_file(name, mode, parent, data, ops); \
	if (!ent) \
		pr_warn("create of %s failed\n", name); \
} while (0)

#define DEBUGFS_SEQ_FILE_CREATE(name, parent, data) \
	DEBUGFS_FILE_CREATE(#name, parent, data, &_##name##_file_ops, 0644)

static void *_vport_state_seq_start(struct seq_file *s, loff_t *pos)
__acquires(RCU)
{
	rcu_read_lock();
	if (*pos >= OPA_VNIC_DBG_NUM_ATTR)
		return NULL;
	return pos;
}

static void *_vport_state_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	++*pos;
	if (*pos >= OPA_VNIC_DBG_NUM_ATTR)
		return NULL;
	return pos;
}

static void _vport_state_seq_stop(struct seq_file *s, void *v)
__releases(RCU)
{
	rcu_read_unlock();
}

static int _vport_state_seq_show(struct seq_file *s, void *v)
{
	struct opa_vnic_adapter *adapter =
				(struct opa_vnic_adapter *)s->private;
	struct __opa_veswport_info *info = &adapter->info;
	loff_t *spos = v;
	int stat;
	char *str;
	u8 i, *val;

	if (v == SEQ_START_TOKEN)
		return 0;

	stat = *spos;
	switch (stat) {
	case OPA_VNIC_DBG_FABRIC_ID:
		seq_printf(s, "fabric_id (fabric id): 0x%x\n",
			   info->vesw.fabric_id);
		break;
	case OPA_VNIC_DBG_VESW_ID:
		seq_printf(s, "vesw_id (virtual eth switch id): 0x%x\n",
			   info->vesw.vesw_id);
		break;
	case OPA_VNIC_DBG_PKEY:
		seq_printf(s, "pkey (partition key): 0x%x\n",
			   info->vesw.pkey);
		break;
	case OPA_VNIC_DBG_ETH_LINK_STATUS:
		seq_printf(s, "eth_link_status (0=unknown 1=up 2=down): %u\n",
			   info->vport.eth_link_status);
		break;
	case OPA_VNIC_DBG_BASE_MAC_ADDR:
		val = info->vport.base_mac_addr;
		str = "%s: %02x:%02x:%02x:%02x:%02x:%02x\n";
		seq_printf(s, str, "base_mac_addr (mac address)",
			   val[0], val[1], val[2], val[3], val[4], val[5]);
		break;
	case OPA_VNIC_DBG_CFG_STATE:
		seq_printf(s, "config_state (0=nop 1=drop 2=init 3=fwd): %u\n",
			   info->vport.config_state);
		break;
	case OPA_VNIC_DBG_OPER_STATE:
		seq_printf(s, "oper_state (0=nop 1=drop 2=init 3=fwd): %u\n",
			   info->vport.oper_state);
		break;
	case OPA_VNIC_DBG_ENCAP_SLID:
		seq_printf(s, "encap_slid (source lid): 0x%x\n",
			   info->vport.encap_slid);
		break;
	case OPA_VNIC_DBG_DEF_PORT_MASK:
		seq_printf(s, "def_port_mask (default port mask): 0x%04x\n",
			   info->vesw.def_port_mask);
		break;
	case OPA_VNIC_DBG_UC_DLID:
		seq_puts(s, "u_ucast_dlid (unknown ucast dlid):");
		for (i = 0; i < OPA_VESW_MAX_NUM_DEF_PORT; i++)
			seq_printf(s, " 0x%x", info->vesw.u_ucast_dlid[i]);
		seq_puts(s, "\n");
		break;
	case OPA_VNIC_DBG_MC_DLID:
		seq_printf(s, "u_mcast_dlid (unknown mcast dlid): 0x%x\n",
			   info->vesw.u_mcast_dlid);
		break;
	case OPA_VNIC_DBG_ETH_MTU:
		seq_printf(s, "eth_mtu (ethernet mtu): %u\n",
			   info->vesw.eth_mtu);
		break;
	case OPA_VNIC_DBG_SC_UC:
		seq_printf(s, "non_vlan_sc_uc (non-vlan ucast sc): 0x%x\n",
			   info->vport.non_vlan_sc_uc);
		break;
	case OPA_VNIC_DBG_SC_MC:
		seq_printf(s, "non_vlan_sc_mc (non-vlan mcast sc): 0x%x\n",
			   info->vport.non_vlan_sc_mc);
		break;
	case OPA_VNIC_DBG_PCP_SC_UC:
		val = info->vport.pcp_to_sc_uc;
		str = "%s: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n";
		seq_printf(s, str, "pcp_to_sc_uc (vlan ucast sc)",
			   val[0], val[1], val[2], val[3],
			   val[4], val[5], val[6], val[7]);
		break;
	case OPA_VNIC_DBG_PCP_SC_MC:
		val = info->vport.pcp_to_sc_mc;
		str = "%s: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n";
		seq_printf(s, str, "pcp_to_sc_mc (vlan mcast sc)",
			   val[0], val[1], val[2], val[3],
			   val[4], val[5], val[6], val[7]);
		break;
	case OPA_VNIC_DBG_VL_UC:
		seq_printf(s, "non_vlan_vl_uc (non-vlan ucast vl): 0x%x\n",
			   info->vport.non_vlan_vl_uc);
		break;
	case OPA_VNIC_DBG_VL_MC:
		seq_printf(s, "non_vlan_vl_mc (non-vlan mcast vl): 0x%x\n",
			   info->vport.non_vlan_vl_mc);
		break;
	case OPA_VNIC_DBG_PCP_VL_UC:
		val = info->vport.pcp_to_vl_uc;
		str = "%s: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n";
		seq_printf(s, str, "pcp_to_vl_uc (vlan ucast vl)",
			   val[0], val[1], val[2], val[3],
			   val[4], val[5], val[6], val[7]);
		break;
	case OPA_VNIC_DBG_PCP_VL_MC:
		val = info->vport.pcp_to_vl_mc;
		str = "%s: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n";
		seq_printf(s, str, "pcp_to_vl_mc (vlan mcast vl)",
			   val[0], val[1], val[2], val[3],
			   val[4], val[5], val[6], val[7]);
		break;
	case OPA_VNIC_DBG_ENCAP_RC:
		seq_printf(s, "encap_rc (routing control): 0x%08x\n",
			   info->vesw.rc);
		break;
	case OPA_VNIC_DBG_MACTBL_DIGEST:
		seq_printf(s, "MAC Table Digest: %u\n",
			   info->vport.mac_tbl_digest);
		break;
	default:
		return SEQ_SKIP;
	}

	return 0;
}

static ssize_t _vport_state_write(struct file *file, const char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct seq_file *s = (struct seq_file *)file->private_data;
	struct opa_vnic_adapter *adapter =
				(struct opa_vnic_adapter *)s->private;
	struct __opa_veswport_info *info = &adapter->info;
	char debug_buf[256];
	ssize_t len;
	u32 value;
	int cnt;

	if (*ppos != 0)
		return 0;

	if (count >= sizeof(debug_buf))
		return -ENOSPC;

	len = simple_write_to_buffer(debug_buf, sizeof(debug_buf) - 1,
				     ppos, buf, count);
	if (len < 0)
		return len;

	debug_buf[len] = '\0';
	if (strncmp(debug_buf, "fabric_id", 9) == 0) {
		cnt = kstrtouint(strim(&debug_buf[9]), 0, &value);
		if (!cnt)
			info->vesw.fabric_id = value;
	} else if (strncmp(debug_buf, "vesw_id", 7) == 0) {
		cnt = kstrtouint(strim(&debug_buf[7]), 0, &value);
		if (!cnt)
			info->vesw.vesw_id = value;
	} else if (strncmp(debug_buf, "pkey", 4) == 0) {
		cnt = kstrtouint(strim(&debug_buf[4]), 0, &value);
		if (!cnt)
			info->vesw.pkey = value;
	} else if (strncmp(debug_buf, "base_mac_addr", 13) == 0) {
		u8 i, mac[6];

		cnt = sscanf(&debug_buf[13], "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
			     &mac[0], &mac[1], &mac[2],
			     &mac[3], &mac[4], &mac[5]);
		if (cnt == 6)
			for (i = 0; i < 6; i++)
				info->vport.base_mac_addr[i] = mac[i];
	} else if (strncmp(debug_buf, "config_state", 12) == 0) {
		cnt = kstrtouint(strim(&debug_buf[12]), 0, &value);
		if (!cnt)
			info->vport.config_state = value;
	} else if (strncmp(debug_buf, "encap_slid", 10) == 0) {
		cnt = kstrtouint(strim(&debug_buf[10]), 0, &value);
		if (!cnt)
			info->vport.encap_slid = value;
	} else if (strncmp(debug_buf, "def_port_mask", 13) == 0) {
		cnt = kstrtouint(strim(&debug_buf[13]), 16, &value);
		if (!cnt)
			info->vesw.def_port_mask = value;
	} else if (strncmp(debug_buf, "u_ucast_dlid", 12) == 0) {
		u32 idx;
		int val;

		cnt = sscanf(&debug_buf[12], "%u %i", &idx, &val);
		if ((cnt == 2) && (idx < OPA_VESW_MAX_NUM_DEF_PORT))
			info->vesw.u_ucast_dlid[idx] = (u32)val;
	} else if (strncmp(debug_buf, "u_mcast_dlid", 12) == 0) {
		cnt = kstrtouint(strim(&debug_buf[12]), 0, &value);
		if (!cnt)
			info->vesw.u_mcast_dlid = value;
	} else if (strncmp(debug_buf, "eth_mtu", 7) == 0) {
		cnt = kstrtouint(strim(&debug_buf[7]), 0, &value);
		if (!cnt)
			info->vesw.eth_mtu = value;
	} else if (strncmp(debug_buf, "non_vlan_sc_uc", 14) == 0) {
		cnt = kstrtouint(strim(&debug_buf[14]), 0, &value);
		if (!cnt)
			info->vport.non_vlan_sc_uc = value;
	} else if (strncmp(debug_buf, "non_vlan_sc_mc", 14) == 0) {
		cnt = kstrtouint(strim(&debug_buf[14]), 0, &value);
		if (!cnt)
			info->vport.non_vlan_sc_mc = value;
	} else if (strncmp(debug_buf, "pcp_to_sc_uc", 12) == 0) {
		int i, sc[8];

		cnt = sscanf(&debug_buf[12], "%i %i %i %i %i %i %i %i",
			     &sc[0], &sc[1], &sc[2], &sc[3],
			     &sc[4], &sc[5], &sc[6], &sc[7]);
		if (cnt == 8)
			for (i = 0; i < 8; i++)
				info->vport.pcp_to_sc_uc[i] = sc[i];
	} else if (strncmp(debug_buf, "pcp_to_sc_mc", 12) == 0) {
		int i, sc[8];

		cnt = sscanf(&debug_buf[12], "%i %i %i %i %i %i %i %i",
			     &sc[0], &sc[1], &sc[2], &sc[3],
			     &sc[4], &sc[5], &sc[6], &sc[7]);
		if (cnt == 8)
			for (i = 0; i < 8; i++)
				info->vport.pcp_to_sc_mc[i] = sc[i];
	} else if (strncmp(debug_buf, "non_vlan_vl_uc", 14) == 0) {
		cnt = kstrtouint(strim(&debug_buf[14]), 0, &value);
		if (!cnt)
			info->vport.non_vlan_vl_uc = value;
	} else if (strncmp(debug_buf, "non_vlan_vl_mc", 14) == 0) {
		cnt = kstrtouint(strim(&debug_buf[14]), 0, &value);
		if (!cnt)
			info->vport.non_vlan_vl_mc = value;
	} else if (strncmp(debug_buf, "pcp_to_vl_uc", 12) == 0) {
		int i, vl[8];

		cnt = sscanf(&debug_buf[12], "%i %i %i %i %i %i %i %i",
			     &vl[0], &vl[1], &vl[2], &vl[3],
			     &vl[4], &vl[5], &vl[6], &vl[7]);
		if (cnt == 8)
			for (i = 0; i < 8; i++)
				info->vport.pcp_to_vl_uc[i] = vl[i];
	} else if (strncmp(debug_buf, "pcp_to_vl_mc", 12) == 0) {
		int i, vl[8];

		cnt = sscanf(&debug_buf[12], "%i %i %i %i %i %i %i %i",
			     &vl[0], &vl[1], &vl[2], &vl[3],
			     &vl[4], &vl[5], &vl[6], &vl[7]);
		if (cnt == 8)
			for (i = 0; i < 8; i++)
				info->vport.pcp_to_vl_mc[i] = vl[i];
	} else if (strncmp(debug_buf, "encap_rc", 8) == 0) {
		cnt = kstrtouint(strim(&debug_buf[8]), 16, &value);
		if (!cnt)
			info->vesw.rc = value;
	} else if (strncmp(debug_buf, "reset", 5) == 0) {
		struct opa_veswport_info port_info;

		vema_get_pod_values(&port_info);
		opa_vnic_set_vesw_info(adapter, &port_info.vesw);
		opa_vnic_set_per_veswport_info(adapter, &port_info.vport);
		opa_vnic_release_mac_tbl(adapter);
	}

	/* process the new config settings */
	opa_vnic_process_vema_config(adapter);
	return count;
}

DEBUGFS_SEQ_FILE_OPS(vport_state);
DEBUGFS_SEQ_FILE_OPEN(vport_state)
DEBUGFS_FILE_OPS(vport_state);

static ssize_t mac_tbl_read(struct file *file, char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct opa_vnic_adapter *adapter = file_inode(file)->i_private;
	struct opa_veswport_mactable *tbl;
	char *debug_buf;
	ssize_t len = 0;
	int i, extra, rc = 0;

	if (*ppos != 0)
		return 0;

	if (count < OPA_VNIC_MACTBL_STR_SIZE)
		return -ENOSPC;

	debug_buf = kmalloc(OPA_VNIC_MACTBL_STR_SIZE, GFP_KERNEL);
	if (!debug_buf)
		return -ENOMEM;

	/* allocate veswport mac table */
	extra = sizeof(struct opa_veswport_mactable_entry) *
					OPA_VNIC_MAC_TBL_MAX_ENTRIES;
	tbl = kzalloc(sizeof(*tbl) + extra, GFP_KERNEL);
	if (!tbl) {
		rc = -ENOMEM;
		goto read_done;
	}

	/* get the whole table */
	tbl->offset = cpu_to_be16(0);
	tbl->num_entries = cpu_to_be16(OPA_VNIC_MAC_TBL_MAX_ENTRIES);

	opa_vnic_query_mac_tbl(adapter, tbl);
	len += scnprintf(debug_buf + len, OPA_VNIC_MACTBL_STR_SIZE - len,
			 "Mac Table Digest: %u\n",
			 be32_to_cpu(tbl->mac_tbl_digest));
	for (i = 0; i < OPA_VNIC_MAC_TBL_MAX_ENTRIES; i++) {
		struct opa_veswport_mactable_entry *entry =
							&tbl->tbl_entries[i];
		u8 *mac_addr = entry->mac_addr;
		u8 empty_mac[ETH_ALEN] = { 0 };

		/* if the entry is not there (null), skip */
		if (!memcmp(mac_addr, empty_mac, ARRAY_SIZE(empty_mac)))
			continue;

		len += scnprintf(debug_buf + len,
				 OPA_VNIC_MACTBL_STR_SIZE - len,
				 "%4d: %02x:%02x:%02x:%02x:%02x:%02x  0x%x\n",
				 i, mac_addr[0], mac_addr[1], mac_addr[2],
				 mac_addr[3], mac_addr[4], mac_addr[5],
				 be32_to_cpu(entry->dlid_sd));
	}
	kfree(tbl);

	rc = simple_read_from_buffer(buf, count, ppos, debug_buf, len);
read_done:
	kfree(debug_buf);
	return rc;
}

static ssize_t mac_tbl_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	struct opa_vnic_adapter *adapter = file_inode(file)->i_private;
	struct opa_veswport_mactable *tbl;
	int i, extra, cnt, rc, num_bytes;
	u32 offset, num_entries, digest;
	char *debug_buf, *buf_ptr;
	ssize_t len;

	if (*ppos != 0)
		return 0;

	if (count >= OPA_VNIC_MACTBL_STR_SIZE)
		return -ENOSPC;

	debug_buf = kmalloc(count + 1, GFP_KERNEL);
	if (!debug_buf)
		return -ENOMEM;

	rc = simple_write_to_buffer(debug_buf, count, ppos, buf, count);
	if (rc < 0)
		goto write_err;

	len = rc;
	debug_buf[len] = '\0';

	/* read offset and number of entries */
	buf_ptr = debug_buf;
	cnt = sscanf(buf_ptr, "%u %u %u %n", &digest, &offset, &num_entries,
		     &num_bytes);
	if ((cnt != 3) || !num_entries ||
	    ((offset + num_entries) > OPA_VNIC_MAC_TBL_MAX_ENTRIES)) {
		v_err("Invalid input\n");
		rc = -EINVAL;
		goto write_err;
	}

	/* allocate veswport mac table */
	extra = sizeof(struct opa_veswport_mactable_entry) * num_entries;
	tbl = kzalloc(sizeof(*tbl) + extra, GFP_KERNEL);
	if (!tbl) {
		rc = -ENOMEM;
		goto write_err;
	}

	/* build the veswport mac table */
	tbl->mac_tbl_digest = cpu_to_be32(digest);
	tbl->offset = cpu_to_be16(offset);
	tbl->num_entries = cpu_to_be16(num_entries);
	for (i = 0; i < num_entries; i++) {
		struct opa_veswport_mactable_entry *entry =
							&tbl->tbl_entries[i];
		u8 *mac_addr = entry->mac_addr;
		u32 dlid_sd;

		buf_ptr += num_bytes;
		cnt = sscanf(buf_ptr, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx  %x%n",
			     &mac_addr[0], &mac_addr[1], &mac_addr[2],
			     &mac_addr[3], &mac_addr[4], &mac_addr[5],
			     &dlid_sd, &num_bytes);
		if (cnt != 7)  {
			v_err("Invalid input\n");
			rc = -EINVAL;
			goto write_invalid;
		}
		entry->dlid_sd = cpu_to_be32(dlid_sd);
	}

	/* update mac table */
	rc = opa_vnic_update_mac_tbl(adapter, tbl);
write_invalid:
	kfree(tbl);
	rc = rc ? : count;
write_err:
	kfree(debug_buf);
	return rc;
}

static const struct file_operations mac_tbl_file_ops = {
	.owner   = THIS_MODULE,
	.write   = mac_tbl_write,
	.read    = mac_tbl_read,
};

void opa_vnic_dbg_vport_init(struct opa_vnic_adapter *adapter)
{
	struct net_device *dev = adapter->netdev;

	if (!opa_vnic_dbg_root)
		return;

	adapter->dentry  = debugfs_create_dir(dev->name,
					      opa_vnic_dbg_root);
	if (!adapter->dentry) {
		pr_warn("init of opa vnic debugfs failed\n");
		return;
	}

	DEBUGFS_SEQ_FILE_CREATE(vport_state, adapter->dentry, adapter);
	DEBUGFS_FILE_CREATE("mac_tbl", adapter->dentry, adapter,
			    &mac_tbl_file_ops, 0644);
}

void opa_vnic_dbg_vport_exit(struct opa_vnic_adapter *adapter)
{
	debugfs_remove_recursive(adapter->dentry);
}

static int ctrl_add_vport_set(void *data, u64 val)
{
	struct opa_vnic_vema_port *port = data;
	struct opa_vnic_adapter *adapter;
	u8 vport = (u8)val;

	adapter = vema_add_vport(port, vport);
	return IS_ERR(adapter);
}

DEFINE_SIMPLE_ATTRIBUTE(ctrl_add_vport, NULL, ctrl_add_vport_set, "%llu\n");

void opa_vnic_dbg_ctrl_init(struct opa_vnic_ctrl_port *cport)
{
	struct opa_vnic_vema_port *port;
	int i;

	if (!opa_vnic_dbg_root)
		return;

	for (i = 1; i <= cport->num_ports; i++) {
		char name[255];

		port = vema_get_port(cport, i);
		if (port->dentry)
			continue;
		snprintf(name, sizeof(name), "%s.%02x",
			 dev_name(&cport->ibdev->dev), i);
		port->dentry = debugfs_create_dir(name, opa_vnic_dbg_root);
		if (port->dentry)
			DEBUGFS_FILE_CREATE("add_vport", port->dentry, port,
					    &ctrl_add_vport, 0200);
	}
}

void opa_vnic_dbg_ctrl_exit(struct opa_vnic_ctrl_port *cport)
{
	struct opa_vnic_vema_port *port;
	int i;

	for (i = 1; i <= cport->num_ports; i++) {
		port = vema_get_port(cport, i);
		debugfs_remove_recursive(port->dentry);
		port->dentry = NULL;
	}
}

void opa_vnic_dbg_init(void)
{
	opa_vnic_dbg_root = debugfs_create_dir(opa_vnic_driver_name, NULL);
	if (IS_ERR(opa_vnic_dbg_root))
		opa_vnic_dbg_root = NULL;
	if (!opa_vnic_dbg_root)
		pr_warn("init of hfi vnic debugfs failed\n");
}

void opa_vnic_dbg_exit(void)
{
	debugfs_remove_recursive(opa_vnic_dbg_root);
	opa_vnic_dbg_root = NULL;
}
