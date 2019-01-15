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
#include <linux/export.h>
#include <linux/thread_info.h>
#include <linux/ctype.h>
#include <linux/errno.h>
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/bug.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/pci.h>
#include "../hfi1/hfi.h"
#include "compat.h"

void msix_setup(struct pci_dev *pcidev, int pos, u32 *msixcnt,
		struct hfi1_msix_entry *hfi1_msix_entry)
{
	int ret;
	int nvec = *msixcnt;
	struct msix_entry *msix_entry;
	int i;

	/*
	 * We can't pass hfi1_msix_entry array to msix_setup
	 * so use a dummy msix_entry array and copy the allocated
	 * irq back to the hfi1_msix_entry array.
	 */
	msix_entry = kmalloc_array(nvec, sizeof(*msix_entry), GFP_KERNEL);
	if (!msix_entry) {
		ret = -ENOMEM;
		goto do_intx;
	}

	for (i = 0; i < nvec; i++)
		msix_entry[i] = hfi1_msix_entry[i].msix;

	ret = pci_enable_msix_range(pcidev, msix_entry, 1, nvec);
	if (ret < 0)
		goto free_msix_entry;
	nvec = ret;

	for (i = 0; i < nvec; i++)
		hfi1_msix_entry[i].msix = msix_entry[i];

	kfree(msix_entry);
	*msixcnt = nvec;
	return;

free_msix_entry:
	kfree(msix_entry);

do_intx:
	*msixcnt = 0;
	hfi1_enable_intx(pcidev);
}
EXPORT_SYMBOL(msix_setup);

/**
 * debugfs_use_file_start - mark the beginning of file data access
 * @dentry: the dentry object whose data is being accessed.
 * @srcu_idx: a pointer to some memory to store a SRCU index in.
 *
 * Up to a matching call to debugfs_use_file_finish(), any
 * successive call into the file removing functions debugfs_remove()
 * and debugfs_remove_recursive() will block. Since associated private
 * file data may only get freed after a successful return of any of
 * the removal functions, you may safely access it after a successful
 * call to debugfs_use_file_start() without worrying about
 * lifetime issues.
 *
 * If -%EIO is returned, the file has already been removed and thus,
 * it is not safe to access any of its data. If, on the other hand,
 * it is allowed to access the file data, zero is returned.
 *
 * Regardless of the return code, any call to
 * debugfs_use_file_start() must be followed by a matching call
 * to debugfs_use_file_finish().
 */
int debugfs_use_file_start(struct dentry *dentry, int *srcu_idx)
	__acquires(&debugfs_srcu)
{
	*srcu_idx = srcu_read_lock(&debugfs_srcu);
	barrier();
	if (d_unlinked(dentry))
		return -EIO;
	return 0;
}
EXPORT_SYMBOL(debugfs_use_file_start);

/**
 * debugfs_use_file_finish - mark the end of file data access
 * @srcu_idx: the SRCU index "created" by a former call to
 *            debugfs_use_file_start().
 *
 * Allow any ongoing concurrent call into debugfs_remove() or
 * debugfs_remove_recursive() blocked by a former call to
 * debugfs_use_file_start() to proceed and return to its caller.
 */
void debugfs_use_file_finish(int srcu_idx) __releases(&debugfs_srcu)
{
	srcu_read_unlock(&debugfs_srcu, srcu_idx);
}
EXPORT_SYMBOL(debugfs_use_file_finish);
