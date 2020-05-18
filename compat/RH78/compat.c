/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Copyright(c) 2020 Intel Corporation.
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
