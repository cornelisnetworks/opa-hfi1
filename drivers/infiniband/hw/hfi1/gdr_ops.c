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
 * This file includes code obtained from: https://github.com/NVIDIA/gdrcopy/
 * under the following copyright and license.
 *
 * Copyright (c) 2014, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 */
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/vmalloc.h>
#include <linux/io.h>
#include <linux/aio.h>
#include <linux/bitmap.h>
#include <linux/file.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/sched.h>
#include <linux/io.h>

#include <rdma/ib.h>

#include "mmu_rb.h"
#include "hfi.h"
#include "device.h"
#include "trace.h"
#include "nv-p2p.h"
#include "gpu.h"
#include "gdr_ops.h"

static int gdr_major;
static struct class *gdr_class;
static struct device *gdr_device;
static dev_t gdr_dev;

#define GDR_DRIVER_NAME "hfi1_gdr_chr"
#define GDR_CLASS_NAME "hfi1_gdr"
#define GDR_DEV_NAME "hfi1_gdr"

/**
 * struct hfi1_gdrdata - Private data for gdr operations driver
 * @ioctl_busy_flag - an atomic used to serialize gdr device ioctl operations.
 * @gdr_handler - Head of a red/black tree of gpu memory buffer descriptors
 * @map_this_mr - A memory descriptor pointer to be mmapped by this driver.
 * @mr_lock - a mutex for resolving races with the Nvidia callback handler.
 * @gdr_wq - A work queue to delete nodes from the delete list.
 * @n_pages_locked - Total number of pages pinned in the cache.
 * @lru_list - Head of the LRU list.
 *
 * A file descriptor's private_data field point to one of these structures.
 * Most of the state in this structure is manipulated through this driver's
 * ioctl() system call handler.
 *
 * It maintains a red/black tree and mutex lock for all the pinned
 * and mmapped gpu buffers created.
 *
 * The ioctl_busy_flag is used to ensure that only one thread
 * at a time is executing in this ioctl() handler.  Any thread that enters
 * this ioctl() handler while it is "busy", will receive an -EINVAL error code.
 * This means there is no concurrent execution of most of the code in this
 * driver.  This is acceptable because the primary user of this driver is PSM,
 * and the core of PSM is single-threaded.
 *
 * The mr_lock mutex serializes access to this red/black tree as well
 * as all state information for each of the pinned GPU buffers.
 *
 * map_this_mr is used as a way to efficiently communicate across
 * the vm_mmap() function call in do_pin_and_mmap_gpu_buf() to this
 * driver's mmap handler function hfi1_gdr_mmap(), a pointer to a
 * gdr_mr struct that describes the GPU buffer that is to be mmapped
 * into the process's user virtual adddress space.
 */
struct hfi1_gdrdata {
	atomic_t ioctl_busy_flag;
	struct mmu_rb_handler *gdr_handler;
	void *map_this_mr;	/* consider replacing with a linked list */
	struct mutex mr_lock;
	struct workqueue_struct *gdr_wq;
	unsigned long n_pages_locked;
	struct list_head lru_list;
};

/**
 * struct gdr_mr - describes a gpu buffer and it's pinned and mmaped state.
 * @rb_node: This memory region's place holder in the RB tree of cached
 *	     memory region structures. This node contains the starting address
 *	     and length of a gpu buffer.
 * @list: List entry for adding to a list.
 * @host_addr: if not NULL, it is the host address mapping of this gpu buffer.
 * @mr_handle: a pseudo-random number used by this driver's mmap_handler
 * @page_table: When a GPU buf is pinned this points to that GPU buf's
 *		NVidia page table.
 * @gd: A pointer to the hfi1_gdrdata this structure is linked to.
 * @ref_cnt: A reference count on this structure.
 *
 * Each struct gdr_mr represents a mmapped and pinned gpu buffer.
 * The hfi1_gdrdata structure for this driver contains the head of
 * a read/black tree of these these structures.  Access to and modification
 * of the read/black tree and the content of structures in this structure
 * are controlled by the mr_lock mutex in the hfi1_gdrdata structure.
 */
struct gdr_mr {
	struct mmu_rb_node rb_node;
	struct list_head list;
	u64 host_addr;
	u32 mr_handle;
	nvidia_p2p_page_table_t *page_table;
	struct hfi1_gdrdata *gd;
	struct kref ref_cnt;
};

/**
 * gdrdrv_munmap() - unmap a pinned gpu page from process's address space.
 * @mr: - A pointer to a struct gdr_mr describing the gpu buf being unmapped.
 *
 * Unmap host_addr from the user's address space.
 *
 * This could be called during process exit after a user-mode SEGFAULT.
 * Under this circumstance, we must not call vm_munmap(), otherwise the kernel
 * will segfault and panic.
 *
 */
static inline void gdrdrv_munmap(struct gdr_mr *mr)
{
	int unmap_ret = 0;

	if (mr->host_addr && !(current->flags & PF_EXITING)) {
		unmap_ret = vm_munmap(mr->host_addr, mr->rb_node.len);
		WARN_ON(unmap_ret);
	}
	mr->host_addr = 0;
}

static inline void
mr_complete(struct kref *kref)
{
	struct gdr_mr *mr;
	struct hfi1_gdrdata *gd;

	mr = container_of(kref, struct gdr_mr, ref_cnt);
	gd = mr->gd;
	WARN_ON(gd->map_this_mr == mr);
	hfi1_mmu_rb_remove(mr->gd->gdr_handler, &mr->rb_node);
	list_del(&mr->list);
	kfree(mr);
	return;
}

static inline void
acquire_callback_mr_ref(struct gdr_mr *mr)
{
	kref_get(&mr->ref_cnt);
}

static inline void
release_callback_mr_ref(struct gdr_mr *mr)
{
	kref_put(&mr->ref_cnt, mr_complete);
}

static inline void
acquire_ioctl_mr_ref(struct gdr_mr *mr)
{
	kref_get(&mr->ref_cnt);
}

static inline void
release_ioctl_mr_ref(struct gdr_mr *mr)
{
	kref_put(&mr->ref_cnt, mr_complete);
}

/**
 * handle_to_offset() - convert a 32-bit number to a vm_mmap() offset argument.
 * @handle: an unsigned 32 bit psuedo random value.
 *
 * The 32 bit psuedo-random handle value is used to uniquely label a
 * struct gdr_mr, for use by this driver's mmap handler.
 *
 * This shifts left the 32-bit handle value to a "page-aligned" value that
 * can be passed as the offset argument to vm_mmap().  The vm_mmap()
 * function then shifts its offset argument 12 bits to the right
 * before passing assigning to the vm_pgoff member of the vm_area struct
 * that is passed to the mmap handler.
 *
 * This way, the vm_pgoff member contains the original 32-bit handle
 * value.
 *
 * Return: A value that can be passed as an offset into a vm_mmap() call.
 */
static inline off_t
handle_to_offset(u32 handle)
{
	return (off_t)handle << PAGE_SHIFT;
}

/**
 * handle_from_vm_pgoff() - convert a vm_pgoff value to a 32-bit handle.
 * @pgoff: a page-aligned value passed into this driver's mmap handler.
 *
 * Convert the vm_pagoff value from the mmap handler's vm_area_struct into
 * a 32-bit handle value.
 *
 * Return: The unique 32-bit pseudo random value from the vm_mmap() argument.
 */
static inline u32
handle_from_vm_pgoff(unsigned long pgoff)
{
	return (u32)pgoff;
}

/**
 * get_random_handle()
 *
 * Generate a pseudo-random handle value, to be used during mmap operations.
 *
 * Return: a 32-bit pseudo random value.
 */
static inline u32
get_random_handle(void)
{
	return (u32)get_cycles();
}

/*
 * File operation functions
 */
static int hfi1_gdr_open(struct inode *inode, struct file *fp);
static int hfi1_gdr_release(struct inode *inode, struct file *fp);
static int hfi1_gdr_mmap(struct file *fp, struct vm_area_struct *vma);
static long hfi1_gdr_ioctl(struct file *fp, unsigned int cmd,
			    unsigned long arg);

static const struct file_operations hfi1_gdr_ops = {
	.owner = THIS_MODULE,
	.open = hfi1_gdr_open,
	.release = hfi1_gdr_release,
	.unlocked_ioctl = hfi1_gdr_ioctl,
	.mmap = hfi1_gdr_mmap,
	.llseek = noop_llseek,
};

static bool gdr_rb_filter(struct mmu_rb_node *node, unsigned long addr,
			  unsigned long len)
{
	return (bool)(node->addr == addr);
}

static int gdr_rb_insert(void *arg, struct mmu_rb_node *mnode)
{
	return 0;
}

static int gdr_rb_evict(void *arg, struct mmu_rb_node *mnode,
			void *evict_arg, bool *stop)
{
	return 0;
}

static void gdr_rb_remove(void *arg, struct mmu_rb_node *mnode)
{
}

static int gdr_rb_invalidate(void *arg, struct mmu_rb_node *mnode)
{
	return 0;
}

static struct mmu_rb_ops gdr_rb_ops = {
	.filter = gdr_rb_filter,
	.insert = gdr_rb_insert,
	.evict = gdr_rb_evict,
	.remove = gdr_rb_remove,
	.invalidate = gdr_rb_invalidate
};

/**
 * hfi1_gdr_open() - the open file_operations handler for this driver.
 * @inode: Pointer to an inode for this special file.
 * @filep: Pointer to an open file structure for this open instance.
 *
 * Allocate and initialize a hfi1_gdrdata structure.
 *
 * Return:
 * 0 -	success, -ENOMEM for a failed memory allocation.
 */
static int hfi1_gdr_open(struct inode *inode, struct file *filep)
{
	int ret = 0;
	struct hfi1_gdrdata *gd;

	gd = kzalloc(sizeof(*gd), GFP_KERNEL);
	if (!gd)
		return -ENOMEM;

	filep->private_data = gd;
	mutex_init(&gd->mr_lock);
	ret = hfi1_mmu_rb_register_gpu(gd, &gdr_rb_ops,
				       gd->gdr_wq, &gd->gdr_handler);
	if (ret) {
		filep->private_data = NULL;
		kfree(gd);
		return ret;
	}

	INIT_LIST_HEAD(&gd->lru_list);
	gd->n_pages_locked = 0;

	return ret;
}

/**
 * gdrdrv_get_pages_free_callback() - Callback handler for unpinning a GPU buf.
 * @data: - A pointer to a struct gdr_mr describing the gpu buf being freed.
 *
 * This is a callback function that the Nvidia driver calls when
 * a user frees a GPU buffer that has been pinned. This function unmaps and
 * unpins that gpu buffer.
 *
 * The GPU buffer can ALSO be unmmapped and unpinned through the
 * HFI1_IOCTL_GDR_GPU_MUNMAP_UNPIN ioctl, calling the nvidia_p2p_put_pages()
 * function. It's possible for that ioctl() operation to race with this
 * callback.
 *
 * Ultimately, the nvidia_p2p_put_pages() function determines which code
 * path wins this race.  If nvidia_p2p_put_pages() "wins", then
 * this callback function will not be called for that GPU buffer.  If this
 * callback handler "wins", then nvidia_p2p_put_pages() will fail with an
 * -EINVAL error code.
 *
 *  Both this ioctl() function and this callback handler acquire
 *  the mr_lock mutex to serialize operations on the memory mapping
 *  of this GPU buf. So it's important that the ioctl() function release
 *  this mr_lock BEFORE calling nvidia_p2p_put_pages(), otherwise that
 *  ioctl() function could deadlock on that mutex with this callback handler.
 *
 *  mr->page_table SHOULD NEVER be NULL when this function is entered.
 *  If that happens, it indicates a bug either in this driver, or in
 *  the NVidia driver.  But out of a sense of parania, we WARN on this
 *  case, and call free_gpu_page_table() ONLY when this pointer is NOT
 *  NULL.
 */
static void gdrdrv_get_pages_free_callback(void *data)
{
	struct hfi1_gdrdata *gd;
	struct gdr_mr *mr = data;
	unsigned int npages;

	gd = mr->gd;
	mutex_lock(&gd->mr_lock);
	gdrdrv_munmap(mr);

	/*
	 * mr->page_table SHOULD NEVER be NULL in this code, but see comment
	 * above about paranoia.
	 */
	WARN_ON(!mr->page_table);
	if (mr->page_table) {
		free_gpu_page_table(mr->page_table);
		mr->page_table = NULL;
		npages = num_user_pages_gpu(mr->rb_node.addr, mr->rb_node.len);
		gd->n_pages_locked -= npages;
	}
	release_callback_mr_ref(mr);
	mutex_unlock(&gd->mr_lock);
}

/**
 * do_munmap_and_unpin_gpu_buf() - unpin and unmap a gpu buffer.
 * @mr: A pointer to a struct gdr_mr for the gpu buffer to be pinned/mmapped
 *
 * This is called when the user has requested a GPU buffer to be unpinned,
 * or also at various places in this driver to unwind from an error situation,
 * or to "evict" an entry from the cache.
 *
 * This function can race with gdrdrv_get_pages_free_callback(). See comments
 * below for details on how that race is resolved.
 *
 * The caller of this function must hold an "ioctl" reference on this mr.
 * This function releases that "ioctl" reference.  It may also under
 * some cirumstances release the "callback" reference.
 *
 * This mr structure MAY be freed during the releasing of these references.
 */
static void
do_munmap_and_unpin_gpu_buf(struct gdr_mr *mr)
{
	struct hfi1_gdrdata *gd = mr->gd;
	unsigned int npages;
	int ret;

	mutex_lock(&gd->mr_lock);
	gdrdrv_munmap(mr);
	mutex_unlock(&gd->mr_lock);
	/*
	 * This function call can race with gdrdrv_get_pages_free_callback().
	 * The Nvidia driver's nvidia_p2p_put_pages() function serializes these
	 * events. nvidia_p2p_put_pages() has three expected return values:
	 *
	 *	0 - nvidia_p2p_put_pages() is successful, indicating
	 *	    that nvidia_p2p_put_pages() has unpinned this GPU buffer,
	 *	    and the gdrdrv_get_pages_free_callback() has been
	 *	    unregistered for that GPU buffer.
	 *
	 *	-EINVAL - nvidia_p2p_put_pages() lost the race with
	 *		  gdrdrv_get_pages_free_callback().  The callback
	 *		  handler unmapped and unpinned the GPU buffer.
	 *
	 *	-EIO - This is an "unknown" error.
	 *
	 * If gdrdrv_get_pages_free_callback() executes first, then it will
	 * unpin this buffer (it was already unmmapped above), and
	 * it will release the "callback mr reference".  In this case
	 * the nvidia_p2p_put_pages() will return -EINVAL, so in this
	 * case we need only release the "ioctl mr reference".
	 *
	 * If nvidia_p2p_put_pages() executes first, then it will unpin
	 * this GPU buffer and also unregister the free callback handler.
	 * So the free callback handler will never be called for this
	 * gdr_mr. In this case, nvidia_p2p_put_pages() returns 0 value
	 * "sucess". This means this code path needs to also release the
	 * "callback mr reference".
	 *
	 * In the case where nvidia_p2p_put_pages() succeeds, the "free
	 * callback handler" does NOT get called for this GPU buffer.
	 * nvidia_p2p_put_pages() deregisters the "free callback" handler
	 * from this GPU buffer.
	 */
	ret = nvidia_p2p_put_pages(0, 0, mr->rb_node.addr, mr->page_table);
	mutex_lock(&gd->mr_lock);
	npages = num_user_pages_gpu(mr->rb_node.addr, mr->rb_node.len);
	gd->n_pages_locked -= npages;
	release_ioctl_mr_ref(mr);
	if (!ret) {
		mr->page_table = NULL;
		release_callback_mr_ref(mr);
	}

	mutex_unlock(&gd->mr_lock);
}

/**
 * create_mr() - create a new memory region
 * @gd: A pointer to the hfi1_gdrdata for this open file descriptor.
 * @gpu_buf_addr: GPU buffer start address
 * @gpu_buf_size: GPU buffer size
 *
 * kref_init() intializes the reference count on this gdr_mr
 * to 1.  This is treated as the "ioctl_mr_ref()" for this
 * gdr_mr.
 *
 * Return:
 * Pointer to the new mr if successful. NULL otherwise.
 */
static struct gdr_mr *create_mr(struct hfi1_gdrdata *gd,
				u64 gpu_buf_addr,
				u32 gpu_buf_size)
{
	struct gdr_mr *mr;

	mr = kzalloc(sizeof(*mr), GFP_KERNEL);
	if (!mr)
		return mr;

	mr->rb_node.addr = gpu_buf_addr;
	mr->rb_node.len = gpu_buf_size;
	mr->gd = gd;
	mr->mr_handle = get_random_handle();
	mr->host_addr = 0;
	mr->page_table = NULL;
	kref_init(&mr->ref_cnt);

	return mr;
}

/**
 * evict_gpu_cache() - Evict certain number of pages from a cache
 * @gd: A pointer to the hfi1_gdrdata for this open file descriptor.
 * @target: Target number of pages to be evicted from the cache.
 *
 * The caller must acquire gd->mr_lock mutex before calling.
 *
 * Return:
 * Number of pages evicted from the cache.
 */
static u32 evict_gpu_cache(struct hfi1_gdrdata *gd, u32 target)
{
	struct gdr_mr *mr, *ptr;
	unsigned int npages, cleared = 0;

	list_for_each_entry_safe_reverse(mr, ptr, &gd->lru_list, list) {
		npages = num_user_pages_gpu(mr->rb_node.addr, mr->rb_node.len);
		/* this node will be evicted, add its pages to our count */
		cleared += npages;
		acquire_ioctl_mr_ref(mr);
		mutex_unlock(&gd->mr_lock);
		do_munmap_and_unpin_gpu_buf(mr);
		mutex_lock(&gd->mr_lock);
		/* have enough pages been cleared? */
		if (cleared >= target)
			break;
	}
	return cleared;
}

/**
 * do_pin_and_mmap_gpu_buf() - pin and mmap a gpu buffer
 * @fp: A pointer to an open file structure for this file.
 * @gd: A pointer to the hfi1_gdrdata for this open file.
 * @mr: A pointer to struct gdr_mr which contains the starting address and
 *      the size of the gpu buffer.
 *
 * The caller has searched the red/black tree of gdr_mr structures and
 * didn't find one that matches the GPU desired buffer.  This routine pins and
 * mmaps the GPU buffer and adds it to the tree of gdr_mr structures.
 *
 * Side Effects:
 *	This function uses gd->map_this_mr to indirectly pass a pointer to a
 *	gdr_mr struct to this driver's mmap handler, hfi1_gdr_mmap(), outside
 *	the normal call stack mechanism.
 *
 *	gd->map_this_mr is set prior to calling vm_mmap().  vm_mmap() will
 *	call hfi1_gdr_mmap() to do this driver-specific memory map operation.
 *	The gd->map_this_mr pointer allows hfi1_gdr_mmap() to quickly find
 *	this descriptor for the physical addresses that are to be mmapped.
 *
 *	When vm_mmap() returns, the gd->map_this_mr is cleared as clean up
 *	after the mmap operation has completed.
 *
 * Return:
 * 0 - success,
 * other - unable to pin or mmap the gpu buffer
 */
static int
do_pin_and_mmap_gpu_buf(struct file *fp,
			struct hfi1_gdrdata *gd,
			struct gdr_mr *mr)
{
	unsigned long virtual, max_cache_pages;
	unsigned int npages, target, cleared;
	int ret = 0;

	mutex_lock(&gd->mr_lock);

	max_cache_pages = ((gpu_cache_size << 20) >> NV_GPU_PAGE_SHIFT);
	npages = num_user_pages_gpu(mr->rb_node.addr, mr->rb_node.len);

	if (gd->n_pages_locked + npages > max_cache_pages) {
		if (npages > max_cache_pages) {
			mutex_unlock(&gd->mr_lock);
			return -EINVAL;
		}
		target = npages - (max_cache_pages - gd->n_pages_locked);
		cleared = evict_gpu_cache(gd, target);
		WARN_ON(cleared < target);
	}

	ret = get_gpu_pages(mr->rb_node.addr, mr->rb_node.len,
			    &mr->page_table,
			    gdrdrv_get_pages_free_callback,
			    mr);

	if (ret) {
		/*
		 * As per NVIDIA's documentation, nvidia_p2p_get_pages API
		 * returns -EINVAL if an invalid argument was supplied.
		 * While testing the code, it was found that the API may
		 * return -EINVAL and not -ENOMEM if there isn't enough GPU
		 * BAR memory to pin the required number of pages. So if the
		 * error return is -ENOMEM or -EINVAL, we make an attempt to
		 * evict npages from the pinned buffer cache and try
		 * pinning the buffer again.
		 */
		if ((ret == -ENOMEM) || (ret == -EINVAL)) {
			target = npages;
			cleared = evict_gpu_cache(gd, target);
			ret = get_gpu_pages(mr->rb_node.addr,
					    mr->rb_node.len,
					    &mr->page_table,
					    gdrdrv_get_pages_free_callback,
					    mr);
			if (!ret)
				goto pin_success;
		}
		mutex_unlock(&gd->mr_lock);
		kfree(mr);
		return ret;
	}
pin_success:
	gd->n_pages_locked += npages;

	acquire_callback_mr_ref(mr);

	/*
	 * Save this mr so that this driver's mmap handler hfi1_gdr_mmap()
	 * can find it quickly and process it.  On success, the pinned
	 * GPU memory described by this mr will be mmapped into the user's
	 * address space.
	 */
	gd->map_this_mr = mr;

	mutex_unlock(&gd->mr_lock);

	/*
	 * mmap the set of physical pages that in the mr->page_table
	 * list of physical page addresses.  mr->page_table was
	 * constructed by the get_gpu_pages() call above.
	 */
	virtual = vm_mmap(fp, 0, mr->rb_node.len,
			  PROT_READ|PROT_WRITE,
			  MAP_SHARED,
			  handle_to_offset(mr->mr_handle));

	mutex_lock(&gd->mr_lock);

	/*
	 * The mmap operation on this pinned GPU buffer has completed,
	 * so clean up.  This cleanup is important, as hfi1_gdr_mmap()
	 * tests for a NULL gd->map_this_mr to identify cases where the
	 * application has called the mmap(2) system call.
	 */
	gd->map_this_mr = NULL;

	if (!mr->page_table) {
		/*
		 * In this case, the free callback handler has unpinned
		 * this gdr_mr structure.  But it's POSSIBLE that the
		 * mapping of that pinned buffer succeeded.
		 *
		 * But the virtual address of that mapping isn't available
		 * until the vm_mmap() call has returned here.  So the
		 * callback handler can not have done the proper unmap.
		 * So we need to do the munmap here, even though the
		 * GPU buffer has already been unpinned.
		 */
		if (!IS_ERR((void *)virtual)) {
			int munmap_ret;

			munmap_ret = vm_munmap(mr->host_addr, mr->rb_node.len);
			WARN_ON(munmap_ret);
		}
		release_ioctl_mr_ref(mr);
		mutex_unlock(&gd->mr_lock);
		return -EINVAL;
	}

	/*
	 * This is a case where the GPU buffer is still pinned,
	 * but this driver's mmap handler failed for some other
	 * reason.
	 *
	 * So the mr->host_addr is not set in this error case,
	 * but we still want to unpin this GPU buffer.
	 */
	if (IS_ERR((void *)virtual)) {
		WARN_ON(1);
		mutex_unlock(&gd->mr_lock);
		do_munmap_and_unpin_gpu_buf(mr);
		return virtual;
	}

	mr->host_addr = virtual;
	ret = hfi1_mmu_rb_insert(gd->gdr_handler, &mr->rb_node);
	if (ret)
		WARN_ON(ret);
	list_add(&mr->list, &gd->lru_list);
	mutex_unlock(&gd->mr_lock);

	return ret;
}

/**
 * fetch_user_query_ioctl_params - fetch and validate arguments.
 * @arg
 * @query_params
 *
 * Fetch from user space the query parameter block and validate its content.
 *
 * Return:
 * 0 on success
 * -EFAULT on bad parameter block address
 * -EINVAL on invalid content of parameter block
 */
int
fetch_user_query_ioctl_params(unsigned long arg,
			      struct hfi1_gdr_query_params *query_params)
{
	if (copy_from_user(query_params,
			   (struct hfi_gdr_query_params __user *)arg,
			   sizeof(*query_params)))
		return -EFAULT;

	if (query_params->query_params_in.version != HFI1_GDR_VERSION)
		return -ENODEV;

	if ((!query_params->query_params_in.gpu_buf_size) ||
	    (query_params->query_params_in.gpu_buf_addr & ~NV_GPU_PAGE_MASK) ||
	    (query_params->query_params_in.gpu_buf_size & ~NV_GPU_PAGE_MASK))
		return -EINVAL;


	return 0;
}

/**
 * ioctl_gpu_buf_cache_evict() - process the HFI1_IOCTL_GDR_GPU_CACHE_EVICT
 * @gd: A pointer to the hfi1_gdrdata structure for this file.
 * @arg: A pointer to the struct hfi1_gdr_cache_evict_params argument.
 *
 * This function handles ioctl() requests to evict a certain number of
 * pages from a GPU buffer cache and/or to query the number of pages
 * in the cache. Reqests to evict pages will have non-zero pages_to_evict
 * input parameter. Number of pages evicted and number of pages in the cache
 * will be filled in the output parameters.
 *
 * Return:
 * 0 - success,
 * -EFAULT - The copy_from_user() or copy_to_user() function failed.
 */
static int
ioctl_gpu_buf_cache_evict(struct hfi1_gdrdata *gd,
			  unsigned long arg)
{
	unsigned int cleared = 0, target;
	struct hfi1_gdr_cache_evict_params evict_params;

	if (copy_from_user(&evict_params,
			   (struct hfi1_gdr_cache_evict_params __user *)arg,
			   sizeof(evict_params)))
		return -EFAULT;

	if (evict_params.evict_params_in.version != HFI1_GDR_VERSION)
		return -ENODEV;

	target = evict_params.evict_params_in.pages_to_evict;

	mutex_lock(&gd->mr_lock);
	if (target > 0)
		cleared = evict_gpu_cache(gd, target);
	evict_params.evict_params_out.pages_evicted = cleared;
	evict_params.evict_params_out.pages_in_cache = gd->n_pages_locked;
	mutex_unlock(&gd->mr_lock);

	if (copy_to_user((struct hfi1_gdr_cache_evict_params __user *)arg,
			 &evict_params,
			 sizeof(evict_params)))
		return -EFAULT;

	return 0;
}

/**
 * ioctl_gpu_buf_pin_mmap() - process the HFI1_IOCTL_GDR_GPU_PIN_MMAP
 * @fp: A pointer to the open file structure for this file.
 * @gd: A pointer to the hfi1_gdrdata structure for this file.
 * @arg: A pointer to the struct hfi1_gdr_query_params argument.
 *
 * This function handles ioctl() requests to return a host address
 * for a pinned and mmapped gpu buffer.
 *
 * Search the red/black tree for the desired gpu buffer among the tree
 * of struct gdr_mr structures.  If it isn't found, then pin and mmap it,
 * and add it to the red/black tree
 *
 * On success, return to the user the user host address of this mapping.
 * This "fast path" in this code is when the gpu buffer is already found
 * in the red/black tree.  We want this case to run with minimal overhead.
 *
 * Return:
 * 0 - success,
 * -EFAULT - The copy_from_user() or copy_to_user() function failed,
 * -EINVAL - The gpu buffer described is not properly aligned,
 * -EINVAL - The operation requested was not valid,
 * -ENOENT - An unpin was requested, but the desired gpu buffer was not found,
 */
static int
ioctl_gpu_buf_pin_mmap(struct file *fp,
		       struct hfi1_gdrdata *gd,
		       unsigned long arg)
{
	struct hfi1_gdr_query_params query_params;
	struct gdr_mr *mr = NULL;
	int ret = 0;
	struct mmu_rb_node *rb_node;

	ret = fetch_user_query_ioctl_params(arg, &query_params);
	if (ret)
		return ret;

	mutex_lock(&gd->mr_lock);
	rb_node = hfi1_mmu_rb_search_addr(gd->gdr_handler,
				  query_params.query_params_in.gpu_buf_addr,
				  query_params.query_params_in.gpu_buf_size);
	if (rb_node) {
		/*
		 * A buffer found in the cache that partially or completely
		 * overlaps the address range.
		 */
		mr = container_of(rb_node, struct gdr_mr, rb_node);
		if (rb_node->len >=
				query_params.query_params_in.gpu_buf_size) {
			/*
			 * Buffer found in the cache that has the same
			 * starting address and is the same length as or is
			 * longer than the requested buffer.
			 *
			 * This code path never releases mr_lock after
			 * the gdr_mr struct has been found in the r/b tree,
			 * until after it is completely finished with that
			 * gdr_mr struct.
			 *
			 * So while LOGICALLY we could acquire and release
			 * an ioctl reference in this code path, there is no
			 * need to actually do that.
			 */
			query_params.query_params_out.host_buf_addr =
								mr->host_addr;
			/*
			 * Since this mr is being "used", move this mr to
			 * the end of the list.
			 */
			list_del(&mr->list);
			list_add(&mr->list, &gd->lru_list);
			mutex_unlock(&gd->mr_lock);
			goto skip_pin_mmap;
		} else {
			/*
			 * rb_node->len < gpu_buf_size
			 *
			 * A buffer found in the cache that has the same
			 * starting address but is of lesser length.
			 * Need to unmap and unpin the old old smaller
			 * GPU buffer, freeing the struct gdr_mr in the
			 * processs.
			 *
			 * A new struct gdr_mr will be allocated, and a
			 * new, larger GPU buffer will be pinned and
			 * mmapped below.
			 */
			acquire_ioctl_mr_ref(mr);
			mutex_unlock(&gd->mr_lock);
			do_munmap_and_unpin_gpu_buf(mr);
			mutex_lock(&gd->mr_lock);
		}
	}
	mr = create_mr(gd,
		       query_params.query_params_in.gpu_buf_addr,
		       query_params.query_params_in.gpu_buf_size);
	mutex_unlock(&gd->mr_lock);
	ret = do_pin_and_mmap_gpu_buf(fp, gd, mr);
	if (!ret) {
		mutex_lock(&gd->mr_lock);
		query_params.query_params_out.host_buf_addr = mr->host_addr;
		release_ioctl_mr_ref(mr);
		mutex_unlock(&gd->mr_lock);
	}

skip_pin_mmap:
	if ((!ret) && (copy_to_user((struct hfi_gdr_query_params __user *)arg,
				    &query_params,
				    sizeof(query_params))))
		return -EFAULT;

	return ret;
}

/**
 * ioctl_gpu_buf_unmap_unpin() - process the HFI1_IOCTL_GDR_GPU_UNPIN_MUNMAP
 * @fp: A pointer to the open file structure for this file.
 * @gd: A pointer to the hfi1_gdrdata structure for this file.
 * @arg: A pointer to the struct hfi1_gdr_query_params argument.
 *
 * This function handles the ioctl() request to unpin and unmap a gpu buffer.
 *
 * Return:
 * 0 - success,
 * -EFAULT - The copy_from_user() or copy_to_user() function failed,
 * -EINVAL - The gpu buffer described is not properly aligned,
 * -EINVAL - The operation requested was not valid,
 * -ENOENT - An unpin was requested, but the desired gpu buffer was not found,
 */
static int
ioctl_gpu_buf_munmap_unpin(struct file *fp,
			   struct hfi1_gdrdata *gd,
			   unsigned long arg)
{
	struct hfi1_gdr_query_params query_params;
	struct gdr_mr *mr = NULL;
	int ret = 0;
	struct mmu_rb_node *rb_node;

	ret = fetch_user_query_ioctl_params(arg, &query_params);
	if (ret)
		return ret;

	mutex_lock(&gd->mr_lock);
	rb_node = hfi1_mmu_rb_search_addr(gd->gdr_handler,
				query_params.query_params_in.gpu_buf_addr,
				query_params.query_params_in.gpu_buf_size);
	if (rb_node) {
		mr = container_of(rb_node, struct gdr_mr, rb_node);
		acquire_ioctl_mr_ref(mr);
	}
	mutex_unlock(&gd->mr_lock);

	if (mr)
		do_munmap_and_unpin_gpu_buf(mr);
	else
		ret = -ENOENT;

	return ret;
}

/**
 * hfi1_gdr_ioctl() - This is this driver's ioctl() handler
 * @fp: A pointer to the open file structure for this file.
 * @cmd: The ioctl() command to execute.
 * @arg: The argument to this ioctl() request.
 *
 * Only one thread at a time is allowed to execute this ioctl() handler.
 * If there is a thread executing in this handler, and a second thread
 * calls this ioctl(), the second thread will get a -EINVAL failure.
 *
 * There are two recognized commands to this ioctl.
 *
 *	HFI1_IOCTL_GDR_GPU_PIN_MMAP, a request to either retrieve the
 *	host address for a pinned/mmapped gpu buffer, or a request
 *	to unpin and unmap a previously pinned gpu buffer.
 *
 *	HFI1_IOCTL_GDR_GPU_MUNMAP_UNPIN, a request to munmap and unpin
 *	a gpu buffer.
 *
 * Return:
 * 0 - success,
 * -EINVAL - There is already a thread executing in this ioctl handler,
 * -EINVAL - the cmd argument is not a known value.
 */
static long hfi1_gdr_ioctl(struct file *fp, unsigned int cmd,
			   unsigned long arg)
{
	struct hfi1_gdrdata *gd = fp->private_data;
	int ret = 0;

	if (atomic_cmpxchg(&gd->ioctl_busy_flag, 0, 1))
		return -EINVAL;

	switch (cmd) {
	case HFI1_IOCTL_GDR_GPU_PIN_MMAP:
		ret = ioctl_gpu_buf_pin_mmap(fp, gd, arg);
		break;
	case HFI1_IOCTL_GDR_GPU_MUNMAP_UNPIN:
		ret = ioctl_gpu_buf_munmap_unpin(fp, gd, arg);
		break;
	case HFI1_IOCTL_GDR_GPU_CACHE_EVICT:
		ret = ioctl_gpu_buf_cache_evict(gd, arg);
		break;
	default:
		ret =  -EINVAL;
	}

	WARN_ON(atomic_read(&gd->ioctl_busy_flag) != 1);
	atomic_set(&gd->ioctl_busy_flag, 0);

	return ret;
}

/**
 * gdr_mmap_phys_mem_wcomb()
 * @vma: A pointer to a vm_area_struct for physical memory segment to be mmaped.
 * @vaddr: A virtual address to mmap a physical memory segment.
 * @paddr: The physical address of the physical memory segment.
 * @size: The size of the physical memory segment.
 *
 * This function remaps a contiguous physical memory region into the user's
 * address space.  THe mapping is in write combining mode.
 *
 * Return:
 * 0 - success,
 * -EAGAIN - the mmap request failed
 */
static int gdr_mmap_phys_mem_wcomb(struct vm_area_struct *vma,
				   unsigned long vaddr,
				   unsigned long paddr,
				   size_t size)
{
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	if (io_remap_pfn_range(vma, vaddr,
			       PHYS_PFN(paddr),
			       size,
			       vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

/**
 * hfi1_gdr_mmap() - This driver's mmap handler.
 * @filep: A pointer to the open file structure for this file.
 * @vma: A pointer to the vma_area struct.
 *
 * This driver's mmap handler is normally invoked through this driver's
 * ioctl handler when it is mmapping a pinned GPU buffer into the user's
 * address space.
 *
 * This driver's mmap handler COULD also be called by the user calling
 * the mmap() system call.  This is not a valid case for this driver,
 * and that mmap() system call will fail.
 *
 * Return:
 * 0 - success,
 * -EINVAL - The requested gpu buffer description is invalid
 */
static int hfi1_gdr_mmap(struct file *filep, struct vm_area_struct *vma)
{
	int ret = 0;
	size_t size = vma->vm_end - vma->vm_start;
	struct hfi1_gdrdata *gd = filep->private_data;
	struct gdr_mr *mr;
	int p = 0;
	unsigned long vaddr, prev_page_paddr;
	int phys_contiguous = 1;

	mutex_lock(&gd->mr_lock);

	mr = gd->map_this_mr;
	/*
	 * map_this_mr being NULL indicates that the user has
	 * called mmap directly on this file descriptor.  We want
	 * to fail this mmap attempt.
	 */
	if (!mr) {
		ret = -EINVAL;
		goto out;
	}

	/*
	 * If the handle value from vm_pgoff does not match mr_handle,
	 * this means the user has called mmap() system call and
	 * raced with the ioctl() calling vm_mmap(). Fail this case as well.
	 */
	if (mr->mr_handle != handle_from_vm_pgoff(vma->vm_pgoff)) {
		ret = -EINVAL;
		goto out;
	}

	/*
	 * This code path lost a race with the free callback handler,
	 * which has unpinned and unmapped this GPU buffer.
	 */
	if (!mr->page_table) {
		ret = -EINVAL;
		goto out;
	}

	/*
	 * check for physically contiguous IO range
	 */
	vaddr = vma->vm_start;
	prev_page_paddr = mr->page_table->pages[0]->physical_address;
	phys_contiguous = 1;
	for (p = 1; p < mr->page_table->entries; ++p) {
		struct nvidia_p2p_page *page = mr->page_table->pages[p];
		unsigned long page_paddr = page->physical_address;
		if (prev_page_paddr + NV_GPU_PAGE_SIZE != page_paddr) {
			phys_contiguous = 0;
			break;
		}
		prev_page_paddr = page_paddr;
	}

	if (phys_contiguous) {
		size_t len = min(size,
				 NV_GPU_PAGE_SIZE * mr->page_table->entries);
		unsigned long page0_paddr =
			mr->page_table->pages[0]->physical_address;
		ret = gdr_mmap_phys_mem_wcomb(vma,
					      vaddr,
					      page0_paddr,
					      len);
		if (ret)
			goto out;

	} else {
		/*
		 * If not contiguous, map individual GPU pages separately.
		 * In this case, write-combining performance can be really
		 * bad, not sure why.
		 */
		p = 0;
		while (size && p < mr->page_table->entries) {
			struct nvidia_p2p_page *page = mr->page_table->pages[p];
			unsigned long page_paddr = page->physical_address;
			size_t len = min(NV_GPU_PAGE_SIZE, size);

			ret = gdr_mmap_phys_mem_wcomb(vma,
						      vaddr,
						      page_paddr,
						      len);
			if (ret)
				goto out;

			vaddr += len;
			size -= len;
			++p;
		}
	}

out:
	mutex_unlock(&gd->mr_lock);

	return ret;
}

/**
 * hfi1_gdr_release() - This is the release function for this handler.
 * @inode: A pointer to the inode form this file.
 * @filep: A pointer to the open file structure for this file.
 *
 * This function unpins and unmaps all gpu buffers in the list of
 * struct gdr_mr structures.
 *
 * Return:
 * 0 - success
 */
static int hfi1_gdr_release(struct inode *inode, struct file *filep)
{
	struct gdr_mr *mr;
	struct mmu_rb_node *mnode;
	struct hfi1_gdrdata *gd = filep->private_data;

	filep->private_data = NULL;

	mutex_lock(&gd->mr_lock);
	while ((mnode = hfi1_mmu_rb_first_cached(gd->gdr_handler))) {
		mr = container_of(mnode, struct gdr_mr, rb_node);
		acquire_ioctl_mr_ref(mr);
		mutex_unlock(&gd->mr_lock);
		do_munmap_and_unpin_gpu_buf(mr);
		mutex_lock(&gd->mr_lock);
	}
	mutex_unlock(&gd->mr_lock);

	hfi1_mmu_rb_unregister(gd->gdr_handler);

	kfree(gd);

	return 0;
}

static char *gdr_devnode(struct device *dev, umode_t *mode)
{
	if (mode)
		*mode = 0666;
	return kasprintf(GFP_KERNEL, "%s", dev_name(dev));
}


static int add_gdr_dev(void)
{
	int ret = 0;

	gdr_major = register_chrdev(0, GDR_DRIVER_NAME, &hfi1_gdr_ops);
	if (gdr_major < 0) {
		gdr_major = 0;
		ret = -ENODEV;
		goto out;
	}

	gdr_dev = MKDEV(gdr_major, 0);

	gdr_class = class_create(THIS_MODULE, GDR_CLASS_NAME);
	if (IS_ERR(gdr_class)) {
		ret = PTR_ERR(gdr_class);
		gdr_class = NULL;
		goto out;
	}

	gdr_class->devnode = gdr_devnode;

	gdr_device = device_create(gdr_class, NULL, gdr_dev,
				      NULL, "%s", GDR_DEV_NAME);

	if (IS_ERR(gdr_device)) {
		ret = -PTR_ERR(gdr_device);
		gdr_device = NULL;
		goto out;
	}

out:
	return ret;
}

static void remove_gdr_dev(void)
{
	if (gdr_device) {
		device_unregister(gdr_device);
		gdr_device = NULL;
	}

	if (gdr_class) {
		class_destroy(gdr_class);
		gdr_class = NULL;
	}

	if (gdr_major) {
		unregister_chrdev(gdr_major, GDR_DRIVER_NAME);
		gdr_major = 0;
	}
}

/**
 * hfi1_gdr_device_create() - Create gdr device and its file in /dev
 *
 * Return: 0 on sucess, negative error code on failure
 */
int hfi1_gdr_device_create()
{
	int ret = 0;

	ret = add_gdr_dev();
	if (ret)
		remove_gdr_dev();

	return ret;
}

/**
 * hfi1_gdr_device_remove() - Remove gdr device and its file in /dev
 */
void hfi1_gdr_device_remove()
{
	remove_gdr_dev();
}

