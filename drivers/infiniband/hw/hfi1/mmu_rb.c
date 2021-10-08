/*
 * Copyright(c) 2020 Cornelis Networks, Inc.
 * Copyright(c) 2016 - 2017 Intel Corporation.
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
#include <linux/list.h>
#include <linux/rculist.h>
#include <linux/mmu_notifier.h>
#include <linux/interval_tree_generic.h>
#ifndef NEED_MM_HELPER_FUNCTIONS
#include <linux/sched/mm.h>
#endif

#include "mmu_rb.h"
#include "trace.h"

static unsigned long mmu_node_start(struct mmu_rb_node *);
static unsigned long mmu_node_last(struct mmu_rb_node *);
static int mmu_notifier_range_start(struct mmu_notifier *,
		const struct mmu_notifier_range *);
#ifndef MMU_NOTIFIER_RANGE_START_USES_MMU_NOTIFIER_RANGE
static void compat_mmu_notifier_range_start(struct mmu_notifier *mn,
					    struct mm_struct *mm,
					    unsigned long start,
					    unsigned long end)
{
	struct mmu_notifier_range r = {
		.mm = mm,
		.start = start,
		.end = end,
	};

	(void)mmu_notifier_range_start(mn, &r);
}
#endif
static struct mmu_rb_node *__mmu_rb_search(struct mmu_rb_handler *,
					   unsigned long, unsigned long);
static void do_remove(struct mmu_rb_handler *handler,
		      struct list_head *del_list);
static void handle_remove(struct work_struct *work);

static const struct mmu_notifier_ops mn_opts = {
#ifdef MMU_INVALIDATE_DOES_NOT_BLOCK
	.flags = MMU_INVALIDATE_DOES_NOT_BLOCK,
#endif
#ifdef MMU_NOTIFIER_RANGE_START_USES_MMU_NOTIFIER_RANGE
	.invalidate_range_start = mmu_notifier_range_start,
#else
	.invalidate_range_start = compat_mmu_notifier_range_start,
#endif
};

INTERVAL_TREE_DEFINE(struct mmu_rb_node, node, unsigned long, __last,
		     mmu_node_start, mmu_node_last, static, __mmu_int_rb);

static unsigned long mmu_node_start(struct mmu_rb_node *node)
{
	return node->addr & PAGE_MASK;
}

static unsigned long mmu_node_last(struct mmu_rb_node *node)
{
	return PAGE_ALIGN(node->addr + node->len) - 1;
}

int hfi1_mmu_rb_register(void *ops_arg,
			 struct mmu_rb_ops *ops,
			 struct workqueue_struct *wq,
			 struct mmu_rb_handler **handler)
{
	struct mmu_rb_handler *h;
	int ret;

	h = kzalloc(sizeof(*h), GFP_KERNEL);
	if (!h)
		return -ENOMEM;
#ifdef NO_RB_ROOT_CACHE
	h->root = RB_ROOT;
#else
	h->root = RB_ROOT_CACHED;
#endif
	h->ops = ops;
	h->ops_arg = ops_arg;
	INIT_HLIST_NODE(&h->mn.hlist);
	spin_lock_init(&h->lock);
	h->mn.ops = &mn_opts;
	INIT_WORK(&h->del_work, handle_remove);
	INIT_LIST_HEAD(&h->del_list);
	INIT_LIST_HEAD(&h->lru_list);
	h->wq = wq;

	ret = mmu_notifier_register(&h->mn, current->mm);
	if (ret) {
		kfree(h);
		return ret;
	}

	/* Distro does not support embedding the mm and taking the ref */
#ifdef NO_MMU_NOTIFIER_MM
	h->mm = current->mm;
#endif
#ifdef NO_NOTIFIER_REG_GRAB
	mmgrab(h->mm);
#endif
	h->registered = true;
	*handler = h;
	return 0;
}

void hfi1_mmu_rb_unregister(struct mmu_rb_handler *handler)
{
	struct mmu_rb_node *rbnode;
	struct rb_node *node;
	unsigned long flags;
	struct list_head del_list;

	/* Unregister first so we don't get any more notifications. */
	if (handler->registered) {
#ifdef NO_MMU_NOTIFIER_MM
		mmu_notifier_unregister(&handler->mn, handler->mm);
#else
		mmu_notifier_unregister(&handler->mn, handler->mn.mm);
#endif
#ifdef NO_NOTIFIER_REG_GRAB
#ifdef NO_MMU_NOTIFIER_MM
		mmdrop(handler->mm);
#else
		mmdrop(handler->mn.mm);
#endif
#endif
	} else {
		/* The grab was unconditional for the GPU case */
#ifdef NO_MMU_NOTIFIER_MM
		mmdrop(handler->mm);
#else
		mmdrop(handler->mn.mm);
#endif
	}

	/*
	 * Make sure the wq delete handler is finished running.  It will not
	 * be triggered once the mmu notifiers are unregistered above.
	 */
	flush_work(&handler->del_work);

	INIT_LIST_HEAD(&del_list);

	spin_lock_irqsave(&handler->lock, flags);
#ifdef NO_RB_ROOT_CACHE
	while ((node = rb_first(&handler->root))) {
#else
	while ((node = rb_first_cached(&handler->root))) {
#endif
		rbnode = rb_entry(node, struct mmu_rb_node, node);
#ifdef NO_RB_ROOT_CACHE
		rb_erase(node, &handler->root);
#else
		rb_erase_cached(node, &handler->root);
#endif
		/* move from LRU list to delete list */
		list_move(&rbnode->list, &del_list);
	}
	spin_unlock_irqrestore(&handler->lock, flags);

	do_remove(handler, &del_list);

	kfree(handler);
}

int hfi1_mmu_rb_insert(struct mmu_rb_handler *handler,
		       struct mmu_rb_node *mnode)
{
	struct mmu_rb_node *node;
	unsigned long flags;
	int ret = 0;

	trace_hfi1_mmu_rb_insert(mnode->addr, mnode->len);

#ifdef NO_MMU_NOTIFIER_MM
	if (current->mm != handler->mm)
#else
	if (current->mm != handler->mn.mm)
#endif
		return -EPERM;
	spin_lock_irqsave(&handler->lock, flags);
	node = __mmu_rb_search(handler, mnode->addr, mnode->len);
	if (node) {
		ret = -EINVAL;
		goto unlock;
	}
	__mmu_int_rb_insert(mnode, &handler->root);
	list_add(&mnode->list, &handler->lru_list);

	ret = handler->ops->insert(handler->ops_arg, mnode);
	if (ret) {
		__mmu_int_rb_remove(mnode, &handler->root);
		list_del(&mnode->list); /* remove from LRU list */
	}
	mnode->handler = handler;
unlock:
	spin_unlock_irqrestore(&handler->lock, flags);
	return ret;
}

/* Caller must hold handler lock */
static struct mmu_rb_node *__mmu_rb_search(struct mmu_rb_handler *handler,
					   unsigned long addr,
					   unsigned long len)
{
	struct mmu_rb_node *node = NULL;

	trace_hfi1_mmu_rb_search(addr, len);
	if (!handler->ops->filter) {
		node = __mmu_int_rb_iter_first(&handler->root, addr,
					       (addr + len) - 1);
	} else {
		for (node = __mmu_int_rb_iter_first(&handler->root, addr,
						    (addr + len) - 1);
		     node;
		     node = __mmu_int_rb_iter_next(node, addr,
						   (addr + len) - 1)) {
			if (handler->ops->filter(node, addr, len))
				return node;
		}
	}
	return node;
}

bool hfi1_mmu_rb_remove_unless_exact(struct mmu_rb_handler *handler,
				     unsigned long addr, unsigned long len,
				     struct mmu_rb_node **rb_node)
{
	struct mmu_rb_node *node;
	unsigned long flags;
	bool ret = false;

	spin_lock_irqsave(&handler->lock, flags);
	node = __mmu_rb_search(handler, addr, len);
	if (node) {
		if (node->addr == addr && node->len == len)
			goto unlock;
		__mmu_int_rb_remove(node, &handler->root);
		list_del(&node->list); /* remove from LRU list */
		ret = true;
	}
unlock:
	spin_unlock_irqrestore(&handler->lock, flags);
	*rb_node = node;
	return ret;
}

void hfi1_mmu_rb_evict(struct mmu_rb_handler *handler, void *evict_arg)
{
	struct mmu_rb_node *rbnode, *ptr;
	struct list_head del_list;
	unsigned long flags;
	bool stop = false;

	INIT_LIST_HEAD(&del_list);

	spin_lock_irqsave(&handler->lock, flags);
	list_for_each_entry_safe_reverse(rbnode, ptr, &handler->lru_list,
					 list) {
		if (handler->ops->evict(handler->ops_arg, rbnode, evict_arg,
					&stop)) {
			__mmu_int_rb_remove(rbnode, &handler->root);
			/* move from LRU list to delete list */
			list_move(&rbnode->list, &del_list);
		}
		if (stop)
			break;
	}
	spin_unlock_irqrestore(&handler->lock, flags);

	while (!list_empty(&del_list)) {
		rbnode = list_first_entry(&del_list, struct mmu_rb_node, list);
		list_del(&rbnode->list);
		handler->ops->remove(handler->ops_arg, rbnode);
	}
}

/*
 * It is up to the caller to ensure that this function does not race with the
 * mmu invalidate notifier which may be calling the users remove callback on
 * 'node'.
 */
void hfi1_mmu_rb_remove(struct mmu_rb_handler *handler,
			struct mmu_rb_node *node)
{
	unsigned long flags;

	/* Validity of handler and node pointers has been checked by caller. */
	trace_hfi1_mmu_rb_remove(node->addr, node->len);
	spin_lock_irqsave(&handler->lock, flags);
	__mmu_int_rb_remove(node, &handler->root);
	list_del(&node->list); /* remove from LRU list */
	spin_unlock_irqrestore(&handler->lock, flags);

	handler->ops->remove(handler->ops_arg, node);
}

static int mmu_notifier_range_start(struct mmu_notifier *mn,
		const struct mmu_notifier_range *range)
{
	struct mmu_rb_handler *handler =
		container_of(mn, struct mmu_rb_handler, mn);
#ifdef NO_RB_ROOT_CACHE
	struct rb_root *root = &handler->root;
#else
	struct rb_root_cached *root = &handler->root;
#endif
	struct mmu_rb_node *node, *ptr = NULL;
	unsigned long flags;
	bool added = false;

	spin_lock_irqsave(&handler->lock, flags);
	for (node = __mmu_int_rb_iter_first(root, range->start, range->end-1);
	     node; node = ptr) {
		/* Guard against node removal. */
		ptr = __mmu_int_rb_iter_next(node, range->start,
					     range->end - 1);
		trace_hfi1_mmu_mem_invalidate(node->addr, node->len);
		if (handler->ops->invalidate(handler->ops_arg, node)) {
			__mmu_int_rb_remove(node, root);
			/* move from LRU list to delete list */
			list_move(&node->list, &handler->del_list);
			added = true;
		}
	}
	spin_unlock_irqrestore(&handler->lock, flags);

	if (added)
		queue_work(handler->wq, &handler->del_work);

	return 0;
}

/*
 * Call the remove function for the given handler and the list.  This
 * is expected to be called with a delete list extracted from handler.
 * The caller should not be holding the handler lock.
 */
static void do_remove(struct mmu_rb_handler *handler,
		      struct list_head *del_list)
{
	struct mmu_rb_node *node;

	while (!list_empty(del_list)) {
		node = list_first_entry(del_list, struct mmu_rb_node, list);
		list_del(&node->list);
		handler->ops->remove(handler->ops_arg, node);
	}
}

/*
 * Work queue function to remove all nodes that have been queued up to
 * be removed.  The key feature is that mm->mmap_lock is not being held
 * and the remove callback can sleep while taking it, if needed.
 */
static void handle_remove(struct work_struct *work)
{
	struct mmu_rb_handler *handler = container_of(work,
						struct mmu_rb_handler,
						del_work);
	struct list_head del_list;
	unsigned long flags;

	/* remove anything that is queued to get removed */
	spin_lock_irqsave(&handler->lock, flags);
	list_replace_init(&handler->del_list, &del_list);
	spin_unlock_irqrestore(&handler->lock, flags);

	do_remove(handler, &del_list);
}

#ifdef NVIDIA_GPU_DIRECT
/**
 * hfi1_mmu_rb_first_cached() - Return the first node in a red/black tree.
 * @handler: - A pointer to the root/control structure of a red/black tree.
 *
 * This function layers on top of the Linux interval red/black tree
 * implementation.
 *
 * If the tree is NOT empty, then return a pointer to the first node in
 * that tree.  Otherwise return NULL.
 *
 * Return: A pointer to the first node in the tree, or NULL if tree is empty.
 */
struct mmu_rb_node *hfi1_mmu_rb_first_cached(struct mmu_rb_handler *handler)
{
	struct mmu_rb_node *rbnode = NULL;
	struct rb_node *node;
	unsigned long flags;

	spin_lock_irqsave(&handler->lock, flags);
#ifdef NO_RB_ROOT_CACHE
	node = rb_first(&handler->root);
#else
	node = rb_first_cached(&handler->root);
#endif
	if (node)
		rbnode = rb_entry(node, struct mmu_rb_node, node);
	spin_unlock_irqrestore(&handler->lock, flags);

	return rbnode;
}

/**
 * hfi1_mmu_rb_search_addr() - Search tree for entry with matching start addr
 * @handler: - A pointer to the root/control structure of a red/black tree.
 * @addr: - The starting buffer address to search for in the tree.
 * @len: - The length of the buffer to search for in the tree.
 *
 * This function layers on top of the Linux interval red/black tree
 * implementation.
 *
 * It searches r/b tree indicated by the handler argument, trying to find a
 * node that matches the "addr" starting point. If a node is found, then
 * return a pointer to that node.
 *
 * Return: A pointer to a tree node, if one is found.  Otherwise return NULL.
 */
struct mmu_rb_node *hfi1_mmu_rb_search_addr(struct mmu_rb_handler *handler,
					    unsigned long addr,
					    unsigned long len)
{
	struct mmu_rb_node *ret_node = NULL;
	unsigned long flags;

	spin_lock_irqsave(&handler->lock, flags);
	ret_node = __mmu_rb_search(handler, addr, len);
	spin_unlock_irqrestore(&handler->lock, flags);

	return ret_node;
}

/*
 * This API may be used to invalidate cached page pinned buffers that are in the
 * virutal address range start to (end - 1) when the mapping becomes invalid.
 *
 * This API is intended to be invoked from functions that do not refer to
 * struct mmu_rb_handler or struct mmu_notifier but have a pointer to the
 * root of the RB tree that stores the page cache.
 * For example, this API can be used in case of buffers pinned to GPU memory
 * pages. When the mapping becomes invalid, a callback function registered
 * with the NVIDIA driver will be invoked. This callback does not get a
 * reference to struct mmu_notifier.
 */
void hfi1_gpu_cache_invalidate(struct mmu_rb_handler *handler,
			       unsigned long start, unsigned long end)
{
	struct mmu_notifier_range r = {
#ifdef NO_MMU_NOTIFIER_MM
		.mm = handler->mm,
#else
		.mm = handler->mn.mm,
#endif
		.start = start,
		.end = end,
	};

	(void)mmu_notifier_range_start(&handler->mn, &r);
}

/*
 * This API is a copy/paste of hfi1_mmu_rb_register() without
 * the notifier registration call.
 *
 * Changes in hfi_mmu_rb_register() need to be reflected
 * in this routine.
 */
int hfi1_mmu_rb_register_gpu(void *ops_arg,
			     struct mmu_rb_ops *ops,
			     struct workqueue_struct *wq,
			     struct mmu_rb_handler **handler)
{
	struct mmu_rb_handler *h;

	h = kzalloc(sizeof(*h), GFP_KERNEL);
	if (!h)
		return -ENOMEM;
#ifdef NO_RB_ROOT_CACHE
	h->root = RB_ROOT;
#else
	h->root = RB_ROOT_CACHED;
#endif
	h->ops = ops;
	h->ops_arg = ops_arg;
	INIT_HLIST_NODE(&h->mn.hlist);
	spin_lock_init(&h->lock);
	h->mn.ops = &mn_opts;
	INIT_WORK(&h->del_work, handle_remove);
	INIT_LIST_HEAD(&h->del_list);
	INIT_LIST_HEAD(&h->lru_list);
	h->wq = wq;
#ifdef NO_MMU_NOTIFIER_MM
	h->mm = current->mm;
#else
	h->mn.mm = current->mm;
#endif
	mmgrab(current->mm);
	*handler = h;
	return 0;
}
#endif /* NVIDIA_GPU_DIRECT */
