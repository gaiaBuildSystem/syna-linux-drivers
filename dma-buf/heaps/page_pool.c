// SPDX-License-Identifier: GPL-2.0
/*
 * Memory Allocator page pool helpers
 *
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) 2026 Synaptics, Inc.
 */

#include <linux/list.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include "page_pool.h"
#include <linux/module.h>

struct page *sys_page_pool_alloc(struct page_pool *pool)
{
	struct page *page = NULL;

	BUG_ON(!pool);

	mutex_lock(&pool->mutex);
	if (pool->high_count) {
		page = list_first_entry(&pool->high_items, struct page, lru);
		pool->high_count--;
	} else if (pool->low_count) {
		page = list_first_entry(&pool->low_items, struct page, lru);
		pool->low_count--;
	}
	if (page)
		list_del(&page->lru);
	mutex_unlock(&pool->mutex);

	return page;
}

void sys_page_pool_free(struct page_pool *pool, struct page *page)
{
	BUG_ON(pool->order != compound_order(page));

	mutex_lock(&pool->mutex);
	if (PageHighMem(page)) {
		list_add_tail(&page->lru, &pool->high_items);
		pool->high_count++;
	} else {
		list_add_tail(&page->lru, &pool->low_items);
		pool->low_count++;
	}
	mutex_unlock(&pool->mutex);
}

int sys_page_pool_nr_pages(struct page_pool *pool)
{
	int nr_total_pages, count;

	mutex_lock(&pool->mutex);
	count = pool->low_count + pool->high_count;
	nr_total_pages = count << pool->order;
	mutex_unlock(&pool->mutex);

	return nr_total_pages;
}

struct page_pool *sys_page_pool_create(gfp_t gfp_mask, unsigned int order)
{
	struct page_pool *pool = kmalloc(sizeof(*pool), GFP_KERNEL);

	if (!pool)
		return NULL;
	pool->high_count = 0;
	pool->low_count = 0;
	INIT_LIST_HEAD(&pool->low_items);
	INIT_LIST_HEAD(&pool->high_items);
	pool->gfp_mask = gfp_mask | __GFP_COMP;
	pool->order = order;
	mutex_init(&pool->mutex);
	INIT_LIST_HEAD(&pool->list);

	return pool;
}

void sys_page_pool_destroy(struct page_pool *pool)
{
	kfree(pool);
}

MODULE_LICENSE("GPL v2");
