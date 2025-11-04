// SPDX-License-Identifier: GPL-2.0
/*
 * kg_resource_memory.c/
 *
 * Copyright (C) 2025 Samsung Electronics
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/mm.h>
#if defined(CONFIG_ANDROID_VENDOR_HOOKS)
#include <trace/hooks/mm.h>
#endif
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/alarmtimer.h>

#include "kg_main.h"
#include "kg_resource_control.h"

enum memory_size {
	DRAM_3GB_OR_LOWER,
	DRAM_4GB,
	DRAM_6GB,
	DRAM_8GB_OR_HIGHER,
	__MAX_DRAM_SIZE
};

enum timer_type {
	ALLOC_TIMER,
	DEADLINE_TIMER,
};

#define NUM_OF_RATIO (12)
#define ALLOC_INTERVAL_SEC (KG_RESOURCE_DEVICE_REBOOT_SEC / NUM_OF_RATIO) // 5 minutes

static unsigned int memory_restriction_table[__MAX_DRAM_SIZE][NUM_OF_RATIO] = {
	{0, 1, 2, 3, 4, 6,  8, 11, 14, 18, 23, 30},  // 3GB OR LOWER
	{0, 1, 2, 3, 5, 7,  9, 12, 16, 21, 28, 37},  // 4GB
	{0, 1, 2, 3, 5, 7,  9, 12, 17, 24, 33, 45},  // 6GB
	{0, 1, 2, 3, 5, 7, 10, 14, 21, 32, 45, 60}}; // 8GB OR HIGHER

#define GB_TO_PAGES(x) ((x) << (30 - PAGE_SHIFT))
#define PAGES_TO_MB(x) ((x) >> (20 - PAGE_SHIFT))
#define PAGES_TO_B(x)  ((x) << (PAGE_SHIFT))

struct kg_memory_buffer {
	struct list_head list;
	unsigned long size;
	void *addr;
};

static LIST_HEAD(memory_buffer_list);
static DEFINE_MUTEX(memory_buffer_list_mutex);
static void kg_memory_alloc_workfn(struct work_struct *work);
static DECLARE_WORK(kg_memory_alloc_work, kg_memory_alloc_workfn);

static atomic_t kg_memory_restriction_started;
static atomic_t kg_memory_restriction_type;
static unsigned long resource_memory_restriction_size;
static unsigned long resource_memory_restriction_time_sec;

static unsigned long resource_memory_interval_timer_sec = ALLOC_INTERVAL_SEC;
static unsigned long resource_memory_deadline_timer_sec = (ALLOC_INTERVAL_SEC * NUM_OF_RATIO);

static unsigned long total_pages;
static unsigned int resource_memory_size_idx;

static struct alarm kg_memory_deadline_timer;
static struct alarm kg_memory_alloc_timer;

enum memory_restriction_type {
	MEMORY_NO_RESTRICTION,
	MEMORY_WEAK,
	MEMORY_STRONG,
};

#if IS_ENABLED(CONFIG_KG_DRV_TEST)
static unsigned int resource_memory_alloc_test_size_mb;

static int kg_resource_memory_alloc_test(const char *buf, const struct kernel_param *kp)
{
	static void *test_addr;
	u64 value;
	int err;

	err = kstrtou64(buf, 10, &value);
	if (err) {
		KG_LOG("%s failed to kstrtou64 by %x\n", __func__, err);
		return err;
	}

	if (value > 100)
		value = 100;

	if (!total_pages)
		total_pages = totalram_pages();

	resource_memory_alloc_test_size_mb = PAGES_TO_MB(total_pages * value / 100);
	KG_LOG("%s value: %llu, total_pages: %lu, alloc: %u MB\n", __func__,
			value, total_pages, resource_memory_alloc_test_size_mb);

	if (test_addr) {
		vfree(test_addr);
		test_addr = NULL;
	}

	if (resource_memory_alloc_test_size_mb) {
		test_addr = vmalloc((unsigned long)resource_memory_alloc_test_size_mb * SZ_1M);
		if (!test_addr) {
			KG_LOG("%s failed to vmalloc: %u MB\n", __func__, resource_memory_alloc_test_size_mb);
			resource_memory_alloc_test_size_mb = 0;
		}
	}

	return 0;
}
module_param_call(resource_memory_alloc_test, &kg_resource_memory_alloc_test, NULL, NULL, 0200);

module_param(resource_memory_interval_timer_sec, ulong, 0600);
module_param(resource_memory_deadline_timer_sec, ulong, 0600);
#endif

static unsigned long get_timer_sec(enum timer_type type)
{
	unsigned long sec;

	if (type == ALLOC_TIMER)
		sec = resource_memory_interval_timer_sec;
	else
		sec = resource_memory_deadline_timer_sec;

	if (atomic_read(&kg_memory_restriction_type) == MEMORY_WEAK)
		sec *= 2;

	return sec;
}

static void kg_memory_timer_start(enum timer_type type)
{
	u64 sec;
	struct alarm *alarm;

	if (type == ALLOC_TIMER)
		alarm = &kg_memory_alloc_timer;
	else
		alarm = &kg_memory_deadline_timer;

	sec = get_timer_sec(type);

	KG_LOG("%s timer type %d is started %lu sec\n", __func__, type, sec);
	alarm_start_relative(alarm, ktime_set(sec, 0));
}

static void kg_memory_timer_stop(enum timer_type type)
{
	int ret;
	struct alarm *alarm;

	if (type == ALLOC_TIMER)
		alarm = &kg_memory_alloc_timer;
	else
		alarm = &kg_memory_deadline_timer;

	ret = alarm_cancel(alarm);
	if (ret == 1)
		KG_LOG("%s timer type %d stopped\n", __func__, type);
	else if (ret == 0)
		KG_LOG("%s timer type %d was not active\n", __func__, type);
}

static enum alarmtimer_restart kg_memory_deadline_timer_func(struct alarm *alarm, ktime_t now)
{
	unsigned int key_info_err = kg_hlmd_get_key_info_error();
	int type = atomic_read(&kg_memory_restriction_type);

	KG_LOG("%s deadline timer expired: %lu sec\n", __func__, get_timer_sec(DEADLINE_TIMER));
	kg_drv_info_print();

	if (key_info_err)
		panic("reboot by kg key info error: 0x%x, type: %d", key_info_err, type);
	else
		panic("reboot by kg policy (memory restriction: %d)", type);

	return ALARMTIMER_NORESTART;
}

static enum alarmtimer_restart kg_memory_alloc_timer_func(struct alarm *alarm, ktime_t now)
{
	KG_LOG("%s alloc timer expired: %lu sec\n", __func__, get_timer_sec(ALLOC_TIMER));
	schedule_work(&kg_memory_alloc_work);

	/* restart alloc timer */
	kg_memory_timer_start(ALLOC_TIMER);

	return ALARMTIMER_NORESTART;
}

static unsigned long kg_memory_buffer_free(struct kg_memory_buffer *buffer)
{
	unsigned long size = 0;

	if (!buffer)
		goto out;

	size = buffer->size;
	vfree(buffer->addr);
	kfree(buffer);
out:
	return size;
}

static struct kg_memory_buffer *kg_memory_buffer_alloc(unsigned long size)
{
	struct kg_memory_buffer *buffer;

	buffer = kmalloc(sizeof(struct kg_memory_buffer), GFP_KERNEL);
	if (ZERO_OR_NULL_PTR(buffer)) {
		KG_LOG("%s failed to kmalloc\n", __func__);
		goto kmalloc_failed;
	}

	buffer->addr = vmalloc(size);
	if (!buffer->addr) {
		KG_LOG("%s failed to vmalloc %lu MB\n", __func__, size / SZ_1M);
		goto vmalloc_failed;
	}
	buffer->size = size;

	return buffer;

vmalloc_failed:
	kfree(buffer);
kmalloc_failed:
	return NULL;
}

static unsigned long kg_memory_cal_size(unsigned long alloced_sec, unsigned long cur_size)
{
	unsigned long diff_sec = (ktime_get_boottime_ns() / NSEC_PER_SEC) - alloced_sec;
	int ratio_index = diff_sec / get_timer_sec(ALLOC_TIMER);
	unsigned long size, ratio;

	if (ratio_index >= NUM_OF_RATIO)
		ratio_index = NUM_OF_RATIO - 1;

	ratio = memory_restriction_table[resource_memory_size_idx][ratio_index];
	size = PAGES_TO_B(total_pages) * ratio / 100;

	if (size > cur_size)
		size -= cur_size;
	else
		size = 0;

	KG_LOG("%s idx: %d diff: %lu sec size: %lu MB cur: %lu MB\n", __func__,
			ratio_index, diff_sec, size / SZ_1M, cur_size / SZ_1M);

	return size;
}

static void kg_memory_alloc_workfn(struct work_struct *work)
{
	unsigned long size;
	struct kg_memory_buffer *buffer;

	size = kg_memory_cal_size(resource_memory_restriction_time_sec, resource_memory_restriction_size);
	if (size) {
		buffer = kg_memory_buffer_alloc(size);
		if (!buffer)
			goto out;

		mutex_lock(&memory_buffer_list_mutex);
		list_add_tail(&buffer->list, &memory_buffer_list);
		mutex_unlock(&memory_buffer_list_mutex);

		resource_memory_restriction_size += size;
	}
out:
	KG_LOG("%s %lu sec %lu MB\n", __func__, resource_memory_restriction_time_sec,
			resource_memory_restriction_size / SZ_1M);
}

static unsigned long kg_memory_free_all_buffers(void)
{
	unsigned long freed = 0;
	struct kg_memory_buffer *s, *s2;
	LIST_HEAD(to_destroy);

	mutex_lock(&memory_buffer_list_mutex);
	list_splice_init(&memory_buffer_list, &to_destroy);
	mutex_unlock(&memory_buffer_list_mutex);

	if (list_empty(&to_destroy))
		goto out;

	list_for_each_entry_safe(s, s2, &to_destroy, list) {
		freed += kg_memory_buffer_free(s);
	}

out:
	return freed;
}

static void kg_resource_memory_restriction_start(void)
{
	if (atomic_read(&kg_memory_restriction_started))
		return;

	atomic_set(&kg_memory_restriction_started, 1);
	resource_memory_restriction_time_sec = ktime_get_boottime_ns() / NSEC_PER_SEC;
	kg_memory_timer_start(DEADLINE_TIMER);
	kg_memory_timer_start(ALLOC_TIMER);
}

static void kg_resource_memory_restriction_stop(void)
{
	unsigned long freed_1st, freed_2nd, alloced_size;

	if (!atomic_read(&kg_memory_restriction_started))
		return;

	/* try to free all buffers earlier */
	freed_1st = kg_memory_free_all_buffers();

	/* need to stop timer before flush_work */
	kg_memory_timer_stop(DEADLINE_TIMER);
	kg_memory_timer_stop(ALLOC_TIMER);

	/* make sure that alloc work does not work */
	flush_work(&kg_memory_alloc_work);

	/* try to free remained buffers */
	freed_2nd = kg_memory_free_all_buffers();

	alloced_size = resource_memory_restriction_size;
	resource_memory_restriction_size = 0;
	resource_memory_restriction_time_sec = 0;
	KG_LOG("%s (%lu + %lu) MB freed (%lu)\n", __func__, freed_1st / SZ_1M,
		       freed_2nd / SZ_1M, alloced_size / SZ_1M);

	atomic_set(&kg_memory_restriction_started, 0);
}

static int kg_resource_memory_notifier(struct notifier_block *nb, unsigned long resource, void *unused)
{
#if IS_ENABLED(CONFIG_KG_DRV_TEST)
	if (resource_memory_alloc_test_size_mb) {
		KG_LOG("%s ignore: 0x%lx , %lu MB alloced by alloc test node\n",
			__func__, resource, resource_memory_alloc_test_size_mb);
		return NOTIFY_DONE;
	}
#endif

	if (resource & KG_RESOURCE_MEMORY_WEAK) {
		atomic_set(&kg_memory_restriction_type, MEMORY_WEAK);
		kg_resource_memory_restriction_start();
	} else if (resource & KG_RESOURCE_MEMORY_STRONG) {
		atomic_set(&kg_memory_restriction_type, MEMORY_STRONG);
		kg_resource_memory_restriction_start();
	} else {
		atomic_set(&kg_memory_restriction_type, MEMORY_NO_RESTRICTION);
		kg_resource_memory_restriction_stop();
	}

	return NOTIFY_DONE;
}

static struct notifier_block kg_resource_memory_block = {
	.notifier_call = kg_resource_memory_notifier,
	.priority = 1
};

static int kg_resource_memory_info_notifier(struct notifier_block *nb, unsigned long unused, void *v)
{
	struct seq_file *m = (struct seq_file *)v;
	int restriction_started = atomic_read(&kg_memory_restriction_started);
	int restriction_type = atomic_read(&kg_memory_restriction_type);

	if (m) {
		seq_puts(m, "== resource memory ==\n");
		seq_printf(m, "\tTotal Ram: %lu MB , memory_size_idx: %d\n",
				PAGES_TO_MB(total_pages), resource_memory_size_idx);
		seq_printf(m, "\tdeadline_timer_sec: %lu , interval_timer_sec: %lu\n",
				get_timer_sec(DEADLINE_TIMER),
				get_timer_sec(ALLOC_TIMER));
		seq_printf(m, "\trestriction_started: %d , restriction_type: %d\n",
				restriction_started, restriction_type);
		seq_printf(m, "\t\tstart time: %lu sec , alloced size: %lu MB\n",
				resource_memory_restriction_time_sec,
				resource_memory_restriction_size / SZ_1M);
#if IS_ENABLED(CONFIG_KG_DRV_TEST)
		seq_printf(m, "\talloc test: %u MB\n",	resource_memory_alloc_test_size_mb);
#endif
		seq_puts(m, "\n");
	} else {
		KG_LOG("== resource memory ==\n");
		KG_LOG("\tTotal Ram: %lu MB , memory_size_idx: %d\n",
				PAGES_TO_MB(total_pages), resource_memory_size_idx);
		KG_LOG("\tdeadline_timer_sec: %lu , interval_timer_sec: %lu\n",
				get_timer_sec(DEADLINE_TIMER),
				get_timer_sec(ALLOC_TIMER));
		KG_LOG("\trestriction_started: %d , restriction_type: %d\n",
				restriction_started, restriction_type);
		KG_LOG("\t\tstart time: %lu sec , alloced size: %lu MB\n",
				resource_memory_restriction_time_sec,
				resource_memory_restriction_size / SZ_1M);
#if IS_ENABLED(CONFIG_KG_DRV_TEST)
		KG_LOG("\talloc test: %u MB\n",	resource_memory_alloc_test_size_mb);
#endif
		KG_LOG("\n");

	}

	return NOTIFY_DONE;
}

static struct notifier_block kg_resource_memory_info_block = {
	.notifier_call = kg_resource_memory_info_notifier,
	.priority = 1
};

static unsigned int get_memory_size_idx(unsigned long pages)
{
	unsigned int idx;

	if (pages < GB_TO_PAGES(3))
		idx = DRAM_3GB_OR_LOWER;
	else if (pages < GB_TO_PAGES(4))
		idx = DRAM_4GB;
	else if (pages < GB_TO_PAGES(6))
		idx = DRAM_6GB;
	else
		idx = DRAM_8GB_OR_HIGHER;

	return idx;
}

#if defined(CONFIG_ANDROID_VENDOR_HOOKS)
static void kg_resource_memory_show_mem(void *data, unsigned int filter, nodemask_t *nodemask)
{
	if (!resource_memory_restriction_size)
		return;

	pr_info("KGAlloced: %lu kB\n", resource_memory_restriction_size / SZ_1K);
}

static void kg_resource_memory_meminfo(void *data, struct seq_file *m)
{
	if (!resource_memory_restriction_size)
		return;

	seq_printf(m, "%-16s%8lu kB\n", "KGAlloced:", resource_memory_restriction_size / SZ_1K);
}
#endif

void __init kg_resource_memory_init(void)
{
	int ret;

	ret = kg_resource_control_notifier_register(&kg_resource_memory_block);
	if (ret)
		KG_LOG("%s failed to kg_resource_control_notifier_register : %d\n", __func__, ret);

	ret = kg_drv_info_notifier_register(&kg_resource_memory_info_block);
	if (ret)
		KG_LOG("%s failed to kg_drv_info_notifier_register %d\n", __func__, ret);

	alarm_init(&kg_memory_deadline_timer, ALARM_BOOTTIME, kg_memory_deadline_timer_func);
	alarm_init(&kg_memory_alloc_timer, ALARM_BOOTTIME, kg_memory_alloc_timer_func);
	total_pages = totalram_pages();
	resource_memory_size_idx = get_memory_size_idx(total_pages);

	KG_LOG("%s Total Ram %lu MB, %u\n", __func__, PAGES_TO_MB(total_pages), resource_memory_size_idx);

#if defined(CONFIG_ANDROID_VENDOR_HOOKS)
	register_trace_android_vh_show_mem(kg_resource_memory_show_mem, NULL);
	register_trace_android_vh_meminfo_proc_show(kg_resource_memory_meminfo, NULL);
#endif
}
