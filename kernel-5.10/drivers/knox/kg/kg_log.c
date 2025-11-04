// SPDX-License-Identifier: GPL-2.0
/*
 * kg_log.c/
 *
 * Copyright (C) 2025 Samsung Electronics
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/sched/clock.h>
#include <linux/rtc.h>
#include <linux/seq_file.h>

#include "kg_log.h"

static void *kg_log_buffer;
static unsigned long curr_log_idx;
static int max_index;

static DEFINE_SPINLOCK(kg_log_lock);

#define KG_LOG_SIZE SZ_32K
#define COMM_LEN 16
#define LOGLINE_MAX 81

struct kg_log_struct { // 128 bytes
	u64 ts_nsec_kernel; // kernel log time, 8 bytes
	u64 ts_nsec_boot; // uptime, 8 bytes
	u64 ts_nsec_real; // real time, 8 bytes
	char comm[COMM_LEN]; // 16 bytes
	pid_t pid; // 4 bytes
	u8 cpu; // 1 bytes
	u8 in_interrupt; // 1 bytes
	bool valid; // 1 bytes
	char buf[LOGLINE_MAX];
};

static struct kg_log_struct *get_new_log_entry(void)
{
	struct kg_log_struct *log_entry = (struct kg_log_struct *)kg_log_buffer;
	unsigned long index, flags;

	spin_lock_irqsave(&kg_log_lock, flags);
	index = curr_log_idx++;
	index %= max_index;
	log_entry[index].valid = false;
	spin_unlock_irqrestore(&kg_log_lock, flags);

	return &log_entry[index];
}

static void copy_log_entry(struct kg_log_struct *origin, struct kg_log_struct *target)
{
	unsigned long flags;

	spin_lock_irqsave(&kg_log_lock, flags);
	*target = *origin;
	spin_unlock_irqrestore(&kg_log_lock, flags);
}

static unsigned long get_curr_log_idx(void)
{
	unsigned long idx, flags;

	spin_lock_irqsave(&kg_log_lock, flags);
	idx = curr_log_idx;
	spin_unlock_irqrestore(&kg_log_lock, flags);

	return idx;
}

static void set_valid(struct kg_log_struct *log_entry)
{
	unsigned long flags;

	spin_lock_irqsave(&kg_log_lock, flags);
	log_entry->valid = true;
	spin_unlock_irqrestore(&kg_log_lock, flags);
}

void kg_log(const char *fmt, ...)
{
	struct kg_log_struct *log_entry;
	va_list args;
	int n;

	if (!kg_log_buffer)
		return;

	log_entry = get_new_log_entry();

	log_entry->ts_nsec_kernel = local_clock();
	log_entry->ts_nsec_boot = ktime_get_boottime_ns();
	log_entry->ts_nsec_real = ktime_get_real_ns();
	snprintf(log_entry->comm, COMM_LEN, "%s", current->comm);
	log_entry->pid = task_pid_nr(current);
	log_entry->cpu = raw_smp_processor_id();
	log_entry->in_interrupt = in_interrupt() ? 1 : 0;

	va_start(args, fmt);
	n = vsnprintf(log_entry->buf, LOGLINE_MAX, fmt, args);
	va_end(args);

	set_valid(log_entry);
}

void kg_log_u64_array(char *str, unsigned long data[], int size)
{
	char buf[LOGLINE_MAX];
	int ret;
	int remain = LOGLINE_MAX;
	char *p = buf;
	int i;

	ret = snprintf(p, remain, "%s", str);
	if (ret < 0)
		goto out;

	p += ret;
	remain -= ret;
	if (remain <= 0)
		goto print_str;

	for (i = 0; i < size; i++) {
		ret = snprintf(p, remain, "%lu,", data[i]);
		if (ret < 0)
			goto out;

		p += ret;
		remain -= ret;
		if (remain <= 0)
			goto print_str;
	}

	/* remove last ',' */
	if (remain && *(p - 1) == ',')
		*(p - 1) = '\0';

print_str:
	KG_LOG("%s\n", buf);
out:
	return;
}

static void print_log_entry(struct seq_file *m, struct kg_log_struct *log_entry)
{
	struct kg_log_struct log_tmp;
	struct rtc_time tm;
	unsigned long ts_nsec, rem_nsec;

	copy_log_entry(log_entry, &log_tmp);

	if (!log_tmp.valid)
		return;

	// kernel log time
	ts_nsec = log_tmp.ts_nsec_kernel;
	rem_nsec = do_div(ts_nsec, NSEC_PER_SEC);
	if (m)
		seq_printf(m, "[%5lu.%06lu] ", ts_nsec, rem_nsec / 1000);
	else
		pr_cont("[%5lu.%06lu] ", ts_nsec, rem_nsec / 1000);

	// uptime
	ts_nsec = log_tmp.ts_nsec_boot;
	rem_nsec = do_div(ts_nsec, NSEC_PER_SEC);
	if (m)
		seq_printf(m, "[%5lu.%06lu] ", ts_nsec, rem_nsec / 1000);
	else
		pr_cont("[%5lu.%06lu] ", ts_nsec, rem_nsec / 1000);

	// real time
	ts_nsec = log_tmp.ts_nsec_real;
	rem_nsec = do_div(ts_nsec, NSEC_PER_SEC);
	rtc_time64_to_tm(ts_nsec, &tm);
	if (m)
		seq_printf(m, "[%d-%02d-%02d %02d:%02d:%02d.%06lu UTC] ",
				tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
				tm.tm_hour, tm.tm_min, tm.tm_sec, rem_nsec / 1000);
	else
		pr_cont("[%d-%02d-%02d %02d:%02d:%02d.%06lu UTC] ",
				tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
				tm.tm_hour, tm.tm_min, tm.tm_sec, rem_nsec / 1000);

	// meta info
	if (m)
		seq_printf(m, "%c[%1d:%15s:%5d] ", log_tmp.in_interrupt ? 'I' : ' ',
			log_tmp.cpu, log_tmp.comm, log_tmp.pid);
	else
		pr_cont("%c[%1d:%15s:%5d] ", log_tmp.in_interrupt ? 'I' : ' ',
			log_tmp.cpu, log_tmp.comm, log_tmp.pid);

	// log
	if (m)
		seq_printf(m, "%s", log_tmp.buf);
	else
		pr_cont("%s", log_tmp.buf);
}

void kg_log_print(struct seq_file *m)
{
	struct kg_log_struct *log_entry = (struct kg_log_struct *)kg_log_buffer;
	unsigned long curr_idx;
	int start_idx, end_idx, i;

	if (!log_entry)
		return;

	curr_idx = get_curr_log_idx();

	if (curr_idx >= max_index) {
		start_idx = (curr_idx % max_index);
		end_idx = ((curr_idx - 1) % max_index);
	} else {
		start_idx = 0;
		end_idx = curr_idx - 1;
	}

	if (start_idx) {
		for (i = start_idx ; i < max_index; i++)
			print_log_entry(m, &log_entry[i]);
	}

	for (i = 0; i <= end_idx; i++)
		print_log_entry(m, &log_entry[i]);

}

static int kg_log_show(struct seq_file *m, void *private)
{
	kg_log_print(m);

	return 0;
}

DEFINE_PROC_SHOW_ATTRIBUTE(kg_log);

int __init kg_log_init(void)
{
	struct proc_dir_entry *proc_entry;
	int log_entry_size = sizeof(struct kg_log_struct);

	kg_log_buffer = vzalloc(KG_LOG_SIZE);
	if (!kg_log_buffer) {
		pr_err("%s: failed to vzalloc\n", __func__);
		goto vzalloc_failed;
	}

	proc_entry = proc_create("kg_log", 0440, NULL, &kg_log_proc_ops);
	if (!proc_entry) {
		pr_err("%s : failed to create proc entry\n", __func__);
		goto proc_create_failed;
	}

	max_index = KG_LOG_SIZE / log_entry_size;
	KG_LOG("%s max_index: %d , entry: %d\n", __func__, max_index, log_entry_size);

	return 0;

proc_create_failed:
	vfree(kg_log_buffer);
	kg_log_buffer = NULL;
vzalloc_failed:
	return -ENOMEM;
}
