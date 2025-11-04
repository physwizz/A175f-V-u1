// SPDX-License-Identifier: GPL-2.0
/*
 * kg_resource_cpu.c/
 *
 * Copyright (C) 2025 Samsung Electronics
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/cpufreq.h>
#include <linux/alarmtimer.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

#include "kg_main.h"
#include "kg_resource_control.h"

enum cpu_restriction_type {
	CPU_NO_RESTRICTION,
	CPU_WEAK,
	CPU_STRONG,
};

#define NO_LIMIT 100
#define LIMIT_TO_MINFREQ 0

static atomic_t kg_cpu_restriction_started;
static atomic_t kg_cpu_restriction_type;
static struct freq_qos_request __percpu *pcpu_max_freq_limit_req;
static unsigned long *cpu_maxfreq_limit_info;
static unsigned long resource_cpu_restriction_time_sec;
static int resource_cpu_restriction_percent = NO_LIMIT;
static struct alarm kg_cpu_limit_maxfreq_timer;
static unsigned long resource_cpu_limit_maxfreq_timer_sec = (KG_RESOURCE_DEVICE_REBOOT_SEC / 2); // 30 minutes
static void kg_cpu_limit_maxfreq_workfn(struct work_struct *work);
static DECLARE_WORK(kg_cpu_limit_maxfreq_work, kg_cpu_limit_maxfreq_workfn);

#if IS_ENABLED(CONFIG_KG_DRV_TEST)
static int resource_cpu_limit_maxfreq_ratio_test = NO_LIMIT;

static void kg_resource_cpu_limit_maxfreq(int ratio);
static int kg_resource_cpu_limit_maxfreq_test(const char *buf, const struct kernel_param *kp)
{
	u64 value;
	int err;

	err = kstrtou64(buf, 10, &value);
	if (err) {
		KG_LOG("%s failed to kstrtou64 by %x\n", __func__, err);
		return err;
	}

	if (value > NO_LIMIT)
		value = NO_LIMIT;

	if (!pcpu_max_freq_limit_req) {
		pcpu_max_freq_limit_req = alloc_percpu(struct freq_qos_request);
		if (!pcpu_max_freq_limit_req) {
			KG_LOG("%s: failed to alloc_percpu\n", __func__);
			goto out;
		}
	}

	if (ZERO_OR_NULL_PTR(cpu_maxfreq_limit_info)) {
		cpu_maxfreq_limit_info = kcalloc(nr_cpu_ids, sizeof(unsigned long), GFP_KERNEL);
		if (ZERO_OR_NULL_PTR(cpu_maxfreq_limit_info)) {
			KG_LOG("%s: failed to kcalloc\n", __func__);
			goto out;
		}
	}

	resource_cpu_limit_maxfreq_ratio_test = value;
	kg_resource_cpu_limit_maxfreq(value);
out:
	return 0;
}
module_param_call(resource_cpu_limit_maxfreq_test, &kg_resource_cpu_limit_maxfreq_test, NULL, NULL, 0200);
module_param(resource_cpu_limit_maxfreq_timer_sec, ulong, 0600);
#endif

static int max_freq_limit_get_target_freq(struct cpufreq_policy *policy, int cpu, int ratio)
{
	struct cpufreq_frequency_table *pos, *table = policy->freq_table;
	int cal_freq = policy->cpuinfo.max_freq * ratio / 100;
	int real_freq = policy->cpuinfo.min_freq;

	if (!table) {
		KG_LOG("%s: cpu%d policy->freq_table is NULL\n", __func__, cpu);
		return -ENODEV;
	}

	if (policy->freq_table_sorted == CPUFREQ_TABLE_UNSORTED) {
		KG_LOG("%s: cpu%d freq_table_sorted is CPUFREQ_TABLE_UNSORTED\n", __func__, cpu);
		return -EINVAL;
	}

	if (ratio == LIMIT_TO_MINFREQ)
		return real_freq;

	cpufreq_for_each_valid_entry(pos, table) {
		if (policy->freq_table_sorted == CPUFREQ_TABLE_SORTED_ASCENDING &&
				cal_freq < pos->frequency)
			break;

		real_freq = pos->frequency;

		if (policy->freq_table_sorted == CPUFREQ_TABLE_SORTED_DESCENDING &&
				cal_freq >= pos->frequency)
			break;
	}

	return real_freq;
}

static void max_freq_limit_add_qos(int cpu, int ratio)
{
	struct freq_qos_request *req = per_cpu_ptr(pcpu_max_freq_limit_req, cpu);
	struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);
	int ret = 0;
	int target_freq;

	if (!policy) {
		KG_LOG("%s: cpu%d no policy\n", __func__, cpu);
		return;
	}

	if (freq_qos_request_active(req)) {
		ret = freq_qos_remove_request(req);
		if (ret < 0) {
			KG_LOG("%s: cpu%d failed to freq_qos_remove_request (%d)\n", __func__, cpu, ret);
			goto put_policy;
		}
	}

	if (ratio == NO_LIMIT) {
		cpu_maxfreq_limit_info[cpu] = 0;
		goto put_policy;
	}

	target_freq = max_freq_limit_get_target_freq(policy, cpu, ratio);
	if (target_freq <= 0) {
		KG_LOG("%s: cpu%d failed to max_freq_limit_get_target_freq\n", __func__, cpu);
		goto put_policy;
	}

	ret = freq_qos_add_request(&policy->constraints, req, FREQ_QOS_MAX, target_freq);
	if (ret < 0)
		KG_LOG("%s: cpu%d failed to freq_qos_add_request (%d)\n", __func__, cpu, ret);
	else
		cpu_maxfreq_limit_info[cpu] = target_freq;

put_policy:
	cpufreq_cpu_put(policy);
}

static void kg_resource_cpu_limit_maxfreq(int ratio)
{
	int cpu;

	resource_cpu_restriction_percent = ratio;
	if (ratio == NO_LIMIT)
		KG_LOG("%s remove limit maxfreq\n", __func__);
	else if (ratio == LIMIT_TO_MINFREQ)
		KG_LOG("%s limit maxfreq to minfreq\n", __func__);
	else
		KG_LOG("%s: limit maxfreq to %d percent\n", __func__, ratio);

	for_each_possible_cpu(cpu) {
		max_freq_limit_add_qos(cpu, ratio);
	}
}

static unsigned long get_timer_sec(void)
{
	unsigned long sec = resource_cpu_limit_maxfreq_timer_sec;

	if (atomic_read(&kg_cpu_restriction_type) == CPU_WEAK)
		sec *= 2;

	return sec;
}

static void kg_cpu_timer_stop(void)
{
	int ret;

	ret = alarm_cancel(&kg_cpu_limit_maxfreq_timer);
	if (ret == 1)
		KG_LOG("%s timer is stopped\n", __func__);
	else if (ret == 0)
		KG_LOG("%s timer was not active\n", __func__);
}

static void kg_cpu_timer_start(void)
{
	u64 sec = get_timer_sec();

	KG_LOG("%s timer is started %lu sec\n", __func__, sec);
	alarm_start_relative(&kg_cpu_limit_maxfreq_timer, ktime_set(sec, 0));
}

static void kg_cpu_limit_maxfreq_workfn(struct work_struct *work)
{
	kg_resource_cpu_limit_maxfreq(LIMIT_TO_MINFREQ);
}

static enum alarmtimer_restart kg_cpu_limit_maxfreq_timer_func(struct alarm *alarm, ktime_t now)
{
	KG_LOG("%s limit maxfreq timer expired: %lu sec\n", __func__,
			get_timer_sec());
	schedule_work(&kg_cpu_limit_maxfreq_work);

	return ALARMTIMER_NORESTART;
}

static void kg_resource_cpu_restriction_start(void)
{
	if (atomic_read(&kg_cpu_restriction_started))
		return;

	atomic_set(&kg_cpu_restriction_started, 1);
	resource_cpu_restriction_time_sec = ktime_get_boottime_ns() / NSEC_PER_SEC;
	kg_resource_cpu_limit_maxfreq(CONFIG_KG_RESOURCE_CPU_MAXFREQ_LIMIT);
	kg_cpu_timer_start();
}

static void kg_resource_cpu_restriction_stop(void)
{
	if (!atomic_read(&kg_cpu_restriction_started))
		return;

	/* need to stop timer before flush_work */
	kg_cpu_timer_stop();

	/* make sure that limit maxfreq work does not work */
	flush_work(&kg_cpu_limit_maxfreq_work);

	resource_cpu_restriction_time_sec = 0;
	kg_resource_cpu_limit_maxfreq(NO_LIMIT);
	atomic_set(&kg_cpu_restriction_started, 0);
}

static int kg_resource_cpu_notifier(struct notifier_block *nb, unsigned long resource, void *unused)
{
#if IS_ENABLED(CONFIG_KG_DRV_TEST)
	if (resource_cpu_limit_maxfreq_ratio_test != NO_LIMIT) {
		KG_LOG("%s ignore: 0x%lx , ratio %d is set by test node\n",
			__func__, resource, resource_cpu_limit_maxfreq_ratio_test);
		return NOTIFY_DONE;
	}
#endif

	if (resource & KG_RESOURCE_MEMORY_WEAK) {
		atomic_set(&kg_cpu_restriction_type, CPU_WEAK);
		kg_resource_cpu_restriction_start();
	} else if (resource & KG_RESOURCE_MEMORY_STRONG) {
		atomic_set(&kg_cpu_restriction_type, CPU_STRONG);
		kg_resource_cpu_restriction_start();
	} else {
		atomic_set(&kg_cpu_restriction_type, CPU_NO_RESTRICTION);
		kg_resource_cpu_restriction_stop();
	}

	return NOTIFY_DONE;
}

static struct notifier_block kg_resource_cpu_block = {
	.notifier_call = kg_resource_cpu_notifier,
	.priority = 1
};

static int kg_resource_cpu_info_notifier(struct notifier_block *nb, unsigned long unused, void *v)
{
	struct seq_file *m = (struct seq_file *)v;
	int restriction_started = atomic_read(&kg_cpu_restriction_started);
	int restriction_type = atomic_read(&kg_cpu_restriction_type);
	int i;

	if (m) {
		seq_puts(m, "== resource cpu ==\n");
		seq_printf(m, "\tlimit_maxfreq_timer_sec: %lu\n", get_timer_sec());
		seq_printf(m, "\trestriction_started: %d , restriction_type: %d\n",
				restriction_started, restriction_type);
#if IS_ENABLED(CONFIG_KG_DRV_TEST)
		seq_printf(m, "\tmaxfreq_ratio_test: %d\n", resource_cpu_limit_maxfreq_ratio_test);
#endif
		seq_printf(m, "\t\tpercentage: %d , start time: %lu sec\n",
				resource_cpu_restriction_percent,
				resource_cpu_restriction_time_sec);
		seq_puts(m, "\t\tlimit maxfreq: ");
		for (i = 0; i < nr_cpu_ids; i++)
			seq_printf(m, "%lu ", cpu_maxfreq_limit_info[i]);
		seq_puts(m, "\n");

		seq_puts(m, "\n");
	} else {
		KG_LOG("== resource cpu ==\n");
		KG_LOG("\tlimit_maxfreq_timer_sec: %lu\n", get_timer_sec());
		KG_LOG("\trestriction_started: %d , restriction_type: %d\n",
				restriction_started, restriction_type);

#if IS_ENABLED(CONFIG_KG_DRV_TEST)
		KG_LOG("\tmaxfreq_ratio_test: %d\n", resource_cpu_limit_maxfreq_ratio_test);
#endif
		KG_LOG("\t\tpercentage: %d , start time: %lu sec\n",
				resource_cpu_restriction_percent,
				resource_cpu_restriction_time_sec);
		kg_log_u64_array("\t\tlimit maxfreq: ", cpu_maxfreq_limit_info, nr_cpu_ids);

		KG_LOG("\n");
	}

	return NOTIFY_DONE;
}

static struct notifier_block kg_resource_cpu_info_block = {
	.notifier_call = kg_resource_cpu_info_notifier,
	.priority = 1
};

void __init kg_resource_cpu_init(void)
{
	int ret;

	alarm_init(&kg_cpu_limit_maxfreq_timer, ALARM_BOOTTIME, kg_cpu_limit_maxfreq_timer_func);

	pcpu_max_freq_limit_req = alloc_percpu(struct freq_qos_request);
	if (!pcpu_max_freq_limit_req) {
		KG_LOG("%s: failed to alloc_percpu\n", __func__);
		return;
	}

	cpu_maxfreq_limit_info = kzalloc(sizeof(unsigned long) * nr_cpu_ids, GFP_KERNEL);
	if (ZERO_OR_NULL_PTR(cpu_maxfreq_limit_info)) {
		KG_LOG("%s: failed to kzalloc\n", __func__);
		goto free_percpu;
	}

	ret = kg_resource_control_notifier_register(&kg_resource_cpu_block);
	if (ret)
		KG_LOG("%s failed to kg_resource_control_notifier_register : %d\n", __func__, ret);

	ret = kg_drv_info_notifier_register(&kg_resource_cpu_info_block);
	if (ret)
		KG_LOG("%s failed to kg_drv_info_notifier_register %d\n", __func__, ret);

	return;

free_percpu:
	free_percpu(pcpu_max_freq_limit_req);
	pcpu_max_freq_limit_req = NULL;
}
