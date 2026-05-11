// SPDX-License-Identifier: GPL-2.0
/*
 * kg_main.c/
 *
 * Copyright (C) 2025 Samsung Electronics
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/timer.h>
#include <linux/kthread.h>

#include <linux/delay.h>

#include <linux/proc_fs.h>

#include "kg_main.h"
#include "kg_key_info.h"
#include "kg_resource_control.h"
#include "kg_ta_call.h"
#include "kg_dr.h"
#include "kg_netlink.h"
#include "kg_kernel_api.h"

enum {
	KG_STATE_PREENROLL = 0,
	KG_STATE_CHECKING,
	KG_STATE_ACTIVE,
	KG_STATE_LOCK,
	KG_STATE_COMPLETE,
	KG_STATE_ERROR,
	KG_STATE_BROKEN,
	KG_STATE_ALLZERO,
};

enum {
	KG_KEY_INFO_ERR_FUSEBIT_BIT,
	KG_KEY_INFO_ERR_AP_SERIAL_BIT,
	KG_KEY_INFO_ERR_DEVICE_STATE_BIT,
	KG_KEY_INFO_ERR_CLIENT_ERR_BIT,
	KG_KEY_INFO_ERR_NO_UPDATE_BIT,
	KG_KEY_INFO_ERR_TA_ERR_BIT,
	__MAX_KG_KEY_INFO_ERR,
};

enum {
	KG_KSI_ACTION_TYPE_CHECKIN = 0,
	KG_KSI_ACTION_TYPE_LOCK_UNLOCK,
	KG_KSI_ACTION_TYPE_DR,
	KG_KSI_ACTION_TYPE_CLIENT_WAKEUP = 10,
};

#define KG_KSI_VERSION 1
#define KG_KSI_BUF_LEN 200

#define RETRY_TIME_10_SECS (10 * MSEC_PER_SEC)

#define KG_KEY_INFO_ERR_FUSEBIT ((1UL << KG_KEY_INFO_ERR_FUSEBIT_BIT))
#define KG_KEY_INFO_ERR_AP_SERIAL ((1UL << KG_KEY_INFO_ERR_AP_SERIAL_BIT))
#define KG_KEY_INFO_ERR_DEVICE_STATE ((1UL << KG_KEY_INFO_ERR_DEVICE_STATE_BIT))
#define KG_KEY_INFO_ERR_CLIENT_ERR ((1UL << KG_KEY_INFO_ERR_CLIENT_ERR_BIT))
#define KG_KEY_INFO_ERR_NO_UPDATE ((1UL << KG_KEY_INFO_ERR_NO_UPDATE_BIT))
#define KG_KEY_INFO_ERR_TA_ERR ((1UL << KG_KEY_INFO_ERR_TA_ERR_BIT))

struct kg_hlmd_info_t {
	struct task_struct *thread;
	struct kg_key_info key_info;
	unsigned int key_info_err;
	unsigned int lock_decision;
	atomic_t is_running;
	unsigned long err_cnt[__MAX_KG_KEY_INFO_ERR];
	bool default_lock_enabled;
	tz_kg_kernel_lockinfo_t ta_info;
} kg_hlmd_info;

static atomic_t hlmd_wakeup_req_count;
static DECLARE_WAIT_QUEUE_HEAD(kg_hlmd_req_wait);
static DECLARE_WAIT_QUEUE_HEAD(kg_hlmd_done_wait);

static void kg_hlmd_timer_start(bool periodic_check);

static void kg_hlmd_set_is_running(bool is_running)
{
	atomic_set(&kg_hlmd_info.is_running, is_running ? 1 : 0);
}

static bool kg_hlmd_is_running(void)
{
	if (atomic_read(&kg_hlmd_info.is_running))
		return true;

	return false;
}

void kg_hlmd_wakeup(bool from_client)
{
	if (!kg_hlmd_is_running()) {
		KG_LOG("%s kg hlmd is not running\n", __func__);
		return;
	}

	KG_LOG("%s start\n", __func__);
	atomic_inc(&hlmd_wakeup_req_count);
	wake_up(&kg_hlmd_req_wait);

	if (from_client) {
		if (wait_event_interruptible(kg_hlmd_done_wait,
					     atomic_read(&hlmd_wakeup_req_count) == 0)) {
			KG_LOG("%s failed to wait_event_interruptible\n", __func__);
		}
		kg_hlmd_timer_start(true);
	}
	KG_LOG("%s end\n", __func__);
}

unsigned int kg_hlmd_get_key_info_error(void)
{
	return kg_hlmd_info.key_info_err;
}

static bool use_alarm_timer;
static struct alarm kg_hlmd_alarmtimer;
static atomic_t kg_hlmd_timer_initialized;
static unsigned long hlmd_periodic_check_sec = 60 * 60; // 1 hour
static unsigned long hlmd_first_check_sec = 60 * 10; // 10 minutes
static unsigned long hlmd_timer_start_sec;

static void kg_hlmd_timer_expire_func(void)
{
	kg_hlmd_wakeup(false);

	if (kg_hlmd_is_running())
		kg_hlmd_timer_start(true);
}

static void kg_hlmd_timer_func(struct timer_list *timer)
{
	KG_LOG("%s timer expired\n", __func__);
	kg_hlmd_timer_expire_func();
}

static DEFINE_TIMER(kg_hlmd_timer, kg_hlmd_timer_func);

#if IS_ENABLED(CONFIG_KG_DRV_TEST)
module_param(hlmd_periodic_check_sec, ulong, 0600);
module_param(hlmd_first_check_sec, ulong, 0600);
#endif

static bool kg_hlmd_timer_is_initialized(void)
{
	if (atomic_read(&kg_hlmd_timer_initialized))
		return true;

	return false;
}

static enum alarmtimer_restart kg_hlmd_alarmtimer_func(struct alarm *alarm, ktime_t now)
{
	KG_LOG("%s timer expired\n", __func__);
	kg_hlmd_timer_expire_func();

	return ALARMTIMER_NORESTART;
}

static void kg_hlmd_timer_start(bool periodic_check)
{
	unsigned long sec = periodic_check ? hlmd_periodic_check_sec : hlmd_first_check_sec;

	if (!kg_hlmd_timer_is_initialized()) {
		KG_LOG("%s kg hlmd timer is not initialized\n", __func__);
		return;
	}

	if (!kg_hlmd_is_running()) {
		KG_LOG("%s kg hlmd is not running\n", __func__);
		return;
	}

	KG_LOG("%s timer is started %lu sec\n", __func__, sec);
	hlmd_timer_start_sec = ktime_get_boottime_ns() / NSEC_PER_SEC;

	if (use_alarm_timer)
		alarm_start_relative(&kg_hlmd_alarmtimer, ktime_set(sec, 0));
	else
		mod_timer(&kg_hlmd_timer, get_jiffies_64() + sec * HZ);
}

static void kg_hlmd_timer_stop(void)
{
	int ret;

	if (!kg_hlmd_timer_is_initialized()) {
		KG_LOG("%s kg hlmd timer is not initialized\n", __func__);
		return;
	}

	if (use_alarm_timer) {
		ret = alarm_cancel(&kg_hlmd_alarmtimer);
		if (ret == 1)
			KG_LOG("%s timer is stopped\n", __func__);
		else if (ret == 0)
			KG_LOG("%s timer was not active\n", __func__);
	} else {
		ret = del_timer(&kg_hlmd_timer);
		if (ret == 1)
			KG_LOG("%s active timer is stopped\n", __func__);
		else if (ret == 0)
			KG_LOG("%s inactive timer is stopped\n", __func__);
	}
}

static void kg_hlmd_timer_init(void)
{
	if (use_alarm_timer)
		alarm_init(&kg_hlmd_alarmtimer, ALARM_BOOTTIME, kg_hlmd_alarmtimer_func);

	atomic_set(&kg_hlmd_timer_initialized, 1);

	/* It will be restarted at the boot completed */
	kg_hlmd_timer_start(true);
}

#if IS_ENABLED(CONFIG_KG_DRV_TEST)
static int kg_hlmd_timer_test(const char *buf, const struct kernel_param *kp)
{
	u64 value;
	int err;

	err = kstrtou64(buf, 10, &value);
	if (err) {
		KG_LOG("%s failed to kstrtou64 by %x\n", __func__, err);
		return err;
	}

	if (value == 0) {
		kg_hlmd_timer_stop();
		kg_resource_control_notifier_call(0);
	} else if (value == 1) {
		kg_hlmd_timer_start(true);
	} else if (value == 2) {
		kg_hlmd_timer_start(false);
	}

	return 0;
}
module_param_call(hlmd_timer_test, &kg_hlmd_timer_test, NULL, NULL, 0200);
#endif

static int kg_boot_notify(const char *buf, const struct kernel_param *kp)
{
	u64 value;
	int err;

	if (!kg_hlmd_is_running())
		return 0;

	err = kstrtou64(buf, 10, &value);
	if (err) {
		KG_LOG("%s failed to kstrtou64 by %x\n", __func__, err);
		return err;
	}

	if (value == 1) {
		KG_LOG("%s boot completed\n", __func__);
		kg_hlmd_wakeup(false);
		kg_hlmd_timer_start(false);
	} else if (value == 2) {
		KG_LOG("%s on shutdown\n", __func__);
	} else {
		KG_LOG("%s %d\n", __func__, value);
	}

	return 0;
}
module_param_call(boot_notify, &kg_boot_notify, NULL, NULL, 0200);

static void kg_print_err_cnt(struct seq_file *m)
{
	int i;

	if (m) {
		seq_puts(m, "\terr_cnt: ");

		for (i = 0; i < __MAX_KG_KEY_INFO_ERR; i++)
			seq_printf(m, "%lu ", kg_hlmd_info.err_cnt[i]);

		seq_puts(m, "\n");
	} else {
		kg_log_u64_array("\terr_cnt: ", kg_hlmd_info.err_cnt,
				__MAX_KG_KEY_INFO_ERR);
	}
}

static void kg_update_err_cnt(unsigned int key_info_err)
{
	int i;

	if (key_info_err) {
		for (i = 0; i < __MAX_KG_KEY_INFO_ERR; i++) {
			if (key_info_err & (1 << i))
				kg_hlmd_info.err_cnt[i]++;
		}
	}
}

static unsigned int kg_validate_client_key_info(void)
{
	struct kg_key_info client;
	struct kg_key_info *kgdrv = &kg_hlmd_info.key_info;
	unsigned int err = 0;
	unsigned long time_diff_sec, time_compare_sec;

	kg_get_client_key_info(&client);

	kgdrv->ts_nsec_update = ktime_get_boottime_ns();
	time_diff_sec = (kgdrv->ts_nsec_update - client.ts_nsec_update) / NSEC_PER_SEC;

	if (client.ts_nsec_update) {
		if (strncmp(kgdrv->fuse_bit, client.fuse_bit, LEN_FUSE_BIT) != 0) {
			KG_LOG("%s fuse_bit: %s %s\n", __func__,
					kgdrv->fuse_bit, client.fuse_bit);
			err |= KG_KEY_INFO_ERR_FUSEBIT;
		}

		if (strncasecmp(kgdrv->ap_serial, client.ap_serial, LEN_AP_SERIAL) != 0) {
			KG_LOG("%s kg_drv ap_serial: %s\n", __func__, kgdrv->ap_serial);
			KG_LOG("%s client ap_serial: %s\n", __func__, client.ap_serial);
			err |= KG_KEY_INFO_ERR_AP_SERIAL;
		}

		if (kgdrv->device_state != client.device_state) {
			KG_LOG("%s device_state: %d %d\n", __func__,
					kgdrv->device_state, client.device_state);
			err |= KG_KEY_INFO_ERR_DEVICE_STATE;
		}

		if (client.err) {
			KG_LOG("%s client err: 0x%lx\n", __func__, client.err);
			// err |= KG_KEY_INFO_ERR_CLIENT_ERR;
		}

		time_compare_sec = hlmd_periodic_check_sec;
	} else {
		time_compare_sec = hlmd_first_check_sec;
	}

	if (time_diff_sec >= time_compare_sec) {
		KG_LOG("%s time_diff_sec: %lu\n", __func__, time_diff_sec);
		err |= KG_KEY_INFO_ERR_NO_UPDATE;
	}

	if (err)
		kg_key_info_print(&client);

	return err;
}

static unsigned int kg_get_resource_lock_policy(void)
{
	unsigned int lock_policy = 0;

	if (kg_hlmd_info.ta_info.memory_performance == 1)
		lock_policy |= KG_RESOURCE_MEMORY_WEAK;
	else if (kg_hlmd_info.ta_info.memory_performance == 2)
		lock_policy |= KG_RESOURCE_MEMORY_STRONG;
	if (kg_hlmd_info.ta_info.deny_camera)
		lock_policy |= KG_RESOURCE_CAMERA;

	return lock_policy;
}

static unsigned int kg_get_action_target(unsigned int *key_info_err, bool dr_hl_disabled,
		bool dr_dl_disabled)
{
	int state = kg_hlmd_info.key_info.device_state;
	unsigned long locks_summary = kg_hlmd_info.ta_info.locks_summary;
	unsigned int resource_lock_policy = kg_get_resource_lock_policy();
	unsigned int resource_lock_decision = 0;

	if (dr_hl_disabled) {
		if (*key_info_err & (~KG_KEY_INFO_ERR_TA_ERR)) {
			KG_LOG("%s ignore all of err except ta err: 0x%x\n", __func__, *key_info_err);
			*key_info_err &= KG_KEY_INFO_ERR_TA_ERR;
		}

		if (*key_info_err) {
			if (!kg_hlmd_info.default_lock_enabled || dr_dl_disabled) {
				KG_LOG("%s ignore ta err: %d %d\n", __func__,
						kg_hlmd_info.default_lock_enabled,
						dr_dl_disabled);
				*key_info_err = 0;
			}
		}
	}

	if (*key_info_err) {
		if (!dr_hl_disabled) {
			if (resource_lock_policy & KG_RESOURCE_MEMORY_WEAK)
				resource_lock_decision |= KG_RESOURCE_MEMORY_WEAK;
			else
				resource_lock_decision |= KG_RESOURCE_MEMORY_STRONG;
		}

		if (kg_hlmd_info.default_lock_enabled && !dr_dl_disabled)
			resource_lock_decision |= KG_RESOURCE_CAMERA;

		if (!resource_lock_decision)
			*key_info_err = 0;
	} else if (state == KG_STATE_LOCK || state == KG_STATE_ERROR) {
		if (kg_hlmd_info.default_lock_enabled && !dr_dl_disabled)
			resource_lock_decision |= KG_RESOURCE_CAMERA;
	} else if (state == KG_STATE_ACTIVE) {
		if (!dr_dl_disabled)
			resource_lock_decision |= resource_lock_policy;
	}

	KG_LOG("%s s: %d , r: 0x%lx , e: 0x%x , p: 0x%x , d: 0x%x\n", __func__,
		state, locks_summary, *key_info_err, resource_lock_policy, resource_lock_decision);

	return resource_lock_decision;
}

static void kg_ksi_checkin_events(int ta_err)
{
	char actionType[2];
	char details[KG_KSI_BUF_LEN];
	int ret;
	struct kg_key_info *kg_drv = &kg_hlmd_info.key_info;

	ret = snprintf(actionType, sizeof(actionType), "%d", KG_KSI_ACTION_TYPE_CHECKIN);
	if (ret < 0)
		goto out;

	ret = snprintf(details, KG_KSI_BUF_LEN, "kgState=%d;"
			"actionStatus=%d;actionErrorCode=%lx;fuseBit=%s;",
			ta_err ? KG_STATE_ERROR : kg_drv->device_state,
			ta_err ? 1 : 0, (long)ta_err, kg_drv->fuse_bit);
	if (ret < 0)
		goto out;

	kgsm_send_message(actionType, details, KG_KSI_VERSION);

	return;
out:
	KG_LOG("%s error: %d\n", __func__, ret);
}

static unsigned int ksiPreviousActionTarget;
static void kg_ksi_lock_events(bool is_dr, unsigned int actionTarget, unsigned int actionReason,
		unsigned int originalActionTarget, unsigned int originalActionReason)
{
	char actionType[3];
	char details[KG_KSI_BUF_LEN];
	int ret;
	struct kg_key_info *kg_drv = &kg_hlmd_info.key_info;
	unsigned long actionErrorCode = kg_hlmd_info.ta_info.locks_summary;

	if (actionTarget == ksiPreviousActionTarget)
		return;

	ret = snprintf(actionType, sizeof(actionType), "%d",
			is_dr ? KG_KSI_ACTION_TYPE_DR : KG_KSI_ACTION_TYPE_LOCK_UNLOCK);
	if (ret < 0)
		goto out;

	ret = snprintf(details, KG_KSI_BUF_LEN, "kgState=%d;"
			"actionStatus=%d;actionErrorCode=%lx;fuseBit=%s;"
			"actionTarget=%x;previousActionTarget=%x;actionReason=%x;",
			kg_drv->device_state,
			0, actionErrorCode, kg_drv->fuse_bit,
			actionTarget, ksiPreviousActionTarget, actionReason);
	if (ret < 0)
		goto out;

	if (is_dr) {
		ret = snprintf(details + ret, KG_KSI_BUF_LEN - ret,
				"originalActionTarget=%x;originalActionReason=%x;",
				originalActionTarget, originalActionReason);
		if (ret < 0)
			goto out;
	}

	ksiPreviousActionTarget = actionTarget;
	kgsm_send_message(actionType, details, KG_KSI_VERSION);

	/* try to wakeup client */
	if (actionReason & KG_KEY_INFO_ERR_NO_UPDATE) {
		ret = snprintf(actionType, sizeof(actionType), "%d",
			KG_KSI_ACTION_TYPE_CLIENT_WAKEUP);
		if (ret < 0)
			goto out;

		kgsm_send_message(actionType, NULL, KG_KSI_VERSION);
	}

	return;
out:
	KG_LOG("%s error: %d\n", __func__, ret);
}

static void kg_apply_lock_policy(unsigned int key_info_err)
{
	bool dr_hl_disabled = kg_dr_hl_disabled();
	bool dr_dl_disabled = kg_dr_dl_disabled();
	bool is_dr_set = (dr_hl_disabled || dr_dl_disabled);
	unsigned int dr_key_info_err = key_info_err, final_key_info_err;
	unsigned int action_target, dr_action_target, final_action_target;

	action_target = kg_get_action_target(&key_info_err, false, false);
	if (is_dr_set) {
		dr_action_target = kg_get_action_target(&dr_key_info_err,
				dr_hl_disabled, dr_dl_disabled);
		final_action_target = dr_action_target;
		final_key_info_err = dr_key_info_err;
	} else {
		final_action_target = action_target;
		final_key_info_err = key_info_err;
	}

	kg_hlmd_info.key_info_err = final_key_info_err;
	kg_hlmd_info.lock_decision = final_action_target;

	kg_resource_control_notifier_call(final_action_target);

	kg_ksi_lock_events(is_dr_set, final_action_target, final_key_info_err,
			action_target, key_info_err);
}

#if IS_ENABLED(CONFIG_KG_DRV_TEST)
static int kg_ksi_events_test_handler(const char *buf, const struct kernel_param *kp)
{
	char actionType[2];
	char details[KG_KSI_BUF_LEN];
	int version = 0;
	int ret;

	ret = sscanf(buf, "%s %s %d", actionType, details, &version);
	if (ret != 3) {
		KG_LOG("%s: failed to sscanf %d\n", __func__, ret);
		return 0;
	}

	KG_LOG("kg_ksi_test ret: %d actionType: %s details: %s version: %d\n",
			ret, actionType, details, version);

	kgsm_send_message(actionType, details, KG_KSI_VERSION);
	return 0;
}

module_param_call(ksi_events_test, &kg_ksi_events_test_handler, NULL, NULL, 0200);
#endif

static bool kg_device_state_is_completed(void)
{
	return kg_hlmd_info.key_info.device_state == KG_STATE_COMPLETE;
}

static int kg_update_ta_info(void)
{
	int ret, retry_cnt = 3;
	tz_kg_kernel_lockinfo_t info;

	memset(&info, 0, sizeof(tz_kg_kernel_lockinfo_t));

retry:
	ret = kg_ta_get_info(&info);
	if (ret || info.kg_state > KG_STATE_ERROR || info.kg_state <= KG_STATE_CHECKING) {
		if (!kg_dr_hl_disabled() || !kg_dr_dl_disabled()) {
			if (retry_cnt--) {
				msleep(RETRY_TIME_10_SECS);
				goto retry;
			} else {
				KG_LOG("%s retry failed ret: 0x%x , state: %d\n",
					__func__, ret, info.kg_state);

				// return error when kg_state is wrong
				if (!ret)
					ret = -1;
			}
		}
	} else {
		kg_hlmd_info.ta_info = info;
		kg_hlmd_info.key_info.device_state = info.kg_state;
	}

	return ret;
}

static int kg_set_key_info_err(unsigned int err)
{
	int ret, retry_cnt = 3;
	bool dr_hl_disabled = kg_dr_hl_disabled();

	if (err && dr_hl_disabled) {
		KG_LOG("%s ignore all err: 0x%x\n", __func__, err);
		err = 0;
	}

retry:
	ret = kg_ta_set_key_info_err(err);
	if (ret) {
		if (!dr_hl_disabled) {
			if (retry_cnt--) {
				msleep(RETRY_TIME_10_SECS);
				goto retry;
			} else {
				KG_LOG("%s retry failed err: 0x%x , ret: 0x%x\n", __func__, err, ret);
			}
		}
	}

	return ret;
}

static int kg_hlmd(void *data)
{
	int ta_err;
	unsigned int key_info_err;

	KG_LOG("%s start\n", __func__);

	while (!kthread_should_stop()) {
		if (wait_event_interruptible(kg_hlmd_req_wait,
					     atomic_read(&hlmd_wakeup_req_count) > 0)) {
			KG_LOG("%s failed to wait_event_interruptible\n", __func__);
			goto out;
		}

		if (kthread_should_stop()) {
			KG_LOG("%s kthread_should_stop\n", __func__);
			goto out;
		}

		KG_LOG("%s loop start\n", __func__);

		ta_err = kg_update_ta_info();
		if (ta_err) {
			key_info_err = KG_KEY_INFO_ERR_TA_ERR;
			goto set_key_info_err;
		}

		if (kg_device_state_is_completed()) {
			KG_LOG("%s device state is completed\n", __func__);
			goto out;
		}

		key_info_err = kg_validate_client_key_info();

set_key_info_err:
		ta_err = kg_set_key_info_err(key_info_err);
		if (ta_err) {
			key_info_err |= KG_KEY_INFO_ERR_TA_ERR;
		} else {
			ta_err = kg_update_ta_info();
			if (ta_err)
				key_info_err |= KG_KEY_INFO_ERR_TA_ERR;
		}

		kg_ksi_checkin_events(ta_err);
		kg_apply_lock_policy(key_info_err);
		kg_update_err_cnt(key_info_err);

		if (atomic_dec_and_test(&hlmd_wakeup_req_count))
			wake_up_all(&kg_hlmd_done_wait);

		KG_LOG("%s loop end\n", __func__);
	}

out:
	kg_hlmd_set_is_running(false);
	kg_hlmd_timer_stop();
	if (atomic_read(&hlmd_wakeup_req_count)) {
		atomic_set(&hlmd_wakeup_req_count, 0);
		wake_up_all(&kg_hlmd_done_wait);
	}
	kg_ksi_checkin_events(0);
	kg_set_key_info_err(0);
	kg_apply_lock_policy(0);
	KG_LOG("%s end\n", __func__);

	return 0;
}

RAW_NOTIFIER_HEAD(kg_drv_info_notifier);

int kg_drv_info_notifier_register(struct notifier_block *n)
{
	int ret;

	ret = raw_notifier_chain_register(&kg_drv_info_notifier, n);
	if (ret)
		KG_LOG("%s failed to register notifier : %d\n", __func__, ret);

	return ret;
}

static void kg_print_hlmd_info(struct seq_file *m)
{
	struct kg_key_info *kg_drv = &kg_hlmd_info.key_info;
	tz_kg_kernel_lockinfo_t *ta_info = &kg_hlmd_info.ta_info;

	if (m) {
		seq_puts(m, "== hlmd info ==\n");
		seq_printf(m, "\tfuse_bit: %s , ap_serial: %s\n",
				kg_drv->fuse_bit, kg_drv->ap_serial);
		seq_printf(m, "\tis_running: %d , default_lock_enabled: %d\n",
				kg_hlmd_is_running(),
				kg_hlmd_info.default_lock_enabled ? 1 : 0);
		seq_printf(m, "\tdevice state: %d , latest update: %lu sec\n",
				kg_drv->device_state,
				kg_drv->ts_nsec_update / NSEC_PER_SEC);
		seq_printf(m, "\tkey_info_err: 0x%x , lock_decision: 0x%x\n",
				kg_hlmd_info.key_info_err,
				kg_hlmd_info.lock_decision);
		kg_print_err_cnt(m);

		seq_puts(m, "\tTA info\n");
		seq_printf(m, "\t\tkg_state: %u\n", ta_info->kg_state);
		seq_printf(m, "\t\tlocks_summary: 0x%lx\n",
			ta_info->locks_summary);

		seq_printf(m, "\t\tkey_info_error: 0x%x\n",
			ta_info->key_info_error);
		seq_puts(m, "\t\tresource restriction policy\n");
		seq_printf(m, "\t\t\tmemory: %u , camera: %u\n",
			ta_info->memory_performance, ta_info->deny_camera);

		seq_puts(m, "\ttimer info\n");
		seq_printf(m, "\t\tuse_alarm_timer: %d\n", use_alarm_timer);
		seq_printf(m, "\t\tperiodic_check: %lu sec , first_check: %lu sec\n",
				hlmd_periodic_check_sec, hlmd_first_check_sec);
		seq_printf(m, "\t\tstart_time: %lu sec , cur_time: %lu sec\n",
				hlmd_timer_start_sec,
				ktime_get_boottime_ns() / NSEC_PER_SEC);
		seq_puts(m, "\n");
	} else {
		KG_LOG("== hlmd info ==\n");
		KG_LOG("\tfuse_bit: %s , ap_serial: %s\n",
				kg_drv->fuse_bit, kg_drv->ap_serial);
		KG_LOG("\tis_running: %d , default_lock_enabled: %d\n",
				kg_hlmd_is_running(),
				kg_hlmd_info.default_lock_enabled ? 1 : 0);
		KG_LOG("\tdevice state: %d , latest update: %lu sec\n",
				kg_drv->device_state,
				kg_drv->ts_nsec_update / NSEC_PER_SEC);
		KG_LOG("\tkey_info_err: 0x%x , lock_decision: 0x%x\n",
				kg_hlmd_info.key_info_err,
				kg_hlmd_info.lock_decision);
		kg_print_err_cnt(m);

		KG_LOG("\tTA info\n");
		KG_LOG("\t\tkg_state: %u\n", ta_info->kg_state);
		KG_LOG("\t\tlocks_summary: 0x%lx\n",
			ta_info->locks_summary);

		KG_LOG("\t\tkey_info_error: 0x%x\n",
			ta_info->key_info_error);
		KG_LOG("\t\tresource restriction policy\n");
		KG_LOG("\t\t\tmemory: %u , camera: %u\n",
			ta_info->memory_performance, ta_info->deny_camera);

		KG_LOG("\ttimer info\n");
		KG_LOG("\t\tuse_alarm_timer: %d\n", use_alarm_timer);
		KG_LOG("\t\tperiodic_check: %lu sec , first_check: %lu sec\n",
				hlmd_periodic_check_sec, hlmd_first_check_sec);
		KG_LOG("\t\tstart_time: %lu sec , cur_time: %lu sec\n",
				hlmd_timer_start_sec,
				ktime_get_boottime_ns() / NSEC_PER_SEC);
		KG_LOG("\n");
	}
}

void kg_drv_info_print(void)
{
	// print kg log first
	kg_log_print(NULL);
	kg_print_hlmd_info(NULL);
	raw_notifier_call_chain(&kg_drv_info_notifier, 0, NULL);
}

static int kg_drv_info_show(struct seq_file *m, void *private)
{
	kg_print_hlmd_info(m);
	raw_notifier_call_chain(&kg_drv_info_notifier, 0, m);
	return 0;
}

DEFINE_PROC_SHOW_ATTRIBUTE(kg_drv_info);

static void kg_drv_create_info_node(void)
{
	struct proc_dir_entry *proc_entry;

	proc_entry = proc_create("kg_drv_info", 0440, NULL, &kg_drv_info_proc_ops);
	if (!proc_entry)
		KG_LOG("%s : failed to create proc entry\n", __func__);
}

static int __init kg_hlm_init(void)
{
	KG_LOG("%s fuse_bit: %s ap_serial: %s\n", __func__,
		kg_hlmd_info.key_info.fuse_bit, kg_hlmd_info.key_info.ap_serial);

	kg_hlmd_info.thread = kthread_run(kg_hlmd, NULL, "kg_hlmd");

	if (IS_ERR(kg_hlmd_info.thread)) {
		KG_LOG("%s failed to kthread_run\n", __func__);
		return -1;
	}

	kg_hlmd_set_is_running(true);

	kg_hlmd_info.default_lock_enabled = true;
	KG_LOG("%s default lock is enabled\n", __func__);

	kg_hlmd_timer_init();

	return 0;
}

static int __init kg_drv_init(void)
{
	int ret;
	unsigned long default_lock = 0;

	kg_drv_create_info_node();
	if (!kg_key_info_init(&kg_hlmd_info.key_info))
		return 0;

	kg_dr_init();
	if (!kg_is_finance_device()) {
		KG_LOG("%s: non finance device\n", __func__);
		return 0;
	}

	ret = kg_log_init();
	KG_LOG("%s: kg_log_init %d\n", __func__, ret);

	ret = kgsm_netlink_init();
	KG_LOG("%s: kgsm_netlink_init %d\n", __func__, ret);

	if (kg_hlm_init())
		return 0;

	if (kg_hlmd_info.default_lock_enabled) {
		default_lock = KG_RESOURCE_CAMERA;
		ksiPreviousActionTarget = KG_RESOURCE_CAMERA;
	}

	kg_resource_control_init(default_lock);
	kg_resource_memory_init();
	kg_resource_cpu_init();
	kg_ta_call_init();

	return 0;
}

module_init(kg_drv_init);
MODULE_LICENSE("GPL");
