// SPDX-License-Identifier: GPL-2.0
/*
 * kg_ta_call.c/
 *
 * Copyright (C) 2025 Samsung Electronics
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>

#include "kg_main.h"
#include "kg_ta_call.h"

#if IS_ENABLED(CONFIG_KG_DRV_TEST)
static int ta_call_ret_test;
module_param(ta_call_ret_test, int, 0600);

static int ta_call_device_state_test;
module_param(ta_call_device_state_test, int, 0600);

static int ta_call_memory_restriction_test;
module_param(ta_call_memory_restriction_test, int, 0600);

static int ta_call_camera_restriction_test;
module_param(ta_call_camera_restriction_test, int, 0600);
#endif

static DEFINE_MUTEX(ta_call_mutex);
static struct wakeup_source *ta_call_wakelock;

static void kg_ta_call_wakelock(bool awake)
{
	if (!ta_call_wakelock) {
		KG_LOG("%s ta_call_wakelock is NULL\n", __func__);
		return;
	}

	if (awake)
		__pm_stay_awake(ta_call_wakelock);
	else
		__pm_relax(ta_call_wakelock);
}

static int kg_ta_call_common(int cmd, kg_ta_call_ctx_t *ctx)
{
	int ret;

	mutex_lock(&ta_call_mutex);
	kg_ta_call_wakelock(true);
	ret = kg_ta_call(cmd, ctx);
	kg_ta_call_wakelock(false);
	mutex_unlock(&ta_call_mutex);

#if IS_ENABLED(CONFIG_KG_DRV_TEST)
	if (ta_call_ret_test) {
		KG_LOG("%s ret : %d ta_call_ret_test: %d\n", __func__, ret, ta_call_ret_test);
		ret = ta_call_ret_test;
	}
#endif

	return ret;
}

static void kg_ta_get_info_hook(void *output, void *resp)
{
	tz_kg_kernel_lockinfo_t *info = (tz_kg_kernel_lockinfo_t *)output;
	tci_kernel_message_t *resp_buf = (tci_kernel_message_t *)resp;

	*info = resp_buf->payload.msg_tz_kg_kernel_lockinfo;
}

int kg_ta_get_info(tz_kg_kernel_lockinfo_t *info)
{
	int ret;
	kg_ta_call_ctx_t ctx = {
		.output = info,
		.kg_ta_call_output_hook = kg_ta_get_info_hook,
	};

	ret = kg_ta_call_common(KG_CMD_KERNEL_GET_LOCKS_SUMMARY, &ctx);

	KG_LOG("%s ret: 0x%x , state: %d\n", __func__, ret, info->kg_state);

#if IS_ENABLED(CONFIG_KG_DRV_TEST)
	if (ta_call_device_state_test) {
		KG_LOG("%s ta_call_device_state_test: %d\n", __func__, ta_call_device_state_test);
		info->kg_state = ta_call_device_state_test;
	}

	if (ta_call_memory_restriction_test) {
		KG_LOG("%s ta_call_memory_restriction_test: %d\n", __func__, ta_call_memory_restriction_test);
		info->memory_performance = ta_call_memory_restriction_test;
	}

	if (ta_call_camera_restriction_test) {
		KG_LOG("%s ta_call_camera_restriction_test: %d\n", __func__, ta_call_camera_restriction_test);
		info->deny_camera = ta_call_camera_restriction_test;
	}
#endif

	return ret;
}

static void kg_ta_set_key_info_err_hook(void *input, void *req)
{
	tz_kg_kernel_lockinfo_t *info = (tz_kg_kernel_lockinfo_t *)input;
	tci_kernel_message_t *req_buf = (tci_kernel_message_t *)req;

	req_buf->payload.msg_tz_kg_kernel_lockinfo = *info;
}


int kg_ta_set_key_info_err(unsigned int err)
{
	int ret;
	tz_kg_kernel_lockinfo_t info = {
		.key_info_error = err,
	};

	kg_ta_call_ctx_t ctx = {
		.input = &info,
		.kg_ta_call_input_hook = kg_ta_set_key_info_err_hook,
	};

	ret = kg_ta_call_common(KG_CMD_KERNEL_SET_KEY_INFO_ERORR_FLAG, &ctx);

	KG_LOG("%s err: 0x%x , ret: 0x%x\n", __func__, err, ret);
	return ret;
}

#if IS_ENABLED(CONFIG_KG_DRV_TEST)
static int kg_ta_call_info_notifier(struct notifier_block *nb, unsigned long unused, void *v)
{
	struct seq_file *m = (struct seq_file *)v;

	if (m) {
		seq_puts(m, "== kg ta call ==\n");
		seq_printf(m, "\tta_call_device_state_test: %d\n", ta_call_device_state_test);
		seq_printf(m, "\tta_call_ret_test: %d\n", ta_call_ret_test);
		seq_printf(m, "\tta_call_memory_restriction_test: %d\n", ta_call_memory_restriction_test);
		seq_printf(m, "\tta_call_camera_restriction_test: %d\n", ta_call_camera_restriction_test);

		seq_puts(m, "\n");
	} else {
		KG_LOG("== kg ta call ==\n");
		KG_LOG("\tta_call_device_state_test: %d\n", ta_call_device_state_test);
		KG_LOG("\tta_call_ret_test: %d\n", ta_call_ret_test);
		KG_LOG("\tta_call_memory_restriction_test: %d\n", ta_call_memory_restriction_test);
		KG_LOG("\tta_call_camera_restriction_test: %d\n", ta_call_camera_restriction_test);

		KG_LOG("\n");
	}

	return NOTIFY_DONE;
}

static struct notifier_block kg_ta_call_info_block = {
	.notifier_call = kg_ta_call_info_notifier,
	.priority = 1
};

static int kg_ta_call_cmd(int cmd)
{
	int ret;

	ret = kg_ta_call_common(cmd, NULL);
	KG_LOG("%s cmd: %d , ret: 0x%x\n", __func__, cmd, ret);

	return ret;
}

static int kg_ta_call_cmd_test(const char *buf, const struct kernel_param *kp)
{
	uint32_t cmd;
	int ret;
	tz_kg_kernel_lockinfo_t info;

	ret = kstrtou32(buf, 10, &cmd);
	if (ret) {
		KG_LOG("%s failed to kstrtou32 by %x\n", __func__, ret);
		return ret;
	}

	KG_LOG("%s cmd %d start\n", __func__, cmd);

	switch (cmd) {
	case KG_CMD_KERNEL_GET_LOCKS_SUMMARY:
		memset(&info, 0, sizeof(tz_kg_kernel_lockinfo_t));
		ret = kg_ta_get_info(&info);
		KG_LOG("%s ret: 0x%x , state: %d\n", __func__, ret, info.kg_state);
		break;
	default:
		ret = kg_ta_call_cmd(cmd);
		break;
	}

	KG_LOG("%s cmd: %d , ret: 0x%x\n", __func__, cmd, ret);

	return 0;
}
module_param_call(ta_call_cmd_test, &kg_ta_call_cmd_test, NULL, NULL, 0200);
#endif

void __init kg_ta_call_init(void)
{
#if IS_ENABLED(CONFIG_KG_DRV_TEST)
	int ret;

	ret = kg_drv_info_notifier_register(&kg_ta_call_info_block);
	if (ret)
		KG_LOG("%s failed to kg_drv_info_notifier_register %d\n", __func__, ret);
#endif
	ta_call_wakelock = wakeup_source_register(NULL, "kg_ta_call");
	if (!ta_call_wakelock)
		KG_LOG("%s failed to wakeup_source_register\n", __func__);
}
