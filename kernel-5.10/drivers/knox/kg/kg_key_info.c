// SPDX-License-Identifier: GPL-2.0
/*
 * kg_key_info.c/
 *
 * Copyright (C) 2025 Samsung Electronics
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/rtc.h>

#include "kg_main.h"
#include "kg_key_info.h"

#define STR(x) _STR(x)
#define _STR(x) #x

#define KG_CLIENT_UID 5959
#define ANDROID_MAX_UID 100000

static struct kg_key_info kg_client_key_info;
static bool kg_drv_key_info_initialized;

static unsigned int enable;
module_param(enable, uint, 0400);

static char *fuse_bit;
module_param(fuse_bit, charp, 0400);

static char *ap_serial;
module_param(ap_serial, charp, 0400);

char *kg_get_ap_serial(void)
{
	return ap_serial;
}

bool kg_is_finance_device(void)
{
	/* fuse_bit = "10" => fuse_bit[0] = '1' , fuse_bit[1] => '0'
	 * fuse_bit = "11" => fuse_bit[0] = '1' , fuse_bit[1] => '1'
	 */

	if (kg_drv_key_info_initialized && fuse_bit[0] == '1')
		return true;

	return false;
}

#define NUM_UPDATE_CNT 18
static unsigned long client_key_info_update_cnt[NUM_UPDATE_CNT];

static unsigned int kg_drv_enabled(void)
{
	return enable;
}

static void kg_print_update_cnt(struct seq_file *m)
{
	int i;

	if (m) {
		seq_puts(m, "\t\tclient_update_cnt: ");

		for (i = 0; i < NUM_UPDATE_CNT; i++)
			seq_printf(m, "%lu ", client_key_info_update_cnt[i]);

		seq_puts(m, "\n");
	} else {
		kg_log_u64_array("\t\tclient_update_cnt: ", client_key_info_update_cnt,
				NUM_UPDATE_CNT);
	}
}

static void kg_inc_update_cnt(unsigned long diff_sec)
{
	int i = diff_sec / (60 * 10);

	if (i >= NUM_UPDATE_CNT)
		i = NUM_UPDATE_CNT - 1;

	client_key_info_update_cnt[i]++;
}

static DEFINE_MUTEX(client_key_info_mutex);

void kg_get_client_key_info(struct kg_key_info *tmp)
{
	mutex_lock(&client_key_info_mutex);
	*tmp = kg_client_key_info;
	mutex_unlock(&client_key_info_mutex);
}

static bool kg_try_get_client_key_info(struct kg_key_info *tmp)
{
	bool ret = false;

	if (mutex_trylock(&client_key_info_mutex)) {
		*tmp = kg_client_key_info;
		mutex_unlock(&client_key_info_mutex);
		ret = true;
	}

	return ret;
}

static void kg_set_client_key_info(struct kg_key_info *tmp)
{
	unsigned long time_diff = tmp->ts_nsec_update - kg_client_key_info.ts_nsec_update;

	kg_inc_update_cnt(time_diff / NSEC_PER_SEC);
	mutex_lock(&client_key_info_mutex);
	kg_client_key_info = *tmp;
	mutex_unlock(&client_key_info_mutex);
}

#if IS_ENABLED(CONFIG_KG_DRV_TEST)
static bool client_key_info_test_enabled;
#endif

static int kg_client_key_info_show(char *buf, const struct kernel_param *kp)
{
	int i;

	i = sprintf(buf, "0x%x", kg_hlmd_get_key_info_error());

	return i;
}

static int kg_client_key_info_handler(const char *buf, const struct kernel_param *kp)
{
	struct kg_key_info tmp;
	int ret;
	unsigned int uid = (unsigned int)current_uid().val % ANDROID_MAX_UID;

	if (!kg_drv_enabled()) {
		KG_LOG("%s kg_drv is disabled\n", __func__);
		return 0;
	}

	if (!kg_is_finance_device()) {
		KG_LOG("%s non finance device\n", __func__);
		return 0;
	}

#if IS_ENABLED(CONFIG_KG_DRV_TEST)
	if (uid == 0) {
		if (buf[0] == 'x' || buf[0] == 'X') {
			client_key_info_test_enabled = false;
			return 0;
		} else {
			client_key_info_test_enabled = true;
		}
	} else if (client_key_info_test_enabled) {
		KG_LOG("%s client_key_info_test_enabled %u\n", __func__, uid);
		return 0;
	} else if (uid != KG_CLIENT_UID) {
		KG_LOG("%s write by uid %d is not allowed\n", __func__, uid);
		return 0;
	}
#else
	if (uid != KG_CLIENT_UID) {
		KG_LOG("%s write by uid %d is not allowed\n", __func__, uid);
		return 0;
	}
#endif

	ret = sscanf(buf, "%" STR(LEN_FUSE_BIT) "s:%" STR(LEN_AP_SERIAL) "s:%1d:0x%16lx",
			tmp.fuse_bit,
			tmp.ap_serial,
			&tmp.device_state,
			&tmp.err);

	tmp.ts_nsec_update = ktime_get_boottime_ns();
	KG_LOG("%s %d %d\n", __func__, ret, uid);

	if (ret == 4) {
		kg_set_client_key_info(&tmp);
	} else {
		KG_LOG("%s uid: %d , ret: %d\n", __func__, uid, ret);
		KG_LOG("%s\n", buf);
		KG_LOG("%s input format is wrong\n", __func__);
	}

	kg_hlmd_wakeup(true);

	return 0;
}
module_param_call(key_info, &kg_client_key_info_handler, &kg_client_key_info_show, NULL, 0660);

static int kg_key_info_notifier(struct notifier_block *nb, unsigned long unused, void *v)
{
	struct seq_file *m = (struct seq_file *)v;
	struct kg_key_info client;

	if (m) {
		seq_puts(m, "== key info ==\n");
		seq_printf(m, "\tkg_drv_enabled: %d , kg_drv_key_info_initialized: %d\n",
				kg_drv_enabled(), kg_drv_key_info_initialized ? 1 : 0);
		seq_printf(m, "\tfuse_bit: %s , ap_serial: %s\n", fuse_bit, ap_serial);

		if (kg_drv_enabled() && kg_is_finance_device()) {
			kg_get_client_key_info(&client);
			seq_puts(m, "\tclient info\n");
			kg_print_update_cnt(m);
#if IS_ENABLED(CONFIG_KG_DRV_TEST)
			seq_printf(m, "\t\tclient_key_info_test_enabled: %d\n",
					client_key_info_test_enabled ? 1 : 0);
#endif
			seq_printf(m, "\t\tfuse_bit: %s , ap_serial: %s\n",
					client.fuse_bit, client.ap_serial);
			seq_printf(m, "\t\tdevice state: %d , err: 0x%lx , update: %lu sec\n",
					client.device_state, client.err,
					client.ts_nsec_update / NSEC_PER_SEC);
		}

		seq_puts(m, "\n");
	} else {
		KG_LOG("== key info ==\n");
		KG_LOG("\tkg_drv_enabled: %d , kg_drv_key_info_initialized: %d\n",
				kg_drv_enabled(), kg_drv_key_info_initialized ? 1 : 0);
		KG_LOG("\tfuse_bit: %s , ap_serial: %s\n", fuse_bit, ap_serial);

		if (kg_drv_enabled() && kg_is_finance_device()) {
			KG_LOG("\tclient info\n");
			kg_print_update_cnt(m);
#if IS_ENABLED(CONFIG_KG_DRV_TEST)
			KG_LOG("\t\tclient_key_info_test_enabled: %d\n",
					client_key_info_test_enabled ? 1 : 0);
#endif
			if (kg_try_get_client_key_info(&client)) {
				KG_LOG("\t\tfuse_bit: %s , ap_serial: %s\n",
						client.fuse_bit, client.ap_serial);
				KG_LOG("\t\tdevice state: %d , err: 0x%lx , update: %lu sec\n",
						client.device_state, client.err,
						client.ts_nsec_update / NSEC_PER_SEC);
			}
		}

		KG_LOG("\n");
	}

	return NOTIFY_DONE;
}

static struct notifier_block kg_key_info_block = {
	.notifier_call = kg_key_info_notifier,
	.priority = 1
};

bool __init kg_key_info_init(struct kg_key_info *kg_drv_key_info)
{
	int len_fuse_bit, len_ap_serial, ret;

	ret = kg_drv_info_notifier_register(&kg_key_info_block);
	if (ret)
		KG_LOG("%s failed to kg_drv_info_notifier_register %d\n", __func__, ret);

	if (!kg_drv_enabled())
		goto out;

	if (!fuse_bit || !ap_serial)
		goto out;

	len_fuse_bit = strlen(fuse_bit);
	len_ap_serial = strlen(ap_serial);

	if (len_fuse_bit != LEN_FUSE_BIT) {
		KG_LOG("%s fuse_bit is invalid %s\n", __func__, fuse_bit);
		goto out;
	}

	if (len_ap_serial != LEN_AP_SERIAL) {
		KG_LOG("%s ap_serial is invalid %s\n", __func__, ap_serial);
		goto out;
	}

	strncpy(kg_drv_key_info->fuse_bit, fuse_bit, LEN_FUSE_BIT);
	strncpy(kg_drv_key_info->ap_serial, ap_serial, LEN_AP_SERIAL);
	kg_drv_key_info_initialized = true;

out:
	return kg_drv_key_info_initialized;
}
