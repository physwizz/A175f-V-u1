// SPDX-License-Identifier: GPL-2.0
/*
 * kg_resource_control.c/
 *
 * Copyright (C) 2025 Samsung Electronics
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>

#include "kg_main.h"
#include "kg_resource_control.h"

BLOCKING_NOTIFIER_HEAD(kg_resource_control_notifier);

static bool resource_control_enabled;

static unsigned long resource_locked;

#if IS_ENABLED(CONFIG_KG_DRV_TEST)
static bool resource_control_test_enabled;

static int kg_resource_control_test(const char *buf, const struct kernel_param *kp)
{
	u64 value;
	int err;

	if (!resource_control_enabled) {
		KG_LOG("%s kg resource control is disabled\n", __func__);
		return 0;
	}

	err = kstrtou64(buf, 10, &value);
	if (err) {
		KG_LOG("%s failed to kstrtou64 by %x\n", __func__, err);
		return err;
	}

	value &= KG_RESOURCES_MASK;
	KG_LOG("%s value: 0x%lx\n", __func__, value);
	if (value)
		resource_control_test_enabled = true;
	else
		resource_control_test_enabled = false;

	resource_locked = value;
	KG_LOG("%s resource_locked: 0x%lx\n", __func__, resource_locked);
	blocking_notifier_call_chain(&kg_resource_control_notifier, resource_locked, NULL);

	return 0;
}
module_param_call(resource_control_test, &kg_resource_control_test, NULL, NULL, 0200);
#endif

int kg_resource_control_notifier_register(struct notifier_block *n)
{
	int ret;

	if (!resource_control_enabled)
		return -ENODEV;

	ret = blocking_notifier_chain_register(&kg_resource_control_notifier, n);
	if (ret)
		KG_LOG("%s failed to register notifier : %d\n", __func__, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(kg_resource_control_notifier_register);

int kg_resource_control_notifier_unregister(struct notifier_block *n)
{
	int ret;

	if (!resource_control_enabled)
		return -ENODEV;

	ret = blocking_notifier_chain_unregister(&kg_resource_control_notifier, n);
	if (ret)
		KG_LOG("%s failed to unregister notifier : %d\n", __func__, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(kg_resource_control_notifier_unregister);

bool kg_resource_is_locked(unsigned long kg_resource)
{
	bool ret = false;

	if (resource_locked & kg_resource)
		ret = true;

	return ret;
}
EXPORT_SYMBOL_GPL(kg_resource_is_locked);

void kg_resource_control_notifier_call(unsigned long value)
{
	if (!resource_control_enabled)
		return;

#if IS_ENABLED(CONFIG_KG_DRV_TEST)
	if (resource_control_test_enabled) {
		KG_LOG("%s %d has been ignored by test value %d\n",
				__func__, value, resource_locked);
		return;
	}
#endif

	value &= KG_RESOURCES_MASK;
	resource_locked = value;
	KG_LOG("%s resource_locked: 0x%lx\n", __func__, resource_locked);
	blocking_notifier_call_chain(&kg_resource_control_notifier, resource_locked, NULL);
}

static int kg_resource_control_info_notifier(struct notifier_block *nb, unsigned long unused, void *v)
{
	struct seq_file *m = (struct seq_file *)v;

	if (m) {
		seq_puts(m, "== resource control ==\n");
#if IS_ENABLED(CONFIG_KG_DRV_TEST)
		seq_printf(m, "\tresource_control_test_enabled: %d\n",
				resource_control_test_enabled ? 1 : 0);
#endif
		seq_printf(m, "\tresource_locked: 0x%lx\n", resource_locked);
		seq_puts(m, "\n");
	} else {
		KG_LOG("== resource control ==\n");
#if IS_ENABLED(CONFIG_KG_DRV_TEST)
		KG_LOG("\tresource_control_test_enabled: %d\n",
				resource_control_test_enabled ? 1 : 0);
#endif
		KG_LOG("\tresource_locked: 0x%lx\n", resource_locked);
		KG_LOG("\n");
	}

	return NOTIFY_DONE;
}

static struct notifier_block kg_resource_control_info_block = {
	.notifier_call = kg_resource_control_info_notifier,
	.priority = 1
};

void __init kg_resource_control_init(unsigned long init_value)
{
	int ret;

	ret = kg_drv_info_notifier_register(&kg_resource_control_info_block);
	if (ret)
		KG_LOG("%s failed to kg_drv_info_notifier_register %d\n", __func__, ret);

	resource_locked = init_value;
	resource_control_enabled = true;
	KG_LOG("%s resource_locked: 0x%lx\n", __func__, resource_locked);
}
