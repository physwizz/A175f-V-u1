/*
 * Copyright (C) 2020 Samsung Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program;
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/version.h>
#include <linux/i2c.h>
#include <linux/clk.h>

#include "sec_star.h"
#include "sec_k250a.h"

//#define K250A_DEBUG

#undef USE_INTERNAL_PULLUP //if use external pull-up, not necessary

#define ERR(msg...)     pr_err("[star-k250a] : " msg)
#define INFO(msg...)    pr_info("[star-k250a] : " msg)

struct k250a_dev {
	struct i2c_client *client;
	struct regulator *vdd;
	unsigned int reset_gpio;
	struct pinctrl *pinctrl;
	struct pinctrl_state *nvm_on_pin;
	struct pinctrl_state *nvm_off_pin;
	struct mutex platform_lock;
	bool platform_opened;
#if defined(USE_INTERNAL_PULLUP)
#define SCL_GPIO_NUM	335
#define SDA_GPIO_NUM	320
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_i2c;
	struct pinctrl_state *pin_gpio;
#endif
	sec_star_t *star;

#ifdef K250A_DEBUG
	/*
	 *  1ms ~ 20ms: usec if val >= 1000 && val < 20000
	 * 20ms ~ 50ms: msec if val >=   20 && val <=   50
	 */
	unsigned int pwr_on_delay;
	unsigned int pwr_off_delay;
#endif
};

static struct k250a_dev g_k250a;

#ifdef K250A_DEBUG
static bool is_delay_usleep_range(unsigned int val)
{
	if (val >= 1000 && val < 20000)
		return true;

	return false;
}

static bool is_delay_msleep(unsigned int val)
{
	if (val >= 20 && val <= 50)
		return true;

	return false;
}

static void k250a_wait(char *tag, unsigned int val)
{
	if (val == 0) {
		INFO("%s no wait\n", tag);
		return;
	}

	if (is_delay_usleep_range(val)) {
		usleep_range(val, val + 1000);
		INFO("%s usleep_range %d\n", tag, val);
		return;
	}

	if (is_delay_msleep(val)) {
		msleep(val);
		INFO("%s msleep %d\n", tag, val);
		return;
	}


	ERR("%s invalid wait %d\n", tag, val);
}
#endif/*K250A_DEBUG*/

static int k250a_poweron(void)
{
	int ret = 0;

	INFO("k250a_poweron\n");

	if (g_k250a.vdd == NULL) {
		if (g_k250a.reset_gpio == 0) {
			ERR("%s: reset_gpio is null!\n", __func__);
			return 0;
		}

		INFO("rest pin control instead of vdd\n");

		gpio_set_value(g_k250a.reset_gpio, 0);
		usleep_range(1000, 2000);
		gpio_set_value(g_k250a.reset_gpio, 1);

		msleep(POWER_ON_DELAY_MS);
		return 0;
	}

	ret = regulator_is_enabled(g_k250a.vdd);
	if (!ret) {
		ret = regulator_enable(g_k250a.vdd);
		if (ret) {
			ERR("%s - enable failed, ret=%d\n", __func__, ret);
			return ret;
		}
	} else if (ret < 0) {
		INFO("%s err on regulator_is_enabled %d\n", __func__, ret);
		return ret;
	} else {
		INFO("%s power is already on\n", __func__);
	}

	usleep_range(1000, 2000);
	if (g_k250a.nvm_on_pin) {
		if (pinctrl_select_state(g_k250a.pinctrl, g_k250a.nvm_on_pin))
			ERR("nvm on pinctrl set error\n");
		else
			INFO("nvm on pinctrl set\n");
	}

#if defined(USE_INTERNAL_PULLUP)
	if (pinctrl_select_state(g_k250a.pinctrl, g_k250a.pin_i2c) < 0) {
		ERR("failed to pinctrl_select_state for gpio");
	}
#endif

#ifndef K250A_DEBUG
	msleep(POWER_ON_DELAY_MS);
#else
	k250a_wait("k250a_poweron", g_k250a.pwr_on_delay);
#endif

	return 0;
}

static int k250a_poweroff(void)
{
	int ret = 0;

	INFO("k250a_poweroff\n");

	if (g_k250a.vdd == NULL) {
		ERR("%s: vdd is null!\n", __func__);
		return 0;
	}
	if (g_k250a.nvm_off_pin) {
		if (pinctrl_select_state(g_k250a.pinctrl, g_k250a.nvm_off_pin))
			ERR("nvm off pinctrl set error\n");
		else
			INFO("nvm off pinctrl set\n");
	}

#if defined(USE_INTERNAL_PULLUP)
	if (pinctrl_select_state(g_k250a.pinctrl, g_k250a.pin_gpio) < 0) {
		ERR("failed to pinctrl_select_state for gpio");
	}
#endif

	ret = regulator_is_enabled(g_k250a.vdd);
	if (ret > 0) {
		ret = regulator_disable(g_k250a.vdd);
		if (ret) {
			ERR("%s - disable failed, ret=%d\n", __func__, ret);
			return ret;
		}
	} else if (ret < 0) {
		INFO("%s err on regulator_is_enabled %d\n", __func__, ret);
		return ret;
	} else {
		INFO("%s power is already off\n", __func__);
	}

#ifndef K250A_DEBUG
	usleep_range(POWER_OF_DELAY_US, POWER_OF_DELAY_US + 5000);
#else
	k250a_wait("k250a_poweroff", g_k250a.pwr_off_delay);
#endif

	return 0;
}

static int k250a_force_off(void)
{
	int ret = 0;
	int retry = 3;

	INFO("k250a force off\n");

	if (g_k250a.vdd == NULL) {

		if (g_k250a.reset_gpio == 0)
			return -ENODEV;

		gpio_set_value(g_k250a.reset_gpio, 0);
	} else {
		//force disable
		while (retry-- > 0 && regulator_is_enabled(g_k250a.vdd)) {
			ret = regulator_disable(g_k250a.vdd);
			if (ret) {
				ERR("%s - disable failed, ret=%d\n", __func__, ret);
				break;
			}
		}
	}
	usleep_range(15000, 20000);

	return ret;
}

static int k250a_reset(void)
{
	int ret = 0;

	ret = k250a_force_off();
	if (ret) {
		ERR("%s - force off failed\n", __func__);
		return ret;
	}

	return k250a_poweron();
}

static star_dev_t star_dev = {
	.name = "k250a",
	.hal_type = SEC_HAL_I2C,
	.client = NULL,
	.power_on = k250a_poweron,
	.power_off = k250a_poweroff,
	.reset = k250a_reset,
	.force_off = k250a_force_off,
};

#ifdef K250A_DEBUG
/** check if buf has range('-') format
 * @buf		buf to be checked
 * @size	buf size
 * @retval	0 if args are ok, -1 if '-' included
 */
static int secdp_check_store_args(const char *buf, size_t size)
{
	int ret;

	if (strnchr(buf, size, '-')) {
		ERR("%s: range is forbidden!\n", __func__);
		ret = -1;
		goto exit;
	}

	ret = 0;
exit:
	return ret;
}

static ssize_t pwr_on_delay_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	int rc = 0;
	unsigned int val = g_k250a.pwr_on_delay;

	INFO("%s: %d\n", __func__, val);

	if (is_delay_msleep(val))
		rc = snprintf(buf, SZ_32, "%d [ms]\n", val);
	else if (is_delay_usleep_range(val))
		rc = snprintf(buf, SZ_32, "%d [us]\n", val);
	else
		rc = snprintf(buf, SZ_32, "??? %d\n", val);

	return rc;
}

static ssize_t pwr_on_delay_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t size)
{
	int val[10] = {0,};

	if (secdp_check_store_args(buf, size)) {
		ERR("%s: args error!\n", __func__);
		goto exit;
	}

	get_options(buf, ARRAY_SIZE(val), val);
	g_k250a.pwr_on_delay = val[1];
	INFO("%s: %d\n", __func__, g_k250a.pwr_on_delay);
exit:
	return size;
}
static CLASS_ATTR_RW(pwr_on_delay);

static ssize_t pwr_off_delay_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	int rc = 0;
	unsigned int val = g_k250a.pwr_off_delay;

	INFO("%s: %d\n", __func__, val);

	if (is_delay_msleep(val))
		rc = snprintf(buf, SZ_32, "%d [ms]\n", val);
	else if (is_delay_usleep_range(val))
		rc = snprintf(buf, SZ_32, "%d [us]\n", val);
	else
		rc = snprintf(buf, SZ_32, "??? %d\n", val);

	return rc;
}

static ssize_t pwr_off_delay_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t size)
{
	int val[10] = {0,};

	if (secdp_check_store_args(buf, size)) {
		ERR("%s: args error!\n", __func__);
		goto exit;
	}

	get_options(buf, ARRAY_SIZE(val), val);
	g_k250a.pwr_off_delay = val[1];
	INFO("%s: %d\n", __func__, g_k250a.pwr_off_delay);
exit:
	return size;
}
static CLASS_ATTR_RW(pwr_off_delay);

struct k250a_sysfs_private {
	struct device	*dev;
	struct class	sysfs_class;
};

struct k250a_sysfs_private *gk250a_sysfs;


enum {
	PWR_ON_DELAY = 0,
	PWR_OFF_DELAY,
};

static struct attribute *k250a_class_attrs[] = {
	[PWR_ON_DELAY]		= &class_attr_pwr_on_delay.attr,
	[PWR_OFF_DELAY]		= &class_attr_pwr_off_delay.attr,
	NULL,

};
ATTRIBUTE_GROUPS(k250a_class);

static int k250a_sysfs_register(struct i2c_client *client)
{
	int rc = 0;
	struct class *sysfs_class = NULL;
	struct k250a_sysfs_private *sysfs = NULL;

	sysfs = devm_kzalloc(&client->dev, sizeof(*sysfs), GFP_KERNEL);
	if (!sysfs) {
		rc = -EINVAL;
		ERR("%s: alloc failed %d\n", __func__, rc);
		goto error;
	}

	sysfs->dev = &client->dev;

	sysfs_class = &sysfs->sysfs_class;
	sysfs_class->name = "k250a";
	sysfs_class->class_groups = k250a_class_groups;
	rc = class_register(sysfs_class);
	if (rc) {
		ERR("%s: cannot register k250a_class %d\n", __func__, rc);
		goto free_class;
	}

	gk250a_sysfs = sysfs;
	INFO("%s: success\n", __func__);
	return rc;

free_class:
	devm_kfree(&client->dev, sysfs);
error:
	return rc;
}

static void k250a_sysfs_unregister(struct i2c_client *client)
{
	struct class *sysfs_class = &gk250a_sysfs->sysfs_class;

	class_unregister(sysfs_class);

	devm_kfree(gk250a_sysfs->dev, gk250a_sysfs);
	gk250a_sysfs = NULL;
}
#endif/*K250A_DEBUG*/

#if (KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE)
static int k250a_probe(struct i2c_client *client)
#else
static int k250a_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
	struct device_node *np = client->dev.of_node;

	INFO("Entry : %s\n", __func__);

	if (np) {
		g_k250a.vdd = devm_regulator_get_optional(&(client->dev), "1p8_pvdd");
		if (IS_ERR(g_k250a.vdd)) {
			ERR("%s - 1p8_pvdd can not be used\n", __func__);
			g_k250a.vdd = NULL;
		}

		if (of_property_read_u32(np, "reset_gpio", &(g_k250a.reset_gpio)) < 0) {
			ERR("%s - Reset Control can not be used\n", __func__);
			g_k250a.reset_gpio = 0;
		} else {
			if (gpio_request(g_k250a.reset_gpio, "sec-reset") < 0) {
				ERR("%s - Failed to request sec-reset gpio\n", __func__);
				g_k250a.reset_gpio = 0;
			} else {
				INFO("%s - Reset GPIO Num : %u\n", __func__, g_k250a.reset_gpio);
				if (gpio_direction_output(g_k250a.reset_gpio, 1) < 0) {
					ERR("Failed to set reset gpio");
				}
			}
		}
	} else {
		return -ENODEV;
	}

	g_k250a.pinctrl = devm_pinctrl_get(client->adapter->dev.parent);
	if (IS_ERR(g_k250a.pinctrl)) {
		ERR("devm_pinctrl_get failed\n");
		return -1;
	}
	g_k250a.nvm_on_pin = pinctrl_lookup_state(g_k250a.pinctrl, "nvm_on");
	if (IS_ERR(g_k250a.nvm_on_pin)) {
		ERR("pinctrl_lookup_state failed-default\n");
		g_k250a.nvm_on_pin = NULL;
	}
	g_k250a.nvm_off_pin = pinctrl_lookup_state(g_k250a.pinctrl, "nvm_off");
	if (IS_ERR(g_k250a.nvm_off_pin)) {
		ERR("pinctrl_lookup_state failed-nvm_off\n");
		g_k250a.nvm_off_pin = NULL;
	} else if (pinctrl_select_state(g_k250a.pinctrl, g_k250a.nvm_off_pin)) {
		ERR("nvm off pinctrl set error\n");
	} else {
		INFO("nvm off pinctrl set\n");
	}

#if defined(USE_INTERNAL_PULLUP)
	g_k250a.pinctrl = devm_pinctrl_get(client->adapter->dev.parent);
	if (IS_ERR(g_k250a.pinctrl)) {
		ERR("failed to devm_pinctrl_get");
		return -1;
	}

	g_k250a.pin_i2c = pinctrl_lookup_state(g_k250a.pinctrl, "default");
	if (IS_ERR(g_k250a.pin_i2c)) {
		ERR("failed to pinctrl_lookup_state for i2c");
		devm_pinctrl_put(g_k250a.pinctrl);
		return -1;
	}

	g_k250a.pin_gpio = pinctrl_lookup_state(g_k250a.pinctrl, "gpio");
	if (IS_ERR(g_k250a.pin_gpio)) {
		ERR("failed to pinctrl_lookup_state for gpio");
		devm_pinctrl_put(g_k250a.pinctrl);
		return -1;
	}

	if (pinctrl_select_state(g_k250a.pinctrl, g_k250a.pin_gpio) < 0) {
		ERR("failed to pinctrl_select_state for gpio");
		devm_pinctrl_put(g_k250a.pinctrl);
		return -1;
	}

	if (gpio_request(SCL_GPIO_NUM, "sec-scl") < 0) {
		ERR("failed to get scl gpio");
		devm_pinctrl_put(g_k250a.pinctrl);
		return -1;
	}

	if (gpio_request(SDA_GPIO_NUM, "sec-sda") < 0) {
		ERR("failed to get sda gpio");
		devm_pinctrl_put(g_k250a.pinctrl);
		return -1;
	}

	if (gpio_direction_output(SCL_GPIO_NUM, 0) < 0) {
		ERR("failed to set scl gpio");
		devm_pinctrl_put(g_k250a.pinctrl);
		return -1;
	}

	if (gpio_direction_output(SDA_GPIO_NUM, 0) < 0) {
		ERR("failed to set sda gpio");
		devm_pinctrl_put(g_k250a.pinctrl);
		return -1;
	}
#endif

	g_k250a.client = client;
	star_dev.client = client;
	g_k250a.star = star_open(&star_dev);
	if (g_k250a.star == NULL) {
		return -ENODEV;
	}

#ifdef K250A_DEBUG
	g_k250a.pwr_on_delay  = 17000;
	g_k250a.pwr_off_delay = 15000;
	k250a_sysfs_register(client);
#endif

	INFO("Exit : %s\n", __func__);
	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int k250a_remove(struct i2c_client *client)
#else
static void k250a_remove(struct i2c_client *client)
#endif
{
	INFO("Entry : %s\n", __func__);

#ifdef K250A_DEBUG
	k250a_sysfs_unregister(client);
#endif

#if defined(USE_INTERNAL_PULLUP)
	devm_pinctrl_put(g_k250a.pinctrl);
	gpio_free(SCL_GPIO_NUM);
	gpio_free(SDA_GPIO_NUM);
#endif
	if (g_k250a.reset_gpio != 0) {
		gpio_free(g_k250a.reset_gpio);
	}
	star_close(g_k250a.star);
	INFO("Exit : %s\n", __func__);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	return 0;
#endif
}

static const struct i2c_device_id k250a_id[] = {
	{"k250a", 0},
	{}
};

static const struct of_device_id k250a_match_table[] = {
	{ .compatible = "sec_k250a",},
	{},
};

static struct i2c_driver k250a_driver = {
	.id_table = k250a_id,
	.probe = k250a_probe,
	.remove = k250a_remove,
	.driver = {
		.name = "k250a",
		.owner = THIS_MODULE,
		.of_match_table = k250a_match_table,
	},
};

static int __init k250a_init(void)
{
	INFO("Entry : %s\n", __func__);

	return i2c_add_driver(&k250a_driver);
}
module_init(k250a_init);

static void __exit k250a_exit(void)
{
	INFO("Entry : %s\n", __func__);
	i2c_del_driver(&k250a_driver);
}

module_exit(k250a_exit);

MODULE_AUTHOR("Sec");
MODULE_DESCRIPTION("K250A driver");
MODULE_LICENSE("GPL");
