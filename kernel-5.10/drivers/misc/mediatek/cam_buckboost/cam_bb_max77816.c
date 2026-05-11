// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Samsung Electronics Inc.
 */

#include <linux/kernel.h>
#include <linux/of_gpio.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/pinctrl/consumer.h>

#include "cam_bb_max77816.h"

#define PFX "[CAM_BB D/D]"
#define LOG_DBG(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)
#define LOG_INF(format, args...) pr_info(PFX "[%s] " format, __func__, ##args)
#define LOG_ERR(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)

static struct cam_bb_max77816_info buckboost_pinfo;
static struct pinctrl *gpio_ctrl;
static struct pinctrl_state *bb_en_state_on;
static struct pinctrl_state *bb_en_state_off;

static int buckboost_power_on_cnt;

static int cam_bb_max77816_i2c_read(struct i2c_client *client, u8 addr,  u8 *value)
{
	int ret = -1;
	int retry = 3;
	u8 wr_data[2] = {0, };
	struct i2c_msg msg[2];

	/* Check if camera buckboost probed or not, to prevent KP */
	if (!client) {
		LOG_ERR("cam_bb_max66718 not probed.\n");
	} else {
		wr_data[0] = addr;

		msg[0].addr = client->addr;
		msg[0].flags = 0;
		msg[0].buf = wr_data;
		msg[0].len = 1;

		msg[1].addr = client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].buf = value;
		msg[1].len = 1;

		do {
			ret = i2c_transfer(client->adapter, msg, 2);
			if (ret != 2)
				LOG_ERR("client->addr 0x%02x read_addr 0x%02x error (ret == %d)\n", client->addr, addr, ret);
			else
				break;
		} while (--retry);
	}
	LOG_DBG("addr:%#x, data:%#x\n", addr, *value);
	return ret;
}

static int cam_bb_max77816_i2c_write(struct i2c_client *client, u8 addr,  u8 value)
{
	int ret = -1;
	int retry = 3;
	u8 wr_data[2] = {0, };
	struct i2c_msg msg;

	/* Check if camera buckboost probed or not, to prevent KP */
	if (!client) {
		LOG_ERR("cam_bb_max66718 not probed.\n");
	} else {
		wr_data[0] = addr;
		wr_data[1] = value;

		msg.addr = client->addr;
		msg.flags = 0;
		msg.buf = wr_data;
		msg.len = 2;

		do {
			ret = i2c_transfer(client->adapter, &msg, 1);
			if (ret != 1) {
				LOG_ERR("addr 0x%02x value 0x%02x error (ret == %d)\n", addr, value, ret);
			} else
				break;
		} while (--retry);
	}
	LOG_DBG("addr:%#x, data:%#x\n", addr, value);
	return ret;
}

int cam_bb_max77816_control(bool enable)
{
	u8 check;
	u8 (*data)[CAM_BB_MAX];
	int size;
	int ret = 0;
	int i;

	if (gpio_ctrl == NULL || bb_en_state_on == NULL || bb_en_state_off == NULL) {
		LOG_ERR("GPIO is not prepared\n");
		return -EIO;
	}

	LOG_DBG("-E\n");

	if (enable) {
		if (pinctrl_select_state(gpio_ctrl, bb_en_state_on) < 0) {
			LOG_ERR("bb en on: failed\n");
			return -EIO;
		}
		LOG_DBG("enable GPIO: High\n");
		usleep_range(1000, 1100);

		size = ARRAY_SIZE(max77816_en);
		data = max77816_en;

		if (!buckboost_pinfo.client) {
			LOG_ERR("i2c is not prepared\n");
			return -ENODEV;
		}

		for (i = 0; i < size; i++) {
			cam_bb_max77816_i2c_write(buckboost_pinfo.client, data[i][CAM_BB_ADDR], data[i][CAM_BB_VAL]);
			cam_bb_max77816_i2c_read(buckboost_pinfo.client, data[i][CAM_BB_ADDR], &check);

			if (check != data[i][CAM_BB_VAL]) {
				LOG_ERR("failed: add: 0x%02x, write=0x%02x, read=0x%02x\n",
						data[i][CAM_BB_ADDR], data[i][CAM_BB_VAL], check);
				ret = -EINVAL;
			}
		}
		usleep_range(1000, 1100);
		buckboost_power_on_cnt++;
		LOG_INF("Camera buck boost: ON\n");
	} else {
		buckboost_power_on_cnt--;
		if (buckboost_power_on_cnt <= 0)
			buckboost_power_on_cnt = 0;

		if (buckboost_power_on_cnt) {
			LOG_INF("Camera buck boost: skip OFF, using other side[count:%d]\n", buckboost_power_on_cnt);
			goto out;
		}

		if (pinctrl_select_state(gpio_ctrl, bb_en_state_off) < 0) {
			LOG_ERR("bb en off: failed\n");
			return -EIO;
		}
		LOG_DBG("enable GPIO: Low\n");
		LOG_INF("Camera buck boost: OFF\n");
	}

out:
	LOG_DBG("-X\n");
	return ret;
}
EXPORT_SYMBOL_GPL(cam_bb_max77816_control);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
static int cam_bb_max77816_probe(struct i2c_client *client)
#else
static int cam_bb_max77816_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	LOG_INF("-E\n");

	gpio_ctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(gpio_ctrl)) {
		LOG_ERR("Failed to get pinctrl\n");
		gpio_ctrl = NULL;
		return PTR_ERR(gpio_ctrl);
	}

	bb_en_state_on = pinctrl_lookup_state(gpio_ctrl, "bb_en_on");
	if (IS_ERR(bb_en_state_on)) {
		LOG_ERR("Failed to look up 'bb_en_on'\n");
		bb_en_state_on = NULL;
		return PTR_ERR(bb_en_state_on);
	}

	bb_en_state_off = pinctrl_lookup_state(gpio_ctrl, "bb_en_off");
	if (IS_ERR(bb_en_state_off)) {
		LOG_ERR("Failed to look up 'bb_en_off'\n");
		bb_en_state_off = NULL;
		return PTR_ERR(bb_en_state_off);
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		LOG_ERR("i2c check fail\n");
		return -EIO;
	}

	buckboost_pinfo.client = client;
	i2c_set_clientdata(client, &buckboost_pinfo);

	LOG_INF("-X\n");
	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static void cam_bb_max77816_remove(struct i2c_client *client)
{
	buckboost_power_on_cnt = 0;
	return;
}
#else
static int cam_bb_max77816_remove(struct i2c_client *client)
{
	buckboost_power_on_cnt = 0;
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id cam_bb_max77816_match_table[] = {
	{.compatible = "camera_buckboost,max77816",},
	{}
};
#endif

struct i2c_driver cam_bb_max77816_driver = {
	.probe = cam_bb_max77816_probe,
	.remove = cam_bb_max77816_remove,
	.driver = {
		.name = "cam_bb_max77816",
		.owner = THIS_MODULE,
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = of_match_ptr(cam_bb_max77816_match_table),
#endif
	},
};

static int __init cam_bb_max77816_init(void)
{
	int ret = 0;

	LOG_INF("-E\n");
	ret = i2c_add_driver(&cam_bb_max77816_driver);
	if (ret)
		LOG_ERR("cam_bb_max77816 registration failed, ret = %d\n", ret);
	else
		LOG_INF("cam_bb_max77816 registered\n");

	buckboost_power_on_cnt = 0;
	LOG_INF("-X\n");
	return ret;
}

static void __exit cam_bb_max77816_exit(void)
{
	i2c_del_driver(&cam_bb_max77816_driver);
}

module_init(cam_bb_max77816_init);
module_exit(cam_bb_max77816_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Camera BuckBoost MAX77816 Driver");
