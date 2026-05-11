/*
 *  Copyright (C) 2024, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include "../../utility/shub_utility.h"
#include "../../comm/shub_comm.h"
#include "../../sensor/back_tap.h"
#include "../../sensorhub/shub_device.h"
#include "../../sensormanager/shub_sensor.h"
#include "../../sensormanager/shub_sensor_manager.h"
#include "../../utility/shub_dev_core.h"
#include "../shub_factory.h"
#include "back_tap_factory.h"

#include <linux/slab.h>
#include <linux/of.h>

static struct device *backtap_sysfs_device;
static uint8_t backtap_type = 3;

static ssize_t backtap_peak_thd_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct back_tap_data *data = get_sensor(SENSOR_TYPE_BACK_TAP)->data;
	shub_infof("Curr Back Tap THD %d", (int)data->backtap_thd);

	return snprintf(buf, PAGE_SIZE,	"%d\n", (int)data->backtap_thd);
}

static ssize_t backtap_peak_thd_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct back_tap_data *data = get_sensor(SENSOR_TYPE_BACK_TAP)->data;
	int msg_buf;
	int ret = 0;

	ret = kstrtoint(buf, 10, &msg_buf);
	if (ret < 0) {
		shub_errf("kstrtoint fail");
		return ret;
	}

	shub_infof("msg_buf = %d\n", msg_buf);

	if (msg_buf < 0 || msg_buf > 2) {
		shub_errf("Invalid value:%d\n", msg_buf);
		return -EINVAL;
	}

	data->backtap_thd = (uint8_t)msg_buf;

	ret = shub_send_command(CMD_SETVALUE, SENSOR_TYPE_BACK_TAP, CAL_DATA,
				(char *)&data->backtap_thd, sizeof(data->backtap_thd));
	if (ret < 0)
		shub_errf("send backtap_thd failed");

	shub_infof("Back Tap New THD %d", (int)data->backtap_thd);

	return size;
}

static ssize_t backtap_type_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	shub_infof("Curr Type %d", (int)backtap_type);

	return snprintf(buf, PAGE_SIZE,	"%d\n", (int)backtap_type);
}

static ssize_t backtap_type_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int msg_buf;
	int ret = 0;

	ret = kstrtoint(buf, 10, &msg_buf);
	if (ret < 0) {
		shub_errf("kstrtoint fail");
		return ret;
	}

	shub_infof("msg_buf = %d\n", msg_buf);

	if (msg_buf <= 0 || msg_buf >= 4) {
		shub_errf("send backtap_type failed");
		return -EINVAL;
	}

	backtap_type = (uint8_t)msg_buf;

	ret = shub_send_command(CMD_SETVALUE, SENSOR_TYPE_BACK_TAP, SENSOR_FACTORY,
				(char *)&backtap_type, sizeof(backtap_type));
	if (ret < 0)
		shub_errf("send backtap_thd failed");
	else
		shub_infof("Back_Tap_Type %d", (int)backtap_type);

	return size;
}

static DEVICE_ATTR(backtap_peak_thd, 0664, backtap_peak_thd_show, backtap_peak_thd_store);
static DEVICE_ATTR(backtap_type, 0664, backtap_type_show, backtap_type_store);

static struct device_attribute *backtap_attrs[] = {
	&dev_attr_backtap_peak_thd,
	&dev_attr_backtap_type,
	NULL,
};

static void initialize_backtap_factorytest(void)
{
	sensor_device_create(&backtap_sysfs_device, NULL, "back_tap_sensor");
	add_sensor_device_attr(backtap_sysfs_device, backtap_attrs);
}

static void remove_backtap_factorytest(void)
{
	remove_sensor_device_attr(backtap_sysfs_device, backtap_attrs);
	sensor_device_destroy(backtap_sysfs_device);
}

void initialize_back_tap_factory(bool en, int mode)
{
	if (en)
		initialize_backtap_factorytest();
	else {
		if (mode == INIT_FACTORY_MODE_REMOVE_EMPTY && get_sensor(SENSOR_TYPE_BACK_TAP))
			shub_infof("support backtap sysfs");
		else
			remove_backtap_factorytest();
	}
}
