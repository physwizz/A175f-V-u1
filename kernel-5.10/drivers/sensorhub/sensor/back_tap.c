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

#include <linux/of_gpio.h>
#include <linux/slab.h>

#include "../comm/shub_comm.h"
#include "../sensorhub/shub_device.h"
#include "../sensormanager/shub_sensor.h"
#include "../utility/shub_utility.h"
#include "../sensormanager/shub_sensor.h"
#include "../sensormanager/shub_sensor_manager.h"
#include "back_tap.h"

/* 1: Double Tap
 * 2: Triple Tap
 * 3: Double and Triple Tap
 */
//static int backtap_type = 3;
static struct back_tap_data back_tap_data;

void print_back_tap_debug(int type)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_BACK_TAP);
	struct sensor_event *event = &(sensor->last_event_buffer);
	struct back_tap_event *sensor_value = (struct back_tap_event *)(event->value);

	shub_info("%s(%u) : %d, %lld(ms), %lld(ms), %lld(ms) / %lld", sensor->name,
		SENSOR_TYPE_BACK_TAP, sensor_value->tap_result, sensor_value->first_tap, sensor_value->second_tap,
		sensor_value->third_tap, event->timestamp);
}

void print_backtap_type_debug(int type)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_BACK_TAP);
	struct sensor_event *event = &(sensor->last_event_buffer);
	struct back_tap_event *sensor_value = (struct back_tap_event *)(event->value);

	shub_info("%s(%u) : %d, %lld(ms), %lld(ms), %lld(ms) / %lld", sensor->name,
		SENSOR_TYPE_BACK_TAP, sensor_value->tap_result, sensor_value->first_tap, sensor_value->second_tap,
		sensor_value->third_tap, event->timestamp);
}

static struct sensor_funcs back_tap_sensor_func = {
	.print_debug = print_back_tap_debug,
	.print_debug  = print_backtap_type_debug,
};

int init_back_tap(bool en)
{
	int ret = 0;
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_BACK_TAP);

	if (!sensor)
		return 0;

	if (en) {

		ret = init_default_func(sensor, "back_tap", 25, 25, sizeof(struct back_tap_event));
		sensor->funcs = &back_tap_sensor_func;
		sensor->data = (void *)&back_tap_data;
	} else {
		destroy_default_func(sensor);
	}

	return ret;
}

