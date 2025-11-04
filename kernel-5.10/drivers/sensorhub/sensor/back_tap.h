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

#ifndef __SHUB_BACK_TAP_H__
#define __SHUB_BACK_TAP_H__

#include <linux/device.h>

struct back_tap_event {
	u8 tap_result;
	u64 first_tap;
	u64 second_tap;
	u64 third_tap;
} __attribute__((__packed__));

struct back_tap_data {
	uint8_t backtap_thd;
  //uint8_t backtap_type;
};

#endif /* __SHUB_BACK_TAP_H__ */
