// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Samsung Electronics Inc.
 */

#ifndef _CM_BB_MAX77816_H_
#define _CM_BB_MAX77816_H_

int cam_bb_max77816_control(bool enable);

struct cam_bb_max77816_info {
	struct i2c_client *client;
};

enum {
	CAM_BB_ADDR = 0,
	CAM_BB_VAL,
	CAM_BB_MAX,
};

u8 max77816_en[][CAM_BB_MAX] = {
	/* addr value */
	{0x02, 0x4F},
	{0x03, 0x72},
	{0x04, 0x37},
};

#endif /* _CM_BB_MAX77816_H_ */
