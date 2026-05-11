/*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*/
#ifndef SX9385_H
#define SX9385_H

#include "sx9385_notifier.h"

#define VENDOR_NAME              "SEMTECH"
#define NOTI_MODULE_NAME         "grip_notifier"

enum ic_num {
	MAIN_GRIP = 0,
	SUB_GRIP,
	SUB2_GRIP,
	WIFI_GRIP,
	GRIP_MAX_CNT
};

/* IrqStat 0:Inactive 1:Active */
#define IRQSTAT_RESET_FLAG		0x80
#define IRQSTAT_TOUCH_FLAG		0x20
#define IRQSTAT_RELEASE_FLAG	0x10
#define IRQSTAT_COMPDONE_FLAG	0x08
#define IRQSTAT_CONV_FLAG		0x04

#define SX9385_STAT_COMPSTAT_ALL_FLAG (0x02 | 0x04)
#define SX9385_PHEN_REF     0x28

//Registers refered by the driver
#define REG_IRQ_SRC         0x0
#define REG_PROX_STATUS     0x1
#define REG_COMPENSATION    0x2
#define REG_IRQ_MSK         0x4
#define REG_PHEN            0x9
#define REG_GNRL_CTRL2      0xB
#define REG_RESET           0x38
#define REG_DEV_INFO        0x3E
#define REG_REV_INFO        0x3F
#define REG_PH0_OFFSET      0x4B
#define REG_PH0_USEFUL      0xBA
#define REG_DLT_VAR_PH2     0xC2
#define REG_PH0_AVERAGE     0xC7
#define REG_PH0_DIFF        0xCB

#define REG_AFE_PARAM2_PH1  0x49
#define REG_AFE_PARAM2_PH2  0x4F
#define REG_AFE_PARAM2_PH3  0x56
#define REG_AFE_PARAM2_PH4  0x5C

#define REG_AFE_PARAM3_PH1  0x49

#define REG_PROX_CTRL4_PH2  0x7D
#define REG_PROX_CTRL5_PH2  0x7E

#define REG_PROX_CTRL4_PH4  0x83
#define REG_PROX_CTRL5_PH4  0x84
#define REG_PROX_CTRL3_PH2	0x7C
#define REG_PROX_CTRL3_PH4	0x82

#define REG_PROX_CTRL1_PH2	0x7A
#define REG_PROX_CTRL0_PH12	0x79
#define REG_PROX_CTRL0_PH34	0x7F

#define REG_PROX_CTRL1_PH4	0x80
#define REG_AFE_PARAM0_PH2	0x4D
#define REG_AFE_PARAM0_PH4	0x5A

#define REG_AFE_PARAM0_PH2_MSB	0x4D
#define REG_AFE_PARAM1_PH2_LSB	0x4E
#define REG_AFE_PARAM0_PH4_MSB	0x5A
#define REG_AFE_PARAM1_PH4_LSB	0x5B

#define REG_USEFILTER0_PH2		0x92
#define REG_USEFILTER0_PH4		0x9E


#define REG_PROX_CTRL4_PH2		0x7D
#define REG_PROX_CTRL4_PH4		0x83

/* RegStat0  */
#define SX9385_PROXSTAT_CH1_FLAG			(1 << 4)
#define SX9385_PROXSTAT_CH2_FLAG			(1 << 5)

/*      SoftReset */
#define SX9385_SOFTRESET				0xDE
#define SX9385_WHOAMI_VALUE				0x13
#define SX9385_REV_VALUE				0x13

#define MAX_NUM_STATUS_BITS (8)

#define SENSOR_ATTR_SIZE 65
#define NUM_MAIN_PHASES 2 //PH2 and PH4 are the main phases of sx9385

#define PANEL_ON_REGISTER_VALUE 0x09
#define PANEL_OFF_REGISTER_VALUE 0x01

#define SX9385_MODE_SLEEP 0
#define SX9385_MODE_NORMAL 1

#define MAX_I2C_FAIL_COUNT 3
#define NUM_OF_ALL_PHASES 4

enum {
	OFF = 0,
	ON = 1
};

enum convert_func {
	SSCANF_TWO_HEX_VALUE_FUNCTION = 0,
	SSCANF_HEX_VALUE_FUNCTION,
	KSTRTO8_FUNCTION,
	KSTTOUL_FUNCTION,
};

enum grip_error_state {
	FAIL_UPDATE_PREV_STATE = 1,
	FAIL_SETUP_REGISTER,
	FAIL_I2C_ENABLE,
	FAIL_I2C_READ_3_TIMES,
	FAIL_DATA_STUCK,
	FAIL_RESET,
	FAIL_MCC_RESET,
	FAIL_IRQ_MISS_MATCH,
	FAIL_ENABLE_SCAN_ERROR
};

struct channel {
	unsigned int grip_code;
	unsigned int unknown_code;
	unsigned int noti_code;

	int is_unknown_mode;
	int noti_enable;
	int diff_cnt;
	int again_m;
	int dgain_m;

	s32 useful_avg;
	s32 capMain;
	s32 useful;
	s32 sum_diff;
	s32 sum_useful;
	u16 detect_threshold;
	u16 release_threshold;
	u16 offset;
	s16 avg;
	s16 diff;
	s16 diff_avg;
	s16 max_diff;
	s16 max_normal_diff;

	u8 num;
	u8 cs_mapping;
	u8 prox_mask;
	u8 state_miss_matching_count;
	s8 state;
	s8 prev_state;

	bool first_working;
	bool unknown_sel;
	bool enabled;
};

struct grip_notifier_block {
#if IS_ENABLED(CONFIG_PDIC_NOTIFIER)
	struct notifier_block pdic_nb;
#endif
#if IS_ENABLED(CONFIG_HALL_NOTIFIER)
	struct notifier_block hall_nb;
#endif
#if IS_ENABLED(CONFIG_SENSORS_SUPPROT_PANEL_NOTIFIER) && IS_ENABLED(CONFIG_SEC_PANEL_NOTIFIER_V2)
	struct notifier_block panel_nb;
#endif
#if IS_ENABLED(CONFIG_TABLET_MODEL_CONCEPT)
#if IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V3) || IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V2) || IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO)
	struct notifier_block pogo_nb;
#endif
#endif
};

struct sx9385_p {
	struct i2c_client *client;
	struct input_dev *input;
	struct device *factory_device;
	struct delayed_work init_work;
	struct delayed_work irq_work;
	struct delayed_work debug_work;
	struct work_struct cal_work;
	struct wakeup_source *grip_ws;
	struct mutex mode_mutex;
	struct mutex read_mutex;
	struct mutex cal_mutex;
	struct mutex notifier_mutex;
	struct device_attribute *sensor_attributes[SENSOR_ATTR_SIZE];
	struct grip_notifier_block noti_block;

	struct regulator *dvdd_vreg;	/* regulator */
	const char *dvdd_vreg_name;	/* regulator name */
	struct channel *ch[NUM_MAIN_PHASES];
	struct reg_addr_val_s *regs_addr_val;

	atomic_t enable;

	int gpio_nirq;
	int pre_attach;
	int debug_count;
	int debug_zero_count;
	int irq_count;
	int abnormal_mode;
	int ldo_en;
	int irq;
	int motion;
	int num_regs;
	int num_of_channels;
	int set_up_channels;
	int num_of_refs;
	int unknown;
/* REF channel */
	int again_r[2];
	int dgain_r[2];

	s32 refMain[2];
	u32 err_state;

	u8 prev_status;
	u8 ic_num;
	u8 i2c_fail_count;
	u8 initial_set_phases;

	bool is_irq_active;
	bool skip_data;
	bool check_abnormal_working;
	bool is_suspend;
};

struct input_dev *grip_noti_input_dev;
struct mutex noti_lock;
int noti_input_count;
int sx9385_module_count;

#define GRIP_ERR(fmt, ...) pr_err("[GRIP_%s] %s "fmt, grip_name[data->ic_num], __func__, ##__VA_ARGS__)
#define GRIP_INFO(fmt, ...) pr_info("[GRIP_%s] %s "fmt, grip_name[data->ic_num], __func__, ##__VA_ARGS__)
#define GRIP_WARN(fmt, ...) pr_warn("[GRIP_%s] %s "fmt, grip_name[data->ic_num], __func__, ##__VA_ARGS__)

#if !IS_ENABLED(CONFIG_SENSORS_CORE_AP)
extern int sensors_create_symlink(struct input_dev *inputdev);
extern void sensors_remove_symlink(struct input_dev *inputdev);
extern int sensors_register(struct device **dev, void *drvdata,
			    struct device_attribute *attributes[], char *name);
extern void sensors_unregister(struct device *dev,
			       struct device_attribute *attributes[]);
#endif

#if IS_ENABLED(CONFIG_SENSORS_GRIP_FAILURE_DEBUG)
extern void update_grip_error(u8 idx, u32 error_state);
#endif

#endif
