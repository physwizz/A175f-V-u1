/*
 * Driver for the SX9385
 * Copyright (c) 2011 Semtech Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/pm_wakeup.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/power_supply.h>
#include <linux/version.h>
#include "sx9385.h"
#include "sx9385_kunit.h"

#if IS_ENABLED(CONFIG_SENSORS_CORE_AP)
#include <linux/sensor/sensors_core.h>
#endif

#if IS_ENABLED(CONFIG_TABLET_MODEL_CONCEPT)
#if IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V3)
#include "../input/sec_input/stm32/pogo_notifier_v3.h"
#elif IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V2) || IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO)
#include <linux/input/pogo_i2c_notifier.h>
#endif
#endif

#define I2C_M_WR				0 /* for i2c Write */
//#define I2C_M_RD				1 /* for i2c Read */

#define HAS_ERROR			   -1
#define IDLE					2
#define ACTIVE				    1

#define DIFF_READ_NUM			10
#define GRIP_LOG_TIME			15 /* 30 sec */

#define REF_OFF_MAIN_OFF		0x00

#define IRQ_PROCESS_CONDITION   (IRQSTAT_TOUCH_FLAG	\
				| IRQSTAT_RELEASE_FLAG	\
				| IRQSTAT_COMPDONE_FLAG)

#define GRIP_HAS_ERR   -1
#define GRIP_WORKING	1
#define GRIP_RELEASE	2

#define INTERRUPT_HIGH	1
#define INTERRUPT_LOW	0

#define UNKNOWN_ON		1
#define UNKNOWN_OFF 	2

#define DIFF_OVERFLOW -32768

#define PH1 0x08
#define PH2 0x10
#define PH3 0x20
#define PH4 0x40

#define NUM_OF_MAX_CHANNEL 2

#define CH_BOTH 0x3
#define CH2_ONLY 0x2
#define CH1_ONLY 0x1

#define SUCCESS 0

#define HYST_6_PERCENT 0x01
#define HYST_12_PERCENT 0x02
#define HYST_25_PERCENT 0x03
#define HYST_MASK 0x30
#define HYST_SHIFT 4

#define DEFAULT_THRESHOLD_FAIL_VALUE 10000
#define DEFAULT_HYST_FAIL_VALUE 10000

#define INT16_MAX 32767
#define INT16_MIN -32768

#define AGAIN_DGAIN_SCALE_FACTOR 15
#define PROXOFFSET_SCALE_FACTOR 38000
#define PROXUSEFUL_SCALE_FACTOR 125

#define MAX_RETRY_CNT 500

#define CS0_REG	0x01
#define CS1_REG	0x04
#define CS2_REG	0x10
#define CS3_REG	0x40

#define CS_UNKNOWN	0xFF
#define CS0	0x00
#define CS1	0x01
#define CS2	0x02
#define CS3	0x03


enum UNKNOWN_MODE_TYPE {
	TYPE_USB = 1,
	TYPE_HALL,
	TYPE_BOOT,
	TYPE_FORCE,
	TYPE_OVERFLOW,
	TYPE_PANEL,
	TYPE_POGO
};

enum entry_path {
	BY_INTERRUPT_HANDLER = 0,
	BY_ENABLE_FUNC,
	BY_DEBUG_WORK
};

struct reg_addr_val_s {
	u8 addr;
	u8 val;
};

static const char *grip_name[GRIP_MAX_CNT] = {
	"MAIN",
	"SUB",
	"SUB2",
	"WIFI"
};

static const char *device_name[GRIP_MAX_CNT] = {
	"SX9385",
	"SX9385_SUB",
	"SX9385_SUB2",
	"SX9385_WIFI"
};

static const char *module_name[GRIP_MAX_CNT] = {
	"grip_sensor",
	"grip_sensor_sub",
	"grip_sensor_sub2",
	"grip_sensor_wifi"
};

static uint32_t vdd_retry_cnt;
#if IS_ENABLED(CONFIG_SENSORS_REGION_DEPENDENCY)
static bool is_grip_mounted;
#endif

__visible_for_testing int sx9385_set_mode(struct sx9385_p *data, unsigned char mode);
static int sx9385_disable_all_phases(struct sx9385_p *data);
static int sx9385_enable_all_phases(struct sx9385_p *data);

#if IS_ENABLED(CONFIG_PDIC_NOTIFIER) && IS_ENABLED(CONFIG_USB_TYPEC_MANAGER_NOTIFIER)
static void sx9385_handle_usb_typec_event(struct sx9385_p *, void *);
#endif
#if IS_ENABLED(CONFIG_HALL_NOTIFIER)
static void sx9385_handle_hall_event(struct sx9385_p *, void *);
#endif
#if IS_ENABLED(CONFIG_SENSORS_SUPPROT_PANEL_NOTIFIER) && IS_ENABLED(CONFIG_SEC_PANEL_NOTIFIER_V2)
static int sx9385_get_register_value_for_panel_state(struct sx9385_p *, struct panel_notifier_event_data *);
static int sx9385_write_register_value_for_panel_state(struct sx9385_p *, u32, struct panel_notifier_event_data *);
static void sx9385_handle_panel_event(struct sx9385_p *, void *);
#endif

typedef int (*str_convert_func)(const char *, void *, void *);

static int sx9385_sscanf_two_hex(const char *buf, unsigned int *reg_address, unsigned int *val)
{
	if (sscanf(buf, "%x,%x", reg_address, val) != 2)
		return -EINVAL;
	return 0;
}

static int sx9385_sscanf_hex(const char *buf, unsigned int *reg_address)
{
	if (sscanf(buf, "%x", reg_address) != 1)
		return -EINVAL;
	return 0;
}

static int sx9385_kstrtou8_func(const char *buf, unsigned char *val)
{
	int ret = kstrtou8(buf, 0, val);

	if (ret)
		return ret;
	return 0;
}

static int sx9385_kstrtoul_func(const char *buf, unsigned long *val)
{
	int ret = kstrtoul(buf, 0, val);

	if (ret)
		return ret;
	return 0;
}

static int sx9385_sscanf_two_hex_wrapper(const char *buf, void *reg_address, void *val)
{
	return sx9385_sscanf_two_hex(buf, (unsigned int *)reg_address, (unsigned int *)val);
}

static int sx9385_sscanf_hex_wrapper(const char *buf, void *reg_address, void *unused)
{
	return sx9385_sscanf_hex(buf, (unsigned int *)reg_address);
}

static int sx9385_kstrtou8_func_wrapper(const char *buf, void *val, void *unused)
{
	return sx9385_kstrtou8_func(buf, (unsigned char *)val);
}

static int sx9385_kstrtoul_func_wrapper(const char *buf, void *val, void *unused)
{
	return sx9385_kstrtoul_func(buf, (unsigned long *)val);
}

str_convert_func func1 = (str_convert_func)sx9385_sscanf_two_hex_wrapper;
str_convert_func func2 = (str_convert_func)sx9385_sscanf_hex_wrapper;
str_convert_func func3 = (str_convert_func)sx9385_kstrtou8_func_wrapper;
str_convert_func func4 = (str_convert_func)sx9385_kstrtoul_func_wrapper;

__visible_for_testing int sx9385_convert_input(int func_id, const char *buf, void *val1, void *val2)
{
	str_convert_func selected_func = NULL;

	switch (func_id) {
	case SSCANF_TWO_HEX_VALUE_FUNCTION:
		selected_func = func1;
		break;
	case SSCANF_HEX_VALUE_FUNCTION:
		selected_func = func2;
		break;
	case KSTRTO8_FUNCTION:
		selected_func = func3;
		break;
	case KSTTOUL_FUNCTION:
		selected_func = func4;
		break;
	default:
		return -EINVAL;
	}

	return selected_func(buf, val1, val2);
}

static int sx9385_get_nirq_state(struct sx9385_p *data)
{
	return gpio_get_value_cansleep(data->gpio_nirq);
}

static bool sx9385_is_unavailable_channel(struct channel *ch)
{
	return (ch == NULL);
}

static void sx9385_send_event(struct sx9385_p *data, u8 state, struct channel *ch)
{
	if (state == ACTIVE) {
		ch->state = ACTIVE;
		data->prev_status |= ch->prox_mask;
		GRIP_INFO("ch[%d] touched\n", ch->num);
	} else if (state == IDLE) {
		ch->state = IDLE;
		data->prev_status &= ~(ch->prox_mask);
		GRIP_INFO("ch[%d] released\n", ch->num);
	} else {
		ch->state = HAS_ERROR;
		data->prev_status &= ~(ch->prox_mask);
		GRIP_INFO("ch[%d] fail_safe!\n", ch->num);
	}

	if (data->skip_data == true) {
		GRIP_INFO("skip grip event\n");
		input_report_rel(data->input, ch->grip_code, GRIP_RELEASE);
		if (ch->unknown_sel)
			input_report_rel(data->input, ch->unknown_code, UNKNOWN_OFF);
		input_sync(data->input);
		return;
	}

	input_report_rel(data->input, ch->grip_code, ch->state);

	if (ch->unknown_sel)
		input_report_rel(data->input, ch->unknown_code, ch->is_unknown_mode);
	input_sync(data->input);
}

__visible_for_testing
__mockable
void enter_unknown_mode(struct sx9385_p *data, int type, struct channel *ch)
{
	if (ch->noti_enable && !data->skip_data && ch->unknown_sel) {
		data->motion = 0;
		ch->first_working = false;
		if (ch->is_unknown_mode == UNKNOWN_OFF) {
			ch->is_unknown_mode = UNKNOWN_ON;
			if (atomic_read(&data->enable) == ON) {
				input_report_rel(data->input, ch->unknown_code, UNKNOWN_ON);
				input_sync(data->input);
			}
			GRIP_INFO("UNKNOWN Re-enter\n");
		} else {
			GRIP_INFO("already UNKNOWN\n");
		}
		input_report_rel(grip_noti_input_dev, ch->noti_code, type);
		input_sync(grip_noti_input_dev);
	}
}

__visible_for_testing
__mockable
void enter_error_mode(struct sx9385_p *data, enum grip_error_state err_state)
{
	int i;

	if (data->is_irq_active) {
		disable_irq(data->irq);
		disable_irq_wake(data->irq);
		data->is_irq_active = false;
	}

	data->check_abnormal_working = true;
	data->err_state |= 0x1 << err_state;

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (sx9385_is_unavailable_channel(data->ch[i]))
			continue;
		if (atomic_read(&data->enable) == ON)
			sx9385_send_event(data, HAS_ERROR, data->ch[i]);
		enter_unknown_mode(data, TYPE_FORCE + err_state, data->ch[i]);
	}

#if IS_ENABLED(CONFIG_SENSORS_GRIP_FAILURE_DEBUG)
	update_grip_error(data->ic_num, data->err_state);
#endif

	GRIP_ERR("%d\n", data->err_state);
}

static void check_irq_error(struct sx9385_p *data, u8 irq_state,
	enum entry_path path, struct channel *ch)
{
	if (data->is_irq_active && !data->check_abnormal_working) {
		if (path == BY_INTERRUPT_HANDLER) {
			ch->state_miss_matching_count = 0;
			ch->prev_state = irq_state;
		} else if (path == BY_ENABLE_FUNC) {
			ch->prev_state = irq_state;
		} else if (ch->prev_state != irq_state) {
			ch->state_miss_matching_count++;
			GRIP_INFO("ch[%d] prev %x state %x miss_cnt %d\n", ch->num,
				ch->prev_state, irq_state, ch->state_miss_matching_count);
			ch->prev_state = irq_state;
		}

		if (ch->state_miss_matching_count >= 3) {
			GRIP_INFO("enter_error_mode with IRQ\n");
			sx9385_set_mode(data, SX9385_MODE_SLEEP);
			enter_error_mode(data, FAIL_IRQ_MISS_MATCH);
		}
	}
}

static int sx9385_i2c_write(struct sx9385_p *data, u8 reg_addr, u8 buf)
{
	struct i2c_msg msg;
	int ret = -1;
	unsigned char w_buf[2];

	w_buf[0] = reg_addr;
	w_buf[1] = buf;

	msg.addr = data->client->addr;
	msg.flags = I2C_M_WR;
	msg.len = 2;
	msg.buf = (char *)w_buf;

	if (data->i2c_fail_count < MAX_I2C_FAIL_COUNT)
		ret = i2c_transfer(data->client->adapter, &msg, 1);

	if (ret < 0) {
		if (data->i2c_fail_count < MAX_I2C_FAIL_COUNT) {
			data->i2c_fail_count++;
			GRIP_ERR("err %d, %u\n", ret, data->i2c_fail_count);
		}
		if (data->i2c_fail_count >= MAX_I2C_FAIL_COUNT && !data->check_abnormal_working)
			enter_error_mode(data, FAIL_I2C_READ_3_TIMES);
	} else {
		data->i2c_fail_count = 0;
	}

	return ret;
}

__visible_for_testing
__mockable
int sx9385_i2c_write_retry(struct sx9385_p *data, u8 reg_addr, u8 buf)
{
	int i, ret;

	for (i = 0; i < 3; i++) {
		ret = sx9385_i2c_write(data, reg_addr, buf);
		if (ret >= 0)
			return ret;
	}
	return ret;
}

__visible_for_testing
__mockable
int sx9385_i2c_read(struct sx9385_p *data, u8 reg_addr, u8 *buf)
{
	struct i2c_msg msg[2];
	int ret = -1;

	msg[0].addr = data->client->addr;
	msg[0].flags = I2C_M_WR;
	msg[0].len = 1;
	msg[0].buf = &reg_addr;

	msg[1].addr = data->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = buf;

	if (data->i2c_fail_count < MAX_I2C_FAIL_COUNT) {
			ret = i2c_transfer(data->client->adapter, msg, 2);
			if (ret < 0) {
				GRIP_ERR("err %d, %u\n", ret, msg[0].addr);
		}
	}

	if (ret < 0) {
		if (data->i2c_fail_count < MAX_I2C_FAIL_COUNT) {
			data->i2c_fail_count++;
			GRIP_ERR("err %d, %u\n", ret, data->i2c_fail_count);
		}
		if (data->i2c_fail_count >= MAX_I2C_FAIL_COUNT && !data->check_abnormal_working)
			enter_error_mode(data, FAIL_I2C_READ_3_TIMES);
	} else {
		data->i2c_fail_count = 0;
	}

	return ret;
}

static int sx9385_i2c_read_retry(struct sx9385_p *data, u8 reg_addr, u8 *buf)
{
	int i, ret;

	for (i = 0; i < 3; i++) {
		ret = sx9385_i2c_read(data, reg_addr, buf);
		if (ret >= 0)
			return ret;
	}

	return ret;
}

static int sx9385_read_reg_stat(struct sx9385_p *data)
{
	if (data) {
		u8 value = 0;
		if (sx9385_i2c_read_retry(data, REG_IRQ_SRC, &value) >= 0)
			return (value & 0x00FF);
	}
	return 0;
}

__visible_for_testing DEFINE_REDIRECT_MOCKABLE(sx9385_calculate_threshold, RETURNS(u32), PARAMS(u8));
__visible_for_testing u32 REAL_ID(sx9385_calculate_threshold)(u8 th_buf)
{
	u32 threshold = ((u32)th_buf * (u32)th_buf) >> 1;
	return threshold;
}

__visible_for_testing DEFINE_REDIRECT_MOCKABLE(sx9385_calculate_hyst, RETURNS(u32), PARAMS(u8, u32));
__visible_for_testing u32 REAL_ID(sx9385_calculate_hyst)(u8 hyst_buf, u32 threshold)
{
	u32 hyst = 0;

	switch ((hyst_buf & HYST_MASK) >> HYST_SHIFT) {
	case HYST_6_PERCENT:
		hyst = threshold >> 4;
		break;
	case HYST_12_PERCENT:
		hyst = threshold >> 3;
		break;
	case HYST_25_PERCENT:
		hyst = threshold >> 2;
		break;
	default:
		hyst = 0;
		break;
	}

	return hyst;
}

static void sx9385_initialize_register(struct sx9385_p *data)
{
	int ret, i;
	u32 reg_val;
	u16 reg_addr;
	u8 th_reg[] = {REG_PROX_CTRL5_PH2, REG_PROX_CTRL5_PH4};
	u8 hyst_reg[] = {REG_PROX_CTRL4_PH2, REG_PROX_CTRL4_PH4};

	for (i = 0; i < data->num_regs; i++) {
		reg_addr = data->regs_addr_val[i].addr;
		reg_val  = data->regs_addr_val[i].val;

		if (reg_addr == REG_PHEN) {
			data->initial_set_phases = reg_val;
			reg_val = SX9385_PHEN_REF | reg_val;
		}

		ret = sx9385_i2c_write_retry(data, reg_addr, reg_val);
		if (ret < 0) {
			GRIP_ERR("Failed to write reg=0x%x value=0x%X\n", reg_addr, reg_val);
			enter_error_mode(data, FAIL_SETUP_REGISTER);
			return;
		}

		if (reg_addr != REG_COMPENSATION) {
			u8 buf = 0;

			sx9385_i2c_read_retry(data, reg_addr, &buf);
			if (reg_val != buf) {
				GRIP_ERR("addr[0x%x] : write =0x%x read=0x%X\n", reg_addr, reg_val, buf);
				sx9385_set_mode(data, SX9385_MODE_SLEEP);
				enter_error_mode(data, FAIL_SETUP_REGISTER);
			}
		}
	}

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		u8 val = 0;

		if (sx9385_is_unavailable_channel(data->ch[i]))
			continue;

		ret = sx9385_i2c_read_retry(data, th_reg[i], &val);
		if (ret < 0) {
			data->ch[i]->detect_threshold = 0;
			data->ch[i]->release_threshold = 0;
			GRIP_ERR("detect threshold update1 fail\n");
		} else {
			data->ch[i]->detect_threshold = sx9385_calculate_threshold(val);
			data->ch[i]->release_threshold = sx9385_calculate_threshold(val);
			val = 0;
			ret = sx9385_i2c_read(data, hyst_reg[i], &val);
			if (ret < 0) {
				GRIP_ERR("detect hyst_reg fail\n");
			} else {
				data->ch[i]->detect_threshold += sx9385_calculate_hyst(val, data->ch[i]->detect_threshold);
				data->ch[i]->release_threshold += sx9385_calculate_hyst(val, data->ch[i]->release_threshold);
			}
		}

		if (ret < 0)
			enter_error_mode(data, FAIL_SETUP_REGISTER);
		GRIP_INFO("detect threshold %u\n", data->ch[i]->detect_threshold);
	}
}

static int sx9385_hardware_check(struct sx9385_p *data)
{
	int ret;
	u8 whoami = 0, loop = 0;

	//Check th IRQ Status
	while (sx9385_get_nirq_state(data) == INTERRUPT_LOW) {
		sx9385_read_reg_stat(data);
		GRIP_INFO("irq state : %d\n", sx9385_get_nirq_state(data));
		if (++loop > 10)
			return -ENXIO;
		usleep_range(10000, 11000);
	}

	ret = sx9385_i2c_read_retry(data, REG_DEV_INFO, &whoami);

	GRIP_INFO("whoami 0x%x\n", whoami);

	if (whoami != SX9385_WHOAMI_VALUE)
		return -ENODEV;

	return ret;
}

__visible_for_testing
__mockable
int sx9385_set_phase_enable(struct sx9385_p *data, u8 ph)
{
	int ret = 0;
	u8 phases[] = {PH1, PH2, PH3, PH4};

	if (data->initial_set_phases & phases[ph]) {
		u8 temp = 0;

		ret = sx9385_i2c_read_retry(data, REG_GNRL_CTRL2, &temp);
		if (ret < 0) {
			GRIP_ERR("ph en err 1\n");
			return ret;
		}
		temp &= ~(1 << 7); //clear

		ret = sx9385_i2c_write_retry(data, REG_GNRL_CTRL2, temp);
		if (ret < 0) {
			GRIP_ERR("ph en err 2\n");
			return ret;
		}

		usleep_range(1000, 1100);
		temp = 0;
		ret = sx9385_i2c_read_retry(data, REG_PHEN, &temp);
		if (ret < 0) {
			GRIP_ERR("ph en err 3\n");
			return ret;
		}

		temp |= phases[ph];
		ret = sx9385_i2c_write_retry(data, REG_PHEN, temp);
		if (ret < 0)
			GRIP_ERR("ph en err 4\n");

		usleep_range(5000, 5100);
	}

	return ret;
}


static int wait_for_convstat_clear(struct sx9385_p *data)
{
	int retry = 0;
	u8 convstat = 0xFF;

	for (retry = 0; retry < 10; retry++) {
		if (sx9385_i2c_read(data, REG_PROX_STATUS, &convstat) < 0)
			return -EIO;

		convstat &= 0x04;
		if (convstat == 0)
			return 0;

		usleep_range(10000, 11000);
	}

	GRIP_INFO("convstat %u\n", convstat);
	return -ETIMEDOUT;
}

static void sx9385_print_registers(struct sx9385_p *data)
{
	int i;
	u8 buf;

	for (i = 0; i < data->num_regs; i++) {
		buf = 0;
		sx9385_i2c_read(data, data->regs_addr_val[i].addr, &buf);
		GRIP_INFO("reg=0x%x value=0x%X\n", data->regs_addr_val[i].addr, buf);
	}
}

__visible_for_testing
__mockable
int sx9385_set_phase_disable(struct sx9385_p *data, u8 ph)
{
	int ret = 0;
	u8 phases[] = {PH1, PH2, PH3, PH4};

	if (data->initial_set_phases & phases[ph]) {
		u8 temp = 0;

		ret = sx9385_i2c_read_retry(data, REG_PHEN, &temp);
		if (ret < 0) {
			GRIP_ERR("ph disalbe err\n");
			return ret;
		}
		temp &= ~phases[ph];
		if (temp == 0x00) {
			ret = wait_for_convstat_clear(data);
			if (ret < 0) {
				GRIP_ERR("ph convstat_clear err\n");
				return ret;
			}
		}
		ret = sx9385_i2c_write_retry(data, REG_PHEN, temp);
		if (ret < 0) {
			GRIP_ERR("ph disalbe err\n");
			return ret;
		}

		usleep_range(5000, 5100);
	}

	return ret;
}


static int sx9385_manual_offset_calibration(struct sx9385_p *data)
{
	int ret = 0, ph;

	GRIP_INFO("\n");
	if (data->check_abnormal_working)
		return -1;
#if IS_ENABLED(CONFIG_SEC_FACTORY)
	if (data->is_suspend) {
		GRIP_INFO("skip to calbrate in suspend\n");
		return -1;
	}
#endif
	mutex_lock(&data->cal_mutex);
	__pm_stay_awake(data->grip_ws);
	for (ph = 0; ph < NUM_OF_ALL_PHASES; ph++) {
		ret = sx9385_set_phase_disable(data, ph);
		if (ret < 0)
			break;
		ret = sx9385_set_phase_enable(data, ph);
		if (ret < 0)
			break;
	}
	__pm_relax(data->grip_ws);
	mutex_unlock(&data->cal_mutex);

	if (ret < 0)
		GRIP_ERR("Failed to calibrate. ret=%d\n", ret);

	return ret;
}

__visible_for_testing
__mockable
void sx9385_schedule_work(struct work_struct *work)
{
	schedule_work(work);
}

static void sx9385_cal_work_func(struct work_struct *work)
{
	struct sx9385_p *data = container_of((struct work_struct *)work,
						 struct sx9385_p, cal_work);

	sx9385_manual_offset_calibration(data);
}


static void sx9385_get_gain(struct sx9385_p *data)
{
	const int again_phm_default = 3875; //X10,000
	int i = 0;
	u8 gain_reg[] = {REG_AFE_PARAM2_PH2, REG_AFE_PARAM2_PH4};

	if (data->check_abnormal_working)
		return;

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		u8 msByte = 0;
		u8 again, dgain;

		if (sx9385_is_unavailable_channel(data->ch[i]))
			continue;

		if (sx9385_i2c_read(data, gain_reg[i], &msByte) < 0)
			GRIP_INFO("fail get gain val:%d\n", i);

		again = (msByte) & 0x1F;
		data->ch[i]->again_m = again_phm_default * again;

		dgain = (msByte >> 5) & 0x07;
		if (dgain)
			data->ch[i]->dgain_m = 1 << (dgain - 1);
		else
			data->ch[i]->dgain_m = 1;
	}
}

static void sx9385_get_ref_gain(struct sx9385_p *data)
{
	const int again_phm_default = 3875; //X10,000
	int i;
	u8 gain_reg[] = {REG_AFE_PARAM2_PH1, REG_AFE_PARAM2_PH3};

	if (data->check_abnormal_working)
		return;

	for (i = 0; i < data->num_of_refs; i++) {
		u8 msByte = 0;
		u8 again, dgain;

		if (sx9385_i2c_read(data, gain_reg[i], &msByte) < 0)
			GRIP_INFO("fail get gain val:%d\n", i);

		again = (msByte) & 0x1F;
		data->again_r[i] = again_phm_default * again;

		dgain = (msByte >> 5) & 0x07;
		if (dgain)
			data->dgain_r[i] = 1 << (dgain - 1);
		else
			data->dgain_r[i] = 1;
	}
}

__visible_for_testing int sx9385_calculate_coffset(u8 msb, u8 lsb)
{
	return ((msb & 0xFE) >> 1) * PROXOFFSET_SCALE_FACTOR + (((msb & 0x01) << 8) + lsb) * PROXUSEFUL_SCALE_FACTOR;
}

__visible_for_testing int sx9385_calculate_cuseful(int useful, int again, int dgain)
{
	if (dgain == 0)
		dgain = 1;

	return (useful * again) / (dgain << AGAIN_DGAIN_SCALE_FACTOR);
}

__visible_for_testing int sx9385_calculate_avg(u8 msb, u8 mid, u8 lsb)
{
	return (s16)(msb & 0x03) << 14 | mid << 6 | lsb >> 2;
}

__visible_for_testing int sx9385_calculate_diff(int use, int avg)
{
	int diff = use - avg;

	if (diff > INT16_MAX)
		return INT16_MAX;
	if (diff < INT16_MIN)
		return INT16_MIN;
	return diff;
}

static int sx9385_read_and_calculate_useful(struct sx9385_p *data, u8 reg_use_msb[], int i) {
	u8 msb = 0, lsb = 0;

	sx9385_i2c_read(data, reg_use_msb[i], &msb);
	sx9385_i2c_read(data, reg_use_msb[i] + 1, &lsb);

	return (s16)(msb << 8 | lsb);
}

static int sx9385_read_and_calculate_offset(struct sx9385_p *data, u8 reg_off_msb[], int i) {
	u8 msb = 0, lsb = 0;

	sx9385_i2c_read(data, reg_off_msb[i], &msb);
	sx9385_i2c_read(data, reg_off_msb[i] + 1, &lsb);

	return (msb << 8) | lsb;
}

static int sx9385_read_and_calculate_avg(struct sx9385_p *data, u8 reg_avg_msb[], int i) {
	u8 msb = 0, mid = 0, lsb = 0;

	sx9385_i2c_read(data, reg_avg_msb[i], &msb);
	sx9385_i2c_read(data, reg_avg_msb[i] + 1, &mid);
	sx9385_i2c_read(data, reg_avg_msb[i] + 2, &lsb);

	return sx9385_calculate_avg(msb, mid, lsb);
}

static void sx9385_process_useful_offset_avg(struct sx9385_p *data, int i)
{
	u8 reg_avg_msb[] = {0xC7, 0xDA};
	u8 reg_use_msb[] = {0xBD, 0xD0};
	u8 reg_off_msb[] = {0x51, 0x5E};

	data->ch[i]->useful = sx9385_read_and_calculate_useful(data, reg_use_msb, i);
	data->ch[i]->offset = sx9385_read_and_calculate_offset(data, reg_off_msb, i);
	data->ch[i]->avg = sx9385_read_and_calculate_avg(data, reg_avg_msb, i);
}

__visible_for_testing void sx9385_calculate_capMain_and_diff_data(struct sx9385_p *data, int i)
{
	struct channel *chData = data->ch[i];
	int coffset, cuseful;

	coffset = sx9385_calculate_coffset(chData->offset >> 8, chData->offset & 0xFF);
	cuseful = sx9385_calculate_cuseful(chData->useful, chData->again_m, chData->dgain_m);
	chData->capMain = coffset + cuseful;
	chData->diff = sx9385_calculate_diff(chData->useful, chData->avg);
}

static void sx9385_get_data(struct sx9385_p *data)
{
	struct channel *chData;
	int i;

	mutex_lock(&data->read_mutex);

	if (wait_for_convstat_clear(data) < 0) {
		mutex_unlock(&data->read_mutex);
		return;
	}

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		chData = data->ch[i];
		if (sx9385_is_unavailable_channel(chData))
			continue;

		sx9385_process_useful_offset_avg(data, i);
		sx9385_calculate_capMain_and_diff_data(data, i);

		GRIP_INFO("ch[%d]: diff= %d useful= %d average= %d capMain= %d offset= %d skip= %d\n", i,
			chData->diff, chData->useful, chData->avg, chData->capMain, chData->offset, data->skip_data);
	}

	mutex_unlock(&data->read_mutex);
}

static int sx9385_disable_all_phases(struct sx9385_p *data)
{
	u8 ph;
	int ret;

	for (ph = 0; ph < NUM_OF_ALL_PHASES; ph++) {
		ret = sx9385_set_phase_disable(data, ph);
		if (ret < 0) {
			GRIP_ERR("Failed to disable phase %d, ret = %d\n", ph, ret);
			return ret;
		}
	}

	return 0;
}

static int sx9385_enable_all_phases(struct sx9385_p *data)
{
	u8 ph;
	int ret;

	for (ph = 0; ph < NUM_OF_ALL_PHASES; ph++) {
		ret = sx9385_set_phase_enable(data, ph);
		if (ret < 0) {
			GRIP_ERR("Failed to enable phase %d, ret = %d\n", ph, ret);
			sx9385_disable_all_phases(data);
			return ret;
		}
	}

	return 0;
}

static bool sx9385_is_abnormal_condition(struct sx9385_p *data)
{
	return data->check_abnormal_working || data->i2c_fail_count >= MAX_I2C_FAIL_COUNT;
}

__visible_for_testing int sx9385_set_mode(struct sx9385_p *data, unsigned char mode)
{
	int ret = -EINVAL;

	if (sx9385_is_abnormal_condition(data)) {
		GRIP_INFO("abnormal working\n");
		return -1;
	}

	GRIP_INFO("%u\n", mode);

	mutex_lock(&data->mode_mutex);

	if (mode == SX9385_MODE_SLEEP)
		ret = sx9385_disable_all_phases(data);
	else if (mode == SX9385_MODE_NORMAL)
		ret = sx9385_enable_all_phases(data);

	mutex_unlock(&data->mode_mutex);

	if (ret < 0)
		enter_error_mode(data, FAIL_ENABLE_SCAN_ERROR);

	return ret;
}

static void sx9385_check_status(struct sx9385_p *data, struct channel *ch)
{
	int ret;
	u8 status = 0;

	if (data->check_abnormal_working)
		return;

	ret = sx9385_i2c_read_retry(data, REG_PROX_STATUS, &status);
	GRIP_INFO("ret %d 0x%x\n", ret, status);

	if ((ret < 0) || (ch->detect_threshold == 0)
			|| (data->check_abnormal_working == true))
		sx9385_send_event(data, HAS_ERROR, ch);
	else if (status & ch->prox_mask)
		sx9385_send_event(data, ACTIVE, ch);
	else
		sx9385_send_event(data, IDLE, ch);
}

static void sx9385_set_enable(struct sx9385_p *data, int enable)
{
	int pre_enable = atomic_read(&data->enable);

	GRIP_INFO("%d\n", enable);
	if (data->check_abnormal_working == true) {
		if (enable)
			atomic_set(&data->enable, ON);
		else
			atomic_set(&data->enable, OFF);

		if (enable) {
			GRIP_INFO("abnormal working\n");
			enter_error_mode(data, FAIL_UPDATE_PREV_STATE);
		}
		return;
	}

	if (enable) {
		if (pre_enable == OFF) {
			int i;
			u8 status[2] = {0, 0};

			sx9385_get_data(data);

			for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
				if (sx9385_is_unavailable_channel(data->ch[i]))
					continue;

				sx9385_check_status(data, data->ch[i]);
				data->ch[i]->diff_avg = 0;
				data->ch[i]->diff_cnt = 0;
				data->ch[i]->useful_avg = 0;
				if (data->ch[i]->state == HAS_ERROR)
					status[i] = IDLE;
				else
					status[i] = data->ch[i]->state;
			}

			msleep(20);
			/* make interrupt high */
			sx9385_read_reg_stat(data);

			/* enable interrupt */
			if (sx9385_i2c_write(data, REG_IRQ_MSK, 0x30) < 0)
				GRIP_INFO("set enable irq reg fail\n");

			if (data->is_irq_active == false) {
				enable_irq(data->irq);
				enable_irq_wake(data->irq);
				data->is_irq_active = true;
			}

			for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
				if (sx9385_is_unavailable_channel(data->ch[i]))
					continue;

				check_irq_error(data, status[i], BY_ENABLE_FUNC, data->ch[i]);
			}

			atomic_set(&data->enable, ON);
		}
	} else {
		if (pre_enable == ON) {
			/* disable interrupt */
			if (sx9385_i2c_write(data, REG_IRQ_MSK, 0x00) < 0)
				GRIP_INFO("set disable irq reg fail\n");

			if (data->is_irq_active == true) {
				disable_irq(data->irq);
				disable_irq_wake(data->irq);
				data->is_irq_active = false;
			}
			atomic_set(&data->enable, OFF);
		}
	}
}

static void sx9385_set_debug_work(struct sx9385_p *data, u8 enable,
				  unsigned int time_ms)
{
	if (enable == ON && !data->check_abnormal_working) {
		data->debug_count = 0;
		data->debug_zero_count = 0;
		schedule_delayed_work(&data->debug_work,
					  msecs_to_jiffies(time_ms));
	} else {
		cancel_delayed_work_sync(&data->debug_work);
	}
}

static void sx9385_get_refcap(struct sx9385_p *data)
{
	int idx;
	int coffset, cuseful;
	s16 use[NUM_MAIN_PHASES];
	u16 off[NUM_MAIN_PHASES];
	u8 reg_use_msb[] = {0xBA, 0xCD};
	u8 reg_off_msb[] = {0x4B, 0x58};

	for (idx = 0; idx < data->num_of_refs; idx++) {
		use[idx] = sx9385_read_and_calculate_useful(data, reg_use_msb, idx);
		off[idx] = sx9385_read_and_calculate_offset(data, reg_off_msb, idx);
		coffset = sx9385_calculate_coffset(off[idx] >> 8, off[idx] & 0xFF);
		cuseful = sx9385_calculate_cuseful(use[idx], data->again_r[idx], data->dgain_r[idx]);
		data->refMain[idx] = coffset + cuseful;

		GRIP_INFO("refCap[%d] = %d\n", idx+1, data->refMain[idx]);
	}
}

static void sx9385_read_channel_mapping(struct sx9385_p *data)
{
	int ret, i;
	u8 cs_addr[] = {0x50, 0x5D};
	u8 cs_reg;

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (sx9385_is_unavailable_channel(data->ch[i]))
			continue;

		ret = sx9385_i2c_read(data, cs_addr[i], &cs_reg);
		if (ret < 0) {
			data->ch[i]->cs_mapping = CS_UNKNOWN;
			continue;
		}

		switch (cs_reg) {
		case CS0_REG:
			data->ch[i]->cs_mapping = CS0;
			GRIP_INFO("ch[%d] CS0\n", i);
			break;
		case CS1_REG:
			data->ch[i]->cs_mapping = CS1;
			GRIP_INFO("ch[%d] CS1\n", i);
			break;
		case CS2_REG:
			data->ch[i]->cs_mapping = CS2;
			GRIP_INFO("ch[%d] CS2\n", i);
			break;
		case CS3_REG:
			data->ch[i]->cs_mapping = CS3;
			GRIP_INFO("ch[%d] CS3\n", i);
			break;
		default:
			GRIP_INFO("ch[%d] UNKNOWN\n", i);
			data->ch[i]->cs_mapping = CS_UNKNOWN;
		}
	}
}

static ssize_t sx9385_enable_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&data->enable));
}

static ssize_t sx9385_enable_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, size_t size)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 enable;

	if (sx9385_convert_input(KSTRTO8_FUNCTION, buf, &enable, NULL) != 0)
		return -EINVAL;

	GRIP_INFO("new val %u\n", enable);
	if ((enable == 0) || (enable == 1))
		sx9385_set_enable(data, (int)enable);

	return size;
}

static DEVICE_ATTR(enable, 0664, sx9385_enable_show, sx9385_enable_store);

static struct attribute *sx9385_attributes[] = {
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group sx9385_attribute_group = {
	.attrs = sx9385_attributes
};

static ssize_t sx9385_ref_cap_show(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);

	sx9385_get_refcap(data);

	return sprintf(buf, "%d\n", data->refMain[0]);
}
static ssize_t sx9385_ref_cap_b_show(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);

	sx9385_get_refcap(data);

	return sprintf(buf, "%d\n", data->refMain[1]);
}
static ssize_t sx9385_get_offset_calibration_show(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 reg_value = 0;

	sx9385_i2c_read(data, REG_COMPENSATION, &reg_value); //check

	return sprintf(buf, "%d\n", reg_value);
}

static ssize_t sx9385_set_offset_calibration_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	unsigned long val;

	if (sx9385_convert_input(KSTTOUL_FUNCTION, buf, &val, NULL) != 0)
		return -EINVAL;

	if (val)
		sx9385_manual_offset_calibration(data);

	return count;
}

static ssize_t sx9385_sw_reset_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	int retry = 0;
	u8 compstat = 0xFF;

	GRIP_INFO("\n");
	sx9385_manual_offset_calibration(data);
	msleep(400);

	while (retry++ <= 15) {
		sx9385_i2c_read(data, REG_COMPENSATION, &compstat);
		GRIP_INFO("compstat 0x%x\n", compstat);
		if (data->set_up_channels == CH_BOTH)
			compstat &= 0x0A;
		else if (data->set_up_channels == CH2_ONLY)
			compstat &= 0x08;
		else if (data->set_up_channels == CH1_ONLY)
			compstat &= 0x02;

		if (compstat == 0)
			break;

		msleep(50);
	}

	sx9385_get_data(data);

	if (compstat == 0)
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);

	sx9385_print_registers(data);
	return snprintf(buf, PAGE_SIZE, "%d\n", -1);
}

static ssize_t sx9385_vendor_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR_NAME);
}

static ssize_t sx9385_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%s\n", device_name[data->ic_num]);
}

static ssize_t sx9385_touch_mode_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "1\n");
}

static ssize_t sx9385_scan_period_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 scan_period = 0;

	sx9385_i2c_read(data, REG_GNRL_CTRL2, &scan_period);

	scan_period = scan_period & 0x7F;

	return snprintf(buf, PAGE_SIZE, "%ld\n",
			(long int)(((long int)scan_period << 12) / 4000));
}

static ssize_t sx9385_phase_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "1\n");
}

static ssize_t sx9385_onoff_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", !data->skip_data);
}

static ssize_t sx9385_onoff_store(struct device *dev,
				  struct device_attribute *attr, const char *buf, size_t count)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 val;

	if (sx9385_convert_input(KSTRTO8_FUNCTION, buf, &val, NULL) != 0)
		return -EINVAL;

	if (val == 0) {
		int i;

		data->skip_data = true;
		for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
			if (sx9385_is_unavailable_channel(data->ch[i]))
				continue;

			if (atomic_read(&data->enable) == ON) {
				input_report_rel(data->input, data->ch[i]->grip_code, GRIP_RELEASE);
				if (data->ch[i]->unknown_sel)
					input_report_rel(data->input, data->ch[i]->unknown_code, UNKNOWN_OFF);
				input_sync(data->input);
			}

			data->motion = 1;
			data->ch[i]->is_unknown_mode = UNKNOWN_OFF;
			data->ch[i]->first_working = false;
		}
	} else {
		data->skip_data = false;
	}

	GRIP_INFO("%u\n", val);
	return count;
}

static ssize_t sx9385_register_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	char *p = buf;
	int idx, i = 0;
	u8 val;

	for (idx = 0; idx < data->num_regs; idx++) {
		sx9385_i2c_read(data, data->regs_addr_val[idx].addr, &val);
		i += snprintf(p + i, PAGE_SIZE - i, "(0x%02x)=0x%02x\n", data->regs_addr_val[idx].addr, val);
	}

	return i;
}

static ssize_t sx9385_register_write_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	int reg_address = 0, val = 0;

	if (sx9385_convert_input(SSCANF_TWO_HEX_VALUE_FUNCTION, buf, &reg_address, &val) != 0)
		return -EINVAL;

	sx9385_i2c_write(data, (unsigned char)reg_address, (unsigned char)val);
	GRIP_INFO("Regi 0x%x Val 0x%x\n", reg_address, val);

	sx9385_get_gain(data);
	sx9385_get_ref_gain(data);

	return count;
}

// static unsigned int register_read_store_reg; // It's not used
static unsigned int register_read_store_val;
static ssize_t sx9385_register_read_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%02x\n", register_read_store_val);
}

static ssize_t sx9385_register_read_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	int regist = 0;
	u8 val = 0;

	GRIP_INFO("\n");

	if (sx9385_convert_input(SSCANF_HEX_VALUE_FUNCTION, buf, &val, NULL) != 0)
		return -EINVAL;

	sx9385_i2c_read(data, regist, &val);
	GRIP_INFO("Regi 0x%2x Val 0x%2x\n", regist, val);
//	register_read_store_reg = (unsigned int)regist; // It's not used
	register_read_store_val = (unsigned int)val;
	return count;
}

/* show far near status reg data */
static ssize_t sx9385_status_regdata_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 val0 = 0;

	sx9385_i2c_read(data, REG_PROX_STATUS, &val0);
	GRIP_INFO("Status 0x%2x\n", val0);

	return sprintf(buf, "%d\n", val0);
}

/* check if manual calibrate success or not */
static ssize_t sx9385_cal_state_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	int status;
	u8 val = 0;

	sx9385_i2c_read(data, REG_COMPENSATION, &val);
	GRIP_INFO("Regi 0x01 Val 0x%2x\n", val);
	if ((val & 0x0F) == 0)
		status = 1;
	else
		status = 0;

	return sprintf(buf, "%d\n", status);
}

static ssize_t sx9385_motion_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%s\n",
		data->motion == 1 ? "motion_detect" : "motion_non_detect");
}

static ssize_t sx9385_motion_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 val;

	if (sx9385_convert_input(KSTRTO8_FUNCTION, buf, &val, NULL) != 0)
		return -EINVAL;

	data->motion = val;

	GRIP_INFO("%u\n", val);
	return count;
}

static ssize_t sx9385_noti_enable_store(struct device *dev,
					 struct device_attribute *attr, const char *buf, size_t size)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	int i;
	u8 enable;

	if (sx9385_convert_input(KSTRTO8_FUNCTION, buf, &enable, NULL) != 0)
		return -EINVAL;

	GRIP_INFO("new val %d\n", (int)enable);

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (sx9385_is_unavailable_channel(data->ch[i]))
			continue;

		data->ch[i]->noti_enable = enable & data->ch[i]->unknown_sel;
		if (data->ch[i]->noti_enable)
			enter_unknown_mode(data, TYPE_BOOT, data->ch[i]);
		else {
			data->motion = 1;
			data->ch[i]->first_working = false;
			data->ch[i]->is_unknown_mode = UNKNOWN_OFF;
		}

	}
	return size;
}

static ssize_t sx9385_noti_enable_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->ch[0]->noti_enable);
}

static ssize_t sx9385_raw_data_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);

	sx9385_get_data(data);

	if (data->ch[0]->diff_cnt == 0) {
		data->ch[0]->sum_diff = (s32)data->ch[0]->diff;
		data->ch[0]->sum_useful = data->ch[0]->useful;
	} else {
		data->ch[0]->sum_diff += (s32)data->ch[0]->diff;
		data->ch[0]->sum_useful += data->ch[0]->useful;
	}

	if (++data->ch[0]->diff_cnt >= DIFF_READ_NUM) {
		data->ch[0]->diff_avg = (s16)(data->ch[0]->sum_diff / DIFF_READ_NUM);
		data->ch[0]->useful_avg = data->ch[0]->sum_useful / DIFF_READ_NUM;
		data->ch[0]->diff_cnt = 0;
	}

	return snprintf(buf, PAGE_SIZE, "%ld,%ld,%u,%d,%d\n", (long int)data->ch[0]->capMain,
			(long int)data->ch[0]->useful, data->ch[0]->offset, data->ch[0]->diff, data->ch[0]->avg);
}


static ssize_t sx9385_diff_avg_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->ch[0]->diff_avg);
}

static ssize_t sx9385_useful_avg_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%ld\n", (long int)data->ch[0]->useful_avg);
}



static ssize_t sx9385_avgnegfilt_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 avgnegfilt = 0;

	sx9385_i2c_read(data, REG_PROX_CTRL3_PH2, &avgnegfilt);

	avgnegfilt = (avgnegfilt & 0x70) >> 4;

	if (avgnegfilt == 7)
		return snprintf(buf, PAGE_SIZE, "1\n");
	else if (avgnegfilt > 0 && avgnegfilt < 7)
		return snprintf(buf, PAGE_SIZE, "1-1/%d\n", 1 << avgnegfilt);
	else if (avgnegfilt == 0)
		return snprintf(buf, PAGE_SIZE, "0\n");

	return snprintf(buf, PAGE_SIZE, "not set\n");
}


static ssize_t sx9385_avgposfilt_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 avgposfilt = 0;

	sx9385_i2c_read(data, REG_PROX_CTRL3_PH2, &avgposfilt);
	avgposfilt = avgposfilt & 0x0F;

	if (avgposfilt == 0x0F)
		return snprintf(buf, PAGE_SIZE, "1\n");
	else if (avgposfilt > 0 && avgposfilt <= 0x0A)
		return snprintf(buf, PAGE_SIZE, "1-1/%d\n", 1 << avgposfilt);
	else
		return snprintf(buf, PAGE_SIZE, "0\n");
}



static ssize_t sx9385_gain_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 msByte = 0, dgain;

	sx9385_i2c_read(data, REG_AFE_PARAM2_PH2, &msByte);

	dgain = (msByte >> 5) & 0x07;
	if (dgain > 0 && dgain < 5)
		return snprintf(buf, PAGE_SIZE, "x%u\n", 1 << (dgain - 1));

	return snprintf(buf, PAGE_SIZE, "Reserved\n");
}

static ssize_t sx9385_avgthresh_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 avgthresh = 0;

	sx9385_i2c_read(data, REG_PROX_CTRL1_PH2, &avgthresh);
	avgthresh = avgthresh & 0x3F;

	return snprintf(buf, PAGE_SIZE, "%ld\n", 512 * (long int)avgthresh);
}


static ssize_t sx9385_rawfilt_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 rawfilt = 0;

	sx9385_i2c_read(data, REG_PROX_CTRL0_PH12, &rawfilt);
	rawfilt = rawfilt & 0x07;

	if (rawfilt > 0 && rawfilt < 8)
		return snprintf(buf, PAGE_SIZE, "1-1/%d\n", 1 << rawfilt);
	else
		return snprintf(buf, PAGE_SIZE, "0\n");
}

static ssize_t sx9385_sampling_freq_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 sampling_freq = 0;
	const char *table[16] = {
		"250", "200", "166.67", "142.86", "125", "100",	"83.33", "71.43",
		"62.50", "50", "41.67", "35.71", "27.78", "20.83", "15.62", "7.81"
	};

	sx9385_i2c_read(data, REG_AFE_PARAM0_PH2, &sampling_freq);
	sampling_freq = sampling_freq & 0x0F;

	return snprintf(buf, PAGE_SIZE, "%skHz\n", table[sampling_freq]);
}

static ssize_t sx9385_again_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 again = 0;
	const char *table[32] = {
		"+/-0.3875", "+/-0.775", "+/-1.1625", "+/-1.55",
		"+/-1.9375", "+/-2.325", "+/-2.7125", "+/-3.1",
		"+/-3.4875", "+/-3.875", "+/-4.2625", "+/-4.65",
		"+/-5.0375", "+/-5.425", "+/-5.8125", "+/-6.2",
		"+/-6.5875", "+/-6.975", "+/-7.3625", "+/-7.75",
		"+/-8.1375", "+/-8.525", "+/-8.9125", "+/-9.3",
		"+/-9.6875", "+/-10.075", "+/-10.4625", "+/-10.85",
		"+/-11.2375", "+/-11.625", "+/-12.0125", "+/-12.4",
	};

	sx9385_i2c_read(data, REG_AFE_PARAM2_PH2, &again);
	again = again & 0x1F;

	return snprintf(buf, PAGE_SIZE, "%spF\n", table[again]);
}

static ssize_t sx9385_hysteresis_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	const char *table[4] = {"None", "+/-6%", "+/-12%", "+/-25%"};
	u8 hyst = 0;

	sx9385_i2c_read(data, REG_PROX_CTRL4_PH2, &hyst);
	hyst = (hyst & 0x30) >> 4;

	return snprintf(buf, PAGE_SIZE, "%s\n", table[hyst]);
}

static ssize_t sx9385_resolution_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 resolution = 0;

	sx9385_i2c_read(data, REG_AFE_PARAM0_PH2_MSB, &resolution);
	resolution = (resolution & 0x70) >> 4;

	return snprintf(buf, PAGE_SIZE, "%u\n", 1 << (resolution + 3));
}

static ssize_t sx9385_useful_filt_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 useful_filt = 0;

	sx9385_i2c_read(data, REG_USEFILTER0_PH2, &useful_filt);
	useful_filt = useful_filt & 0x80;

	return snprintf(buf, PAGE_SIZE, "%s\n", useful_filt ? "on" : "off");
}

static ssize_t sx9385_closedeb_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 closedeb = 0;

	sx9385_i2c_read(data, REG_PROX_CTRL4_PH2, &closedeb);
	closedeb = closedeb & 0x0C;
	closedeb = closedeb >> 2;

	if (closedeb == 0)
		return snprintf(buf, PAGE_SIZE, "off");
	return snprintf(buf, PAGE_SIZE, "%d\n", 1 << closedeb);
}

static ssize_t sx9385_fardeb_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 fardeb = 0;

	sx9385_i2c_read(data, REG_PROX_CTRL4_PH2, &fardeb);
	fardeb  = fardeb  & 0x03;

	if (fardeb == 0)
		return snprintf(buf, PAGE_SIZE, "off");
	return snprintf(buf, PAGE_SIZE, "%d\n", 1 << fardeb);
}

static ssize_t sx9385_irq_count_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	int ret = 0;
	s16 max_diff_val = 0;

	if (data->irq_count) {
		ret = -1;
		max_diff_val = data->ch[0]->max_diff;
	} else {
		max_diff_val = data->ch[0]->max_normal_diff;
	}

	GRIP_INFO("called\n");

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n",
			ret, data->irq_count, max_diff_val);
}

static ssize_t sx9385_irq_count_store(struct device *dev,
					  struct device_attribute *attr, const char *buf, size_t count)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 onoff;

	if (sx9385_convert_input(KSTRTO8_FUNCTION, buf, &onoff, NULL) != 0)
		return -EINVAL;

	mutex_lock(&data->read_mutex);

	if (onoff == 0) {
		data->abnormal_mode = OFF;
	} else if (onoff == 1) {
		data->abnormal_mode = ON;
		data->irq_count = 0;
		data->ch[0]->max_diff = 0;
		data->ch[0]->max_normal_diff = 0;
	} else {
		GRIP_ERR("unknown val %d\n", onoff);
	}

	mutex_unlock(&data->read_mutex);

	GRIP_INFO("%d\n", onoff);

	return count;
}

static ssize_t sx9385_normal_threshold_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u32 threshold = 0, hyst = 0;
	u8 th_buf = 0, hyst_buf = 0;

	if (sx9385_i2c_read(data, REG_PROX_CTRL5_PH2, &th_buf) < 0)
		GRIP_ERR("detect threshold update fail\n");
	else
		threshold = sx9385_calculate_threshold(th_buf);

	if (sx9385_i2c_read(data, REG_PROX_CTRL4_PH2, &hyst_buf) < 0)
		GRIP_ERR("detect hyst update fail\n");
	else
		hyst = sx9385_calculate_hyst(hyst_buf, threshold);

	return snprintf(buf, PAGE_SIZE, "%d,%d\n",
			threshold + hyst, threshold - hyst);
}

static ssize_t sx9385_status_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	int status;
	u8 val = 0;

	sx9385_i2c_read(data, REG_PROX_STATUS, &val);

	if ((val & 0x10) == 0)//bit3 for proxstat
		status = 0;
	else
		status = 1;

	return sprintf(buf, "%d\n", status);
}

static ssize_t sx9385_unknown_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%s\n",
		(data->ch[0]->is_unknown_mode == UNKNOWN_ON) ?	"UNKNOWN" : "NORMAL");
}

static ssize_t sx9385_unknown_state_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 val;

	if (sx9385_convert_input(KSTRTO8_FUNCTION, buf, &val, NULL) != 0)
		return -EINVAL;

	if (val == 1)
		enter_unknown_mode(data, TYPE_FORCE, data->ch[0]);
	else if (val == 0)
		data->ch[0]->is_unknown_mode = UNKNOWN_OFF;
	else
		GRIP_INFO("Invalid Val %u\n", val);

	GRIP_INFO("%u\n", val);

	return count;
}

static ssize_t sx9385_cs_map_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);

	sx9385_read_channel_mapping(data);

	if (data->ch[0]->cs_mapping == CS_UNKNOWN)
		return snprintf(buf, PAGE_SIZE, "UNKNOWN\n");

	return snprintf(buf, PAGE_SIZE, "CS%u\n", data->ch[0]->cs_mapping);
}

static ssize_t sx9385_unknown_state_b_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 val;

	if (sx9385_convert_input(KSTRTO8_FUNCTION, buf, &val, NULL) != 0)
		return -EINVAL;

	if (val == 1)
		enter_unknown_mode(data, TYPE_FORCE, data->ch[1]);
	else if (val == 0)
		data->ch[1]->is_unknown_mode = UNKNOWN_OFF;
	else
		GRIP_INFO("Invalid Val %u\n", val);

	GRIP_INFO("%u\n", val);

	return count;
}

static ssize_t sx9385_raw_data_b_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);

	sx9385_get_data(data);

	if (data->ch[1]->diff_cnt == 0) {
		data->ch[1]->sum_diff = (s32)data->ch[1]->diff;
		data->ch[1]->sum_useful = data->ch[1]->useful;
	} else {
		data->ch[1]->sum_diff += (s32)data->ch[1]->diff;
		data->ch[1]->sum_useful += data->ch[1]->useful;
	}

	if (++data->ch[1]->diff_cnt >= DIFF_READ_NUM) {
		data->ch[1]->diff_avg = (s16)(data->ch[1]->sum_diff / DIFF_READ_NUM);
		data->ch[1]->useful_avg = data->ch[1]->sum_useful / DIFF_READ_NUM;
		data->ch[1]->diff_cnt = 0;
	}

	return snprintf(buf, PAGE_SIZE, "%ld,%ld,%u,%d,%d\n", (long int)data->ch[1]->capMain,
			(long int)data->ch[1]->useful, data->ch[1]->offset, data->ch[1]->diff, data->ch[1]->avg);
}

static ssize_t sx9385_irq_count_b_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	int ret = 0;
	s16 max_diff_val;

	if (data->irq_count) {
		ret = -1;
		max_diff_val = data->ch[1]->max_diff;
	} else {
		max_diff_val = data->ch[1]->max_normal_diff;
	}

	GRIP_INFO("called\n");

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n",
			ret, data->irq_count, max_diff_val);
}

static ssize_t sx9385_irq_count_b_store(struct device *dev,
					  struct device_attribute *attr, const char *buf, size_t count)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 onoff;

	if (sx9385_convert_input(KSTRTO8_FUNCTION, buf, &onoff, NULL) != 0)
		return -EINVAL;

	mutex_lock(&data->read_mutex);

	if (onoff == 0) {
		data->abnormal_mode = OFF;
	} else if (onoff == 1) {
		data->abnormal_mode = ON;
		data->irq_count = 0;
		data->ch[1]->max_diff = 0;
		data->ch[1]->max_normal_diff = 0;
	} else {
		GRIP_ERR("unknown val %d\n", onoff);
	}

	mutex_unlock(&data->read_mutex);

	GRIP_INFO("%d\n", onoff);

	return count;
}

static ssize_t sx9385_diff_avg_b_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->ch[1]->diff_avg);
}
static ssize_t sx9385_useful_avg_b_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%ld\n", (long int)data->ch[1]->useful_avg);
}
static ssize_t sx9385_avgnegfilt_b_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 avgnegfilt = 0;

	sx9385_i2c_read(data, REG_PROX_CTRL3_PH4, &avgnegfilt);

	avgnegfilt = (avgnegfilt & 0x70) >> 4;

	if (avgnegfilt == 7)
		return snprintf(buf, PAGE_SIZE, "1\n");
	else if (avgnegfilt > 0 && avgnegfilt < 7)
		return snprintf(buf, PAGE_SIZE, "1-1/%d\n", 1 << avgnegfilt);
	else if (avgnegfilt == 0)
		return snprintf(buf, PAGE_SIZE, "0\n");

	return snprintf(buf, PAGE_SIZE, "not set\n");
}
static ssize_t sx9385_avgposfilt_b_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 avgposfilt = 0;

	sx9385_i2c_read(data, REG_PROX_CTRL3_PH4, &avgposfilt);
	avgposfilt = avgposfilt & 0x0F;

	if (avgposfilt == 0x0F)
		return snprintf(buf, PAGE_SIZE, "1\n");
	else if (avgposfilt > 0 && avgposfilt <= 0x0A)
		return snprintf(buf, PAGE_SIZE, "1-1/%d\n", 1 << avgposfilt);
	else
		return snprintf(buf, PAGE_SIZE, "0\n");
}

static ssize_t sx9385_gain_b_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 msByte = 0;
	u8 dgain;

	sx9385_i2c_read(data, REG_AFE_PARAM2_PH4, &msByte);
	dgain = (msByte >> 5) & 0x07;

	if (dgain > 0 && dgain < 5)
		return snprintf(buf, PAGE_SIZE, "x%u\n", 1 << (dgain - 1));

	return snprintf(buf, PAGE_SIZE, "Reserved\n");
}

static ssize_t sx9385_avgthresh_b_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 avgthresh = 0;

	sx9385_i2c_read(data, REG_PROX_CTRL1_PH4, &avgthresh);
	avgthresh = avgthresh & 0x3F;

	return snprintf(buf, PAGE_SIZE, "%ld\n", 512 * (long int)avgthresh);
}
static ssize_t sx9385_rawfilt_b_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 rawfilt = 0;

	sx9385_i2c_read(data, REG_PROX_CTRL0_PH34, &rawfilt);
	rawfilt = rawfilt & 0x07;

	if (rawfilt > 0 && rawfilt < 8)
		return snprintf(buf, PAGE_SIZE, "1-1/%d\n", 1 << rawfilt);
	else
		return snprintf(buf, PAGE_SIZE, "0\n");
}

static ssize_t sx9385_sampling_freq_b_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 sampling_freq = 0;
	const char *table[16] = {
		"250", "200", "166.67", "142.86", "125", "100",	"83.33", "71.43",
		"62.50", "50", "41.67", "35.71", "27.78", "20.83", "15.62", "7.81"
	};

	sx9385_i2c_read(data, REG_AFE_PARAM0_PH4, &sampling_freq);
	sampling_freq = sampling_freq & 0x0F;

	return snprintf(buf, PAGE_SIZE, "%skHz\n", table[sampling_freq]);
}

static ssize_t sx9385_again_b_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 again = 0;
	const char *table[32] = {
		"+/-0.3875", "+/-0.775", "+/-1.1625", "+/-1.55",
		"+/-1.9375", "+/-2.325", "+/-2.7125", "+/-3.1",
		"+/-3.4875", "+/-3.875", "+/-4.2625", "+/-4.65",
		"+/-5.0375", "+/-5.425", "+/-5.8125", "+/-6.2",
		"+/-6.5875", "+/-6.975", "+/-7.3625", "+/-7.75",
		"+/-8.1375", "+/-8.525", "+/-8.9125", "+/-9.3",
		"+/-9.6875", "+/-10.075", "+/-10.4625", "+/-10.85",
		"+/-11.2375", "+/-11.625", "+/-12.0125", "+/-12.4",
	};

	sx9385_i2c_read(data, REG_AFE_PARAM2_PH4, &again);
	again = again & 0x1F;

	return snprintf(buf, PAGE_SIZE, "%spF\n", table[again]);
}
static ssize_t sx9385_hysteresis_b_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	const char *table[4] = {"None", "+/-6%", "+/-12%", "+/-25%"};
	u8 hyst = 0;

	sx9385_i2c_read(data, REG_PROX_CTRL4_PH4, &hyst);
	hyst = (hyst & 0x30) >> 4;

	return snprintf(buf, PAGE_SIZE, "%s\n", table[hyst]);
}
static ssize_t sx9385_resolution_b_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 resolution = 0;

	sx9385_i2c_read(data, REG_AFE_PARAM0_PH4_MSB, &resolution);
	resolution = (resolution & 0x70) >> 4;

	return snprintf(buf, PAGE_SIZE, "%u\n", 1 << (resolution + 3));
}

static ssize_t sx9385_useful_filt_b_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 useful_filt = 0;

	sx9385_i2c_read(data, REG_USEFILTER0_PH4, &useful_filt);
	useful_filt = useful_filt & 0x80;

	return snprintf(buf, PAGE_SIZE, "%s\n", useful_filt ? "on" : "off");
}

static ssize_t sx9385_closedeb_b_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 closedeb = 0;

	sx9385_i2c_read(data, REG_PROX_CTRL4_PH4, &closedeb);
	closedeb = closedeb & 0x0C;
	closedeb = closedeb >> 2;

	if (closedeb == 0)
		return snprintf(buf, PAGE_SIZE, "off");

	return snprintf(buf, PAGE_SIZE, "%d\n", 1 << closedeb);
}

static ssize_t sx9385_fardeb_b_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u8 fardeb = 0;

	sx9385_i2c_read(data, REG_PROX_CTRL4_PH4, &fardeb);
	fardeb  = fardeb  & 0x03;

	if (fardeb == 0)
		return snprintf(buf, PAGE_SIZE, "off");

	return snprintf(buf, PAGE_SIZE, "%d\n", 1 << fardeb);
}

static ssize_t sx9385_normal_threshold_b_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	u32 threshold = 0, hyst = 0;
	u8 th_buf = 0, hyst_buf = 0;

	if (sx9385_i2c_read(data, REG_PROX_CTRL5_PH4, &th_buf) < 0)
		GRIP_ERR("detect threshold update fail\n");
	else
		threshold = sx9385_calculate_threshold(th_buf);

	if (sx9385_i2c_read(data, REG_PROX_CTRL4_PH4, &hyst_buf) < 0)
		GRIP_ERR("detect hyst update fail\n");
	else
		hyst = sx9385_calculate_hyst(hyst_buf, threshold);

	return snprintf(buf, PAGE_SIZE, "%d,%d\n",
			threshold + hyst, threshold - hyst);
}

static ssize_t sx9385_status_b_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	int status;
	u8 val = 0;

	sx9385_i2c_read(data, REG_PROX_STATUS, &val);

	if ((val & 0x20) == 0)//bit3 for proxstat
		status = 0;
	else
		status = 1;

	return sprintf(buf, "%d\n", status);
}

static ssize_t sx9385_unknown_state_b_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%s\n",
		(data->ch[1]->is_unknown_mode == UNKNOWN_ON) ?	"UNKNOWN" : "NORMAL");
}

static ssize_t sx9385_cs_map_b_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sx9385_p *data = dev_get_drvdata(dev);

	sx9385_read_channel_mapping(data);

	if (data->ch[1]->cs_mapping == CS_UNKNOWN)
		return snprintf(buf, PAGE_SIZE, "UNKNOWN\n");

	return snprintf(buf, PAGE_SIZE, "CS%u\n", data->ch[1]->cs_mapping);
}

#if IS_ENABLED(CONFIG_SENSORS_REGION_DEPENDENCY)
static ssize_t sx9385_is_grip_mounted_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", is_grip_mounted);
}

static DEVICE_ATTR(is_grip_mounted, 0444, sx9385_is_grip_mounted_show, NULL);

static struct device_attribute *is_grip_mounted_attrs[] = {
	&dev_attr_is_grip_mounted,
	NULL,
};

static void sx9385_grip_mounted_register(struct sx9385_p *data, int offset)
{
	is_grip_mounted = true;
	memcpy(data->sensor_attributes + offset, is_grip_mounted_attrs, sizeof(is_grip_mounted_attrs));
}
#endif

static DEVICE_ATTR(grip_sar_enable, 0664, sx9385_enable_show, sx9385_enable_store);
static DEVICE_ATTR(grip_ref_cap, 0444, sx9385_ref_cap_show, NULL);
static DEVICE_ATTR(grip_ref_cap_b, 0444, sx9385_ref_cap_b_show, NULL);
static DEVICE_ATTR(regproxdata, 0444, sx9385_status_regdata_show, NULL);
static DEVICE_ATTR(cal_state, 0444, sx9385_cal_state_show, NULL);
static DEVICE_ATTR(menual_calibrate, 0664,
		   sx9385_get_offset_calibration_show,
		   sx9385_set_offset_calibration_store);
static DEVICE_ATTR(register_write, 0220, NULL, sx9385_register_write_store);
static DEVICE_ATTR(register_read_all, 0444, sx9385_register_show, NULL);
static DEVICE_ATTR(register_read, 0664,
			sx9385_register_read_show, sx9385_register_read_store);
static DEVICE_ATTR(reset, 0444, sx9385_sw_reset_show, NULL);
static DEVICE_ATTR(name, 0444, sx9385_name_show, NULL);
static DEVICE_ATTR(vendor, 0444, sx9385_vendor_show, NULL);
static DEVICE_ATTR(mode, 0444, sx9385_touch_mode_show, NULL);
static DEVICE_ATTR(onoff, 0664, sx9385_onoff_show, sx9385_onoff_store);
static DEVICE_ATTR(scan_period, 0444, sx9385_scan_period_show, NULL);
static DEVICE_ATTR(phase, 0444, sx9385_phase_show, NULL);
//static DEVICE_ATTR(resistor_filter_input, S_IRUGO, sx9385_resistor_filter_input_show, NULL);
static DEVICE_ATTR(motion, 0664, sx9385_motion_show, sx9385_motion_store);
static DEVICE_ATTR(noti_enable, 0664, sx9385_noti_enable_show, sx9385_noti_enable_store);

static DEVICE_ATTR(avg_negfilt, 0444, sx9385_avgnegfilt_show, NULL);
static DEVICE_ATTR(avg_posfilt, 0444, sx9385_avgposfilt_show, NULL);
static DEVICE_ATTR(avg_thresh, 0444, sx9385_avgthresh_show, NULL);
static DEVICE_ATTR(rawfilt, 0444, sx9385_rawfilt_show, NULL);
static DEVICE_ATTR(sampling_freq, 0444, sx9385_sampling_freq_show, NULL);
static DEVICE_ATTR(gain, 0444, sx9385_gain_show, NULL);
static DEVICE_ATTR(analog_gain, 0444, sx9385_again_show, NULL);
static DEVICE_ATTR(hysteresis, 0444, sx9385_hysteresis_show, NULL);
static DEVICE_ATTR(irq_count, 0664, sx9385_irq_count_show, sx9385_irq_count_store);
static DEVICE_ATTR(resolution, 0444, sx9385_resolution_show, NULL);
static DEVICE_ATTR(useful_filt, 0444, sx9385_useful_filt_show, NULL);
static DEVICE_ATTR(closedeb, 0444, sx9385_closedeb_show, NULL);
static DEVICE_ATTR(fardeb, 0444, sx9385_fardeb_show, NULL);
static DEVICE_ATTR(unknown_state, 0664, sx9385_unknown_state_show, sx9385_unknown_state_store);
static DEVICE_ATTR(proxstatus, 0444, sx9385_status_show, NULL);
static DEVICE_ATTR(raw_data, 0444, sx9385_raw_data_show, NULL);
static DEVICE_ATTR(diff_avg, 0444, sx9385_diff_avg_show, NULL);
static DEVICE_ATTR(useful_avg, 0444, sx9385_useful_avg_show, NULL);
static DEVICE_ATTR(normal_threshold, 0444, sx9385_normal_threshold_show, NULL);
static DEVICE_ATTR(cs_map, 0444, sx9385_cs_map_show, NULL);

static DEVICE_ATTR(avg_negfilt_b, 0444, sx9385_avgnegfilt_b_show, NULL);
static DEVICE_ATTR(avg_posfilt_b, 0444, sx9385_avgposfilt_b_show, NULL);
static DEVICE_ATTR(avg_thresh_b, 0444, sx9385_avgthresh_b_show, NULL);
static DEVICE_ATTR(rawfilt_b, 0444, sx9385_rawfilt_b_show, NULL);
static DEVICE_ATTR(sampling_freq_b, 0444, sx9385_sampling_freq_b_show, NULL);
static DEVICE_ATTR(gain_b, 0444, sx9385_gain_b_show, NULL);
static DEVICE_ATTR(analog_gain_b, 0444, sx9385_again_b_show, NULL);
static DEVICE_ATTR(hysteresis_b, 0444, sx9385_hysteresis_b_show, NULL);
static DEVICE_ATTR(irq_count_b, 0664, sx9385_irq_count_b_show, sx9385_irq_count_b_store);
static DEVICE_ATTR(resolution_b, 0444, sx9385_resolution_b_show, NULL);
static DEVICE_ATTR(useful_filt_b, 0444, sx9385_useful_filt_b_show, NULL);
static DEVICE_ATTR(closedeb_b, 0444, sx9385_closedeb_b_show, NULL);
static DEVICE_ATTR(fardeb_b, 0444, sx9385_fardeb_b_show, NULL);
static DEVICE_ATTR(unknown_state_b, 0664, sx9385_unknown_state_b_show, sx9385_unknown_state_b_store);
static DEVICE_ATTR(proxstatus_b, 0444, sx9385_status_b_show, NULL);
static DEVICE_ATTR(raw_data_b, 0444, sx9385_raw_data_b_show, NULL);
static DEVICE_ATTR(diff_avg_b, 0444, sx9385_diff_avg_b_show, NULL);
static DEVICE_ATTR(useful_avg_b, 0444, sx9385_useful_avg_b_show, NULL);
static DEVICE_ATTR(normal_threshold_b, 0444, sx9385_normal_threshold_b_show, NULL);
static DEVICE_ATTR(cs_map_b, 0444, sx9385_cs_map_b_show, NULL);

static struct device_attribute *grip_common_sensor_attrs[] = {
	&dev_attr_grip_sar_enable,
	&dev_attr_grip_ref_cap,
	&dev_attr_grip_ref_cap_b,
	&dev_attr_regproxdata,
	&dev_attr_cal_state,
	&dev_attr_menual_calibrate,
	&dev_attr_register_write,
	&dev_attr_register_read,
	&dev_attr_register_read_all,
	&dev_attr_reset,
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_mode,
	&dev_attr_onoff,
	&dev_attr_scan_period,
	&dev_attr_phase,
	&dev_attr_motion,
	&dev_attr_noti_enable,
	NULL,
};

__visible_for_testing struct device_attribute *grip_ch1_sensor_attrs[] = {
	&dev_attr_avg_negfilt,
	&dev_attr_avg_posfilt,
	&dev_attr_avg_thresh,
	&dev_attr_rawfilt,
	&dev_attr_sampling_freq,
	&dev_attr_gain,
	&dev_attr_analog_gain,
	&dev_attr_hysteresis,
	&dev_attr_irq_count,
	&dev_attr_resolution,
	&dev_attr_useful_filt,
	&dev_attr_closedeb,
	&dev_attr_fardeb,
	&dev_attr_unknown_state,
	&dev_attr_proxstatus,
	&dev_attr_raw_data,
	&dev_attr_diff_avg,
	&dev_attr_useful_avg,
	&dev_attr_normal_threshold,
	&dev_attr_cs_map,
	NULL,
};

static struct device_attribute *grip_ch2_sensor_attrs[] = {
	&dev_attr_avg_negfilt_b,
	&dev_attr_avg_posfilt_b,
	&dev_attr_avg_thresh_b,
	&dev_attr_rawfilt_b,
	&dev_attr_sampling_freq_b,
	&dev_attr_gain_b,
	&dev_attr_analog_gain_b,
	&dev_attr_hysteresis_b,
	&dev_attr_irq_count_b,
	&dev_attr_resolution_b,
	&dev_attr_useful_filt_b,
	&dev_attr_closedeb_b,
	&dev_attr_fardeb_b,
	&dev_attr_unknown_state_b,
	&dev_attr_proxstatus_b,
	&dev_attr_raw_data_b,
	&dev_attr_diff_avg_b,
	&dev_attr_useful_avg_b,
	&dev_attr_normal_threshold_b,
	&dev_attr_cs_map_b,
	NULL,
};

static void sx9385_check_diff_overflow(struct sx9385_p *data, int ch_num)
{
	s16 dif[NUM_MAIN_PHASES];
	u8 reg_dif_msb[] = {0xCB, 0xDE};
	u8 msb, lsb;

	sx9385_i2c_read(data, reg_dif_msb[ch_num] + 1, &lsb);
	sx9385_i2c_read(data, reg_dif_msb[ch_num], &msb);
	dif[ch_num] = msb<<8 | lsb;

	if (dif[ch_num] == DIFF_OVERFLOW)
		enter_unknown_mode(data, TYPE_OVERFLOW, data->ch[ch_num]);
}

static void sx9385_touch_process_ch(struct sx9385_p *data, struct channel *ch, u8 status)
{
	if (data->abnormal_mode) {
		if (status & ch->prox_mask) {
			if (ch->max_diff < ch->diff)
				ch->max_diff = ch->diff;
			data->irq_count++;
		}
	}

	if (ch->state == IDLE) {
		if (status & ch->prox_mask) {
			if (ch->is_unknown_mode == UNKNOWN_ON && data->motion)
				ch->first_working = true;
			sx9385_send_event(data, ACTIVE, ch);
		} else {
			sx9385_check_diff_overflow(data, ch->num);
			GRIP_INFO("0x%x already released.\n", status);
		}
	} else { /* User released button */
		if (!(status & ch->prox_mask)) {
			if (ch->is_unknown_mode == UNKNOWN_ON && data->motion) {
				GRIP_INFO("unknown mode off\n");
				ch->is_unknown_mode = UNKNOWN_OFF;
			}
			sx9385_check_diff_overflow(data, ch->num);
			sx9385_send_event(data, IDLE, ch);
		} else {
			GRIP_INFO("0x%x still touched\n", status);
		}
	}

	/* Test Code */
	if (ch->state == HAS_ERROR)
		status = IDLE;
	else
		status = ch->state;

	check_irq_error(data, status, BY_INTERRUPT_HANDLER, ch);
}

static void sx9385_touch_process(struct sx9385_p *data)
{
	int i;
	u8 status = 0, status_dif;

	sx9385_i2c_read(data, REG_PROX_STATUS, &status);

	GRIP_INFO("0x%x\n", status);

	sx9385_get_data(data);
	status_dif = data->prev_status ^ status;

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (sx9385_is_unavailable_channel(data->ch[i]))
			continue;

		if (status_dif & data->ch[i]->prox_mask)
			sx9385_touch_process_ch(data, data->ch[i], status);
	}
}

static void sx9385_process_interrupt(struct sx9385_p *data)
{
	int status;
	/* since we are not in an interrupt don't need to disable irq. */
	status = sx9385_read_reg_stat(data);

	GRIP_INFO("status %d\n", status);

	if (status & IRQ_PROCESS_CONDITION)
		sx9385_touch_process(data);
}

static void sx9385_irq_work_func(struct work_struct *work)
{
	struct sx9385_p *data = container_of((struct delayed_work *)work,
						 struct sx9385_p, irq_work);

	if (sx9385_get_nirq_state(data) == INTERRUPT_LOW)
		sx9385_process_interrupt(data);
	else
		GRIP_ERR("nirq read high %d\n", sx9385_get_nirq_state(data));
}

static void sx9385_check_first_working(struct sx9385_p *data, struct channel *ch)
{
	if (ch->noti_enable && data->motion) {
		if (ch->detect_threshold < ch->diff) {
			ch->first_working = true;
			GRIP_INFO("first working detected %d\n", ch->diff);
		} else if (ch->release_threshold > ch->diff) {
			if (ch->first_working) {
				ch->is_unknown_mode = UNKNOWN_OFF;
				GRIP_INFO("Release detected %d, unknown mode off\n", ch->diff);
			}
		}
	}
}

static void sx9385_debug_work_func(struct work_struct *work)
{
	struct sx9385_p *data = container_of((struct delayed_work *)work,
						 struct sx9385_p, debug_work);
	int i;

	if (data->check_abnormal_working == true)
		return;

	if ((atomic_read(&data->enable) == ON)
			&& (data->abnormal_mode)) {
		sx9385_get_data(data);
		for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
			if (sx9385_is_unavailable_channel(data->ch[i]))
				continue;

			if (data->ch[i]->max_normal_diff < data->ch[i]->diff)
				data->ch[i]->max_normal_diff = data->ch[i]->diff;
		}
	} else if (data->debug_count >= GRIP_LOG_TIME) {
		sx9385_get_data(data);
		for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
			if (sx9385_is_unavailable_channel(data->ch[i]))
				continue;

			if (data->ch[i]->is_unknown_mode == UNKNOWN_ON && data->motion)
				sx9385_check_first_working(data, data->ch[i]);
		}
		data->debug_count = 0;
	} else {
		data->debug_count++;
	}

	if (atomic_read(&data->enable) == ON) {
		u8 buf = 0;

		if (sx9385_i2c_read(data, REG_PROX_STATUS, &buf) < 0)
			return;

		for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
			u8 status;

			if (sx9385_is_unavailable_channel(data->ch[i]))
				continue;

			if (buf & (data->ch[i]->prox_mask))
				status = ACTIVE;
			else
				status = IDLE;
			check_irq_error(data, status, BY_DEBUG_WORK, data->ch[i]);
		}
	}
	schedule_delayed_work(&data->debug_work, msecs_to_jiffies(2000));
}

static irqreturn_t sx9385_interrupt_thread(int irq, void *pdata)
{
	struct sx9385_p *data = pdata;

	__pm_wakeup_event(data->grip_ws, jiffies_to_msecs(3 * HZ));
	schedule_delayed_work(&data->irq_work, msecs_to_jiffies(0));

	return IRQ_HANDLED;
}

static int sx9385_input_init(struct sx9385_p *data)
{
	struct input_dev *dev = NULL;
	unsigned int grip_code[2] = {REL_MISC, REL_DIAL};
	unsigned int unknown_code[2] = {REL_X, REL_Y};
	int ret = -ENOMEM, i;

	/* Create the input device */
	dev = input_allocate_device();
	if (!dev)
		return ret;

	dev->name =  module_name[data->ic_num];
	dev->id.bustype = BUS_I2C;

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (sx9385_is_unavailable_channel(data->ch[i]))
			continue;
		input_set_capability(dev, EV_REL, grip_code[i]);
		input_set_capability(dev, EV_REL, unknown_code[i]);
	}

	input_set_drvdata(dev, data);

	ret = input_register_device(dev);
	if (ret < 0) {
		input_free_device(dev);
		return ret;
	}

#if IS_ENABLED(CONFIG_SENSORS_CORE_AP)
	ret = sensors_create_symlink(&dev->dev.kobj, dev->name);
#else
	ret = sensors_create_symlink(dev);
#endif
	if (ret < 0) {
		GRIP_ERR("fail to create symlink %d\n", ret);
		input_unregister_device(dev);
		return ret;
	}

	ret = sysfs_create_group(&dev->dev.kobj, &sx9385_attribute_group);
	if (ret < 0) {
		GRIP_ERR("fail to create sysfs group %d\n", ret);
#if IS_ENABLED(CONFIG_SENSORS_CORE_AP)
		sensors_remove_symlink(&dev->dev.kobj, dev->name);
#else
		sensors_remove_symlink(dev);
#endif
		input_unregister_device(dev);
		return ret;
	}

	/* save the input pointer and finish initialization */
	data->input = dev;

	return 0;
}

static bool is_already_created_input_dev(void)
{
	if (grip_noti_input_dev != NULL)
		return true;
	return false;
}

static bool create_shared_noti_input_dev(struct i2c_client *client, struct sx9385_p *data)
{
	int ret = -ENODEV;

	mutex_lock(&noti_lock);
	if (is_already_created_input_dev()) {
		++noti_input_count;
		mutex_unlock(&noti_lock);
		return true;
	}

	grip_noti_input_dev = input_allocate_device();

	if (!grip_noti_input_dev) {
		GRIP_ERR("input_allocate_device failed\n");
		mutex_unlock(&noti_lock);
		return false;
	}

	grip_noti_input_dev->name = NOTI_MODULE_NAME;
	grip_noti_input_dev->id.bustype = BUS_I2C;

	input_set_capability(grip_noti_input_dev, EV_REL, REL_X);
	input_set_drvdata(grip_noti_input_dev, data);

	ret = input_register_device(grip_noti_input_dev);
	if (ret) {
		input_free_device(grip_noti_input_dev);
		grip_noti_input_dev = NULL;
		GRIP_ERR("failed to register input dev for noti (%d)\n", ret);
		mutex_unlock(&noti_lock);
		return false;
	}

	++noti_input_count;
	mutex_unlock(&noti_lock);

	return true;
}

void remove_shared_noti_input_dev(struct sx9385_p *data)
{
	mutex_lock(&noti_lock);
	if (--noti_input_count == 0) {
		input_unregister_device(grip_noti_input_dev);
		grip_noti_input_dev = NULL;
		GRIP_INFO("noti_input_dev remove complete : %d\n", noti_input_count);
	}
	mutex_unlock(&noti_lock);
}

void sx9385_destroy_noti_lock(void)
{
	if (--sx9385_module_count == 0)
		mutex_destroy(&noti_lock);
}

static int sx9385_noti_input_init(struct sx9385_p *data)
{
	int i;

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (sx9385_is_unavailable_channel(data->ch[i]))
			continue;

		data->ch[i]->unknown_sel = (data->unknown >> i) & 0x01;
	}

	if (!create_shared_noti_input_dev(data->client, data))
		return -1;

	return 0;
}

static void sx9385_get_revision(struct sx9385_p *data)
{
	s8 rev = -1;
	int ret;

	ret = sx9385_i2c_read(data, REG_REV_INFO, &rev);

	if (ret >= 0)
		GRIP_INFO("0x%x\n", rev);
}

static int sx9385_setup_pin(struct sx9385_p *data)
{
	int ret;

	ret = gpio_request(data->gpio_nirq, "SX9385_nIRQ");
	if (ret) {
		GRIP_ERR("gpio %d req fail %d\n", data->gpio_nirq, ret);
	} else {
		ret = gpio_direction_input(data->gpio_nirq);
		if (ret) {
			GRIP_ERR("fail to set gpio %d as input %d\n", data->gpio_nirq, ret);
			gpio_free(data->gpio_nirq);
		}
	}

	return ret;
}

static void sx9385_initialize_variable(struct sx9385_p *data)
{
	int i = 0;
	unsigned int grip_code[2] = {REL_MISC, REL_DIAL};
	unsigned int unknown_code[2] = {REL_X, REL_Y};

	data->skip_data = false;
	data->pre_attach = -1;
	data->motion = 1;
	data->is_suspend = false;

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (sx9385_is_unavailable_channel(data->ch[i]))
			continue;
		data->ch[i]->num = i;
		data->ch[i]->grip_code = grip_code[i];
		data->ch[i]->unknown_code = unknown_code[i];
		data->ch[i]->is_unknown_mode = UNKNOWN_OFF;
		data->ch[i]->first_working = false;
		data->ch[i]->state = IDLE;
		data->ch[i]->prox_mask = 1 << (4 + i);
	}
	atomic_set(&data->enable, OFF);
}

static int sx9385_configure_gpio_ldo(struct device_node *dNode, int ic_num)
{
	int gpio_ldo = 0;
	int ret = 0;

	gpio_ldo = of_get_named_gpio(dNode, "sx9385,ldo_en", 0);
	if (gpio_ldo >= 0) {
		ret = gpio_request(gpio_ldo, "grip_ldo_en");
		if (ret < 0) {
			pr_info("[GRIP_%s] gpio %d request failed %d\n", grip_name[ic_num], gpio_ldo, ret);
			return -EPROBE_DEFER;
		}

		gpio_direction_output(gpio_ldo, 1);
		pr_info("[GRIP_%s] ldo_en power %d\n", grip_name[ic_num], gpio_get_value(gpio_ldo));
		gpio_free(gpio_ldo);

		if (gpio_get_value_cansleep(gpio_ldo) == 0)
			return -EPROBE_DEFER;
	}

	return ret;
}

static int sx9385_check_pmic_setup_complete(struct device_node *dNode, int ic_num)
{
	struct regulator *dvdd_vreg = NULL;	/* regulator */
	char *dvdd_vreg_name = NULL;	/* regulator name */

	if (of_property_read_string_index(dNode, "sx9385,dvdd_vreg_name", 0, (const char **)&dvdd_vreg_name))
		dvdd_vreg_name = NULL;

	if (dvdd_vreg_name) {
		if (dvdd_vreg == NULL) {
			dvdd_vreg = regulator_get_optional(NULL, dvdd_vreg_name);
			if (IS_ERR(dvdd_vreg))
				return -EPROBE_DEFER;

			pr_info("[GRIP_%s] %s regulator_is_enabled %d\n", grip_name[ic_num], __func__, regulator_is_enabled(dvdd_vreg));
		}
	}

	return 0;
}

#ifdef CONFIG_SENSORS_FOLDABLE_SMD
static bool sx9385_is_smd_dependency(struct device *dev, int ic_num)
{
	int upper_c2c_det = -1;
	int gpio_level = 0;	// low:Set, high:SMD

	upper_c2c_det = of_get_named_gpio(dev->of_node, "upper-c2c-det-gpio", 0);

	if (gpio_is_valid(upper_c2c_det)) {
		gpio_level = gpio_get_value(upper_c2c_det);

		if (gpio_level) {
			pr_info("[GRIP_%s] %s upper-c2c-det-gpio %d(SMD)\n", grip_name[ic_num], __func__, upper_c2c_det);
			return true;
		}
	}

	return false;
}
#endif

static int sx9385_power_setting(struct device *dev, int ic_num)
{
	struct device_node *dNode = dev->of_node;
	int ret = 0;

#ifdef CONFIG_SENSORS_FOLDABLE_SMD
	if (sx9385_is_smd_dependency(dev, ic_num))
		return 0;
#endif

	vdd_retry_cnt++;
	if (vdd_retry_cnt > MAX_RETRY_CNT)
		return ret;

	if (vdd_retry_cnt && !(vdd_retry_cnt % 50))
		pr_info("[GRIP_%s] %s vdd retry (%d)\n", grip_name[ic_num], __func__, vdd_retry_cnt);

	ret = sx9385_configure_gpio_ldo(dNode, ic_num);
	if (ret < 0)
		return ret;

	ret = sx9385_check_pmic_setup_complete(dNode, ic_num);
	if (ret < 0)
		return ret;

	usleep_range(10000, 11000);

	return 0;
}

static int sx9385_parse_dt(struct sx9385_p *data, struct device *dev)
{
	struct device_node *dNode = dev->of_node;
	int ret = 0;

	if (dNode == NULL)
		return -ENODEV;

	data->gpio_nirq = of_get_named_gpio(dNode, "sx9385,nirq-gpio", 0);

	if (data->gpio_nirq < 0) {
		GRIP_ERR("get gpio_nirq err\n");
		return -ENODEV;
	}
	GRIP_INFO("get gpio_nirq %d\n", data->gpio_nirq);

	ret = of_property_read_u32(dNode, "sx9385,set_up_channels", &data->set_up_channels);
	if (ret < 0) {
		data->set_up_channels = 0;
		GRIP_ERR("error setup channel\n");
	} else {
		GRIP_INFO("set_up_channels %d\n", data->set_up_channels);
	}

	ret = of_property_read_u32(dNode, "sx9385,unknown_sel", &data->unknown);
	GRIP_INFO("unknown_sel %d\n", data->unknown);
	if (ret < 0) {
		GRIP_ERR("unknown_sel read fail %d\n", ret);
		data->unknown = 3;
	}
	//load register settings
	ret = of_property_read_u32(dNode, "sx9385,num_of_channels", &data->num_of_channels);
	if (ret < 0) {
		GRIP_ERR("get num_of_channels fail %d\n", ret);
		data->num_of_channels = 1;
	}
	ret = of_property_read_u32(dNode, "sx9385,num_of_refs", &data->num_of_refs);
	if (ret < 0) {
		GRIP_ERR("get num_of_refs fail %d\n", ret);
		data->num_of_refs = 1;
	}

	//load register settings
	of_property_read_u32(dNode, "sx9385,reg-num", &data->num_regs);
	GRIP_INFO("number of registers= %d\n", data->num_regs);

	if (unlikely(data->num_regs <= 0)) {
		GRIP_ERR("Invalid reg_num= %d\n", data->num_regs);
		return -EINVAL;
	} else {
		// initialize platform reg data array
		data->regs_addr_val = kzalloc(sizeof(struct reg_addr_val_s) * data->num_regs, GFP_KERNEL);
		if (unlikely(!data->regs_addr_val)) {
			GRIP_ERR("Failed to alloc memory, num_reg= %d\n", data->num_regs);
			return -ENOMEM;
		}

		if (of_property_read_u8_array(dNode, "sx9385,reg-init",
								(u8 *)data->regs_addr_val,
								data->num_regs * 2)) {
			GRIP_ERR("Failed to load registers from the dts\n");
			return -EINVAL;
		}
	}
	return 0;
}

#if (IS_ENABLED(CONFIG_PDIC_NOTIFIER) && IS_ENABLED(CONFIG_USB_TYPEC_MANAGER_NOTIFIER)) ||\
	IS_ENABLED(CONFIG_HALL_NOTIFIER) ||\
	(IS_ENABLED(CONFIG_SENSORS_SUPPROT_PANEL_NOTIFIER) && IS_ENABLED(CONFIG_SEC_PANEL_NOTIFIER_V2))

#if !IS_ENABLED(CONFIG_TABLET_MODEL_CONCEPT)
static int sx9385_enter_unknown_mode_to_valid_channels(struct sx9385_p *data, int type)
{
	bool is_valid_channel = false;
	int i;

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (sx9385_is_unavailable_channel(data->ch[i]))
			continue;

		is_valid_channel = true;
		enter_unknown_mode(data, type, data->ch[i]);
	}
	return is_valid_channel;
}
#endif //!CONFIG_TABLET_MODEL_CONCEPT

static bool sx9385_check_event(struct sx9385_p *data,
				unsigned long action, unsigned long expected_action)
{
	GRIP_INFO("action %ld, expected_action %ld", action, expected_action);

	return action != expected_action;
}

static int sx9385_handle_event_for_common_notifier(struct sx9385_p *data, unsigned long action,
				unsigned long expected_action, void *event_data, int event_type)
{
	mutex_lock(&data->notifier_mutex);

	if (!event_data) {
		mutex_unlock(&data->notifier_mutex);
		return 0;
	}

	if (sx9385_check_event(data, action, expected_action)) {
		mutex_unlock(&data->notifier_mutex);
		return 0;
	}
#if !IS_ENABLED(CONFIG_TABLET_MODEL_CONCEPT)
	if (!sx9385_enter_unknown_mode_to_valid_channels(data, event_type)) {
		mutex_unlock(&data->notifier_mutex);
		return 0;
	}
#endif //!CONFIG_TABLET_MODEL_CONCEPT
	switch (event_type) {
#if IS_ENABLED(CONFIG_PDIC_NOTIFIER) && IS_ENABLED(CONFIG_USB_TYPEC_MANAGER_NOTIFIER)
	case TYPE_USB:
		sx9385_handle_usb_typec_event(data, event_data);
		break;
#endif
#if IS_ENABLED(CONFIG_HALL_NOTIFIER)
	case TYPE_HALL:
		sx9385_handle_hall_event(data, event_data);
		break;
#endif
#if IS_ENABLED(CONFIG_SENSORS_SUPPROT_PANEL_NOTIFIER) && IS_ENABLED(CONFIG_SEC_PANEL_NOTIFIER_V2)
	case TYPE_PANEL:
		sx9385_handle_panel_event(data, event_data);
		break;
#endif
#if IS_ENABLED(CONFIG_TABLET_MODEL_CONCEPT)
#if IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V3) || IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V2) || IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO)
	case TYPE_POGO:
		sx9385_handle_pogo_event(data, event_data);
		break;
#endif
#endif
	default:
		break;
	}

	mutex_unlock(&data->notifier_mutex);

	return 0;
}
#endif

#if IS_ENABLED(CONFIG_PDIC_NOTIFIER) && IS_ENABLED(CONFIG_USB_TYPEC_MANAGER_NOTIFIER)
static void sx9385_handle_usb_typec_event(struct sx9385_p *data, void *event_data)
{
	PD_NOTI_ATTACH_TYPEDEF *usb_typec_info = event_data;

#if IS_ENABLED(CONFIG_TABLET_MODEL_CONCEPT)
	return;
#endif //CONFIG_TABLET_MODEL_CONCEPT

	if (data->pre_attach == usb_typec_info->attach)
		return;

	GRIP_INFO("src %d id %d attach %d rprd %d\n",
		usb_typec_info->src, usb_typec_info->id,
		usb_typec_info->attach, usb_typec_info->rprd);

	sx9385_schedule_work(&data->cal_work);
	data->pre_attach = usb_typec_info->attach;
}

__visible_for_testing
int sx9385_pdic_handle_notification(struct notifier_block *nb,
					   unsigned long action, void *pdic_data)
{
	struct sx9385_p *data = container_of(nb, struct sx9385_p, noti_block.pdic_nb);

	return sx9385_handle_event_for_common_notifier(data, action,
					PDIC_NOTIFY_ID_ATTACH, pdic_data, TYPE_USB);
}
#endif

#if IS_ENABLED(CONFIG_HALL_NOTIFIER)
static void sx9385_handle_hall_event(struct sx9385_p *data, void *event_data)
{
	struct hall_notifier_context *hall_notifier = event_data;

	GRIP_INFO("%s attach\n", hall_notifier->name);
#if IS_ENABLED(CONFIG_TABLET_MODEL_CONCEPT)
	if (strncmp(hall_notifier->name, "certify_hall", sizeof("certify_hall") - 1) == 0 ||
		strncmp(hall_notifier->name, "hall_wacom", sizeof("hall_wacom") - 1) == 0) {
		sx9385_schedule_work(&data->cal_work);
		GRIP_INFO("reset only without unknown, %s\n", hall_notifier->name);
	} else
		GRIP_ERR("%s reset skip, hall_notifier_name\n", hall_notifier->name);
#else //CONFIG_TABLET_MODEL_CONCEPT
	sx9385_schedule_work(&data->cal_work);
#endif //CONFIG_TABLET_MODEL_CONCEPT
}

__visible_for_testing int sx9385_hall_notifier(struct notifier_block *nb,
				unsigned long action, void *hall_data)
{
	struct sx9385_p *data =
			container_of(nb, struct sx9385_p, noti_block.hall_nb);

	return sx9385_handle_event_for_common_notifier(data, action,
					HALL_ATTACH, hall_data, TYPE_HALL);
}
#endif

#if IS_ENABLED(CONFIG_TABLET_MODEL_CONCEPT)
#if IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V3) || IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V2) || IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO)
static int sx9385_pogo_notifier(struct notifier_block *nb,
		unsigned long action, void *pogo_data)
{
	struct sx9385_p *data = container_of(nb, struct sx9385_p, noti_block.pogo_nb);

	return sx9385_handle_event_for_common_notifier(data, action,
					POGO_NOTIFIER_ID_ATTACHED, pogo_data, TYPE_POGO);

}

static void sx9385_handle_pogo_event(struct sx9385_p *data, void *event_data)
{
	GRIP_INFO("pogo attach\n");
	sx9385_schedule_work(&data->cal_work);
}
#endif
#endif //CONFIG_TABLET_MODEL_CONCEPT

#if IS_ENABLED(CONFIG_SENSORS_SUPPROT_PANEL_NOTIFIER) && IS_ENABLED(CONFIG_SEC_PANEL_NOTIFIER_V2)
static void sx9385_handle_panel_event(struct sx9385_p *data, void *event_data)
{
	struct panel_notifier_event_data *panel_notifier = event_data;

	int reg_val = sx9385_get_register_value_for_panel_state(data, panel_notifier);

	if (reg_val < 0)
		return;

	sx9385_write_register_value_for_panel_state(data, reg_val, panel_notifier);
}

static int sx9385_get_register_value_for_panel_state(struct sx9385_p *data, struct panel_notifier_event_data *panel_notifier)
{
	u32 reg_val;

	if (panel_notifier->state == PANEL_EVENT_PANEL_STATE_ON) {
		reg_val = PANEL_ON_REGISTER_VALUE;
	} else if (panel_notifier->state == PANEL_EVENT_PANEL_STATE_OFF) {
		reg_val = PANEL_OFF_REGISTER_VALUE;
	} else {
		GRIP_ERR("Invalid panel state %d\n", panel_notifier->state);
		return -EINVAL;
	}
	GRIP_INFO("get_register_value : %d\n", reg_val);
	return reg_val;
}

static int sx9385_write_register_value_for_panel_state(struct sx9385_p *data, u32 reg_val, struct panel_notifier_event_data *panel_notifier)
{
	u16 reg_addr[] = {REG_AFE_PARAM0_PH2, REG_AFE_PARAM0_PH4};
	int i, ret = 0;

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (sx9385_is_unavailable_channel(data->ch[i]))
			continue;

		ret = sx9385_i2c_write_retry(data, reg_addr[i], reg_val);
		if (ret < 0) {
			GRIP_ERR("Failed to update register 0x%x to 0x%X on panel %d\n",
					reg_addr[i], reg_val, panel_notifier->state);
			enter_error_mode(data, FAIL_SETUP_REGISTER);
			return 0;
		}

		GRIP_INFO("Updated register 0x%x to 0x%X on panel %d\n",
				reg_addr[i], reg_val, panel_notifier->state);
	}
	return ret;
}

__visible_for_testing
int sx9385_panel_notifier(struct notifier_block *nb,
				unsigned long action, void *panel_data)
{
	struct sx9385_p *data =	container_of(nb, struct sx9385_p, noti_block.panel_nb);

	return sx9385_handle_event_for_common_notifier(data, action,
					PANEL_EVENT_PANEL_STATE_CHANGED, panel_data, TYPE_PANEL);
}
#endif

static void sx9385_init_notifier(struct sx9385_p *data)
{
#if IS_ENABLED(CONFIG_PDIC_NOTIFIER) && IS_ENABLED(CONFIG_USB_TYPEC_MANAGER_NOTIFIER)
	GRIP_INFO("regi pdic notifier\n");
	manager_notifier_register(&data->noti_block.pdic_nb,
				  sx9385_pdic_handle_notification,
				  MANAGER_NOTIFY_PDIC_SENSORHUB);
#endif
#if IS_ENABLED(CONFIG_HALL_NOTIFIER)
	GRIP_INFO("regi hall notifier\n");
	data->noti_block.hall_nb.priority = 1;
	data->noti_block.hall_nb.notifier_call = sx9385_hall_notifier;
	hall_notifier_register(&data->noti_block.hall_nb);
#endif
#if IS_ENABLED(CONFIG_SENSORS_SUPPROT_PANEL_NOTIFIER) && IS_ENABLED(CONFIG_SEC_PANEL_NOTIFIER_V2)
	GRIP_INFO("regi panel notifier\n");
	data->noti_block.panel_nb.priority = 1;
	data->noti_block.panel_nb.notifier_call = sx9385_panel_notifier;
	panel_notifier_register(&data->noti_block.panel_nb);
#endif
#if defined(CONFIG_TABLET_MODEL_CONCEPT)
#if IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V3) || IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO_V2) || IS_ENABLED(CONFIG_KEYBOARD_STM32_POGO)
	pogo_notifier_register(&data->noti_block.pogo_nb, isg6320_pogo_notifier, POGO_NOTIFY_DEV_SENSOR);
#endif
#endif
}

static void sx9385_init_work_func(struct work_struct *work)
{
	struct sx9385_p *data = container_of((struct delayed_work *)work,
						 struct sx9385_p, init_work);
	int ret;

	if (data && !data->check_abnormal_working) {
		GRIP_INFO("initialize\n");

		sx9385_initialize_register(data);

		sx9385_set_mode(data, SX9385_MODE_NORMAL);

		if (!data->check_abnormal_working) {
			msleep(20);
			sx9385_manual_offset_calibration(data);
		}

		ret = sx9385_read_reg_stat(data);
		if (ret)
			GRIP_ERR("read reg stat : %d\n", ret);

		sx9385_get_gain(data);
		sx9385_init_notifier(data);
	}
}

static int get_channel_attr_size(int ch_idx)
{
	int channel_attr_size[] = {
		sizeof(grip_ch1_sensor_attrs),
		sizeof(grip_ch2_sensor_attrs),
	};
	return channel_attr_size[ch_idx];
}

static int sx9385_create_sysfs_attributes(struct sx9385_p *data)
{
	struct device_attribute **ch_sensor_attrs[] = {
		grip_ch1_sensor_attrs,
		grip_ch2_sensor_attrs,
	};
	int grip_sensor_attr_size =  sizeof(grip_common_sensor_attrs) / sizeof(ssize_t *);
	int ch_size = 0, offset = 0;
	int i, ret = 0;

	memcpy(data->sensor_attributes, grip_common_sensor_attrs, sizeof(grip_common_sensor_attrs));
	offset = grip_sensor_attr_size - 1;

	for (i = 0; i < NUM_OF_MAX_CHANNEL; i++) {
		if (sx9385_is_unavailable_channel(data->ch[i]))
			continue;

		ch_size = get_channel_attr_size(i) / sizeof(ssize_t *);

		if (ch_size + grip_sensor_attr_size > SENSOR_ATTR_SIZE) {
			GRIP_ERR("fail mem size of sensor_attr is exceeded size %d, %d\n", ch_size, grip_sensor_attr_size);
			return -ENOMEM;
		}
		memcpy(data->sensor_attributes + offset, ch_sensor_attrs[i], get_channel_attr_size(i));
		offset = offset + ch_size - 1;
		grip_sensor_attr_size = grip_sensor_attr_size + ch_size - 1;
	}
#if IS_ENABLED(CONFIG_SENSORS_REGION_DEPENDENCY)
	sx9385_grip_mounted_register(data, offset);
#endif
	ret = sensors_register(&data->factory_device, data, data->sensor_attributes,
				(char *)module_name[data->ic_num]);

	return ret;
}

static bool sx9385_is_unset_channel(u8 set_up_channels, int ch_idx)
{
	int channel = 1 << ch_idx;

	return !(set_up_channels & channel);
}

static void exception_set_up_channels(struct sx9385_p *data)
{
	if (data->set_up_channels == 0) {
		if (data->num_of_channels == 1) {
			data->set_up_channels = CH1_ONLY;
		} else if (data->num_of_channels == 2) {
			data->set_up_channels = CH_BOTH;
		}
	}
}

static int sx9385_setup_channels(struct sx9385_p *data)
{
	int ch_idx;

//exception_set_up_channels : After branching out of A16 and M16 models, the code will be deleted
	exception_set_up_channels(data);

	for (ch_idx = 0; ch_idx < NUM_OF_MAX_CHANNEL; ch_idx++) {
		if (sx9385_is_unset_channel(data->set_up_channels, ch_idx)) {
			GRIP_ERR("%dch skip mem alloc\n", ch_idx + 1);
			data->ch[ch_idx] = NULL;
			continue;
		}
		data->ch[ch_idx] = kzalloc(sizeof(struct channel), GFP_KERNEL);
		if (data->ch[ch_idx] == NULL) {
			GRIP_ERR("%ch Fail to channel mem alloc\n", ch_idx + 1);
			return -ENOMEM;
		}
	}

	return SUCCESS;
}

static void sx9385_free_channel_mem(struct sx9385_p *data)
{
	int ch_idx;

	for (ch_idx = 0; ch_idx < NUM_OF_MAX_CHANNEL; ch_idx++) {
		if (sx9385_is_unavailable_channel(data->ch[ch_idx]))
			continue;
		kfree(data->ch[ch_idx]);
	}
}
#if IS_ENABLED(CONFIG_SENSORS_REGION_DEPENDENCY)
static int sx9385_regional_init(struct device *dev, int ic_num)
{
	int grip_disable_gpio;
	int ret = 0;
	struct pinctrl *p;
	struct pinctrl_state *state;

	grip_disable_gpio = of_get_named_gpio(dev->of_node, "grip-disable-gpio", 0);

	pr_info("[GRIP_%s] grip disable gpio pin num : %d\n", grip_name[ic_num], grip_disable_gpio);

	if (gpio_get_value(grip_disable_gpio) == 0) {

		pr_info("[GRIP_%s] grip disable gpio low\n", grip_name[ic_num]);

		is_grip_mounted = false;

		p = pinctrl_get(dev);
		if (IS_ERR_OR_NULL(p))
			pr_info("[GRIP_%s] fail get pinctrl address\n", grip_name[ic_num]);

		state = pinctrl_lookup_state(p, "pulldown");
		if (IS_ERR_OR_NULL(state)) {
			pr_info("[GRIP_%s] could not find pull down state\n", grip_name[ic_num]);
		} else {
			ret = pinctrl_select_state(p, state);
			if (ret)
				pr_info("[GRIP_%s] Pinctrl state selection fail\n", grip_name[ic_num]);
		}

		ret = sensors_register(&dev, NULL, is_grip_mounted_attrs, (char *)module_name[ic_num]);
		if (ret < 0)
			pr_info("[GRIP_%s] sensors_register fail\n", grip_name[ic_num]);

		return HAS_ERROR;
	}

	return SUCCESS;
}
#endif

#if (KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE)
static int sx9385_probe(struct i2c_client *client)
#else
static int sx9385_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
	struct sx9385_p *data = NULL;
	enum ic_num ic_num;
	int ret;

	ic_num = (enum ic_num) of_device_get_match_data(&client->dev);
#if IS_ENABLED(CONFIG_SENSORS_REGION_DEPENDENCY)
	ret = sx9385_regional_init(&client->dev, ic_num);
	if (ret < 0) {
		pr_info("[GRIP_%s] probe fail grip not mount\n", grip_name[ic_num]);
		return ret;
	}
#endif

	ret = sx9385_power_setting(&client->dev, ic_num);
	if (ret < 0)
		return ret;

	pr_info("[GRIP_%s] %s start 0x%x\n", grip_name[ic_num], __func__, client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_info("[GRIP_%s] i2c func err\n", grip_name[ic_num]);
		goto exit;
	}

	data = kzalloc(sizeof(struct sx9385_p), GFP_KERNEL);
	if (data == NULL) {
		pr_info("[GRIP_%s] Fail to mem alloc\n", grip_name[ic_num]);
		ret = -ENOMEM;
		goto exit_kzalloc;
	}

	data->ic_num = ic_num;
	i2c_set_clientdata(client, data);
	data->client = client;
	data->factory_device = &client->dev;
	data->grip_ws = wakeup_source_register(&client->dev, "grip_wake_lock");
	mutex_init(&data->mode_mutex);
	mutex_init(&data->read_mutex);
	mutex_init(&data->cal_mutex);
	mutex_init(&data->notifier_mutex);

	ret = sx9385_parse_dt(data, &client->dev);
	if (ret < 0) {
		GRIP_ERR("of_node err\n");
		ret = -ENODEV;
		goto exit_of_node;
	}

	if (sx9385_setup_channels(data))
		goto exit_channel_kmalloc;

	ret = sx9385_input_init(data);
	if (ret < 0)
		goto exit_input_init;

	ret = sx9385_noti_input_init(data);
	if (ret < 0)
		goto exit_noti_input_init;

	ret = sx9385_setup_pin(data);
	if (ret) {
		GRIP_ERR("could not setup pin\n");
		goto exit_setup_pin;
	}

	/* read chip id */
	ret = sx9385_hardware_check(data);
	if (ret < 0) {
		GRIP_ERR("chip id check fail %d\n", ret);
		goto exit_chip_reset;
	}

	sx9385_initialize_variable(data);
	INIT_WORK(&data->cal_work, sx9385_cal_work_func);
	INIT_DELAYED_WORK(&data->init_work, sx9385_init_work_func);
	INIT_DELAYED_WORK(&data->irq_work, sx9385_irq_work_func);
	INIT_DELAYED_WORK(&data->debug_work, sx9385_debug_work_func);

	data->irq = gpio_to_irq(data->gpio_nirq);
	/* initailize interrupt reporting */
	ret = request_threaded_irq(data->irq, NULL, sx9385_interrupt_thread,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					device_name[data->ic_num], data);
	if (ret < 0) {
		GRIP_ERR("fail to req thread irq %d ret %d\n", data->irq, ret);
		goto exit_request_threaded_irq;
	}
	disable_irq(data->irq);
	data->is_irq_active = false;

	if (sx9385_create_sysfs_attributes(data))
		goto exit_register_failed;

	schedule_delayed_work(&data->init_work, msecs_to_jiffies(300));
	sx9385_set_debug_work(data, ON, 20000);

	sx9385_get_revision(data);

	GRIP_INFO("done!\n");

	return 0;

exit_register_failed:
	free_irq(data->irq, data);
exit_request_threaded_irq:
exit_chip_reset:
	gpio_free(data->gpio_nirq);
exit_setup_pin:
	remove_shared_noti_input_dev(data);
exit_noti_input_init:
	sysfs_remove_group(&data->input->dev.kobj, &sx9385_attribute_group);
#if IS_ENABLED(CONFIG_SENSORS_CORE_AP)
	sensors_remove_symlink(&data->input->dev.kobj, data->input->name);
#else
	sensors_remove_symlink(data->input);
#endif
	input_unregister_device(data->input);
exit_input_init:
exit_channel_kmalloc:
	sx9385_free_channel_mem(data);
exit_of_node:
	mutex_destroy(&data->mode_mutex);
	mutex_destroy(&data->read_mutex);
	mutex_destroy(&data->cal_mutex);
	mutex_destroy(&data->notifier_mutex);
	wakeup_source_unregister(data->grip_ws);
	kfree(data);
exit_kzalloc:
exit:
	sx9385_destroy_noti_lock();
	pr_err("[GRIP_%s] Probe fail!\n", grip_name[ic_num]);
	return ret;

}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static void sx9385_remove(struct i2c_client *client)
#else
static int sx9385_remove(struct i2c_client *client)
#endif
{
	struct sx9385_p *data = (struct sx9385_p *)i2c_get_clientdata(client);

	if (atomic_read(&data->enable) == ON)
		sx9385_set_enable(data, OFF);

	sx9385_set_mode(data, SX9385_MODE_SLEEP);

	cancel_delayed_work_sync(&data->init_work);
	cancel_delayed_work_sync(&data->irq_work);
	cancel_delayed_work_sync(&data->debug_work);
	free_irq(data->irq, data);
	gpio_free(data->gpio_nirq);

	wakeup_source_unregister(data->grip_ws);
	sensors_unregister(data->factory_device, data->sensor_attributes);
#if IS_ENABLED(CONFIG_SENSORS_CORE_AP)
	sensors_remove_symlink(&data->input->dev.kobj, data->input->name);
#else
	sensors_remove_symlink(data->input);
#endif
	sysfs_remove_group(&data->input->dev.kobj, &sx9385_attribute_group);
	input_unregister_device(data->input);
	remove_shared_noti_input_dev(data);
	mutex_destroy(&data->mode_mutex);
	mutex_destroy(&data->read_mutex);
	mutex_destroy(&data->cal_mutex);
	mutex_destroy(&data->notifier_mutex);

	kfree(data);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
	return;
#else
	return 0;
#endif
}

static int sx9385_suspend(struct device *dev)
{
	struct sx9385_p *data = dev_get_drvdata(dev);
	int cnt = 0;

	GRIP_INFO("\n");
	data->is_suspend = true;
	/* before go to sleep, make the interrupt pin as high*/
	while ((sx9385_get_nirq_state(data) == INTERRUPT_LOW) && (cnt++ < 3)) {
		sx9385_read_reg_stat(data);
		msleep(20);
	}
	if (cnt >= 3)
		GRIP_ERR("s/w reset fail(%d)\n", cnt);

	sx9385_set_debug_work(data, OFF, 1000);
#if IS_ENABLED(CONFIG_SENSORS_IRQ_PROCESSING_DELAY)
	if (data->is_irq_active)
		disable_irq(data->irq);
#endif
	return 0;
}
static int sx9385_resume(struct device *dev)
{
	struct sx9385_p *data = dev_get_drvdata(dev);

	GRIP_INFO("\n");
	data->is_suspend = false;
	sx9385_set_debug_work(data, ON, 1000);
#if IS_ENABLED(CONFIG_SENSORS_IRQ_PROCESSING_DELAY)
	if (data->is_irq_active)
		enable_irq(data->irq);
#endif
	return 0;
}

static void sx9385_shutdown(struct i2c_client *client)
{
	struct sx9385_p *data = i2c_get_clientdata(client);

	GRIP_INFO("\n");
	sx9385_set_debug_work(data, OFF, 1000);
	if (atomic_read(&data->enable) == ON)
		sx9385_set_enable(data, OFF);

	sx9385_set_mode(data, SX9385_MODE_SLEEP);
}

static const struct of_device_id sx9385_match_table[] = {
#if IS_ENABLED(CONFIG_SENSORS_SX9385)
	{ .compatible = "sx9385", .data = (void *)MAIN_GRIP},
#endif
#if IS_ENABLED(CONFIG_SENSORS_SX9385_SUB)
	{ .compatible = "sx9385_sub", .data = (void *)SUB_GRIP},
#endif
#if IS_ENABLED(CONFIG_SENSORS_SX9385_SUB2)
	{ .compatible = "sx9385_sub2", .data = (void *)SUB2_GRIP},
#endif
#if IS_ENABLED(CONFIG_SENSORS_SX9385_WIFI)
	{ .compatible = "sx9385_wifi", .data = (void *)WIFI_GRIP},
#endif
	{},
};

static const struct i2c_device_id sx9385_id[] = {
	{ "SX9385", 0 },
	{ }
};

static const struct dev_pm_ops sx9385_pm_ops = {
	.suspend = sx9385_suspend,
	.resume = sx9385_resume,
};

static struct i2c_driver sx9385_driver = {
	.driver = {
		.name	= "SX9385",
		.owner	= THIS_MODULE,
		.of_match_table = sx9385_match_table,
		.pm = &sx9385_pm_ops
	},
	.probe		= sx9385_probe,
	.remove		= sx9385_remove,
	.shutdown	= sx9385_shutdown,
	.id_table	= sx9385_id,
};

static void grip_module_count(void)
{
#if IS_ENABLED(CONFIG_SENSORS_SX9385)
	sx9385_module_count++;
#endif
#if IS_ENABLED(CONFIG_SENSORS_SX9385_SUB)
	sx9385_module_count++;
#endif
#if IS_ENABLED(CONFIG_SENSORS_SX9385_WIFI)
	sx9385_module_count++;
#endif
#if IS_ENABLED(CONFIG_SENSORS_SX9385_SUB2)
	sx9385_module_count++;
#endif
}

static int __init sx9385_init(void)
{
	int ret;

	pr_info("[GRIP] sx9385_driver init\n");

	mutex_init(&noti_lock);
	grip_module_count();

	ret = i2c_add_driver(&sx9385_driver);
	if (ret)
		pr_err("[GRIP] sx9385_driver probe fail\n");

	return ret;

}

static void __exit sx9385_exit(void)
{
	i2c_del_driver(&sx9385_driver);
}

module_init(sx9385_init);
module_exit(sx9385_exit);

MODULE_DESCRIPTION("Semtech Corp. SX9380 Capacitive Touch Controller Driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
