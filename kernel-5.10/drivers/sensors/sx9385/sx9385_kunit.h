#ifndef SX9385_KUNIT_H
#define SX9385_KUNIT_H

#include "sx9385.h"

#if IS_ENABLED(CONFIG_SEC_KUNIT)
#include <kunit/mock.h>
#define __mockable __weak
#define __visible_for_testing
extern struct device_attribute *grip_ch1_sensor_attrs[];
__visible_for_testing DECLARE_REDIRECT_MOCKABLE(sx9385_calculate_threshold, RETURNS(u32), PARAMS(u8));
__visible_for_testing DECLARE_REDIRECT_MOCKABLE(sx9385_calculate_hyst, RETURNS(u32), PARAMS(u8, u32));
__visible_for_testing int sx9385_convert_input(int func_id, const char *buf, void *val1, void *val2);
__visible_for_testing int sx9385_calculate_coffset(u8 msb, u8 lsb);
__visible_for_testing int sx9385_calculate_cuseful(int use, int again, int dgain);
__visible_for_testing int sx9385_calculate_diff(int use, int avg);
__visible_for_testing int sx9385_calculate_avg(u8 msb, u8 mid, u8 lsb);
__visible_for_testing void sx9385_calculate_capMain_and_diff_data(struct sx9385_p *data, int i);
__visible_for_testing int sx9385_set_mode(struct sx9385_p *data, unsigned char mode);

#if IS_ENABLED(CONFIG_SENSORS_SUPPROT_PANEL_NOTIFIER) && IS_ENABLED(CONFIG_SEC_PANEL_NOTIFIER_V2)
__visible_for_testing
int sx9385_panel_notifier(struct notifier_block *nb, unsigned long action, void *panel_data);
#endif
#if IS_ENABLED(CONFIG_HALL_NOTIFIER)
__visible_for_testing
int sx9385_hall_notifier(struct notifier_block *nb, unsigned long action, void *hall_data);
#endif
#if IS_ENABLED(CONFIG_PDIC_NOTIFIER) && IS_ENABLED(CONFIG_USB_TYPEC_MANAGER_NOTIFIER)
__visible_for_testing
int sx9385_pdic_handle_notification(struct notifier_block *nb, unsigned long action, void *pdic_data);
#endif

#else
#include "sx9385_kunit_dummy.h"
#define __mockable
#define __visible_for_testing static
#endif

#endif
