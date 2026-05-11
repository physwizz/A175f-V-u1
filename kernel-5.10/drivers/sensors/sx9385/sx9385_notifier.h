#ifndef SX9385_NOTIFIER_H
#define SX9385_NOTIFIER_H

#if IS_ENABLED(CONFIG_PDIC_NOTIFIER)
#include <linux/usb/typec/common/pdic_notifier.h>
#endif

#if IS_ENABLED(CONFIG_USB_TYPEC_MANAGER_NOTIFIER)
#include <linux/usb/typec/manager/usb_typec_manager_notifier.h>
#endif

#if IS_ENABLED(CONFIG_HALL_NOTIFIER)
#include <linux/hall/hall_ic_notifier.h>
#define HALL_NAME "hall"
#define HALL_CERT_NAME "certify_hall"
#define HALL_FLIP_NAME "flip"
#define HALL_ATTACH 1
#define HALL_DETACH 0
#endif

#if IS_ENABLED(CONFIG_SENSORS_SUPPROT_PANEL_NOTIFIER) && IS_ENABLED(CONFIG_SEC_PANEL_NOTIFIER_V2)
#include <linux/sec_panel_notifier_v2.h>
#endif

#endif
