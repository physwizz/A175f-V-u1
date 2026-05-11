/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __KG_RESOURCE_CONTROL_H__
#define __KG_RESOURCE_CONTROL_H__

#include <linux/kg_drv.h>

#define KG_RESOURCE_DEVICE_REBOOT_SEC (60 * 60) // 1 hour

extern void __init kg_resource_control_init(unsigned long init_value);
extern void __init kg_resource_memory_init(void);

#if IS_ENABLED(CONFIG_KG_RESOURCE_CPU)
extern void __init kg_resource_cpu_init(void);
#else
static inline void kg_resource_cpu_init(void) { }
#endif

extern void kg_resource_control_notifier_call(unsigned long value);

#endif /* __KG_RESOURCE_CONTROL_H__ */
