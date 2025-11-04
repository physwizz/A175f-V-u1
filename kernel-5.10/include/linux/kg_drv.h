/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_KG_DRV_H
#define _LINUX_KG_DRV_H

#include <linux/notifier.h>
#include <linux/types.h>
#include <linux/errno.h>

enum {
	KG_RESOURCE_MEMORY_WEAK_BIT,
	KG_RESOURCE_MEMORY_STRONG_BIT,
	KG_RESOURCE_CAMERA_BIT,
	__MAX_NR_RESOURCES
};

#define KG_RESOURCE_MEMORY_WEAK ((1UL << KG_RESOURCE_MEMORY_WEAK_BIT))
#define KG_RESOURCE_MEMORY_STRONG ((1UL << KG_RESOURCE_MEMORY_STRONG_BIT))
#define KG_RESOURCE_CAMERA ((1UL << KG_RESOURCE_CAMERA_BIT))
#define KG_RESOURCES_MASK ((1UL << __MAX_NR_RESOURCES) - 1)

#if IS_ENABLED(CONFIG_KG_DRV)
extern int kg_resource_control_notifier_register(struct notifier_block *n);
extern int kg_resource_control_notifier_unregister(struct notifier_block *n);
extern bool kg_resource_is_locked(unsigned long kg_resource);
#else
static inline int kg_resource_control_notifier_register(struct notifier_block *n) { return -ENODEV; }
static inline int kg_resource_control_notifier_unregister(struct notifier_block *n) { return -ENODEV; }
static inline bool kg_resource_is_locked(unsigned long kg_resource) { return false; }
#endif

#endif	/* _LINUX_KG_DRV_H */
