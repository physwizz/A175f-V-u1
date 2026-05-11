/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __PANEL_DEBUG_H__
#define __PANEL_DEBUG_H__

#include <linux/printk.h>
#if IS_ENABLED(CONFIG_SEC_DEBUG)
#include <linux/sec_debug.h>
#endif
#include <linux/kernel.h>
#include <linux/version.h>
#include "util.h"

extern int panel_log_level;
extern int panel_cmd_log;

#define USDM_PR_PREFIX		""
#define USDM_PR_FMT_NO_ID		"%s: %s:%d "
#define USDM_PR_ARG_NO_ID(name)	(name), __func__, __LINE__
#define USDM_PR_FMT		"%s[%d]: %s:%d "
#define USDM_PR_ARG(name, id)	(name), (id), __func__, __LINE__

#define usdm_pr_err_no_id(name, log_lv, fmt, ...)					\
	do {									\
		if ((log_lv) >= 3) 						\
			pr_err(USDM_PR_PREFIX USDM_PR_FMT_NO_ID fmt,			\
				USDM_PR_ARG_NO_ID((name)), ##__VA_ARGS__);	\
	} while (0)

#define usdm_pr_warn_no_id(name, log_lv, fmt, ...)					\
	do {									\
		if ((log_lv) >= 5) 						\
			pr_warn(USDM_PR_PREFIX USDM_PR_FMT_NO_ID fmt,			\
				USDM_PR_ARG_NO_ID((name)), ##__VA_ARGS__);	\
	} while (0)

#define usdm_pr_info_no_id(name, log_lv, fmt, ...)					\
	do {									\
		if ((log_lv) >= 6) 						\
			pr_info(USDM_PR_PREFIX USDM_PR_FMT_NO_ID fmt,			\
				USDM_PR_ARG_NO_ID((name)), ##__VA_ARGS__);	\
	} while (0)

#define usdm_pr_debug_no_id(name, log_lv, fmt, ...)				\
	do {									\
		if ((log_lv) >= 7) 						\
			pr_info(USDM_PR_PREFIX USDM_PR_FMT_NO_ID fmt,			\
				USDM_PR_ARG_NO_ID((name)), ##__VA_ARGS__);	\
	} while (0)

#define usdm_pr_err(name, id, log_lv, fmt, ...)					\
	do {									\
		if ((log_lv) >= 3) 						\
			pr_err(USDM_PR_PREFIX USDM_PR_FMT fmt,			\
				USDM_PR_ARG((name), (id)), ##__VA_ARGS__);	\
	} while (0)

#define usdm_pr_warn(name, id, log_lv, fmt, ...)					\
	do {									\
		if ((log_lv) >= 5) 						\
			pr_warn(USDM_PR_PREFIX USDM_PR_FMT fmt,			\
				USDM_PR_ARG((name), (id)), ##__VA_ARGS__);	\
	} while (0)

#define usdm_pr_info(name, id, log_lv, fmt, ...)					\
	do {									\
		if ((log_lv) >= 6) 						\
			pr_info(USDM_PR_PREFIX USDM_PR_FMT fmt,			\
				USDM_PR_ARG((name), (id)), ##__VA_ARGS__);	\
	} while (0)

#define usdm_pr_debug(name, id, log_lv, fmt, ...)				\
	do {									\
		if ((log_lv) >= 7) 						\
			pr_info(USDM_PR_PREFIX USDM_PR_FMT fmt,			\
				USDM_PR_ARG((name), (id)), ##__VA_ARGS__);	\
	} while (0)


#define PANEL_DRIVER_NAME "panel-drv"
#define panel_drv_info(_panel, fmt, ...)   \
	usdm_pr_info(PANEL_DRIVER_NAME, ((_panel) ? (_panel)->id : -1), panel_log_level, fmt, ##__VA_ARGS__)

#define panel_drv_warn(_panel, fmt, ...)   \
	usdm_pr_warn(PANEL_DRIVER_NAME, ((_panel) ? (_panel)->id : -1), panel_log_level, fmt, ##__VA_ARGS__)

#define panel_drv_err(_panel, fmt, ...)    \
	usdm_pr_err(PANEL_DRIVER_NAME, ((_panel) ? (_panel)->id : -1), panel_log_level, fmt, ##__VA_ARGS__)

#define panel_drv_dbg(_panel, fmt, ...)\
	usdm_pr_debug(PANEL_DRIVER_NAME, ((_panel) ? (_panel)->id : -1), panel_log_level, fmt, ##__VA_ARGS__)


#define panel_info(fmt, ...)							\
	usdm_pr_info_no_id(PANEL_DRIVER_NAME, panel_log_level, fmt, ##__VA_ARGS__)

#define panel_warn(fmt, ...)							\
	usdm_pr_warn_no_id(PANEL_DRIVER_NAME, panel_log_level, fmt, ##__VA_ARGS__)

#define panel_err(fmt, ...)							\
	usdm_pr_err_no_id(PANEL_DRIVER_NAME, panel_log_level, fmt, ##__VA_ARGS__)

#define panel_dbg(fmt, ...)							\
	usdm_pr_debug_no_id(PANEL_DRIVER_NAME, panel_log_level, fmt, ##__VA_ARGS__)

#define panel_cmd_log_enabled(_x_)	((panel_cmd_log) & (1 << (_x_)))

/*
 * debug level low return 0.
 * debug level mid or high return over 0.
 */
static inline int panel_debug_level(void)
{
#if IS_ENABLED(CONFIG_SEC_DEBUG)
#if IS_ENABLED(CONFIG_DRM_MEDIATEK_V2)
	return !is_debug_level_low();
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0)
	return sec_debug_get_force_upload();
#else
	return secdbg_mode_enter_upload();
#endif
#endif
#else
	return 1;
#endif
}

#define PANEL_BUG() \
	do { \
		if (!panel_debug_level()) \
			panel_err("PANEL_BUG detected\n"); \
		else \
			BUG(); \
	} while (0)

#define PANEL_BUG_ON(cond) do { if (unlikely(cond)) PANEL_BUG(); } while (0)

#endif /* __PANEL_DEBUG_H__ */
