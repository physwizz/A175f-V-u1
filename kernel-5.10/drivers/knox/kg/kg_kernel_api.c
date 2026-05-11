/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd. All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kmod.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/string.h>
#include "kg_kernel_api.h"
#include "kg_netlink.h"
#include "kg_log.h"

noinline int kgsm_send_message(const char *feature_code,
		const char *detail,
		int64_t value)
{
	int ret;
	size_t len;

	if (!kgsm_service_ready()) {
		ret = -EPERM;
		goto exit_send;
	}

	if (!kgsm_is_initialized()) {
		KG_LOG("KGSM not initialized\n");
		ret = -EACCES;
		goto exit_send;
	}

	if (!feature_code) {
		KG_LOG("Invalid feature code\n");
		ret = -EINVAL;
		goto exit_send;
	}

	if (!detail)
		detail = "";

	len = strnlen(detail, MAX_ALLOWED_DETAIL_LENGTH);

	ret = kgsm_send_netlink_message(feature_code, detail, value);
	if (ret == KG_NETLINK_SUCCESS) {
		KG_LOG("send netlink message success - {'%s', %zu bytes, %lld}\n",
				feature_code, len, value);
	}

exit_send:
	return ret;
}
