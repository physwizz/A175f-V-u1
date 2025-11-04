/*
 * Copyright (c) 2020-2021 Samsung Electronics Co., Ltd. All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/version.h>
#include <net/genetlink.h>
#include "kg_netlink.h"
#include "kg_log.h"

static int is_kgsm_initialized;

static int kgsm_service_callback(struct sk_buff *skb,
					       struct genl_info *info);
static atomic_t kgservice_ready = ATOMIC_INIT(0);

static struct nla_policy kgsm_netlink_policy[KGSM_ATTR_COUNT + 1] = {
	[KGSM_VALUE] = { .type = NLA_U64 },
	[KGSM_FEATURE_CODE] = { .type = NLA_STRING, .len = FEATURE_CODE_LENGTH + 1},
	[KGSM_DETAIL] = { .type = NLA_STRING, .len = MAX_ALLOWED_DETAIL_LENGTH + 1},
	[KGSM_SERVICE_READY] = { .type = NLA_U32 },
};

static const struct genl_ops kgsm_kernel_ops[] = {
	{
		.cmd = KGSM_MSG_CMD,
		.doit = kgsm_service_callback,
	},
};

struct genl_multicast_group kgsm_group[] = {
	{
		.name = KGSM_GROUP,
	},
};

static struct genl_family kgsm_family = {
	.name = KGSM_FAMILY,
	.version = 1,
	.maxattr = KGSM_ATTR_MAX,
	.module = THIS_MODULE,
	.ops = kgsm_kernel_ops,
	.policy = kgsm_netlink_policy,
	.mcgrps = kgsm_group,
	.n_mcgrps = ARRAY_SIZE(kgsm_group),
	.n_ops = ARRAY_SIZE(kgsm_kernel_ops),
};

int __init kgsm_netlink_init(void)
{
	int ret;

	ret = genl_register_family(&kgsm_family);
	if (ret == KG_NETLINK_SUCCESS)
		is_kgsm_initialized = true;
	else
		KG_LOG("Netlink register failed: %d\n", ret);

	return ret;
}

void __exit kgsm_netlink_exit(void)
{
	int ret;

	ret = genl_unregister_family(&kgsm_family);
	is_kgsm_initialized = false;
	if (ret != KG_NETLINK_SUCCESS)
		KG_LOG("Netlink unregister failed: %d\n", ret);
}

static int kgsm_service_callback(struct sk_buff *skb,
		struct genl_info *info)
{
	if (atomic_add_unless(&kgservice_ready, 1, 1))
		KG_LOG("kg service ready\n");

	return KG_NETLINK_SUCCESS;
}

int kgsm_service_ready(void)
{
	return atomic_read(&kgservice_ready);
}

int kgsm_is_initialized(void)
{
	return (is_kgsm_initialized == true) ? true : false;
}

int kgsm_send_netlink_message(const char *feature_code, const char *detail, int64_t value)
{
	int ret;
	void *msg_head;
	size_t detail_len;
	struct sk_buff *skb;

	skb = genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
	if (skb == NULL) {
		KG_LOG("genlmsg_new error\n");
		return -ENOMEM;
	}

	msg_head = genlmsg_put(skb, 0, 0,
			&kgsm_family, 0, KGSM_MSG_CMD);
	if (msg_head == NULL) {
		KG_LOG("genlmsg_put error\n");
		nlmsg_free(skb);
		return -ENOMEM;
	}

	ret = nla_put(skb, KGSM_VALUE, sizeof(value), &value);
	if (ret) {
		KG_LOG("nla_put value error\n");
		nlmsg_free(skb);
		return ret;
	}

	ret = nla_put(skb, KGSM_FEATURE_CODE,
			FEATURE_CODE_LENGTH + 1, feature_code);
	if (ret) {
		KG_LOG("nla_put feature error\n");
		nlmsg_free(skb);
		return ret;
	}

	detail_len = strnlen(detail, MAX_ALLOWED_DETAIL_LENGTH);
	ret = nla_put(skb, KGSM_DETAIL, detail_len + 1, detail);
	if (ret) {
		KG_LOG("nla_put detail error\n");
		nlmsg_free(skb);
		return ret;
	}

	genlmsg_end(skb, msg_head);
	ret = genlmsg_multicast(&kgsm_family, skb, 0, 0, GFP_KERNEL);
	if (ret) {
		KG_LOG("genlmsg_multicast error\n");
		return ret;
	}

	return KG_NETLINK_SUCCESS;
}
