// SPDX-License-Identifier: GPL-2.0
/*
 * kg_ta_qsee.c/
 *
 * Copyright (C) 2025 Samsung Electronics
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/qseecom_kernel.h>

#include "kg_ta_call.h"
#include "kg_log.h"

static int kg_ta_initialize(struct qseecom_handle **handle)
{
	int ret;

	ret = qseecom_start_app(handle, "tz_kg", TA_BUFFER_TOTAL_SIZE);
	if (ret)
		KG_LOG("%s failed to qseecom_start_app 0x%x\n", __func__, ret);

	return ret;
}

static int kg_ta_invoke_cmd(struct qseecom_handle *handle, uint32_t cmd)
{
	int ret;
	tz_kernel_msg_header_t *reqmsg_header = (tz_kernel_msg_header_t *)handle->sbuf;
	tz_kernel_msg_header_t *respmsg_header = (tz_kernel_msg_header_t *)(handle->sbuf + TA_REQ_BUFFER_SIZE);
	uint32_t status;

	reqmsg_header->content_id = KG_CMD_PROTOCOL_KERNEL;
	reqmsg_header->id = cmd;
	reqmsg_header->len = TA_REQ_BUFFER_SIZE;
	reqmsg_header->pid = current->pid;

	ret = qseecom_send_command(handle,
			reqmsg_header, TA_REQ_BUFFER_SIZE,
			respmsg_header, TA_REQ_BUFFER_SIZE);

	if (ret) {
		KG_LOG("%s: failed to qseecom_send_command 0x%x\n", __func__, ret);
	} else {
		/* return value from TA */
		status = respmsg_header->status;
		if (status)
			KG_LOG("%s cmd %d , TA err: 0x%x\n", __func__, cmd, status);
	}

	return ret;
}

static int kg_ta_finalize(struct qseecom_handle *handle)
{
	int ret = qseecom_shutdown_app(&handle);

	if (ret)
		KG_LOG("%s: failed to qseecom_shutdown_app 0x%x\n", __func__, ret);

	return ret;
}

int kg_ta_call(uint32_t cmd, kg_ta_call_ctx_t *ctx)
{
	struct qseecom_handle *qhandle;
	int ret, ret_fin;

	ret = kg_ta_initialize(&qhandle);
	if (ret)
		goto failed_kg_ta_initialize;

	/* caller write req buffer via hook function */
	if (ctx && ctx->kg_ta_call_input_hook && ctx->input)
		ctx->kg_ta_call_input_hook(ctx->input, qhandle->sbuf);

	ret = kg_ta_invoke_cmd(qhandle, cmd);

	/* caller read resp buffer via hook function */
	if (!ret && ctx && ctx->kg_ta_call_output_hook && ctx->output)
		ctx->kg_ta_call_output_hook(ctx->output, qhandle->sbuf + TA_REQ_BUFFER_SIZE);

	ret_fin = kg_ta_finalize(qhandle);

	/* if there is an error in the ret value of kg_ta_invoke_cmd, keep the value */
	if (!ret)
		ret = ret_fin;

failed_kg_ta_initialize:
	return ret;
}
