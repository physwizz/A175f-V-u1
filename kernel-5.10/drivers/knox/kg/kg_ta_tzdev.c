// SPDX-License-Identifier: GPL-2.0
/*
 * kg_ta_tzdev.c/
 *
 * Copyright (C) 2025 Samsung Electronics
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/vmalloc.h>

#include "tee_client_api.h"

#include "kg_ta_call.h"
#include "kg_log.h"

static TEEC_UUID kg_ta_uuid = {
	0x00000000, 0x0000, 0x0000, {0x00, 0x00, 0x6b, 0x6e, 0x78, 0x67, 0x75, 0x64}
};

typedef struct teegris_ta_call_handle {
	TEEC_Context *context;
	TEEC_SharedMemory *sharedMem;
	TEEC_Session *session;
	TEEC_Operation *operation;
} teegris_ta_call_handle_t;

static void free_memory_for_call_context(teegris_ta_call_handle_t *handle)
{
	if (handle->sharedMem) {
		if (handle->sharedMem->buffer) {
			vfree(handle->sharedMem->buffer);
			handle->sharedMem->buffer = NULL;
		}
	}
}

static TEEC_Result init_memory_for_call_context(teegris_ta_call_handle_t *handle)
{
	void *buffer;
	int buffer_size;

	memset(handle->context, 0, sizeof(*handle->context));
	memset(handle->sharedMem, 0, sizeof(*handle->sharedMem));
	memset(handle->session, 0, sizeof(*handle->session));
	memset(handle->operation, 0, sizeof(*handle->operation));

	buffer_size = TA_BUFFER_TOTAL_SIZE;

	buffer = vzalloc(buffer_size);
	if (!buffer) {
		KG_LOG("%s failed to vzalloc for sharedMem buffer\n", __func__);
		goto alloc_failed;
	}

	handle->sharedMem->size = buffer_size;
	handle->sharedMem->buffer = buffer;
	handle->sharedMem->flags = TEEC_MEM_INPUT | TEEC_MEM_OUTPUT;

	handle->operation->paramTypes = TEEC_PARAM_TYPES(
		TEEC_MEMREF_PARTIAL_INOUT, TEEC_MEMREF_PARTIAL_OUTPUT, TEEC_NONE, TEEC_NONE);

	handle->operation->params[0].memref.parent = handle->sharedMem;
	handle->operation->params[0].memref.size = buffer_size / 2;
	handle->operation->params[0].memref.offset = 0;
	handle->operation->params[1].memref.parent = handle->sharedMem;
	handle->operation->params[1].memref.size = buffer_size / 2;
	handle->operation->params[1].memref.offset = buffer_size / 2;

	return TEEC_SUCCESS;

alloc_failed:
	return TEEC_ERROR_OUT_OF_MEMORY;
}

static TEEC_Result kg_ta_initialize(teegris_ta_call_handle_t *handle, uint32_t *origin)
{
	TEEC_Result result = TEEC_SUCCESS;

	*origin = 0x0;

	result = init_memory_for_call_context(handle);
	if (result != TEEC_SUCCESS) {
		KG_LOG("%s failed to alloc_memory_for_call_context\n", __func__);
		goto failed_to_alloc_memory;
	}

	result = TEEC_InitializeContext(NULL, handle->context);
	if (result != TEEC_SUCCESS) {
		KG_LOG("%s failed to TEEC_InitializeContext result: 0x%x\n", __func__, result);
		goto free_memory;
	}

	result = TEEC_RegisterSharedMemory(handle->context, handle->sharedMem);
	if (result != TEEC_SUCCESS) {
		KG_LOG("%s failed to TEEC_RegisterSharedMemory result: 0x%x\n", __func__, result);
		goto finalize_context;
	}

	result = TEEC_OpenSession(handle->context, handle->session, &kg_ta_uuid, TEEC_LOGIN_PUBLIC,
			NULL, NULL, origin);
	if (result != TEEC_SUCCESS) {
		KG_LOG("%s failed to TEEC_OpenSession result: 0x%x , origin: 0x%x\n",
			__func__, result, *origin);
		goto release_sharedMem;
	}

	return TEEC_SUCCESS;

release_sharedMem:
	TEEC_ReleaseSharedMemory(handle->sharedMem);
finalize_context:
	TEEC_FinalizeContext(handle->context);
free_memory:
	free_memory_for_call_context(handle);
failed_to_alloc_memory:
	return result;
}

static TEEC_Result kg_ta_invoke_cmd(teegris_ta_call_handle_t *handle, uint32_t cmd, uint32_t *origin)
{
	TEEC_Result result = TEEC_SUCCESS;
	tz_kernel_msg_header_t *reqmsg_header = (tz_kernel_msg_header_t *)handle->sharedMem->buffer;
	tz_kernel_msg_header_t *respmsg_header = (tz_kernel_msg_header_t *)(handle->sharedMem->buffer + TA_REQ_BUFFER_SIZE);
	uint32_t status;

	reqmsg_header->content_id = KG_CMD_PROTOCOL_KERNEL;
	reqmsg_header->id = cmd;
	reqmsg_header->len = TA_REQ_BUFFER_SIZE;
	reqmsg_header->pid = current->pid;

	result = TEEC_InvokeCommand(handle->session, cmd, handle->operation, origin);

	if (result != TEEC_SUCCESS) {
		KG_LOG("%s: failed to TEEC_InvokeCommand result: 0x%x , origin: 0x%x\n",
				__func__, result, *origin);
	} else {
		/* return value from TA */
		status = respmsg_header->status;
		if (status)
			KG_LOG("%s cmd %d , TA err: 0x%x\n", __func__, cmd, status);
	}

	return result;
}

static void kg_ta_finalize(teegris_ta_call_handle_t *handle)
{
	TEEC_CloseSession(handle->session);
	TEEC_ReleaseSharedMemory(handle->sharedMem);
	TEEC_FinalizeContext(handle->context);

	free_memory_for_call_context(handle);
}

int kg_ta_call(uint32_t cmd, kg_ta_call_ctx_t *ctx)
{
	uint32_t origin;
	TEEC_Context context;
	TEEC_SharedMemory sharedMem;
	TEEC_Session session;
	TEEC_Operation operation;
	TEEC_Result result;
	teegris_ta_call_handle_t handle = {
		.context = &context,
		.sharedMem = &sharedMem,
		.session = &session,
		.operation = &operation
	};

	result = kg_ta_initialize(&handle, &origin);
	if (result != TEEC_SUCCESS)
		goto failed_kg_ta_initialize;

	/* caller write req buffer via hook function */
	if (ctx && ctx->kg_ta_call_input_hook && ctx->input)
		ctx->kg_ta_call_input_hook(ctx->input, handle.sharedMem->buffer);

	result = kg_ta_invoke_cmd(&handle, cmd, &origin);

	/* caller read resp buffer via hook function */
	if (result == TEEC_SUCCESS && ctx && ctx->kg_ta_call_output_hook && ctx->output)
		ctx->kg_ta_call_output_hook(ctx->output, handle.sharedMem->buffer + TA_REQ_BUFFER_SIZE);

	kg_ta_finalize(&handle);

failed_kg_ta_initialize:
	return (int)result;
}
