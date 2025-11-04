#ifndef __KG_TA_CALL_H__
#define __KG_TA_CALL_H__

#define MC_ALIGN_SIZE 0x40
#define MC_ALIGN_MASK (MC_ALIGN_SIZE - 1)
#define MC_ALIGN(x) \
	((x + MC_ALIGN_SIZE) & (~MC_ALIGN_MASK))

#define KG_CMD_PROTOCOL_KERNEL 0x400

typedef enum kg_ta_cmd {
	KG_CMD_KERNEL_GET_LOCKS_SUMMARY = 0x500, // KG-KERNEL
	KG_CMD_KERNEL_SET_KEY_INFO_ERORR_FLAG,
} kg_cmd_t;

typedef struct tz_kernel_msg_header {
	uint32_t id;
	uint32_t content_id;
	uint32_t len;
	uint32_t status;
	uint32_t pid;
} __packed tz_kernel_msg_header_t;

typedef struct tz_kg_kernel_lockinfo {
	uint32_t kg_state;
	uint64_t locks_summary;
	uint32_t key_info_error;
	uint32_t memory_performance;
	uint32_t deny_camera;
} __packed tz_kg_kernel_lockinfo_t;

typedef struct tci_kernel_message {
	tz_kernel_msg_header_t header;
	union{
		tz_kg_kernel_lockinfo_t msg_tz_kg_kernel_lockinfo;
	} __packed payload;
} __packed tci_kernel_message_t;

#define TA_REQ_BUFFER_SIZE (MC_ALIGN(sizeof(tci_kernel_message_t)))
/* TA_BUFFER_TOTAL_SIZE = req_buf + resp_buf */
#define TA_BUFFER_TOTAL_SIZE (TA_REQ_BUFFER_SIZE * 2)

typedef struct kg_ta_call_ctx {
	void *input;
	void *output;
	void (*kg_ta_call_input_hook)(void *input, void *req);
	void (*kg_ta_call_output_hook)(void *output, void *resp);
} kg_ta_call_ctx_t;

extern void kg_ta_call_init(void);
extern int kg_ta_call(uint32_t cmd, kg_ta_call_ctx_t *ctx);
extern int kg_ta_get_info(tz_kg_kernel_lockinfo_t *ta_info);
extern int kg_ta_set_key_info_err(unsigned int err);

#endif /* __KG_TA_CALL_H__ */
