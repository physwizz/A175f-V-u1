#ifndef __KG_KEY_INFO_H__
#define __KG_KEY_INFO_H__

#include "kg_log.h"

#define LEN_FUSE_BIT 2
#define LEN_AP_SERIAL 32

struct kg_key_info {
	char fuse_bit[LEN_FUSE_BIT + 1];
	char ap_serial[LEN_AP_SERIAL + 1];
	int device_state;
	unsigned long err;
	u64 ts_nsec_update;
};

extern bool __init kg_key_info_init(struct kg_key_info *kg_drv_key_info);
extern void kg_get_client_key_info(struct kg_key_info *tmp);
extern char *kg_get_ap_serial(void);
extern bool kg_is_finance_device(void);

static inline void kg_key_info_print(struct kg_key_info *info)
{
	KG_LOG("%s %s %s %d 0x%lx %lu\n", __func__, info->fuse_bit, info->ap_serial,
		info->device_state, info->err, info->ts_nsec_update / NSEC_PER_SEC);
}

#endif /* __KG_KEY_INFO_H__ */
