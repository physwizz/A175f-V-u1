#ifndef KG_NETLINK_H
#define KG_NETLINK_H

#define KGSM_FAMILY "KG Family"
#define KGSM_GROUP "KG Group"

#define FEATURE_CODE_LENGTH        (9)
#define MAX_ALLOWED_DETAIL_LENGTH  (1024)

#define KG_NETLINK_SUCCESS (0)

enum kgsm_operations {
	KGSM_MSG_CMD,
};

enum kgsm_attribute_ids {
	/* Numbering must start from 1 */
	KGSM_VALUE = 1,
	KGSM_FEATURE_CODE,
	KGSM_DETAIL,
	KGSM_SERVICE_READY,
	KGSM_ATTR_COUNT,
};
#define KGSM_ATTR_MAX (KGSM_ATTR_COUNT - 1)

extern int kgsm_is_initialized(void);
extern int __init kgsm_netlink_init(void);
extern void __exit kgsm_netlink_exit(void);
extern int kgsm_service_ready(void);
extern int kgsm_send_netlink_message(const char *feature_code, const char *detail, int64_t value);

#endif //KG_NETLINK_H
