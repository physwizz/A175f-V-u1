#ifndef KG_KERNEL_API_H
#define KG_KERNEL_API_H

#include <linux/types.h>

extern noinline int kgsm_send_message(const char *feature_code, const char *detail, int64_t value);

#endif //KG_KERNEL_API_H
