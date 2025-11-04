/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __KG_MAIN_H__
#define __KG_MAIN_H__

#include <linux/seq_file.h>

#include "kg_log.h"

extern void kg_hlmd_wakeup(bool from_client);
extern int kg_drv_info_notifier_register(struct notifier_block *n);
extern void kg_drv_info_print(void);
extern unsigned int kg_hlmd_get_key_info_error(void);

#endif /* __KG_MAIN_H__ */
