/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __KG_DR_H__
#define __KG_DR_H__

enum {
	KG_DR_TYPE_HL_DISABLE_BIT,
	KG_DR_TYPE_DL_DISABLE_BIT,
	__MAX_NR_DR_TYPE
};

#define KG_DR_TYPE_HL_DISABLE ((1UL << KG_DR_TYPE_HL_DISABLE_BIT))
#define KG_DR_TYPE_DL_DISABLE ((1UL << KG_DR_TYPE_DL_DISABLE_BIT))
#define KG_DR_TYPE_MASK ((1UL << __MAX_NR_DR_TYPE) - 1)

extern void __init kg_dr_init(void);
extern bool kg_dr_hl_disabled(void);
extern bool kg_dr_dl_disabled(void);

#endif /* __KG_DR_H__ */
