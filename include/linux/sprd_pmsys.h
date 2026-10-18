/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_SPRD_PMSYS_H
#define _LINUX_SPRD_PMSYS_H

#include <linux/err.h>
#include <linux/kconfig.h>
struct device;
struct sprd_pmsys;

#if IS_REACHABLE(CONFIG_SPRD_UDX710_PMSYS)
/* The image is prepared with CM4 held. Release only after SIPC has installed
 * its mailbox receiver, so the firmware's initial OPEN messages are retained.
 */
struct sprd_pmsys *sprd_pmsys_get(struct device *consumer);
int sprd_pmsys_start(struct sprd_pmsys *pmsys);
#else
static inline struct sprd_pmsys *sprd_pmsys_get(struct device *consumer)
{
	return ERR_PTR(-EOPNOTSUPP);
}
static inline int sprd_pmsys_start(struct sprd_pmsys *pmsys)
{
	return -EOPNOTSUPP;
}
#endif
#endif
