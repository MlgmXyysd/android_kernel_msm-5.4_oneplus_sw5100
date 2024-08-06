// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2022, The Linux Foundation. All rights reserved.
 */

#ifndef __OPLUS_SWITCH_H
#define __OPLUS_SWITCH_H

#include <linux/notifier.h>

typedef enum {
    SWITCH_UI,
    SWITCH_LCD = SWITCH_UI,
    SWITCH_TP = SWITCH_UI,
    SWITCH_WHEEL = SWITCH_UI,
    SWITCH_MT,
    SWITCH_NFC,
} switch_id_t;

enum {
    SWITCH_PRE_ACTION,
    SWITCH_POST_ACTION,
};

enum em_switch_state {
	SWITCH_DISCONNECT = 0,
	SWITCH_CONNECT = 1,
};

bool get_switch_status_by_id (switch_id_t switch_id);
int oplus_switch_notifier_register(switch_id_t switch_id, struct device *dev, struct notifier_block *nb);
int oplus_switch_notifier_unregister(switch_id_t switch_id, struct device *dev, struct notifier_block *nb);

uint32_t get_all_switch_status(void); /*return bit field value*/


#endif /*__OPLUS_SWITCH_H*/
