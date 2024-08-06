// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2023, The Linux Foundation. All rights reserved.
 */

#ifndef __OPLUS_BCSP_HOOK_H
#define __OPLUS_BCSP_HOOK_H

#include "oplus_data_hook.h"

int hci_bcsp_register_hook(struct data_hook *hook_data);
void hci_bcsp_unregister_hook(void *hb);

#endif /*__OPLUS_BCSP_HOOK_H*/
