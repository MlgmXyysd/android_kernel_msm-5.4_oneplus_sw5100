// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2023, The Linux Foundation. All rights reserved.
 */

#ifndef __OPLUS_DATA_HOOK_H
#define __OPLUS_DATA_HOOK_H


struct hook_ops {
	int (*open)(void *hook_data);
	int (*close)(void *hook_data);
	int (*write)(void *hook_data, const void *data, int len);//to dev
	int (*write_iter)(void *hook_data, struct iov_iter *from);//to dev
	int (*read_cb)(void *hook_data, const void *data, int len);// to user
};

struct data_hook {
	void                      *hook_data;
	const struct hook_ops     *hook_data_ops;
};

//int hci_bcsp_register_hook(struct data_hook *hook_data);
//void hci_bcsp_unregister_hook(void *hook_data);

#endif /*__OPLUS_DATA_HOOK_H*/
