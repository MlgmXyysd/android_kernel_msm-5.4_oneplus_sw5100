/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
 */
#ifndef _MSM_DRM_NOTIFY_H_
#define _MSM_DRM_NOTIFY_H_

#include <linux/notifier.h>

/* A hardware display blank change occurred */
#define MSM_DRM_EVENT_BLANK			0x01
/* A hardware display blank early change occurred */
#define MSM_DRM_EARLY_EVENT_BLANK		0x02
/* A hardware display idle change occurred */
#define MSM_DRM_EVENT_IDLE			0x03

/*
 * add for OnScreenFingerprint
*/
/* event for onscreenfingerprint scene */
#define MSM_DRM_ONSCREENFINGERPRINT_EVENT	0x10


enum {
	/* panel: power on */
	MSM_DRM_BLANK_UNBLANK,
	/* panel: power off */
	MSM_DRM_BLANK_POWERDOWN,
	/* panel: enter idle */
	MSM_DRM_ENTER_IDLE,
	/* panel: exit idle */
	MSM_DRM_EXIT_IDLE,
};

enum msm_drm_display_id {
	/* primary display */
	MSM_DRM_PRIMARY_DISPLAY,
	/* external display */
	MSM_DRM_EXTERNAL_DISPLAY,
	MSM_DRM_DISPLAY_MAX
};

struct msm_drm_notifier {
	enum msm_drm_display_id id;
	void *data;
};

int msm_drm_register_client(struct notifier_block *nb);
int msm_drm_unregister_client(struct notifier_block *nb);
#endif
