/*
 * oplus_sensorhub.h - Linux kernel modules for Oplus SensorHub
 *
 * Copyright (C), 2008-2023, Oplus Mobile Comm Corp., Ltd.
 * Author: Oplus
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 */

#ifndef __LINUX_OPLUS_SENSORHUB_H
#define __LINUX_OPLUS_SENSORHUB_H

#define MCU_STATES_UEVENT

#include <linux/notifier.h>

struct oplus_sensorhub_platform_data {
    int gpio_reset;
    bool gpio_reset_active_low;
    int gpio_power;
    bool gpio_power_active_low;
    int gpio_buck_enable;
    bool gpio_buck_enable_active_low;
    int gpio_ap_suspend;
    bool gpio_ap_suspend_active_low;
    int gpio_ap_state;
    bool gpio_ap_state_active_low;
    int gpio_dump;
    bool gpio_dump_active_low;
};

enum {
    SNSHUB_PRE_RESET,
    SNSHUB_POST_RESET,
};

int oplus_sensorhub_reset_notifier_register(struct device *dev, struct notifier_block *nb);
int oplus_sensorhub_reset_notifier_unregister(struct device *dev, struct notifier_block *nb);

void oplus_sensorhub_assert_reset(struct device *dev);
void oplus_sensorhub_deassert_reset(struct device *dev);
void oplus_sensorhub_reset(struct device *dev);
void oplus_sensorhub_power_off_once(struct device *dev);
int oplus_sensorhub_hw_gpio_in(struct device *dev);

bool is_mcureseting(void);
void wait_mcureseting(void);

#ifdef MCU_STATES_UEVENT
typedef enum {
    MCU_STATES_FIRST = 0,
    MCU_STATES_NORMAL,
    MCU_STATES_CRASH,
    MCU_STATES_DUMP,
    MCU_STATES_MAX,
} snshub_mcu_states_t;

void oplus_sensorhub_states_uevent(int index);
void oplus_sensorhub_wakeup_data_uevent(void);

uint oplus_sensorhub_get_disable_uevent(void);
void oplus_sensorhub_set_disable_uevent(uint val);
#endif

#endif
