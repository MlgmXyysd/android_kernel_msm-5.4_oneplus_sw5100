/*
 * oplus_shutdown.c - Linux kernel modules for OPLUS shutdown
 *
 * Copyright (C), 2008-2019, OPLUS Mobile Comm Corp., Ltd.
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


#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>

int oplus_enable_ship_mode_flag = 0;
EXPORT_SYMBOL(oplus_enable_ship_mode_flag);

static struct proc_dir_entry *oplusShutdown = NULL;

static ssize_t enable_ship_mode_read_proc(struct file *file, char __user *buf,
        size_t count,loff_t *off) {
    char page[256] = {0};
    size_t len = 0;
    len = sprintf(page,"%d",oplus_enable_ship_mode_flag);

    if(len > *off)
        len -= *off;
    else
        len = 0;

    if(copy_to_user(buf,page,(unsigned long)(len < count ? len : count))){
        return -EFAULT;
    }
    *off += len < count ? len : count;
    return (len < count ? len : count);
}

static ssize_t enable_ship_mode_write_proc(struct file *file, const char __user *buf,
        size_t count,loff_t *off) {
    char page[256] = {0};
    int temp = 0;
    int index = 0;

    if (count > sizeof(page)) {
        count = sizeof(page);
    }

    if(copy_from_user(page,buf,(unsigned long)count)) {
        return -EFAULT;
    }

    for (index = 0; index < sizeof(page); index++) {
        if (page[index] >= '0' && page[index] <= '9' ) {
            temp = temp * 10 + page[index] - '0';
        } else {
            break;
        }
    }

    oplus_enable_ship_mode_flag = temp;

    return count;
}

struct file_operations enable_ship_mode_proc_fops = {
    .read = enable_ship_mode_read_proc,
    .write = enable_ship_mode_write_proc,
};

static int __init oplus_shutdown_init(void)
{
    struct proc_dir_entry *pentry;

    oplusShutdown =  proc_mkdir("oplusShutdown", NULL);
    if(!oplusShutdown) {
        pr_err("can't create oplusShutdown proc\n");
        goto ERROR_INIT_VERSION;
    }
    pentry = proc_create("enable_ship_mode", S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP, oplusShutdown, &enable_ship_mode_proc_fops);
    if(!pentry) {
        pr_err("create enable_ship_mode proc failed.\n");
        goto ERROR_INIT_VERSION;
    }
    return 0;
ERROR_INIT_VERSION:
    remove_proc_entry("oplusShutdown", NULL);
    return -ENOENT;
}
arch_initcall(oplus_shutdown_init);

MODULE_AUTHOR("Oplus");
MODULE_DESCRIPTION("shutdown into ship mode");
MODULE_LICENSE("GPL");
