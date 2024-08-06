// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2022, The Linux Foundation. All rights reserved.
 */
#define pr_fmt(fmt) "osw:%s():%d: " fmt "\n", __func__ , __LINE__

#include <linux/device.h>
#include <linux/err.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/gpio.h>
#include <linux/suspend.h>
#include <asm/uaccess.h>
#include "include/oplus_switch.h"
#include "../../fs/proc/internal.h"
//#include "internal.h"


//////////////////////////////////////////////////////////////////////
/*Module log config*/

static int osw_log_level = LOGLEVEL_WARNING;
module_param(osw_log_level, int, S_IRUGO|S_IWUSR|S_IWGRP);
MODULE_PARM_DESC(osw_log_level, "Oplus switch log level");

#define log_e(fmt, ...)       do{if(osw_log_level >= LOGLEVEL_ERR) \
	printk(KERN_ERR "osw:%s():%d: " fmt "\n", __func__ , __LINE__, ##__VA_ARGS__);}while(0)

#define log_w(fmt, ...)       do{if(osw_log_level >= LOGLEVEL_WARNING) \
	printk(KERN_ERR "osw:%s():%d: " fmt "\n", __func__ , __LINE__, ##__VA_ARGS__);}while(0)

#define log_n(fmt, ...)       do{if(osw_log_level >= LOGLEVEL_NOTICE) \
	printk(KERN_ERR "osw:%s():%d: " fmt "\n", __func__ , __LINE__, ##__VA_ARGS__);}while(0)

#define log_i(fmt, ...)       do{if(osw_log_level >= LOGLEVEL_INFO) \
	printk(KERN_ERR "osw:%s():%d: " fmt "\n", __func__ , __LINE__, ##__VA_ARGS__);}while(0)


struct device_switch_gpio_cfg {
	int gpio;
	bool gpio_active_low;
};

typedef struct {
	switch_id_t     switch_id;
	struct device_switch_gpio_cfg gpio_cfg;
	struct mutex lock;
	struct blocking_notifier_head bnh;
	bool status;
} device_switch_t;

typedef struct {
	const char      *name;
	switch_id_t     switch_id;
	device_switch_t *device_switch;
} proc_switch_t;

#define DEVICE_SWITCH_CNT_MAX   (4)
static device_switch_t  s_device_switch[DEVICE_SWITCH_CNT_MAX];
static unsigned int     device_switch_cnt = 0;

static proc_switch_t    *sp_proc_switch = NULL;
static unsigned int     proc_switch_cnt = 0;

//////////////////////////////////////////////////////////////////////
static device_switch_t *get_device_switch_by_id (switch_id_t switch_id)
{
	if (switch_id < DEVICE_SWITCH_CNT_MAX) {
		return &s_device_switch[switch_id];
	}
	log_w("switch_id:%d/%d/%d", switch_id, device_switch_cnt, DEVICE_SWITCH_CNT_MAX);
	return NULL;
}

bool get_switch_status_by_id (switch_id_t switch_id)
{
	static int dump_print = 0;
	if (likely(switch_id < DEVICE_SWITCH_CNT_MAX)) {
		if (switch_id == s_device_switch[switch_id].switch_id)
			return s_device_switch[switch_id].status;
	} else {
		log_w("switch_id:%d/%d/%d", switch_id, device_switch_cnt, DEVICE_SWITCH_CNT_MAX);
	}

	if (0 == (dump_print & 0x3ff)) {
		log_e("s_device_switch addr is 0x%px size:0x%x,"
			"device_switch_cnt addr is 0x%px, val:0x%x,(0x%x)",
			s_device_switch, sizeof(device_switch_t),
			&device_switch_cnt, device_switch_cnt, dump_print);

		print_hex_dump(KERN_ERR, "osw:", DUMP_PREFIX_OFFSET, 16, 1,
			s_device_switch, sizeof(s_device_switch), 1);
	}
	dump_print++;
	return false;
}
EXPORT_SYMBOL_GPL(get_switch_status_by_id);

/*return bit field value*/
uint32_t get_all_switch_status (void)
{
	uint32_t allswitch_status = 0;
	int i;
	for (i = 0; i < device_switch_cnt; i++) {
		if (s_device_switch[i].status) {
			allswitch_status |= BIT(i);
		}
	}
	if (allswitch_status) {
		log_i("switch 0x%x on sdm ", allswitch_status);
	}
	return allswitch_status;
}
EXPORT_SYMBOL_GPL(get_all_switch_status);

int oplus_switch_notifier_register(switch_id_t switch_id, struct device *dev, struct notifier_block *nb)
{
	device_switch_t *device_switch = get_device_switch_by_id(switch_id);

	if (!nb || !device_switch) {
		dev_err(dev, "%s() switch_id=%d, nb=%p notifier register failed\n", __func__, switch_id, nb);
		return -EINVAL;
	}

	return blocking_notifier_chain_register(&device_switch->bnh, nb);
}
EXPORT_SYMBOL_GPL(oplus_switch_notifier_register);

int oplus_switch_notifier_unregister(switch_id_t switch_id, struct device *dev, struct notifier_block *nb)
{
	device_switch_t *device_switch = get_device_switch_by_id(switch_id);

	if (!nb || !device_switch) {
		dev_err(dev, "%s() switch_id=%d, nb=%p notifier register failed\n", __func__, switch_id, nb);
		return -EINVAL;
	}

	return blocking_notifier_chain_unregister(&device_switch->bnh, nb);
}
EXPORT_SYMBOL_GPL(oplus_switch_notifier_unregister);

//////////////////////////////////////////////////////////////////////
#define ACTIVE_DELAY_TIME_US	        (3)

static inline
int device_switch_hw_get(struct device_switch_gpio_cfg *gpio_cfg)
{
	int value = 0;
	if (!gpio_cfg) {
		log_e("gpio_cfg is NULL");
		return -EINVAL;
	}
	if (!gpio_is_valid(gpio_cfg->gpio)) {
		log_e("gpio_cfg->gpio(%d) is not valid", gpio_cfg->gpio);
		return -EINVAL;
	}
	value = gpio_get_value(gpio_cfg->gpio);
	log_i("gpio_cfg->gpio(%d), value(%d)", gpio_cfg->gpio, value);

	return (!!value) ^ (!!gpio_cfg->gpio_active_low);
}

static inline
int device_switch_hw_set(struct device_switch_gpio_cfg *gpio_cfg, bool enable)
{
	int value = (!!gpio_cfg->gpio_active_low) ^ (!!enable);
	if (!gpio_cfg) {
		log_e("gpio_cfg is NULL");
		return -EINVAL;
	}
	if (!gpio_is_valid(gpio_cfg->gpio)) {
		log_e("gpio_cfg->gpio(%d) is not valid", gpio_cfg->gpio);
		return -EINVAL;
	}
	log_i("gpio_cfg->gpio(%d), value(%d)", gpio_cfg->gpio, value);

	//return gpio_set_value(gpio_cfg->gpio, value);
	return gpio_direction_output(gpio_cfg->gpio, value);
}

static inline
int device_switch_get_status(device_switch_t *dev_sw)
{
	int ret = 0;

	if (!dev_sw) {
		log_e("dev_sw is NULL");
		return -EINVAL;
	}

	ret = device_switch_hw_get(&(dev_sw->gpio_cfg));
	if (ret >= 0) {
		dev_sw->status = ret;
	}

	return ret;
}

static inline
int device_switch_set_action(device_switch_t *dev_sw, bool enable)
{
	int rc = 0;
	int ret = 0;
	char pdata[4] = {0};
	pdata[0] = !enable;
	pdata[1] = !!enable;
	pdata[2] = !enable;

	if (!dev_sw) {
		log_e("dev_sw is NULL");
		return -EINVAL;
	}

	if (mutex_trylock(&dev_sw->lock)) {
		rc = blocking_notifier_call_chain(&dev_sw->bnh, SWITCH_PRE_ACTION, (void*)pdata);
		if (rc) {
			ret = -EIO;
			log_e("blocking_notifier_call_chain: SWITCH_PRE_ACTION (%d)", ret);
		}
		{
			/* assert reset pin */
			if (device_switch_hw_set(&(dev_sw->gpio_cfg), enable)) {
				ret = device_switch_hw_set(&(dev_sw->gpio_cfg), enable);
			}
			if (ret) {
				ret = -EIO;
				log_w("1st failed, set status %d ", !!enable);
			}
		}

		dev_sw->status = device_switch_hw_get(&(dev_sw->gpio_cfg));
		if (dev_sw->status != !!enable) {
			ret = -EIO;
			log_w("2th failed, set status %d ", dev_sw->status);
			dev_sw->status = !enable;
		}

		udelay(ACTIVE_DELAY_TIME_US);
		pdata[2] = dev_sw->status;

		ret = blocking_notifier_call_chain(&dev_sw->bnh, SWITCH_POST_ACTION, (void*)pdata);
		if (ret) {
			ret = -EIO;
			log_e("blocking_notifier_call_chain:SWITCH_POST_ACTION (%d)", ret);
		}
		mutex_unlock(&dev_sw->lock);
	} else {
		/* another resetting is in progress, wait it done */
		ret = -EBUSY;
	}

	if (ret) {
		log_e("failed:enable:(%d),retrun (%d)", enable, ret);
	}

	return ret;
}

static inline
int proc_switch_get_status(proc_switch_t *proc_switch)
{
	if (!proc_switch) {
		log_e("proc_switch is NULL");
		return -EINVAL;
	}

	return device_switch_get_status(proc_switch->device_switch);
}

static inline
int proc_switch_action(proc_switch_t *proc_switch, bool enable)
{
	if (!proc_switch) {
		log_e("proc_switch is NULL");
		return -EINVAL;
	}

	return 	device_switch_set_action(proc_switch->device_switch, enable);
}


//////////////////////////////////////////////////////////////////////
/*proc file*/
static struct proc_dir_entry *oplusswitch_root = NULL;

//static
ssize_t proc_switch_read(struct file *file, char __user *buf, size_t count,loff_t *off)
{
	struct inode *inode = NULL;
	proc_switch_t *proc_switch_node = NULL;

	char result[16] = {0};
	int len = 1;

	if (!file || !buf || !off ||(count < 1)) {
		pr_err("Invalid argument (%x, %x, %d, %x)",
			(unsigned int)file, (unsigned int)buf,
			count, (unsigned int)off);
		return -EINVAL;
	}

	if (unlikely(!strncmp(current->comm, "cat", strlen("cat")))) { //be used cat debug
		if (0 != *off) {
			return 0;
		}
	}

	if (len < count)
		len = 2;

	inode = file->f_path.dentry->d_inode;
	proc_switch_node = (proc_switch_t *)PDE_DATA(inode);
	log_i("proc_switch_node = %x ", (unsigned int)proc_switch_node);
	if (!proc_switch_node) {
		log_w("proc_switch_node is NULL");
		return -EIO;
	}

	log_i("proc switch id(%d) name:%s ",
		proc_switch_node->switch_id, proc_switch_node->name);

	result[0] = proc_switch_get_status(proc_switch_node);
	if (result[0] < 0) {
		pr_err("get switch status error");
		return -EIO;
	}

	result[0] += 0x30;

	if(copy_to_user(buf, result, len)){
		pr_err("copy_to_user missing some data");
		return -EFAULT;
	}
	*off += len;
	return len;

}

static ssize_t proc_switch_write(struct file *file, const char __user *buf, size_t count,loff_t *off)
{
	struct inode *inode = NULL;
	proc_switch_t *proc_switch_node = NULL;
	char page[16] = {0};
	int enable = 1;
	int res = 0;

	if (!file || !buf || !off ||(count < 1)) {
		pr_err("Invalid argument (%x, %x, %d, %x)",
			(unsigned int)file, (unsigned int)buf,
			count, (unsigned int)off);
		return -EINVAL;
	}

	inode = file->f_path.dentry->d_inode;
	proc_switch_node = (proc_switch_t *)PDE_DATA(inode);
	log_i("proc_switch_node = %x", (unsigned int)proc_switch_node);

	if (!proc_switch_node) {
		log_w("proc_switch_node is NULL");
		return -EIO;
	}

	log_i("proc switch id(%d) name:%s ",
		proc_switch_node->switch_id, proc_switch_node->name);

	if (count > sizeof(page)) {
		count = sizeof(page);
	}

	if(copy_from_user(page,buf,count)) {
		pr_err("copy_from_user missing some data");
		return -EFAULT;
	}

	if ((0 == page[0]) || (0x30 == page[0])) {
		enable = 0;
	}

	//log_i("enable = %d", enable);
	res = proc_switch_action(proc_switch_node, enable);

	pr_err("%s(%d) set %s(%d), enable(%d), result:%d",
		current->comm, current->pid,
		proc_switch_node->name, proc_switch_node->switch_id,
		enable, res);

	if (0 == res) {
		res = count;
	}

	return res;
}


struct file_operations device_switch_proc_fops = {
	.read = proc_switch_read,
	.write = proc_switch_write,
};

//////////////////////////////////////////////////////////////////////


static proc_switch_t *device_switch_parse_dt(struct device *dev)
{
	struct device_node *node, *pp;
	proc_switch_t *proc_switch = NULL;
	proc_switch_t *proc_switch_node = NULL;
	int gpio_count = 0;
	int proc_count = 0;
	int i = 0, j = 0;
	int ret = 0;
	node = dev->of_node;
	if (!node) {
		dev_err(dev, "device_switch_parse_dt node is null\n");
		return ERR_PTR(-ENODEV);
	}


	/*switch gpio*/
	gpio_count = of_gpio_count(node);

	for (i=0; i < gpio_count; ++i) {
		enum of_gpio_flags flags = 0;
		struct device_switch_gpio_cfg *gpio_cfg = &s_device_switch[i].gpio_cfg;
		s_device_switch[i].switch_id = i;
		s_device_switch[i].status = true;
		gpio_cfg->gpio = -1;

		gpio_cfg->gpio = of_get_gpio_flags(node, i, &flags);
		log_n("dt gpio cfg:%d, %d", gpio_cfg->gpio, flags);
		if (gpio_cfg->gpio < 0) {
			dev_err(dev, "Unable to read gpio number\n");
			return ERR_PTR(-EINVAL);
		} else if (gpio_is_valid(gpio_cfg->gpio)) {
			if (gpio_request(gpio_cfg->gpio, NULL)) {
				gpio_cfg->gpio = -1;
				log_w("switch gpio cfg:%d, %d", ret, flags);
			} else {
				gpio_cfg->gpio_active_low = (flags & OF_GPIO_ACTIVE_LOW) ? true : false;
				gpio_direction_output(gpio_cfg->gpio, (!!gpio_cfg->gpio_active_low) ^ (true));
				log_i("switch gpio cfg:%d, %d", ret, gpio_cfg->gpio_active_low);
			}
		} else {
			dev_err(dev, "gpio %d is invalid\n", gpio_cfg->gpio);
		}
	}
	device_switch_cnt = gpio_count;


	/* switch proc */
	proc_count = of_get_available_child_count(node);

	if (proc_count == 0) {
		log_e("gpio count %d,proc count:%d", gpio_count, proc_count);
		return ERR_PTR(-ENODEV);
	}
	log_n("gpio count %d,proc count:%d", gpio_count, proc_count);

	proc_switch = devm_kzalloc(dev,
			     sizeof(*proc_switch) * proc_count,
			     GFP_KERNEL);

	i = 0;
	for_each_available_child_of_node(node, pp) {
		proc_switch_node = &proc_switch[i++];
		proc_switch_node->device_switch = NULL;
		proc_switch_node->name = (const char *)of_get_property(pp, "label", NULL);

		if (!proc_switch_node->name) {
			log_w("proc_switch_node name is NULL, check next");
			continue;
		}

		if (of_property_read_u32(pp, "index", &proc_switch_node->switch_id)) {
			dev_err(dev, "proc switch wrong citation: %s\n",
				proc_switch_node->name);
			return ERR_PTR(-EINVAL);
		}
		if (proc_switch_node->switch_id < device_switch_cnt) {
			log_n("proc_switch_node name:%s, %d", proc_switch_node->name, proc_switch_node->switch_id);
		} else {
			log_e("proc_switch_node name:%s, switch_id %d is out of range device_switch_cnt(%d)",
				proc_switch_node->name, proc_switch_node->switch_id, device_switch_cnt);
		}

		{
			struct proc_dir_entry *pentry = NULL;
			const char *name = proc_switch_node->name;
			device_switch_t *device_switch = NULL;

			for (j = 0; j < device_switch_cnt; j++) {
				if (proc_switch_node->switch_id == s_device_switch[j].switch_id) {
					device_switch = &s_device_switch[j];
				}
			}
			if (!device_switch) {
				pr_err("proc switch device_switch = %x", (unsigned int)proc_switch_node);
			} else {
				proc_switch_node->device_switch = device_switch;
				log_i("proc switch device_switch = %x", (unsigned int)proc_switch_node);
			}

			pentry = proc_create_data(name, S_IRUGO | S_IWUSR | S_IWGRP,
				oplusswitch_root, &device_switch_proc_fops, proc_switch_node);
			if(!pentry) {
				log_w("proc create switch %s failed.", name);
			} else {
				log_i("proc create switch data = %x ", (unsigned int)pentry->data);
			}
		}
	}

	proc_switch_cnt = proc_count;

	return proc_switch;
}




static int device_switch_probe(struct platform_device *pdev)
{
	extern unsigned int is_user_build;
	proc_switch_t *proc_switch = NULL;

	dev_info(&pdev->dev, "Oplus device_switch_probe\n");

	proc_switch = device_switch_parse_dt(&pdev->dev);
	if (IS_ERR(proc_switch)) {
		dev_err(&pdev->dev, "Oplus switch probe failed\n");
		return PTR_ERR(proc_switch);
	}

	sp_proc_switch = proc_switch;

	if (is_user_build) {
		osw_log_level = LOGLEVEL_ERR;
	}

	dev_info(&pdev->dev, "Oplus switch probe successful\n");

	return 0;
}

int device_switch_remove(struct platform_device * pdev)
{
	return 0;
}

static int switch_suspend(struct device *dev)
{
	(void)dev;
#ifdef CONFIG_DEEPSLEEP
	if (PM_SUSPEND_MEM == mem_sleep_current) {
		if (get_all_switch_status()) {
			log_e("have switch on sdm, prevent deep sleep");
			return -EBUSY;
		}
	}
#endif
	return 0;
}

static int switch_resume(struct device *dev)
{
	(void)dev;
#ifdef CONFIG_DEEPSLEEP
	//if (PM_SUSPEND_MEM == mem_sleep_current) {
	//}
#endif
	return 0;
}

static const struct dev_pm_ops switch_pm_ops = {
	.resume         = switch_resume,
	.suspend        = switch_suspend,
};

static struct of_device_id of_device_switch_match_tbl[] = {
	{ .compatible = "oplus,switch", },
	{},
};

static struct platform_driver device_switch_driver = {
	.driver = {
		.name = "device_switch",
		.of_match_table = of_device_switch_match_tbl,
		.pm    = &switch_pm_ops,
	},
	.probe = device_switch_probe,
	.remove = device_switch_remove,
};

static int __init proc_switch_init(void)
{
	extern unsigned int is_user_build;
	int i = 0;

	if (!is_user_build) {
		osw_log_level = LOGLEVEL_NOTICE;
	}

	for (i=0; i < DEVICE_SWITCH_CNT_MAX; i++) {
		BLOCKING_INIT_NOTIFIER_HEAD(&s_device_switch[i].bnh);
		mutex_init(&s_device_switch[i].lock);
	}

	oplusswitch_root =  proc_mkdir("switch", NULL);
	if(!oplusswitch_root) {
		pr_err("can't create Oplus switch rootdir");
		goto remove_rootdir;
	}

	return platform_driver_register(&(device_switch_driver));

remove_rootdir:
	remove_proc_entry("switch", NULL);
	return -ENOENT;
}

arch_initcall(proc_switch_init);


MODULE_AUTHOR("Oplus");
MODULE_DESCRIPTION("Oplus switch driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform: Oplus-switch");


