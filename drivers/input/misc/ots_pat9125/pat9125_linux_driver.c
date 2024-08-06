/* drivers/input/misc/ots_pat9125/pat9125_linux_driver.c
 *
 * Copyright (c) 2016 ~ 2022, The Linux Foundation. All rights reserved.
 *
 */

#include <linux/input.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/msm_drm_notify.h>
#include <linux/suspend.h>

#include "pixart_ots.h"

#undef pr_fmt
#define pr_fmt(fmt) PAT9125_DEV_NAME ":%s(): " fmt, __func__

#include "../../../oplus/include/oplus_switch.h"

#include "../../../oplus/include/oplus.h"


uint static_xcpi = PIXART_PAT9125_CPI_RESOLUTION_X_STATIC;
uint auto_suspend = PIXART_SAMPLING_AUTO_REDUCTION_TIMEOUT_US;

module_param(static_xcpi, uint, 0664);
MODULE_PARM_DESC(static_xcpi, "pixart pat9125 cpi resolution x static");

module_param(auto_suspend, uint, 0664);
MODULE_PARM_DESC(auto_suspend, "pixart sampling auto reduction time");


enum em_drm_state {
	DRM_BLANK_ACTIVED = MSM_DRM_BLANK_UNBLANK, /*panel: actived*/
	DRM_BLANK_IDLE, /*panel: power off or AON*/
};

struct pixart_pat9125_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct wakeup_source *ws;
	int irq_gpio;
	bool inverse_x;
#ifdef AXIS_Y_ENABLE
	bool inverse_y;
#ifdef PRESS_KEY_ENABLE
	u32 press_keycode;
	bool press_en;
#endif
#endif
#ifdef REGULATOR_VDD
	struct regulator *vdd;
#endif
#ifdef REGULATOR_VLD
	struct regulator *vld;
#endif
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;
	int irq_en;
	int calibrateing;
	int switch_state;
	struct notifier_block switch_nb;
#ifdef CONFIG_FB
	struct notifier_block drm_notifier;
	enum em_drm_state drm_state;
	int drm_blank;
	struct work_struct pwr_work;
	struct workqueue_struct *pwr_on_workqueue;
#endif
};

/* Declaration of suspend and resume functions */
static int pat9125_suspend(struct device *dev);
static int pat9125_resume(struct device *dev);

static int pat9125_i2c_write(struct i2c_client *client, u8 reg, u8 *data,
		int len)
{
	u8 buf[MAX_BUF_SIZE];
	int ret = 0, i;
	struct device *dev = &client->dev;

	buf[0] = reg;
	if (len >= MAX_BUF_SIZE) {
		dev_err(dev, "%s Failed: buffer size is %d [Max Limit is %d]\n",
			__func__, len, MAX_BUF_SIZE);
		return -ENODEV;
	}
	for (i = 0 ; i < len; i++)
		buf[i+1] = data[i];
	/* Returns negative errno, or else the number of bytes written. */
	ret = i2c_master_send(client, buf, len+1);
	if (ret != len+1)
		dev_err(dev, "%s Failed: writing to reg 0x%x:0x%x\n", __func__, reg, *data);

	return ret;
}

static int pat9125_i2c_read(struct i2c_client *client, u8 reg, u8 *data)
{
	u8 buf[MAX_BUF_SIZE];
	int ret = 0;
#if 0
	struct device *dev = &client->dev;
	buf[0] = reg;
	/*
	 * If everything went ok (1 msg transmitted), return #bytes transmitted,
	 * else error code. thus if transmit is ok return value 1
	 */
	ret = i2c_master_send(client, buf, 1);
	if (ret != 1) {
		dev_err(dev, "%s Failed: writing to reg 0x%x\n", __func__, reg);
		return ret;
	}
	/* returns negative errno, or else the number of bytes read */
	ret = i2c_master_recv(client, buf, 1);
	if (ret != 1) {
		dev_err(dev, "%s Failed: reading reg 0x%x\n", __func__, reg);
		return ret;
	}
#else
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = buf,
		}
	};

	if (i2c_transfer(client->adapter, msg, 2) != 2) {
		dev_err(&client->dev, "i2c transfer failed rd 0x%x\n", reg);
		ret = -EIO;
	} else {
		ret = 0;
	}
#endif
	*data = buf[0];
	return ret;
}

u8 read_data(struct i2c_client *client, u8 addr)
{
	u8 data = 0x00;

	if (pat9125_i2c_read(client, addr, &data)) {
		data = 0x00;
	}
	return data;
}

void write_data(struct i2c_client *client, u8 addr, u8 data)
{
	pat9125_i2c_write(client, addr, &data, 1);
}

static void inline remove_dirty_data(struct pixart_pat9125_data *pdata)
{
	if ((SWITCH_DISCONNECT != pdata->switch_state)
		&& (DRM_BLANK_ACTIVED == pdata->drm_state)) {
		if (gpio_is_valid(pdata->irq_gpio) && (0 == gpio_get_value(pdata->irq_gpio))) {
			u8 motion = read_data(pdata->client, PIXART_PAT9125_MOTION_STATUS_REG);
			u8 REL_x = 0, REL_y = 0, REL_xh = 0;
			if (motion & PIXART_PAT9125_VALID_MOTION_DATA) {
				/*Ignore dirty data*/
				REL_x = read_data(pdata->client, PIXART_PAT9125_DELTA_X_LO_REG);
				REL_y = read_data(pdata->client, PIXART_PAT9125_DELTA_Y_LO_REG);
				REL_xh= read_data(pdata->client, PIXART_PAT9125_DELTA_XY_HI_REG);
			}
			pr_info("Ignore dirty data(0x%02x, 0x%02x, 0x%02x, 0x%02x)\n",
				motion, REL_x, REL_y, REL_xh);
		}
	}
}


static irqreturn_t pat9125_irq(int irq, void *dev_data)
{
	static int err_unmotion_cnt = 0;
	#ifdef DYNAMIC_RESOLUTION
	int unmotion_cnt = 0;
	#endif
	u8 motion;
	u16 delta_xy_hi = 0, delta_x = 0;
	#ifdef AXIS_Y_ENABLE
	u16 delta_y = 0;
	#endif
	struct pixart_pat9125_data *data = dev_data;
	struct input_dev *ipdev = data->input;
	struct device *dev = &data->client->dev;

	__pm_stay_awake(data->ws);

	if (unlikely(0 == data->irq_en)) {
		pr_err("irq_en is disabled, exit isr\n");
		goto Exit_IRQ;
	}

	motion = read_data(data->client, PIXART_PAT9125_MOTION_STATUS_REG);

	if (unlikely(!(motion & PIXART_PAT9125_VALID_MOTION_DATA))) {
		bool switch_status = get_switch_status_by_id (SWITCH_WHEEL);

		if ((!switch_status)||(SWITCH_DISCONNECT == data->switch_state)) {
			pr_err("switch_io_status:%d, switch_state:%d, disable irq&exit isr\n",
				switch_status, data->switch_state);
			disable_irq_nosync(data->client->irq);
			irq_set_irq_type(data->client->irq, IRQ_TYPE_EDGE_RISING);
			data->switch_state = SWITCH_DISCONNECT;
			data->irq_en = 0;
		} else if (err_unmotion_cnt % 10 == 0) {
			if((err_unmotion_cnt < 100)||(err_unmotion_cnt % 100 == 0)) {
				pr_err("switch_io_status:%d, switch_state:%d, cnt:%d exit isr\n",
					switch_status, data->switch_state, err_unmotion_cnt);
			} else {
				if (switch_status) {
					disable_irq_nosync(data->client->irq);
					irq_set_irq_type(data->client->irq, IRQ_TYPE_EDGE_RISING);
					data->irq_en = 0;
					pr_err("switch_io_status:%d, cnt:%d , set irq is rinsing, exit isr\n",
										switch_status, err_unmotion_cnt);
				}
			}
		} else {
			usleep_range(PIXART_SAMPLING_PERIOD_US_MIN,
						PIXART_SAMPLING_PERIOD_US_MAX);
		}
		err_unmotion_cnt++;
		goto Exit_IRQ;
	}
	err_unmotion_cnt = 0;

	#ifdef DYNAMIC_RESOLUTION
	write_data(data->client, (u8)PIXART_PAT9125_SET_CPI_RES_X_REG, get_cpi_resolution_x());
	#endif

	while (motion & PIXART_PAT9125_VALID_MOTION_DATA) {
		/* check if MOTION bit is set or not */
		delta_x = read_data(data->client,
				PIXART_PAT9125_DELTA_X_LO_REG);
		#ifdef AXIS_Y_ENABLE
		delta_y = read_data(data->client,
				PIXART_PAT9125_DELTA_Y_LO_REG);
		#endif

		delta_xy_hi = read_data(data->client,
				PIXART_PAT9125_DELTA_XY_HI_REG);

		delta_x |= (delta_xy_hi&0x00f0) << 4;
		if (delta_x & 0x0800) {
			delta_x |= 0xf000;
		}

		/* Inverse x depending upon the device orientation */
		//delta_x = (data->inverse_x) ? -delta_x : delta_x;
		delta_x = (get_display_vflip()) ? -delta_x : delta_x;

		#ifdef AXIS_Y_ENABLE
		delta_y |= (delta_xy_hi&0x000f) << 8;
		if (delta_y & 0x0800) {
			delta_y |= 0xf000;
		}
		/* Inverse y depending upon the device orientation */
		delta_y = (data->inverse_y) ? -delta_y : delta_y;
		#endif

		#ifdef AXIS_Y_ENABLE
			dev_dbg(dev, "motion = %x, delta_x = %x, delta_y = %x\n",
				motion, delta_x, delta_y);
		#else
			dev_dbg(dev, "motion = %x, delta_x = %x\n",
				motion, delta_x);
		#endif


		if (0 != delta_x) {
			/* Send delta_x as REL_WHEEL for rotation */
			input_report_rel(ipdev, REL_WHEEL, (s16) delta_x);
			input_sync(ipdev);
		}

		#if defined(AXIS_Y_ENABLE) && defined(PRESS_KEY_ENABLE)
			if (data->press_en && delta_y != 0) {
				if ((s8) delta_y > 0) {
					/* Send DOWN event for press keycode */
					input_report_key(ipdev, data->press_keycode, 1);
					input_sync(ipdev);
				} else {
					/* Send UP event for press keycode */
					input_report_key(ipdev, data->press_keycode, 0);
					input_sync(ipdev);
				}
			}
		#endif

		#ifdef DYNAMIC_RESOLUTION
		unmotion_cnt = 0;
		#endif

		do {
			usleep_range(PIXART_SAMPLING_PERIOD_US_MIN,
						PIXART_SAMPLING_PERIOD_US_MAX);

			if (unlikely(0 == data->irq_en)) {
				motion = 0;
				pr_err("irq disabled, exit isr\n");
				goto Exit_IRQ;
			}

			motion = read_data(data->client,
					PIXART_PAT9125_MOTION_STATUS_REG);

			#ifdef DYNAMIC_RESOLUTION
			unmotion_cnt++;
			#else
			break;
			#endif
		} while (!(motion & PIXART_PAT9125_VALID_MOTION_DATA)
		#ifdef DYNAMIC_RESOLUTION
				&& (unmotion_cnt < (auto_suspend / PIXART_SAMPLING_PERIOD_US_MIN)) //PIXART_SAMPLING_AUTO_REDUCTION_CNT
		#endif
			);

	};
	#ifdef DYNAMIC_RESOLUTION
	if (likely(0 != data->irq_en)) {
		write_data(data->client, (u8)PIXART_PAT9125_SET_CPI_RES_X_REG, static_xcpi);
			//PIXART_PAT9125_CPI_RESOLUTION_X_STATIC);
		write_data(data->client, (u8)PIXART_PAT9125_OP_MODE_REG,
			PIXART_PAT9125_FORCE_SLEEP2);
	}
	#endif

Exit_IRQ:
	__pm_relax(data->ws);
	return IRQ_HANDLED;
}

static ssize_t pat9125_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct pixart_pat9125_data *data =
		(struct pixart_pat9125_data *) dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int mode;

	if (kstrtoint(buf, 10, &mode)) {
		dev_err(dev, "failed to read input for sysfs\n");
		return -EINVAL;
	}

	if (mode == 1)
		pat9125_suspend(&client->dev);
	else if (mode == 0)
		pat9125_resume(&client->dev);

	return count;
}


#ifdef REGISTER_RW_TEST
static u8 pat9125_test_rw_addr = 0;
static ssize_t pat9125_test_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char s[256], *p = s;
	int reg_data = 0, i;
	long rd_addr, wr_addr, wr_data;
	struct pixart_pat9125_data *data =
		(struct pixart_pat9125_data *) dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	for (i = 0; i < count; i++)
		s[i] = buf[i];
	*(s+1) = '\0';
	*(s+4) = '\0';
	*(s+7) = '\0';
	/* example(in console): echo w 12 34 > rw_reg */
	if (*p == 'w') {
		p += 2;
		if (!kstrtol(p, 16, &wr_addr)) {
			p += 3;
			if (!kstrtol(p, 16, &wr_data)) {
				pr_err("w 0x%x, 0x%x\n", wr_addr, wr_data);
				pat9125_test_rw_addr = (u8)wr_addr;
				dev_dbg(dev, "w 0x%x 0x%x\n",
					(u8)wr_addr, (u8)wr_data);
				write_data(client, (u8)wr_addr, (u8)wr_data);
			}
		}
	}
	/* example(in console): echo r 12 > rw_reg */
	else if (*p == 'r') {
		p += 2;
		if (!kstrtol(p, 16, &rd_addr)) {
			pat9125_test_rw_addr = (u8)rd_addr;
			reg_data = read_data(client, (u8)rd_addr);
			pr_err("r 0x%x 0x%x\n", rd_addr, reg_data);
			dev_dbg(dev, "r 0x%x 0x%x\n",
				(unsigned int)rd_addr, reg_data);
			
		}
	}
	return count;
}

static ssize_t pat9125_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pixart_pat9125_data *data = (struct pixart_pat9125_data *) dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int reg_data = read_data(client, (u8)pat9125_test_rw_addr);
	pr_err("r 0x%x 0x%x\n", pat9125_test_rw_addr, reg_data);
	return snprintf(buf, PAGE_SIZE, "r 0x%02x 0x%02x", pat9125_test_rw_addr, reg_data);
}
#endif /*#ifdef REGISTER_RW_TEST*/

static ssize_t pat9125_cpi_resolution_x_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct pixart_pat9125_data *data =
		(struct pixart_pat9125_data *) dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	long val;

	if (kstrtol(buf, 10, &val) != 0) {
		pr_err("kstrtol failed\n");
		return -EINVAL;
	}

	if (val > 255) {
		pr_err("kstrtol result lagre %d > 255\n", val);
		return -EINVAL;
	}
	set_cpi_resolution_x((u8)val);

	write_data(client, (u8)PIXART_PAT9125_SET_CPI_RES_X_REG, (u8)val);
	return count;
}

static ssize_t pat9125_cpi_resolution_x_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d", get_cpi_resolution_x());
}

static ssize_t pat9125_calibrate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct pixart_pat9125_data *pdata =
		(struct pixart_pat9125_data *) dev_get_drvdata(dev);
	struct i2c_client *client = pdata->client;

	if ((0 == buf[0])||('0' == buf[0])) { //stop calibrate
		if (0 != pdata->calibrateing) {
			if ((SWITCH_DISCONNECT != pdata->switch_state)
				&& (DRM_BLANK_ACTIVED == pdata->drm_state)) {
				remove_dirty_data(pdata);
				if (0 == pdata->irq_en) {
					pdata->irq_en = 1;
					pr_err("calibrate: stop, enable_irq\n");
					irq_set_irq_type(client->irq, IRQ_TYPE_EDGE_FALLING|IRQ_TYPE_LEVEL_LOW);
					enable_irq(client->irq);
					write_data(client, (u8)PIXART_PAT9125_SET_CPI_RES_X_REG, static_xcpi);
						//PIXART_PAT9125_CPI_RESOLUTION_X_STATIC);
					write_data(client, (u8)PIXART_PAT9125_OP_MODE_REG,
						PIXART_PAT9125_FORCE_SLEEP2);
				}
			} else {
				pr_err("calibrate: stop, disconnect or inactived\n");
			}
			pdata->calibrateing = 0;
		} else {
			pr_err("calibrate: stoped\n");
		}
	} else { //begin calibrate
		if (0 == pdata->calibrateing) {
			if ((SWITCH_DISCONNECT != pdata->switch_state)
				&& (DRM_BLANK_ACTIVED == pdata->drm_state)) {
				if (0 != pdata->irq_en) {
					pdata->irq_en = 0;
					pr_err("calibrate: begin, disable_irq\n");
					disable_irq_nosync(client->irq);
					irq_set_irq_type(client->irq, IRQ_TYPE_EDGE_RISING);
					write_data(client, (u8)PIXART_PAT9125_SET_CPI_RES_X_REG, get_cpi_resolution_x());
					write_data(client, (u8)PIXART_PAT9125_OP_MODE_REG,
						PIXART_PAT9125_SLEEP1_SLEEP2_DIS);
				}
				pdata->calibrateing = 1;
			} else {
				pr_err("calibrate: begin, error return -1\n");
				count = -1;
			}
		} else {
			pr_err("calibrate: testing\n");
		}
	}

	return count;
}

static ssize_t pat9125_calibrate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 motion;
	u16 delta_xy_hi = 0, delta_x = 0;
	#ifdef AXIS_Y_ENABLE
	u16 delta_y = 0;
	#endif
	struct pixart_pat9125_data *data =
		(struct pixart_pat9125_data *) dev_get_drvdata(dev);

	motion = read_data(data->client, PIXART_PAT9125_MOTION_STATUS_REG);
	/* check if MOTION bit is set or not */
	if (motion & PIXART_PAT9125_VALID_MOTION_DATA) {
		delta_x = read_data(data->client,
				PIXART_PAT9125_DELTA_X_LO_REG);
		#ifdef AXIS_Y_ENABLE
		delta_y = read_data(data->client,
				PIXART_PAT9125_DELTA_Y_LO_REG);
		#endif
		delta_xy_hi = read_data(data->client,
				PIXART_PAT9125_DELTA_XY_HI_REG);
		delta_x |= (delta_xy_hi&0x00f0) << 4;
		if (delta_x & 0x0800) {
			delta_x |= 0xf000;
		}
		/* Fixed inverse x during calibration*/
		delta_x = -delta_x;

		#ifdef AXIS_Y_ENABLE
		delta_y = (delta_xy_hi&0x000f) << 8;
		if (delta_y & 0x0800) {
			delta_y |= 0xf000;
		}
		/* Inverse y depending upon the device orientation */
		delta_y = (data->inverse_y) ? -delta_y : delta_y;
		#endif

		#ifdef AXIS_Y_ENABLE
		pr_err("calibrate: motion = 0x%x, delta_x = 0x%x, delta_y = 0x%x\n",
			motion, delta_x, delta_y);
		#else
		pr_err("calibrate: motion = 0x%x, delta_x = 0x%x\n", motion, delta_x);
		#endif
	}

#ifdef AXIS_Y_ENABLE
	return snprintf(buf, PAGE_SIZE, "%d, %d", (s16)delta_x, (s16)delta_y);
#else
	return snprintf(buf, PAGE_SIZE, "%d", (s16)delta_x);
#endif

}


static DEVICE_ATTR(suspend, S_IRUGO | S_IWUSR | S_IWGRP,
		NULL, pat9125_suspend_store);
#ifdef REGISTER_RW_TEST
static DEVICE_ATTR(test, S_IRUGO | S_IWUSR | S_IWGRP,
		pat9125_test_show, pat9125_test_store);
#endif /*#ifdef REGISTER_RW_TEST*/
static DEVICE_ATTR(cpi_resolution_x, S_IRUGO | S_IWUSR | S_IWGRP,
		pat9125_cpi_resolution_x_show, pat9125_cpi_resolution_x_store);

static DEVICE_ATTR(calibrate, S_IRUGO | S_IWUSR | S_IWGRP,
		pat9125_calibrate_show, pat9125_calibrate_store);

static struct attribute *pat9125_attr_list[] = {
#ifdef REGISTER_RW_TEST
	&dev_attr_test.attr,
#endif /*#ifdef REGISTER_RW_TEST*/
	&dev_attr_suspend.attr,
	&dev_attr_cpi_resolution_x.attr,
	&dev_attr_calibrate.attr,
	NULL,
};

static struct attribute_group pat9125_attr_grp = {
	.attrs = pat9125_attr_list,
};

static int pixart_pinctrl_init(struct pixart_pat9125_data *data)
{
	int err;
	struct device *dev = &data->client->dev;

	data->pinctrl = devm_pinctrl_get(&(data->client->dev));
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		err = PTR_ERR(data->pinctrl);
		dev_err(dev, "Target does not use pinctrl %d\n", err);
		return err;
	}

	data->pinctrl_state_active = pinctrl_lookup_state(data->pinctrl,
			PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(data->pinctrl_state_active)) {
		err = PTR_ERR(data->pinctrl_state_active);
		dev_err(dev, "Can not lookup active pinctrl state %d\n", err);
		return err;
	}

	data->pinctrl_state_suspend = pinctrl_lookup_state(data->pinctrl,
			PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(data->pinctrl_state_suspend)) {
		err = PTR_ERR(data->pinctrl_state_suspend);
		dev_err(dev, "Can not lookup suspend pinctrl state %d\n", err);
		return err;
	}

	data->pinctrl_state_release = pinctrl_lookup_state(data->pinctrl,
			PINCTRL_STATE_RELEASE);
	if (IS_ERR_OR_NULL(data->pinctrl_state_release)) {
		err = PTR_ERR(data->pinctrl_state_release);
		dev_err(dev, "Can not lookup release pinctrl state %d\n", err);
		return err;
	}
	return 0;
}

int pat9125_regulator_init(struct pixart_pat9125_data *data)
{
#if (defined(REGULATOR_VDD) || defined(REGULATOR_VLD))
	struct device *dev = &data->client->dev;
#endif

#ifdef REGULATOR_VDD
	data->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(data->vdd)) {
		dev_err(dev, "Failed to get regulator vdd %ld\n",
					PTR_ERR(data->vdd));
		//return PTR_ERR(data->vdd);
		data->vdd = NULL;
	} else {
		//int err = 0;
		//err = regulator_set_voltage(data->vdd, VDD_VTG_MIN_UV, VDD_VTG_MAX_UV);
		//if (err) {
		//	dev_err(dev, "Failed to set voltage for vdd reg %d\n", err);
		//	return err;
		//}
		//err = regulator_set_load(data->vdd, VDD_ACTIVE_LOAD_UA);
		//if (err < 0) {
		//	dev_err(dev, "Failed to set opt mode for vdd reg %d\n", err);
		//	return err;
		//}
	}
#endif

#ifdef REGULATOR_VLD
	data->vld = devm_regulator_get(dev, "vld");
	if (IS_ERR(data->vld)) {
		dev_err(dev, "Failed to get regulator vld %ld\n",
					PTR_ERR(data->vld));
		return PTR_ERR(data->vld);
	} else {
		//int err = 0;
		//err = regulator_set_voltage(data->vld, VLD_VTG_MIN_UV, VLD_VTG_MAX_UV);
		//if (err) {
		//	dev_err(dev, "Failed to set voltage for vld reg %d\n", err);
		//	return err;
		//}
		//err = regulator_set_load(data->vld, VLD_ACTIVE_LOAD_UA);
		//if (err < 0) {
		//	dev_err(dev, "Failed to set opt mode for vld reg %d\n", err);
		//	return err;
		//}
	}
#endif
	return 0;
}

static int pat9125_power_on(struct pixart_pat9125_data *data, bool on)
{
	int err = 0;
	struct device *dev = &data->client->dev;

	if (on) {
		#ifdef REGULATOR_VDD
		err = regulator_enable(data->vdd);
		if (err) {
			dev_err(dev, "Failed to enable vdd reg %d\n", err);
			return err;
		}
		//usleep_range(DELAY_BETWEEN_REG_US, DELAY_BETWEEN_REG_US + 1);
		#endif /*REGULATOR_VDD*/

		#ifdef REGULATOR_VLD
		err = regulator_enable(data->vld);
		dev_err(dev, "regulator_enable vld \n");
		if (err) {
			dev_err(dev, "Failed to enable vld reg %d\n", err);
			return err;
		}
		usleep_range(DELAY_BETWEEN_REG_US, DELAY_BETWEEN_REG_US + 100);
		#endif

		/*
		 * Initialize pixart sensor after some delay, when vdd
		 * regulator is enabled
		 */
		if ((0 != data->switch_state) && (!ots_sensor_init(data->client))) {
			err = -ENODEV;
			dev_err(dev, "Failed to initialize sensor %d\n", err);
			//return err;
		}

		remove_dirty_data(data);

	} else {
		#ifdef REGULATOR_VLD
		dev_err(dev, "regulator_disable vld \n");
		err = regulator_disable(data->vld);
		if (err) {
			dev_err(dev, "Failed to disable vld reg %d\n", err);
			return err;
		}
		#endif

		#ifdef REGULATOR_VDD
		err = regulator_disable(data->vdd);
		if (err) {
			dev_err(dev, "Failed to disable vdd reg %d\n", err);
			return err;
		}
		#endif
	}

	return 0;
}

static int pat9125_parse_dt(struct device *dev,
		struct pixart_pat9125_data *data)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	data->inverse_x = of_property_read_bool(np, "pixart,inverse-x");
	#ifdef AXIS_Y_ENABLE
	data->inverse_y = of_property_read_bool(np, "pixart,inverse-y");
	#ifdef PRESS_KEY_ENABLE
	data->press_en = of_property_read_bool(np, "pixart,press-enabled");
	if (data->press_en) {
		u32 temp_val;
		ret = of_property_read_u32(np, "pixart,press-keycode",
						&temp_val);
		if (!ret) {
			data->press_keycode = temp_val;
		} else {
			dev_err(dev, "Unable to parse press-keycode\n");
			return ret;
		}
	}
	#endif /*PRESS_KEY_ENABLE*/
	#endif /*AXIS_Y_ENABLE*/
	data->irq_gpio = of_get_named_gpio_flags(np, "pixart,irq-gpio",
						0, NULL);

	return ret;
}


static int switch_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct pixart_pat9125_data *pdata =
		container_of(self, struct pixart_pat9125_data, switch_nb);
	char *switch_states = (char *)data;
	if(NULL == switch_states) {
		pr_err("data error\n");
		return -1;
	}

	//pr_info("switch callback enable:%d current:%d\n", switch_states[1], switch_states[2]);
	if(((event == SWITCH_PRE_ACTION)
			&& (switch_states[1] == SWITCH_DISCONNECT))        /*switch to mcu*/
		|| ((event == SWITCH_POST_ACTION)
			&& (switch_states[1] != SWITCH_DISCONNECT))) {/*switch to ap*/
		pdata->switch_state = switch_states[1];

		if (DRM_BLANK_ACTIVED == pdata->drm_state) {
			if (SWITCH_DISCONNECT == pdata->switch_state) {
				if (0 != pdata->irq_en) {
					dev_info(&(pdata->client->dev), "%s: disable_irq!(%d)\n",
						__func__, pdata->switch_state);
					pdata->irq_en = 0;
					disable_irq_nosync(pdata->client->irq);
					irq_set_irq_type(pdata->client->irq, IRQ_TYPE_EDGE_RISING);
				}
			} else {
				if (0 == pdata->irq_en) {
					dev_info(&(pdata->client->dev), "%s: enable_irq!(%d)\n",
						__func__, pdata->switch_state);
					pdata->irq_en = 1;
					irq_set_irq_type(pdata->client->irq, IRQ_TYPE_EDGE_FALLING|IRQ_TYPE_LEVEL_LOW);
					enable_irq(pdata->client->irq);
				}
			}
		}
	}

	return 0;
}


#ifdef CONFIG_FB
static void pat_pwron_retry_work(struct work_struct *work)
{
	struct pixart_pat9125_data *pdata = container_of(
			work, struct pixart_pat9125_data, pwr_work);

	if (pat9125_power_on(pdata, true) < 0) {
		dev_err(&(pdata->client->dev), "Failed to enable regulators\n");
	}

	if (SWITCH_DISCONNECT != pdata->switch_state) {
		dev_info(&(pdata->client->dev), "%s: enable_irq\n", __func__);
		if ((pdata->pinctrl) && (pdata->pinctrl_state_active)) {
			int err = pinctrl_select_state(pdata->pinctrl, pdata->pinctrl_state_active);
			if (err < 0) {
				dev_err(&(pdata->client->dev), "Could not set pin to active state %d\n", err);
			}
		} else {
			dev_err(&(pdata->client->dev), "pdata->pinctrl or pdata->pinctrl_state_active is null\n");
		}

		if (0 == pdata->irq_en) {
			pdata->irq_en = 1;
			irq_set_irq_type(pdata->client->irq, IRQ_TYPE_EDGE_FALLING|IRQ_TYPE_LEVEL_LOW);
			enable_irq(pdata->client->irq);
			write_data(pdata->client, (u8)PIXART_PAT9125_SET_CPI_RES_X_REG, static_xcpi);//PIXART_PAT9125_CPI_RESOLUTION_X_STATIC);
			write_data(pdata->client, (u8)PIXART_PAT9125_CONFIG_REG, PIXART_PAT9125_POWERDOWN_DIS);
		}
	}
}


static int drm_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct pixart_pat9125_data *pdata =
		container_of(self, struct pixart_pat9125_data, drm_notifier);
	struct msm_drm_notifier *evdata = data;
	int *blank;
	int equivalent_blank = 0;

	if ((event != MSM_DRM_EVENT_BLANK && event != MSM_DRM_EVENT_IDLE) || !evdata)
		goto exit;

	blank = evdata->data;
	if(MSM_DRM_EVENT_IDLE == event) {
		if(MSM_DRM_ENTER_IDLE == *blank) {
			equivalent_blank = MSM_DRM_BLANK_POWERDOWN;
			blank = &equivalent_blank;
			dev_info(&(pdata->client->dev), "%s: Handle enter idle event as POWERDOWN!\n", __func__);
		} else if (MSM_DRM_EXIT_IDLE == *blank) {
			equivalent_blank = MSM_DRM_BLANK_UNBLANK;
			blank = &equivalent_blank;
			dev_info(&(pdata->client->dev), "%s: Handle exit idle as UNBLANK!\n", __func__);
		} else {
			dev_err(&(pdata->client->dev), "%s: Error:unknown event(%d) and blank(%d)\n",
				__func__, event, *blank);
			goto exit;
		}
	}

	pdata->drm_blank = *blank;

	if (*blank == DRM_BLANK_ACTIVED) {
		dev_info(&(pdata->client->dev), "%s: UNBLANK!\n", __func__);
		if (pdata->drm_state != DRM_BLANK_ACTIVED) {
			pdata->drm_state = DRM_BLANK_ACTIVED;
			//enable_irq(pdata->client->irq);
			queue_work(pdata->pwr_on_workqueue, &pdata->pwr_work);
		}
	} else if (*blank != DRM_BLANK_ACTIVED /*== MSM_DRM_BLANK_POWERDOWN*/) {
		cancel_work_sync(&pdata->pwr_work);
		dev_info(&(pdata->client->dev), "%s: BLANK!(%d)\n", __func__, pdata->drm_state);
		if (0 != pdata->irq_en) {
			dev_info(&(pdata->client->dev), "%s: disable_irq!(%d)\n",
				__func__, pdata->drm_state);
			disable_irq_nosync(pdata->client->irq);
			irq_set_irq_type(pdata->client->irq, IRQ_TYPE_EDGE_RISING);
			pdata->irq_en = 0;
		}

		if ((pdata->pinctrl) && (pdata->pinctrl_state_release)) {
			int err = pinctrl_select_state(pdata->pinctrl, pdata->pinctrl_state_release);
			if (err < 0) {
				dev_err(&(pdata->client->dev), "Could not set pin to release state %d\n", err);
			}
		} else {
			dev_err(&(pdata->client->dev), "pdata->pinctrl or pdata->pinctrl_state_release is null\n");
		}


		if (pdata->drm_state == DRM_BLANK_ACTIVED) {
			if (SWITCH_DISCONNECT != pdata->switch_state) {
				write_data(pdata->client, (u8)PIXART_PAT9125_CONFIG_REG, PIXART_PAT9125_POWERDOWN_EN);
				write_data(pdata->client, (u8)PIXART_PAT9125_SET_CPI_RES_X_REG, 0);
				write_data(pdata->client, (u8)PIXART_PAT9125_SET_CPI_RES_Y_REG, 0);
				remove_dirty_data(pdata);
			}
			if (pat9125_power_on(pdata, false) < 0) {
				dev_err(&(pdata->client->dev), "Failed to disable regulators\n");
			}
			pdata->drm_state = DRM_BLANK_IDLE;
		}
	}

exit:
	return 0;
}

static void pat_setup_drm_notifier(struct pixart_pat9125_data *pdata)
{
	int rc;

	pdata->drm_blank = -1;
	pdata->drm_state = DRM_BLANK_ACTIVED;

	pdata->drm_notifier.notifier_call = drm_notifier_callback;

	rc = msm_drm_register_client(&pdata->drm_notifier);
	if (rc)
		dev_err(&(pdata->client->dev), "Unable to register drm_notifier: %d\n", rc);
}
#endif


static int pat9125_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	struct pixart_pat9125_data *data;
	struct input_dev *input;
	struct device *dev = &client->dev;
	dev_err(dev, "pat9125_i2c_probe: entry\n");
	err = i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE);
	if (err < 0) {
		dev_err(dev, "I2C not supported\n");
		return -ENXIO;
	}

	if (client->dev.of_node) {
		data = devm_kzalloc(dev, sizeof(struct pixart_pat9125_data),
				GFP_KERNEL);
		if (!data)
			return -ENOMEM;
		err = pat9125_parse_dt(dev, data);
		if (err) {
			dev_err(dev, "DT parsing failed, errno:%d\n", err);
			return err;
		}
	} else {
		data = client->dev.platform_data;
		if (!data) {
			dev_err(dev, "Invalid pat9125 data\n");
			return -EINVAL;
		}
	}
	data->client = client;

	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "Failed to alloc input device\n");
		return -ENOMEM;
	}

	input_set_capability(input, EV_REL, REL_WHEEL);
#ifdef PRESS_KEY_ENABLE
	if (data->press_en)
		input_set_capability(input, EV_KEY, data->press_keycode);
#endif

	i2c_set_clientdata(client, data);
	input_set_drvdata(input, data);
	input->name = PAT9125_DEV_NAME;

	data->input = input;
	err = input_register_device(data->input);
	if (err < 0) {
		dev_err(dev, "Failed to register input device\n");
		return err;
	}

	err = pixart_pinctrl_init(data);
	if (!err && data->pinctrl) {
		/*
		 * Pinctrl handle is optional. If pinctrl handle is found
		 * let pins to be configured in active state. If not
		 * found continue further without error.
		 */
		err = pinctrl_select_state(data->pinctrl,
				data->pinctrl_state_active);
		if (err < 0)
			dev_err(dev, "Could not set pin to active state %d\n", err);
	} else {
		if (gpio_is_valid(data->irq_gpio)) {
			err = devm_gpio_request(dev, data->irq_gpio,
						"pixart_pat9125_irq_gpio");
			if (err) {
				dev_err(dev, "Couldn't request gpio %d\n", err);
				return err;
			}
			err = gpio_direction_input(data->irq_gpio);
			if (err) {
				dev_err(dev, "Couldn't set dir for gpio %d\n", err);
				return err;
			}

		} else {
			dev_err(dev, "Invalid gpio %d\n", data->irq_gpio);
			return -EINVAL;
		}
	}

	err = pat9125_regulator_init(data);
	if (err) {
		dev_err(dev, "Failed to init regulator, %d\n", err);
		return err;
	}

	data->ws = wakeup_source_register(dev, "pixart");
	if (!data->ws) {
		err = -ENOMEM;
		goto err_wakeup_source_register;
	}

	err = sysfs_create_group(&(input->dev.kobj), &pat9125_attr_grp);
	if (err) {
		dev_err(dev, "Failed to create sysfs group, errno:%d\n", err);
		goto err_sysfs_create;
	}

	data->switch_state = 1;
	data->switch_nb.notifier_call = switch_notifier_callback;
	err = oplus_switch_notifier_register(SWITCH_WHEEL, dev, &data->switch_nb);
	if (err) {
		dev_err(dev, "%s: oplus_switch_notifier_register failed\n", __func__);
	}

#ifdef CONFIG_FB
	INIT_WORK(&data->pwr_work, pat_pwron_retry_work);
	data->pwr_on_workqueue = create_singlethread_workqueue("pat_pwron_wq");
	if(!data->pwr_on_workqueue)
		dev_err(dev, "%s: create pat power workqueue failed\n", __func__);
	pat_setup_drm_notifier(data);
#endif

	err = pat9125_power_on(data, true);
	if (err) {
		dev_err(dev, "Failed to power-on the sensor %d\n", err);
		goto err_power_on;
	}

	data->irq_en = 1;
	err = devm_request_threaded_irq(dev, client->irq, NULL, pat9125_irq,
			 IRQF_ONESHOT | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_LOW,
			"pixart_pat9125_irq", data);
	if (err) {
		dev_err(dev, "Req irq %d failed, errno:%d\n", client->irq, err);
		goto err_request_threaded_irq;
	}
	dev_err(dev, "pat9125_i2c_probe: succeed\n");
	return 0;

err_request_threaded_irq:
err_power_on:
	#ifdef REGULATOR_VDD
	//regulator_set_load(data->vdd, 0);
	#endif
	#ifdef REGULATOR_VLD
	//regulator_set_load(data->vld, 0);
	#endif
	if (pat9125_power_on(data, false) < 0)
		dev_err(dev, "Failed to disable regulators\n");
err_sysfs_create:
	wakeup_source_unregister(data->ws);
err_wakeup_source_register:
	if (data->pinctrl)
		if (pinctrl_select_state(data->pinctrl,
				data->pinctrl_state_release) < 0)
			dev_err(dev, "Couldn't set pin to release state\n");

	return err;
}

static int pat9125_i2c_remove(struct i2c_client *client)
{
	struct pixart_pat9125_data *data = i2c_get_clientdata(client);
	struct device *dev = &data->client->dev;

#ifdef CONFIG_FB
	if(data->pwr_on_workqueue)
		destroy_workqueue(data->pwr_on_workqueue);
	msm_drm_unregister_client(&data->drm_notifier);
#endif

	if (oplus_switch_notifier_unregister(SWITCH_WHEEL, dev, &data->switch_nb)) {
		dev_err(dev, "%s: oplus_switch_notifier_unregister failed\n", __func__);
	}

	wakeup_source_unregister(data->ws);

	sysfs_remove_group(&(data->input->dev.kobj), &pat9125_attr_grp);
	if (data->pinctrl)
		if (pinctrl_select_state(data->pinctrl,
				data->pinctrl_state_release) < 0)
			dev_err(dev, "Couldn't set pin to release state\n");
	#ifdef REGULATOR_VDD
	regulator_set_load(data->vdd, 0);
	#endif
	#ifdef REGULATOR_VLD
	//regulator_set_load(data->vld, 0);
	#endif
	//pat9125_power_on(data, false);
	return 0;
}

#ifdef CONFIG_PM
#ifndef CONFIG_FB
static int pat9125_suspend(struct device *dev)
{
	int rc;
	struct pixart_pat9125_data *data =
		(struct pixart_pat9125_data *) dev_get_drvdata(dev);

	if (0 != pdata->irq_en) {
		pdata->irq_en = 0;
		disable_irq(data->client->irq);
	}

	if (data->pinctrl) {
		rc = pinctrl_select_state(data->pinctrl,
				data->pinctrl_state_suspend);
		if (rc < 0)
			dev_err(dev, "Could not set pin to suspend state %d\n",
									rc);
	}

	rc = pat9125_power_on(data, false);
	if (rc) {
		dev_err(dev, "Failed to disable regulators %d\n", rc);
		return rc;
	}

	return 0;
}

static int pat9125_resume(struct device *dev)
{
	int rc;
	struct pixart_pat9125_data *data =
		(struct pixart_pat9125_data *) dev_get_drvdata(dev);

	if (data->pinctrl) {
		rc = pinctrl_select_state(data->pinctrl,
				data->pinctrl_state_active);
		if (rc < 0)
			dev_err(dev, "Could not set pin to active state %d\n",
									rc);
	}

	rc = pat9125_power_on(data, true);
	if (rc) {
		dev_err(dev, "Failed to power-on the sensor %d\n", rc);
		goto err_sensor_init;
	}

	if (0 == pdata->irq_en) {
		pdata->irq_en = 1;
		irq_set_irq_type(data->client->irq, IRQ_TYPE_EDGE_FALLING|IRQ_TYPE_LEVEL_LOW);
		enable_irq(data->client->irq);
	}

	return 0;

err_sensor_init:
	if (data->pinctrl)
		if (pinctrl_select_state(data->pinctrl,
				data->pinctrl_state_suspend) < 0)
			dev_err(dev, "Couldn't set pin to suspend state\n");
	if (pat9125_power_on(data, false) < 0)
		dev_err(dev, "Failed to disable regulators\n");

	return rc;
}
#else /*#ifndef CONFIG_FB*/
static int pat9125_suspend(struct device *dev)
{
#ifdef CONFIG_DEEPSLEEP
	struct pixart_pat9125_data *pdata =
		(struct pixart_pat9125_data *) dev_get_drvdata(dev);
	if (PM_SUSPEND_MEM == mem_sleep_current) {
		if (0 != pdata->irq_en) {
			dev_info(&(pdata->client->dev), "%s: disable_irq!(%d)\n",
				__func__, pdata->drm_state);
			disable_irq_nosync(pdata->client->irq);
			irq_set_irq_type(pdata->client->irq, IRQ_TYPE_EDGE_RISING);
			pdata->irq_en = 0;
		}
	}
#endif

	return 0;
}

static int pat9125_resume(struct device *dev)
{
	return 0;
}
#endif /*#ifndef CONFIG_FB*/
#endif /*#ifdef CONFIG_PM*/


static const struct i2c_device_id pat9125_device_id[] = {
	{PAT9125_DEV_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, pat9125_device_id);

static const struct dev_pm_ops pat9125_pm_ops = {
	.suspend = pat9125_suspend,
	.resume = pat9125_resume
};

static const struct of_device_id pixart_pat9125_match_table[] = {
	{ .compatible = "pixart,pat9125",},
	{ },
};

static struct i2c_driver pat9125_i2c_driver = {
	.driver = {
		   .name = PAT9125_DEV_NAME,
		   .owner = THIS_MODULE,
		   .pm = &pat9125_pm_ops,
		   .of_match_table = pixart_pat9125_match_table,
		   },
	.probe = pat9125_i2c_probe,
	.remove = pat9125_i2c_remove,
	.id_table = pat9125_device_id,
};
module_i2c_driver(pat9125_i2c_driver);

MODULE_AUTHOR("pixart");
MODULE_DESCRIPTION("pixart pat9125 driver");
MODULE_LICENSE("GPL");
