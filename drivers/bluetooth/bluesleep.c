/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.


 * Copyright (C) 2006-2007 - Motorola
 * Copyright (c) 2008-2010, The Linux Foundation. All rights reserved.
 * Copyright (c) 2013, LGE Inc.

 * Date         Author           Comment
 * -----------  --------------   --------------------------------
 * 2006-Apr-28	Motorola	 The kernel module for running the Bluetooth(R)
 *                               Sleep-Mode Protocol from the Host side
 *  2006-Sep-08  Motorola        Added workqueue for handling sleep work.
 *  2007-Jan-24  Motorola        Added mbm_handle_ioi() call to ISR.
 *  2009-Aug-10  Motorola        Changed "add_timer" to "mod_timer" to solve
 *                               race when flurry of queued work comes in.
 */

#define BT_BLUEDROID_SUPPORT 1

#define pr_fmt(fmt)	"Bluetooth: %s: " fmt, __func__

#include <linux/module.h>	/* kernel module definitions */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/irq.h>
#include <linux/ioport.h>
#include <linux/param.h>
#include <linux/bitops.h>
#include <linux/termios.h>
//add by ouyangxun
#include <linux/pm_wakeup.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/serial_core.h>
#include <linux/msm_geni_serial.h>

#if !BT_BLUEDROID_SUPPORT
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h> /* event notifications */
#include "hci_uart.h"
#else
#define BT_INFO  pr_info
#define BT_ERR   pr_err
#endif

#define BT_SLEEP_DBG
#ifndef BT_SLEEP_DBG
#define BT_DBG(fmt, arg...)
#endif
/*
 * Defines
 */

#define VERSION		"1.2"
#define PROC_DIR	"bluetooth/sleep"

#define POLARITY_LOW 0
#define POLARITY_HIGH 1

#define BT_PORT_ID	0

/* enable/disable wake-on-bluetooth */
#define BT_ENABLE_IRQ_WAKE 1


enum {
	DEBUG_USER_STATE = 1U << 0,
	DEBUG_SUSPEND = 1U << 1,
	DEBUG_BTWAKE = 1U << 2,
	DEBUG_VERBOSE = 1U << 3,
};

static int debug_mask = DEBUG_BTWAKE ;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

struct bluesleep_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
	unsigned host_wake;
	unsigned ext_wake;
	unsigned host_wake_irq;
	struct uart_port *uport;
	struct wakeup_source * wake_source;
	int irq_polarity;
	int has_ext_wake;
};

static DEFINE_MUTEX(mutex_bluesleep_work);

/* work function */
static void bluesleep_sleep_work(struct work_struct *work);

/* work queue */
DECLARE_DELAYED_WORK(sleep_workqueue, bluesleep_sleep_work);

/* Macros for handling sleep work */
#define bluesleep_rx_busy()     schedule_delayed_work(&sleep_workqueue, 0)
#define bluesleep_tx_busy()     schedule_delayed_work(&sleep_workqueue, 0)
#define bluesleep_rx_idle()     schedule_delayed_work(&sleep_workqueue, 0)
#define bluesleep_tx_idle()     schedule_delayed_work(&sleep_workqueue, 0)
#define bluesleep_wait_rx_idle()     flush_delayed_work(&sleep_workqueue)


/* 0.5 second timeout */
#define TX_TIMER_INTERVAL  (HZ / 2)

/* state variable names and bit positions */
#define BT_PROTO	0x01
#define BT_TXDATA	0x02
#define BT_ASLEEP	0x04
#define BT_EXT_WAKE	0x08
#define BT_SUSPEND	0x10

#define PROC_BTWAKE	0
#define PROC_HOSTWAKE	1
#define PROC_PROTO	2
#define PROC_ASLEEP	3
#if BT_BLUEDROID_SUPPORT
#define PROC_LPM	4
#define PROC_BTWRITE	5
#endif

#if BT_BLUEDROID_SUPPORT
static bool has_lpm_enabled;
#else
/* global pointer to a single hci device. */
static struct hci_dev *bluesleep_hdev;
#endif

static struct platform_device *bluesleep_uart_dev;
static struct bluesleep_info *bsi;

/* module usage */
static atomic_t open_count = ATOMIC_INIT(1);

/*
 * Local function prototypes
 */
#if !BT_BLUEDROID_SUPPORT
static int bluesleep_hci_event(struct notifier_block *this,
			unsigned long event, void *data);
#endif
static int bluesleep_start(void);
static void bluesleep_stop(void);

/*
 * Global variables
 */

/** Global state flags */
static unsigned long flags;

/** Tasklet to respond to change in hostwake line */
static struct tasklet_struct hostwake_task;

/** Transmission timer */
static void bluesleep_tx_timer_expire(struct timer_list *unused);
static DEFINE_TIMER(tx_timer, bluesleep_tx_timer_expire);

/** Lock for state transitions */
static spinlock_t rw_lock;

#if !BT_BLUEDROID_SUPPORT
/** Notifier block for HCI events */
struct notifier_block hci_event_nblock = {
	.notifier_call = bluesleep_hci_event,
};
#endif

struct proc_dir_entry *bluetooth_dir, *sleep_dir;

/*
 * Local functions
 */

static void hsuart_power(int on)
{
	static DEFINE_MUTEX(mutex_hsuart_power);
	static int last_on = -1;
	struct uart_port *uport = bsi->uport;

	pr_err("last_on = %d on = %d \n", last_on, on);

	if (unlikely(NULL == uport)) {
		pr_err("uport is NULL !!!\n");
		return;
	}

	mutex_lock(&mutex_hsuart_power);
	if (unlikely(last_on == on)) {
		WARN_ON(true);
	} else if (!test_bit(BT_SUSPEND, &flags)) {
		last_on = on;
		if (on) {
			vote_clock_on(uport);
			msm_geni_serial_set_mctrl(uport, TIOCM_RTS);
			if (bsi->has_ext_wake == 1){
				gpio_set_value(bsi->ext_wake, 1);
				if (debug_mask & DEBUG_BTWAKE)
					pr_info("BT WAKE: set to wake\n");
			}
		} else {
			msm_geni_serial_set_mctrl(uport, 0);
			vote_clock_off(uport);
		}
	} else {
		pr_err("BT_SUSPEND is ture");
	}
	mutex_unlock(&mutex_hsuart_power);
}

/**
 * @return 1 if the Host can go to sleep, 0 otherwise.
 */
int bluesleep_can_sleep(void)
{
	static int host_wake_cnt = 0;
	int  bt_host_wake = gpio_get_value(bsi->host_wake);
	bool bt_ext_wake = test_bit(BT_EXT_WAKE, &flags);

	/* check if WAKE_BT_GPIO and BT_WAKE_GPIO are both deasserted */
	if (debug_mask & DEBUG_VERBOSE) {
		pr_info("(%d != %d) && (%d) &&(%p)\n",
			bt_host_wake, bsi->irq_polarity,
			(bt_ext_wake), bsi->uport);
	}

	if ((bt_host_wake != bsi->irq_polarity)
		&& bt_ext_wake
		&& (bsi->uport != NULL)) {
		host_wake_cnt = 0;
		return 1; //Host can go to sleep
	}

	if (bt_ext_wake && (bt_host_wake == bsi->irq_polarity)) {
		host_wake_cnt++;
		if ((0 == host_wake_cnt % 10)) {
			pr_err("bt_host_wake is %d, cnt(%d)\n", bt_host_wake, host_wake_cnt);
		}
	} else {
		host_wake_cnt = 0;
	}
	return 0;
}

void bluesleep_sleep_wakeup(void)
{
	if (test_bit(BT_ASLEEP, &flags)) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("waking up...\n");

		__pm_stay_awake(bsi->wake_source);
		/* Start the timer */
		mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL));
		clear_bit(BT_EXT_WAKE, &flags);
		clear_bit(BT_ASLEEP, &flags);
		/*Activating UART */
		hsuart_power(1);
	} else {
		if (debug_mask & DEBUG_VERBOSE)
			pr_info("BT_ASLEEP is false\n");
	}
}

/**
 * @brief@  main sleep work handling function which update the flags
 * and activate and deactivate UART ,check FIFO.
 */
static void bluesleep_sleep_work(struct work_struct *work)
{
	struct uart_port *uport;
        int  bt_host_wake;
	mutex_lock(&mutex_bluesleep_work);
	if (bluesleep_can_sleep()) {
		/* already asleep, this is an error case */
		if (test_bit(BT_ASLEEP, &flags)) {
			if (debug_mask & DEBUG_SUSPEND)
				pr_info("already asleep\n");
			//bsi->uport = NULL;
			goto Exit;
		}

		uport = bsi->uport;
		if ((uport)&&(msm_geni_serial_tx_empty(uport))) {
			if (debug_mask & DEBUG_SUSPEND)
				pr_info("going to sleep...\n");
			set_bit(BT_ASLEEP, &flags);
			if (bsi->has_ext_wake == 1)
				gpio_set_value(bsi->ext_wake, 0);
			/*Deactivating UART */
			hsuart_power(0);
			if (!has_lpm_enabled) {
				bsi->uport = NULL;
				pr_err("set bsi->uport = NULL\n");
			}

			/* UART clk is not turned off immediately. Release
			 * wakelock after 500 ms.
			 */
			 __pm_wakeup_event(bsi->wake_source, 500);

		} else {
			if (debug_mask & DEBUG_BTWAKE)
				pr_err("restart tx_timer (%d)\n", TX_TIMER_INTERVAL);
                        bt_host_wake = gpio_get_value(bsi->host_wake);
			if(bt_host_wake != bsi->irq_polarity)
			{
				if (debug_mask & DEBUG_BTWAKE)
					pr_err("restart tx_timer mcu uart is in deepsleep mode (%d)\n", TX_TIMER_INTERVAL);
				if (bsi->has_ext_wake == 1)
                                {
					gpio_set_value(bsi->ext_wake, 0);
					udelay(30);
					gpio_set_value(bsi->ext_wake, 1);
				}
			}
			mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL));
			goto Exit;
		}
	} else if (test_bit(BT_EXT_WAKE, &flags)
			&& !test_bit(BT_ASLEEP, &flags)) {
		mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL));
		if (debug_mask & DEBUG_VERBOSE)
			pr_info("BT WAKE: set to wake\n");
		if (bsi->has_ext_wake == 1)
			gpio_set_value(bsi->ext_wake, 1);
		clear_bit(BT_EXT_WAKE, &flags);
	} else {
		bluesleep_sleep_wakeup();
		if (!has_lpm_enabled) {
			pr_err("lpm is disabled\n");
		}
	}
Exit:
	mutex_unlock(&mutex_bluesleep_work);
}

/**
 * A tasklet function that runs in tasklet context and reads the value
 * of the HOST_WAKE GPIO pin and further defer the work.
 * @param data Not used.
 */
static void bluesleep_hostwake_task(unsigned long data)
{
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("hostwake line change\n");

	if (!has_lpm_enabled) {
		pr_err("lpm is disabled, (%d,%d),(0x%lx)\n",
			gpio_get_value(bsi->host_wake), gpio_get_value(bsi->ext_wake), flags);
		return;
	}

	spin_lock(&rw_lock);
	if ((gpio_get_value(bsi->host_wake) == bsi->irq_polarity))
		bluesleep_rx_busy();
	else
		bluesleep_rx_idle();

	spin_unlock(&rw_lock);
}
/**
 * Function to check wheather bluetooth can sleep when btwrite was deasserted
 * by bluedroid.
 */
static void bluesleep_tx_allow_sleep(void)
{
	unsigned long irq_flags;

	if (debug_mask & DEBUG_VERBOSE)
		pr_err("%s\n", __FUNCTION__);
	if (debug_mask & DEBUG_SUSPEND)
		pr_err("Tx has been idle\n");

	spin_lock_irqsave(&rw_lock, irq_flags);

	clear_bit(BT_TXDATA, &flags);

	spin_unlock_irqrestore(&rw_lock, irq_flags);
}

/**
 * Handles proper timer action when outgoing data is delivered to the
 * HCI line discipline. Sets BT_TXDATA.
 */
static void bluesleep_outgoing_data(void)
{
	unsigned long irq_flags;
	spin_lock_irqsave(&rw_lock, irq_flags);

	/* log data passing by */
	set_bit(BT_TXDATA, &flags);

	spin_unlock_irqrestore(&rw_lock, irq_flags);

	/* if the tx side is sleeping... */
	if (test_bit(BT_EXT_WAKE, &flags)) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("tx was sleeping\n");
			bluesleep_sleep_wakeup();
	}
}

#if !BT_BLUEDROID_SUPPORT
/**
 * Handles HCI device events.
 * @param this Not used.
 * @param event The event that occurred.
 * @param data The HCI device associated with the event.
 * @return <code>NOTIFY_DONE</code>.
 */
static int bluesleep_hci_event(struct notifier_block *this,
				unsigned long event, void *data)
{
	struct hci_dev *hdev = (struct hci_dev *) data;
	struct hci_uart *hu;
	struct uart_state *state;

	if (!hdev)
		return NOTIFY_DONE;

	switch (event) {
	case HCI_DEV_REG:
		if (!bluesleep_hdev) {
			bluesleep_hdev = hdev;
			hu  = (struct hci_uart *) hdev->driver_data;
			state = (struct uart_state *) hu->tty->driver_data;
			bsi->uport = state->uart_port;
			/* if bluetooth started, start bluesleep*/
			bluesleep_start();
		}
		break;
	case HCI_DEV_UNREG:
		bluesleep_stop();
		bluesleep_hdev = NULL;
		//bsi->uport = NULL;
		/* if bluetooth stopped, stop bluesleep also */
		break;
	case HCI_DEV_WRITE:
		bluesleep_outgoing_data();
		break;
	}

	return NOTIFY_DONE;
}
#endif

/**
 * Handles transmission timer expiration.
 * @param data Not used.
 */
static void bluesleep_tx_timer_expire(struct timer_list *unused)
{
	unsigned long irq_flags;

	if (debug_mask & DEBUG_VERBOSE)
		pr_info("Tx timer expired\n");

	spin_lock_irqsave(&rw_lock, irq_flags);

	/* were we silent during the last timeout? */
	if (!test_bit(BT_TXDATA, &flags)) {
		if (debug_mask & DEBUG_VERBOSE)
			pr_info("Tx has been idle\n");
		if (debug_mask & DEBUG_BTWAKE)
			pr_info("BT WAKE: set to sleep\n");
		set_bit(BT_EXT_WAKE, &flags);
		bluesleep_tx_idle();
	} else {
		if (debug_mask & DEBUG_VERBOSE)
			pr_info("Tx data during last period\n");
		if (bsi->has_ext_wake == 1)
			gpio_set_value(bsi->ext_wake, 1);
		mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL));
	}

	/* clear the incoming data flag */
	//clear_bit(BT_TXDATA, &flags);

	spin_unlock_irqrestore(&rw_lock, irq_flags);
}

/**
 * Schedules a tasklet to run when receiving an interrupt on the
 * <code>HOST_WAKE</code> GPIO pin.
 * @param irq Not used.
 * @param dev_id Not used.
 */
static irqreturn_t bluesleep_hostwake_isr(int irq, void *dev_id)
{
	/* schedule a tasklet to handle the change in the host wake line */
	tasklet_schedule(&hostwake_task);
	return IRQ_HANDLED;
}

/**
 * Starts the Sleep-Mode Protocol on the Host.
 * @return On success, 0. On error, -1, and <code>errno</code> is set
 * appropriately.
 */
static int bluesleep_start(void)
{
	int retval;
	unsigned long irq_flags;

	spin_lock_irqsave(&rw_lock, irq_flags);

	if (test_bit(BT_PROTO, &flags)) {
		spin_unlock_irqrestore(&rw_lock, irq_flags);
		return 0;
	}

	spin_unlock_irqrestore(&rw_lock, irq_flags);

	if (!atomic_dec_and_test(&open_count)) {
		atomic_inc(&open_count);
		return -EBUSY;
	}

	/* start the timer */

	mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL));

	/* assert BT_WAKE */
	if (debug_mask & DEBUG_BTWAKE)
		pr_info("BT WAKE: set to wake\n");
	if (bsi->has_ext_wake == 1)
		gpio_set_value(bsi->ext_wake, 1);
	clear_bit(BT_EXT_WAKE, &flags);
#if BT_ENABLE_IRQ_WAKE
	enable_irq(bsi->host_wake_irq);
	retval = enable_irq_wake(bsi->host_wake_irq);
	if (retval < 0) {
		BT_ERR("Couldn't enable BT_HOST_WAKE as wakeup interrupt");
		goto fail;
	}
#endif
	set_bit(BT_PROTO, &flags);
	set_bit(BT_ASLEEP, &flags);
	
	__pm_stay_awake(bsi->wake_source);
	return 0;
fail:
	del_timer(&tx_timer);
	atomic_inc(&open_count);

	return retval;
}

/**
 * Stops the Sleep-Mode Protocol on the Host.
 */
static void bluesleep_stop(void)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&rw_lock, irq_flags);

	if (!test_bit(BT_PROTO, &flags)) {
		spin_unlock_irqrestore(&rw_lock, irq_flags);
		return;
	}
	
	del_timer(&tx_timer);
	//del_timer_sync(&tx_timer);
	clear_bit(BT_PROTO, &flags);

	/* deassert BT_WAKE */
	if (debug_mask & DEBUG_BTWAKE)
		pr_info("BT WAKE: set to sleep\n");
	if (bsi->has_ext_wake == 1)
		gpio_set_value(bsi->ext_wake, 0);
	set_bit(BT_EXT_WAKE, &flags);

	if (!test_bit(BT_ASLEEP, &flags)) {
		set_bit(BT_ASLEEP, &flags);
		spin_unlock_irqrestore(&rw_lock, irq_flags);
		hsuart_power(0);
	} else {
		spin_unlock_irqrestore(&rw_lock, irq_flags);
	}

	atomic_inc(&open_count);

#if BT_ENABLE_IRQ_WAKE
	if (disable_irq_wake(bsi->host_wake_irq))
		BT_ERR("Couldn't disable hostwake IRQ wakeup mode");
	disable_irq(bsi->host_wake_irq);
#endif

	__pm_wakeup_event(bsi->wake_source, 500);

	bluesleep_wait_rx_idle();
}

void bluesleep_setup_uart_port(struct platform_device *uart_dev)
{
	bluesleep_uart_dev = uart_dev;
}

static int bluesleep_populate_dt_pinfo(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int tmp;

	tmp = of_get_named_gpio(np, "bt_host_wake", 0);
	if (tmp < 0) {
		BT_ERR("couldn't find host_wake gpio");
		return -ENODEV;
	}
	bsi->host_wake = tmp;

	tmp = of_get_named_gpio(np, "bt_ext_wake", 0);
	if (tmp < 0)
		bsi->has_ext_wake = 0;
	else
		bsi->has_ext_wake = 1;

	if (bsi->has_ext_wake)
		bsi->ext_wake = tmp;

	gpio_direction_output(bsi->ext_wake, 0);
	gpio_direction_input(bsi->host_wake);

	BT_INFO("bt_host_wake %d, bt_ext_wake %d",
			bsi->host_wake,
			bsi->ext_wake);

	bsi->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(bsi->pinctrl)) {
		pr_err("pinctrl not defined\n");
		return PTR_ERR(bsi->pinctrl);
	}

	bsi->gpio_state_active =
		pinctrl_lookup_state(bsi->pinctrl, "wake_irq_active");
	if (IS_ERR_OR_NULL(bsi->gpio_state_active)) {
		pr_err("pinctrl lookup failed for default\n");
		return PTR_ERR(bsi->gpio_state_active);
	}

	bsi->gpio_state_suspend =
		pinctrl_lookup_state(bsi->pinctrl, "wake_irq_suspend");
	if (IS_ERR_OR_NULL(bsi->gpio_state_suspend)) {
		pr_err("pinctrl lookup failed for sleep\n");
		return PTR_ERR(bsi->gpio_state_suspend);
	}

	tmp = pinctrl_select_state(bsi->pinctrl, bsi->gpio_state_active);
	if (tmp) {
		pr_err("failed to select active state\n");
		pinctrl_select_state(bsi->pinctrl, bsi->gpio_state_suspend);
		return tmp;
	}

	return 0;
}

static int bluesleep_populate_pinfo(struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
				"gpio_host_wake");
	if (!res) {
		BT_ERR("couldn't find host_wake gpio");
		return -ENODEV;
	}
	bsi->host_wake = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
				"gpio_ext_wake");
	if (!res)
		bsi->has_ext_wake = 0;
	else
		bsi->has_ext_wake = 1;

	if (bsi->has_ext_wake)
		bsi->ext_wake = res->start;

	return 0;
}

static int bluesleep_probe(struct platform_device *pdev)
{
	//struct resource *res;
	int ret;

	bsi = kzalloc(sizeof(struct bluesleep_info), GFP_KERNEL);
	if (!bsi)
		return -ENOMEM;

	if (pdev->dev.of_node) {
		ret = bluesleep_populate_dt_pinfo(pdev);
		if (ret < 0) {
			BT_ERR("couldn't populate info from dt");
			return ret;
		}
	} else {
		ret = bluesleep_populate_pinfo(pdev);
		if (ret < 0) {
			BT_ERR("couldn't populate info");
			return ret;
		}
	}

	/* configure host_wake as input */
	ret = gpio_request_one(bsi->host_wake, GPIOF_IN, "bt_host_wake");
	if (ret < 0) {
		BT_ERR("failed to configure input direction for GPIO %d err %d",
				bsi->host_wake, ret);
		goto free_bsi;
	}

	if (debug_mask & DEBUG_BTWAKE)
		pr_info("BT WAKE: set to wake\n");
	if (bsi->has_ext_wake) {
		/* configure ext_wake as output mode*/
		ret = gpio_request_one(bsi->ext_wake,
				GPIOF_OUT_INIT_LOW, "bt_ext_wake");
		if (ret < 0) {
			BT_ERR("failed to configure output direction for GPIO %d err %d",
					bsi->ext_wake, ret);
			goto free_bt_host_wake;
		}
	}
	clear_bit(BT_EXT_WAKE, &flags);

	//res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
	//					"host_wake");
	//if (!res) {
	//	BT_ERR("couldn't find host_wake irq");
	//	ret = -ENODEV;
	//	goto free_bt_host_wake;
	//}
	//bsi->host_wake_irq = res->start;
	gpio_direction_input(bsi->host_wake);

	/* Initialize IRQ */
	bsi->host_wake_irq = gpio_to_irq(bsi->host_wake);

	if (bsi->host_wake_irq < 0) {
		BT_ERR("couldn't find host_wake irq");
		ret = -ENODEV;
		goto free_bt_ext_wake;
	}

	bsi->irq_polarity = POLARITY_HIGH;/*low edge (falling edge)*/

	bsi->wake_source = wakeup_source_register(&pdev->dev, "bluesleep");
	if (!bsi->wake_source) {
		ret = -ENOMEM;
		goto free_bt_ext_wake;
	}
	clear_bit(BT_SUSPEND, &flags);

	BT_INFO("host_wake_irq %d, polarity %d",
			bsi->host_wake_irq,
			bsi->irq_polarity);

	ret = request_irq(bsi->host_wake_irq, bluesleep_hostwake_isr,
			 IRQF_TRIGGER_RISING,
			"bluetooth hostwake", NULL);
	if (ret  < 0) {
		BT_ERR("Couldn't acquire BT_HOST_WAKE IRQ");
		goto free_bt_ext_wake;
	}
	disable_irq(bsi->host_wake_irq);

	pr_info("bluesleep_probe ok\n");
	return 0;

free_bt_ext_wake:
	gpio_free(bsi->ext_wake);
free_bt_host_wake:
	gpio_free(bsi->host_wake);
free_bsi:
	kfree(bsi);
	return ret;
}

static int bluesleep_remove(struct platform_device *pdev)
{
	free_irq(bsi->host_wake_irq, NULL);
	gpio_free(bsi->host_wake);
	gpio_free(bsi->ext_wake);
	wakeup_source_unregister(bsi->wake_source);
	kfree(bsi);
	return 0;
}


static int bluesleep_resume(struct platform_device *pdev)
{
	struct uart_port *uport = bsi->uport;
	if (test_bit(BT_SUSPEND, &flags)) {
		clear_bit(BT_SUSPEND, &flags);
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("bluesleep resuming...(%p)\n", uport);
// 		if ((gpio_get_value(bsi->host_wake) == bsi->irq_polarity)) {
// #if BT_BLUEDROID_SUPPORT
// 			if (!has_lpm_enabled) {
// 				pr_err("lpm is disabled,flags(0x%lx),(%d,%d),\n", flags,
// 					gpio_get_value(bsi->host_wake), gpio_get_value(bsi->ext_wake));
// 				return 0;
// 			}
// #endif
// 			if (debug_mask & DEBUG_SUSPEND)
// 				pr_info("bluesleep resume from BT event...\n");
// 			hsuart_power(1);
// 		}
	}
	return 0;
}

static int bluesleep_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("bluesleep suspending...\n");
	set_bit(BT_SUSPEND, &flags);
	return 0;
}

static struct of_device_id bluesleep_match_table[] = {
	{ .compatible = "bcm,bluesleep" },
	{}
};


static int bluesleep_prepare(struct device *dev)
{
	struct uart_port *uport = bsi->uport;
	if (debug_mask & DEBUG_SUSPEND)
			pr_info("blueslee prepare...(%p)\n", uport);
	return 0;
}

static void bluesleep_complete(struct device *dev)
{
	struct uart_port *uport = bsi->uport;
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("bluesleep complete...(%p)\n", uport);
	if ((gpio_get_value(bsi->host_wake) == bsi->irq_polarity)) {
#if BT_BLUEDROID_SUPPORT
		if (!has_lpm_enabled) {
			pr_err("lpm is disabled,flags(0x%lx),(%d,%d),\n", flags,
				gpio_get_value(bsi->host_wake), gpio_get_value(bsi->ext_wake));
			return ;
		}
#endif
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("bluesleep resume from BT event...\n");
		hsuart_power(1);
	}
}

static const struct dev_pm_ops bluesleep_pm = {
	.prepare	= bluesleep_prepare,
	.complete	= bluesleep_complete,
};

static struct platform_driver bluesleep_driver = {
	.probe = bluesleep_probe,
	.remove = bluesleep_remove,
	.suspend = bluesleep_suspend,
	.resume = bluesleep_resume,
	.driver = {
		.name = "bluesleep",
		.owner = THIS_MODULE,
		.of_match_table = bluesleep_match_table,
		.pm = &bluesleep_pm ,
	},
};

static int bluesleep_proc_show(struct seq_file *m, void *v)
{
	switch ((long)m->private) {
	case PROC_BTWAKE:
		seq_printf(m, "btwake:%u\n", test_bit(BT_EXT_WAKE, &flags));
		break;
	case PROC_HOSTWAKE:
		seq_printf(m, "hostwake: %u\n", gpio_get_value(bsi->host_wake));
		break;
	case PROC_PROTO:
		seq_printf(m, "proto: %u\n",
				test_bit(BT_PROTO, &flags) ? 1 : 0);
		break;
	case PROC_ASLEEP:
		seq_printf(m, "asleep: %u\n",
				test_bit(BT_ASLEEP, &flags) ? 1 : 0);
		break;
	default:
		return 0;
	}
	return 0;
}

static ssize_t bluesleep_proc_write(struct file *file, const char *buf,
	size_t count, loff_t *pos)
{
	void *data = PDE_DATA(file_inode(file));
	char lbuf[32];

	if (count >= sizeof(lbuf))
		count = sizeof(lbuf)-1;

	if (copy_from_user(lbuf, buf, count))
		return -EFAULT;
	lbuf[count] = 0;

	switch ((long)data) {
	case PROC_BTWAKE:
		pr_err("PROC_BTWAKE:(%s)\n", lbuf);
		if (lbuf[0] == '0') {
			if (debug_mask & DEBUG_BTWAKE)
				pr_info("BT WAKE: set to wake\n");
			if (bsi->has_ext_wake == 1)
				gpio_set_value(bsi->ext_wake, 1);
			clear_bit(BT_EXT_WAKE, &flags);
		} else if (buf[0] == '1') {
			if (debug_mask & DEBUG_BTWAKE)
				pr_info("BT WAKE: set to sleep\n");
			if (bsi->has_ext_wake == 1)
				gpio_set_value(bsi->ext_wake, 0);
			set_bit(BT_EXT_WAKE, &flags);
		}
		break;
	case PROC_PROTO:
		if (lbuf[0] == '0')
			bluesleep_stop();
		else
			bluesleep_start();
		break;
	case PROC_LPM:
		pr_err("PROC_LPM:(%s)\n", lbuf);
		if (lbuf[0] == '0') {
			/* HCI_DEV_UNREG */
			bluesleep_stop();
			has_lpm_enabled = false;
			//bsi->uport = NULL; //move to bluesleep_sleep_work() to set
		} else {
			/* HCI_DEV_REG */
			if (!has_lpm_enabled) {
				has_lpm_enabled = true;
				bsi->uport = msm_geni_get_uart_port(BT_PORT_ID);
				/* if bluetooth started, start bluesleep*/
				bluesleep_start();
			}
		}
		break;
	case PROC_BTWRITE:
		mutex_lock(&mutex_bluesleep_work);
		pr_err("PROC_BTWRITE:(%s)\n", lbuf);
		/* HCI_DEV_WRITE */
		if (lbuf[0] != '0')
			bluesleep_outgoing_data();
		else
			bluesleep_tx_allow_sleep();
		mutex_unlock(&mutex_bluesleep_work);
		break;
	default:
		return 0;
	}

	return count;
}

static int bluesleep_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, bluesleep_proc_show, PDE_DATA(inode));
}

static const struct file_operations bluesleep_proc_readwrite_fops = {
	.owner	= THIS_MODULE,
	.open	= bluesleep_proc_open,
	.read   = seq_read,
	.write  = bluesleep_proc_write,
};
static const struct file_operations bluesleep_proc_read_fops = {
	.owner	= THIS_MODULE,
	.open	= bluesleep_proc_open,
	.read   = seq_read,
};

/**
 * Initializes the module.
 * @return On success, 0. On error, -1, and <code>errno</code> is set
 * appropriately.
 */
static int __init bluesleep_init(void)
{
	int retval;
	struct proc_dir_entry *ent;

	BT_INFO("BlueSleep Mode Driver Ver %s", VERSION);

	retval = platform_driver_register(&bluesleep_driver);
	if (retval)
		return retval;

	if (bsi == NULL)
		return 0;

#if !BT_BLUEDROID_SUPPORT
	bluesleep_hdev = NULL;
#endif

	bluetooth_dir = proc_mkdir("bluetooth", NULL);
	if (bluetooth_dir == NULL) {
		BT_ERR("Unable to create /proc/bluetooth directory");
		return -ENOMEM;
	}

	sleep_dir = proc_mkdir("sleep", bluetooth_dir);
	if (sleep_dir == NULL) {
		BT_ERR("Unable to create /proc/%s directory", PROC_DIR);
		return -ENOMEM;
	}

	/* Creating read/write "btwake" entry */
	ent = proc_create_data("btwake", S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH,
			sleep_dir, &bluesleep_proc_readwrite_fops,
			(void *)PROC_BTWAKE);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/%s/btwake entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	/* read only proc entries */
	ent = proc_create_data("hostwake", S_IRUGO, sleep_dir,
				&bluesleep_proc_read_fops,
				(void *)PROC_HOSTWAKE);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/%s/hostwake entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	/* read/write proc entries */
	ent = proc_create_data("proto", S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH,
			sleep_dir, &bluesleep_proc_readwrite_fops,
			(void *)PROC_PROTO);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/%s/proto entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	/* read only proc entries */
	ent = proc_create_data("asleep", S_IRUGO,
			sleep_dir, &bluesleep_proc_read_fops,
			(void *)PROC_ASLEEP);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/%s/asleep entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

#if BT_BLUEDROID_SUPPORT
	/* read/write proc entries */
	ent = proc_create_data("lpm", S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH,
			sleep_dir, &bluesleep_proc_readwrite_fops,
			(void *)PROC_LPM);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/%s/lpm entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	/* read/write proc entries */
	ent = proc_create_data("btwrite", S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH,
			sleep_dir, &bluesleep_proc_readwrite_fops,
			(void *)PROC_BTWRITE);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/%s/btwrite entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

#endif

	flags = 0; /* clear all status bits */

	/* Initialize spinlock. */
	spin_lock_init(&rw_lock);

	/* Initialize timer */
	//init_timer(&tx_timer);
	//tx_timer.function = bluesleep_tx_timer_expire;
	//tx_timer.data = 0;

	/* initialize host wake tasklet */
	tasklet_init(&hostwake_task, bluesleep_hostwake_task, 0);

	/* assert bt wake */
	if (debug_mask & DEBUG_BTWAKE)
		pr_info("BT WAKE: set to sleep\n");
	if (bsi->has_ext_wake == 1)
		gpio_set_value(bsi->ext_wake, 0);
	clear_bit(BT_EXT_WAKE, &flags);
#if !BT_BLUEDROID_SUPPORT
	hci_register_notifier(&hci_event_nblock);
#endif
	return 0;

fail:
#if BT_BLUEDROID_SUPPORT
	remove_proc_entry("btwrite", sleep_dir);
	remove_proc_entry("lpm", sleep_dir);
#endif
	remove_proc_entry("asleep", sleep_dir);
	remove_proc_entry("proto", sleep_dir);
	remove_proc_entry("hostwake", sleep_dir);
	remove_proc_entry("btwake", sleep_dir);
	remove_proc_entry("sleep", bluetooth_dir);
	remove_proc_entry("bluetooth", 0);
	return retval;
}

/**
 * Cleans up the module.
 */
static void __exit bluesleep_exit(void)
{
	if (bsi == NULL)
		return;

	/* assert bt wake */
	if (bsi->has_ext_wake == 1)
		gpio_set_value(bsi->ext_wake, 0);
	clear_bit(BT_EXT_WAKE, &flags);
	if (test_bit(BT_PROTO, &flags)) {
		if (disable_irq_wake(bsi->host_wake_irq))
			BT_ERR("Couldn't disable hostwake IRQ wakeup mode");
		free_irq(bsi->host_wake_irq, NULL);
		del_timer(&tx_timer);
		if (test_bit(BT_ASLEEP, &flags))
			hsuart_power(1);
	}

#if !BT_BLUEDROID_SUPPORT
	hci_unregister_notifier(&hci_event_nblock);
#endif
	platform_driver_unregister(&bluesleep_driver);

#if BT_BLUEDROID_SUPPORT
	remove_proc_entry("btwrite", sleep_dir);
	remove_proc_entry("lpm", sleep_dir);
#endif
	remove_proc_entry("asleep", sleep_dir);
	remove_proc_entry("proto", sleep_dir);
	remove_proc_entry("hostwake", sleep_dir);
	remove_proc_entry("btwake", sleep_dir);
	remove_proc_entry("sleep", bluetooth_dir);
	remove_proc_entry("bluetooth", 0);
}

void oplus_bt_lmp_wake_set_state(bool assert) 
{
	pr_err("PROC_BTWAKE:(%s)\n", assert ? "assert" : "deassert");
	if (assert) {
		if (debug_mask & DEBUG_BTWAKE)
			pr_info("BT WAKE: set to wake\n");
		if (bsi->has_ext_wake == 1)
			gpio_set_value(bsi->ext_wake, 1);
		clear_bit(BT_EXT_WAKE, &flags);
	} else {
		if (debug_mask & DEBUG_BTWAKE)
			pr_info("BT WAKE: set to sleep\n");
		if (bsi->has_ext_wake == 1)
			gpio_set_value(bsi->ext_wake, 0);
		set_bit(BT_EXT_WAKE, &flags);
	}
}
EXPORT_SYMBOL_GPL(oplus_bt_lmp_wake_set_state);

void oplus_bt_write(bool assert)
{
		//mutex_lock(&mutex_bluesleep_work);
		//pr_err("PROC_BTWRITE:(%s)\n", assert ? "assert" : "deassert");
		/* HCI_DEV_WRITE */
		if (assert) {
			bluesleep_outgoing_data();
		} else {
			bluesleep_tx_allow_sleep();
		}

		//mutex_unlock(&mutex_bluesleep_work);
}
EXPORT_SYMBOL_GPL(oplus_bt_write);

module_init(bluesleep_init);
module_exit(bluesleep_exit);

MODULE_DESCRIPTION("Bluetooth Sleep Mode Driver ver %s " VERSION);
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
