// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2023 Oplus. All rights reserved.
 */
/*******************************************************
** File: - comm_master_plus.c
** Description: oplus sensorhub driver
** Version: 3.0
** Author: Oplus
*******************************************************/
#define LOGTAG "ocmp:"
#define DRIVER_VERSION "3.0.0.221212"
#define pr_fmt(fmt)  LOGTAG"%s():%d: " fmt "\n", __func__ , __LINE__

#define DATA_HOOK_EN
#define PRINT_LAST_N_PKG_INFO
//#define PRINT_WAKEUP_INFO
//#define SPEEDTEST
//#define DUAL_ACK
//#define STAND_ALONE_CLASS
#define STAND_ALONE_USED

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/time.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kfifo.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/poll.h>
#include <linux/pm_qos.h>
#include <uapi/linux/sched/types.h>
#include <linux/sched/task.h>
#include <linux/timekeeping.h>
#include <linux/suspend.h>
#include <linux/rtc.h>

#include <./include/oplus_bcsp_hook.h>
#include "oplus_sensorhub/oplus_sensorhub.h"

#define SNSHUB_KFIFO_SIZE                           (128 * 1024)
#define OTHERS_KFIFO_SIZE                           (16 * 1024)


#ifdef SPEEDTEST
#define SPEEDTEST_CTRL_CHANNEL_ID                    952
#define SPEEDTEST_DATA_CHANNEL_ID                    953
static char upload_speed_string[512];
static int upload_speed_string_ready = 0;
#endif

#ifdef COLLECT_WAKEUP_INFO
extern unsigned int pm_wakeup_irq(void);
#define wakeup_data_uevent()    oplus_sensorhub_wakeup_data_uevent()
#define WAKEUP_DATA_KFIFO_SIZE                      (1*1024)
#define WAKEUP_DATA_LEN                             (200)
static struct kfifo wakeup_data;
static struct mutex wakeup_data_fifo_lock;
#endif

struct spidev_data {
#ifdef STAND_ALONE_CLASS
	struct class        class;
#endif
	uint8_t             name[8];
	struct device       *dev;
	struct spi_device   *spi;

	uint32_t            spi_bits_per_word;
	uint32_t            spi_master;
	uint32_t            spi_slave_wr_addr;
	uint32_t            spi_slave_rd_addr;
	uint32_t            continuous_read;
	uint32_t            next_rd_pkg_len;
	uint32_t            rd_pkg_len_last_ten_max;
	uint8_t             rd_pkg_len_dec_cnt;

	void                *spi_rx_buffer; //4k
	uint8_t             *recv_frame;

	void                *spi_tx_buffer; //64K
	void                *send_frame;
	uint32_t            tx_buf_size;

	struct mutex        write_lock;
	//struct mutex        dual_comm_lock;

	int                 s2m_int_gpio;
	int                 s2m_ans_gpio;
	int                 m2s_int_gpio;
	int                 m2s_ans_gpio;

	int                 s2m_int_irq;
	int                 s2m_ans_irq;

	struct semaphore    s2m_int_pu_sem;
	struct semaphore    s2m_ans_pu_sem;

	struct semaphore    m2s_tx_fifo_sem;

	struct task_struct  *snshub_read_tsk;
	struct task_struct  *snshub_write_tsk;
	int                 is_suspended;

	struct wakeup_source upload_wakesrc;
	struct wakeup_source download_wakesrc;
	struct wakeup_source resever_wakesrc;

#ifdef PRINT_WAKEUP_INFO
	uint8_t             print_wakeup;
#endif
//first frame need't check selnum
	uint8_t             is_first_up_frame;
	uint8_t             is_first_dl_frame;

	uint16_t            up_selnum;
	uint16_t            dn_selnum;
};

struct comm_fifo {
	struct kfifo        fifo;
	struct mutex        lock;
	struct semaphore    packet_cnt_sem;
};

struct data_cdev {
	struct data_hook    *hook_process;
	dev_t               devt;
	struct spidev_data  *spidev;
	struct device       *dev;
	unsigned            users;
	uint16_t            channel_id;
	const char          *dev_name;

	struct comm_fifo    rx_fifo;
	struct comm_fifo    tx_fifo;
	wait_queue_head_t   waitq;

	struct mutex        read_lock;
	struct mutex        write_lock;

	uint8_t             *write_packet_area;
	uint8_t             *read_packet_area;

	struct wakeup_source wakesrc;
	char ws_name[32];

	struct cdev_count_info {  /*statistics*/
		int             rx_fifo_in_failed;
		int             rx_fifo_full;
		int             rx_packet_to_fifo;
		int             rx_packet_to_user;

		int             tx_fifo_full;
		int             tx_packet_to_fifo;
		int             tx_packet_to_mcu;

		int             tx_packet_send_suc;
		int             tx_packet_send_fail;
	} cci;
};

#define CDEV_HOOK_EN    0
#define CDEV_BIG_FIFO   1
#define CDEV_WAKEUP     2
#define STAND_ALONE     3 //only one

#define UPGRADE_CH_FLAG (BIT(STAND_ALONE))
#define BT_CH_FLAG      (BIT(CDEV_HOOK_EN)|BIT(CDEV_BIG_FIFO))

struct cdev_info_list_unit {
	const char          *dev_name;
	uint16_t            channel_id;
	unsigned long       flags;
	struct data_cdev    *cdev_info;
};

/****************************
 * {"node name", channel_id, flags}
 ****************************/
static struct cdev_info_list_unit g_cdev_info_list[] = {
	// {"dcc_datah",       7, BIT(CDEV_WAKEUP)},
	// {"dcc_data",        1, BIT(CDEV_BIG_FIFO)|BIT(CDEV_WAKEUP)},
	// {"mcu_factory",     2, 0},
	// {"gps_data",        3, 0},
	// {"std_sns",         4, 0},
	// {"sns_batch",       5, 0},
	// //{"mcu_echo",        6, 0},
	// {"dcc_battery",     8, 0},
	{"hci0",           10, BT_CH_FLAG},
	{"mcu_upgrade",     9, UPGRADE_CH_FLAG},
};

/**************************************************
 * The following code is about log print
 *************************************************/

#define LOG_ID_ERR               0
#define LOG_ID_WARN              1
#define LOG_ID_NOTICE            2
#define LOG_ID_INFO              3
#define LOG_ID_PACKET_INFO       4
#define LOG_ID_DN_SEQ            5
#define LOG_ID_UP_SEQ            6
#define LOG_ID_PACKET_TO_FIFO    7
#define LOG_ID_PACKET_TO_MCU     8
#define LOG_ID_DATA_TO_USER      9
#define LOG_ID_DATA_FROM_USER    10
#define LOG_ID_HOOK_DATA_SEND    11
#define LOG_ID_HOOK_DATA_RECV    12

struct log_unit {
	const char *name;
	int sw;
};

static struct log_unit log_array[32] = {
	{"log err",            1},
	{"log warn",           1},
	{"log notice",         0},
	{"log info",           0},
	{"log packet info",    0},
	{"download seq",       0},
	{"upload seq",         0},
	{"packet to fifo",     0},
	{"packet to mcu",      0},
	{"data to user",       0},
	{"data from user",     0},
	{"hook data send",     0},
	{"hook data recv",     0},
};

#define log_e(fmt, ...)       if(log_array[LOG_ID_ERR].sw) \
	printk(KERN_ERR LOGTAG"%s():%d: " fmt "\n", __func__ , __LINE__, ##__VA_ARGS__)

#define log_w(fmt, ...)       if(log_array[LOG_ID_WARN].sw) \
	printk(KERN_WARNING LOGTAG"%s():%d: " fmt "\n", __func__ , __LINE__, ##__VA_ARGS__)

#define log_n(fmt, ...)       if(log_array[LOG_ID_NOTICE].sw) \
	printk(KERN_NOTICE LOGTAG"%s():%d: " fmt "\n", __func__ , __LINE__, ##__VA_ARGS__)

#define log_i(fmt, ...)       if(log_array[LOG_ID_INFO].sw) \
	printk(KERN_INFO LOGTAG"%s():%d: " fmt "\n", __func__ , __LINE__, ##__VA_ARGS__)

#define log_hex(log_id, data, size, fmt, ...) \
	do { \
		if (log_array[log_id].sw) { \
			printk(KERN_ERR LOGTAG"hex(%s), size(%d), [" fmt "]\n",log_array[log_id].name, size, ##__VA_ARGS__); \
			print_hex_dump(KERN_ERR, LOGTAG, DUMP_PREFIX_OFFSET, 16, 1, data, size, 1); \
		} \
	} while (0)

static struct spidev_data *spidev = NULL;
static volatile int close_comm_state = 0x0;//bit0:download;bit1:upload

#ifdef PRINT_LAST_N_PKG_INFO
#define HEAD_DATA_LEN           (48)
typedef struct {
	struct timespec64  timestamp;
	uint32_t           serialno;
	uint8_t            data[HEAD_DATA_LEN];
} _StLastNPkgInfo;

#define RECORD_DATA_MAX         (16)
#define OUTBUF_DATA_MAX         (160)
static _StLastNPkgInfo fromuserdata[RECORD_DATA_MAX] = {0};
static _StLastNPkgInfo tomcudata[RECORD_DATA_MAX] = {0};
static int fromuserdataindex=0;
static int tomcudataindex=0;

static int32_t print_onepkginfo(_StLastNPkgInfo *pPI, uint8_t outbuf[OUTBUF_DATA_MAX])
{
	struct rtc_time tm;
	int count = 0, i=0;

	memset(outbuf, 0, OUTBUF_DATA_MAX);
//print timestamp serialno
	rtc_time64_to_tm(pPI->timestamp.tv_sec, &tm);
	count += snprintf(outbuf + count, OUTBUF_DATA_MAX-1 - count,
				"%d-%02d-%02d %02d:%02d:%02d.%09lu:%08x:",
				tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec,
				pPI->timestamp.tv_nsec, pPI->serialno);

//print data
	for (i=0; i<HEAD_DATA_LEN; i++) {
		count += snprintf(outbuf + count, OUTBUF_DATA_MAX-1 - count,
				"%02x", pPI->data[i]);
	}
	if (OUTBUF_DATA_MAX-1 <= count) {
		log_e("error %d", count);
	}
	//log_e("%d:%s", count, outbuf);
	printk(KERN_ERR LOGTAG"%s\n", outbuf);
	return count;
}

static uint32_t print_lastpkginfo(void)
{
	int i=0;
	int32_t count = 0;

	log_e("fromuser:");
	for (i=0; i<RECORD_DATA_MAX; i++) {
		uint8_t outbuf[OUTBUF_DATA_MAX] = {0};
		count += print_onepkginfo(&fromuserdata[i], outbuf);
	}

	log_e("tomcu:");
	for (i=0; i<RECORD_DATA_MAX; i++) {
		uint8_t outbuf[OUTBUF_DATA_MAX] = {0};
		count += print_onepkginfo(&tomcudata[i], outbuf);
	}
	return count;
}

static int snshub_reset_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{

	if((SNSHUB_PRE_RESET == event)) {
		print_lastpkginfo();
	}

	return 0;
}

static struct notifier_block snshub_reset_nb = {.notifier_call = snshub_reset_notifier_callback};

#define PING_REQ			(0x01012701000801F5ULL)
#define PING_RES			(0x01021301000C21F5ULL)

//print ping request
static void user_ping_req_print(const void *inbuf, uint32_t len)
{

	if (13 == len) {
		uint64_t pingdata = 0;
		memcpy(&pingdata, inbuf+1, sizeof(pingdata));

		if (PING_REQ == pingdata) {
			log_e("fromuser:ping request %d", len);
			print_hex_dump(KERN_ERR, LOGTAG, DUMP_PREFIX_OFFSET, 16, 1, inbuf, len, 1);
			return;
		}
	}
	return;
}

static void h5_ping_req_print(const void *inbuf, uint32_t len)
{
	if (20 == len) {
		uint64_t pingdata = 0;
		memcpy(&pingdata, inbuf+5, sizeof(pingdata));

		if (PING_REQ == pingdata) {
			log_e("h5 send:ping request %d", len);
			print_hex_dump(KERN_ERR, LOGTAG, DUMP_PREFIX_OFFSET, 16, 1, inbuf, len, 1);
			return;
		}
	}
	return;
}

static void spi_ping_req_print(const void *inbuf, uint32_t len)
{

	if (24 == len) {
		uint64_t pingdata = 0;
		memcpy(&pingdata, inbuf+4+5, sizeof(pingdata));

		if (PING_REQ == pingdata) {
			log_e("spi send:ping request %d", len);
			print_hex_dump(KERN_ERR, LOGTAG, DUMP_PREFIX_OFFSET, 16, 1, inbuf, len, 1);
			return;
		}
	}
	return;
}

//print ping response
static void spi_ping_res_print(const void *inbuf, uint32_t len)
{
	if (24 == len) {
		uint64_t pingdata = 0;
		memcpy(&pingdata, inbuf+5, sizeof(pingdata));

		if (PING_RES == pingdata) {
			log_e("spi receive:ping response %d", len);
			print_hex_dump(KERN_ERR, LOGTAG, DUMP_PREFIX_OFFSET, 16, 1, inbuf, len, 1);
			return;
		}
	}
	return;
}

static void h5_ping_res_print(const void *inbuf, uint32_t len)
{
	if (17 == len) {
		uint64_t pingdata = 0;
		memcpy(&pingdata, inbuf+1, sizeof(pingdata));

		if (PING_RES == pingdata) {
			log_e("h5 receive:ping response %d", len);
			print_hex_dump(KERN_ERR, LOGTAG, DUMP_PREFIX_OFFSET, 16, 1, inbuf, len, 1);
			return;
		}
	}
	return;
}

static void user_ping_res_print(const void *inbuf, uint32_t len)
{
	if (17 == len) {
		uint64_t pingdata = 0;
		memcpy(&pingdata, inbuf+1, sizeof(pingdata));

		if (PING_RES == pingdata) {
			log_e("touser:ping response %d", len);
			print_hex_dump(KERN_ERR, LOGTAG, DUMP_PREFIX_OFFSET, 16, 1, inbuf, len, 1);
			return;
		}
	}
	return;
}

//save data
void save_fromuserdata(uint8_t *inbuf, uint32_t len)
{
	static uint32_t serialno = 0;

	if (0 == len) {
		log_e("error len is 0");
		return;
	}

	memset(&fromuserdata[fromuserdataindex], 0, sizeof(_StLastNPkgInfo));

	if (len > HEAD_DATA_LEN)
		len = HEAD_DATA_LEN;

	ktime_get_real_ts64(&(fromuserdata[fromuserdataindex].timestamp));

	fromuserdata[fromuserdataindex].serialno = serialno;
	serialno++;

	memcpy(fromuserdata[fromuserdataindex].data, inbuf, len);

	fromuserdataindex++;
	fromuserdataindex &= (RECORD_DATA_MAX - 1);

	user_ping_req_print(inbuf, len);
	return;
}

void save_tomcudata(uint8_t *inbuf, uint32_t len)
{
	static uint32_t serialno = 0;
	if (0 == len) {
		log_e("error len is 0");
		return;
	}

	memset(&tomcudata[tomcudataindex], 0, sizeof(_StLastNPkgInfo));
	if (len > HEAD_DATA_LEN)
		len = HEAD_DATA_LEN;

	ktime_get_real_ts64(&(tomcudata[tomcudataindex].timestamp));

	tomcudata[tomcudataindex].serialno = serialno;
	serialno++;

	memcpy(tomcudata[tomcudataindex].data, inbuf, len);

	tomcudataindex++;
	tomcudataindex &= (RECORD_DATA_MAX - 1);
	return;
}
#endif


#ifdef STAND_ALONE_USED
static uint16_t gsau_channel_id = 0;

uint16_t __always_inline get_sau_ch_id(void)
{
	return gsau_channel_id;
}

void __always_inline set_sau_ch_id(uint16_t ch_id)
{
	gsau_channel_id = ch_id;
}

uint16_t __always_inline is_sau_filter(uint16_t ch_id)
{
	return (unlikely(get_sau_ch_id()) && (get_sau_ch_id() != ch_id));
}
#endif

/**************************************************
 * The following code is master phy layer.
 *************************************************/
// upper layer h5 timeout is 250ms, low layer's timeout shoud be less then upper layer's
#define WAIT_s2m_ans_PU_TIMEOUT           200
#define WAIT_s2m_ans_PD_TIMEOUT           500
#ifdef DUAL_ACK
#define WAIT_s2m_int_PU_TIMEOUT           2000
#endif

static inline void wakeup_source_init(struct wakeup_source *ws, const char *name)
{
	if (ws) {
		memset(ws, 0, sizeof(*ws));
		ws->name = name;
		wakeup_source_add(ws);
	} else {
		log_e("wakeup_source_init fail, ws is null");
	}
}

static void do_gettimeofday(struct timeval *tv)
{
	struct timespec64 ts;
	if (!tv) return;
	//clock_gettime(CLOCK_REALTIME, &ts);
	ktime_get_real_ts64(&ts);
	tv->tv_sec = ts.tv_sec;
	tv->tv_usec = (int)ts.tv_nsec / 1000;
}

static inline void m2s_int_set_value(int level)
{
	gpio_direction_output(spidev->m2s_int_gpio, level);
}

static inline void m2s_ans_set_value(int level)
{
	gpio_direction_output((spidev)->m2s_ans_gpio, level);
}

static inline int s2m_int_get_value(void)
{
	return gpio_get_value(spidev->s2m_int_gpio);
}

static void slave_control_tx(void) {
	m2s_int_set_value(1);
}

static void slave_control_rx(void) {
	m2s_ans_set_value(1);
}

static void slave_ready_cb(struct spi_device *spi, const void *tx, const void *rx)
{
	if (tx || rx) {
		if (tx)
			slave_control_tx();
		if (rx)
			slave_control_rx();
	} else {
		log_e("tx rx all NULL!!!");
	}
	return;
}
static inline void m2s_ans_send_pulse(int us)
{
	gpio_direction_output((spidev)->m2s_ans_gpio, 1);
	(us < 1000) ? udelay(us) : msleep(us / 1000);
	gpio_direction_output((spidev)->m2s_ans_gpio, 0);
}

static inline int wait_s2m_ans_pullup(void)
{
	if (down_timeout(&spidev->s2m_ans_pu_sem,
			msecs_to_jiffies(WAIT_s2m_ans_PU_TIMEOUT)) >= 0) {
		return 0;
	} else if (gpio_get_value(spidev->s2m_ans_gpio) == 1) {
		log_e("s2m_ans_pu_sem not set, but s2m_ans_gpio is high");
		return 0;
	} else
		return -ETIMEDOUT;
}

#ifdef DUAL_ACK
static inline int wait_s2m_int_pullup(void)
{
	if (down_timeout(&spidev->s2m_int_pu_sem,
			msecs_to_jiffies(WAIT_s2m_int_PU_TIMEOUT)) >= 0)
		return 0;
	else
		return -ETIMEDOUT;
}
#endif

static inline int wait_s2m_ans_pulldown(void)
{
	int i;
	for (i = 0; i < 1000; ++i) {
		if (gpio_get_value(spidev->s2m_ans_gpio) == 0)
			return 0;
		udelay(1);
	}

	for (i = 0; i < WAIT_s2m_ans_PD_TIMEOUT; ++i) {
		if (gpio_get_value(spidev->s2m_ans_gpio) == 0)
			return 0;
		msleep(1);
	}
	return -ETIMEDOUT;
}

static irqreturn_t s2m_int_irq_fn(int irq, void *handle)
{
	struct device *dev = (struct device *)handle;

	if (!dev ||!spidev) {
		/*NOTE:Submit code, please do not open "log_e".But own debug can open "log_e".*/
		//log_e("NULL pointer dev %p or spidev %p", dev, spidev);
		return IRQ_HANDLED;
	}

	if (irq == spidev->s2m_int_irq) {
		#ifndef DUAL_ACK
		if (gpio_get_value(spidev->s2m_int_gpio) == 1) {
		#endif
			__pm_wakeup_event(&spidev->upload_wakesrc, 1000);
			up(&spidev->s2m_int_pu_sem);
		#ifndef DUAL_ACK
		} else {
			/*NOTE:Submit code, please do not open "log_e".But own debug can open "log_e".*/
			//log_e("after rising edge, s2m_int is 0");
		}
		#endif
	} else {
		/*NOTE:Submit code, please do not open "log_e".But own debug can open "log_e".*/
		//log_e("(%d, %p) irq not int pin", irq, handle);
	}
	return IRQ_HANDLED;
}

static irqreturn_t s2m_ans_irq_fn(int irq, void *handle)
{
	struct device *dev = (struct device *)handle;

	if (!dev ||!spidev ) {
		/*NOTE:Submit code, please do not open "log_e".But own debug can open "log_e".*/
		//log_e("NULL pointer dev %p or spidev %p", dev, spidev);
		return IRQ_HANDLED;
	}

	if (irq == spidev->s2m_ans_irq) {
		#ifndef DUAL_ACK
		// if (gpio_get_value(spidev->s2m_ans_gpio) == 1) {
		#endif
			up(&spidev->s2m_ans_pu_sem);
		#ifndef DUAL_ACK
		// } else {
		// 	/*NOTE:Submit code, please do not open "log_e".But own debug can open "log_e".*/
		// 	//log_e("after rising edge, s2m_ans is 0");
		// }
		#endif
	} else {
		/*NOTE:Submit code, please do not open "log_e".But own debug can open "log_e".*/
		//log_e("(%d, %p) irq not int pin", irq, handle);
	}
	return IRQ_HANDLED;
}


/*********************************
 * @brief start spi sync
 * @param tx: tx_buf, could use NULL
 * @param rx: rx_buf, could use NULL
 * @param size: size of sync data
 * @ret: 0 is success, < 0 is fail
 *********************************/
static int spidev_spi_sync(void *tx, void *rx, size_t size)
{
	int status = 0;
	struct spi_message message;
	struct spi_transfer    t = {
		.tx_buf        = tx,
		.rx_buf        = rx,
		.len        = size,
		.bits_per_word = spidev->spi_bits_per_word,
	};

	while (spidev->is_suspended)
		msleep(10);

	spi_message_init(&message);
	spi_message_add_tail(&t, &message);

	if (spidev->spi != NULL)
		status = spi_sync(spidev->spi, &message);
	else
		status = -ESHUTDOWN;

	return status == 0 ? 0 : -EFAULT;
}

#ifdef MCU_STATES_UEVENT
#define S2M_ANS_PU_CON_TIMEOUT_COUNT        (3)
#define S2M_ANS_PD_CON_TIMEOUT_COUNT        (3)
#define DN_WRITE_SEQ_FAIL_COUNT             (5)

#define mcu_states_first()  do {oplus_sensorhub_states_uevent(MCU_STATES_FIRST);}while(0)
#define mcu_states_crash()  do { \
	log_e("uevent (crash), but not report"); \
} while(0)

#else
#define mcu_states_first()  do {}while(0)
#define mcu_states_crash()  do {}while(0)
#endif /*#ifdef MCU_STATES_UEVENT*/



/**************************************************
 * The following code is master data link layer.
 *************************************************/

static struct datalink_count_info{
	int s2m_ans_pu_timeout;
	int s2m_ans_pu_con_timeout;
	int dn_write_seq_fail;
	int s2m_ans_pd_timeout;
	int s2m_ans_pd_con_timeout;
	int dn_frame_suc;
	int dn_frame_check_err;

	int up_frame_discontinuous;
	int up_frame_check_err;
	int up_frame_suc;
	int up_disturb_signal;
	int up_read_seq_fail;
	int s2m_int_down_trylock;
} dci;

typedef enum {
	DL_STATUS_SUCCESS,
	DL_STATUS_TIMEOUT,
	DL_STATUS_CHECK_ERR,
	DL_STATUS_REPEAT,
	DL_STATUS_UPLOAD_SYNC_FAIL,
} comm_dl_status_t;

typedef struct {
	uint16_t selnum;
	union {
		uint16_t info;
		struct {
			uint16_t first_packet: 1;
			uint16_t : 7;  //reserved
			uint16_t check: 8;  //head check
		} info_b;
	};
	uint16_t checksum;
	uint16_t size;
	uint8_t  packet[0];
} comm_frame_t;
#define frame_packet(ptr)   ((comm_packet_t *)(((comm_frame_t*)(ptr))->packet))

typedef enum {
	PACKET_CHECK_SUCCESS,
	PACKET_CHECK_DISCONT,
	PACKET_CHECK_SUMERR,
	PACKET_CHECK_REPEAT,
} packet_check_t;



#define PHY_MST_TX_BUFF_SIZE                    (64 * 1024)
#define PHY_BUFF_SIZE                           PAGE_SIZE
#define SLAVE_ADDR_LEN                          4 //used as master
#define SLAVE_STATUS_LEN                        4 //used as master
#define SLAVE_SKIP_LEN                          (SLAVE_ADDR_LEN + SLAVE_STATUS_LEN) //used as master
#define DL_BUFF_LEN                             (PHY_BUFF_SIZE - SLAVE_SKIP_LEN)
#define DL_FRAME_HEAD_SIZE                      sizeof(comm_frame_t) //8
#define DL_MTU                                  (DL_BUFF_LEN - DL_FRAME_HEAD_SIZE)
#define XFER_MIN_LEN                            (80) // > fifo len 64

#define UP_CHECKERR_THRESHOLD                     100
#define DN_CHECKERR_THRESHOLD                     50

static uint16_t get_checksum16(void *data, uint16_t size)
{
	int i;
	uint32_t sum32 = 0;

	for (i = 0; i < size / 2; ++i)
		sum32 += ((uint16_t *)data)[i];

	if (size % 2)
		sum32 += ((uint8_t *)data)[size - 1];

	return (sum32 > 0xFFFF) ?
		~((uint16_t)((sum32 >> 16) + (sum32 & 0xFFFF))) :
		~((uint16_t)sum32);
}

struct time_measure_info {
	struct timeval begin;
	struct timeval end;
};

static inline void time_measure_begin(struct time_measure_info *info)
{
	do_gettimeofday(&(info->begin));
}

static inline void time_measure_end(struct time_measure_info *info)
{
	do_gettimeofday(&(info->end));
}

static inline int time_measure_get_us(struct time_measure_info *info)
{
	return (int)((info->end.tv_sec - info->begin.tv_sec) * 1000000
			+ (info->end.tv_usec - info->begin.tv_usec));
}

/***********************************************
 * after fill check bitfield, byte sum = 0xFF
 **********************************************/
static inline void fill_framehead_bitfield(comm_frame_t *frame)
{
	int i = 0;
	uint8_t sum = 0;
	frame->info_b.check = 0;
	for (i = 0; i < sizeof(comm_frame_t); ++i)
		sum += ((uint8_t *)frame)[i];

	frame->info_b.check = (uint8_t)0xFF - sum;
}

static inline int check_framehead(comm_frame_t *frame)
{
	int i = 0;
	uint8_t sum = 0;
	for (i = 0; i < sizeof(comm_frame_t); ++i) {
		sum += ((uint8_t *)frame)[i];
	}
	return ((sum == (uint8_t)0xFF) && (frame->size <= DL_MTU)) ? 0 : -1;
}


#define is_master(dev_data)  true
#ifndef is_master
static bool __always_inline is_master(struct spidev_data *spidev)
{
	return spidev->spi_master;
}
#endif

const int peer_spi_slave_rx_buf_addr = 0x20000000;
const int peer_spi_slave_tx_buf_addr = peer_spi_slave_rx_buf_addr + 4096;
#define SPI_WR (0 << 30)
#define SPI_RD (1 << 30)
#define SPI_ADDR_WR(a) (((a) >> 2) | SPI_WR)
#define SPI_ADDR_RD(a) (((a) >> 2) | SPI_RD)


static inline uint32_t switch_byte_order(uint32_t val)
{
	return val;
	/*
	uint32_t ret;
	unsigned char* buf = (unsigned char*)&ret;
	buf[3] = (val>>0)&0xFF;
	buf[2] = (val>>8)&0xFF;
	buf[1] = (val>>16)&0xFF;
	buf[0] = (val>>24)&0xFF;
	return ret;
	*/
}

static inline uint32_t spi_addr_wr(uint32_t addr) {
	return switch_byte_order(SPI_ADDR_WR(addr));
}

static inline uint32_t spi_addr_rd(uint32_t addr) {
	return switch_byte_order(SPI_ADDR_RD(addr));
}

static void enable_continuous_read()
{
	unsigned int buf[2]={0};
	int ret = 0;
	buf[0] = spi_addr_wr(0xfffc0004);
	buf[1] = switch_byte_order(0x1);
	ret = spidev_spi_sync(buf, NULL, sizeof(buf));
	if (0 != ret) {
		log_e("init spi slave failed");
	}
}

static int download_spi_seq(void *data, uint16_t size)
{
	int ret = 0;
	size_t xfer_size;
	comm_frame_t *frame = (comm_frame_t *)spidev->send_frame;

	if (is_master(spidev)) {
		*((uint32_t*)spidev->spi_tx_buffer) = spidev->spi_slave_wr_addr;
	}
	//frame head
	frame->selnum = spidev->dn_selnum;
	frame->info = 0x00;
	frame->info_b.first_packet = spidev->is_first_dl_frame;
	frame->size = size;
	frame->checksum = get_checksum16(frame->packet, size);
	fill_framehead_bitfield(frame);

	//spi sync tx
	xfer_size = roundup(sizeof(*frame)+size, 4) + 4;//up 4Byte align + 4 byte 0 OR slave addr len
	if (is_master(spidev)) {
	//	xfer_sizexfer_size += SLAVE_ADDR_LEN;
		if (xfer_size < 80)
			xfer_size = 80;
	}

	if (xfer_size > spidev->tx_buf_size)
		xfer_size = spidev->tx_buf_size;

	ret = spidev_spi_sync(spidev->spi_tx_buffer, NULL, xfer_size);
	if (0 != ret) {
		log_e("download spi sync failed, ret(%d)!", ret);
	}

	return ret;
}

static int upload_spi_seq(void)
{
	int retry_cnt = 1;
	int ret = 0;
try_again:
	//clear buff
	memset(spidev->spi_rx_buffer, 0, PHY_BUFF_SIZE);
	#ifdef DUAL_ACK
	memset(spidev->recv_frame, 0, PHY_BUFF_SIZE);
	#endif

	//spi rx
	if (is_master(spidev)) {
		uint32_t size = spidev->next_rd_pkg_len;
		if ((PHY_BUFF_SIZE - XFER_MIN_LEN) < spidev->next_rd_pkg_len) {
			size = PHY_BUFF_SIZE;
			spidev->next_rd_pkg_len = PHY_BUFF_SIZE;
		}
		*((uint32_t*)spidev->spi_rx_buffer) = spidev->spi_slave_rd_addr;
		ret = spidev_spi_sync(spidev->spi_rx_buffer, spidev->spi_rx_buffer, size);
	} else {
		ret = spidev_spi_sync(NULL, spidev->spi_rx_buffer, PHY_BUFF_SIZE);
	}

	if (0 != ret) {
		log_e("upload spi sync fail,ret(%d)", ret);
	} else {
		if (is_master(spidev)) {
			comm_frame_t *frame = (comm_frame_t *)(spidev->recv_frame);
			#ifdef DUAL_ACK
			memcpy(spidev->recv_frame, spidev->spi_rx_buffer + SLAVE_SKIP_LEN, DL_BUFF_LEN);
			#endif
			if (retry_cnt) {
				const uint32_t *a1 = (uint32_t*)spidev->recv_frame;
				if ((*a1) && (0xffffffff != *a1) && (*a1 == *(a1+1))
						&& (*a1 == *(a1+2))) {//eq is continuous read failed
					enable_continuous_read();
					spidev->next_rd_pkg_len = PHY_BUFF_SIZE;
					retry_cnt--;
					log_e("upload continuous read may not enabled (0x%x, 0x%x, 0x%x), retry", *a1, *(a1+1), *(a1+2));
					goto try_again;
				}
				if (spidev->next_rd_pkg_len < (frame->size + DL_FRAME_HEAD_SIZE + SLAVE_SKIP_LEN)) {
					uint32_t size = roundup(frame->size + DL_FRAME_HEAD_SIZE + SLAVE_SKIP_LEN, 32);
					spidev->next_rd_pkg_len = size;
					//spidev->rd_pkg_len_dec_cnt = 0;
					//spidev->rd_pkg_len_last_ten_max = XFER_MIN_LEN;
					retry_cnt--;
					goto try_again;
				}
			}
			//m2s_ans_set_value(1);
		} else {
			#ifdef DUAL_ACK
			memcpy(spidev->recv_frame, spidev->spi_rx_buffer, DL_BUFF_LEN);
			#endif
		}
	}

	return ret;
}


/*
 *this function check frame data
*/
static packet_check_t check_frame_data(struct spidev_data *spidev)
{
	comm_frame_t *frame = (comm_frame_t *)(spidev->recv_frame);
	if (check_framehead(frame) < 0 || get_checksum16(frame->packet, frame->size) !=  frame->checksum) {
		const uint32_t *a1 = (uint32_t*)spidev->recv_frame;
		log_e("%d:checksum error, %d, %d, 0x%x, 0x%x", spidev->up_selnum, frame->selnum, frame->size, *a1, *(a1+1));

		if (is_master(spidev)) {
			if ((frame->info_b.first_packet) || (frame->selnum < 3)) {
				spidev->continuous_read = 0;
			} else {
				if ((*a1) && (*a1 == *(a1+1))) {//eq is continuous read failed
					spidev->continuous_read = 0;
				}
			}
		}
		return PACKET_CHECK_SUMERR;
	}

	return PACKET_CHECK_SUCCESS;
}


/*
 * this function check data first,
 * if status is success, function copy data to dest data,
 * otherwise, return err status and don't copy data.
 */
static packet_check_t check_and_get_data_from_ram(void)
{
	packet_check_t status = PACKET_CHECK_SUCCESS;
	comm_frame_t *frame = (comm_frame_t *)(spidev->recv_frame);

	if (is_master(spidev)) {
		uint32_t size = roundup(frame->size + SLAVE_SKIP_LEN + DL_FRAME_HEAD_SIZE, 32);
		if (size < XFER_MIN_LEN) {
			size = XFER_MIN_LEN;
		}

		if (size < spidev->next_rd_pkg_len) {
			spidev->rd_pkg_len_dec_cnt++;
			if (spidev->rd_pkg_len_last_ten_max < size)
				spidev->rd_pkg_len_last_ten_max = size;
			if (spidev->rd_pkg_len_dec_cnt >= 32) {
				spidev->rd_pkg_len_dec_cnt = 0;
				log_i("%d:change next_rd_pkg_len (%d->%d)",
						spidev->up_selnum, spidev->next_rd_pkg_len, spidev->rd_pkg_len_last_ten_max);
				spidev->next_rd_pkg_len = spidev->rd_pkg_len_last_ten_max;
				spidev->rd_pkg_len_last_ten_max = XFER_MIN_LEN;
			}
		} else {
			if (spidev->next_rd_pkg_len < size) {
				log_i("%d:change next_rd_pkg_len (%d->%d)", spidev->up_selnum, spidev->next_rd_pkg_len, size);
				spidev->next_rd_pkg_len = size;
			}
			spidev->rd_pkg_len_dec_cnt = 0;
			spidev->rd_pkg_len_last_ten_max = XFER_MIN_LEN;
		}
	}

	if (frame->info_b.first_packet || spidev->is_first_up_frame) {
		if (frame->info_b.first_packet) {
			spidev->continuous_read = 0;
			mcu_states_first();
			log_n("%d:the MCU just reset, received first frame", spidev->up_selnum);
		}
	} else if (frame->selnum == spidev->up_selnum - 1) {
		status = PACKET_CHECK_REPEAT;
	} else if (frame->selnum != spidev->up_selnum) {
		status = PACKET_CHECK_DISCONT;
		log_n("%d:upload may lost %d packet(s)", spidev->up_selnum, frame->selnum - spidev->up_selnum);
	}

	spidev->up_selnum = frame->selnum + 1;
	return status;
}


static comm_dl_status_t
comm_master_dl_download(void *data, size_t data_size)
{
	comm_dl_status_t status = DL_STATUS_TIMEOUT;
	struct time_measure_info time_info;
	struct time_measure_info time_info_ack;

	wait_mcureseting();

	while(!down_trylock(&spidev->s2m_ans_pu_sem));
	if (gpio_get_value(spidev->m2s_int_gpio)) {
		gpio_direction_output(spidev->m2s_int_gpio, 0);
		udelay(1);
	}
	if (wait_s2m_ans_pulldown() < 0) {
		++dci.s2m_ans_pd_timeout;
		++dci.s2m_ans_pd_con_timeout;
#ifdef MCU_STATES_UEVENT
		if (S2M_ANS_PD_CON_TIMEOUT_COUNT <= dci.s2m_ans_pd_con_timeout) {
			mcu_states_crash();
		}
#endif
		log_e("%d:write init failed, s2m_ans is 1!, count %d", spidev->dn_selnum, dci.s2m_ans_pd_con_timeout);
		status = DL_STATUS_TIMEOUT;
		goto out_dgram_write_nolock;
	}
	dci.s2m_ans_pd_con_timeout = 0;

	if (is_master(spidev)) {
		//write begin handshake
		m2s_int_set_value(1);
		spi_ping_req_print(data, data_size);
		if (wait_s2m_ans_pullup() < 0) {
			++dci.s2m_ans_pu_timeout;
			++dci.s2m_ans_pu_con_timeout;
			status = DL_STATUS_TIMEOUT;
#ifdef MCU_STATES_UEVENT
			if ((S2M_ANS_PU_CON_TIMEOUT_COUNT <= dci.s2m_ans_pu_con_timeout)
				/*&& (dci.s2m_ans_pu_con_timeout < 10)*/) {
				mcu_states_crash();
			}
#endif
			if (dci.s2m_ans_pu_con_timeout <= 10 || dci.s2m_ans_pu_con_timeout % 10 == 0) {
				log_e("%d:write begin handshake failed, continuous times %d",
						spidev->dn_selnum, dci.s2m_ans_pu_con_timeout);
				if (dci.s2m_ans_pu_con_timeout >= 10)
					log_e("%d:MCU maybe: crashed, updating firmware, or in debug mode!", spidev->dn_selnum);
			}
			goto out_dgram_write;
		}
		dci.s2m_ans_pu_con_timeout = 0;
	    time_measure_begin(&time_info_ack);
	}
	//mutex_lock(&spidev->dual_comm_lock);

	//spi write seq
	if (download_spi_seq(data, data_size) < 0) {
		++dci.dn_write_seq_fail;
		status = DL_STATUS_TIMEOUT;
#ifdef MCU_STATES_UEVENT
		if ((DN_WRITE_SEQ_FAIL_COUNT <= dci.dn_write_seq_fail)
			/*&& (dci.dn_write_seq_fail < 10)*/) {
			mcu_states_crash();
		}
#endif

		if (dci.dn_write_seq_fail <= 10 || dci.dn_write_seq_fail % 10 == 0) {
			log_e("%d:dn write seq failed, continuous times %d",
					spidev->dn_selnum, dci.dn_write_seq_fail);
			if (dci.dn_write_seq_fail >= 10)
				log_e("%d:MCU maybe: crashed, updating firmware, or in debug mode!", spidev->dn_selnum);
		}
		goto out_dgram_write;
	}
	dci.dn_write_seq_fail = 0;

	//end handshake
	if (!is_master(spidev)) {
		if (wait_s2m_ans_pullup() < 0) {
			++dci.s2m_ans_pu_timeout;
			++dci.s2m_ans_pu_con_timeout;
			status = DL_STATUS_TIMEOUT;
#ifdef MCU_STATES_UEVENT
			if ((S2M_ANS_PU_CON_TIMEOUT_COUNT <= dci.s2m_ans_pu_con_timeout)
				/*&& (dci.s2m_ans_pu_con_timeout < 10)*/) {
				mcu_states_crash();
			}
#endif
			if (dci.s2m_ans_pu_con_timeout <= 10 || dci.s2m_ans_pu_con_timeout % 10 == 0) {
				log_e("%d:write end handshake failed, continuous times %d",
						spidev->dn_selnum, dci.s2m_ans_pu_con_timeout);
				if (dci.s2m_ans_pu_con_timeout >= 10)
					log_e("%d:MCU maybe: crashed, updating firmware, or in debug mode!", spidev->dn_selnum);
			}
			goto out_dgram_write;
		}
		dci.s2m_ans_pu_con_timeout = 0;
	}

	if (is_master(spidev)) {
		m2s_int_set_value(0);
        time_measure_end(&time_info_ack);
        if (time_measure_get_us(&time_info_ack) > 20*1000) {
            log_e("%d:download ack take too long %d us.", spidev->dn_selnum, time_measure_get_us(&time_info_ack));
        }
	}

	time_measure_begin(&time_info);
	if (wait_s2m_ans_pulldown() < 0) {
		log_e("%d:write end handshake failed!", spidev->dn_selnum);
		status = DL_STATUS_TIMEOUT;
		++dci.s2m_ans_pd_timeout;
		++dci.s2m_ans_pd_con_timeout;
		goto out_dgram_write;
	}

	//check slave status
	time_measure_end(&time_info);
	if (time_measure_get_us(&time_info) < DN_CHECKERR_THRESHOLD * 1000) {
		status = DL_STATUS_SUCCESS;
		spidev->is_first_dl_frame = 0;
		++dci.dn_frame_suc;
	} else {
		comm_frame_t *frame;
		log_e("%d:MCU reports a download packet check err! Take %d us.", spidev->dn_selnum, time_measure_get_us(&time_info));
		frame = (comm_frame_t *)spidev->send_frame;
		log_e("%d:frame header %d %d %d %d", spidev->dn_selnum, frame->selnum, frame->info, frame->size, frame->checksum);

		log_hex(LOG_ID_ERR, frame->packet, frame->size > 32 ? 32 : frame->size, "first 32B data");

		if(frame->size > 32) {
			log_hex(LOG_ID_ERR, frame->packet + frame->size - 32, 32, "last 32B data");
		}
		status = DL_STATUS_CHECK_ERR;
		++dci.dn_frame_check_err;
		goto out_dgram_write;
	}

	++spidev->dn_selnum;
out_dgram_write:
	//mutex_unlock(&spidev->dual_comm_lock);
out_dgram_write_nolock:
	m2s_int_set_value(0);
	return status;
}

static comm_dl_status_t
comm_master_dl_upload(void)
{
	comm_dl_status_t status;

	__pm_relax(&spidev->upload_wakesrc);
	down(&spidev->s2m_int_pu_sem);
	__pm_stay_awake(&spidev->upload_wakesrc);

	//mutex_lock(&spidev->dual_comm_lock);

	while(!down_trylock(&spidev->s2m_int_pu_sem))
		++dci.s2m_int_down_trylock;

	if (is_master(spidev)) {
		if (unlikely((0 == spidev->continuous_read))) {
			enable_continuous_read();
			spidev->continuous_read = 1;
		}
	}

	if (upload_spi_seq() < 0) {
		status = DL_STATUS_CHECK_ERR;
		++dci.up_read_seq_fail;
		goto out_dl_upload;
	}

	//end handshake
	if (is_master(spidev)) {
		m2s_ans_set_value(1);
		udelay(60);
	}

	if (PACKET_CHECK_SUMERR != check_frame_data(spidev)) {
		m2s_ans_set_value(0);//pull low ans before calling the function
		switch (check_and_get_data_from_ram()) {
		case PACKET_CHECK_SUCCESS:
			status = DL_STATUS_SUCCESS;
			spidev->is_first_up_frame = 0;
			++dci.up_frame_suc;
			break;
		case PACKET_CHECK_DISCONT:
			status = DL_STATUS_SUCCESS;
			++dci.up_frame_discontinuous;
			break;
		case PACKET_CHECK_SUMERR:
			status = DL_STATUS_CHECK_ERR;
			log_i("%d:!!!Error check sumerr should not appear!!!", spidev->up_selnum);
			break;
		case PACKET_CHECK_REPEAT:
			status = DL_STATUS_REPEAT;
			//++dci.up_frame_repeat;
			break;
		}
	} else {
		status = DL_STATUS_CHECK_ERR;
		++dci.up_frame_check_err;
	}

out_dl_upload:
	log_i("%d:upload status = %d", spidev->up_selnum, status);
	if ((DL_STATUS_CHECK_ERR == status) && get_sau_ch_id()) {
		m2s_ans_send_pulse(UP_CHECKERR_THRESHOLD * 1000 / 2 * 3);
	}
	//mutex_unlock(&spidev->dual_comm_lock);
	m2s_ans_set_value(0);//reset ap ans gpio
	return status;
}

/****************************************************
 * The following code is master app layer. (temp)
 ****************************************************/
#define PACKET_DOWNLOAD_MAX_TRY_TIMES   6
#define PACKET_HEAD_SIZE                4
#define PACKET_PAYLOAD_SIZE             (DL_MTU - PACKET_HEAD_SIZE)

typedef struct {
	uint16_t channel_id;
	uint16_t data_size;
	uint8_t  data[0];
} comm_packet_t;


static inline struct
cdev_info_list_unit *get_cdev_info_list_unit_by_channel_id(int channel_id)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(g_cdev_info_list); ++i) {
		if (channel_id == g_cdev_info_list[i].channel_id)
			return &g_cdev_info_list[i];
	}
	return NULL;
}

static struct data_cdev *get_cdev_info_by_channel_id(int channel_id)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(g_cdev_info_list); ++i) {
		if (channel_id == g_cdev_info_list[i].channel_id)
			return g_cdev_info_list[i].cdev_info;
	}
	return g_cdev_info_list[0].cdev_info;  //avoid null pointer
}

static struct data_cdev *get_next_cdev_for_download(void)
{
	int i;

	__pm_relax(&spidev->download_wakesrc);
	down(&spidev->m2s_tx_fifo_sem);
	__pm_stay_awake(&spidev->download_wakesrc);

	//get_next_cdev_for_download()
	for (i = 0; i < ARRAY_SIZE(g_cdev_info_list); ++i) {
		if (!test_bit(STAND_ALONE, &g_cdev_info_list[i].flags)
			&& !kfifo_is_empty(&g_cdev_info_list[i].cdev_info->tx_fifo.fifo))
			return g_cdev_info_list[i].cdev_info;
	}
	return NULL;  //avoid null pointer
}



//fifo ops begin
static inline ssize_t
cdev_fifo_in_frame(struct comm_fifo *pfifo, const comm_packet_t *packet)
{
	int status = -ENOMEM;
	int len = 0;
	if (!pfifo || !packet) {
		log_e("Invalid argument pfifo or packet is NULL");
		return -EINVAL;
	}

	len = packet->data_size + PACKET_HEAD_SIZE;
	mutex_lock(&pfifo->lock);
	if (kfifo_avail(&pfifo->fifo) > len) {
		kfifo_in(&pfifo->fifo, packet, len);
		//up(&pfifo->packet_cnt_sem);
		status = len;
	}
	mutex_unlock(&pfifo->lock);
	return status;
}

/*need caller add lock*/
static inline int
cdev_fifo_out_frame(struct comm_fifo *pfifo, comm_packet_t *packet)
{
	int status = 0;
	int data_size = 0;

	if (!pfifo || !packet) {
		log_e("Invalid argument pfifo or packet is NULL");
		return -EINVAL;
	}

	mutex_lock(&pfifo->lock);
	//take data head
	status = kfifo_out(&pfifo->fifo, packet, PACKET_HEAD_SIZE);
	data_size = packet->data_size;

	if ((PACKET_HEAD_SIZE != status)
		|| (0 == data_size)
		|| (kfifo_out(&pfifo->fifo, packet->data, data_size) != data_size)){ //take data
		log_e("impossible situation occured %d,%d", status, data_size);
		//err solve
		log_e("reset_fifo and sem to avoid fatal err");
		kfifo_reset(&pfifo->fifo);
		while (!down_trylock(&pfifo->packet_cnt_sem));
		data_size = 0;
		status = -EFAULT;
	}
	mutex_unlock(&pfifo->lock);
	status += data_size;
	return status;
}
//fifo ops end

#define DIRECTION_M2S 0
#define DIRECTION_S2M 1
#define STANDARD_CHANNEL_INFO_DUMP 0
static inline void print_packet_info(void *data, size_t size, uint16_t channel_id, uint8_t direction)
{
	static int test_msgid[4] = {0};
	uint8_t cls = (uint8_t)((((uint16_t *)data)[0]) >> 8);
	uint8_t id = (uint8_t)((((uint16_t *)data)[0]) & 0xff);

	if (!log_array[LOG_ID_PACKET_INFO].sw)
		return;

	if ((channel_id == 1) || (channel_id == 7)) {
		if (((uint16_t *)data)[0] < 1024) {
			printk(KERN_INFO LOGTAG"%s (id=%d) size=%d\n",
				direction ? "S->M" : "M->S", ((uint16_t *)data)[0], ((uint16_t *)data)[1]);
		} else {
			//class24,id1,id2 is stress testing msg, lessen print log
			if ((cls == 24) && ((id == 1) || (id == 2))) {
				++test_msgid[id];
				if (test_msgid[id] > 99) {
					printk(KERN_INFO LOGTAG"%s (id=%d, class=%d) size=%d times=%d\n", direction ? "S->M" : "M->S",
						id, cls, ((uint16_t *)data)[1], test_msgid[id]);
					test_msgid[id] = 0;
				}
			} else {
				printk(KERN_INFO LOGTAG"%s (id=%d, class=%d) size=%d\n", direction ? "S->M" : "M->S",
					id, cls,((uint16_t *)data)[1]);
			}
		}
	}
	#if STANDARD_CHANNEL_INFO_DUMP
	else {
		printk(KERN_INFO LOGTAG"%s (%s) size=%d\n",
			direction ? "S->M" : "M->S", get_cdev_info_by_channel_id(channel_id)->dev_name, (int)size);
	}
	#endif
}

#ifdef COLLECT_WAKEUP_INFO
static void wake_up_data_save(uint8_t *data){

	mutex_lock(&wakeup_data_fifo_lock);
	if (kfifo_avail(&wakeup_data) < 2) {
		log_e("wakeup_data is full, cover it!");
	}
	kfifo_in(&wakeup_data, data, 2);
	if(kfifo_len(&wakeup_data) >= WAKEUP_DATA_LEN){
		wakeup_data_uevent();
	}
	mutex_unlock(&wakeup_data_fifo_lock);
}
#endif

static void distribute_packet_to_data_cdev(comm_packet_t *packet)
{
	struct cdev_info_list_unit *list_unit;
	struct data_cdev  *cdev = NULL;
	int               retry_cnt = 0;
	ssize_t           status = -ENOBUFS;
	int               fifo_len = 0;

	log_hex(LOG_ID_UP_SEQ, packet, (packet->data_size + PACKET_HEAD_SIZE) > 32 ?32:(packet->data_size + PACKET_HEAD_SIZE), "to SDM");

#ifdef SPEEDTEST
	if (packet->channel_id == SPEEDTEST_DATA_CHANNEL_ID)
		return;
	if (packet->channel_id == SPEEDTEST_CTRL_CHANNEL_ID) {
		memcpy(upload_speed_string, packet->data, 512);
		upload_speed_string_ready = 1;
		return;
	}
#endif

	//cdev = get_cdev_info_by_channel_id(packet->channel_id);
	list_unit = get_cdev_info_list_unit_by_channel_id(packet->channel_id);

	if (!list_unit || !list_unit->cdev_info) {
		log_e("could not match data cdev!");
		return;
	}
	cdev = list_unit->cdev_info;

#ifdef STAND_ALONE_USED
	if (is_sau_filter(cdev->channel_id)) {
		log_e("ch(%d) stand alone used, ch(%d) data abandoned",
			get_sau_ch_id(), cdev->channel_id);
		return;
	}
#endif

#ifdef DATA_HOOK_EN
	if (cdev->hook_process) {
		if (cdev->users) {
			struct data_hook *hook_process = cdev->hook_process;
			log_hex(LOG_ID_HOOK_DATA_RECV, packet, packet->data_size + PACKET_HEAD_SIZE, "[channel id %d]", packet->channel_id);
			spi_ping_res_print(packet->data, packet->data_size);
			hook_process->hook_data_ops->read_cb(hook_process->hook_data, packet->data, packet->data_size);
		} else {
			log_i("hook is not open");
		}
		return;
	}
#endif

	log_hex(LOG_ID_PACKET_TO_FIFO, packet, packet->data_size + PACKET_HEAD_SIZE, "to %s", cdev->dev_name);

#ifdef PRINT_WAKEUP_INFO
	if (spidev->print_wakeup) {
		spidev->print_wakeup = 0;
		if (pm_wakeup_irq() == spidev->s2m_int_irq) {
#ifdef COLLECT_WAKEUP_INFO
			if(test_bit(CDEV_WAKEUP, &list_unit->flags)){
				wake_up_data_save( (uint8_t *) (packet->data));//save wakeup msg_id
			}
#endif
			log_e("wakeup by packet: channel(%s) size(%d)", cdev->dev_name, packet->data_size);
			print_hex_dump(KERN_ERR, LOGTAG, DUMP_PREFIX_OFFSET, 16, 1, (uint8_t *)packet,
				PACKET_HEAD_SIZE + packet->data_size > 16 ? 16 : PACKET_HEAD_SIZE + packet->data_size, 0);
		}
	}
#endif

//push packet to rx_fifo
#define USER_READ_MAX_TRY_TIMES        (6)
#define CHANNEL_H_READ_WAIT_TIMES_MS        (200)
#define CHANNEL_O_READ_WAIT_TIMES_MS        (20)
	do {
		if (test_bit(CDEV_BIG_FIFO, &list_unit->flags)) {
			fifo_len = kfifo_len(&cdev->rx_fifo.fifo);
			/*
			 * msleep 50ms for wait read when fifo used over 80%, the framework consume about 4ms
			 * to process each frame of data
			 */
			if (fifo_len  > (SNSHUB_KFIFO_SIZE*80)/100 ) {
				log_e("fifo_len size(%d) almost full sleep 50 ms", fifo_len);
				msleep(50);
			}
		}

		status = cdev_fifo_in_frame(&cdev->rx_fifo, packet);
		if ( status > 0) {
			up(&cdev->rx_fifo.packet_cnt_sem);
			__pm_wakeup_event(&cdev->wakesrc, 1000);
			++cdev->cci.rx_packet_to_fifo;
			status = 0;
			break;
		}

		++cdev->cci.rx_fifo_full;
		if (cdev->cci.rx_fifo_full < 10 || cdev->cci.rx_fifo_full % 100 == 0)
			log_e("rx_fifo of %s is full, try(%d), %d,%d\n", \
				cdev->dev_name, retry_cnt, cdev->cci.rx_fifo_full, cdev->cci.rx_packet_to_fifo);
		msleep((1 == cdev->channel_id)?CHANNEL_H_READ_WAIT_TIMES_MS:CHANNEL_O_READ_WAIT_TIMES_MS);
	} while (++retry_cnt < USER_READ_MAX_TRY_TIMES);

	if ((0 != status) || (cdev->cci.rx_fifo_in_failed)){
		if (0 != status) {
			if (0 == cdev->cci.rx_fifo_in_failed) {
				log_e("rx_fifo of %s, a frame abandoned!", cdev->dev_name);
			}
			cdev->cci.rx_fifo_in_failed++;
		} else {
			if (cdev->cci.rx_fifo_in_failed > 1) {
				log_i("rx_fifo of %s, total %d frame abandoned!", cdev->dev_name, cdev->cci.rx_fifo_in_failed);
			}
			cdev->cci.rx_fifo_in_failed = 0;
		}
	}
	return;
}

#define PACKET_DOWNLOAD_MAX_TRY_TIMES   6
/*Note need to hold the lock before calling*/
static inline int comm_master_download_try(void)
{
	comm_packet_t *packet = frame_packet(spidev->send_frame);
	int try_cnt = 0, check_err_cnt = 0;
	if (unlikely((packet->data_size)
		> (spidev->tx_buf_size - SLAVE_SKIP_LEN - DL_FRAME_HEAD_SIZE - PACKET_HEAD_SIZE))) {
		log_e("send data(%d) too long.", (packet->data_size + PACKET_HEAD_SIZE));
	}

	log_hex(LOG_ID_PACKET_TO_MCU, packet, packet->data_size + PACKET_HEAD_SIZE, "channel id %d", packet->channel_id);
	do {
		comm_dl_status_t dl_status = comm_master_dl_download(packet, packet->data_size + PACKET_HEAD_SIZE);
		if (DL_STATUS_SUCCESS == dl_status) {
			if (try_cnt)
				log_w("packet send successfully, but try %d times.", try_cnt);
			break;
		}
		++try_cnt;
		if (close_comm_state & 0x01) {
			log_e("download_comm stop, try%d", try_cnt);
			break;
		}
		if (DL_STATUS_CHECK_ERR == dl_status) {
			++check_err_cnt;
		} else if (DL_STATUS_TIMEOUT == dl_status) {
			if (!is_master(spidev)) {
				log_w("packet send timeout, need sleep %dms.", 50*try_cnt);
				msleep(50*try_cnt);
			}
		}
	} while (try_cnt < PACKET_DOWNLOAD_MAX_TRY_TIMES);

	if (PACKET_DOWNLOAD_MAX_TRY_TIMES <= try_cnt) {
		log_e("packet send failed, try %d times!", try_cnt);
		if (PACKET_DOWNLOAD_MAX_TRY_TIMES <= check_err_cnt) {
			mcu_states_crash();
		}
		return -1;
	}

	save_tomcudata((uint8_t*)packet, packet->data_size + PACKET_HEAD_SIZE);

	return 0;
}

#ifdef SPEEDTEST
static int comm_master_download(const void *buf, size_t size, uint16_t channel_id)
{
	comm_packet_t *packet = frame_packet(spidev->send_frame);

#ifdef STAND_ALONE_USED
	if (is_sau_filter(channel_id)) {
		return -EINVAL;
	}
#endif

	if (size > PACKET_PAYLOAD_SIZE) {
		return -EINVAL;
	}

	packet->channel_id = channel_id;
	packet->data_size = size;
	memcpy(packet->data, buf, size);

	return comm_master_download_try();
}
#endif

/*read_thread*/
static int comm_master_upload_thread(void *arg)
{
	comm_packet_t *packet = (comm_packet_t *)(spidev->recv_frame + DL_FRAME_HEAD_SIZE);
	int con_recv_fail_times = 0;
	struct sched_param param = {.sched_priority = MAX_RT_PRIO / 2};
	comm_dl_status_t dl_status = DL_STATUS_SUCCESS;
	if (sched_setscheduler(spidev->snshub_read_tsk, SCHED_FIFO, &param))
		log_e("sched_setscheduler upload SCHED_FIFO failed!!!");
	log_i("upload thread configed: pid=%d policy=%d priority=%d",spidev->snshub_read_tsk->pid,
		spidev->snshub_read_tsk->policy, spidev->snshub_read_tsk->prio);
	if (gpio_get_value(spidev->s2m_int_gpio) == 1) {
		log_e("s2m_int is high, immediately trigger");
		up(&spidev->s2m_int_pu_sem);
	}

	while (1) {
		dl_status = comm_master_dl_upload();

		while (close_comm_state & 0x02){
			log_e("upload_thread stop");
			msleep(5000);
		}

		if (dl_status == DL_STATUS_SUCCESS) {
			distribute_packet_to_data_cdev(packet);
			if (con_recv_fail_times) {
				log_e("received a packet, size(%d) channel(%s), "
					"before this packet, %d packet received failed!",
					packet->data_size, get_cdev_info_by_channel_id(packet->channel_id)->dev_name, con_recv_fail_times);
				con_recv_fail_times = 0;
			}
			//spare 300ms for dcc
			__pm_wakeup_event(&spidev->resever_wakesrc, 300);
		} else if (dl_status == DL_STATUS_REPEAT) {
			log_n("a packet repeated!");
		} else {
			if((++con_recv_fail_times) > 30){
				log_e("packet received failed(continuous times:%d)!", con_recv_fail_times);
				mcu_states_crash();
				con_recv_fail_times = 0;
			}
		}

	}
	return 0;
}


/*write_thread*/
static int comm_master_download_thread(void *arg)
{
	comm_packet_t *packet = frame_packet(spidev->send_frame);
	int status = 0;
	struct data_cdev  *cdev = NULL;
	struct sched_param param = {.sched_priority = ((MAX_RT_PRIO / 2) + 2) };
	if (sched_setscheduler(spidev->snshub_write_tsk, SCHED_FIFO, &param))
		log_e("sched_setscheduler download SCHED_FIFO failed!!!");
	log_i("download thread configed: pid=%d policy=%d priority=%d",spidev->snshub_write_tsk->pid,
		spidev->snshub_write_tsk->policy, spidev->snshub_write_tsk->prio);

	mutex_lock(&spidev->write_lock);
	while (1) {
		mutex_unlock(&spidev->write_lock);
		//change thread
		if (gpio_get_value(spidev->s2m_int_gpio) == 1) {
			msleep(1);
		}

		cdev = get_next_cdev_for_download();

		mutex_lock(&spidev->write_lock);
		if (NULL == cdev) {
			log_e("could not match data cdev! m2s_tx_fifo_sem semaphore error!!!");
			//msleep(10);
			continue;
		}

		status = 0;
		memset(spidev->spi_tx_buffer, 0, spidev->tx_buf_size);

		//get packet from tx_fifo
		status = cdev_fifo_out_frame(&cdev->tx_fifo , packet);
		//data get error so do next
		if (status < 0) {
			continue;
		}

		if (close_comm_state & 0x01){
			log_e("download_thread stop, a frame of data be discarded.");
			continue;
		}

#ifdef STAND_ALONE_USED
		if (is_sau_filter(cdev->channel_id)) {
			log_e("ch(%d) stand alone used, ch(%d) data not send",
				get_sau_ch_id(), cdev->channel_id);
			continue;
		}
#endif

		//send data to mcu
		++cdev->cci.tx_packet_to_mcu;
		if (comm_master_download_try() == 0) {
			int data_size = packet->data_size;
			print_packet_info((packet->data), data_size, packet->channel_id, DIRECTION_M2S);
			++cdev->cci.tx_packet_send_suc;
		} else {
			status = -EFAULT;
			++cdev->cci.tx_packet_send_fail;
		}
	}
	mutex_unlock(&spidev->write_lock);
	return 0;
}


#ifdef DATA_HOOK_EN
static ssize_t
rfifo_in(struct data_cdev *read_info, bool nonblock)
{
	comm_packet_t       *packet;
	int                 status = 0;
	int                 data_size = 0;
	int                 retry_cnt = 0;

	if (!read_info) {
		log_e("wrtie_info is NULL");
		return -EFAULT;
	}

	packet = (comm_packet_t *)read_info->read_packet_area;
	data_size = packet->data_size;

#define USER_WRITE_MAX_TRY_TIMES        (4)
#define CHANNEL_H_WAIT_TIMES_MS         (300)
#define CHANNEL_O_WAIT_TIMES_MS         (30)  //O is other
	retry_cnt = 0;
	do {
		++retry_cnt;
		status = cdev_fifo_in_frame(&read_info->rx_fifo, packet);
		if ( status > 0) {
			__pm_wakeup_event(&read_info->wakesrc, 1000);
			up(&read_info->rx_fifo.packet_cnt_sem);
			wake_up(&read_info->waitq);
			++read_info->cci.rx_packet_to_fifo;
			status = data_size;
			break;
		}

		++read_info->cci.rx_fifo_full;

		if (((USER_WRITE_MAX_TRY_TIMES == retry_cnt) && (read_info->cci.rx_fifo_full < 10))
			|| (read_info->cci.rx_fifo_full % 10 == 0)) {
			log_e("tx_fifo of %s is full, a frame will be abandoned, try(%d), %d,%d,%d\n",
				read_info->dev_name, retry_cnt, read_info->cci.rx_fifo_full,
				read_info->cci.rx_packet_to_fifo, read_info->cci.rx_packet_to_user);
		}

		if (nonblock) {  //nio
			log_i("use nio but not cache buf");
			break; //goto out_spidev_write;
		}
		msleep((1 == read_info->channel_id)?CHANNEL_H_WAIT_TIMES_MS:CHANNEL_O_WAIT_TIMES_MS);
	} while(retry_cnt < USER_WRITE_MAX_TRY_TIMES);
	return status;
}

static int copy_to_rfifo (void *pdev , const void *buf, const int count)
{
	int                 status = 0;
	struct data_cdev    *read_info = (struct data_cdev*)pdev;
	comm_packet_t  *packet;

	if ((NULL == buf) || (0 == count)) {
		log_e("buf is NULL or count is %d \n", count);
		return -EINVAL;
	}

	h5_ping_res_print(buf, count);

	if (NULL == read_info) {
		log_e("read_info uninit\n");
		return -EINVAL;
	}
	packet = (comm_packet_t *)read_info->read_packet_area;

	mutex_lock(&read_info->read_lock);

	memset(packet, 0, PACKET_HEAD_SIZE + PACKET_PAYLOAD_SIZE);
	packet->channel_id = read_info->channel_id;
	packet->data_size = count;
	memcpy((packet->data), buf, count);
	log_hex(LOG_ID_PACKET_TO_FIFO, (packet->data), count, "%s copy_to_rfifo", read_info->dev_name);

	status = rfifo_in(read_info, 0/*!O_NONBLOCK*/);

	mutex_unlock(&read_info->read_lock);
	return status;
}

#endif


static int
rfifo_out(struct data_cdev *read_info, bool nonblock)
{
	struct spidev_data  *spidev;
	comm_packet_t  *packet;
	//int               status = 0;

	if (!read_info) {
		log_e("read_info is NULL");
		return -EFAULT;
	}

	spidev = read_info->spidev;
	packet = (comm_packet_t *)read_info->read_packet_area;

	memset(packet, 0, PACKET_HEAD_SIZE + PACKET_PAYLOAD_SIZE);

	//wait when conditions
	if (down_trylock(&read_info->rx_fifo.packet_cnt_sem)) {  //can't get semaphore
		__pm_relax(&read_info->wakesrc);
		if (nonblock) {  //nio
			log_i("use nio but no data to read");
			//goto out_cdev_read;
			return 0;
		} else {  //bio
		//wait when no data to read,
		//inter "if" when interrupted by signal
			if (down_interruptible(&read_info->rx_fifo.packet_cnt_sem)) {
				//log_i("rx_fifo_packet_cnt_sem interrupted by signal");
				return -EINTR;
			}
		}
	}

	return cdev_fifo_out_frame(&read_info->rx_fifo, packet);
}
#if 0
ssize_t copy_from_rfifo (void *pdev, char *buf, size_t count)
{
	int         result = count;

	struct data_cdev    *read_info = (struct data_cdev*)pdev;
	comm_packet_t       *packet = (comm_packet_t *)read_info->read_packet_area;

	mutex_lock(&read_info->read_lock);

	if (0 == read_info->left_size) {
		if (rfifo_out(read_info, false/*!O_NONBLOCK*/) > 0) {
			read_info->offset = 0;
			read_info->left_size = packet->data_size;
		}
	}
	result = read_info->left_size;
	if (read_info->left_size) {
		if (unlikely(count < read_info->left_size)) {
			log_e("count%d < left_size%d, need to slice", count, read_info->left_size);
			result = count;
		}

		read_info->left_size -= result;
		read_info->offset += result;

		memcpy(buf, read_info->read_packet_area + sizeof(comm_packet_t) + read_info->offset, result);
		log_hex(LOG_ID_DATA_TO_USER, buf, result, "%s", read_info->dev_name);
	}
	mutex_unlock(&read_info->read_lock);
	return result;
}

EXPORT_SYMBOL(copy_from_rfifo);
#endif

static ssize_t
spidev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int                 status = 0;
	unsigned long       missing = 0;
	int                 data_size;


	//static uint8_t packet_area[PACKET_HEAD_SIZE + PACKET_PAYLOAD_SIZE] = {0};
	//comm_packet_t *packet = (comm_packet_t *)packet_area;
	struct data_cdev    *read_info = filp->private_data;
	comm_packet_t       *packet = (comm_packet_t *)read_info->read_packet_area;

	mutex_lock(&read_info->read_lock);

	status = rfifo_out(read_info, filp->f_flags & O_NONBLOCK);

	//copy data to user
	if (status > 0) {
		data_size = packet->data_size;
		if (count < data_size) {
			log_e("missing some data (%d bytes) to user because count < data_size", (data_size-count));
			data_size = count;
		}

		if (10 == read_info->channel_id) {
			user_ping_res_print(packet->data, data_size);
		}

		missing = copy_to_user(buf, packet->data, data_size);
		if (!missing) {
			status = data_size;
			print_packet_info(packet->data, data_size, read_info->channel_id, DIRECTION_S2M);
			++read_info->cci.rx_packet_to_user;
		} else {
			log_e("copy_to_user missing some data");
			status = -EFAULT;
		}
		log_hex(LOG_ID_DATA_TO_USER, packet->data, packet->data_size, "%s", read_info->dev_name);
	}
	mutex_unlock(&read_info->read_lock);

	return status;
}

static ssize_t
wfifo_in(struct data_cdev *wrtie_info, bool nonblock)
{
	struct spidev_data  *spidev;
	comm_packet_t  *packet;
	int                 status = 0;
	int                 data_size = 0;
	int                 retry_cnt = 0;


	if (!wrtie_info) {
		log_e("wrtie_info is NULL");
		return -EFAULT;
	}

	packet = (comm_packet_t *)wrtie_info->write_packet_area;
	spidev = wrtie_info->spidev;
	data_size = packet->data_size;

#define USER_WRITE_MAX_TRY_TIMES        (4)
#define CHANNEL_H_WAIT_TIMES_MS         (300)
#define CHANNEL_O_WAIT_TIMES_MS         (30)  //O is other
	retry_cnt = 0;
	do {
		++retry_cnt;
		status = cdev_fifo_in_frame(&wrtie_info->tx_fifo, packet);
		if ( status > 0) {
			++wrtie_info->cci.tx_packet_to_fifo;
			status = data_size;
			up(&spidev->m2s_tx_fifo_sem);
			break;
		}

		++wrtie_info->cci.tx_fifo_full;

		if (((USER_WRITE_MAX_TRY_TIMES == retry_cnt) && (wrtie_info->cci.tx_fifo_full < 10))
			|| (wrtie_info->cci.tx_fifo_full % 10 == 0)) {
			log_e("tx_fifo of %s is full, a frame will be abandoned, try(%d), %d,%d,%d",
				wrtie_info->dev_name, retry_cnt, wrtie_info->cci.tx_fifo_full,
				wrtie_info->cci.tx_packet_to_fifo, wrtie_info->cci.tx_packet_to_mcu);
		}

		if (nonblock) {  //nio
			log_i("use nio but not cache buf");
			break; //goto out_spidev_write;
		}
		msleep((1 == wrtie_info->channel_id)?CHANNEL_H_WAIT_TIMES_MS:CHANNEL_O_WAIT_TIMES_MS);
	} while(retry_cnt < USER_WRITE_MAX_TRY_TIMES);
	return status;
}

#ifdef DATA_HOOK_EN

static int copy_to_wfifo (void *pdev , const void *buf, const int count)
{
	int                 status = 0;
	struct data_cdev    *wrtie_info = (struct data_cdev*)pdev;
	comm_packet_t  *packet;

	if (NULL == wrtie_info) {
		log_e("wrtie_info uninit");
		return -EINVAL;
	}
	packet = (comm_packet_t *)wrtie_info->write_packet_area;

	mutex_lock(&wrtie_info->write_lock);

	memset(packet, 0, PACKET_HEAD_SIZE + PACKET_PAYLOAD_SIZE);
	packet->channel_id = wrtie_info->channel_id;
	packet->data_size = count;
	memcpy((packet->data), buf, count);
	log_hex(LOG_ID_HOOK_DATA_SEND, (packet->data), count, "%s copy_to_wfifo", wrtie_info->dev_name);
	h5_ping_req_print(packet->data, count);
	status = wfifo_in(wrtie_info, 0/*!O_NONBLOCK*/);

	mutex_unlock(&wrtie_info->write_lock);
	return status;
}

#endif

#ifdef STAND_ALONE_USED
static ssize_t spidev_sau_write(struct data_cdev *cdev, const char __user *buf, size_t count)
{
	struct spidev_data *spidev = cdev->spidev;
	comm_packet_t *packet = frame_packet(spidev->send_frame);
	unsigned long           missing;
	ssize_t                 status = -ENOBUFS;

	if (count > spidev->tx_buf_size - SLAVE_SKIP_LEN) {
		log_e("write data length(%d) > (%d) ", count, spidev->tx_buf_size);
		return -EINVAL;
	}

	mutex_lock(&spidev->write_lock);

	packet->channel_id = cdev->channel_id;
	packet->data_size = count;

	missing = copy_from_user((packet->data), buf, count);
	log_hex(LOG_ID_DATA_FROM_USER, (packet->data), count, "%s", cdev->dev_name);
	if (missing) {
		log_e("copy from user missing");
		status = -EFAULT;
		goto out_spidev_sau_write;
	}

	status = comm_master_download_try();

out_spidev_sau_write:
	mutex_unlock(&spidev->write_lock);
	return status?status:count;
}
#endif

static ssize_t spidev_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *f_pos)
{
	struct data_cdev        *wrtie_info;
	ssize_t                 status = -ENOBUFS;
	unsigned long           missing;
	//static DEFINE_MUTEX(packet_area_mutex);
	//static uint8_t packet_area[PACKET_HEAD_SIZE + PACKET_PAYLOAD_SIZE] = {0};
	//comm_packet_t *packet = (comm_packet_t *)packet_area;
	comm_packet_t *packet = NULL;

	wrtie_info = filp->private_data;

	if (unlikely(!wrtie_info)) {
		log_e("private_data is NULL");
		return -EINVAL;
	}

#ifdef STAND_ALONE_USED
	if (unlikely(0 != get_sau_ch_id())) {
		if (get_sau_ch_id() != wrtie_info->channel_id) {
			log_e("ch(%d) write data length(%d) failed, because ch(%d) stand alone used",
				wrtie_info->channel_id, count, get_sau_ch_id());
			return count;
		} else if (is_master(wrtie_info->spidev)){
			return spidev_sau_write(wrtie_info, buf, count);
		}
	}
#endif
	if (unlikely(count > PACKET_PAYLOAD_SIZE)) {
		log_e("write data length(%d) more then max count(PACKET_PAYLOAD_SIZE)", count);
		return -EINVAL;
	}

	packet = (comm_packet_t *)wrtie_info->write_packet_area;
	mutex_lock(&wrtie_info->write_lock);
	memset(packet, 0, PACKET_HEAD_SIZE + PACKET_PAYLOAD_SIZE);
	packet->channel_id = wrtie_info->channel_id;
	packet->data_size = count;

	missing = copy_from_user((packet->data), buf, count);
	log_hex(LOG_ID_DATA_FROM_USER, (packet->data), count, "%s", wrtie_info->dev_name);
	if (unlikely(missing)) {
		log_e("copy from user missing");
		status = -EFAULT;
		goto out_spidev_write;
	}

	if (wrtie_info->hook_process) {
		struct data_hook *hook_process = wrtie_info->hook_process;
		//log_e("call hook_process->hook_data_ops->write");
		save_fromuserdata(packet->data, packet->data_size);
		user_ping_req_print(packet->data, packet->data_size);
		status = hook_process->hook_data_ops->write(hook_process->hook_data,
			(void *)(packet->data), packet->data_size);
		//log_e("hook_process->hook_data_ops->write return %d", status);
		if (status != packet->data_size) {
			status = -EFAULT;
		}
	} else {
		status = wfifo_in(wrtie_info, filp->f_flags & O_NONBLOCK);
	}

out_spidev_write:
	mutex_unlock(&wrtie_info->write_lock);

	//mutex_unlock(&packet_area_mutex);
	return status;
}

static ssize_t spidev_write_iter(struct kiocb *iocb, struct iov_iter *from)
{
	struct file *file = iocb->ki_filp;
	struct data_cdev *wrtie_info = file->private_data;
	ssize_t status = -EFAULT;

	mutex_lock(&wrtie_info->write_lock);

	log_i("channel_id %d, from user", wrtie_info->channel_id);

#ifdef STAND_ALONE_USED
	if (unlikely(0 != get_sau_ch_id())
		&& (get_sau_ch_id() != wrtie_info->channel_id)) {
		size_t count = iov_iter_count(from);
		log_e("ch(%d) write data length(%d) failed, because ch(%d) stand alone used",
			wrtie_info->channel_id, count, get_sau_ch_id());
		return count;
	}
#endif

	if (wrtie_info->hook_process) {
		struct data_hook *hook_process = wrtie_info->hook_process;
		//log_e("call hook_process->hook_data_ops->write");
		status = hook_process->hook_data_ops->write_iter(hook_process->hook_data, from);
		//log_e("hook_process->hook_data_ops->write return %d", status);
	} else {
		log_e("not support writev");
	}
	mutex_unlock(&wrtie_info->write_lock);
	return status;
}


static __poll_t spidev_poll(struct file *filp, poll_table *wait)
{
	struct data_cdev        *cdev_info;
	ssize_t                 mask = 0;

	cdev_info = filp->private_data;

	/*将等待队列添加到poll_table */
	poll_wait(filp, &cdev_info->waitq, wait);

	if (kfifo_len(&cdev_info->rx_fifo.fifo))
		mask |= POLLIN | POLLRDNORM; /* readable */

	return mask;
}

static int spidev_open(struct inode *inode, struct file *filp)
{
	struct cdev_info_list_unit *list_unit;
	struct data_cdev    *data_cdev;
	int            status = -ENXIO;
	int i;

	for (i = 0; i < ARRAY_SIZE(g_cdev_info_list); ++i) {
		if (g_cdev_info_list[i].cdev_info->devt == inode->i_rdev) {
			status = 0;
			list_unit = &g_cdev_info_list[i];
			break;
		}
	}

	if (status) {
		log_e("spidev: nothing for minor %d", iminor(inode));
		goto err_find_dev;
	}

	data_cdev = list_unit->cdev_info;

	if (0 == data_cdev->users) {
#ifdef STAND_ALONE_USED
		if (test_bit(STAND_ALONE, &list_unit->flags)) {
			set_sau_ch_id(data_cdev->channel_id);
			oplus_sensorhub_set_disable_uevent(1);
			if (is_master(spidev)) {
				spidev->tx_buf_size = PHY_MST_TX_BUFF_SIZE - 8;
			}
		}
#endif
#ifdef DATA_HOOK_EN
	do {
		struct data_hook *hook_process;
		hook_process = data_cdev->hook_process;
		if (hook_process) {
			hook_process->hook_data_ops->open(hook_process->hook_data);
		}
	} while(0);
#endif
	}

	data_cdev->users++;
	filp->private_data = data_cdev;
	nonseekable_open(inode, filp);
	return 0;
err_find_dev:
	return status;
}

static int spidev_release(struct inode *inode, struct file *filp)
{
	struct data_cdev *data_cdev = filp->private_data;

	if (1 == data_cdev->users) {
#ifdef STAND_ALONE_USED
		if (get_sau_ch_id() == data_cdev->channel_id) {
			set_sau_ch_id(0);
			oplus_sensorhub_set_disable_uevent(0);
			spidev->tx_buf_size = PHY_BUFF_SIZE;
		}
#endif
#ifdef DATA_HOOK_EN
		do {
			struct data_hook *hook_process;
			hook_process = data_cdev->hook_process;
			if (hook_process) {
				hook_process->hook_data_ops->close(hook_process->hook_data);
			}
		} while(0);
#endif
	}

	if (data_cdev->users) {
		data_cdev->users--;
	}

	return 0;
}

static const struct file_operations spidev_fops = {
	.owner      = THIS_MODULE,
	.write      = spidev_write,
	.read       = spidev_read,
	.write_iter = spidev_write_iter,
	.open       = spidev_open,
	.release    = spidev_release,
	.poll       = spidev_poll,
	.llseek =    no_llseek,
};

#ifdef DATA_HOOK_EN
static const struct hook_ops hook_data_ops = {
	//.open    = ,
	//.close   = ,
	.write   = copy_to_wfifo,
	.read_cb = copy_to_rfifo,
};
#endif

/******************************************************************************/
static ssize_t
log_show(struct class *class, struct class_attribute *attr, char *buf)
{
	int i;
	ssize_t count = 0;
	for (i = 0; i < 32; ++i) {
		if (log_array[i].name) {
			if (i % 3 == 0)
				count += snprintf(buf + count, PAGE_SIZE-count, "\n");
			count += snprintf(buf + count, PAGE_SIZE-count,
				"bit %d, sw %d [%s]\n", i, log_array[i].sw, log_array[i].name);
		}
	}
	count += snprintf(buf + count, PAGE_SIZE-count,
				"\ninput OCT value to change log level\n");
	return count;
}

static ssize_t
log_store(struct class *class, struct class_attribute *attr,
		const char *buf, size_t count)
{
	char *end;
	int i;
	uint32_t new = simple_strtol(buf, &end, 0);
	if (end == buf)
		return -EINVAL;
	for (i = 0; i < 32; ++i) {
		log_array[i].sw = ((new >> i) & 1) ? 1 : 0;
	}

	return count;
}
#ifndef STAND_ALONE_CLASS
static CLASS_ATTR_RW(log);
#endif

static ssize_t
info_show(struct class *class, struct class_attribute *attr, char *buf)
{
	int i;
	ssize_t count = 0;

	//gpio_info
	count += snprintf(buf + count, PAGE_SIZE-count,
		"--gpio info:\n"
		" m2s_int: gpio(%d) value(%d) output\n"
		" s2m_ans: gpio(%d) value(%d) irq(%d)\n"
		" s2m_int: gpio(%d) value(%d) irq(%d)\n"
		" m2s_ans: gpio(%d) value(%d) output\n\n",
		spidev->m2s_int_gpio, gpio_get_value(spidev->m2s_int_gpio),
		spidev->s2m_ans_gpio, gpio_get_value(spidev->s2m_ans_gpio), spidev->s2m_ans_irq,
		spidev->s2m_int_gpio, gpio_get_value(spidev->s2m_int_gpio), spidev->s2m_int_irq,
		spidev->m2s_ans_gpio, gpio_get_value(spidev->m2s_ans_gpio));

	//about download(write)
	count += snprintf(buf + count, PAGE_SIZE-count,
		"--download datalink(write) info:\n"
		" s2m_ans_pu_timeout: %d\n" " s2m_ans_pu_con_timeout: %d\n" " dn_write_seq_fail: %d\n"
		" s2m_ans_pd_timeout: %d\n" " dn_frame_suc: %d\n" " dn_frame_check_err: %d\n\n",
		dci.s2m_ans_pu_timeout, dci.s2m_ans_pu_con_timeout, dci.dn_write_seq_fail,
		dci.s2m_ans_pd_timeout, dci.dn_frame_suc, dci.dn_frame_check_err);

	//about upload(read)
	count += snprintf(buf + count, PAGE_SIZE-count,
		"--upload datalink(read) info:\n"
		" up_frame_discontinuous: %d\n" " up_frame_check_err: %d\n"
		" up_frame_suc: %d\n" " up_disturb_signal: %d\n"
		" up_read_seq_fail: %d\n" " s2m_int_down_trylock: %d\n\n",
		dci.up_frame_discontinuous, dci.up_frame_check_err,
		dci.up_frame_suc, dci.up_disturb_signal,
		dci.up_read_seq_fail, dci.s2m_int_down_trylock);

	//about channel
	for (i = 0; i < ARRAY_SIZE(g_cdev_info_list); ++i) {
		count += snprintf(buf + count, PAGE_SIZE-count,
		"--channel %s info:\n"
		" rx_fifo_full: %d\n" " rx_packet_to_fifo: %d\n" " rx_packet_to_user: %d\n"
		" tx_fifo_full: %d\n" " tx_packet_to_fifo: %d\n" " tx_packet_to_mcu: %d\n"
		" tx_packet_send_suc: %d\n" " tx_packet_send_fail: %d\n\n",
		g_cdev_info_list[i].cdev_info->dev_name,
		g_cdev_info_list[i].cdev_info->cci.rx_fifo_full,
		g_cdev_info_list[i].cdev_info->cci.rx_packet_to_fifo,
		g_cdev_info_list[i].cdev_info->cci.rx_packet_to_user,
		g_cdev_info_list[i].cdev_info->cci.tx_fifo_full,
		g_cdev_info_list[i].cdev_info->cci.tx_packet_to_fifo,
		g_cdev_info_list[i].cdev_info->cci.tx_packet_to_mcu,
		g_cdev_info_list[i].cdev_info->cci.tx_packet_send_suc,
		g_cdev_info_list[i].cdev_info->cci.tx_packet_send_fail);
	}

	return count;
}

static ssize_t
info_store(struct class *class, struct class_attribute *attr,
		const char *buf, size_t count)
{
	int i;
	if (strncmp(buf, "clean", strlen("clean")) == 0) {
		memset(&dci, 0, sizeof(dci));
		for (i = 0; i < ARRAY_SIZE(g_cdev_info_list); ++i)
			memset(&(g_cdev_info_list[i].cdev_info->cci), 0,
					sizeof((g_cdev_info_list[i].cdev_info->cci)));
		return count;
	}
	return -EINVAL;
}
#ifndef STAND_ALONE_CLASS
static CLASS_ATTR_RW(info);
#endif

#ifdef SPEEDTEST
struct kthread_parm{
	struct task_struct  *speed_tsk;
	ssize_t             count;
	char                *buf;
};
static int speed_test_complt = 0;

//return speed (Byte/s)
static int speedtest_send_data(uint16_t size, uint16_t times)
{
	struct time_measure_info info;
	static uint8_t buf[4088];
	int i;
	memset(buf, 0xA5, sizeof(buf));
	time_measure_begin(&info);
	for (i = 0; i < times; ++i)
		comm_master_download(buf, size, SPEEDTEST_DATA_CHANNEL_ID);
	time_measure_end(&info);
	return (size * times) / (time_measure_get_us(&info) / (1000 * 100)) * 10;
}

static int speedtest_thread(void *arg)
{
	int upload_speed_timeout = 60;
	struct kthread_parm *kthread = arg;
	int packet_size = 0;
	int speed = 0;
	int i;
	struct sched_param param = {.sched_priority = MAX_RT_PRIO / 2};
	if (sched_setscheduler(kthread->speed_tsk, SCHED_FIFO, &param))
		log_e("sched_setscheduler speedtest SCHED_FIFO failed!!!");
	log_i("speedtest thread configed: pid=%d policy=%d priority=%d",kthread->speed_tsk->pid,
		kthread->speed_tsk->policy, kthread->speed_tsk->prio);

	mutex_lock(&spidev->write_lock);

	//download
	for (i = 1; i <= 3; ++i) {
		switch (i) {
		case 1:
			packet_size = 100;
			break;
		case 2:
			packet_size = 1024;
			break;
		case 3:
			packet_size = 4080 - 8;//-7Byte
			break;
		}
		log_n("download test (size %d) is about to begin", packet_size);
		speed = speedtest_send_data(packet_size, 2000);
		kthread->count += snprintf(kthread->buf + kthread->count, PAGE_SIZE - kthread->count,
			"download: packet_size(%d), speed(%dB/s), frequency(%dHz)\n",
			packet_size, speed, speed / packet_size);
		log_n("download size %d test finished", packet_size);
	}

	//upload
	upload_speed_string_ready = 0;
	comm_master_download("0", 1, SPEEDTEST_CTRL_CHANNEL_ID);

	log_n("upload test is about to begin");
	while ((!upload_speed_string_ready)&&(upload_speed_timeout)) {
		msleep_interruptible(1000);
		upload_speed_timeout--;
	}

	if (upload_speed_string_ready) {
		kthread->count += snprintf(kthread->buf + kthread->count,
			PAGE_SIZE - kthread->count, "%s", upload_speed_string);
	} else {
		kthread->count += snprintf(kthread->buf + kthread->count,
			PAGE_SIZE - kthread->count, "upload: test timeout\n");
	}

	log_n("upload test finished");

	mutex_unlock(&spidev->write_lock);
	speed_test_complt = 1;
	return 0;
}

static ssize_t
speedtest_show(struct class *class, struct class_attribute *attr, char *buf)
{
	ssize_t status = 0;
	struct kthread_parm speedtest = { 0 };
	speedtest.count = 0;
	speedtest.buf = buf;

#ifdef STAND_ALONE_USED
	if (unlikely(0 != get_sau_ch_id())) {
		speedtest.count = snprintf(speedtest.buf, PAGE_SIZE,
			"ch(%d) stand alone used, not support speedtest\n", get_sau_ch_id());
		return speedtest.count;
	}
#endif

	if (is_master(spidev)) {
		enable_continuous_read();
	}
	speed_test_complt = 0;
	/*start speedtest thread*/
	speedtest.speed_tsk = kthread_create(speedtest_thread, &speedtest, "speed_test_thd");
	if (IS_ERR(speedtest.speed_tsk)) {
		log_e("creat speedtest thread failed");
		status = PTR_ERR(speedtest.speed_tsk);
		return status;
	} else {
		get_task_struct(speedtest.speed_tsk);
		wake_up_process(speedtest.speed_tsk);
	}
	while (!speed_test_complt)
		msleep(1000);
	kthread_stop(speedtest.speed_tsk);
	return speedtest.count;
}

static ssize_t
speedtest_store(struct class *class, struct class_attribute *attr,
		const char *buf, size_t count)
{
	return count;
}
#ifndef STAND_ALONE_CLASS
static CLASS_ATTR_RW(speedtest);
#endif
#endif

static ssize_t
commstop_show(struct class *class, struct class_attribute *attr, char *buf)
{
	ssize_t count = 0;
	count += snprintf(buf + count, PAGE_SIZE - count, "comm state(%d)\n", close_comm_state);
	return count;
}

static ssize_t
commstop_store(struct class *class, struct class_attribute *attr,
		const char *buf, size_t count)
{
	static DEFINE_MUTEX(commstop_store);
	log_e("commstop_store:start!");

	mutex_lock(&commstop_store);
	if (buf[0] == '1'){
		log_e("comm suspend!");
		close_comm_state = 0x03;
	} else if (buf[0] == '2'){
		log_e("comm test stop upload");
		close_comm_state = 0x02;
	} else if (buf[0] == '3'){
		log_e("comm test stop download");
		close_comm_state = 0x01;
	} else if (buf[0] == '0'){
		log_e("comm resume!");
		close_comm_state = 0;
	} else {
		log_e("Invalid argument");
		count = -EINVAL;
	}
	log_e("commstop_store:end! %s(%d),%c,%x", current->comm, current->pid, buf[0], close_comm_state);
	mutex_unlock(&commstop_store);

	return count;
}
#ifndef STAND_ALONE_CLASS
static CLASS_ATTR_RW(commstop);
#endif

#ifdef COLLECT_WAKEUP_INFO
static uint8_t swakeup_data_init = 0;

static void wakeup_data_init(void)
{
	if (0 == swakeup_data_init) {
		if (kfifo_alloc(&wakeup_data, WAKEUP_DATA_KFIFO_SIZE, GFP_KERNEL)) {
			log_e("kfifo_alloc failed");
			return;
		}
		swakeup_data_init = 1;
	}
}

static ssize_t
wakeupdata_show(struct class *class, struct class_attribute *attr, char *buf)
{
	ssize_t count = 0;
	int wakeup_data_len = 0;

	if (0 == swakeup_data_init) {
		log_e("wakeup_data uninitialized");
		return count;
	}

	mutex_lock(&wakeup_data_fifo_lock);
	wakeup_data_len = kfifo_len(&wakeup_data);
	if (kfifo_out(&wakeup_data, buf, wakeup_data_len) == wakeup_data_len) {
		count = (ssize_t) wakeup_data_len;
	}else{
		log_e("impossible situation occured in LINE %d", __LINE__);
	}
	mutex_unlock(&wakeup_data_fifo_lock);

	return count;
}

static ssize_t
wakeupdata_store(struct class *class, struct class_attribute *attr,
		const char *buf, size_t count)
{
	return count;
}
#ifndef STAND_ALONE_CLASS
static CLASS_ATTR_RW(wakeupdata);
#endif
#endif


#ifdef STAND_ALONE_CLASS
static const struct class_attribute attributes[] = {
	__ATTR_RW(log),
	__ATTR_RW(info),
	__ATTR_RW(commstop),
#ifdef COLLECT_WAKEUP_INFO
	__ATTR_RW(wakeupdata),
#endif
#ifdef SPEEDTEST
	__ATTR_RW(speedtest),
#endif
};
#else
static struct attribute *occ_class_attrs[] = {
	&class_attr_log.attr,
	&class_attr_info.attr,
	&class_attr_commstop.attr,
#ifdef SPEEDTEST
	&class_attr_speedtest.attr,
#endif
#ifdef COLLECT_WAKEUP_INFO
	&class_attr_wakeupdata.attr,
#endif
	NULL,
};
ATTRIBUTE_GROUPS(occ_class);
#endif


#define N_SPI_MINORS            32    /* ... up to 256 */

static DECLARE_BITMAP(minors, N_SPI_MINORS);
static int major;
#ifndef STAND_ALONE_CLASS
static struct class     gdevclass;
#endif


static int comm_fifo_init(struct comm_fifo *fifo, unsigned int size)
{
	int ret = 0;
	if ((!fifo)|| (size <=0)) {
		pr_err(LOGTAG "Invalid argument fifo is NULL or size %d  ", size);
		return -EINVAL;
	}

	sema_init(&fifo->packet_cnt_sem, 0);
	mutex_init(&fifo->lock);
	ret = kfifo_alloc(&fifo->fifo, size, GFP_KERNEL);

	if (ret) {
		pr_err(LOGTAG "Out of memory %d, %d ", size, ret);
		ret = -ENOMEM;
	}
	return ret;
}


static int data_cdev_init(struct spidev_data *spidev, struct cdev_info_list_unit *list_unit)
{
	int                 status;
	unsigned long        minor;
	struct data_cdev    *data_cdev = NULL;
	unsigned int        fifo_size;
	int ret;

	data_cdev = kzalloc(sizeof(*data_cdev), GFP_KERNEL);

	if (!data_cdev)
		return -ENOMEM;

	data_cdev->write_packet_area = kzalloc(PACKET_HEAD_SIZE + PACKET_PAYLOAD_SIZE, GFP_KERNEL);
	if (!data_cdev->write_packet_area) {
		kfree(data_cdev);
		return -ENOMEM;
	}
	memset(data_cdev->write_packet_area, 0, PACKET_HEAD_SIZE + PACKET_PAYLOAD_SIZE);

	data_cdev->read_packet_area = kzalloc(PACKET_HEAD_SIZE + PACKET_PAYLOAD_SIZE, GFP_KERNEL);
	if (!data_cdev->read_packet_area) {
		kfree(data_cdev->write_packet_area);
		kfree(data_cdev);
		return -ENOMEM;
	}
	memset(data_cdev->read_packet_area, 0, PACKET_HEAD_SIZE + PACKET_PAYLOAD_SIZE);

	data_cdev->spidev = spidev;
	data_cdev->channel_id = list_unit->channel_id;
	data_cdev->dev_name = list_unit->dev_name;

	mutex_init(&data_cdev->read_lock);
	mutex_init(&data_cdev->write_lock);
	init_waitqueue_head(&data_cdev->waitq);

	/* dcc_data channel would generate large amount of data,
		we need to increase rx and tx buffer to avoid data lost */
	if (test_bit(CDEV_BIG_FIFO, &list_unit->flags))
		fifo_size = SNSHUB_KFIFO_SIZE;
	else
		fifo_size = OTHERS_KFIFO_SIZE;

	if (!test_bit(STAND_ALONE, &list_unit->flags)) {
		ret = comm_fifo_init(&data_cdev->tx_fifo, fifo_size);
		if (ret)
			return -ENOMEM;
	}

	ret = comm_fifo_init(&data_cdev->rx_fifo, fifo_size);
	if (ret)
		return -ENOMEM;

#ifdef DATA_HOOK_EN
	if (test_bit(CDEV_HOOK_EN, &list_unit->flags)) {
		struct data_hook *hook = kzalloc(sizeof(*hook), GFP_KERNEL);
		if (!hook) {
			pr_err(LOGTAG "%s():hook kzalloc failed");
			return -ENOMEM;
		}

		hook->hook_data = data_cdev;
		hook->hook_data_ops = &hook_data_ops;

		if (hci_bcsp_register_hook(hook)) {
			dev_err(spidev->dev, "%s register bcsp hook failed", data_cdev->dev_name);
			kfree(hook);
			hook = NULL;
			return -ENOMEM;
		}
		data_cdev->hook_process = hook;
	}
#endif

	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
#ifdef STAND_ALONE_CLASS
		struct class *pdevclass = &spidev->class;
#else
		struct class *pdevclass = &gdevclass;
#endif
		data_cdev->devt = MKDEV(major, minor);
		data_cdev->dev = device_create(pdevclass, spidev->dev, data_cdev->devt,
					data_cdev, list_unit->dev_name);
		status = PTR_ERR_OR_ZERO(data_cdev->dev);
	} else {
		dev_dbg(spidev->dev, "no minor number available!\n");
		data_cdev->dev = NULL;
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
	}

	snprintf(data_cdev->ws_name, sizeof(data_cdev->ws_name), "ws_%s", list_unit->dev_name);
	wakeup_source_init(&(data_cdev->wakesrc), data_cdev->ws_name);

	list_unit->cdev_info = data_cdev;

	return status;
}

#if 0
static int data_cdev_remove(void)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(g_cdev_info_list); i++) {
#ifdef STAND_ALONE_CLASS
		struct class *pdevclass = &spidev->class;
#else
		struct class *pdevclass = &gdevclass;
#endif
		struct data_cdev *data_cdev = g_cdev_info_list[i].cdev_info;
		device_destroy(pdevclass, data_cdev->devt);
		clear_bit(MINOR(data_cdev->devt), minors);
		kfree(data_cdev);
		g_cdev_info_list[i].cdev_info = NULL;
	}
	return 0;
}
#endif

static int get_gpio_from_devtree(struct spidev_data *spidev, const char *gpio_name)
{
	struct device *dev = &(spidev->spi->dev);
	struct device_node *node = dev->of_node;
	enum of_gpio_flags flags;
	int gpio;

	if (!node)
		return -ENODEV;

	gpio = of_get_named_gpio_flags(node, gpio_name, 0, &flags);
	if (gpio < 0 && gpio != -EPROBE_DEFER) {
		log_e("Failed to get gpio flags, error");
		return -ENODEV;
	}
	if (gpio_is_valid(gpio)) {
		if (gpio_request(gpio, "s2m_int_gpio") < 0) {
			log_e("devm_gpio_request, error, gpio(%d)", gpio);
			return -ENODEV;
		}
	}
	return gpio;
}

static int spidev_gpio_request_irq(struct spidev_data *spidev, int gpio,
		unsigned long irq_flags, irqreturn_t (*irqfunc)(int irq, void *handle))
{
	int irq;
	struct device *dev = &(spidev->spi->dev);

	/* Initialize IRQ */
	irq = gpio_to_irq(gpio);
	if (irq < 0) {
		log_e("gpio(%d) gpio_to_irq failed: %d", gpio, irq);
		return -EINVAL;
	}

	if (request_irq(irq, irqfunc, irq_flags, dev_name(dev), dev) < 0) {
		log_e("Error, could not request irq");
		return -EINVAL;
	}
	return irq;
}

static int spidev_gpio_init(struct spidev_data *spidev)
{
	spidev->s2m_int_gpio = get_gpio_from_devtree(spidev, "s2m_int_gpio");
	if (spidev->s2m_int_gpio < 0) {
		dev_err(spidev->dev, "s2m_int_gpio: not find in dts");
		return -ENODEV;
	}
	gpio_direction_input(spidev->s2m_int_gpio);
	spidev->s2m_int_irq = spidev_gpio_request_irq(spidev, spidev->s2m_int_gpio,
			IRQF_TRIGGER_RISING, s2m_int_irq_fn);
	if (spidev->s2m_int_irq < 0) {
		dev_err(spidev->dev, "s2m_int_irq request irq failed %d", spidev->s2m_int_irq);
		return -ENODEV;
	}
	if (enable_irq_wake(spidev->s2m_int_irq) != 0) {
		dev_err(spidev->dev, "s2m_int_irq enable irq wake failed");
		return -ENODEV;
	}
	log_n("s2m_int_gpio: direction(in), gpio(%d), irq(%d)",
			spidev->s2m_int_gpio, spidev->s2m_int_irq);

	spidev->m2s_ans_gpio = get_gpio_from_devtree(spidev, "m2s_ans_gpio");
	if (spidev->m2s_ans_gpio < 0) {
		dev_err(spidev->dev, "m2s_ans_gpio: not find in dts");
		return -ENODEV;
	}
	gpio_direction_output(spidev->m2s_ans_gpio, 0);
	log_n("m2s_ans_gpio: direction(out), gpio(%d)", spidev->m2s_ans_gpio);

	spidev->m2s_int_gpio = get_gpio_from_devtree(spidev, "m2s_int_gpio");
	if (spidev->m2s_int_gpio < 0) {
		dev_err(spidev->dev, "m2s_int_gpio: not find in dts");
		return -ENODEV;
	}
	gpio_direction_output(spidev->m2s_int_gpio, 0);
	log_n("m2s_int_gpio: direction(out), gpio(%d)", spidev->m2s_int_gpio);

	spidev->s2m_ans_gpio = get_gpio_from_devtree(spidev, "s2m_ans_gpio");
	if (spidev->s2m_ans_gpio < 0) {
		dev_err(spidev->dev, "s2m_ans_gpio: not find in dts");
		return -ENODEV;
	}
	gpio_direction_input(spidev->s2m_ans_gpio);
	spidev->s2m_ans_irq = spidev_gpio_request_irq(spidev, spidev->s2m_ans_gpio,
			IRQF_TRIGGER_RISING, s2m_ans_irq_fn);
	if (spidev->s2m_ans_irq < 0) {
		dev_err(spidev->dev, "s2m_ans_irq: request irq failed %d", spidev->s2m_ans_irq);
		return -ENODEV;
	}
	log_n("s2m_ans_gpio: direction(in), gpio(%d), irq(%d)",
			spidev->s2m_ans_gpio, spidev->s2m_ans_irq);

	return 0;
}

static int spidev_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	const void *name;
	int status = 0;
	int i;

	if (dev == NULL) {
		pr_err("dev is null!!!!!");
		return -EINVAL;
	}

	/* Allocate driver data */
	spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
	if (!spidev)
		return -ENOMEM;

	name = of_get_property(dev->of_node, "label", NULL);

	if (!name) {
		dev_err(dev, "cannot get name");
		return -EINVAL;
	}

	strncpy(spidev->name, name, 6);

#ifdef STAND_ALONE_CLASS
	/* register device class */
	spidev->class.name = name;

	spidev->class.owner = THIS_MODULE;
	status = class_register(&spidev->class);
	if (status) {
		dev_err(dev, "cannot allocate driver data");
		return status;
	}

	for (i = 0; i < ARRAY_SIZE(attributes); i++) {
		if (class_create_file(&spidev->class, attributes + i))
			dev_err(dev, "cannot create file");
	}
#endif
	/* Initialize the driver data */
	//first frame need't check selnum
	spidev->is_first_up_frame = 1;
	spidev->is_first_dl_frame = 1;
	spidev->spi = spi;
	spidev->dev = dev;

	mutex_init(&spidev->write_lock);
	//mutex_init(&spidev->dual_comm_lock);

	spidev->spi_master = !(spi->master->slave);

	if (is_master(spidev) != spidev->spi_master) {
		log_e("spi %s is error", (is_master(spidev))?"master":"slave");
		return -EINVAL;
	}

	log_e("spi_master:%s", (is_master(spidev))?"true":"false");
	if (is_master(spidev)) {
		spi->mode = SPI_MODE_3;
		spidev->tx_buf_size = PHY_MST_TX_BUFF_SIZE - 8;
		spidev->spi_slave_wr_addr = spi_addr_wr(peer_spi_slave_tx_buf_addr);
		spidev->spi_slave_rd_addr = spi_addr_rd(peer_spi_slave_rx_buf_addr);
		spidev->next_rd_pkg_len = XFER_MIN_LEN;
		spidev->rd_pkg_len_last_ten_max = XFER_MIN_LEN;
		spidev->rd_pkg_len_dec_cnt = 0;
	} else {
		spi->mode = SPI_MODE_1;//spi slave mode, only mode 1
		spidev->tx_buf_size = PHY_BUFF_SIZE;
		spi->slave_ready_cb = slave_ready_cb;
	}

	//default bits_per_word to 8
	spidev->spi_bits_per_word = 8;
	of_property_read_u32(dev->of_node, "spi_bits_per_word", &spidev->spi_bits_per_word);
	if (spidev->spi_bits_per_word != 8 && spidev->spi_bits_per_word != 32) {
		log_e("invalid dts spi_bits_per_word, use default 8");
		spidev->spi_bits_per_word = 8;
	}

	sema_init(&spidev->s2m_int_pu_sem, 0);
	sema_init(&spidev->s2m_ans_pu_sem, 0);
	sema_init(&spidev->m2s_tx_fifo_sem, 0);

	spidev->spi_rx_buffer = kzalloc(PHY_BUFF_SIZE, GFP_KERNEL);
	if (!spidev->spi_rx_buffer)
		return -ENOMEM;

	spidev->spi_tx_buffer = kzalloc(spidev->tx_buf_size, GFP_KERNEL);
	if (!spidev->spi_tx_buffer)
		return -ENOMEM;

	spidev->send_frame = spidev->spi_tx_buffer;
	if (is_master(spidev)) {
		spidev->send_frame += SLAVE_ADDR_LEN;
		spidev->tx_buf_size = PHY_BUFF_SIZE;
	}

	#ifdef DUAL_ACK
	spidev->recv_frame = kzalloc(PHY_BUFF_SIZE, GFP_KERNEL);
	if (!spidev->recv_frame)
		return -ENOMEM;
	#else
	spidev->recv_frame = spidev->spi_rx_buffer;
	if (is_master(spidev)) {
		spidev->recv_frame += SLAVE_SKIP_LEN;
	}
	#endif

	#ifdef COLLECT_WAKEUP_INFO
	wakeup_data_init();
	#endif

	spi_set_drvdata(spi, spidev);

	wakeup_source_init(&spidev->upload_wakesrc, "upload_wakesrc");
	wakeup_source_init(&spidev->download_wakesrc, "download_wakesrc");
	wakeup_source_init(&spidev->resever_wakesrc, "resever_wakesrc");

	if (spidev_gpio_init(spidev) < 0) {
		log_e("spidev_gpio_init failed!");
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(g_cdev_info_list); ++i)
		data_cdev_init(spidev, &g_cdev_info_list[i]);

	/*start read thread*/
	spidev->snshub_read_tsk = kthread_create(comm_master_upload_thread, spidev, "snshub_up_thd");
	if (IS_ERR(spidev->snshub_read_tsk)) {
		log_e("creat read thread failed");
		status = PTR_ERR(spidev->snshub_read_tsk);
	} else {
		get_task_struct(spidev->snshub_read_tsk);
		wake_up_process(spidev->snshub_read_tsk);
	}

	/*start write thread*/
	spidev->snshub_write_tsk = kthread_create(comm_master_download_thread, spidev, "snshub_down_thd");
	if (IS_ERR(spidev->snshub_write_tsk)) {
		log_e("creat write thread failed");
		status = PTR_ERR(spidev->snshub_write_tsk);
	} else {
		get_task_struct(spidev->snshub_write_tsk);
		wake_up_process(spidev->snshub_write_tsk);
	}

#ifdef PRINT_LAST_N_PKG_INFO
	if(oplus_sensorhub_reset_notifier_register(NULL, &snshub_reset_nb)) {
		log_e("oplus_snshub_reset_notifier_register failed");
	}
#endif

	return status;
}

static int spidev_remove(struct spi_device *spi)
{
	(void)spi;
	return 0;
}

static int snshub_suspend(struct device *dev)
{
	struct spidev_data *spidev = dev->driver_data;
	spidev->is_suspended = true;
#ifdef PRINT_WAKEUP_INFO
	spidev->print_wakeup = 1;
#endif
#ifdef CONFIG_DEEPSLEEP
	if (PM_SUSPEND_MEM == mem_sleep_current) {
		log_n("(deepsleep)");
		disable_irq_wake(spidev->s2m_int_irq);
		disable_irq(spidev->s2m_int_irq);
		disable_irq(spidev->s2m_ans_irq);
	}
#endif
	spidev->next_rd_pkg_len = XFER_MIN_LEN;
	return 0;
}

static int snshub_resume(struct device *dev)
{
	struct spidev_data *spidev = dev->driver_data;
	spidev->is_suspended = false;
#ifdef CONFIG_DEEPSLEEP
	if (PM_SUSPEND_MEM == mem_sleep_current) {
		log_n("(deepsleep)");
		if (gpio_get_value(spidev->s2m_int_gpio) == 1) {
			log_w("snshub resume int high");
			up(&spidev->s2m_int_pu_sem);
		}
		enable_irq(spidev->s2m_ans_irq);
		enable_irq(spidev->s2m_int_irq);
		enable_irq_wake(spidev->s2m_int_irq);
	}
#endif
	return 0;
}

static const struct dev_pm_ops snshub_pm_ops = {
	.resume         = snshub_resume,
	.suspend        = snshub_suspend,
};

static const struct of_device_id spidev_dt_ids[] = {
	{ .compatible = "chip_comm_plus", },
	{},
};

static struct spi_driver spidev_spi_driver = {
	.driver = {
		.name  = "chip_comm",
		//.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(spidev_dt_ids),
		.pm    = &snshub_pm_ops,
	},
	.probe  = spidev_probe,
	.remove = spidev_remove,
};
#define CDEV_NAME "ocmp"

static int __init spidev_init(void)
{
	int status;

	BUILD_BUG_ON(N_SPI_MINORS > 256);
	dev_info(NULL, "<oplus comm master plus driver, verison: %s>\n", DRIVER_VERSION);

#ifndef STAND_ALONE_CLASS
	/* register device class */
	gdevclass.name = CDEV_NAME;
	gdevclass.owner = THIS_MODULE;
	gdevclass.class_groups = occ_class_groups;

	status = class_register(&gdevclass);
	if (status) {
		dev_err(NULL, "cannot register class");
		goto err_register_class;
	}
#endif

	status = register_chrdev(0, CDEV_NAME, &spidev_fops);
	if (status < 0) {
		dev_err(NULL, "register_chrdev failed %d", status);
		goto err_register_chrdev;
	}
	major = status;

	status = spi_register_driver(&spidev_spi_driver);
	if (status < 0) {
		dev_err(NULL, "spi_register_driver failed");
		goto err_register_driver;
	}

	return 0;

	//spi_unregister_driver(&spidev_spi_driver);
err_register_driver:
	unregister_chrdev(major, CDEV_NAME);
err_register_chrdev:
#ifndef STAND_ALONE_CLASS
	class_unregister(&gdevclass);
err_register_class:
#endif
	return status;
}
module_init(spidev_init);

static void __exit spidev_exit(void)
{
	return;
}
module_exit(spidev_exit);

MODULE_AUTHOR("Oplus");
MODULE_DESCRIPTION("Oplus SDW-MCU dual communication driver");
MODULE_LICENSE("GPL");
