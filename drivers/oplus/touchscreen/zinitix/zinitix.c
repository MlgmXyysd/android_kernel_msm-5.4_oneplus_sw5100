/*
 *
 * Zinitix bt541 touchscreen driver
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#if defined(CONFIG_PM_RUNTIME)
#include <linux/pm_runtime.h>
#endif
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/input/mt.h>
#include <linux/regulator/machine.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <asm/io.h>
#include <linux/power_supply.h>

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/firmware.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
//#include <linux/earlysuspend.h>

#ifdef CONFIG_MACH_PXA_SAMSUNG
#include <linux/sec-common.h>
#endif
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include "zinitix_touch.h"

#include <linux/msm_drm_notify.h>
#include <../../../oplus/include/oplus.h>
#include <../../../oplus/include/boot_mode.h>

#if (TSP_TYPE_COUNT == 1)
u8 *m_pFirmware [TSP_TYPE_COUNT] = {(u8*)m_firmware_data,};
#else
u8 *m_pFirmware [TSP_TYPE_COUNT] = {(u8*)m_firmware_data_01,(u8*)m_firmware_data_02,};
#endif
u8 m_FirmwareIdx = 0;

#define TSP_HW_ID_INDEX_0 1
#define TSP_HW_ID_INDEX_1 2

extern char *saved_command_line;


/* if you use isp mode, you must add i2c device :
name = "zinitix_isp" , addr 0x50*/
/* resolution offset */
#define ABS_PT_OFFSET			(-1)

/* PMIC Regulator based supply to TSP */
#define TSP_REGULATOR_SUPPLY		1
/* gpio controlled LDO based supply to TSP */
#define TSP_LDO_SUPPLY			0

enum power_control {
	POWER_OFF,
	POWER_ON,
	POWER_ON_SEQUENCE,
};

/* Key Enum */
enum key_event {
	ICON_BUTTON_UNCHANGE,
	ICON_BUTTON_DOWN,
	ICON_BUTTON_UP,
};

#define FT_VTG_MIN_UV		2600000
#define FT_VTG_MAX_UV		3300000
#define FT_I2C_VTG_MIN_UV	1800000
#define FT_I2C_VTG_MAX_UV	1800000


#define PINCTRL_STATE_ACTIVE	"pmx_ts_active"
#define PINCTRL_STATE_SUSPEND	"pmx_ts_suspend"
#define PINCTRL_STATE_RELEASE	"pmx_ts_release"


#define MAX_RAW_DATA_SZ			576 /* 32x18 */
#define MAX_TRAW_DATA_SZ	\
	(MAX_RAW_DATA_SZ + 4*MAX_SUPPORTED_FINGER_NUM + 2)
/* preriod raw data interval */

#define RAWDATA_DELAY_FOR_HOST		100

struct raw_ioctl {
	int sz;
	u8 *buf;
};

struct reg_ioctl {
	int addr;
	int *val;
};

#define TOUCH_SEC_MODE			48
#define TOUCH_REF_MODE			10
#define TOUCH_NORMAL_MODE		5
#define TOUCH_DELTA_MODE		3
#define TOUCH_DND_MODE			6
#define TOUCH_PDND_MODE			11

#define TOUCH_CHECK_SHORT_MODE	14
#define CHECK_SHORT_DATA_NUM    34

#define ZINITIX_INIT_RETRY_CNT 2

/*  Other Things */
#define INIT_RETRY_CNT			3
#define I2C_SUCCESS			0
#define I2C_FAIL			1

/*------------Link  modify IC flash size 20180523-------------------*/

/* chip code */
#define ZTW523_CHIP_CODE	0xE628
#define ZTW522_CHIP_CODE	0xE532
#define ZT7548_CHIP_CODE	0xE548
#define ZT7538_CHIP_CODE	0xE538
#define ZT7554_CHIP_CODE	0xE700
#define ZTW622_CHIP_CODE    0xE622

/*------------Link  modify IC flash size 20180523-------------------*/

/* Register Map*/
#define BT541_SWRESET_CMD		0x0000
#define BT541_WAKEUP_CMD		0x0001

#define BT541_IDLE_CMD			0x0004
#define BT541_SLEEP_CMD			0x0005

#define BT541_CLEAR_INT_STATUS_CMD	0x0003
#define BT541_CALIBRATE_CMD		0x0006
#define BT541_SAVE_STATUS_CMD		0x0007
#define BT541_SAVE_CALIBRATION_CMD	0x0008
#define BT541_RECALL_FACTORY_CMD	0x000f

#define BT541_THRESHOLD			0x0020

#define BT541_DEBUG_REG			0x0115 /* 0~7 */

#define BT541_TOUCH_MODE		0x0010
#define BT541_CHIP_REVISION		0x0011
#define BT541_FIRMWARE_VERSION		0x0012

#define ZINITIX_USB_DETECT	0x116

#define BT541_MINOR_FW_VERSION		0x0121

#define BT541_VENDOR_ID			0x001C
#define BT541_HW_ID				0x0014

#define BT541_DATA_VERSION_REG		0x0013
#define BT541_SUPPORTED_FINGER_NUM	0x0015
#define BT541_EEPROM_INFO		0x0018
#define BT541_INITIAL_TOUCH_MODE	0x0019

#define BT541_TOTAL_NUMBER_OF_X		0x0060
#define BT541_TOTAL_NUMBER_OF_Y		0x0061

#define BT541_DELAY_RAW_FOR_HOST	0x007f

#define BT541_BUTTON_SUPPORTED_NUM	0x00B0
#define BT541_BUTTON_SENSITIVITY	0x00B2
#define BT541_DUMMY_BUTTON_SENSITIVITY	0X00C8

#define BT541_X_RESOLUTION		0x00C0
#define BT541_Y_RESOLUTION		0x00C1

#define BT541_POINT_STATUS_REG		0x0080
#define BT541_ICON_STATUS_REG		0x00AA

#define BT541_AFE_FREQUENCY		0x0100
#define BT541_DND_N_COUNT		0x0122
#define BT541_DND_U_COUNT		0x0135

#define BT541_RAWDATA_REG		0x0200

#define BT541_EEPROM_INFO_REG		0x0018

#define BT541_INT_ENABLE_FLAG		0x00f0
#define BT541_PERIODICAL_INTERRUPT_INTERVAL	0x00f1

#define BT541_BTN_WIDTH			0x016d

#define BT541_CHECKSUM_REG              0x012c
#define BT541_CHECKSUM_SUCCESS_FLAG     0x55aa

#define BT541_INIT_FLASH		0x01d0
#define BT541_WRITE_FLASH		0x01d1
#define BT541_READ_FLASH		0x01d2

#define ZINITIX_INTERNAL_FLAG_02		0x011e
#define ZINITIX_INTERNAL_FLAG_03		0x011f

#define	ZINITIX_I2C_CHECKSUM_WCNT	0x016a
#define	ZINITIX_I2C_CHECKSUM_RESULT	0x016c

#define ZTX_REG_GESTURE_RESULT              0x0126
#define ZTX_GESTURE_CLICK                   0x01
#define ZTX_GESTURE_SLIDE                   0x04

#define ZTX_REG_GESTURE_SUPPORT             0x011d
#define ZTX_USE_GESTURE_CLICK               0x0089
#define ZTX_USE_GESTURE_SLIDE               0x00C1
#define ZTX_USE_GESTURE_CLICK_AND_SLIDE     0x00C9

/* Interrupt & status register flag bit
-------------------------------------------------
*/
#define BIT_PT_CNT_CHANGE	0
#define BIT_DOWN		1
#define BIT_MOVE		2
#define BIT_UP			3
#define BIT_PALM		4
#define BIT_PALM_REJECT		5
#define RESERVED_0		6
#define RESERVED_1		7
#define BIT_WIDTH_CHANGE	8
#define BIT_PT_NO_CHANGE	9
#define BIT_REJECT		10
#define BIT_PT_EXIST		11
#define RESERVED_2		12
#define BIT_MUST_ZERO		13
#define BIT_DEBUG		14
#define BIT_ICON_EVENT		15

/* button */
#define BIT_O_ICON0_DOWN	0
#define BIT_O_ICON1_DOWN	1
#define BIT_O_ICON2_DOWN	2
#define BIT_O_ICON3_DOWN	3
#define BIT_O_ICON4_DOWN	4
#define BIT_O_ICON5_DOWN	5
#define BIT_O_ICON6_DOWN	6
#define BIT_O_ICON7_DOWN	7

#define BIT_O_ICON0_UP		8
#define BIT_O_ICON1_UP		9
#define BIT_O_ICON2_UP		10
#define BIT_O_ICON3_UP		11
#define BIT_O_ICON4_UP		12
#define BIT_O_ICON5_UP		13
#define BIT_O_ICON6_UP		14
#define BIT_O_ICON7_UP		15


#define SUB_BIT_EXIST		0
#define SUB_BIT_DOWN		1
#define SUB_BIT_MOVE		2
#define SUB_BIT_UP		3
#define SUB_BIT_UPDATE		4
#define SUB_BIT_WAIT		5

/* Mode status */
#define	TS_USB_DETECT_BIT		0
#define	TS_SVIEW_DETECT_BIT		1
#define	TS_SENSIVE_MODE_BIT		2

#define zinitix_bit_set(val, n)		((val) &= ~(1<<(n)), (val) |= (1<<(n)))
#define zinitix_bit_clr(val, n)		((val) &= ~(1<<(n)))
#define zinitix_bit_test(val, n)	((val) & (1<<(n)))
#define zinitix_swap_v(a, b, t)		((t) = (a), (a) = (b), (b) = (t))
#define zinitix_swap_16(s)		(((((s) & 0xff) << 8) | (((s) >> 8) & 0xff)))

#define ZTX_TIMEOUT_COMERR_PM               700

extern int get_lcd_attached(void);
extern unsigned int system_rev;
static volatile int tpd_halt = 0;


/* end header file */

#ifdef SEC_FACTORY_TEST
/* Touch Screen */
#define TSP_CMD_INT_WAIT_TIME       500
#define TSP_CMD_STR_LEN             32
#define TSP_CMD_RESULT_STR_LEN      512
#define TSP_CMD_PARAM_NUM           8
#define TSP_CMD_Y_NUM               18
#define TSP_CMD_X_NUM               30
#define TSP_CMD_NODE_NUM            (TSP_CMD_Y_NUM * TSP_CMD_X_NUM)
#define tostring(x)                 #x

struct tsp_factory_info {
	struct list_head cmd_list_head;
	char cmd[TSP_CMD_STR_LEN];
	char cmd_param[TSP_CMD_PARAM_NUM];
	char cmd_result[TSP_CMD_RESULT_STR_LEN];
	char cmd_buff[TSP_CMD_RESULT_STR_LEN];
	struct mutex cmd_lock;
	bool cmd_is_running;
	u8 cmd_state;
};

struct tsp_raw_data {
	//Link  modified  here on 20190506 for  circle  design the  corner minus value
	s16 ref_data[TSP_CMD_NODE_NUM];
	s16 pref_data[TSP_CMD_NODE_NUM];

	s16 delta_data[TSP_CMD_NODE_NUM];
};

enum {
	WAITING = 0,
	RUNNING,
	OK,
	FAIL,
	NOT_APPLICABLE,
};

struct tsp_cmd {
	struct list_head list;
	const char *cmd_name;
	void (*cmd_func)(void *device_data);
};
#ifdef CONFIG_HAS_EARLYSUSPEND
static void zinitix_early_suspend(struct early_suspend *h);
static void zinitix_late_resume(struct early_suspend *h);
#endif

extern unsigned int get_oplus_board_id(void);
extern unsigned int get_project(void);
extern uint get_display_vflip(void);

static bool ts_set_touchmode(u16 value);

static void fw_update(void *device_data);
static void get_fw_ver_bin(void *device_data);
static void get_fw_ver_ic(void *device_data);
static void get_threshold(void *device_data);
static void module_off_master(void *device_data);
static void module_on_master(void *device_data);
static void module_off_slave(void *device_data);
static void module_on_slave(void *device_data);
static void get_chip_vendor(void *device_data);
static void get_chip_name(void *device_data);
static void get_x_num(void *device_data);
static void get_y_num(void *device_data);
static void not_support_cmd(void *device_data);

/* Vendor dependant command */
/* static void run_reference_read(void *device_data); */
static void get_reference(void *device_data);
static void run_preference_read(void *device_data);
static void run_short_data_read(void *device_data);
static void get_preference(void *device_data);
static void run_delta_read(void *device_data);
static void get_delta(void *device_data);
static void get_module_vendor(void *device_data);
static void get_config_ver(void *device_data);
#ifdef GLOVE_MODE
static void glove_mode(void *device_data);
#endif
#define TSP_CMD(name, func) .cmd_name = name, .cmd_func = func

static struct tsp_cmd tsp_cmds[] = {
	{TSP_CMD("fw_update", fw_update),},
	{TSP_CMD("get_fw_ver_bin", get_fw_ver_bin),},
	{TSP_CMD("get_fw_ver_ic", get_fw_ver_ic),},
	{TSP_CMD("get_threshold", get_threshold),},
	{TSP_CMD("module_off_master", module_off_master),},
	{TSP_CMD("module_on_master", module_on_master),},
	{TSP_CMD("module_off_slave", module_off_slave),},
	{TSP_CMD("module_on_slave", module_on_slave),},
	{TSP_CMD("get_module_vendor", get_module_vendor),},
	{TSP_CMD("get_chip_vendor", get_chip_vendor),},
	{TSP_CMD("get_chip_name", get_chip_name),},
	{TSP_CMD("get_x_num", get_x_num),},
	{TSP_CMD("get_y_num", get_y_num),},
	{TSP_CMD("not_support_cmd", not_support_cmd),},

	/* vendor dependant command */
	{TSP_CMD("run_reference_read", run_preference_read),},
	{TSP_CMD("run_short_data_read", run_short_data_read),},
	{TSP_CMD("get_reference", get_reference),},
	{TSP_CMD("run_dnd_read", run_preference_read),},
	{TSP_CMD("get_dnd", get_preference),},
	{TSP_CMD("run_delta_read", run_delta_read),},
	{TSP_CMD("get_delta", get_delta),},
	{TSP_CMD("get_config_ver", get_config_ver),},
#ifdef GLOVE_MODE
	{TSP_CMD("glove_mode", glove_mode),},
#endif
};

#endif

#define TSP_NORMAL_EVENT_MSG 1
static int m_ts_debug_mode = ZINITIX_DEBUG;
#ifdef USE_TSP_TA_CALLBACKS
static bool ta_connected =0;
#endif
void (*tsp_charger_status_cb)(int);
static u16 m_optional_mode = 0;

struct coord {
	u16	x;
	u16	y;
	u8	width;
	u8	sub_status;
#if (TOUCH_POINT_MODE == 2)
	u8	minor_width;
	u8	angle;
#endif
};

struct point_info {
	u16	status;
#if (TOUCH_POINT_MODE == 1)
	u16	event_flag;
#else
	u8	finger_cnt;
	u8	time_stamp;
#endif
	struct coord coord[MAX_SUPPORTED_FINGER_NUM];
};

struct first_flame_filter_algo_data {
	int flame;
	int dx;
	int dy;
};

struct kalman_filter_algo_data {
	int velocity_last;
	int position_last;
	int weighted_average_speed;
	int history_position;
};

struct filter_algo_data {
	u8 state;
	int xn;
	int yn;
	struct first_flame_filter_algo_data delta_data;
	struct kalman_filter_algo_data x_data;
	struct kalman_filter_algo_data y_data;
};

#define TOUCH_V_FLIP	0x01
#define TOUCH_H_FLIP	0x02
#define TOUCH_XY_SWAP	0x04

struct capa_info {
	u16	vendor_id;
	u16	ic_revision;
	u16	fw_version;
	u16	fw_minor_version;
	u16	reg_data_version;
	u16	threshold;
	u16	key_threshold;
	u16	dummy_threshold;
	u32	ic_fw_size;
	u32	MaxX;
	u32	MaxY;
	u32	MinX;
	u32	MinY;
	u8	gesture_support;
	u16	multi_fingers;
	u16	button_num;
	u16	ic_int_mask;
	u16	x_node_num;
	u16	y_node_num;
	u16	total_node_num;
	u16	hw_id;
	u16	afe_frequency;
	u16	i2s_checksum;
	u16	shift_value;
	u16	N_cnt;
	u16	u_cnt;
};

enum work_state {
	NOTHING = 0,
	NORMAL,
	ESD_TIMER,
	SUSPEND,
	RESUME,
	UPGRADE,
	REMOVE,
	SET_MODE,
	HW_CALIBRAION,
	RAW_DATA,
	PROBE,
};

enum {
	BUILT_IN = 0,
	UMS,
	REQ_FW,
};

struct bt541_ts_info {
	struct i2c_client		*client;
	struct input_dev		*input_dev;
	struct bt541_ts_platform_data	*pdata;
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;

	char				phys[32];

	struct capa_info		cap_info;
	struct point_info		touch_info;
	struct point_info		reported_touch_info;
	struct filter_algo_data filter_data[MAX_SUPPORTED_FINGER_NUM];
	u16				icon_event_reg;
	u16				prev_icon_event;

	int				irq;

	u8				work_state;
	struct semaphore		work_lock;
	struct mutex read_reg_mutex;

	u8 finger_cnt1;

#ifdef USE_TSP_TA_CALLBACKS
	void (*register_cb) (struct tsp_callbacks *tsp_cb);
	struct tsp_callbacks callbacks;
#endif

#ifdef ZTX_ESD_ENABLE
	bool esd_enable;
	struct delayed_work esd_test_work;
#endif /* ZTX_ESD_ENABLE */
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif

	struct semaphore		raw_data_lock;
	u16				touch_mode;
	s16				cur_data[MAX_TRAW_DATA_SZ];
	u8				update;
#ifdef SEC_FACTORY_TEST
	struct tsp_factory_info     *factory_info;
	struct tsp_raw_data         *raw_data;
	struct proc_dir_entry       *proc_entry;
#endif
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	struct regulator *bus_pull;
	bool device_enabled;
	bool checkUMSmode;
	u8 *firmware_data;
	bool aod_mode;
	int debug_level;
	bool power_off_mode;
	bool gesture_mode_en;
#ifdef CONFIG_PM
	bool pm_suspend;
#endif /* CONFIG_PM */
	struct workqueue_struct *ts_workqueue;
	struct work_struct resume_work;
	struct work_struct fwupg_work;
};
struct bt541_ts_info *misc_touch_dev = NULL;


#ifdef USE_TSP_TA_CALLBACKS
static void bt541_set_ta_status(struct bt541_ts_info *info, bool force);
#endif

/* define i2c sub functions*/
static inline s32 read_data(struct i2c_client *client,
	u16 reg, u8 *values, u16 length)
{
	s32 ret;
	int count = 0;
retry:
	/* select register*/
	ret = i2c_master_send(client , (u8 *)&reg , 2);
	if (ret < 0) {
		msleep(1);

		if (++count < 8)
			goto retry;

		return ret;
	}
	/* for setup tx transaction. */
	udelay(DELAY_FOR_TRANSCATION);
	ret = i2c_master_recv(client , values , length);
	if (ret < 0)
		return ret;

	udelay(DELAY_FOR_POST_TRANSCATION);
	return length;
}

static inline s32 write_data(struct i2c_client *client,
	u16 reg, u8 *values, u16 length)
{
	s32 ret;
	//int count = 0;
	u8 pkt[66]; /* max packet , addr: 2bytes, data: 64bytes*/

	if (length + 2 > sizeof(pkt)) {
		return -1;
	}

	pkt[0] = (reg) & 0xff; /* reg addr */
	pkt[1] = (reg >> 8)&0xff;
	memcpy((u8 *)&pkt[2], values, length);

	ret = i2c_master_send(client , pkt , length + 2);
	if (ret < 0) {
		return ret;
	}

	udelay(DELAY_FOR_POST_TRANSCATION);
	return length;
}

static inline s32 write_reg(struct i2c_client *client, u16 reg, u16 value)
{
	if (write_data(client, reg, (u8 *)&value, 2) < 0) {
		return I2C_FAIL;
	}

	return I2C_SUCCESS;
}

static inline s32 write_cmd(struct i2c_client *client, u16 reg)
{
	s32 ret;

	ret = i2c_master_send(client , (u8 *)&reg , 2);
	if (ret < 0) {
		return ret;
	}

	udelay(DELAY_FOR_POST_TRANSCATION);
	return I2C_SUCCESS;
}

static inline s32 read_raw_data(struct i2c_client *client,
		u16 reg, u8 *values, u16 length)
{
	s32 ret;
	int count = 0;

retry:
	/* select register */
	ret = i2c_master_send(client , (u8 *)&reg , 2);
	if (ret < 0) {
		msleep(1);

		if (++count < 8)
			goto retry;

		return ret;
	}

	/* for setup tx transaction. */
	udelay(200);

	ret = i2c_master_recv(client , values , length);
	if (ret < 0)
		return ret;

	udelay(DELAY_FOR_POST_TRANSCATION);
	return length;
}

static inline s32 read_firmware_data(struct i2c_client *client,
	u16 addr, u8 *values, u16 length)
{
	s32 ret;
	/* select register*/

	ret = i2c_master_send(client , (u8 *)&addr , 2);
	if (ret < 0)
		return ret;

	/* for setup tx transaction. */
	msleep(1);

	ret = i2c_master_recv(client , values , length);
	if (ret < 0)
		return ret;

	udelay(DELAY_FOR_POST_TRANSCATION);
	return length;
}

static s32 bt541_write_reg(struct i2c_client *client, u16 reg, u16 value)
{
	s32 ret = 0;
	int i = 0;
	const int MAX_RETRY = 3;

	for(i = 0; i < MAX_RETRY; i++) {
		ret = write_reg(client, reg, value);
		if (ret == I2C_SUCCESS) {
			break;
		}
		msleep(10);
	}

	return ret;
}

static s32 bt541_write_cmd(struct i2c_client *client, u16 reg)
{
	s32 ret = 0;
	int i = 0;
	const int MAX_RETRY = 3;

	for(i = 0; i < MAX_RETRY; i++) {
		ret = write_cmd(client, reg);
		if (ret >= 0) {
			break;
		}
		msleep(10);
	}

	return ret;
}

static bool bt541_power_control(struct bt541_ts_info *info, u8 ctl);
static bool bt541_power_sequence(struct bt541_ts_info *info);
static bool zinitix_resume_sequence(struct bt541_ts_info *info);
static int zinitix_power_control(struct bt541_ts_info *data, int on);


static bool init_touch(struct bt541_ts_info *info);
static bool mini_init_touch(struct bt541_ts_info *info);
static void clear_report_data(struct bt541_ts_info *info);
#ifdef ZTX_ESD_ENABLE
static void bt541_esd_timer_start(struct bt541_ts_info *info);
static void bt541_esd_timer_stop(struct bt541_ts_info *info);
static void bt541_esd_timer_handler(struct work_struct *work);
#endif /* ZTX_ESD_ENABLE */

static void ztx_get_fw_version(struct bt541_ts_info *ts_info);

static int ts_misc_fops_open(struct inode *inode, struct file *filp);
static int ts_misc_fops_close(struct inode *inode, struct file *filp);

static const struct file_operations ts_misc_fops = {
	.owner = THIS_MODULE,
	.open = ts_misc_fops_open,
	.release = ts_misc_fops_close,
};

static struct miscdevice touch_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "zinitix_touch_misc",
	.fops = &ts_misc_fops,
};


struct bt541_ts_info *misc_info;
#ifdef CONFIG_OF
static int zinitix_init_gpio(struct bt541_ts_info* data,bool on );
static int zinitix_hw_reset( struct bt541_ts_info* data,bool on );
static int resume_hw_reset( struct bt541_ts_info* data,bool on );

#endif


#define I2C_BUFFER_SIZE 64

#ifdef SEC_FACTORY_TEST
static bool get_raw_data_lite(struct bt541_ts_info *info, u8 *buff, int skip_cnt)
{
	struct i2c_client *client = info->client;
	struct bt541_ts_platform_data *pdata = info->pdata;
	u32 total_node = info->cap_info.total_node_num;
	int sz;
	int i, j;
	u32 temp_sz;

	info->work_state = RAW_DATA;

	for(i = 0; i < skip_cnt; i++) {
		for (j = 0; j < TSP_CMD_INT_WAIT_TIME && gpio_get_value(pdata->gpio_int); j++) {
			msleep(1);
		}

		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		msleep(1);
	}

	sz = total_node * 2;

	for (j = 0; j < TSP_CMD_INT_WAIT_TIME && gpio_get_value(pdata->gpio_int); j++) {
		msleep(1);
	}

	for(i = 0; sz > 0; i++){
		temp_sz = I2C_BUFFER_SIZE;

		if(sz < I2C_BUFFER_SIZE)
			temp_sz = sz;

		if (read_raw_data(client, BT541_RAWDATA_REG + i, (char *)(buff + (i*I2C_BUFFER_SIZE)), temp_sz) < 0) {
			dev_err(&client->dev, "%s: Failed to read raw data\n", __func__);
			info->work_state = NOTHING;
			return false;
		}
		sz -= I2C_BUFFER_SIZE;
	}

	write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
	info->work_state = NOTHING;
	return true;
}

static bool get_raw_data(struct bt541_ts_info *info, u8 *buff, int skip_cnt) {
	struct i2c_client *client = info->client;
	bool ret = false;

	disable_irq(info->irq);

	down(&info->work_lock);
	if (info->work_state != NOTHING) {
		dev_err(&client->dev, "%s: Other process occupied (%d)\n",
				__func__, info->work_state);
		enable_irq(info->irq);
		up(&info->work_lock);

		return false;
	}

	ret = get_raw_data_lite(info, buff, skip_cnt);

	enable_irq(info->irq);
	up(&info->work_lock);
	return ret;
}
#endif

#ifdef SEC_FACTORY_TEST
static bool get_short_data_lite(struct bt541_ts_info *info, u8 *buff, int skip_cnt)
{
	struct i2c_client *client = info->client;
	struct bt541_ts_platform_data *pdata = info->pdata;
	int i, j;

	info->work_state = RAW_DATA;

	for(i = 0; i < skip_cnt; i++) {
		for (j = 0; j < TSP_CMD_INT_WAIT_TIME && gpio_get_value(pdata->gpio_int); j++) {
			msleep(1);
		}

		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		msleep(1);
	}

	for (j = 0; j < TSP_CMD_INT_WAIT_TIME && gpio_get_value(pdata->gpio_int); j++) {
		msleep(1);
	}

	if (read_raw_data(client, BT541_RAWDATA_REG, (char *)(buff), CHECK_SHORT_DATA_NUM) < 0) {
			dev_err(&client->dev, "%s: Failed to read short data\n", __func__);
			info->work_state = NOTHING;
			return false;
		}

	write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
	info->work_state = NOTHING;
	return true;
}

static bool get_short_data(struct bt541_ts_info *info, u8 *buff, int skip_cnt)
{
	struct i2c_client *client = info->client;
	bool ret = false;

	disable_irq(info->irq);

	down(&info->work_lock);
	if (info->work_state != NOTHING) {
		dev_err(&client->dev, "%s: Other process occupied (%d)\n",
				__func__, info->work_state);
		enable_irq(info->irq);
		up(&info->work_lock);

		return false;
	}

	ret = get_short_data_lite(info, buff, skip_cnt);

	enable_irq(info->irq);
	up(&info->work_lock);
	return ret;
}
#endif
static bool ts_get_raw_data(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;
	u32 total_node = info->cap_info.total_node_num;
	int sz;
	u16 temp_sz;
	int	i;

	if (down_trylock(&info->raw_data_lock)) {
		dev_err(&client->dev, "%s: Failed to occupy work lock\n", __func__);
		info->touch_info.status = 0;

		return true;
	}

	sz = total_node * 2 + sizeof(struct point_info);

	for(i = 0; sz > 0; i++){
		temp_sz = I2C_BUFFER_SIZE;

		if(sz < I2C_BUFFER_SIZE)
			temp_sz = sz;

		if (read_raw_data(client, BT541_RAWDATA_REG + i, (char *)((u8*)(info->cur_data)+ (i*I2C_BUFFER_SIZE)), temp_sz) < 0) {
			dev_err(&client->dev, "%s: Failed to read raw data\n", __func__);
			up(&info->raw_data_lock);

			return false;
		}
		sz -= I2C_BUFFER_SIZE;
	}
	info->update = 1;
	memcpy((u8 *)(&info->touch_info),
		(u8 *)&info->cur_data[total_node],
			sizeof(struct point_info));
	up(&info->raw_data_lock);

	return true;
}

#define COORD_LOG_BUFFER_SIZE 64

static bool ts_read_coord(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;
#if (TOUCH_POINT_MODE == 1)
	int i;
#else
	int i = 0;
	s32 ret = 0;
	char log_buf[COORD_LOG_BUFFER_SIZE] = {0};
#endif

	 zinitix_debug_msg("ts_read_coord+\r\n");

	if (info->touch_mode != TOUCH_POINT_MODE) {
		if (ts_get_raw_data(info) == false)
			return false;

		dev_err(&client->dev, "%s: not read coord, "
			"touch_mode = %d, status = 0x%04X\n",
			__func__, info->touch_mode, info->touch_info.status);

		goto out;
	}

#if (TOUCH_POINT_MODE == 1)
	memset(&info->touch_info,
			0x0, sizeof(struct point_info));

	if (read_data(info->client, BT541_POINT_STATUS_REG,
			(u8 *)(&info->touch_info), 4) < 0) {
		dev_err(&client->dev, "%s: Failed to read point info\n", __func__);

		return false;
	}

	dev_dbg(&client->dev, "status reg = 0x%x , event_flag = 0x%04x\n",
				info->touch_info.status, info->touch_info.event_flag);

	if (info->touch_info.event_flag == 0)
		goto out;

	for (i = 0; i < info->cap_info.multi_fingers; i++) {
		if (zinitix_bit_test(info->touch_info.event_flag, i)) {
			udelay(20);

			if (read_data(info->client, BT541_POINT_STATUS_REG + 2 + ( i * 4),
					(u8 *)(&info->touch_info.coord[i]),
				sizeof(struct coord)) < 0) {
				dev_err(&client->dev, "Failed to read point info\n");

				return false;
			}
		}
	}

#else

		mutex_lock(&info->read_reg_mutex);
		ret = read_data(info->client, BT541_POINT_STATUS_REG,
				(u8 *)(&info->touch_info), sizeof(struct point_info));
		mutex_unlock(&info->read_reg_mutex);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to read point info\n");

			return false;
		}

		if (info->debug_level >= ZTX_DEBUG_LOG_COORD_INFO) {
			for(i = 0; i < sizeof(struct point_info); i ++) {
				snprintf(&log_buf[strlen(log_buf)], sizeof(log_buf) - strlen(log_buf) - 1,
					"%02X ", ((u8 *)(&info->touch_info))[i]);
			}
			dev_info(&client->dev, "coord { %s}\n", log_buf);
		}

#endif

out:
	/* error */
	if (zinitix_bit_test(info->touch_info.status, BIT_MUST_ZERO)) {
		dev_err(&client->dev, "Invalid must zero bit(%04x)\n",
			info->touch_info.status);
		/*write_cmd(info->client, BT541_CLEAR_INT_STATUS_CMD);
		udelay(DELAY_FOR_SIGNAL_DELAY);*/
		return false;
	}

	return true;
}

#ifdef ZTX_ESD_ENABLE
static void bt541_esd_timer_start(struct bt541_ts_info *info)
{
	schedule_delayed_work(&info->esd_test_work, msecs_to_jiffies(ZTX_CHECK_ESD_TIMER));
	info->esd_enable = true;
}

static void bt541_esd_timer_stop(struct bt541_ts_info *info)
{
	info->esd_enable = false;
	cancel_delayed_work(&info->esd_test_work);
}

static void bt541_esd_timer_handler(struct work_struct *work)
{
	struct bt541_ts_info *info = container_of(work,struct bt541_ts_info, esd_test_work.work);
	u16	esd_value = 0x0;
	s32 ret = 0;

	if (!info->esd_enable) {
		return;
	}

	mutex_lock(&info->read_reg_mutex);
	ret = read_data(info->client, ZTX_ESD_REG_ADDR, (u8 *)&esd_value, 2);
	mutex_unlock(&info->read_reg_mutex);

	if (ret < 0) {
		esd_value = 0xFFFF;
	}
	if (esd_value == ZTX_ESD_REG_VALUE) {
		if (info->debug_level >= ZTX_DEBUG_LOG_LOCATION) {
			dev_err(&info->client->dev, "esd check pass\n");
		}
	} else {
		dev_err(&info->client->dev, "esd check failed 0x%04X\n", esd_value);
		down(&info->work_lock);
		disable_irq(misc_info->irq);
		clear_report_data(info);
		resume_hw_reset(info,true);
		if (!bt541_power_sequence(info)) {
			bt541_power_control(info, POWER_OFF);
			bt541_power_control(info, POWER_ON_SEQUENCE);
		}
		enable_irq(misc_info->irq);
		up(&info->work_lock);
	}

	schedule_delayed_work(&info->esd_test_work, msecs_to_jiffies(ZTX_CHECK_ESD_TIMER));
}
#endif /* ZTX_ESD_ENABLE */

static bool bt541_power_sequence(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;
	int retry = 0;
	u16 chip_code;

	info->cap_info.ic_fw_size = 44*1024;

retry_power_sequence:
	if (write_reg(client, 0xc000, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "Failed to send power sequence(vendor cmd enable)\n");
		goto fail_power_sequence;
	}
	udelay(10);

	if (read_data(client, 0xcc00, (u8 *)&chip_code, 2) < 0) {
		dev_err(&client->dev, "Failed to read chip code\n");
		goto fail_power_sequence;
	}

	dev_info(&client->dev, "%s: chip code = 0x%x\n", __func__, chip_code);
	udelay(10);

	//Link modify the flash size
	if(chip_code == ZT7554_CHIP_CODE)
		info->cap_info.ic_fw_size = 64*1024;
	else if((chip_code == ZT7548_CHIP_CODE)||(chip_code == ZTW523_CHIP_CODE)||(chip_code == ZTW622_CHIP_CODE))
		info->cap_info.ic_fw_size = 48*1024;
	else if((chip_code == ZT7538_CHIP_CODE)||(chip_code == ZTW522_CHIP_CODE))
		info->cap_info.ic_fw_size = 44*1024;


	if (write_cmd(client, 0xc004) != I2C_SUCCESS) {
		dev_err(&client->dev, "Failed to send power sequence(intn clear)\n");
		goto fail_power_sequence;
	}
	udelay(10);

	if (write_reg(client, 0xc002, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "Failed to send power sequence(nvm init)\n");
		goto fail_power_sequence;
	}
	msleep(2);

	if (write_reg(client, 0xc001, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "Failed to send power sequence(program start)\n");
		goto fail_power_sequence;
	}
	msleep(FIRMWARE_ON_DELAY);	/* wait for checksum cal */

	dev_info(&client->dev, "bt541_power_sequence: OK\n");

	return true;

fail_power_sequence:
	if (retry++ < ZINITIX_INIT_RETRY_CNT) {
		msleep(CHIP_ON_DELAY);
		dev_info(&client->dev, "retry = %d\n", retry);
		goto retry_power_sequence;
	}

	dev_err(&client->dev, "Failed to send power sequence\n");
	return false;
}

#define ZTX_RESUME_RETRY_TIME 5

static bool zinitix_resume_sequence(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;
	int retry = 0;


retry_zinitix_resume_sequence:
	if (write_reg(client, 0xc000, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "Failed to send power sequence(vendor cmd enable)\n");
		goto fail_zinitix_resume_sequence;
	}
	msleep(1);

	if (write_cmd(client, 0xc004) != I2C_SUCCESS) {
		dev_err(&client->dev, "Failed to send power sequence(intn clear)\n");
		goto fail_zinitix_resume_sequence;
	}
	msleep(1);

	if (write_reg(client, 0xc002, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "Failed to send power sequence(nvm init)\n");
		goto fail_zinitix_resume_sequence;
	}
	msleep(1);

	if (write_reg(client, 0xc001, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "Failed to send power sequence(program start)\n");
		goto fail_zinitix_resume_sequence;
	}
	msleep(1);	/* wait for checksum cal */

	dev_info(&client->dev, "bt541_power_sequence: OK\n");

	return true;

fail_zinitix_resume_sequence:
	if (retry++ < ZTX_RESUME_RETRY_TIME) {
		bt541_power_control(info, POWER_OFF);
		bt541_power_control(info, POWER_ON);
		dev_info(&client->dev, "retry = %d\n", retry);
		goto retry_zinitix_resume_sequence;
	}
	dev_err(&client->dev, "Failed to send power sequence\n");
	return false;
}

static int zinitix_power_control(struct bt541_ts_info *data, int on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
		}
	}

	return rc;
}



static bool bt541_power_control(struct bt541_ts_info *info, u8 ctl)
{
	if (ctl == POWER_OFF) {
		dev_info(&info->client->dev, "%s: POWER_OFF\n", __func__);
		zinitix_power_control(info, 0);
		msleep(CHIP_OFF_DELAY);
	} else if (ctl == POWER_ON_SEQUENCE) {
		dev_info(&info->client->dev, "%s: POWER_ON_SEQUENCE\n", __func__);
		zinitix_power_control(info, 1);
		zinitix_hw_reset(info,true);
		msleep(CHIP_ON_DELAY);
		return bt541_power_sequence(info); //zxt power on sequence
	} else if (ctl == POWER_ON) {
		dev_info(&info->client->dev, "%s: POWER_ON\n", __func__);
		zinitix_power_control(info, 1);
		zinitix_hw_reset(info,true);
		msleep(CHIP_ON_DELAY);
	} else {
		dev_err(&info->client->dev, "%s: unknown ctl %d\n", __func__, ctl);
	}
	return true;
}

#ifdef USE_TSP_TA_CALLBACKS
static void bt541_set_ta_status(struct bt541_ts_info *info, bool force)
{
	printk("bt541_set ta_connected = %d\n", ta_connected);

	if(info == NULL)
		return;

	if (ta_connected)
		zinitix_bit_set(m_optional_mode, TS_USB_DETECT_BIT);
	else
		zinitix_bit_clr(m_optional_mode, TS_USB_DETECT_BIT);
}

static void bt541_charger_status_cb(struct tsp_callbacks *cb, int status)
{
	printk("bt541_charger status = %d\n", status);

	if (status)
		ta_connected = true;
	else
		ta_connected = false;

	bt541_set_ta_status(misc_info, true);

	dev_info(&misc_info->client->dev, "TA %s\n",
		status ? "connected" : "disconnected");
}
#endif
#if TOUCH_FORCE_UPGRADE
static void ts_select_type_hw(struct bt541_ts_info *info) {

#if (TSP_TYPE_COUNT == 1)
	m_FirmwareIdx = 0;
#else
	int i;
	u16 newHWID;

/* In case of TSP IC's firmware is broken,
	it will always be updated to HW ID 02 firmware even though HW 01 device exists*/
	for(i = 0; i < TSP_TYPE_COUNT; i++) {
		newHWID = (u16) (m_pFirmware[i][0x7528] | (m_pFirmware[i][0x7529]<<8));

		if(info->cap_info.hw_id == newHWID)
			break;
	}

	m_FirmwareIdx = i;
	if(i == TSP_TYPE_COUNT)
		m_FirmwareIdx = 1;
	zinitix_printk(KERN_INFO "firmwaretype = %d Firmware HWID = %u cap_info.hw_id = %u i = %d \n",
		m_FirmwareIdx, newHWID, info->cap_info.hw_id, i);
#endif
}

static bool ts_check_need_upgrade(struct bt541_ts_info *info)
{
	struct capa_info *cap = &(info->cap_info);
	u16	new_version;
	u16	new_minor_version;
	u16	new_reg_version;

	u8 *firmware_data = info->firmware_data;

	if (firmware_data == NULL) {
		dev_err(&info->client->dev, "%s: get firmware data failed\n", __func__);
		return false;
	}

	ztx_get_fw_version(info);

	new_version = (u16) (firmware_data[52] | (firmware_data[53]<<8));
	new_minor_version = (u16) (firmware_data[56] | (firmware_data[57]<<8));
	new_reg_version = (u16) (firmware_data[60] | (firmware_data[61]<<8));

	dev_info(&info->client->dev,
		"cur version 0x %02x %02x %02x, new version 0x %02x %02x %02x\n",
		cap->fw_version, cap->fw_minor_version, cap->reg_data_version,
		new_version, new_minor_version, new_reg_version);

	if (cap->fw_version != new_version) {
		return true;
	}

	if (cap->fw_minor_version != new_minor_version) {
		return true;
	}

	if (cap->reg_data_version != new_reg_version) {
		return true;
	}

	return false;
}

static bool ts_fw_checksum(struct bt541_ts_info *info)
{
	u16 chip_check_sum = 0xFFFF;
	int i;

	for (i = 0; i < INIT_RETRY_CNT; i++) {
		if (read_data(info->client, BT541_CHECKSUM_REG, (u8 *)&chip_check_sum, 2) < 0) {
			chip_check_sum = 0xFFFF;
			msleep(10);
		}
		if (chip_check_sum == BT541_CHECKSUM_SUCCESS_FLAG) {
			break;
		}
	}

	dev_info(&info->client->dev, "checksum = 0x%04X\n", chip_check_sum);

	return (chip_check_sum == BT541_CHECKSUM_SUCCESS_FLAG);
}
#endif

#define TC_SECTOR_SZ		8

#if TOUCH_FORCE_UPGRADE
static void ts_check_hwid_in_fatal_state(struct bt541_ts_info *info)
{
		//u16 flash_addr;
		u8 check_data[80];
		int i;
		u16 chip_code;
		u16 hw_id0, hw_id1_1, hw_id1_2;
		int retry = 0;
retry_fatal:
		bt541_power_control(info, POWER_OFF);
		bt541_power_control(info, POWER_ON);
		msleep(10);

		if (write_reg(info->client, 0xc000, 0x0001) != I2C_SUCCESS){
			dev_err(&info->client->dev,"power sequence error (vendor cmd enable)\n");
			goto fail_check_hwid;
		}
		udelay(10);
		if (read_data(info->client, 0xcc00, (u8 *)&chip_code, 2) < 0) {
			dev_err(&info->client->dev,"fail to read chip code\n");
			goto fail_check_hwid;
		}
		dev_info(&info->client->dev,"chip code = 0x%x\n", chip_code);
		udelay(10);
		if (write_cmd(info->client, 0xc004) != I2C_SUCCESS){
			dev_err(&info->client->dev,"power sequence error (intn clear)\n");
			goto fail_check_hwid;
		}
		udelay(10);
		if (write_reg(info->client, 0xc002, 0x0001) != I2C_SUCCESS){
			dev_err(&info->client->dev,"power sequence error (nvm init)\n");
			goto fail_check_hwid;
		}
		//dev_err(&info->client->dev, "init flash\n");
		msleep(5);
		if (write_reg(info->client, 0xc003, 0x0000) != I2C_SUCCESS){
			dev_err(&info->client->dev,"fail to write nvm vpp on\n");
			goto fail_check_hwid;
		}
		if (write_reg(info->client, 0xc104, 0x0000) != I2C_SUCCESS){
			dev_err(&info->client->dev,"fail to write nvm wp disable\n");
			goto fail_check_hwid;
		}
		//dev_err(&info->client->dev, "init flash\n");
		if (write_cmd(info->client, BT541_INIT_FLASH) != I2C_SUCCESS) {
			dev_err(&info->client->dev, "failed to init flash\n");
			goto fail_check_hwid;
		}
		//dev_err(&info->client->dev, "read firmware data\n");
		for (i = 0; i < 80; i+=TC_SECTOR_SZ) {
			if (read_firmware_data(info->client,
				BT541_READ_FLASH,
				(u8*)&check_data[i], TC_SECTOR_SZ) < 0) {
				dev_err(&info->client->dev, "error : read zinitix tc firmare\n");
				goto fail_check_hwid;
			}
		}
		hw_id0 = check_data[48] + check_data[49]*256;
		hw_id1_1 = check_data[70];
		hw_id1_2 = check_data[71];

		//dev_err(&info->client->dev, "eeprom hw id = %d, %d, %d\n", hw_id0, hw_id1_1, hw_id1_2);

		if(hw_id1_1 == hw_id1_2 && hw_id0 != hw_id1_1)
			info->cap_info.hw_id = hw_id1_1;
		else
			info->cap_info.hw_id = hw_id0;

		dev_err(&info->client->dev, "hw id = %d\n", info->cap_info.hw_id);
		msleep(5);
		return;

fail_check_hwid:
	if(retry++ <3) {
		dev_info(&info->client->dev, "fail to check hw id, retry cnt = %d\n", retry);
		msleep(5);
		goto retry_fatal;
	}
}

static bool verify_firmware(struct bt541_ts_info *info,
	u16 addr_low, u16 addr_high, u32 success_flag)
{
	struct i2c_client *client = info->client;
	const u16 VERIFY_REG = 0xcc01;
	u8 data_write[4] = {0};
	u32 data_read = 0;

	data_write[0] = (addr_low) & 0xff; /* addrL */
	data_write[1] = (addr_low >> 8)&0xff;
	data_write[2] = (addr_high) & 0xff; 	/* addrH */
	data_write[3] = (addr_high >> 8)&0xff;
	if (write_data(client, VERIFY_REG, data_write, sizeof(data_write)) < 0) {
		dev_err(&client->dev, "%s: Failed to send verify command\n", __func__);
		return false;
	}
	usleep_range(10, 20);

	if (read_data(client, VERIFY_REG, (u8 *)&data_read, sizeof(data_read)) < 0) {
		dev_err(&client->dev, "%s: Failed to read verify result\n", __func__);
		return false;
	}
	dev_info(&client->dev, "%s: verify data = 0x%x\n", __func__, data_read);

	if (data_read != success_flag) {
		return false;
	}
	return true;
}

static bool ts_upgrade_firmware(struct bt541_ts_info *info,
	const u8 *firmware_data, u32 size)
{
	struct i2c_client *client = info->client;
	struct bt541_ts_platform_data *pdata = info->pdata;
	u16 flash_addr;
	int retry_cnt = 0;
	int i;
	int page_sz = 64;
	u16 chip_code;
	const u16 USEFUL_ADDR_LOW = 0xb6fc;
	const u16 USEFUL_ADDR_HIGH = 0x0018;
	const u32 USEFUL_SUCCESS_FLAG = 0xA5A5A5A5;
	const u16 END_ADDR_LOW = 0xbffc;
	const u16 END_ADDR_HIGH = 0x0018;
	const u32 END_SUCCESS_FLAG = 0xFFFFFFFF;

retry_upgrade:
	//Step 1: Power off/on
	bt541_power_control(info, POWER_OFF);
	bt541_power_control(info, POWER_ON);
	msleep(10);

	//Step 2: Upgrade Sequence
	if (write_cmd(client, 0x01D5) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s(%d): jump to upgrade sequence fail\n", __func__, __LINE__);
		goto fail_upgrade;
	}

	msleep(5);

	if (write_reg(client, 0xc000, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s(%d): power sequence error (vendor cmd enable)\n", __func__, __LINE__);
		goto fail_upgrade;
	}

	usleep_range(10, 20);

	if (read_data(client, 0xcc00, (u8 *)&chip_code, 2) < 0) {
		dev_err(&client->dev, "%s(%d): failed to read chip code\n", __func__, __LINE__);
		goto fail_upgrade;
	}

	dev_info(&client->dev, "%s: chip code = 0x%x\n", __func__, chip_code);

	usleep_range(10, 20);

	if (write_cmd(client, 0xc004) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s(%d): power sequence error (intn clear)\n", __func__, __LINE__);
		goto fail_upgrade;
	}

	usleep_range(10, 20);

	if (write_reg(client, 0xc002, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s(%d): power sequence error (nvm init)\n", __func__, __LINE__);
		goto fail_upgrade;
	}

	msleep(5);

	dev_info(&client->dev, "%s(%d): init flash\n", __func__, __LINE__);

	if (write_reg(client, 0xc003, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s(%d): failed to write nvm vpp on\n", __func__, __LINE__);
		goto fail_upgrade;
	}

	if (write_reg(client, 0xc104, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s(%d): failed to write nvm wp disable\n", __func__, __LINE__);
		goto fail_upgrade;
	}

	//Step 3: Burst mode setting
	if (write_cmd(client, 0x01DF) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s(%d): failed to mass erase\n", __func__, __LINE__);
		goto fail_upgrade;
	}

	msleep(100);

	if (write_reg(client, 0x01D0, 0x0002) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s(%d): failed to init upgrade mode\n", __func__, __LINE__);
		goto fail_upgrade;
	}

	msleep(5); //Enter SRAM fast mode

	//setting package size 64(0x40)
	if (write_reg(client, 0x01D3, 0x0040) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s(%d): failed to set package size\n", __func__, __LINE__);
		goto fail_upgrade;
	}

	//Initial repeat size
	if (write_reg(client, 0x01D4, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s(%d): failed to init repeat size\n", __func__, __LINE__);
		goto fail_upgrade;
	}

	//Enter upgrade mode
	if (write_reg(client, 0x01DE, 0x0001) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s(%d): failed to enter upgrade mode\n", __func__, __LINE__);
		goto fail_upgrade;
	}

	msleep(1);

	//Step 4: Firmware Upgrade
	dev_info(&client->dev, "%s(%d): writing firmware data\n", __func__, __LINE__);
	for (flash_addr = 0; flash_addr < size; ) {
		if (write_data(client,BT541_WRITE_FLASH, (u8 *)&firmware_data[flash_addr], page_sz) < 0)
		{
			dev_err(&client->dev, "%s(%d): write zinitix tc firmare\n", __func__, __LINE__);
			goto fail_upgrade;
		}
		for (i = 0; i < TSP_CMD_INT_WAIT_TIME; i++) {
			if (gpio_get_value(pdata->gpio_int)) {
				break;
			}
			msleep(1);
		}
		flash_addr += page_sz;
	}

	if (write_cmd(client, 0x01DD) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s(%d): failed to flush\n", __func__, __LINE__);
		goto fail_upgrade;
	} else {
		dev_info(&client->dev, "%s(%d): bt541 flush ok\n", __func__, __LINE__);
	}

	for (i = 0; i < TSP_CMD_INT_WAIT_TIME; i++) {
		if (gpio_get_value(pdata->gpio_int)) {
			break;
		}
		msleep(1);
	}
	dev_info(&client->dev, "%s(%d): burst upgrade finished\n", __func__, __LINE__);
	msleep(100);

	if (write_reg(client, 0xc003, 0x0000) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s(%d): nvm write disable\n", __func__, __LINE__);
		goto fail_upgrade;
	}

	dev_info(&client->dev, "%s(%d): init flash\n", __func__, __LINE__);

	if (write_cmd(client, BT541_INIT_FLASH) != I2C_SUCCESS) {
		dev_err(&client->dev, "%s(%d): failed to init flash\n", __func__, __LINE__);
		goto fail_upgrade;
	}

	bt541_power_control(info, POWER_OFF);
	bt541_power_control(info, POWER_ON_SEQUENCE);

	//Step 5: verify firmware
	if (!verify_firmware(info, USEFUL_ADDR_LOW, USEFUL_ADDR_HIGH, USEFUL_SUCCESS_FLAG)) {
		dev_err(&client->dev, "%s(%d): failed to verify firmware\n", __func__, __LINE__);
		goto fail_upgrade;
	}

	if (!verify_firmware(info, END_ADDR_LOW, END_ADDR_HIGH, END_SUCCESS_FLAG)) {
		dev_err(&client->dev, "%s(%d): failed to verify firmware\n", __func__, __LINE__);
		goto fail_upgrade;
	}

	usleep_range(10, 20);

	bt541_power_control(info, POWER_OFF);
	bt541_power_control(info, POWER_ON_SEQUENCE);

	return true;

fail_upgrade:
	bt541_power_control(info, POWER_OFF);

	if (retry_cnt++ < INIT_RETRY_CNT) {
		dev_err(&client->dev, "upgrade failed : so retry... (%d)\n", retry_cnt);
		goto retry_upgrade;
	}

	dev_info(&client->dev, "Failed to upgrade\n");

	return false;
}

static void ztx_upgrade_firmware(struct bt541_ts_info *info, bool force_upgrade)
{
	struct i2c_client *client = info->client;
	int ret = 0;
	const struct firmware *fw = NULL;
	const u16 USEFUL_ADDR_LOW = 0xb6fc;
	const u16 USEFUL_ADDR_HIGH = 0x0018;
	const u32 USEFUL_SUCCESS_FLAG = 0xA5A5A5A5;
	const u16 END_ADDR_LOW = 0xbffc;
	const u16 END_ADDR_HIGH = 0x0018;
	const u32 END_SUCCESS_FLAG = 0xFFFFFFFF;

	info->firmware_data = NULL;

	if (oplus_panel_id == ZINITIX_PANEL_ID_V1) {
		ret = request_firmware(&fw, ZINITIX_FW_FILENAME_V1, &client->dev);
	} else if (oplus_panel_id == ZINITIX_PANEL_ID_V2) {
		ret = request_firmware(&fw, ZINITIX_FW_FILENAME_V2, &client->dev);
	} else {
		ret = request_firmware(&fw, ZINITIX_FW_FILENAME_V1, &client->dev);
	}

	if (ret != 0) {
		dev_err(&client->dev, "%s: failed to request firmware, ret = %d\n", __func__, ret);
		goto on_error;
	}

	info->firmware_data = (u8 *)vmalloc(fw->size);
	if (info->firmware_data == NULL) {
		dev_err(&client->dev, "%s: failed to vmalloc firmware data, ret = %d\n", __func__, ret);
		goto on_error;
	}

	memcpy(info->firmware_data, fw->data, fw->size);

	if (force_upgrade || (!ts_fw_checksum(info)) || ts_check_need_upgrade(info) ||
		(!verify_firmware(info, USEFUL_ADDR_LOW, USEFUL_ADDR_HIGH, USEFUL_SUCCESS_FLAG)) ||
		(!verify_firmware(info, END_ADDR_LOW, END_ADDR_HIGH, END_SUCCESS_FLAG))) {
		dev_info(&client->dev, "%s: upgrade firmware start\n", __func__);
		down(&info->work_lock);
		disable_irq(info->irq);
		ts_upgrade_firmware(info, info->firmware_data, fw->size);
		mini_init_touch(info);
		enable_irq(info->irq);
		up(&info->work_lock);
		dev_info(&client->dev, "%s: upgrade firmware end\n", __func__);
	}

on_error:
	if (fw != NULL) {
		release_firmware(fw);
		fw = NULL;
	}
	if (info->firmware_data != NULL) {
		vfree(info->firmware_data);
		info->firmware_data = NULL;
	}
	return;
}

static void ztx_upgrade_firmware_work(struct work_struct *work)
{
	if (misc_touch_dev) {
		ztx_upgrade_firmware(misc_touch_dev, false);
	}
}
#endif
static bool init_touch(struct bt541_ts_info *info)
{
	struct bt541_ts_platform_data *pdata = info->pdata;
	struct i2c_client *client = info->client;
	struct capa_info *cap = &(info->cap_info);
	u16 reg_val;
	int i;
	u16 chip_eeprom_info;

#if USE_CHECKSUM
	u16 chip_check_sum;
	u8 checksum_err;
#endif
	int retry_cnt = 0;
	char* productionMode = "androidboot.bsp=2";
	char* checkMode = NULL;

	dev_info(&client->dev, "init_touch: B\n");

	checkMode = strstr(saved_command_line, productionMode);
retry_init:
	for (i = 0; i < INIT_RETRY_CNT; i++) {
		if (read_data(client, BT541_EEPROM_INFO_REG,
				(u8 *)&chip_eeprom_info, 2) < 0) {
			dev_err(&client->dev, "Failed to read eeprom info(%d)\n", i);
			msleep(10);
			continue;
		} else
			break;
	}

	if (i == INIT_RETRY_CNT)
		goto fail_init;

#if USE_CHECKSUM
	dev_info(&client->dev, "%s: Check checksum\n", __func__);

	checksum_err = 0;

	for (i = 0; i < INIT_RETRY_CNT; i++) {
		if (read_data(client, BT541_CHECKSUM_REG,
				(u8 *)&chip_check_sum, 2) < 0) {
			msleep(10);
			continue;
		}

#if defined(TSP_VERBOSE_DEBUG)
		dev_info(&client->dev, "0x%04X\n", chip_check_sum);
#endif

		if(chip_check_sum == 0x55aa)
			break;
		else {
			checksum_err = 1;
		break;
	}
	}

	if (i == INIT_RETRY_CNT || checksum_err) {
		dev_err(&client->dev, "Failed to check firmware data\n");

		goto fail_init;
	}
#endif

	if (write_cmd(client, BT541_SWRESET_CMD) != I2C_SUCCESS) {
		dev_err(&client->dev, "Failed to write reset command\n");
		goto fail_init;
	}



	reg_val = 0;
	zinitix_bit_set(reg_val, BIT_PT_CNT_CHANGE);
	zinitix_bit_set(reg_val, BIT_DOWN);
	zinitix_bit_set(reg_val, BIT_MOVE);
	zinitix_bit_set(reg_val, BIT_UP);
#if  SUPPORTED_PALM_TOUCH
	zinitix_bit_set(reg_val, BIT_PALM);

#endif
	zinitix_bit_set(reg_val, BIT_PT_EXIST);


	if (cap->button_num > 0)
		zinitix_bit_set(reg_val, BIT_ICON_EVENT);

	cap->ic_int_mask = reg_val;

	dev_info(&client->dev, "ic_int_mask=%d:\n", cap->ic_int_mask);

	if (write_reg(client, BT541_INT_ENABLE_FLAG, 0x0) != I2C_SUCCESS)
		goto fail_init;

#if USE_WAKEUP_GESTURE
	if (write_reg(client, ZTX_REG_GESTURE_RESULT, 0) != I2C_SUCCESS)
		goto fail_init;
#endif

	dev_info(&client->dev, "%s: Send reset command\n", __func__);
	if (write_cmd(client, BT541_SWRESET_CMD) != I2C_SUCCESS)
		goto fail_init;

	/* get chip information */
	if (read_data(client, BT541_VENDOR_ID,
					(u8 *)&cap->vendor_id, 2) < 0) {
		zinitix_printk("failed to read chip revision\n");
		goto fail_init;
	}

	if (read_data(client, BT541_CHIP_REVISION,
					(u8 *)&cap->ic_revision, 2) < 0) {
		zinitix_printk("failed to read chip revision\n");
		goto fail_init;
	}

//	cap->ic_fw_size = 44*1024;

	if (read_data(client, BT541_HW_ID, (u8 *)&cap->hw_id, 2) < 0) {
		dev_err(&client->dev, "Failed to read hw id\n");
		goto fail_init;
	}
	if (read_data(client, BT541_THRESHOLD, (u8 *)&cap->threshold, 2) < 0)
		goto fail_init;


	if (read_data(client, BT541_BUTTON_SENSITIVITY,
					(u8 *)&cap->key_threshold, 2) < 0)
		goto fail_init;

	if (read_data(client, BT541_DUMMY_BUTTON_SENSITIVITY,
					(u8 *)&cap->dummy_threshold, 2) < 0)
		goto fail_init;

	if (read_data(client, BT541_TOTAL_NUMBER_OF_X,
					(u8 *)&cap->x_node_num, 2) < 0)
		goto fail_init;

	if (read_data(client, BT541_TOTAL_NUMBER_OF_Y,
					(u8 *)&cap->y_node_num, 2) < 0)
		goto fail_init;

	cap->total_node_num = cap->x_node_num * cap->y_node_num;

	if (read_data(client, BT541_DND_N_COUNT,
					(u8 *)&cap->N_cnt, 2) < 0)
		goto fail_init;

	zinitix_debug_msg("N count = %d\n", cap->N_cnt);

	if (read_data(client, BT541_DND_U_COUNT,
					(u8 *)&cap->u_cnt, 2) < 0)
		goto fail_init;

	zinitix_debug_msg("U count = %d\n", cap->u_cnt);

	if (read_data(client, BT541_AFE_FREQUENCY,
					(u8 *)&cap->afe_frequency, 2) < 0)
		goto fail_init;

	zinitix_debug_msg("AFE frequency = %d\n", cap->afe_frequency);


	/* get chip firmware version */
	ztx_get_fw_version(info);

	if (read_data(client, BT541_EEPROM_INFO_REG,
			(u8 *)&chip_eeprom_info, 2) < 0)
		goto fail_init;

	/* initialize */
	if (write_reg(client, BT541_X_RESOLUTION,
			(u16)pdata->x_resolution) != I2C_SUCCESS)
		goto fail_init;

	if (write_reg(client, BT541_Y_RESOLUTION,
			(u16)pdata->y_resolution) != I2C_SUCCESS)
		goto fail_init;

	cap->MinX = (u32)0;
	cap->MinY = (u32)0;
	cap->MaxX = (u32)pdata->x_resolution;
	cap->MaxY = (u32)pdata->y_resolution;

	if (write_reg(client, BT541_BUTTON_SUPPORTED_NUM,
		(u16)cap->button_num) != I2C_SUCCESS)
		goto fail_init;

	if (write_reg(client, BT541_SUPPORTED_FINGER_NUM,
		(u16)MAX_SUPPORTED_FINGER_NUM) != I2C_SUCCESS)
		goto fail_init;

	cap->multi_fingers = MAX_SUPPORTED_FINGER_NUM;

	zinitix_debug_msg("max supported finger num = %d\r\n",
		cap->multi_fingers);
	cap->gesture_support = 0;
	zinitix_debug_msg("set other configuration\r\n");

	if (write_reg(client, BT541_INITIAL_TOUCH_MODE,
					TOUCH_POINT_MODE) != I2C_SUCCESS)
		goto fail_init;

	if (write_reg(client, BT541_TOUCH_MODE, info->touch_mode) != I2C_SUCCESS)
		goto fail_init;

	if (read_data(client, ZINITIX_INTERNAL_FLAG_02,
		(u8 *)&reg_val, 2) < 0)
		goto fail_init;


	cap->i2s_checksum = 0;


	if (write_reg(client, BT541_INT_ENABLE_FLAG,
		cap->ic_int_mask) != I2C_SUCCESS)
		goto fail_init;

	/* Clear interrupt flag */
	for (i = 0; i < 10; i++) {
		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		udelay(10);
	}

	if (info->touch_mode != TOUCH_POINT_MODE) { /* Test Mode */
		if (write_reg(client, BT541_DELAY_RAW_FOR_HOST,
			RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS) {
			dev_err(&client->dev, "%s: Failed to set DELAY_RAW_FOR_HOST\n",
						__func__);

			goto fail_init;
		}
	}
	/* Clear interrupt flag */
	for (i = 0; i < 10; i++) {
		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		udelay(10);
	}

	zinitix_debug_msg("successfully initialized\r\n");
	return true;

fail_init:
	if (++retry_cnt < INIT_RETRY_CNT) {
		bt541_power_control(info, POWER_OFF);
		bt541_power_control(info, POWER_ON_SEQUENCE);

		zinitix_debug_msg("retry to initiallize(retry cnt = %d)\r\n",
				retry_cnt);
		goto	retry_init;

	} else if (retry_cnt == INIT_RETRY_CNT) {

		zinitix_debug_msg("retry to initiallize(retry cnt = %d)\r\n", retry_cnt);
#if TOUCH_FORCE_UPGRADE
		if(checkMode == NULL) {

			ts_check_hwid_in_fatal_state(info);
			ts_select_type_hw(info);
			if (ts_upgrade_firmware(info, m_pFirmware[m_FirmwareIdx],
				cap->ic_fw_size) == false) {
				zinitix_printk("upgrade failed\n");
				return false;
			}
		}
		else
			return true;
		msleep(100);

		goto retry_init;
#endif
	}

	dev_err(&client->dev, "Failed to initiallize\n");

	return false;
}

static bool mini_init_touch(struct bt541_ts_info *info)
{
	struct bt541_ts_platform_data *pdata = info->pdata;
	struct i2c_client *client = info->client;
	int i;

	if (write_cmd(client, BT541_SWRESET_CMD) != I2C_SUCCESS) {
		dev_info(&client->dev, "Failed to write reset command\n");

		goto fail_mini_init;
	}

	/* initialize */
	if (write_reg(client, BT541_X_RESOLUTION,
			(u16)(pdata->x_resolution)) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT541_Y_RESOLUTION,
			(u16)(pdata->y_resolution)) != I2C_SUCCESS)
		goto fail_mini_init;

	dev_info(&client->dev, "touch max x = %d\r\n", pdata->x_resolution);
	dev_info(&client->dev, "touch max y = %d\r\n", pdata->y_resolution);

	if (write_reg(client, BT541_BUTTON_SUPPORTED_NUM,
			(u16)info->cap_info.button_num) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT541_SUPPORTED_FINGER_NUM,
			(u16)MAX_SUPPORTED_FINGER_NUM) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT541_INITIAL_TOUCH_MODE,
			TOUCH_POINT_MODE) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT541_TOUCH_MODE,
			info->touch_mode) != I2C_SUCCESS)
		goto fail_mini_init;


	if (write_reg(client, BT541_INT_ENABLE_FLAG,
			info->cap_info.ic_int_mask) != I2C_SUCCESS)
		goto fail_mini_init;

	/* read garbage data */
	for (i = 0; i < 10; i++) {
		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		udelay(10);
	}

	if (info->touch_mode != TOUCH_POINT_MODE) {
		if (write_reg(client, BT541_DELAY_RAW_FOR_HOST,
				RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS){
			dev_err(&client->dev, "Failed to set BT541_DELAY_RAW_FOR_HOST\n");

			goto fail_mini_init;
		}
	}

	dev_err(&client->dev, "Successfully mini initialized\r\n");
	return true;

fail_mini_init:
	dev_err(&client->dev, "Failed to initialize mini init\n");

	return false;
}

#define FILTER_ALGO_STATE_UNUSED 0
#define FILTER_ALGO_STATE_CLICK 1
#define FILTER_ALGO_STATE_MOVE 2

#define FILTER_ALGO_SCALE 100

#define FIRST_FLAME_SHARE_EQUALLY_SCALE 3
#define FIRST_FLAME_SHARE_EQUALLY_MAX 30

#define KALMAN_SPEED_WEIGHT_HISTORY 1
#define KALMAN_SPEED_WEIGHT_INSTANT 3

#define KALMAN_MID_SPEED 600
#define KALMAN_PARA_1 24000
#define KALMAN_PARA_2 14400
#define KALMAN_SPEED_MIN 250
#define KALMAN_SPEED_MAX 2000

static void zinitix_first_flame_filter_data_init(int dx, int dy, struct first_flame_filter_algo_data * data)
{
	data -> flame = 0;
	data -> dx = dx;
	data -> dy = dy;
}

static void zinitix_kalman_filter_data_init(int position_report, struct kalman_filter_algo_data * data)
{
	data -> velocity_last = 0;
	data -> position_last = position_report;
	data -> history_position = position_report;
	data -> weighted_average_speed = 0;
}

static void zinitix_filter_data_init(struct bt541_ts_info *info, int index, int x, int y)
{
	info->filter_data[index].xn = x;
	info->filter_data[index].yn = y;
	zinitix_first_flame_filter_data_init(0, 0, &info->filter_data[index].delta_data);
	zinitix_kalman_filter_data_init(x * FILTER_ALGO_SCALE, &info->filter_data[index].x_data);
	zinitix_kalman_filter_data_init(y * FILTER_ALGO_SCALE, &info->filter_data[index].y_data);
}

static void zinitix_filter_data_clear(struct bt541_ts_info *info, int index)
{
	info->filter_data[index].state = FILTER_ALGO_STATE_UNUSED;
}

static int zinitix_abs(int v)
{
	return (v>=0?v:-v);
}

static int zinitix_cal_weighted_average_speed(int position_report, struct kalman_filter_algo_data * data)
{
	int ins_speed = zinitix_abs(position_report - data -> history_position); //Instantaneous speed
	int wei_ave_speed = data->weighted_average_speed; //Weighted average speed

	wei_ave_speed = (data->weighted_average_speed * KALMAN_SPEED_WEIGHT_HISTORY +
		ins_speed * KALMAN_SPEED_WEIGHT_INSTANT) / (KALMAN_SPEED_WEIGHT_HISTORY + KALMAN_SPEED_WEIGHT_INSTANT);

	data->weighted_average_speed = wei_ave_speed;
	data -> history_position = position_report;

	return wei_ave_speed;
}

static int zinitix_kalman_filter(int position_report, struct kalman_filter_algo_data * data, int speed)
{
	int R = 16;
	int Q = 1;

	int position_mid;
	int position_now;
	int velocity_mid;
	int velocity_now;

	if (speed < KALMAN_SPEED_MIN) {
		speed = KALMAN_SPEED_MIN;
	} else if (speed > KALMAN_SPEED_MAX) {
		speed = KALMAN_SPEED_MAX;
	}
	if (speed <= KALMAN_MID_SPEED) {
		R = (KALMAN_MID_SPEED - speed) * (KALMAN_MID_SPEED - speed) / KALMAN_PARA_1 + 1;
		Q = 1;
	} else {
		R = 1;
		Q = (speed - KALMAN_MID_SPEED) * (speed - KALMAN_MID_SPEED) / KALMAN_PARA_2 + 1;
	}


	position_mid = data -> position_last;
	velocity_mid = data -> velocity_last + Q;

	position_now = position_mid + velocity_mid * ((int)(position_report - position_mid)) / ((int)(velocity_mid + R));
	velocity_now = ((int)(R * velocity_mid)) / ((int)(velocity_mid + R));

	data -> velocity_last = velocity_now;
	data -> position_last = position_now;

	return position_now;
}

static void zinitix_filter_algo(struct bt541_ts_info *info,
		int index, int x, int y)
{
	int position_report = 0;
	int target_x = 0;
	int target_y = 0;
	int speed = 0;
	int speed_x = 0;
	int speed_y = 0;

	if (info->filter_data[index].state == FILTER_ALGO_STATE_UNUSED) {
		zinitix_filter_data_init(info, index, x, y);
		info->filter_data[index].state = FILTER_ALGO_STATE_CLICK;
	} else if (info->filter_data[index].state == FILTER_ALGO_STATE_CLICK) {
		if (info->filter_data[index].delta_data.flame < FIRST_FLAME_SHARE_EQUALLY_MAX) {
			info->filter_data[index].delta_data.flame ++;
		}
		if (x != info->filter_data[index].xn || y != info->filter_data[index].yn) {
			info->filter_data[index].delta_data.dx = (x - info->filter_data[index].xn) * FILTER_ALGO_SCALE;
			info->filter_data[index].delta_data.dy = (y - info->filter_data[index].yn) * FILTER_ALGO_SCALE;
			info->filter_data[index].delta_data.dx -= info->filter_data[index].delta_data.dx / info->filter_data[index].delta_data.flame;
			info->filter_data[index].delta_data.dy -= info->filter_data[index].delta_data.dy / info->filter_data[index].delta_data.flame;
			info->filter_data[index].delta_data.flame = info->filter_data[index].delta_data.flame * FIRST_FLAME_SHARE_EQUALLY_SCALE;
			if (info->filter_data[index].delta_data.flame > FIRST_FLAME_SHARE_EQUALLY_MAX) {
				info->filter_data[index].delta_data.flame = FIRST_FLAME_SHARE_EQUALLY_MAX;
			}
			target_x = x * FILTER_ALGO_SCALE - info->filter_data[index].delta_data.dx;
			target_y = y * FILTER_ALGO_SCALE - info->filter_data[index].delta_data.dy;
			info->filter_data[index].xn = (target_x + FILTER_ALGO_SCALE / 2) / FILTER_ALGO_SCALE;
			info->filter_data[index].yn = (target_y + FILTER_ALGO_SCALE / 2) / FILTER_ALGO_SCALE;
			zinitix_kalman_filter_data_init(target_x, &info->filter_data[index].x_data);
			zinitix_kalman_filter_data_init(target_y, &info->filter_data[index].y_data);
			info->filter_data[index].state = FILTER_ALGO_STATE_MOVE;
		}
	} else {
		if (info->filter_data[index].delta_data.flame > 0) {
			info->filter_data[index].delta_data.dx -= info->filter_data[index].delta_data.dx / info->filter_data[index].delta_data.flame;
			info->filter_data[index].delta_data.dy -= info->filter_data[index].delta_data.dy / info->filter_data[index].delta_data.flame;
			info->filter_data[index].delta_data.flame --;
			target_x = x * FILTER_ALGO_SCALE - info->filter_data[index].delta_data.dx;
			target_y = y * FILTER_ALGO_SCALE - info->filter_data[index].delta_data.dy;
		} else {
			target_x = x * FILTER_ALGO_SCALE;
			target_y = y * FILTER_ALGO_SCALE;
		}


		speed_x = zinitix_cal_weighted_average_speed(target_x,
			&info->filter_data[index].x_data);
		speed_y = zinitix_cal_weighted_average_speed(target_y,
			&info->filter_data[index].y_data);

		if (speed_x < speed_y) {
			speed = speed_y;
		} else {
			speed = speed_x;
		}

		position_report = zinitix_kalman_filter(target_x,
			&info->filter_data[index].x_data, speed);
		info->filter_data[index].xn = (position_report + FILTER_ALGO_SCALE / 2) / FILTER_ALGO_SCALE;

		position_report = zinitix_kalman_filter(target_y,
			&info->filter_data[index].y_data, speed);
		info->filter_data[index].yn = (position_report + FILTER_ALGO_SCALE / 2) / FILTER_ALGO_SCALE;
	}
}

static void clear_report_data(struct bt541_ts_info *info)
{
	int i;
	u8 reported = 0;
	u8 sub_status;

	input_report_key(info->input_dev, BTN_TOUCH, 0);

	for (i = 0; i < info->cap_info.multi_fingers; i++) {
		zinitix_filter_data_clear(info, i);
		sub_status = info->reported_touch_info.coord[i].sub_status;
		if (zinitix_bit_test(sub_status, SUB_BIT_EXIST)) {
			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev,	MT_TOOL_FINGER, 0);
			reported = true;
			if (!m_ts_debug_mode && TSP_NORMAL_EVENT_MSG)
				printk(KERN_INFO "[TSP] R %02d\r\n", i);
		}
		info->reported_touch_info.coord[i].sub_status = 0;
	}

	if (reported)
		input_sync(info->input_dev);

	info->finger_cnt1=0;
}

#define	PALM_REPORT_WIDTH	200
#define	PALM_REJECT_WIDTH	255

static irqreturn_t bt541_touch_work(int irq, void *data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)data;
	struct bt541_ts_platform_data *pdata = info->pdata;
	struct i2c_client *client = info->client;
	int i;

	u8 sub_status;
	u8 prev_sub_status;
	u32 x, y, maxX, maxY;
	u32 w;
	u32 tmp;
	//u8 read_result = 1;
	u8 palm = 0;
#if USE_WAKEUP_GESTURE
	u16 gesture_flag=0;
	int ret =0;
#endif

#if USE_WAKEUP_GESTURE
	if (info->pm_suspend && info->work_state == SUSPEND) {
		input_report_key(info->input_dev, KEY_F12, 1);
		input_report_key(info->input_dev, KEY_F12, 0);
		input_sync(info->input_dev);
		dev_info(&client->dev,"==touch to wakeup== KEY_F12, quick\n");
		return IRQ_HANDLED;
	}
#endif

	if (down_trylock(&info->work_lock)) {
		dev_err(&client->dev, "%s: Failed to occupy work lock\n", __func__);
		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);

		return IRQ_HANDLED;
	}

#if USE_WAKEUP_GESTURE
		if((tpd_halt)&&(info->work_state == SUSPEND)) //suspend state support gesture
		{
			if (info->pm_suspend) {
				input_report_key(info->input_dev, KEY_F12, 1);
				input_report_key(info->input_dev, KEY_F12, 0);
				input_sync(info->input_dev);
				dev_info(&client->dev,"==touch to wakeup== KEY_F12, skip pm suspend\n");
			} else if (read_data(client, ZTX_REG_GESTURE_RESULT,(u8 *)&gesture_flag, 2) < 0) {
				dev_err(&client->dev," gesture read reg error, force wakeup\r\n");

				ret=0;
				/*eric add 20160920*/
				//write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
			} else {
				/*wake up*/
				/*eric add 20160920*/
				//write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);

				if((gesture_flag == ZTX_GESTURE_CLICK) ||
					(info->aod_mode && gesture_flag == ZTX_GESTURE_SLIDE)) {
					dev_info(&client->dev,"==touch to wakeup== KEY_F12, flag:%d\n", gesture_flag);
					input_report_key(info->input_dev, KEY_F12, 1);
					//input_sync(info->input_dev);
					input_report_key(info->input_dev, KEY_F12, 0);
					input_sync(info->input_dev);

					if (write_reg(client, ZTX_REG_GESTURE_RESULT, 0) != 0)
					{
						dev_err(&client->dev," gesture write reg error!!!\n");
						ret=0;
					}

				} else {
					dev_err(&client->dev,"not handle, gesture wakeup flag:%d\n", gesture_flag);
				}

			}
			goto out;
		}

#endif

	if (info->work_state != NOTHING) {
		dev_err(&client->dev, "%s: Other process occupied\n", __func__);
		udelay(DELAY_FOR_SIGNAL_DELAY);
		// write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		goto out;
	}

	info->work_state = NORMAL;


	if (ts_read_coord(info) == false )
	{
		dev_err(&client->dev, "couldn't read touch_dev coord. read fail\r\n");

		bt541_power_control(info, POWER_ON_SEQUENCE);
		mini_init_touch(info);
		goto out;

	}


	/* invalid : maybe periodical repeated int. */
	if (info->touch_info.status == 0x0)
	{
	    //dev_err(&client->dev, "periodical interrupt\r\n");

		  goto out;
	}

#if SUPPORTED_PALM_TOUCH
	if (zinitix_bit_test(info->touch_info.status, BIT_PALM)) {

		//report up event first then process palm event.
			dev_err(&client->dev, "large touch palm enter\n");
			input_report_key(info->input_dev, KEY_SLEEP, 1);
			//input_sync(info->input_dev);
			input_report_key(info->input_dev, KEY_SLEEP, 0);
			input_sync(info->input_dev);
			palm = 1;
			goto out;
	}
#endif

	info->finger_cnt1 = 0;
	for (i = 0; i < info->cap_info.multi_fingers; i++) {
		sub_status = info->touch_info.coord[i].sub_status;
		prev_sub_status = info->reported_touch_info.coord[i].sub_status;

		if (zinitix_bit_test(sub_status, SUB_BIT_EXIST)) {
			x = info->touch_info.coord[i].x;
			y = info->touch_info.coord[i].y;
			w = info->touch_info.coord[i].width;

			zinitix_filter_algo(info, i, x, y);
			x = info->filter_data[i].xn;
			y = info->filter_data[i].yn;

			 /* transformation from touch to screen orientation */
			if (pdata->orientation & TOUCH_V_FLIP)
				y = info->cap_info.MaxY
					+ info->cap_info.MinY - y;

			if (pdata->orientation & TOUCH_H_FLIP)
				x = info->cap_info.MaxX
					+ info->cap_info.MinX - x;

			maxX = info->cap_info.MaxX;
			maxY = info->cap_info.MaxY;

			if (pdata->orientation & TOUCH_XY_SWAP) {
				zinitix_swap_v(x, y, tmp);
				zinitix_swap_v(maxX, maxY, tmp);
			}

			if (x > maxX || y > maxY) {
				dev_err(&client->dev,
							"Invalid coord %d : x=%d, y=%d\n", i, x, y);
				continue;
			}

			info->touch_info.coord[i].x = x;
			info->touch_info.coord[i].y = y;
			info->finger_cnt1 ++;

			//dev_err(&client->dev, "Finger [%02d] x = %d, y = %d,"
			//						" w = %d, p = %d\n", i, x, y, w, palm);

			if (zinitix_bit_test(sub_status, SUB_BIT_DOWN)){
				if (info->debug_level >= ZTX_DEBUG_LOG_LOCATION) {
					dev_info(&client->dev, "Finger [%02d] down (%d,%d)\n", i,
						info->touch_info.coord[i].x, info->touch_info.coord[i].y);
				} else {
					dev_info(&client->dev, "Finger [%02d] down\n", i);
				}
			}

			if (w == 0)
				w = 1;

			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 1);

#if (TOUCH_POINT_MODE == 2)
			if (palm == 0) {
				if (w >= PALM_REPORT_WIDTH)
					w = PALM_REPORT_WIDTH - 10;
			} else if (palm == 1) {	//palm report
				w = PALM_REPORT_WIDTH;
//				info->touch_info.coord[i].minor_width = PALM_REPORT_WIDTH;
			} else if (palm == 2){	// palm reject
//				x = y = 0;
				w = PALM_REJECT_WIDTH;
//				info->touch_info.coord[i].minor_width = PALM_REJECT_WIDTH;
			}
#endif

			input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, (u32)w);
			input_report_abs(info->input_dev, ABS_MT_PRESSURE, (u32)w);
			input_report_abs(info->input_dev, ABS_MT_WIDTH_MAJOR,
					(u32)((palm == 1) ? w-40 : w));
#if (TOUCH_POINT_MODE == 2)
			input_report_abs(info->input_dev,
				ABS_MT_TOUCH_MINOR, (u32)info->touch_info.coord[i].minor_width);
#endif
			if (get_display_vflip()) {
				input_report_abs(info->input_dev, ABS_MT_POSITION_X, info->pdata->x_resolution - x);
				input_report_abs(info->input_dev, ABS_MT_POSITION_Y, info->pdata->y_resolution - y);
			} else {
				input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
				input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
			}
		} else if (zinitix_bit_test(sub_status, SUB_BIT_UP) ||
			zinitix_bit_test(prev_sub_status, SUB_BIT_EXIST)) {
			if (info->debug_level >= ZTX_DEBUG_LOG_LOCATION) {
				dev_info(&client->dev, "Finger [%02d] up (%d,%d)\n",
					i, info->touch_info.coord[i].x, info->touch_info.coord[i].y);
			} else {
				dev_info(&client->dev, "Finger [%02d] up ver0x%02x hw0x%02x mode0x%02x\n",
					i, info->cap_info.reg_data_version,
					info->cap_info.hw_id, m_optional_mode);
			}
			zinitix_filter_data_clear(info, i);
			memset(&info->touch_info.coord[i], 0x0, sizeof(struct coord));
			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 0);

		} else {
			memset(&info->touch_info.coord[i], 0x0, sizeof(struct coord));
		}
	}
	memcpy((char *)&info->reported_touch_info, (char *)&info->touch_info,
		sizeof(struct point_info));

	if (info->finger_cnt1 > 0) {
		input_report_key(info->input_dev, BTN_TOUCH, 1);
	} else {
		input_report_key(info->input_dev, BTN_TOUCH, 0);
	}

	input_sync(info->input_dev);

out:
	write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);

	if (info->work_state == NORMAL) {
		info->work_state = NOTHING;
	}


	up(&info->work_lock);

	return IRQ_HANDLED;
}

#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB)
static int bt541_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bt541_ts_info *info = i2c_get_clientdata(client);

	dev_err(&client->dev, "bt541_ts_resume start \n");

	if(info->device_enabled) {
		dev_err(&client->dev, "%s: already enabled\n", __func__);
		return 0;
	}
	info->device_enabled = 1;

	down(&info->work_lock);
	if (info->work_state != SUSPEND) {
		dev_err(&client->dev, "%s: Invalid work proceedure (%d)\n",
				__func__, info->work_state);
		up(&info->work_lock);
		return 0;
	}
	info->work_state = RESUME;

	disable_irq(info->irq);

	if (info->power_off_mode) {
		bt541_power_control(info, POWER_ON_SEQUENCE);
		info->power_off_mode = false;
	} else {
		resume_hw_reset(info,true);
		if(!zinitix_resume_sequence(info)){
			dev_err(&client->dev, "zinitix_resume_sequence Fail\n");
		}
		msleep(50);
	}

	info->work_state = NOTHING;

	enable_irq(info->irq);

#ifdef ZTX_ESD_ENABLE
	bt541_esd_timer_start(info);
#endif /* ZTX_ESD_ENABLE */

	up(&info->work_lock);

	if (info->touch_mode != TOUCH_POINT_MODE) {
		dev_err(&client->dev,
			"%s: force reset touch_mode (%d) to TOUCH_POINT_MODE\n",
			__func__, info->touch_mode);
		ts_set_touchmode(TOUCH_POINT_MODE);
	}

	dev_err(&client->dev, "bt541_ts_resume end \n");

	tpd_halt = 0;

	return 0;
}

static void ztx_resume_work(struct work_struct *work)
{
	if (misc_touch_dev && misc_touch_dev->client) {
		bt541_ts_resume(&misc_touch_dev->client->dev);
	}
}


static int bt541_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bt541_ts_info *info = i2c_get_clientdata(client);

	dev_err(&client->dev, "bt541_ts_suspend start!\n");

	if(!info->device_enabled) {
		dev_err(&client->dev, "%s: already disabled\n", __func__);
		return 0;
	}
	info->device_enabled = 0;

	down(&info->work_lock);
	if (info->work_state != NOTHING
		&& info->work_state != SUSPEND) {
		dev_err(&client->dev, "%s: Invalid work proceedure (%d)\n",
			__func__, info->work_state);
		up(&info->work_lock);
		return 0;
	}

#ifdef ZTX_ESD_ENABLE
	bt541_esd_timer_stop(info);
#endif /* ZTX_ESD_ENABLE */

	disable_irq_wake(info->irq);

	clear_report_data(info);

	if ((!info->gesture_mode_en) && (!info->aod_mode)) {
		bt541_power_control(info, POWER_OFF);
		info->power_off_mode = true;
	} else {
		if (bt541_write_cmd(client, BT541_CLEAR_INT_STATUS_CMD) < 0) {
			dev_err(&client->dev,"%s: fail to clear int\n", __func__);
		}

		if (bt541_write_cmd(client, BT541_IDLE_CMD) < 0) {
			dev_err(&client->dev, "%s: fail to send idle cmd\n", __func__);
		}

		if (info->aod_mode) {
			if (bt541_write_reg(client, ZTX_REG_GESTURE_SUPPORT,
							ZTX_USE_GESTURE_CLICK_AND_SLIDE) != I2C_SUCCESS) {
				dev_err(&client->dev, "%s: set use gesture CLICK and SLIDE fail\n", __func__);
			} else {
				dev_info(&client->dev,"%s: set use gesture CLICK and SLIDE\n", __func__);
			}
		} else {
			if (bt541_write_reg(client, ZTX_REG_GESTURE_SUPPORT,
							ZTX_USE_GESTURE_CLICK) != I2C_SUCCESS) {
				dev_err(&client->dev, "%s: set use gesture CLICK fail\n", __func__);
			} else {
				dev_info(&client->dev,"%s: set use gesture CLICK\n", __func__);
			}
		}
		enable_irq_wake(info->irq);
	}

	tpd_halt = 1;
	info->work_state = SUSPEND;

	up(&info->work_lock);

	dev_err(&client->dev, "bt541_ts_suspend end \n");

	return 0;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	if (evdata && evdata->data &&
			misc_touch_dev && misc_touch_dev->client) {
		blank = evdata->data;
		if (*blank == MSM_DRM_BLANK_UNBLANK && event == MSM_DRM_EVENT_BLANK) {
			queue_work(misc_touch_dev->ts_workqueue, &misc_touch_dev->resume_work);
		} else if (*blank == MSM_DRM_BLANK_POWERDOWN && event == MSM_DRM_EARLY_EVENT_BLANK) {
			misc_touch_dev->aod_mode = false;
			cancel_work_sync(&misc_touch_dev->resume_work);
			bt541_ts_suspend(&misc_touch_dev->client->dev);
		} else if (*blank == MSM_DRM_EXIT_IDLE && event == MSM_DRM_EVENT_IDLE) {
			queue_work(misc_touch_dev->ts_workqueue, &misc_touch_dev->resume_work);
		} else if (*blank == MSM_DRM_ENTER_IDLE && event == MSM_DRM_EVENT_IDLE) {
			misc_touch_dev->aod_mode = true;
			cancel_work_sync(&misc_touch_dev->resume_work);
			bt541_ts_suspend(&misc_touch_dev->client->dev);
		}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void zinitix_late_resume(struct early_suspend *h)
{


	dev_err(&client->dev, "zinitix_late_resume\n");

	bt541_ts_resume(&misc_touch_dev->client->dev);
	dev_err(&client->dev, "zinitix_late_resume end\n");

}


static void zinitix_early_suspend(struct early_suspend *h)
{

	dev_err(&client->dev, "zinitix_early_suspend\n");
	bt541_ts_suspend(&misc_touch_dev->client->dev);
	dev_err(&client->dev, "zinitix_early_suspend end\n");

}


#endif	/* CONFIG_FB */
#endif

static void ts_set_touchmode_lite(u16 value)
{
	int i;

	misc_info->work_state = SET_MODE;

	if (value == TOUCH_DND_MODE) {
		if (write_reg(misc_info->client, BT541_DND_N_COUNT,
			SEC_DND_N_COUNT) != I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : "
					"Fail to set BT541_DND_N_COUNT %d.\n", SEC_DND_N_COUNT);
		if (write_reg(misc_info->client, BT541_DND_U_COUNT,
			SEC_DND_U_COUNT) != I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : "
					"Fail to set BT541_DND_U_COUNT %d.\n", SEC_DND_U_COUNT);
		if (write_reg(misc_info->client, BT541_AFE_FREQUENCY,
			SEC_DND_FREQUENCY) != I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : "
					"Fail to set BT541_AFE_FREQUENCY %d.\n", SEC_DND_FREQUENCY);
	}
	if (value == TOUCH_PDND_MODE) {
		if (write_reg(misc_info->client, BT541_DND_N_COUNT,
			SEC_PDND_N_COUNT) != I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : "
					"Fail to set BT541_DND_N_COUNT %d.\n", SEC_PDND_N_COUNT);
		if (write_reg(misc_info->client, BT541_DND_U_COUNT,
			SEC_PDND_U_COUNT) != I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : "
					"Fail to set BT541_DND_U_COUNT %d.\n", SEC_PDND_U_COUNT);
		if (write_reg(misc_info->client, BT541_AFE_FREQUENCY,
			SEC_PDND_FREQUENCY) != I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : "
					"Fail to set BT541_AFE_FREQUENCY %d.\n", SEC_PDND_FREQUENCY);
	}
	else if(misc_info->touch_mode == TOUCH_DND_MODE || misc_info->touch_mode == TOUCH_PDND_MODE) {
		if (write_reg(misc_info->client, BT541_DND_N_COUNT,
			misc_info->cap_info.N_cnt) != I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : "
				"Fail to reset BT541_AFE_FREQUENCY %d.\n",
				misc_info->cap_info.N_cnt);
		if (write_reg(misc_info->client, BT541_DND_U_COUNT,
			misc_info->cap_info.u_cnt) != I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : "
				"Fail to reset BT541_DND_U_COUNT %d.\n",
				misc_info->cap_info.u_cnt);
		if (write_reg(misc_info->client, BT541_AFE_FREQUENCY,
			misc_info->cap_info.afe_frequency) != I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : "
				"Fail to reset BT541_AFE_FREQUENCY %d.\n",
				misc_info->cap_info.afe_frequency);
	}

	if (value == TOUCH_SEC_MODE)
		misc_info->touch_mode = TOUCH_POINT_MODE;
	else
		misc_info->touch_mode = value;

	printk(KERN_INFO "[zinitix_touch] tsp_set_testmode, "
		"touchkey_testmode = %d\r\n", misc_info->touch_mode);

	if (misc_info->touch_mode != TOUCH_POINT_MODE) {
		if (write_reg(misc_info->client, BT541_DELAY_RAW_FOR_HOST,
			RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS)
			zinitix_printk("Fail to set BT541_DELAY_RAW_FOR_HOST.\r\n");
	}

	if (write_reg(misc_info->client, BT541_TOUCH_MODE,
			misc_info->touch_mode) != I2C_SUCCESS)
		printk(KERN_INFO "[zinitix_touch] TEST Mode : "
				"Fail to set ZINITX_TOUCH_MODE %d.\r\n", misc_info->touch_mode);

	if (write_cmd(misc_info->client, BT541_SWRESET_CMD) != I2C_SUCCESS) {
		zinitix_printk("Failed to write reset command\n");
		//dev_err(&client->dev, "Failed to write reset command\n");
		//goto fail_init;
	}
	msleep(400);

	/* clear garbage data */
	for (i = 0; i < 10; i++) {
		msleep(20);
		write_cmd(misc_info->client, BT541_CLEAR_INT_STATUS_CMD);
	}

	misc_info->work_state = NOTHING;
}

static bool ts_set_touchmode(u16 value)
{
	disable_irq(misc_info->irq);

	down(&misc_info->work_lock);
	if (misc_info->work_state != NOTHING) {
		dev_err(&misc_info->client->dev, "%s: other process occupied.. (%d)\n",
			__func__, misc_info->work_state);
		enable_irq(misc_info->irq);
		up(&misc_info->work_lock);
		return false;
	}

	ts_set_touchmode_lite(value);

	enable_irq(misc_info->irq);
	up(&misc_info->work_lock);
	return true;
}

static int ts_upgrade_sequence(const u8 *firmware_data)
{
	disable_irq(misc_info->irq);
	down(&misc_info->work_lock);
	misc_info->work_state = UPGRADE;

#ifdef ZTX_ESD_ENABLE
	bt541_esd_timer_stop(misc_info);
#endif /* ZTX_ESD_ENABLE */
	dev_info(&misc_info->client->dev, "clear all reported points\n");
	clear_report_data(misc_info);

	dev_info(&misc_info->client->dev, "start upgrade firmware\n");
	if (ts_upgrade_firmware(misc_info,
		firmware_data,
		misc_info->cap_info.ic_fw_size) == false) {
		enable_irq(misc_info->irq);
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return -1;
	}

	dev_info(&misc_info->client->dev, "upgrade firmware finish, reset TP\n");
	resume_hw_reset(misc_info,true);
	if (!bt541_power_sequence(misc_info)) {
		bt541_power_control(misc_info, POWER_OFF);
		bt541_power_control(misc_info, POWER_ON_SEQUENCE);
	}

	enable_irq(misc_info->irq);
	misc_info->work_state = NOTHING;
	up(&misc_info->work_lock);
	return 0;
}


#if 1
/*adb operation*/
#ifdef SEC_FACTORY_TEST
static inline void set_cmd_result(struct bt541_ts_info *info, char *buff, int len)
{
	strncat(info->factory_info->cmd_result, buff, len);
}

static inline void set_default_result(struct bt541_ts_info *info)
{
	char delim = ':';
	memset(info->factory_info->cmd_result, 0x00, ARRAY_SIZE(info->factory_info->cmd_result));
	memcpy(info->factory_info->cmd_result, info->factory_info->cmd, strlen(info->factory_info->cmd));
	strncat(info->factory_info->cmd_result, &delim, 1);
}

static void fw_update(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct capa_info *cap = &(info->cap_info);
	int ret = 0;
	u8 *buff = 0;
	mm_segment_t old_fs = {0};
	struct file *fp = NULL;
	long fsize = 0, nread = 0;
	char fw_path[MAX_FW_PATH + 1];
	char result[16] = {0};

	ztx_get_fw_version(info);
	dev_info(&client->dev, "%s: defore update, fw fersion 0x %02x %02x %02x\n",
		__func__, cap->fw_version, cap->fw_minor_version, cap->reg_data_version);

	set_default_result(info);

	switch (info->factory_info->cmd_param[0]) {
	case BUILT_IN:
		dev_info(&client->dev, "%s: BUILT_IN mode\n", __func__);
		ts_select_type_hw(info);
		ret = ts_upgrade_sequence((u8*)m_pFirmware[m_FirmwareIdx]);
		if(ret<0) {
			info->factory_info->cmd_state = 3;
			return;
		}
		break;

	case UMS:
		dev_info(&client->dev, "%s: UMS mode\n", __func__);
		old_fs = get_fs();
		set_fs(KERNEL_DS);

		if (oplus_panel_id == ZINITIX_PANEL_ID_V1) {
			snprintf(fw_path, MAX_FW_PATH, "/sdcard/%s", ZINITIX_FW_FILENAME_V1);
		} else if (oplus_panel_id == ZINITIX_PANEL_ID_V2) {
			snprintf(fw_path, MAX_FW_PATH, "/sdcard/%s", ZINITIX_FW_FILENAME_V2);
		} else {
			snprintf(fw_path, MAX_FW_PATH, "/sdcard/%s", ZINITIX_FW_FILENAME_V1);
		}

		fp = filp_open(fw_path, O_RDONLY, 0);
		if (IS_ERR(fp)) {
			dev_err(&client->dev,
				"file %s open \n", fw_path);
			info->factory_info->cmd_state = 3;
			goto err_open;
		}

		fsize = fp->f_path.dentry->d_inode->i_size;

		if (fsize != info->cap_info.ic_fw_size) {
			dev_err(&client->dev, "invalid fw size!! %d,%d\n", fsize, info->cap_info.ic_fw_size);
			info->factory_info->cmd_state = 3;
			goto err_open;
		}

		buff = (u8 *)vmalloc(fsize);
		if (NULL == buff) {
			dev_err(&client->dev, "failed to alloc buffer for fw\n");
			info->factory_info->cmd_state = 3;
			goto err_alloc;
		}

		nread = vfs_read(fp, (char *)buff, fsize, &fp->f_pos);
		if (nread != fsize) {
			dev_info(&client->dev, "failed to vfs_read fw %d,%d\n", nread, fsize);
			info->factory_info->cmd_state = 3;
			goto err_fw_size;
		}

		filp_close(fp, current->files);
		set_fs(old_fs);
		dev_info(&client->dev, "ums fw is loaded!!\n");
		info->checkUMSmode = true;
		ret = ts_upgrade_sequence((u8 *)buff);
		info->checkUMSmode = false;
		if(ret<0) {
			info->factory_info->cmd_state = 3;
			goto err_upgrade;
		}

		if (buff != NULL) {
			vfree(buff);
		}

		if (fp != NULL) {
			filp_close(fp, NULL);
			set_fs(old_fs);
		}

		break;

	default:
		dev_err(&client->dev, "invalid fw file type!!\n");
		goto not_support;
	}

	info->factory_info->cmd_state = 2;
	snprintf(result, sizeof(result) , "%s", "OK");
	set_cmd_result(info, result,
			strnlen(result, sizeof(result)));

	ztx_get_fw_version(info);
	dev_info(&client->dev, "%s: update success, fw fersion 0x %02x %02x %02x\n",
		__func__, cap->fw_version, cap->fw_minor_version, cap->reg_data_version);

	return;

err_upgrade:
err_fw_size:
	if (buff != NULL) {
		vfree(buff);
	}

if (fp != NULL) {
err_alloc:
	filp_close(fp, NULL);
err_open:
	set_fs(old_fs);
}
not_support:
	dev_info(&client->dev, "%s: update fail\n", __func__);
	snprintf(result, sizeof(result) , "%s", "NG");
	set_cmd_result(info, result, strnlen(result, sizeof(result)));
	return;
}

static void get_fw_ver_bin(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	u16 fw_version, fw_minor_version, reg_version, hw_id;
	u32 version;
	u8 *firmware_data;

	set_default_result(info);

	/* To Do */
	/* modify m_firmware_data */
	ts_select_type_hw(info);
	firmware_data = (u8*)m_pFirmware[m_FirmwareIdx];

	fw_version = (u16)(firmware_data[52] | (firmware_data[53] << 8));
	fw_minor_version = (u16)(firmware_data[56] | (firmware_data[57] << 8));
	reg_version = (u16)(firmware_data[60] | (firmware_data[61] << 8));
	hw_id = (u16)(firmware_data[0x7528] | (firmware_data[0x7529] << 8));
	version = (u32)((u32)(hw_id & 0xff) << 16) | ((fw_version & 0xf ) << 12)
				| ((fw_minor_version & 0xf) << 8) | (reg_version & 0xff);
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "ZI%06X", version);
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_fw_ver_ic(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	u16 fw_version, fw_minor_version, reg_version, hw_id;
	u32 version;

	set_default_result(info);
	ztx_get_fw_version(info);

	fw_version = info->cap_info.fw_version;
	fw_minor_version = info->cap_info.fw_minor_version;
	reg_version = info->cap_info.reg_data_version;
	hw_id = info->cap_info.hw_id;
	version = (u32)((u32)(hw_id & 0xff) << 16) | ((fw_version & 0xf) << 12)
		| ((fw_minor_version & 0xf) << 8) | (reg_version & 0xff);
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "ZI%06X", version);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_threshold(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
		"%d", info->cap_info.threshold);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void module_off_master(void *device_data)
{
	return;
}

static void module_on_master(void *device_data)
{
	return;
}

static void module_off_slave(void *device_data)
{
	return;
}

static void module_on_slave(void *device_data)
{
	return;
}

static void get_module_vendor(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *fdata = info->factory_info;
	char buff[16] = {0};
	int val,val2;

	set_default_result(info);
	if (!(gpio_get_value(info->pdata->tsp_en_gpio)) ) {
		dev_err(&client->dev, "%s: [ERROR] Touch is stopped\n",
				__func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		fdata->cmd_state = NOT_APPLICABLE;
		return;
	}
	if (info->pdata->tsp_vendor1 > 0 && info->pdata->tsp_vendor2 > 0 ) {
		val = gpio_get_value(info->pdata->tsp_vendor1);
		val2 = gpio_get_value(info->pdata->tsp_vendor2);
		dev_info(&info->client->dev,
			"%s: TSP_ID: %d[%d]%d[%d]\n", __func__,
			info->pdata->tsp_vendor1, val,info->pdata->tsp_vendor2, val2);
		snprintf(buff, sizeof(buff), "%s,%d%d", tostring(OK), val,val2);
		fdata->cmd_state = OK;
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		return;
	}
	snprintf(buff, sizeof(buff),  "%s", tostring(NG));
	fdata->cmd_state = FAIL;
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
}


#define BT541_VENDOR_NAME "ZINITIX"

static void get_chip_vendor(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
		"%s", BT541_VENDOR_NAME);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_config_ver(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	//const char *model_name = info->pdata->pname;

	set_default_result(info);

	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

#define BT541_CHIP_NAME "BT541"

static void get_chip_name(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", BT541_CHIP_NAME);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_x_num(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
		"%u", info->cap_info.x_node_num);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_y_num(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
		"%u", info->cap_info.y_node_num);
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void not_support_cmd(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	sprintf(finfo->cmd_buff, "%s", "NA");
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	mutex_lock(&finfo->cmd_lock);
	finfo->cmd_is_running = false;
	mutex_unlock(&finfo->cmd_lock);
	info->factory_info->cmd_state = WAITING;

	//dev_info(&client->dev, "%s: \"%s(%d)\"\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_reference(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	unsigned int val;
	int x_node, y_node;
	int node_num;

	set_default_result(info);

	x_node = finfo->cmd_param[0];
	y_node = finfo->cmd_param[1];

	if (x_node < 0 || x_node >= info->cap_info.x_node_num ||
		y_node < 0 || y_node >= info->cap_info.y_node_num) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "abnormal");
		set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		info->factory_info->cmd_state = FAIL;
		return;
	}

	node_num = x_node * info->cap_info.y_node_num + y_node;

	val = raw_data->ref_data[node_num];
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%u", val);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void run_preference_read(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	//u16 min, max;
	s32 i;

	s16 min[36] = {
		-200, 2211, 4053, 3839, 1990, -200,
		2114, 4201, 4199, 4280, 4563, 2627,
		4033, 4211, 4206, 4319, 4775, 4630,
		4018, 4215, 4194, 4329, 4782, 4679,
		2062, 4194, 4193, 4315, 4624, 3319,
		-200, 2049, 3871, 4176, 2388, -200,
	};
	s16 max[36] = {
		30000,	4963,	7205,	6825,	4570,	30000,
		4792,	7469,	7465,	7610,	8113,	5703,
		7171,	7487,	7476,	7679,	8489,	8232,
		7144,	7493,	7456,	7697,	8502,	8319,
		4700,	7456,	7453,	7671,	8220,	6933,
		30000,	4675,	6883,	7424,	5278,	30000,
	};

	set_default_result(info);

	ts_set_touchmode(TOUCH_PDND_MODE);
	get_raw_data(info, (unsigned char *)raw_data->pref_data, 10);
	ts_set_touchmode(TOUCH_POINT_MODE);

	for(i = 0; i < (info->cap_info.x_node_num * info->cap_info.y_node_num); i++)
	{
		if(((raw_data->pref_data[i]) < min[i]) || ((raw_data->pref_data[i]) > max[i]))
		{
			set_cmd_result(info, finfo->cmd_buff,
					strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
			finfo->cmd_state = FAIL;
				return;
		} else {
			continue;
		}
	}

	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;
	
	return;

}

static void run_short_data_read(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;

	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	const u16 NORMAL_SHORT_VALUE=1000;
	u16 i;

	set_default_result(info);

	ts_set_touchmode(TOUCH_CHECK_SHORT_MODE);
	get_short_data(info, (u8 *)raw_data->pref_data, 10);
	ts_set_touchmode(TOUCH_POINT_MODE);

	for(i = 0; i < (CHECK_SHORT_DATA_NUM>>1); i++)
	{
		if (raw_data->pref_data[i]!=NORMAL_SHORT_VALUE){
			pr_info("Invalid Check Short Value,Test Short!\n");
			finfo->cmd_state = FAIL;
			return;
		}

	}

	finfo->cmd_state = OK;
	return;
}

static void get_preference(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	unsigned int val;
	int x_node, y_node;
	int node_num;

	set_default_result(info);

	x_node = finfo->cmd_param[0];
	y_node = finfo->cmd_param[1];

	if (x_node < 0 || x_node >= info->cap_info.x_node_num ||
		y_node < 0 || y_node >= info->cap_info.y_node_num) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "abnormal");
		set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		info->factory_info->cmd_state = FAIL;

		return;
	}

	node_num = x_node * info->cap_info.y_node_num + y_node;

	val = raw_data->pref_data[node_num];
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%u", val);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void run_delta_read(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	s16 min, max;
	s32 i, j;

	set_default_result(info);

	ts_set_touchmode(TOUCH_DELTA_MODE);
	get_raw_data(info, (u8 *)(u8 *)raw_data->delta_data, 10);
	ts_set_touchmode(TOUCH_POINT_MODE);
	finfo->cmd_state = OK;

	min = (s16)0x7FFF;
	max = (s16)0x8000;

	for(i = 0; i < info->cap_info.x_node_num; i++)
	{
		for(j = 0; j < info->cap_info.y_node_num; j++)
		{
			/*printk("delta_data : %d \n", raw_data->delta_data[j+i]);*/

			if (raw_data->delta_data[i * info->cap_info.y_node_num + j] < min &&
				raw_data->delta_data[i * info->cap_info.y_node_num + j] != 0)
				min = raw_data->delta_data[i * info->cap_info.y_node_num + j];

			if (raw_data->delta_data[i * info->cap_info.y_node_num + j] > max)
				max = raw_data->delta_data[i * info->cap_info.y_node_num + j];

		}
		/*printk("\n");*/
	}

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%d,%d\n", min, max);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	//dev_info(&client->dev, "%s: \"%s\"(%d)\n", __func__, finfo->cmd_buff,
	//	strlen(finfo->cmd_buff));

	return;
}

static void get_delta(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	unsigned int val;
	int x_node, y_node;
	int node_num;

	set_default_result(info);

	x_node = finfo->cmd_param[0];
	y_node = finfo->cmd_param[1];

	if (x_node < 0 || x_node >= info->cap_info.x_node_num ||
		y_node < 0 || y_node >= info->cap_info.y_node_num) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "abnormal");
		set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		info->factory_info->cmd_state = FAIL;

		return;
	}

	node_num = x_node * info->cap_info.y_node_num + y_node;

	val = raw_data->delta_data[node_num];
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%u", val);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	info->factory_info->cmd_state = OK;

	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

#ifdef GLOVE_MODE
static void glove_mode(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	//struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	if (finfo->cmd_param[0] < 0 || finfo->cmd_param[0] > 1) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "NG");
		finfo->cmd_state = FAIL;
	} else {
		if (finfo->cmd_param[0])
			zinitix_bit_set(m_optional_mode, TS_SENSIVE_MODE_BIT);
		else
			zinitix_bit_clr(m_optional_mode, TS_SENSIVE_MODE_BIT);

		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "OK");
		finfo->cmd_state = OK;
	}

	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	mutex_lock(&finfo->cmd_lock);
	finfo->cmd_is_running = false;
	mutex_unlock(&finfo->cmd_lock);

	finfo->cmd_state = WAITING;
	//dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
	//	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}
#endif

static ssize_t store_cmd(struct device *dev, struct device_attribute
				  *devattr, const char *buf, size_t count)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	char *cur, *start, *end;
	char buff[TSP_CMD_STR_LEN] = {0};
	int len, i;
	struct tsp_cmd *tsp_cmd_ptr = NULL;
	char delim = ',';
	bool cmd_found = false;
	int param_cnt = 0;

	if (finfo->cmd_is_running == true) {
		dev_err(&client->dev, "%s: other cmd is running\n", __func__);
		goto err_out;
	}

	/* check lock  */
	mutex_lock(&finfo->cmd_lock);
	finfo->cmd_is_running = true;
	mutex_unlock(&finfo->cmd_lock);

	finfo->cmd_state = RUNNING;

	for (i = 0; i < ARRAY_SIZE(finfo->cmd_param); i++)
		finfo->cmd_param[i] = 0;

	len = (int)count;
	if (*(buf + len - 1) == '\n')
		len--;

	memset(finfo->cmd, 0x00, ARRAY_SIZE(finfo->cmd));
	memcpy(finfo->cmd, buf, len);

	cur = strchr(buf, (int)delim);
	if (cur)
		memcpy(buff, buf, cur - buf);
	else
		memcpy(buff, buf, len);

	/* find command */
	list_for_each_entry(tsp_cmd_ptr, &finfo->cmd_list_head, list) {
		if (!strcmp(buff, tsp_cmd_ptr->cmd_name)) {
			cmd_found = true;
			break;
		}
	}

	/* set not_support_cmd */
	if (!cmd_found) {
		list_for_each_entry(tsp_cmd_ptr, &finfo->cmd_list_head, list) {
			if (!strcmp("not_support_cmd", tsp_cmd_ptr->cmd_name))
				break;
		}
	}

	/* parsing parameters */
	if (cur && cmd_found) {
		cur++;
		start = cur;
		memset(buff, 0x00, ARRAY_SIZE(buff));
		do {
			if (*cur == delim || cur - buf == len) {
				end = cur;
				memcpy(buff, start, end - start);
				*(buff + strlen(buff)) = '\0';
				finfo->cmd_param[param_cnt] =
					(int)simple_strtol(buff, NULL, 10);
				start = cur + 1;
				memset(buff, 0x00, ARRAY_SIZE(buff));
				param_cnt++;
			}
			cur++;
		} while (cur - buf <= len);
	}

	dev_info(&client->dev, "%s: cmd = %s\n", __func__, tsp_cmd_ptr->cmd_name);
/*	for (i = 0; i < param_cnt; i++)
		dev_info(&client->dev, "cmd param %d= %d\n", i, finfo->cmd_param[i]);*/

	tsp_cmd_ptr->cmd_func(info);

err_out:
	return count;
}

static ssize_t show_cmd_status(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	dev_info(&client->dev, "tsp cmd: status:%d\n", finfo->cmd_state);

	if (finfo->cmd_state == WAITING)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "WAITING");

	else if (finfo->cmd_state == RUNNING)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "RUNNING");

	else if (finfo->cmd_state == OK)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "OK");

	else if (finfo->cmd_state == FAIL)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "FAIL");

	else if (finfo->cmd_state == NOT_APPLICABLE)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "NOT_APPLICABLE");

	return snprintf(buf, sizeof(finfo->cmd_buff),
					"%s\n", finfo->cmd_buff);
}

static ssize_t show_cmd_result(struct device *dev, struct device_attribute
				    *devattr, char *buf)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	dev_info(&client->dev, "tsp cmd: result: %s\n", finfo->cmd_result);

	mutex_lock(&finfo->cmd_lock);
	finfo->cmd_is_running = false;
	mutex_unlock(&finfo->cmd_lock);

	finfo->cmd_state = WAITING;

	return snprintf(buf, sizeof(finfo->cmd_result),
					"%s\n", finfo->cmd_result);
}

static DEVICE_ATTR(cmd, S_IWUSR | S_IWGRP, NULL, store_cmd);
static DEVICE_ATTR(cmd_status, S_IRUGO, show_cmd_status, NULL);
static DEVICE_ATTR(cmd_result, S_IRUGO, show_cmd_result, NULL);

static struct attribute *touchscreen_attributes[] = {
	&dev_attr_cmd.attr,
	&dev_attr_cmd_status.attr,
	&dev_attr_cmd_result.attr,
	NULL,
};

static struct attribute_group touchscreen_attr_group = {
	.attrs = touchscreen_attributes,
};




static int init_sec_factory(struct bt541_ts_info *info)
{
	struct device *factory_ts_dev;

	struct tsp_factory_info *factory_info;
	struct tsp_raw_data *raw_data;
	int ret;
	int i;

	factory_info = kzalloc(sizeof(struct tsp_factory_info), GFP_KERNEL);
	if (unlikely(!factory_info)) {
		dev_err(&info->client->dev, "%s: Failed to allocate memory\n",
				__func__);
		ret = -ENOMEM;

		goto err_alloc1;
	}
	raw_data = kzalloc(sizeof(struct tsp_raw_data), GFP_KERNEL);
	if (unlikely(!raw_data)) {
		dev_err(&info->client->dev, "%s: Failed to allocate memory\n",
				__func__);
		ret = -ENOMEM;

		goto err_alloc2;
	}

	INIT_LIST_HEAD(&factory_info->cmd_list_head);
	for (i = 0; i < ARRAY_SIZE(tsp_cmds); i++)
		list_add_tail(&tsp_cmds[i].list, &factory_info->cmd_list_head);


	sec_class = class_create(THIS_MODULE, "tsp");

	factory_ts_dev = device_create(sec_class, NULL, 0, info, "tsp");
	if (unlikely(!factory_ts_dev)) {
		dev_err(&info->client->dev, "Failed to create factory dev\n");
		ret = -ENODEV;
		goto err_create_device;
	}

	ret = sysfs_create_link(&factory_ts_dev->kobj,
		&info->input_dev->dev.kobj, "input");
	if (ret < 0) {
		dev_err(&info->client->dev,
			"%s: Failed to create input symbolic link %d\n",
			__func__, ret);
	}


	ret = sysfs_create_group(&factory_ts_dev->kobj, &touchscreen_attr_group);
	if (unlikely(ret)) {
		dev_err(&info->client->dev, "Failed to create touchscreen sysfs group\n");
		goto err_create_sysfs;
	}

	mutex_init(&factory_info->cmd_lock);
	factory_info->cmd_is_running = false;

	info->factory_info = factory_info;
	info->raw_data = raw_data;

	return ret;

err_create_sysfs:
err_create_device:
	kfree(raw_data);
err_alloc2:
	kfree(factory_info);
err_alloc1:

	return ret;
}

struct bt541_ts_info *g_ztx_ts_info = NULL;
#define RESULT_BUFFER_SIZE        (1024*80*5)
char * g_ztx_test_result_buffer = NULL;
int g_ztx_test_result_buffer_index = 0;
int g_ztx_autotest_result = 0;

static void ztx_get_fw_version(struct bt541_ts_info *ts_info)
{
	struct i2c_client *client = ts_info->client;
	struct capa_info *cap = &(ts_info->cap_info);

	/* get chip firmware version */
	if (read_data(client, BT541_FIRMWARE_VERSION, (u8 *)&cap->fw_version, 2) < 0) {
		cap->fw_version = 0xFFFF;
	}

	if (read_data(client, BT541_MINOR_FW_VERSION, (u8 *)&cap->fw_minor_version, 2) < 0) {
		cap->fw_minor_version = 0xFFFF;
	}

	if (read_data(client, BT541_DATA_VERSION_REG, (u8 *)&cap->reg_data_version, 2) < 0) {
		cap->reg_data_version = 0xFFFF;
	}
}

static int ztx_get_devinfo(struct bt541_ts_info *ts_info, char * strbuf, int strbufsize)
{
	struct capa_info *cap = &(ts_info->cap_info);

	ztx_get_fw_version(ts_info);

	snprintf(strbuf, strbufsize - 1,
		"zinitix\n"
		"%s: 0x%04X\n"
		"%s: 0x%04X\n"
		"%s: 0x%04X\n",
		"Firmware Major Version", cap->fw_version,
		"Firmware Minor Version", cap->fw_minor_version,
		"Register Data Version", cap->reg_data_version);

	return 0;
}

static ssize_t ztx_devinfo_proc_read(struct file *filp,
		char __user *buf, size_t count, loff_t *ppos)
{
	char strbuf[250] = {0};

	if (g_ztx_ts_info == NULL) {
		snprintf(strbuf, sizeof(strbuf) - 1,
			"g_ztx_ts_info == NULL\n");
		return simple_read_from_buffer(buf, count, ppos, strbuf,
			strlen(strbuf));
	}

	ztx_get_devinfo(g_ztx_ts_info, strbuf, sizeof(strbuf));

	return simple_read_from_buffer(buf, count, ppos, strbuf,
			strlen(strbuf));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops ztx_devinfo_proc_fops = {
	.proc_read = ztx_devinfo_proc_read,
	.proc_write = NULL,
};
#else
static const struct file_operations ztx_devinfo_proc_fops = {
	.read = ztx_devinfo_proc_read,
	.write = NULL,
};
#endif

s16 raw_data_min_v1[64] = {
	-200,  466, 1989, 2465, 2429, 1956,   11, -200,
	 474, 2401, 2655, 2609, 2617, 2535, 2807,   49,
	2156, 2726, 2693, 2630, 2630, 2555, 2865, 2496,
	2746, 2798, 2708, 2641, 2628, 2548, 2983, 3152,
	2401, 2657, 2634, 2581, 2633, 2553, 3054, 3067,
	1931, 2626, 2635, 2581, 2639, 2561, 2925, 2590,
	 290, 2321, 2620, 2580, 2650, 2572, 2713,   97,
	-200,  267, 1929, 2393, 2484, 2018,   11, -200,
};
s16 raw_data_max_v1[64] = {
	32767, 3217, 3537, 4382, 4319, 3478, 2192, 32767,
	 3232, 4268, 4720, 4639, 4653, 4507, 4990,  2477,
	 3833, 4846, 4789, 4676, 4676, 4543, 5094,  4437,
	 4882, 4974, 4814, 4696, 4673, 4529, 5304,  5605,
	 4269, 4724, 4683, 4589, 4682, 4539, 5430,  5453,
	 3434, 4669, 4686, 4588, 4692, 4554, 5200,  4604,
	 2906, 4127, 4658, 4587, 4711, 4573, 4824,  2561,
	32767, 2864, 3430, 4255, 4416, 3588, 2121, 32767,
};

s16 raw_data_min_v2[64] = {
	-200,  408, 1939, 2350, 2341, 1861,   16, -200,
	 444, 2323, 2540, 2504, 2494, 2426, 2725,   10,
	2093, 2604, 2565, 2519, 2502, 2440, 2745, 2429,
	2643, 2670, 2576, 2525, 2496, 2434, 2865, 3054,
	2314, 2538, 2508, 2466, 2506, 2447, 2935, 2958,
	1880, 2510, 2507, 2464, 2510, 2452, 2813, 2530,
	 268, 2236, 2492, 2459, 2516, 2461, 2635,   63,
	-200,  251, 1874, 2305, 2387, 1966,   13, -200,
};
s16 raw_data_max_v2[64] = {
	32767, 2414, 3448, 4178, 4161, 3309, 1377, 32767,
	 2479, 4129, 4517, 4452, 4434, 4313, 4844,  1708,
	 3721, 4630, 4560, 4478, 4448, 4338, 4880,  4318,
	 4699, 4747, 4580, 4488, 4439, 4328, 5093,  5430,
	 4114, 4512, 4459, 4384, 4455, 4351, 5217,  5259,
	 3342, 4463, 4457, 4381, 4463, 4359, 5001,  4499,
	 2165, 3976, 4430, 4371, 4472, 4376, 4684,  1802,
	32767, 2135, 3332, 4098, 4245, 3496, 1381, 32767,
};

static int raw_data_test(struct bt541_ts_info *ts_info)
{
	int ret = 0;
	struct bt541_ts_info *info = ts_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	s32 i;
	s32 j;
	s32 index;
	char strbuf[256] = {0};
	s32 len;
	s16 *min = NULL;
	s16 *max = NULL;

	dev_err(&ts_info->client->dev, "%s: start\n", __func__);

	if (oplus_panel_id == ZINITIX_PANEL_ID_V1) {
		min = raw_data_min_v1;
		max = raw_data_max_v1;
	} else if (oplus_panel_id == ZINITIX_PANEL_ID_V2) {
		min = raw_data_min_v2;
		max = raw_data_max_v2;
	} else {
		min = raw_data_min_v1;
		max = raw_data_max_v1;
	}

	memset(raw_data, 0, sizeof(struct tsp_raw_data));
	set_default_result(info);

	ts_set_touchmode_lite(TOUCH_PDND_MODE);
	get_raw_data_lite(info, (unsigned char *)raw_data->pref_data, 10);
	ts_set_touchmode_lite(TOUCH_POINT_MODE);

	if (g_ztx_test_result_buffer != NULL) {
		g_ztx_test_result_buffer_index += snprintf(
			&g_ztx_test_result_buffer[g_ztx_test_result_buffer_index],
			RESULT_BUFFER_SIZE - g_ztx_test_result_buffer_index - 1,
			"============ Test Item: RawData Test\n");
		for(i = 0; i < info->cap_info.x_node_num ; i ++) {
			memset(strbuf, 0, sizeof(strbuf));
			for (j = 0; j < info->cap_info.y_node_num; j ++) {
				len = strlen(strbuf);
				snprintf(&strbuf[len], sizeof(strbuf)-len,
					"%5d, ", raw_data->pref_data[i*info->cap_info.y_node_num+j]);
			}
			g_ztx_test_result_buffer_index += snprintf(
				&g_ztx_test_result_buffer[g_ztx_test_result_buffer_index],
				RESULT_BUFFER_SIZE - g_ztx_test_result_buffer_index - 1,
				"Ch/Tx_%02d:  %s\n", i, strbuf);
		}
	}

	for(i = 0; i < info->cap_info.x_node_num ; i ++) {
		for (j = 0; j < info->cap_info.y_node_num; j ++) {
			index = i*info->cap_info.y_node_num+j;
			if(((raw_data->pref_data[index]) < min[index]) ||
				((raw_data->pref_data[index]) > max[index])) {
				ret = 4;
				if (g_ztx_test_result_buffer != NULL) {
					g_ztx_test_result_buffer_index += snprintf(
						&g_ztx_test_result_buffer[g_ztx_test_result_buffer_index],
						RESULT_BUFFER_SIZE - g_ztx_test_result_buffer_index - 1,
						"test fail,node(%4d,%4d)=%5d,range=[%5d,%5d]\n",
						i, j, (raw_data->pref_data[index]), min[index], max[index]);
				}
			} else {
				continue;
			}
		}
	}
	if (g_ztx_test_result_buffer != NULL) {
		if (ret == 0) {
			g_ztx_test_result_buffer_index += snprintf(
				&g_ztx_test_result_buffer[g_ztx_test_result_buffer_index],
				RESULT_BUFFER_SIZE - g_ztx_test_result_buffer_index - 1,
				"------ RawData Test PASS\n");
		} else {
			g_ztx_test_result_buffer_index += snprintf(
				&g_ztx_test_result_buffer[g_ztx_test_result_buffer_index],
				RESULT_BUFFER_SIZE - g_ztx_test_result_buffer_index - 1,
				"------ RawData Test NG\n");
		}
	}
	dev_err(&ts_info->client->dev, "%s: end, ret = %d\n", __func__, ret);
	return ret;
}

#define TSZ_SHORT_DATA_TOTAL_NUM (CHECK_SHORT_DATA_NUM>>1)
#define TSZ_SHORT_DATA_PRINT_LEN 8

static int short_data_test(struct bt541_ts_info *ts_info)
{
	int ret = 0;
	struct bt541_ts_info *info = ts_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	s32 i;
	s32 j;
	s32 index;
	char strbuf[256] = {0};
	s32 len;

	s16 min[TSZ_SHORT_DATA_TOTAL_NUM] = {
		1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
		1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
		1000,
	};
	s16 max[TSZ_SHORT_DATA_TOTAL_NUM] = {
		1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
		1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
		1000,
	};

	dev_err(&ts_info->client->dev, "%s: start\n", __func__);

	memset(raw_data, 0, sizeof(struct tsp_raw_data));
	set_default_result(info);

	ts_set_touchmode_lite(TOUCH_CHECK_SHORT_MODE);
	get_short_data_lite(info, (u8 *)raw_data->pref_data, 10);
	ts_set_touchmode_lite(TOUCH_POINT_MODE);

	if (g_ztx_test_result_buffer != NULL) {
		g_ztx_test_result_buffer_index += snprintf(
			&g_ztx_test_result_buffer[g_ztx_test_result_buffer_index],
			RESULT_BUFFER_SIZE - g_ztx_test_result_buffer_index - 1,
			"============ Test Item: Short Circuit Test\n");
		for(i = 0; i <= TSZ_SHORT_DATA_TOTAL_NUM / TSZ_SHORT_DATA_PRINT_LEN ; i ++) {
			memset(strbuf, 0, sizeof(strbuf));
			for (j = 0; j < TSZ_SHORT_DATA_PRINT_LEN; j ++) {
				index = i * TSZ_SHORT_DATA_PRINT_LEN + j;
				if (index >= TSZ_SHORT_DATA_TOTAL_NUM) {
					break;
				}
				len = strlen(strbuf);
				snprintf(&strbuf[len], sizeof(strbuf)-len,
					"%5d, ", raw_data->pref_data[i*info->cap_info.y_node_num+j]);
			}
			g_ztx_test_result_buffer_index += snprintf(
				&g_ztx_test_result_buffer[g_ztx_test_result_buffer_index],
				RESULT_BUFFER_SIZE - g_ztx_test_result_buffer_index - 1,
				"Ch/Tx_%02d:  %s\n", i, strbuf);
		}
	}

	for(index = 0; index < TSZ_SHORT_DATA_TOTAL_NUM; index++) {
		if(((raw_data->pref_data[index]) < min[index]) ||
			((raw_data->pref_data[index]) > max[index])) {
			ret = 2;
			if (g_ztx_test_result_buffer != NULL) {
				g_ztx_test_result_buffer_index += snprintf(
					&g_ztx_test_result_buffer[g_ztx_test_result_buffer_index],
					RESULT_BUFFER_SIZE - g_ztx_test_result_buffer_index - 1,
					"test fail,node(%4d)=%5d,range=[%5d,%5d]\n",
					index, (raw_data->pref_data[index]), min[index], max[index]);
			}
		}
	}

	if (g_ztx_test_result_buffer != NULL) {
		if (ret == 0) {
			g_ztx_test_result_buffer_index += snprintf(
				&g_ztx_test_result_buffer[g_ztx_test_result_buffer_index],
				RESULT_BUFFER_SIZE - g_ztx_test_result_buffer_index - 1,
				"------ Short Circuit Test PASS\n");
		} else {
			g_ztx_test_result_buffer_index += snprintf(
				&g_ztx_test_result_buffer[g_ztx_test_result_buffer_index],
				RESULT_BUFFER_SIZE - g_ztx_test_result_buffer_index - 1,
				"------ Short Circuit Test NG\n");
		}
	}
	dev_err(&ts_info->client->dev, "%s: end, ret = %d\n", __func__, ret);
	return ret;
}

static void ztx_test_result_devinfo(struct bt541_ts_info *ts_info)
{
	if (g_ztx_test_result_buffer != NULL) {
		ztx_get_devinfo(ts_info,
			&g_ztx_test_result_buffer[g_ztx_test_result_buffer_index],
			RESULT_BUFFER_SIZE - g_ztx_test_result_buffer_index);
		g_ztx_test_result_buffer_index = strlen(g_ztx_test_result_buffer);
	}
}

static void ztx_test_result_empty_line(void)
{
	if (g_ztx_test_result_buffer != NULL) {
		g_ztx_test_result_buffer_index += snprintf(
			&g_ztx_test_result_buffer[g_ztx_test_result_buffer_index],
			RESULT_BUFFER_SIZE - g_ztx_test_result_buffer_index - 1,
			"\n");

	}
}

static int ztx_autotest(struct bt541_ts_info *ts_info)
{
	int ret = 0; //0:success 1:fail

	dev_err(&ts_info->client->dev, "%s: start\n", __func__);

	//clear report buffer
	if (g_ztx_test_result_buffer == NULL) {
		g_ztx_test_result_buffer = vmalloc(RESULT_BUFFER_SIZE);
	}
	if (g_ztx_test_result_buffer != NULL) {
		memset(g_ztx_test_result_buffer, 0, RESULT_BUFFER_SIZE);
		g_ztx_test_result_buffer_index = 0;
	}

	down(&ts_info->work_lock);
#ifdef ZTX_ESD_ENABLE
	bt541_esd_timer_stop(ts_info);
#endif /* ZTX_ESD_ENABLE */
	disable_irq(misc_info->irq);

	ztx_test_result_devinfo(ts_info);
	ztx_test_result_empty_line();
	ret |= short_data_test(ts_info);
	ztx_test_result_empty_line();
	ret |= raw_data_test(ts_info);
	ztx_test_result_empty_line();

	resume_hw_reset(ts_info,true);
	if (!bt541_power_sequence(ts_info)) {
		bt541_power_control(ts_info, POWER_OFF);
		bt541_power_control(ts_info, POWER_ON_SEQUENCE);
	}
	enable_irq(misc_info->irq);
	up(&ts_info->work_lock);

	if (g_ztx_test_result_buffer != NULL) {
		if (ret == 0) {
			g_ztx_test_result_buffer_index += snprintf(
				&g_ztx_test_result_buffer[g_ztx_test_result_buffer_index],
				RESULT_BUFFER_SIZE - g_ztx_test_result_buffer_index - 1,
				"=======Tp test pass.\n");
		} else {
			g_ztx_test_result_buffer_index += snprintf(
				&g_ztx_test_result_buffer[g_ztx_test_result_buffer_index],
				RESULT_BUFFER_SIZE - g_ztx_test_result_buffer_index - 1,
				"=======Tp test failure.\n");
		}
	}
	dev_err(&ts_info->client->dev, "%s: end, ret = %d\n", __func__, ret);
	return ret;
}

static int ztx_baseline_test_proc_open(struct inode *inode, struct file *file)
{
	if (g_ztx_ts_info == NULL) {
		g_ztx_autotest_result = -1;
		return 0;
	}

	g_ztx_autotest_result = ztx_autotest(g_ztx_ts_info);

	return 0;
}

static ssize_t ztx_baseline_test_proc_read(struct file *filp,
		char __user *buf, size_t count, loff_t *ppos)
{
	char strbuf[250] = {0};

	if (g_ztx_autotest_result == 0) {
		snprintf(strbuf, sizeof(strbuf) - 1, "Status 1\n");
	} else {
		snprintf(strbuf, sizeof(strbuf) - 1, "Error:%d\n", g_ztx_autotest_result);
	}

	return simple_read_from_buffer(buf, count, ppos, strbuf,
			strlen(strbuf));
}


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops ztx_baseline_test_proc_fops = {
	.proc_open = ztx_baseline_test_proc_open,
	.proc_read = ztx_baseline_test_proc_read,
	.proc_write = NULL,
};
#else
static const struct file_operations ztx_baseline_test_proc_fops = {
	.open = ztx_baseline_test_proc_open,
	.read = ztx_baseline_test_proc_read,
	.write = NULL,
};
#endif

static ssize_t ztx_baseline_result_proc_read(struct file *filp,
		char __user *buf, size_t count, loff_t *ppos)
{
	char strbuf[250] = {0};

	if (g_ztx_ts_info == NULL) {
		snprintf(strbuf, sizeof(strbuf) - 1,
			"g_ztx_ts_info == NULL\n");
		return simple_read_from_buffer(buf, count, ppos, strbuf,
			strlen(strbuf));
	}

	if (g_ztx_test_result_buffer == NULL) {
		snprintf(strbuf, sizeof(strbuf) - 1,
			"g_ztx_test_result_buffer == NULL\n");
		return simple_read_from_buffer(buf, count, ppos, strbuf,
			strlen(strbuf));
	}

	return simple_read_from_buffer(buf, count, ppos,
		g_ztx_test_result_buffer, strlen(g_ztx_test_result_buffer));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops ztx_baseline_result_proc_fops = {
	.proc_read = ztx_baseline_result_proc_read,
	.proc_write = NULL,
};
#else
static const struct file_operations ztx_baseline_result_proc_fops = {
	.read = ztx_baseline_result_proc_read,
	.write = NULL,
};
#endif

static ssize_t ztx_debug_level_proc_read(struct file *filp,
		char __user *buf, size_t count, loff_t *ppos)
{
	char strbuf[250] = {0};

	if (g_ztx_ts_info == NULL) {
		snprintf(strbuf, sizeof(strbuf) - 1, "0\n");
	} else {
		snprintf(strbuf, sizeof(strbuf) - 1, "%d\n", g_ztx_ts_info->debug_level);
	}

	return simple_read_from_buffer(buf, count, ppos, strbuf,
		strlen(strbuf));
}

static ssize_t ztx_debug_level_proc_write(struct file *file,
		const char __user *buf, size_t count, loff_t *off)
{
	int value = 0;
	int ret = 0;
	char strbuf[10] = {0};

	if (g_ztx_ts_info == NULL) {
		return -ENXIO;
	}

	if (count > sizeof(strbuf) - 1) {
		return -EINVAL;
	}

	if (copy_from_user(strbuf, buf, sizeof(strbuf))) {
		return -EFAULT;
	}

	strbuf[count] = '\0';

	ret = kstrtoint(strbuf, 10, &value);
	if (ret < 0) {
		return ret;
	}

	if (value < ZTX_DEBUG_LOG_MIN) {
		value = ZTX_DEBUG_LOG_MIN;
	} else if (value > ZTX_DEBUG_LOG_MAX) {
		value = ZTX_DEBUG_LOG_MAX;
	}

	g_ztx_ts_info->debug_level = value;
	dev_info(&g_ztx_ts_info->client->dev, "%s: set debug level to %d\n",
		__func__, g_ztx_ts_info->debug_level);

	return count;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops ztx_debug_level_proc_fops = {
	.proc_read = ztx_debug_level_proc_read,
	.proc_write = ztx_debug_level_proc_write,
};
#else
static const struct file_operations ztx_debug_level_proc_fops = {
	.read = ztx_debug_level_proc_read,
	.write = ztx_debug_level_proc_write,
};
#endif

static ssize_t ztx_check_connection_proc_read(struct file *filp,
		char __user *buf, size_t count, loff_t *ppos)
{
	char strbuf[50] = {0};
	u16	esd_value = 0x0;

	if (g_ztx_ts_info == NULL) {
		snprintf(strbuf, sizeof(strbuf) - 1, "0\n");
	} else {
		if (read_data(g_ztx_ts_info->client, ZTX_ESD_REG_ADDR, (u8 *)&esd_value, 2) < 0) {
			esd_value = 0xFFFF;
		}
		if (esd_value == ZTX_ESD_REG_VALUE) {
			snprintf(strbuf, sizeof(strbuf) - 1, "0\n");
		} else {
			snprintf(strbuf, sizeof(strbuf) - 1, "1\n");
		}
	}

	return simple_read_from_buffer(buf, count, ppos, strbuf,
		strlen(strbuf));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops ztx_check_connection_proc_fops = {
	.proc_read = ztx_check_connection_proc_read,
	.proc_write = NULL,
};
#else
static const struct file_operations ztx_check_connection_proc_fops = {
	.read = ztx_check_connection_proc_read,
	.write = NULL,
};
#endif

static ssize_t ztx_gesture_mode_en_proc_read(struct file *filp,
		char __user *buf, size_t count, loff_t *ppos)
{
	char strbuf[250] = {0};

	if (g_ztx_ts_info == NULL) {
		snprintf(strbuf, sizeof(strbuf) - 1, "0\n");
	} else {
		if (g_ztx_ts_info->gesture_mode_en) {
			snprintf(strbuf, sizeof(strbuf) - 1, "1\n");
		} else {
			snprintf(strbuf, sizeof(strbuf) - 1, "0\n");
		}
	}

	return simple_read_from_buffer(buf, count, ppos, strbuf,
		strlen(strbuf));
}

static ssize_t ztx_gesture_mode_en_proc_write(struct file *file,
		const char __user *buf, size_t count, loff_t *off)
{
	int value = 0;
	int ret = 0;
	char strbuf[10] = {0};

	if (g_ztx_ts_info == NULL) {
		return -ENXIO;
	}

	if (count > sizeof(strbuf) - 1) {
		return -EINVAL;
	}

	if (copy_from_user(strbuf, buf, sizeof(strbuf))) {
		return -EFAULT;
	}

	strbuf[count] = '\0';

	ret = kstrtoint(strbuf, 10, &value);
	if (ret < 0) {
		value = -1;
		dev_err(&g_ztx_ts_info->client->dev,
			"%s: read new gesture_mode_en value failed, ret = %d\n",
			__func__, ret);
	}

	if (value == 0) {
		g_ztx_ts_info->gesture_mode_en = false;
	} else if (value == 1) {
		g_ztx_ts_info->gesture_mode_en = true;
	} else {
		dev_err(&g_ztx_ts_info->client->dev, "%s: unknown gesture_mode_en value %d\n",
			__func__, value);
	}

	dev_info(&g_ztx_ts_info->client->dev, "%s: set gesture_mode_en to %d\n",
		__func__, g_ztx_ts_info->gesture_mode_en);

	return count;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops ztx_gesture_mode_en_proc_fops = {
	.proc_read = ztx_gesture_mode_en_proc_read,
	.proc_write = ztx_gesture_mode_en_proc_write,
};
#else
static const struct file_operations ztx_gesture_mode_en_proc_fops = {
	.read = ztx_gesture_mode_en_proc_read,
	.write = ztx_gesture_mode_en_proc_write,
};
#endif

static int init_proc_group(struct bt541_ts_info *info) {
	int ret = 0;
	info->proc_entry = proc_mkdir("touchpanel", NULL);
	if (IS_ERR_OR_NULL(info->proc_entry)) {
		dev_err(&info->client->dev, "%s: Error, could not create base proc directory\n",
				__func__);
		goto unregister_base_dir;
	}

	if (IS_ERR_OR_NULL(proc_create("devinfo", S_IRUGO,
		info->proc_entry, &ztx_devinfo_proc_fops))) {
		dev_err(&info->client->dev, "%s: Error, could not create proc %s\n",
				__func__, "devinfo");
		goto unregister_base_dir;
	}

	if (IS_ERR_OR_NULL(proc_create("baseline_test", S_IRUGO,
		info->proc_entry, &ztx_baseline_test_proc_fops))) {
		dev_err(&info->client->dev, "%s: Error, could not create proc %s\n",
				__func__, "baseline_result");
		goto unregister_base_dir;
	}

	if (IS_ERR_OR_NULL(proc_create("baseline_result", S_IRUGO,
		info->proc_entry, &ztx_baseline_result_proc_fops))) {
		dev_err(&info->client->dev, "%s: Error, could not create proc %s\n",
				__func__, "baseline_result");
		goto unregister_base_dir;
	}

	if (IS_ERR_OR_NULL(proc_create("debug_level", S_IRUGO | S_IWUGO,
		info->proc_entry, &ztx_debug_level_proc_fops))) {
		dev_err(&info->client->dev, "%s: Error, could not create proc %s\n",
				__func__, "debug_level");
		goto unregister_base_dir;
	}

	if (IS_ERR_OR_NULL(proc_create("check_connection", S_IRUGO,
		info->proc_entry, &ztx_check_connection_proc_fops))) {
		dev_err(&info->client->dev, "%s: Error, could not create proc %s\n",
				__func__, "check_connection");
		goto unregister_base_dir;
	}

	if (IS_ERR_OR_NULL(proc_create("gesture_mode_en", S_IRUGO | S_IWUGO,
		info->proc_entry, &ztx_gesture_mode_en_proc_fops))) {
		dev_err(&info->client->dev, "%s: Error, could not create proc %s\n",
				__func__, "gesture_mode_en");
		goto unregister_base_dir;
	}

	g_ztx_ts_info = info;
	return ret;

unregister_base_dir:
	remove_proc_entry("touchpanel", NULL);
	info->proc_entry = NULL;
	g_ztx_ts_info = NULL;
	return ret;
}

#endif
/*End of adb operation*/
#endif

static int ts_misc_fops_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int ts_misc_fops_close(struct inode *inode, struct file *filp)
{
	return 0;
}

static bool ztx_need_bus_pull()
{
	if (get_project() == 23912 && get_oplus_board_id() <= 1) {
		return true;
	}

	if (get_project() == 23902 && get_oplus_board_id() <= 1) {
		return true;
	}

	if (get_project() == 23911 && get_oplus_board_id() <= 1) {
		return true;
	}

	return false;
}

static int zinitix_power_init(struct bt541_ts_info *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	if (ztx_need_bus_pull()) {
		dev_info(&data->client->dev, "need bus pull\n");
		data->bus_pull = regulator_get(&data->client->dev, "bus_pull");
		if (IS_ERR(data->bus_pull)) {
			rc = PTR_ERR(data->bus_pull);
			dev_err(&data->client->dev,
				"Regulator get failed bus_pull rc=%d\n", rc);
		} else {
			rc = regulator_enable(data->bus_pull);
			if (rc) {
				dev_err(&data->client->dev,
					"Regulator bus_pull enable failed rc=%d\n", rc);
			}
		}
	}

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
					   FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int zinitix_ts_pinctrl_init(struct bt541_ts_info *bt541_data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	bt541_data->ts_pinctrl = devm_pinctrl_get(&(bt541_data->client->dev));
	if (IS_ERR_OR_NULL(bt541_data->ts_pinctrl)) {
		retval = PTR_ERR(bt541_data->ts_pinctrl);
		dev_dbg(&bt541_data->client->dev,
			"Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	bt541_data->pinctrl_state_active
		= pinctrl_lookup_state(bt541_data->ts_pinctrl,
				PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(bt541_data->pinctrl_state_active)) {
		retval = PTR_ERR(bt541_data->pinctrl_state_active);
		dev_err(&bt541_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	bt541_data->pinctrl_state_suspend
		= pinctrl_lookup_state(bt541_data->ts_pinctrl,
			PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(bt541_data->pinctrl_state_suspend)) {
		retval = PTR_ERR(bt541_data->pinctrl_state_suspend);
		dev_err(&bt541_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	bt541_data->pinctrl_state_release
		= pinctrl_lookup_state(bt541_data->ts_pinctrl,
			PINCTRL_STATE_RELEASE);
	if (IS_ERR_OR_NULL(bt541_data->pinctrl_state_release)) {
		retval = PTR_ERR(bt541_data->pinctrl_state_release);
		dev_dbg(&bt541_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_RELEASE, retval);
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(bt541_data->ts_pinctrl);
err_pinctrl_get:
	bt541_data->ts_pinctrl = NULL;
	return retval;
}



#ifdef CONFIG_OF
static const struct of_device_id zinitix_match_table[] = {
	{ .compatible = "zinitix,bt541_ts_device",},
	{},
};
static int zinitix_hw_reset( struct bt541_ts_info* data,bool on )
{
	int err = 0;
	printk("zinitix_hw_reset: on = %d, gpio_reset = %d\n", on, data->pdata->gpio_reset);
	if(on)
	{
		if (gpio_is_valid(data->pdata->gpio_reset)) {
		/*
		* This is intended to save leakage current
		* only. Even if the call(gpio_direction_input)
		* fails, only leakage current will be more but
		* functionality will not be affected.
		*/


		gpio_direction_output(data->pdata->gpio_reset, 1);
		msleep(20);
		gpio_direction_output(data->pdata->gpio_reset, 0);
		msleep(100)/*eric modify 20170109*/;
		gpio_direction_output(data->pdata->gpio_reset, 1);
		//msleep(10);
		//err = gpio_direction_input(data->pdata->gpio_reset);
		if (err) {
		dev_err(&data->client->dev,
			"unable to set direction for gpio "
			"[%d]\n", data->pdata->gpio_reset);
		}
		//gpio_free(data->pdata->gpio_reset);
		}
	}

	return err;
}

static int resume_hw_reset( struct bt541_ts_info* data,bool on )
{
	int err = 0;
	printk("Resume_reset: on = %d, gpio_reset = %d\n", on, data->pdata->gpio_reset);
	if(on)
	{
		if (gpio_is_valid(data->pdata->gpio_reset)) {
		/*
		* This is intended to save leakage current
		* only. Even if the call(gpio_direction_input)
		* fails, only leakage current will be more but
		* functionality will not be affected.
		*/


		gpio_direction_output(data->pdata->gpio_reset, 1);
		msleep(10);
		gpio_direction_output(data->pdata->gpio_reset, 0);
		msleep(100)/*eric modify 20170109*/;
		gpio_direction_output(data->pdata->gpio_reset, 1);
		msleep(20);
		//err = gpio_direction_input(data->pdata->gpio_reset);
		if (err) {
		dev_err(&data->client->dev,
			"unable to set direction for gpio "
			"[%d]\n", data->pdata->gpio_reset);
		}
		//gpio_free(data->pdata->gpio_reset);
		}
	}

	return err;
}


static int zinitix_init_gpio(struct bt541_ts_info* data,bool on )

{
	int err = 0;

	if (on) {
		if (gpio_is_valid(data->pdata->gpio_int)) {
			err = gpio_request(data->pdata->gpio_int,
						"bt541_irq_gpio");
			if (err) {
				dev_err(&data->client->dev,
					"irq gpio request failed");
				zinitix_printk("irq gpio request failed is %d\n",err);
				goto err_irq_gpio_req;
			}

			err = gpio_direction_input(data->pdata->gpio_int);
			if (err) {
				dev_err(&data->client->dev,
					"set_direction for irq gpio failed\n");
				goto err_irq_gpio_req;
			}
		}

		return 0;

	}
	else {
		if (gpio_is_valid(data->pdata->gpio_int))
			gpio_free(data->pdata->gpio_int);


		return 0;
	}
err_irq_gpio_req:
	if (gpio_is_valid(data->pdata->gpio_reset))
		gpio_free(data->pdata->gpio_reset);

	return err;
}



#endif

static int bt541_ts_probe_dt(struct device_node *np,
			 struct device *dev,
			 struct bt541_ts_platform_data *pdata)
{

	int ret = 0;
	u32 temp;


	ret = of_property_read_u32(np, "zinitix,x_resolution", &temp);
	if (ret) {
			dev_err(dev, "Unable to read controller version zinitix,x_resolution\n");
			return ret;
	} else
			pdata->x_resolution = (u16) temp;
	ret = of_property_read_u32(np, "zinitix,y_resolution", &temp);
	if (ret) {
			dev_err(dev, "Unable to read controller version zinitix,y_resolution\n");
			return ret;
	} else
			pdata->y_resolution = (u16) temp;
	ret = of_property_read_u32(np, "zinitix,page_size", &temp);
	if (ret) {
			dev_err(dev, "Unable to read controller version zinitix,page_size\n");
			return ret;
	} else
			pdata->page_size = (u16) temp;
	ret = of_property_read_u32(np, "zinitix,orientation", &temp);
	if (ret) {
			dev_err(dev, "Unable to read controller version zinitix,orientation\n");
			return ret;
	} else
			pdata->orientation = (u8) temp;

	pdata->tsp_vendor1 = of_get_named_gpio(np, "zinitix,vendor1", 0);
	pdata->tsp_vendor2 = of_get_named_gpio(np, "zinitix,vendor2", 0);


	pdata->gpio_reset = of_get_named_gpio(np, "zinitix,reset-gpio", 0);
	if (pdata->gpio_reset < 0) {
		pr_err("%s: of_get_named_gpio failed: tsp_gpio %d\n", __func__,
			pdata->gpio_reset);
		return -EINVAL;
	}

	pdata->gpio_int = of_get_named_gpio(np, "zinitix,irq-gpio", 0);
	if (pdata->gpio_int < 0) {
		pr_err("%s: of_get_named_gpio failed: tsp_gpio %d\n", __func__,
			pdata->gpio_int);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "zinitix,tsp_vdd_supply_type",
				&pdata->tsp_supply_type);
	if (ret < 0) {
		pr_err("%s: failed to read property tsp_vdd_supply_type\n",
			__func__);
		return ret;
	}

	ret = of_property_read_string(np, "zinitix,pname",
				&pdata->pname);
	if (ret < 0) {
		pr_err("%s: failed to read property config_ver\n",
			__func__);
		return ret;
	}

	if (pdata->tsp_supply_type == TSP_LDO_SUPPLY) {
		pdata->tsp_en_gpio = of_get_named_gpio(np, "zinitix,vdd_en-gpio", 0);
		if (pdata->tsp_en_gpio < 0) {
			pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
				pdata->tsp_en_gpio);
			return -EINVAL;
		}
	}

	pr_err("%s: x_resolution :%d, y_resolution :%d, page_size :%d, orientation :%d, gpio_int :%d, vcc_en :%d\n",
		__func__, pdata->x_resolution, pdata->y_resolution, pdata->page_size,
		pdata->orientation, pdata->gpio_int, pdata->tsp_en_gpio);


	return 0;

}

#ifdef USE_TSP_TA_CALLBACKS
void bt541_register_callback(struct tsp_callbacks *cb)
{
	charger_callbacks = cb;
	pr_info("%s\n", __func__);
}
#endif

static int bt541_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *i2c_id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct bt541_ts_platform_data *pdata = NULL;
	struct bt541_ts_info *info;
	struct input_dev *input_dev;
	int ret = 0;


	struct device_node *np = client->dev.of_node;
	zinitix_printk("[TSP]: %s\n", __func__);


	if (client->dev.of_node) {
		if (!pdata) {
			pdata = devm_kzalloc(&client->dev,
					sizeof(*pdata), GFP_KERNEL);
			if (!pdata)
				return -ENOMEM;
		}
		ret = bt541_ts_probe_dt(np, &client->dev, pdata);
		if (ret){
			dev_err(&client->dev, "Error parsing dt %d\n", ret);
			goto err_no_platform_data;
		}

#ifdef USE_TSP_TA_CALLBACKS
		pdata->register_cb = bt541_register_callback;
#endif



	} else if (!pdata) {
		dev_err(&client->dev, "%s: no platform data defined\n",
			__func__);
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "Not compatible i2c function\n");
		ret = -EIO;
		goto err_no_platform_data;
	}

	info = kzalloc(sizeof(struct bt541_ts_info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "%s: Failed to allocate memory\n", __func__);
		ret = -ENOMEM;
		goto err_mem_alloc;
	}

	i2c_set_clientdata(client, info);
	info->client = client;
	info->pdata = pdata;
	info->device_enabled = 1;
	info->aod_mode = false;
	info->debug_level = 0;
	info->power_off_mode = false;
	info->gesture_mode_en = true;
#ifdef CONFIG_PM
	info->pm_suspend = false;
#endif /* CONFIG_PM */
	info->ts_workqueue = create_singlethread_workqueue("zinitix_wq");
	if (!info->ts_workqueue) {
		dev_err(&client->dev, "%s: create zinitix_wq workqueue fail\n", __func__);
	} else {
		INIT_WORK(&info->resume_work, ztx_resume_work);
#if TOUCH_ONESHOT_UPGRADE
		INIT_WORK(&info->fwupg_work, ztx_upgrade_firmware_work);
#endif
#ifdef ZTX_ESD_ENABLE
		INIT_DELAYED_WORK(&info->esd_test_work, bt541_esd_timer_handler);
#endif /* ZTX_ESD_ENABLE */
	}
	misc_touch_dev = info;





	ret = zinitix_ts_pinctrl_init(info);
	if (!ret && info->ts_pinctrl) {
		/*
		 * Pinctrl handle is optional. If pinctrl handle is found
		 * let pins to be configured in active state. If not
		 * found continue further without error.
		 */
		ret = pinctrl_select_state(info->ts_pinctrl,
					info->pinctrl_state_active);
		if (ret < 0) {
			dev_err(&client->dev,
				"failed to select pin to active state");
		}
	}

	ret = gpio_request(info->pdata->gpio_reset,
				"bt541_reset_gpio");
	if (ret) {
		dev_err(&info->client->dev,
			"reset gpio request failed");
		zinitix_printk("reset gpio request failed is %d\n",ret);
		}

	ret = zinitix_init_gpio(info,true);
		if(ret < 0)
			goto err_gpio_request;
	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "Failed to allocate input device\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

#ifdef USE_TSP_TA_CALLBACKS
	info->register_cb = info->pdata->register_cb;
#endif

	info->input_dev = input_dev;
	info->work_state = PROBE;

	ret = zinitix_power_init(info,true);
	if (ret) {
			dev_err(&client->dev, "power init failed");
			goto err_input_register_device;
	}

	/* power on sequence */
	if (bt541_power_control(info, POWER_ON_SEQUENCE) == false) {
		ret = -EPERM;
		goto err_power_sequence;
	}

	/* To Do */
	/* FW version read from tsp */
	memset(&info->reported_touch_info,
		0x0, sizeof(struct point_info));

	memset(&info->filter_data,
		0x0, sizeof(info->filter_data));

	/* init touch mode */
	info->touch_mode = TOUCH_POINT_MODE;
	misc_info = info;
	info->checkUMSmode = false;
	info->firmware_data = NULL;
#ifdef ZTX_ESD_ENABLE
	info->esd_enable = false;
#endif /* ZTX_ESD_ENABLE */

	if (init_touch(info) == false) {
		ret = -EPERM;
		goto err_touch_init;
	}

#ifdef USE_TSP_TA_CALLBACKS
	info->callbacks.inform_charger = bt541_charger_status_cb;
		if (info->register_cb)
			info->register_cb(&info->callbacks);
#endif

	snprintf(info->phys, sizeof(info->phys),
		"%s/input0", dev_name(&client->dev));
	input_dev->name = "sec_touchscreen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->phys = info->phys;
	input_dev->dev.parent = &client->dev;

	set_bit(EV_SYN, info->input_dev->evbit);
	set_bit(EV_KEY, info->input_dev->evbit);
	set_bit(EV_ABS, info->input_dev->evbit);
	set_bit(BTN_TOUCH, info->input_dev->keybit);

#if USE_WAKEUP_GESTURE
	input_set_capability(info->input_dev, EV_KEY, KEY_F12);
#endif

#if SUPPORTED_PALM_TOUCH
	//set_bit(KEY_POWER, info->input_dev->evbit);
	input_set_capability(info->input_dev, EV_KEY, KEY_SLEEP);
	//dev_err(&client->dev,"KEY_SLEEP register finish\n");
#endif

	set_bit(EV_LED, info->input_dev->evbit);
	set_bit(LED_MISC, info->input_dev->ledbit);

	set_bit(INPUT_PROP_DIRECT, info->input_dev->propbit);



	if (pdata->orientation & TOUCH_XY_SWAP) {
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y,
			info->cap_info.MinX,
			info->cap_info.MaxX + ABS_PT_OFFSET,
			0, 0);
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_X,
			info->cap_info.MinY,
			info->cap_info.MaxY + ABS_PT_OFFSET,
			0, 0);
	} else {
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_X,
			info->cap_info.MinX,
			info->cap_info.MaxX + ABS_PT_OFFSET,
			0, 0);
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y,
			info->cap_info.MinY,
			info->cap_info.MaxY + ABS_PT_OFFSET,
			0, 0);
	}

	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MAJOR,
		0, 255, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_WIDTH_MAJOR,
		0, 255, 0, 0);

#if (TOUCH_POINT_MODE == 2)
	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MINOR,
		0, 255, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_PALM,
		0, 1, 0, 0);
#endif

	set_bit(MT_TOOL_FINGER, info->input_dev->keybit);

	input_mt_init_slots(info->input_dev, info->cap_info.multi_fingers, INPUT_MT_DIRECT);

	zinitix_debug_msg("register %s input device \r\n",
		info->input_dev->name);
	input_set_drvdata(info->input_dev, info);
	ret = input_register_device(info->input_dev);
	if (ret) {
		pr_err("unable to register %s input device\r\n",
			info->input_dev->name);
		goto err_touch_init;
	}

	info->work_state = NOTHING;
	sema_init(&info->work_lock, 1);
	mutex_init(&info->read_reg_mutex);

	/* configure irq */
	info->irq = gpio_to_irq(pdata->gpio_int);
	if (info->irq < 0){
		dev_err(&client->dev, "Invalid GPIO_TOUCH_IRQ\n");
		ret = -ENODEV;
		goto err_irq_of_parse;
	}

	pdata->tsp_irq = info->irq;
	ret = request_threaded_irq(info->irq, NULL, bt541_touch_work,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT , BT541_TS_DEVICE, info);

	if (ret) {
		printk(KERN_ERR "unable to register irq.(%s)\r\n",
			info->input_dev->name);
		goto err_request_irq;
	}
#if defined(CONFIG_FB)
	info->fb_notif.notifier_call = fb_notifier_callback;
	ret = msm_drm_register_client(&info->fb_notif);
	if (ret) {
		dev_err(&info->client->dev,
			"Unable to register fb_notifier: %d\n",
			ret);
		msm_drm_unregister_client(&info->fb_notif);
	}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	info->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	info->early_suspend.suspend = zinitix_early_suspend;
	info->early_suspend.resume = zinitix_late_resume;
	register_early_suspend(&info->early_suspend);
#endif

	dev_info(&client->dev, "zinitix touch probe.\r\n");

#if defined(CONFIG_PM_RUNTIME)
	//pm_runtime_enable(&client->dev);
#endif

	sema_init(&info->raw_data_lock, 1);

	ret = misc_register(&touch_misc_device);
	if (ret) {
		dev_err(&client->dev, "Failed to register touch misc device\n");
		goto err_misc_register;
	}

#ifdef SEC_FACTORY_TEST
	ret = init_sec_factory(info);
	if (ret) {
		dev_err(&client->dev, "Failed to init sec factory device\n");

		goto err_kthread_create_failed;
	}

	init_proc_group(info);
#endif

#if USE_WAKEUP_GESTURE/*eric add 20170208*/

	enable_irq_wake(misc_info->irq);
#endif

#if TOUCH_ONESHOT_UPGRADE
	if (info->ts_workqueue) {
		queue_work(info->ts_workqueue, &info->fwupg_work);
	}
#endif

	dev_info(&client->dev, "bt541_ts_probe: SUCCESS\n");

	return ret;

#ifdef SEC_FACTORY_TEST
err_kthread_create_failed:
	kfree(info->factory_info);
	kfree(info->raw_data);
#endif
err_misc_register:
	free_irq(info->irq, info);
err_irq_of_parse:
err_request_irq:
//err_input_register_device2:

	input_unregister_device(info->input_dev);
err_touch_init:
	bt541_power_control(info, POWER_OFF);
err_power_sequence:
	zinitix_power_init(info, false);
err_input_register_device:
	input_free_device(info->input_dev);
err_alloc:
	if (gpio_is_valid(pdata->gpio_reset))
		gpio_free(pdata->gpio_reset);
	if (gpio_is_valid(pdata->gpio_int))
		gpio_free(pdata->gpio_int);
err_gpio_request:

	if (info->ts_pinctrl) {
		if (IS_ERR_OR_NULL(info->pinctrl_state_release)) {
			devm_pinctrl_put(info->ts_pinctrl);
			info->ts_pinctrl = NULL;
		} else {
			ret = pinctrl_select_state(info->ts_pinctrl,
					info->pinctrl_state_release);
			if (ret)
				pr_err("failed to select relase pinctrl state\n");
		}
	}
	kfree(info);
err_mem_alloc:
err_no_platform_data:
	if (IS_ENABLED(CONFIG_OF))
		devm_kfree(&client->dev, (void *)pdata);

	dev_info(&client->dev, "Failed to probe\n");
	return ret;
}

static int bt541_ts_remove(struct i2c_client *client)
{
	int err = 0;
	struct bt541_ts_info *info = i2c_get_clientdata(client);
	struct bt541_ts_platform_data *pdata = info->pdata;

	disable_irq(info->irq);
	down(&info->work_lock);

	info->work_state = REMOVE;

#ifdef SEC_FACTORY_TEST
	kfree(info->factory_info);
	kfree(info->raw_data);
#endif

#ifdef ZTX_ESD_ENABLE
	bt541_esd_timer_stop(info);
#endif /* ZTX_ESD_ENABLE */

	if (info->ts_workqueue) {
		cancel_work_sync(&info->resume_work);
		destroy_workqueue(info->ts_workqueue);
	}

	if (info->irq)
		free_irq(info->irq, info);

	misc_deregister(&touch_misc_device);

	bt541_power_control(info, POWER_OFF);
	zinitix_power_init(info, false);

	if (gpio_is_valid(pdata->gpio_reset))
		gpio_free(pdata->gpio_reset);
	if (gpio_is_valid(pdata->gpio_int))
		gpio_free(pdata->gpio_int);

	if (info->ts_pinctrl) {
		if (IS_ERR_OR_NULL(info->pinctrl_state_release)) {
			devm_pinctrl_put(info->ts_pinctrl);
			info->ts_pinctrl = NULL;
		} else {
			err = pinctrl_select_state(info->ts_pinctrl,
					info->pinctrl_state_release);
			if (err)
				pr_err("failed to select relase pinctrl state\n");
		}
	}


	input_unregister_device(info->input_dev);
	input_free_device(info->input_dev);
	up(&info->work_lock);
	mutex_destroy(&info->read_reg_mutex);
	kfree(info);

	return 0;
}

#if 0
void bt541_ts_shutdown(struct i2c_client *client)
{
	struct bt541_ts_info *info = i2c_get_clientdata(client);

	dev_info(&client->dev, "%s++\n", __func__);
	disable_irq(info->irq);
	down(&info->work_lock);
#ifdef ZTX_ESD_ENABLE
	bt541_esd_timer_stop(info);
#endif /* ZTX_ESD_ENABLE */
	up(&info->work_lock);
	bt541_power_control(info, POWER_OFF);
	dev_info(&client->dev, "%s--\n", __func__);
}
#endif

static struct i2c_device_id bt541_idtable[] = {
	{BT541_TS_DEVICE, 0},
	{ }
};

#ifdef CONFIG_PM
static int ztx_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bt541_ts_info *info = i2c_get_clientdata(client);

	dev_info(&client->dev, "system enters into pm_suspend\n");
	info->pm_suspend = true;

	return 0;
}

static int ztx_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bt541_ts_info *info = i2c_get_clientdata(client);

	dev_info(&client->dev, "system resumes from pm_suspend\n");
	info->pm_suspend = false;

	return 0;
}

static const struct dev_pm_ops zinitix_ts_dev_pm_ops = {
	.suspend = ztx_pm_suspend,
	.resume = ztx_pm_resume,
};
#endif /* CONFIG_PM */

static struct i2c_driver bt541_ts_driver = {
	.probe	= bt541_ts_probe,
	.remove	= bt541_ts_remove,
	#ifdef CONFIG_HAS_EARLYSUSPEND
	.suspend    = zinitix_early_suspend,
	.resume     = zinitix_late_resume,
	#endif
	#if 0
	.shutdown = bt541_ts_shutdown,
	#endif
	.id_table	= bt541_idtable,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= BT541_TS_DEVICE,
	.of_match_table = zinitix_match_table,
	#ifdef CONFIG_PM
		.pm	= &zinitix_ts_dev_pm_ops,
	#endif
	},
};

static int __init bt541_ts_init(void)
{

	int boot_mode = get_boot_mode();
	if (oplus_panel_id != ZINITIX_PANEL_ID_V1 && oplus_panel_id != ZINITIX_PANEL_ID_V2) {
		zinitix_printk("TP is not match this panel id 0x%06X\n", oplus_panel_id);
		return 0;
	}

	if ((MSM_BOOT_MODE__RF == boot_mode)||(MSM_BOOT_MODE__WLAN == boot_mode)) {
		zinitix_printk("TP is not run in this WLAN mode\n");
		return 0;
	}

	if (MSM_BOOT_MODE__FACTORY == boot_mode) {
		zinitix_printk("TP is not run in this FTM mode\n");
		return 0;
	}
	zinitix_printk("[TSP]: %s\n", __func__);

	return i2c_add_driver(&bt541_ts_driver);

}

static void __exit bt541_ts_exit(void)
{
	int boot_mode = get_boot_mode();
	if (oplus_panel_id != ZINITIX_PANEL_ID_V1 && oplus_panel_id != ZINITIX_PANEL_ID_V2) {
		return;
	}

	if ((MSM_BOOT_MODE__RF == boot_mode)||(MSM_BOOT_MODE__WLAN == boot_mode)) {
		return;
	}

	if (MSM_BOOT_MODE__FACTORY == boot_mode) {
		return;
	}
	zinitix_printk("[TSP]: %s\n", __func__);
	i2c_del_driver(&bt541_ts_driver);
}

//module_init(bt541_ts_init);
late_initcall(bt541_ts_init);
module_exit(bt541_ts_exit);

MODULE_DESCRIPTION("touch-screen device driver using i2c interface");
MODULE_AUTHOR("<mika.kim@samsung.com>");
MODULE_LICENSE("GPL");

