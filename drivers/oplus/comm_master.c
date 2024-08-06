// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2023 Oplus. All rights reserved.
 */
/*******************************************************
** File: - comm_master.c
** Description: oplus sensorhub driver
** Version: 2.1
** Date : 2019/06/12
** Author: Oplus
*******************************************************/

#define DRIVER_VERSION "2.1.0.230118_beta"

#define PRINT_WAKEUP_INFO
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
#include <linux/spi/spidev.h>
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

#include <uapi/linux/sched/types.h>
#include <linux/sched/task.h>
#include <linux/timekeeping.h>
#include <linux/suspend.h>

#include "oplus_sensorhub/oplus_sensorhub.h"

#define SNSHUB_KFIFO_SIZE                           (128 * 1024)
#define OTHERS_KFIFO_SIZE                           (16 * 1024)

#define SPEEDTEST
#define DUAL_ACK                                    (1)
#define SMALL_CORE_DUMP_ACTIVE_LEVEL                 1

#ifdef SPEEDTEST
#define SPEEDTEST_CTRL_CHANNEL_ID                    952
#define SPEEDTEST_DATA_CHANNEL_ID                    953
static char upload_speed_string[512];
static int upload_speed_string_ready = 0;
#endif

static struct spidev_data *spidev = NULL;

static volatile int close_comm_state = 0x0;//bit0:download;bit1:upload
#ifdef COLLECT_WAKEUP_INFO
//extern unsigned int pm_wakeup_irq;
extern unsigned int pm_wakeup_irq(void);
#define wakeup_data_uevent() do {oplus_sensorhub_wakeup_data_uevent();}while(0)
#define WAKEUP_DATA_KFIFO_SIZE                      (1*1024)
#define WAKEUP_DATA_LEN                             (200)
static struct kfifo wakeup_data;
static struct mutex wakeup_data_fifo_lock;
#endif

struct spidev_data {
    spinlock_t          spi_lock;
    struct device       *dev;
    struct spi_device   *spi;

    uint32_t           spi_bits_per_word;

    uint8_t            *spi_rx_buffer;
    #ifdef DUAL_ACK
    uint8_t            *spi_rx_buffer_tx;
    #endif
    uint8_t            *spi_tx_buffer;

    uint8_t            *up_frame_data;

    struct mutex        write_lock;
    struct mutex        dual_comm_lock;

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

#if defined(AUTO_RESET_MCU) || defined(MCU_STATES_UEVENT)
    int auto_reset_mcu_switch;
#endif
#ifdef PRINT_WAKEUP_INFO
    uint8_t             print_wakeup;
#endif
};


struct data_cdev {
    dev_t               devt;
    struct spidev_data  *spidev;
    struct device       *dev;
    unsigned            users;
    uint16_t            channel_id;
    const char          *dev_name;

    struct mutex        read_lock;

    struct semaphore    rx_fifo_packet_cnt_sem;
    struct kfifo        rx_fifo;
    struct mutex        rx_fifo_lock;

    struct mutex        write_lock;

    struct kfifo        tx_fifo;
    struct mutex        tx_fifo_lock;

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

struct cdev_info_list_unit {
    const char          *dev_name;
    uint16_t            channel_id;
    struct data_cdev    *cdev_info;
};

/****************************
 * {"node name", channel_id}
 ****************************/
static struct cdev_info_list_unit g_cdev_info_list[] = {
    {"dcc_datah",       7},
    {"dcc_data",        1},
    {"mcu_factory",     2},
    {"gps_data",        3},
    {"std_sns",         4},
    //{"sns_batch",       5},
    //{"dcc_battery",     8},
    //{"mcu_upgrade",     9},
    //{"mcu_echo",        6},
};

#ifdef STAND_ALONE_USED
#define STAND_ALONE_USED_CHANNEL_ID             (9)
extern uint16_t inline get_sau_ch_id(void);
extern void inline set_sau_ch_id(uint16_t ch_id);
extern uint16_t is_sau_filter(uint16_t ch_id);
#endif
/**************************************************
 * The following code is about log print
 *************************************************/

#define    LOG_ID_ERR               0
#define    LOG_ID_WARN              1
#define    LOG_ID_NOTICE            2
#define    LOG_ID_INFO              3
#define    LOG_ID_PACKET_INFO       4
#define    LOG_ID_DN_SEQ            5
#define    LOG_ID_UP_SEQ            6
#define    LOG_ID_PACKET_TO_FIFO    7
#define    LOG_ID_PACKET_TO_MCU     8
#define    LOG_ID_DATA_TO_USER      9
#define    LOG_ID_DATA_FROM_USER    10

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
};

#define log_e(fmt, ...)       if(log_array[LOG_ID_ERR].sw) \
    printk(KERN_ERR "ocm:%s():%d: " fmt "\n", __func__ , __LINE__, ##__VA_ARGS__)

#define log_w(fmt, ...)       if(log_array[LOG_ID_WARN].sw) \
    printk(KERN_WARNING "ocm:%s():%d: " fmt "\n", __func__ , __LINE__, ##__VA_ARGS__)

#define log_n(fmt, ...)       if(log_array[LOG_ID_NOTICE].sw) \
    printk(KERN_NOTICE "ocm:%s():%d: " fmt "\n", __func__ , __LINE__, ##__VA_ARGS__)

#define log_i(fmt, ...)       if(log_array[LOG_ID_INFO].sw) \
    printk(KERN_INFO "ocm:%s():%d: " fmt "\n", __func__ , __LINE__, ##__VA_ARGS__)

#define log_hex(log_id, data, size, fmt, ...) \
    do { \
        if (log_array[log_id].sw) { \
            printk(KERN_ERR "ocm:hex(%s), size(%d), [" fmt "]\n",log_array[log_id].name, size, ##__VA_ARGS__); \
            print_hex_dump(KERN_ERR, "ocm:", DUMP_PREFIX_OFFSET, 16, 1, data, size, 1); \
        } \
    } while (0)

static inline void print_version(void)
{
    printk(KERN_INFO "ocm:<oplus comm master driver, verison: %s>\n", DRIVER_VERSION);
}

/**************************************************
 * The following code is master phy layer.
 *************************************************/

#define WAIT_s2m_ans_PU_TIMEOUT           2000
#define WAIT_s2m_ans_PD_TIMEOUT           500
#ifdef DUAL_ACK
#define WAIT_s2m_int_PU_TIMEOUT              2000
#endif

static inline void wakeup_source_init(struct wakeup_source *ws,
                      const char *name)
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

    disable_irq_nosync(irq);

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
        if (gpio_get_value(spidev->s2m_ans_gpio) == 1) {
        #endif
            up(&spidev->s2m_ans_pu_sem);
        #ifndef DUAL_ACK
        } else {
            /*NOTE:Submit code, please do not open "log_e".But own debug can open "log_e".*/
            //log_e("after rising edge, s2m_ans is 0");
        }
        #endif
    } else {
        /*NOTE:Submit code, please do not open "log_e".But own debug can open "log_e".*/
        //log_e("(%d, %p) irq not int pin", irq, handle);
    }
    return IRQ_HANDLED;
}

// static void spidev_complete(void *arg)
// {
//     complete(arg);
// }

/*********************************
 * @brief start spi sync
 * @param tx: tx_buf, could use NULL
 * @param rx: rx_buf, could use NULL
 * @param size: size of sync data
 * @ret: 0 is success, < 0 is fail
 *********************************/
static int spidev_spi_sync(void *tx, void *rx, size_t size)
{
    // DECLARE_COMPLETION_ONSTACK(done);
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

    // message.complete = spidev_complete;
    // message.context = &done;

    // spin_lock_irq(&spidev->spi_lock);
    if (spidev->spi != NULL)
        status = spi_sync(spidev->spi, &message);
    else
        status = -ESHUTDOWN;
    // spin_unlock_irq(&spidev->spi_lock);

    // if (status != 0)
    //     return -EFAULT;

    // wait_for_completion(&done);
    // status = message.status;
    return status == 0 ? 0 : -EFAULT;
}

#ifdef MCU_STATES_UEVENT
#define S2M_ANS_PU_CON_TIMEOUT_COUNT        (3)
#define S2M_ANS_PD_CON_TIMEOUT_COUNT        (3)
#define DN_WRITE_SEQ_FAIL_COUNT             (20)

#define mcu_states_first()  do {oplus_sensorhub_states_uevent(MCU_STATES_FIRST);}while(0)
#define mcu_states_crash()  do { \
    if (close_comm_state) { \
        log_e("comm stoped, not report uevent"); \
    } \
    oplus_sensorhub_states_uevent(MCU_STATES_CRASH);\
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
} comm_frame_t;

typedef enum {
    PACKET_CHECK_SUCCESS,
    PACKET_CHECK_DISCONT,
    PACKET_CHECK_SUMERR,
    PACKET_CHECK_REPEAT,
} packet_check_t;

#define PHY_DIRECT_SIZE                           120
#define PHY_FIFO_SIZE                             128
#define PHY_SPI_DN_SYNC_SIZE                      122
#define PHY_SPI_UP_SYNC_SIZE                      129
#define FRAME_HEAD_SIZE                           8

#define DL_MTU                                    (4 + 4 + 2048 + 2040)
#define DL_BUFF_LEN                               (4096)

#define UP_CHECKERR_THRESHOLD                     100
#define DN_CHECKERR_THRESHOLD                     50

//first frame need't check selnum
static uint8_t is_first_up_frame = 1;
static uint8_t is_first_dl_frame = 1;

static uint16_t up_selnum = 0;
static uint16_t dn_selnum = 0;


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

static int download_spi_seq(void *data, uint16_t size)
{
    int ret = 0;
    size_t xfer_size;
    comm_frame_t *frame = (comm_frame_t *)spidev->spi_tx_buffer;

    //clear buff
    memset(spidev->spi_tx_buffer, 0, DL_BUFF_LEN);
    //frame head
    frame->selnum = dn_selnum;
    frame->info = 0x00;
    frame->info_b.first_packet = is_first_dl_frame;
    frame->size = size;
    frame->checksum = get_checksum16(data, size);
    fill_framehead_bitfield(frame);
    //frmae data
    memcpy(frame + 1, data, size);
    //spi sync tx
    xfer_size = DL_BUFF_LEN;
    ret = spidev_spi_sync(spidev->spi_tx_buffer, NULL, xfer_size);
    if (0 != ret) {
        log_e("download spi sync failed, ret(%d)!", ret);
    }

    return ret;
}

static int upload_spi_seq(void)
{
    int ret = 0;
    //clear buff
    memset(spidev->spi_rx_buffer, 0, DL_BUFF_LEN);
    memset(spidev->up_frame_data, 0, DL_BUFF_LEN);

    //spi rx
    ret = spidev_spi_sync(NULL, spidev->spi_rx_buffer, DL_BUFF_LEN);
    if (0 != ret) {
        log_e("upload spi sync fail,ret(%d)", ret);
    } else {
        memcpy(spidev->up_frame_data, spidev->spi_rx_buffer, DL_BUFF_LEN);
    }

    return ret;
}


/*
 * this function check data first,
 * if status is success, function copy data to dest data,
 * otherwise, return err status and don't copy data.
 */
static packet_check_t check_and_get_data_from_ram(void *dest_data, uint16_t *data_size)
{
    packet_check_t status = PACKET_CHECK_SUCCESS;
    comm_frame_t *frame = (comm_frame_t *)(spidev->up_frame_data);

    if (check_framehead(frame) < 0 || get_checksum16(frame + 1, frame->size) !=  frame->checksum) {
        return PACKET_CHECK_SUMERR;
    }

    if (frame->info_b.first_packet || is_first_up_frame) {
        if (frame->info_b.first_packet) {
            mcu_states_first();
            log_n("%d:the MCU just reset, received first frame", up_selnum);
        }
    } else if (frame->selnum == up_selnum - 1) {
        status = PACKET_CHECK_REPEAT;
    } else if (frame->selnum != up_selnum) {
        status = PACKET_CHECK_DISCONT;
        log_e("%d:upload may lost %d packet(s)", up_selnum, frame->selnum - up_selnum);
    }

    memcpy(dest_data, frame + 1, frame->size);
    if (data_size)
        *data_size = frame->size;
    up_selnum = frame->selnum + 1;
    return status;
}


static comm_dl_status_t
comm_master_dl_download(void *data, size_t data_size)
{
    comm_dl_status_t status = DL_STATUS_TIMEOUT;
    struct time_measure_info time_info;

    wait_mcureseting();

    mutex_lock(&spidev->dual_comm_lock);
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
        log_e("%d:write init failed, s2m_ans is 1!, count %d", dn_selnum, dci.s2m_ans_pd_con_timeout);
        status = DL_STATUS_TIMEOUT;
        goto out_dgram_write;
    }
    dci.s2m_ans_pd_con_timeout = 0;

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
                    dn_selnum, dci.dn_write_seq_fail);
            if (dci.dn_write_seq_fail >= 10)
                log_e("%d:MCU maybe: crashed, updating firmware, or in debug mode!", dn_selnum);
        }
        goto out_dgram_write;
    }
    if (dci.dn_write_seq_fail > 10) {
        log_n("%d:MCU maybe recovered", dn_selnum);
    }
    dci.dn_write_seq_fail = 0;

    //end handshake
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
                    dn_selnum, dci.s2m_ans_pu_con_timeout);
            if (dci.s2m_ans_pu_con_timeout >= 10)
                log_e("%d:MCU maybe: crashed, updating firmware, or in debug mode!", dn_selnum);
        }
        goto out_dgram_write;
    }
    dci.s2m_ans_pu_con_timeout = 0;

    time_measure_begin(&time_info);
    if (wait_s2m_ans_pulldown() < 0) {
        log_e("%d:write end handshake failed!", dn_selnum);
        status = DL_STATUS_TIMEOUT;
        ++dci.s2m_ans_pd_timeout;
        ++dci.s2m_ans_pd_con_timeout;
        goto out_dgram_write;
    }

    //check slave status
    time_measure_end(&time_info);
    if (time_measure_get_us(&time_info) < DN_CHECKERR_THRESHOLD * 1000) {
        status = DL_STATUS_SUCCESS;
        is_first_dl_frame = 0;
        ++dci.dn_frame_suc;
    } else {
        comm_frame_t *frame;
        log_e("%d:MCU reports a download packet check err! Take %d us.", dn_selnum, time_measure_get_us(&time_info));
        frame = (comm_frame_t *)spidev->spi_tx_buffer;
        log_e("%d:frame header %d %d %d %d", dn_selnum, frame->selnum, frame->info, frame->size, frame->checksum);
        status = DL_STATUS_CHECK_ERR;
        ++dci.dn_frame_check_err;
        goto out_dgram_write;
    }

    ++dn_selnum;
out_dgram_write:
    m2s_int_set_value(0);
    mutex_unlock(&spidev->dual_comm_lock);
    return status;
}

static comm_dl_status_t
comm_master_dl_upload(void *data, uint16_t *data_size, uint16_t data_mem_size)
{
    comm_dl_status_t status;

    while(!down_trylock(&spidev->s2m_int_pu_sem))
        ++dci.s2m_int_down_trylock;

    if (s2m_int_get_value() == 0) {
        __pm_relax(&spidev->upload_wakesrc);
        enable_irq(spidev->s2m_int_irq);
        down(&spidev->s2m_int_pu_sem);
    };
    __pm_stay_awake(&spidev->upload_wakesrc);

    mutex_lock(&spidev->dual_comm_lock);

    if (upload_spi_seq() < 0) {
        status = DL_STATUS_CHECK_ERR;
        ++dci.up_read_seq_fail;
        goto out_dl_upload;
    }

    switch (check_and_get_data_from_ram(data, data_size)) {
    case PACKET_CHECK_SUCCESS:
        status = DL_STATUS_SUCCESS;
        is_first_up_frame = 0;
        ++dci.up_frame_suc;
        break;
    case PACKET_CHECK_DISCONT:
        status = DL_STATUS_SUCCESS;
        ++dci.up_frame_discontinuous;
        break;
    case PACKET_CHECK_SUMERR:
        status = DL_STATUS_CHECK_ERR;
        ++dci.up_frame_check_err;
        break;

    case PACKET_CHECK_REPEAT:
        status = DL_STATUS_REPEAT;
        //++dci.up_frame_repeat;
        break;
    }

out_dl_upload:
    if (status != DL_STATUS_CHECK_ERR) {
        m2s_ans_send_pulse(10);
    } else {
        m2s_ans_send_pulse(UP_CHECKERR_THRESHOLD * 1000 / 2 * 3);
    }
    mutex_unlock(&spidev->dual_comm_lock);
    m2s_ans_set_value(0);//reset ap ams gpio
    return status;
}

/****************************************************
 * The following code is master app layer. (temp)
 ****************************************************/
#define PACKET_DOWNLOAD_MAX_TRY_TIMES   10
#define PACKET_HEAD_SIZE                4
#define PACKET_PAYLOAD_SIZE             (4 + 2048 + 2039)

typedef struct {
    uint16_t channel_id;
    uint16_t data_size;
} comm_packet_t;


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
        if (!kfifo_is_empty(&g_cdev_info_list[i].cdev_info->tx_fifo))
            return g_cdev_info_list[i].cdev_info;
    }
    return NULL;  //avoid null pointer
}

#define DIRECTION_M2S 0
#define DIRECTION_S2M 1
#define STANDARD_CHANNEL_INFO_DUMP 0
static int test_msgid[4] = {0};
static inline void print_packet_info(void *data, size_t size, uint16_t channel_id, uint8_t direction)
{
    uint8_t cls = (uint8_t)((((uint16_t *)data)[0]) >> 8);
    uint8_t id = (uint8_t)((((uint16_t *)data)[0]) & 0xff);

    if (!log_array[LOG_ID_PACKET_INFO].sw)
        return;

    if ((channel_id == 1) || (channel_id == 7)) {
        if (((uint16_t *)data)[0] < 1024) {
            printk(KERN_INFO "ocm:%s (id=%d) size=%d\n",
                direction ? "S->M" : "M->S", ((uint16_t *)data)[0], ((uint16_t *)data)[1]);
        } else {
            //class24,id1,id2 is stress testing msg, lessen print log
            if ((cls == 24) && ((id == 1) || (id == 2))) {
                ++test_msgid[id];
                if (test_msgid[id] > 99) {
                    printk(KERN_INFO "ocm:%s (id=%d, class=%d) size=%d times=%d\n", direction ? "S->M" : "M->S",
                        id, cls, ((uint16_t *)data)[1], test_msgid[id]);
                    test_msgid[id] = 0;
                }
            } else {
                printk(KERN_INFO "ocm:%s (id=%d, class=%d) size=%d\n", direction ? "S->M" : "M->S",
                    id, cls,((uint16_t *)data)[1]);
            }
        }
    }
    #if STANDARD_CHANNEL_INFO_DUMP
    else {
        printk(KERN_INFO "ocm:%s (%s) size=%d\n",
            direction ? "S->M" : "M->S", get_cdev_info_by_channel_id(channel_id)->dev_name, (int)size);
    }
    #endif
}

#ifdef COLLECT_WAKEUP_INFO
static void wake_up_data_save(uint8_t *data)
{
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
    struct data_cdev  *cdev = NULL;
    int               retry_cnt = 0;
    ssize_t           status = -ENOBUFS;
    static int        sleep_flag = 0;
    int               fifo_len = 0;

#ifdef SPEEDTEST
    if (packet->channel_id == SPEEDTEST_DATA_CHANNEL_ID)
        return;
    if (packet->channel_id == SPEEDTEST_CTRL_CHANNEL_ID) {
        memcpy(upload_speed_string, packet + 1, 512);
        upload_speed_string_ready = 1;
    }
#endif

    cdev = get_cdev_info_by_channel_id(packet->channel_id);
    if (!cdev) {
        log_e("could not match data cdev!");
        return;
    }

#ifdef STAND_ALONE_USED
    if (is_sau_filter(cdev->channel_id)) {
        log_e("ch(%d) stand alone used, ch(%d) data abandoned",
            get_sau_ch_id(), cdev->channel_id);
        return;
    }
#endif

    log_hex(LOG_ID_PACKET_TO_FIFO, packet, packet->data_size + PACKET_HEAD_SIZE, "to %s", cdev->dev_name);
    //push packet to rx_fifo
    mutex_lock(&cdev->rx_fifo_lock);
#ifdef PRINT_WAKEUP_INFO
    if (spidev->print_wakeup) {
        spidev->print_wakeup = 0;
        if (pm_wakeup_irq() == spidev->s2m_int_irq) {
#ifdef COLLECT_WAKEUP_INFO
            if((1 == packet->channel_id) || (7 == packet->channel_id)){
                wake_up_data_save( (uint8_t *) (packet + 1));//save wakeup msg_id
            }
#endif
            log_e("wakeup by packet: channel(%s) size(%d)", cdev->dev_name, packet->data_size);
            print_hex_dump(KERN_ERR, "ocm: ",
                DUMP_PREFIX_OFFSET, 16, 1, (uint8_t *)packet,
                PACKET_HEAD_SIZE + packet->data_size > 16 ?
                16 : PACKET_HEAD_SIZE + packet->data_size, 0);
        }
    }
#endif
#define USER_READ_MAX_TRY_TIMES        (2)
#define CHANNEL_H_READ_WAIT_TIMES_MS        (200)
#define CHANNEL_O_READ_WAIT_TIMES_MS        (20)
    while(retry_cnt < USER_READ_MAX_TRY_TIMES) {
        fifo_len = kfifo_len(&cdev->rx_fifo);
        if((sleep_flag == 0) && (fifo_len > (80 * 1024) )){
            sleep_flag = 1;
            mutex_unlock(&cdev->rx_fifo_lock);
            log_e("fifo_len size(%d) almost full sleep 50 ms", fifo_len);
            msleep((1 == cdev->channel_id) ? 50 : 0);
            mutex_lock(&cdev->rx_fifo_lock);
        }
        if(fifo_len < (50 * 1024)){
            sleep_flag = 0;
        }

        if (kfifo_avail(&cdev->rx_fifo) > PACKET_HEAD_SIZE + packet->data_size) {
            kfifo_in(&cdev->rx_fifo, packet, PACKET_HEAD_SIZE + packet->data_size);
            up(&cdev->rx_fifo_packet_cnt_sem);
            __pm_wakeup_event(&cdev->wakesrc, 1000);
            ++cdev->cci.rx_packet_to_fifo;
            status = 0;
            break;
        }

        ++cdev->cci.rx_fifo_full;
        if (cdev->cci.rx_fifo_full < 10 || cdev->cci.rx_fifo_full % 100 == 0)
            log_e("rx_fifo of %s is full, try(%d), %d,%d\n", \
            cdev->dev_name, retry_cnt, cdev->cci.rx_fifo_full, cdev->cci.rx_packet_to_fifo);
        mutex_unlock(&cdev->rx_fifo_lock);
        msleep((1 == cdev->channel_id)?CHANNEL_H_READ_WAIT_TIMES_MS:CHANNEL_O_READ_WAIT_TIMES_MS);
        mutex_lock(&cdev->rx_fifo_lock);
        retry_cnt++;
    }

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
    mutex_unlock(&cdev->rx_fifo_lock);
}


/*
 * GPS_DATAa, std_sns, etc used
 * write data to MCU, call many times spidev_frame_write
 * size must <= SNSHUB_MAX_RX_SIZE
 */
static int comm_master_download(const void *buf, size_t size, uint16_t channel_id)
{
    static uint8_t packet_area[PACKET_HEAD_SIZE + PACKET_PAYLOAD_SIZE] = {0};
    comm_packet_t *packet = (comm_packet_t *)packet_area;
    int i;
    int check_err_cnt = 0;

#ifdef STAND_ALONE_USED
    if (is_sau_filter(channel_id)) {
        return -EINVAL;
    }
#endif

    if (size > PACKET_PAYLOAD_SIZE) {
        return -EINVAL;
    }

    memset(packet_area, 0, PACKET_HEAD_SIZE + PACKET_PAYLOAD_SIZE);
    packet->channel_id = channel_id;
    packet->data_size = size;
    memcpy(packet + 1, buf, size);

    log_hex(LOG_ID_PACKET_TO_MCU, packet, packet->data_size + PACKET_HEAD_SIZE, "channel id %d", channel_id);
    for (i = 1; i <= PACKET_DOWNLOAD_MAX_TRY_TIMES; ++i) {
        comm_dl_status_t dl_status = comm_master_dl_download(packet, packet->data_size + PACKET_HEAD_SIZE);
        if (DL_STATUS_SUCCESS == dl_status) {
            if (i > 1)
                log_w("packet send successfully, but try %d times.", i);
            break;
        }
        if (close_comm_state & 0x01) {
            log_e("download_comm stop, try%d", i);
            break;
        }
        if (DL_STATUS_CHECK_ERR == dl_status) {
            ++check_err_cnt;
            if (PACKET_DOWNLOAD_MAX_TRY_TIMES == check_err_cnt) {
                mcu_states_crash();
            }
        } else if (DL_STATUS_TIMEOUT == dl_status) {
            log_w("packet send timeout, need sleep %dms.", 50*i);
            msleep(50*i);
        }
        if (i == PACKET_DOWNLOAD_MAX_TRY_TIMES) {
            if (dci.s2m_ans_pu_con_timeout < 10)
                log_e("packet send failed, try %d times!", i);
            return -1;
        }
    }
    return 0;
}

/*read_thread*/
static int comm_master_upload_thread(void *arg)
{
    static uint8_t packet_area[PACKET_HEAD_SIZE + PACKET_PAYLOAD_SIZE] = {0};
    comm_packet_t *packet = (comm_packet_t *)packet_area;
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
        dl_status = comm_master_dl_upload(packet, NULL, PACKET_HEAD_SIZE + PACKET_PAYLOAD_SIZE);

        while (close_comm_state & 0x02) {
            //log_e("upload_thread stop");
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
            comm_frame_t *frame = (comm_frame_t *)(spidev->up_frame_data);
            if (dl_status == DL_STATUS_CHECK_ERR) {
                log_e("upload check err! frame header %d %d %d %d", frame->selnum, frame->info, frame->checksum, frame->size);
            }
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
    static uint8_t packet_area[PACKET_HEAD_SIZE + PACKET_PAYLOAD_SIZE] = {0};
    comm_packet_t *packet = (comm_packet_t *)packet_area;
    int status = 0;
    int data_size = 0;
    struct data_cdev  *cdev = NULL;
    struct sched_param param = {.sched_priority = ((MAX_RT_PRIO / 2) + 2) };
    if (sched_setscheduler(spidev->snshub_write_tsk, SCHED_FIFO, &param))
        log_e("sched_setscheduler download SCHED_FIFO failed!!!");
    log_i("download thread configed: pid=%d policy=%d priority=%d",spidev->snshub_write_tsk->pid,
        spidev->snshub_write_tsk->policy, spidev->snshub_write_tsk->prio);

    while (1) {
        cdev = get_next_cdev_for_download();
        status = 0;
        data_size = 0;
        memset(packet_area, 0, PACKET_HEAD_SIZE + PACKET_PAYLOAD_SIZE);

        if (close_comm_state & 0x01){
            //log_e("download_thread stop");
            //msleep(5000);
            continue;
        }

        if (cdev != NULL) {
#ifdef STAND_ALONE_USED
            if (is_sau_filter(cdev->channel_id)) {
                log_e("ch(%d) stand alone used, ch(%d) data not send",
                    get_sau_ch_id(), cdev->channel_id);
                continue;
            }
#endif
            //get packet from tx_fifo
            mutex_lock(&cdev->tx_fifo_lock);
            if (kfifo_out(&cdev->tx_fifo, packet, PACKET_HEAD_SIZE) != PACKET_HEAD_SIZE) {
                status = -EFAULT;
                log_e("impossible situation occured in LINE %d", __LINE__);
            }

            data_size = packet->data_size;
            //take data
            if (kfifo_out(&cdev->tx_fifo, (packet + 1), data_size) != data_size) {
                status = -EFAULT;
                log_e("impossible situation occured in LINE %d", __LINE__);
            }

            //err solve
            if (status < 0) {
                log_e("clean tx_fifo to avoid fatal err");
                kfifo_reset(&cdev->tx_fifo);
            }

            mutex_unlock(&cdev->tx_fifo_lock);

            //data get error so do next
            if (status < 0)
                continue;
            mutex_lock(&cdev->spidev->write_lock);
            if (comm_master_download((packet + 1), data_size, packet->channel_id) == 0) {
                print_packet_info((packet + 1), data_size, packet->channel_id, DIRECTION_M2S);
                ++cdev->cci.tx_packet_send_suc;
                ++cdev->cci.tx_packet_to_mcu;
            } else {
                status = -EFAULT;
                ++cdev->cci.tx_packet_send_fail;
            }
            mutex_unlock(&cdev->spidev->write_lock);
            //change thread
            if (gpio_get_value(spidev->s2m_int_gpio) == 1) {
                msleep(1);
            }
        } else {
            log_e("could not match data cdev! m2s_tx_fifo_sem semaphore error!!!");
            msleep(10);
        }

    }
    return 0;
}

static ssize_t
spidev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct data_cdev    *read_info;
    struct spidev_data  *spidev;
    unsigned long       missing = 0;
    //static uint8_t packet_area[PACKET_HEAD_SIZE + PACKET_PAYLOAD_SIZE] = {0};
    //comm_packet_t *packet = (comm_packet_t *)packet_area;
    comm_packet_t *packet = NULL;
    int                 status = 0;
    int                 data_size = 0;

    read_info = filp->private_data;
    spidev = read_info->spidev;
    packet = (comm_packet_t *)read_info->read_packet_area;

    mutex_lock(&read_info->read_lock);
    memset(packet, 0, PACKET_HEAD_SIZE + PACKET_PAYLOAD_SIZE);

    //wait when conditions
    if (down_trylock(&read_info->rx_fifo_packet_cnt_sem)) {  //can't get semaphore
        __pm_relax(&read_info->wakesrc);
        if (filp->f_flags & O_NONBLOCK) {  //nio
            log_i("use nio but no data to read");
            status = -EAGAIN;
            goto out_spidev_read;
        } else {  //bio
            //wait when no data to read,
            //inter "if" when interrupted by signal
            if (down_interruptible(&read_info->rx_fifo_packet_cnt_sem)) {
                //log_i("rx_fifo_packet_cnt_sem interrupted by signal");
                status = -EINTR;
                goto out_spidev_read;
            }
        }
    }

    mutex_lock(&read_info->rx_fifo_lock);

    if (kfifo_out(&read_info->rx_fifo, packet, PACKET_HEAD_SIZE) != PACKET_HEAD_SIZE) {
        status = -EFAULT;
        log_e("impossible situation occured in LINE %d", __LINE__);
    }

    data_size = packet->data_size;
    //take data
    if (kfifo_out(&read_info->rx_fifo, packet + 1, data_size) != data_size) {
        status = -EFAULT;
        log_e("impossible situation occured in LINE %d", __LINE__);
    }

    //err solve
    if (status < 0) {
        log_e("clean rx_fifo and sem to avoid fatal err");
        kfifo_reset(&read_info->rx_fifo);
        while (!down_trylock(&read_info->rx_fifo_packet_cnt_sem));
    }

    mutex_unlock(&read_info->rx_fifo_lock);

    //copy data to user
    if (status == 0) {
        if (count < data_size) {
            data_size = count;
            log_e("missing some data (%d bytes) to user because count < data_size", data_size - count);
        }
        missing = copy_to_user(buf, packet + 1, data_size);
        log_hex(LOG_ID_DATA_TO_USER, packet + 1, packet->data_size, "%s", read_info->dev_name);
        if (!missing) {
            status = data_size;
            print_packet_info(packet + 1, data_size, read_info->channel_id, DIRECTION_S2M);
            ++read_info->cci.rx_packet_to_user;
        } else {
            log_e("copy_to_user missing some data");
            status = -EFAULT;
        }
    }

out_spidev_read:
    mutex_unlock(&read_info->read_lock);
    return status;
}

static ssize_t spidev_write(struct file *filp,
        const char __user *buf, size_t count, loff_t *f_pos)
{
    struct data_cdev        *wrtie_info;
    struct spidev_data      *spidev;
    ssize_t                 status = -ENOBUFS;
    unsigned long           missing;
    //static DEFINE_MUTEX(packet_area_mutex);
    //static uint8_t packet_area[PACKET_HEAD_SIZE + PACKET_PAYLOAD_SIZE] = {0};
    //comm_packet_t *packet = (comm_packet_t *)packet_area;
    comm_packet_t *packet = NULL;
    int                     retry_cnt = 0;

    wrtie_info = filp->private_data;
    spidev  = wrtie_info->spidev;
    packet = (comm_packet_t *)wrtie_info->write_packet_area;

    if (count > PACKET_PAYLOAD_SIZE) {
        log_e("write data length(%d) more then max count(PACKET_PAYLOAD_SIZE)", count);
        return -EINVAL;
    }

#ifdef STAND_ALONE_USED
    if (is_sau_filter(wrtie_info->channel_id)) {
        log_e("ch(%d) write data length(%d) failed, because ch(%d) stand alone used",
            wrtie_info->channel_id, count, get_sau_ch_id());
        return count;
    }
#endif

    //mutex_lock(&packet_area_mutex);

    mutex_lock(&wrtie_info->write_lock);
    memset(packet, 0, PACKET_HEAD_SIZE + PACKET_PAYLOAD_SIZE);
    packet->channel_id = wrtie_info->channel_id;
    packet->data_size = count;

    missing = copy_from_user((packet + 1), buf, count);
    log_hex(LOG_ID_DATA_FROM_USER, (packet + 1), count, "%s", wrtie_info->dev_name);
    if (missing) {
        log_e("copy from user missing");
        status = -EFAULT;
        goto out_spidev_write;
    }

#define USER_WRITE_MAX_TRY_TIMES        (4)
#define CHANNEL_H_WAIT_TIMES_MS         (300)
#define CHANNEL_O_WAIT_TIMES_MS         (30)  //O is other
    while(retry_cnt < USER_WRITE_MAX_TRY_TIMES) {
        mutex_lock(&wrtie_info->tx_fifo_lock);
        if (kfifo_avail(&wrtie_info->tx_fifo) > (count + PACKET_HEAD_SIZE)) {
            kfifo_in(&wrtie_info->tx_fifo, packet, (count + PACKET_HEAD_SIZE));
            ++wrtie_info->cci.tx_packet_to_fifo;
            status = count;
            up(&spidev->m2s_tx_fifo_sem);
        }
        mutex_unlock(&wrtie_info->tx_fifo_lock);

        if (likely(status == count)) {
            break;
        }

        ++wrtie_info->cci.tx_fifo_full;
        retry_cnt++;
        if ((USER_WRITE_MAX_TRY_TIMES == retry_cnt)
            && (wrtie_info->cci.tx_fifo_full < 10
                || wrtie_info->cci.tx_fifo_full % 10 == 0)) {
            log_e("tx_fifo of %s is full, a frame will be abandoned, try(%d), %d,%d\n",
                wrtie_info->dev_name, retry_cnt, wrtie_info->cci.tx_fifo_full, wrtie_info->cci.tx_packet_to_fifo);
        }
        msleep((1 == wrtie_info->channel_id)?CHANNEL_H_WAIT_TIMES_MS:CHANNEL_O_WAIT_TIMES_MS);

        if (filp->f_flags & O_NONBLOCK) {  //nio
            log_i("use nio but not cache buf");
            break; //goto out_spidev_write;
        }
    };
out_spidev_write:
    mutex_unlock(&wrtie_info->write_lock);

    //mutex_unlock(&packet_area_mutex);
    return status;
}

static int spidev_open(struct inode *inode, struct file *filp)
{
    struct data_cdev    *data_cdev;
    int            status = -ENXIO;
    int i;

    for (i = 0; i < ARRAY_SIZE(g_cdev_info_list); ++i) {
        if (g_cdev_info_list[i].cdev_info->devt == inode->i_rdev) {
            status = 0;
            data_cdev = g_cdev_info_list[i].cdev_info;
            break;
        }
    }

    if (status) {
        log_e("spidev: nothing for minor %d", iminor(inode));
        goto err_find_dev;
    }

    if (0 == data_cdev->users) {
#ifdef STAND_ALONE_USED
        if (STAND_ALONE_USED_CHANNEL_ID == data_cdev->channel_id) {
            set_sau_ch_id(STAND_ALONE_USED_CHANNEL_ID);
        }
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
        if (STAND_ALONE_USED_CHANNEL_ID == data_cdev->channel_id) {
            set_sau_ch_id(0);
        }
#endif
    }

    if (data_cdev->users) {
        data_cdev->users--;
    }

    return 0;
}

static const struct file_operations spidev_fops = {
    .owner =    THIS_MODULE,
    .write =    spidev_write,
    .read =        spidev_read,
    .open =        spidev_open,
    .release =    spidev_release,
    .llseek =    no_llseek,
};


static struct class *spidev_class;

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


#if defined(AUTO_RESET_MCU) || defined(MCU_STATES_UEVENT)

static ssize_t
auto_reset_mcu_show(struct class *class, struct class_attribute *attr, char *buf)
{
    ssize_t count = 0;
    count += snprintf(buf + count, PAGE_SIZE-count, "%d", spidev->auto_reset_mcu_switch);
    return count;
}

static ssize_t
auto_reset_mcu_store(struct class *class, struct class_attribute *attr,
        const char *buf, size_t count)
{
    if (buf[0] == '1')
        spidev->auto_reset_mcu_switch = 1;
    else if (buf[0] == '0')
        spidev->auto_reset_mcu_switch = 0;
    else
        return -EINVAL;

    dci.s2m_ans_pu_con_timeout = -10;

    log_e("auto_reset_mcu switch set to %d", spidev->auto_reset_mcu_switch);

    return count;
}
#endif //defined(AUTO_RESET_MCU) || defined(MCU_STATES_UEVENT)

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
            packet_size = 4080;//-7Byte
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
    speed_test_complt = 0;
    /*start speedtest thread*/
    speedtest.speed_tsk = kthread_create(speedtest_thread, &speedtest, "speed_test_thd");
    if (IS_ERR(speedtest.speed_tsk)) {
        log_e("creat speedtest thread failed\n");
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
#endif

static ssize_t
commstop_show(struct class *class, struct class_attribute *attr, char *buf)
{
    ssize_t count = 0;
    count += snprintf(buf + count, PAGE_SIZE - count, "master comm state(%d)\n", close_comm_state);
    return count;
}

static ssize_t
commstop_store(struct class *class, struct class_attribute *attr,
        const char *buf, size_t count)
{
    log_e("commstop_store:start!");
    if((NULL == spidev->snshub_read_tsk) || (NULL == spidev->snshub_write_tsk)){
        log_e("commstop_store:snshub_read_tsk or snshub_write_tsk for null");
        return -EPERM;
    }

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
        return -EINVAL;
    }

    //kthread_stop(spidev->snshub_write_tsk);
    //kthread_stop(spidev->snshub_read_tsk);

    log_e("commstop_store:end! %s(%d)", current->comm, current->pid);
    return count;
}

#ifdef COLLECT_WAKEUP_INFO
static ssize_t
wakeupdata_show(struct class *class, struct class_attribute *attr, char *buf)
{
    ssize_t count = 0;
    int wakeup_data_len = 0;

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
#endif

static const struct class_attribute attributes[] = {
    __ATTR_RW(log),
    __ATTR_RW(info),
    __ATTR_RW(commstop),
#ifdef COLLECT_WAKEUP_INFO
    __ATTR_RW(wakeupdata),
#endif
#if defined(AUTO_RESET_MCU) || defined(MCU_STATES_UEVENT)
    __ATTR_RW(auto_reset_mcu),
#endif
#ifdef SPEEDTEST
    __ATTR_RW(speedtest),
#endif
};

#define SPIDEV_MAJOR            153    /* assigned */
#define N_SPI_MINORS            32    /* ... up to 256 */

static DECLARE_BITMAP(minors, N_SPI_MINORS);

static int data_cdev_init(struct spidev_data *spidev, struct cdev_info_list_unit *list_unit)
{
    int                 status;
    unsigned long        minor;
    struct spi_device     *spi = spidev->spi;
    struct data_cdev    *data_cdev = NULL;
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

    sema_init(&data_cdev->rx_fifo_packet_cnt_sem, 0);
    mutex_init(&data_cdev->read_lock);

    /* dcc_data channel would generate large amount of data,
        we need to increase rx and tx buffer to avoid data lost */
    if (list_unit->channel_id == 1)
        ret = kfifo_alloc(&data_cdev->rx_fifo, SNSHUB_KFIFO_SIZE, GFP_KERNEL);
    else
        ret = kfifo_alloc(&data_cdev->rx_fifo, OTHERS_KFIFO_SIZE, GFP_KERNEL);

    if (ret)
        return -ENOMEM;
    mutex_init(&data_cdev->rx_fifo_lock);

    mutex_init(&data_cdev->write_lock);

    /* dcc_data channel would generate large amount of data,
        we need to increase rx and tx buffer to avoid data lost */
    if (list_unit->channel_id == 1)
        ret = kfifo_alloc(&data_cdev->tx_fifo, SNSHUB_KFIFO_SIZE, GFP_KERNEL);
    else
        ret = kfifo_alloc(&data_cdev->tx_fifo, OTHERS_KFIFO_SIZE, GFP_KERNEL);

    if (ret)
        return -ENOMEM;
    mutex_init(&data_cdev->tx_fifo_lock);

    minor = find_first_zero_bit(minors, N_SPI_MINORS);
    if (minor < N_SPI_MINORS) {
        data_cdev->devt = MKDEV(SPIDEV_MAJOR, minor);
        data_cdev->dev = device_create(spidev_class, &spi->dev, data_cdev->devt,
                    data_cdev, list_unit->dev_name);
        status = PTR_ERR_OR_ZERO(data_cdev->dev);
    } else {
        dev_dbg(&spi->dev, "no minor number available!\n");
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

int data_cdev_remove(void)
{
    int i = 0;

    for (i = 0; i < ARRAY_SIZE(g_cdev_info_list); i++) {
        struct data_cdev *data_cdev = g_cdev_info_list[i].cdev_info;
        device_destroy(spidev_class, data_cdev->devt);
        clear_bit(MINOR(data_cdev->devt), minors);
        if (data_cdev->users == 0) {

        }
            kfree(data_cdev);
        g_cdev_info_list[i].cdev_info = NULL;
    }
    return 0;
}


static int get_gpio_from_devtree(struct spidev_data *spidev, const char *gpio_name)
{
    struct device *dev = &(spidev->spi->dev);
    struct device_node *node = dev->of_node;
    enum of_gpio_flags flags;
    int gpio;

    if (!node)
        return -ENODEV;

    gpio  = of_get_named_gpio_flags(node, gpio_name, 0, &flags);
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
        log_e("gpio(%d) gpio_to_irq failed: %d\n", gpio, irq);
        return -EINVAL;
    }

    if (request_irq(irq, irqfunc, irq_flags, dev_name(dev), dev) < 0) {
        log_e("Error, could not request irq\n");
        return -EINVAL;
    }
    return irq;
}

static int spidev_gpio_init(struct spidev_data *spidev)
{
    spidev->s2m_int_gpio = get_gpio_from_devtree(spidev, "s2m_int_gpio");
    if (spidev->s2m_int_gpio < 0)
        return -ENODEV;
    gpio_direction_input(spidev->s2m_int_gpio);
    spidev->s2m_int_irq = spidev_gpio_request_irq(spidev, spidev->s2m_int_gpio,
            IRQ_TYPE_LEVEL_HIGH, s2m_int_irq_fn);
    if (spidev->s2m_int_irq < 0)
        return -ENODEV;
    if (enable_irq_wake(spidev->s2m_int_irq) != 0)
        return -EINVAL;
    log_n("s2m_int_gpio: direction(in), gpio(%d), irq(%d)",
            spidev->s2m_int_gpio, spidev->s2m_int_irq);

    spidev->m2s_ans_gpio = get_gpio_from_devtree(spidev, "m2s_ans_gpio");
    if (spidev->m2s_ans_gpio < 0)
        return -ENODEV;
    gpio_direction_output(spidev->m2s_ans_gpio, 0);
    log_n("m2s_ans_gpio: direction(out), gpio(%d)", spidev->m2s_ans_gpio);

    spidev->m2s_int_gpio = get_gpio_from_devtree(spidev, "m2s_int_gpio");
    if (spidev->m2s_int_gpio < 0)
        return -ENODEV;
    gpio_direction_output(spidev->m2s_int_gpio, 0);
    log_n("m2s_int_gpio: direction(out), gpio(%d)", spidev->m2s_int_gpio);

    spidev->s2m_ans_gpio = get_gpio_from_devtree(spidev, "s2m_ans_gpio");
    if (spidev->s2m_ans_gpio < 0)
        return -ENODEV;
    gpio_direction_input(spidev->s2m_ans_gpio);
    spidev->s2m_ans_irq = spidev_gpio_request_irq(spidev, spidev->s2m_ans_gpio,
            IRQF_TRIGGER_RISING, s2m_ans_irq_fn);
    if (spidev->s2m_ans_irq < 0)
        return -ENODEV;
    log_n("s2m_ans_gpio: direction(in), gpio(%d), irq(%d)",
            spidev->s2m_ans_gpio, spidev->s2m_ans_irq);

    return 0;
}

static int spidev_probe(struct spi_device *spi)
{
    struct device *dev = &spi->dev;
    int status = 0;
    int i;

    print_version();
    if (dev == NULL) {
        log_e("dev is null!!!!!");
    }

    spi->mode |= 0x01;//spi slave mode, only mode 1
    spi->slave_ready_cb = slave_ready_cb;
    /* Allocate driver data */
    spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
    if (!spidev)
        return -ENOMEM;

    /* Initialize the driver data */
    spidev->spi = spi;
    spin_lock_init(&spidev->spi_lock);
    mutex_init(&spidev->write_lock);
    mutex_init(&spidev->dual_comm_lock);

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

    spidev->spi_rx_buffer = kzalloc(PAGE_SIZE, GFP_KERNEL);
    if (!spidev->spi_rx_buffer)
        return -ENOMEM;
    #ifdef DUAL_ACK
     spidev->spi_rx_buffer_tx = kzalloc(PAGE_SIZE, GFP_KERNEL);
     if (!spidev->spi_rx_buffer_tx)
         return -ENOMEM;
    #endif
    spidev->spi_tx_buffer = kzalloc(PAGE_SIZE, GFP_KERNEL);
    if (!spidev->spi_tx_buffer)
        return -ENOMEM;

    spidev->up_frame_data = kzalloc(PAGE_SIZE, GFP_KERNEL);
    if (!spidev->up_frame_data)
        return -ENOMEM;

    #ifdef COLLECT_WAKEUP_INFO
    if (kfifo_alloc(&wakeup_data, WAKEUP_DATA_KFIFO_SIZE, GFP_KERNEL))
        return -ENOMEM;
    mutex_init(&wakeup_data_fifo_lock);
    #endif

    spi_set_drvdata(spi, spidev);

    wakeup_source_init(&spidev->upload_wakesrc, "upload_wakesrc");
    wakeup_source_init(&spidev->download_wakesrc, "download_wakesrc");
    wakeup_source_init(&spidev->resever_wakesrc, "resever_wakesrc");

    if (spidev_gpio_init(spidev) < 0)
        log_e("spidev_gpio_init failed!");

    for (i = 0; i < ARRAY_SIZE(g_cdev_info_list); ++i)
    data_cdev_init(spidev, &g_cdev_info_list[i]);

    /*start read thread*/
    spidev->snshub_read_tsk = kthread_create(comm_master_upload_thread, spidev, "snshub_up_thd");
    if (IS_ERR(spidev->snshub_read_tsk)) {
        log_e("creat read thread failed\n");
        status = PTR_ERR(spidev->snshub_read_tsk);
    } else {
        get_task_struct(spidev->snshub_read_tsk);
        wake_up_process(spidev->snshub_read_tsk);
    }

    /*start write thread*/
    spidev->snshub_write_tsk = kthread_create(comm_master_download_thread, spidev, "snshub_down_thd");
    if (IS_ERR(spidev->snshub_write_tsk)) {
        log_e("creat write thread failed\n");
        status = PTR_ERR(spidev->snshub_write_tsk);
    } else {
        get_task_struct(spidev->snshub_write_tsk);
        wake_up_process(spidev->snshub_write_tsk);
    }

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
            log_w("snshub resume int high\n");
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
    { .compatible = "oplus,dual_comm_slave", },
    {},
};

static struct spi_driver spidev_spi_driver = {
    .driver = {
        .name  = "comm_slave",
        //.owner = THIS_MODULE,
        .of_match_table = of_match_ptr(spidev_dt_ids),
        .pm    = &snshub_pm_ops,
    },
    .probe  = spidev_probe,
    .remove = spidev_remove,
};


static int __init spidev_init(void)
{
    int status;
    int i;

    BUILD_BUG_ON(N_SPI_MINORS > 256);
    spidev_class = class_create(THIS_MODULE, "ocm");
    if (IS_ERR(spidev_class)) {
        status = PTR_ERR(spidev_class);
        pr_err("class_create failed %d", status);
        spidev_class = NULL;
        return status;
    }
    for (i = 0; i < ARRAY_SIZE(attributes); i++) {
        if (class_create_file(spidev_class, attributes + i))
            pr_err("spidev_init():cannot create file");
    }

    status = register_chrdev(SPIDEV_MAJOR, "spi", &spidev_fops);
    if (status < 0) {
        pr_err("spidev_init(): register_chrdev failed");
        return status;
    }

    status = spi_register_driver(&spidev_spi_driver);
    if (status < 0) {
        pr_err("spi_register_driver failed");
        return status;
    }

    return 0;
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
