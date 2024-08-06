#include <linux/syscalls.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/rtc.h>
#include <linux/proc_fs.h>
#include "startup_monitor.h"


#define TIME_FORMAT               "yyyy-mm-dd HH:mm:ss"
#define BOOT_STR_SIZE             256


static struct startup_monitor_info *current_info = 0;
extern void log_dump(const char * phx_error);
static struct startup_monitor_action_mapping monitor_action_mapping[] = {
    {ACTION_SET_BOOTSTAGE, set_boot_stage},
    {ACTION_SET_BOOTERROR, set_boot_error},
};

static void get_local_time(char* strtime, size_t strtime_length)
{
    struct timespec ts;
    struct rtc_time tm;
    getnstimeofday(&ts);
    rtc_time_to_tm(ts.tv_sec, &tm);
    snprintf(strtime, strtime_length, "%04d-%02d-%02d %02d:%02d:%02d", tm.tm_year + 1900,
        tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
}

static void update_current_error(const char *error)
{
    if(!current_info)
        return;
    snprintf(current_info->error, sizeof(current_info->error), "%s", error);
}

static void update_happen_time(const char *happen_time)
{
    if(!current_info)
        return;
    snprintf(current_info->happen_time, sizeof(current_info->happen_time), "%s", happen_time);
}

static void kernel_log_booterror(const char * errno, const char *time)
{
    char error_pnt[CMD_STR_MAX_SIZE] = {0};
    snprintf(error_pnt, CMD_STR_MAX_SIZE, "ERROR_%s", errno);
    // panic will diable local interrupts, msleep behaviour is not permissive
        //op_log_boot(error_pnt);
    update_current_error(errno);
    update_happen_time(time);
    pr_info("%s, happen_time: %s\n", errno, time);
}




static void handle_hang_oplus(const char *happen_time)
{
    kernel_log_booterror(ERROR_HANG_OPLUS, happen_time);
    log_dump(ERROR_HANG_OPLUS);
}

static void handle_native_reboot_into_recovery(const char *happen_time)
{
    kernel_log_booterror(ERROR_NATIVE_REBOOT_INTO_RECOVERY, happen_time);
}

static void choose_action_handler(const char * cmd, struct startup_monitor_action_mapping * action_map_array,
    size_t action_map_array_size)
{
    char action[CMD_STR_MAX_SIZE] = {0};
    char *p = strchr(cmd, '@');
    size_t i = 0;
    if(NULL == p)
    {
        pr_err("invalid command\n");
        return;
    }
    strncpy(action, cmd, p - cmd);
    for(; i< action_map_array_size; i++)
    {
        if(!strcmp(action_map_array[i].action, action))
        {
            action_map_array[i].map_func(p + 1);
            return;
        }
    }
    return;
}

static void strip_line_break(const char * line, char *result)
{
    size_t line_length = strlen(line);
    size_t copy_length = strchr(line, '\n') != NULL ? line_length - 1 : line_length;
    strncpy(result, line, copy_length);
}

static void startup_monitor(const char *monitoring_command)
{
    char stripped_command[CMD_STR_MAX_SIZE] = {0};

    if(strlen(monitoring_command) > CMD_STR_MAX_SIZE - 1)
    {
        pr_err("monitor command too long\n");
        return;
    }
    strip_line_break(monitoring_command, stripped_command);
    choose_action_handler(stripped_command, monitor_action_mapping, ARRAY_SIZE(monitor_action_mapping));
}
static struct startup_monitor_action_mapping errno_handle_action_mapping[] = {
    {ERROR_HANG_OPLUS, handle_hang_oplus},
    {ERROR_NATIVE_REBOOT_INTO_RECOVERY, handle_native_reboot_into_recovery},
};

void set_boot_stage(const char *stage)
{
}

EXPORT_SYMBOL(set_boot_stage);

void set_boot_error(const char *errno)
{
    int i = 0;
	char time_pattern[ARRAY_SIZE(TIME_FORMAT) + 1] = {0};

    get_local_time(time_pattern, ARRAY_SIZE(time_pattern));
    for(; i < ARRAY_SIZE(errno_handle_action_mapping); i++)
    {
        if(!strcmp(errno_handle_action_mapping[i].action, errno))
        {
            errno_handle_action_mapping[i].map_func(time_pattern);
            return;
        }
    }
}

EXPORT_SYMBOL(set_boot_error);

static ssize_t startup_monitor_write(struct file *filp, const char *ubuf, size_t cnt, loff_t *data)
{
	char buf[BOOT_STR_SIZE];
	size_t copy_size = cnt;

	if (cnt >= sizeof(buf))
		copy_size = BOOT_STR_SIZE - 1;

	if (copy_from_user(&buf, ubuf, copy_size))
		return -EFAULT;

	buf[copy_size] = 0;
	startup_monitor(buf);

	return cnt;

}

static int startup_monitor_show(struct seq_file *m, void *v)
{
	return 0;
}

static int startup_monitor_open(struct inode *inode, struct file *file)
{
	return single_open(file, startup_monitor_show, inode->i_private);
}

static const struct file_operations startup_monitor_fops = {
	.open = startup_monitor_open,
	.write = startup_monitor_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init startup_monitor_core_init(void)
{
    struct proc_dir_entry *pe;

    pe = proc_create("startup_log", 0666, NULL, &startup_monitor_fops);

	if (!pe)
		return -ENOMEM;
    current_info = (struct startup_monitor_info *)kmalloc(sizeof(struct startup_monitor_info), GFP_KERNEL);
    if(!current_info)
    {
        pr_err("kmalloc to startup_monitor_info failed\n");
        return -1;
    }
    return 0;
}

postcore_initcall(startup_monitor_core_init);

MODULE_DESCRIPTION("startup_monitor core");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("LiFeng Chen");

