#include <linux/syscalls.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/cred.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/kmsg_dump.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <linux/version.h>
#include "startup_monitor.h"


#define TASK_INIT_COMM                                     "init"
#define FILE_MODE_0666                                     0666
#define AID_SYSTEM                                         1000
#define KEYINFO_FILE_SIZE                                  (8 * 1024)
#define KE_LOG_GENERATE_TIMEOUT                            msecs_to_jiffies(4 * 500)

#define KEYINFO_FORMAT  \
"stage: %s\n\
error: %s\n\
happen_time: %s\n\
is_boot_complete: %d\n"

static void dump_hang_oplus_log(const char *);

static struct startup_monitor_action_mapping log_dumper_mapping[] = {
    {ERROR_HANG_OPLUS, dump_hang_oplus_log},
};

static void find_task_by_comm(const char * pcomm, struct task_struct ** t_result)
{
    struct task_struct *g, *t;
    rcu_read_lock();
    for_each_process_thread(g, t)
    {
        if(!strcmp(t->comm, pcomm))
        {
            *t_result = t;
            rcu_read_unlock();
            return;
        }
    }
    t_result = NULL;
    rcu_read_unlock();
}


static void dump_hang_oplus_log(const char *happen_time)
{
    struct task_struct *t_init;
    t_init = NULL;
	//init process,send sign to init collect log
    find_task_by_comm(TASK_INIT_COMM, &t_init);
    if(NULL != t_init )
    {
        pr_info("send hang oplus signal %d at %s", SIGPHX_HANG, happen_time);
        send_sig(SIGPHX_HANG, t_init, 0);
        //native process will handle remaining work
        schedule_timeout_interruptible((30+30) * HZ); //Add 30s for get ftrace log
    }
    // incase native error handle process hang or filesystem not ready
    panic(ERROR_HANG_OPLUS);
}

void log_dump(const char * error)
{
    size_t i = 0;
    for(; i< ARRAY_SIZE(log_dumper_mapping); i++)
    {
        if(!strcmp(log_dumper_mapping[i].action, error))
        {
            log_dumper_mapping[i].map_func(error);
            return;
        }
    }
}

EXPORT_SYMBOL(log_dump);
