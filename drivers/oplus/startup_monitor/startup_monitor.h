#ifndef STARTUP_MONITOR_H
#define STARTUP_MONITOR_H

//signal
#define SIGPHX_HANG                                       (SIGRTMIN + 0x11)
//state
#define ANDROID_BOOT_COMPLETED                            "ANDROID_BOOT_COMPLETED"

//error
#define ERROR_HANG_OPLUS                                   "ERROR_HANG_OPLUS"
#define ERROR_NATIVE_REBOOT_INTO_RECOVERY                 "ERROR_NATIVE_REBOOT_INTO_RECOVERY"

//action
#define ACTION_SET_BOOTSTAGE                              "SET_BOOTSTAGE"
#define ACTION_SET_BOOTERROR                              "SET_BOOTERROR"
#define CMD_STR_MAX_SIZE                                   256
extern void __weak set_boot_error(const char *error);
extern void __weak set_boot_stage(const char *stage);
struct startup_monitor_info {
    char stage[96];
    char error[96];
    char happen_time[64];
};

struct kernel_keyinfo {
    struct startup_monitor_info *baseinfo;
    int is_system_boot_completed;
    //TODO: more information
} ;

struct startup_monitor_action_mapping {
    char action[CMD_STR_MAX_SIZE];
    void (*map_func)(const char *);
};
#endif
