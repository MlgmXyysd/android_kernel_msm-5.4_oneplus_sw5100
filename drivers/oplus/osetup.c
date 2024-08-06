#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <asm/setup.h>
#include <linux/suspend.h>
#include "./include/oplus.h"


unsigned int oplus_hw_id = 0;
unsigned int oplus_board_id = 0;
unsigned int oplus_board_type = 0;
unsigned int oplus_panel_id = 0x097006;
unsigned int panel_present = 1;
unsigned int dsi_relock_en = 1;  /*dsi pll relock failed flag*/
unsigned int user_printk_disable_uart = 1;
unsigned int is_user_build = 0;
unsigned int is_recovery_mode = 0;
unsigned int display_vflip = 0;
unsigned int shutdown_mode = 0;

EXPORT_SYMBOL(oplus_hw_id);
EXPORT_SYMBOL(oplus_board_id);
EXPORT_SYMBOL(oplus_board_type);
EXPORT_SYMBOL(oplus_panel_id);
EXPORT_SYMBOL(panel_present);
EXPORT_SYMBOL(dsi_relock_en);
EXPORT_SYMBOL(user_printk_disable_uart);
EXPORT_SYMBOL(is_user_build);
EXPORT_SYMBOL(is_recovery_mode);
EXPORT_SYMBOL(display_vflip);
EXPORT_SYMBOL(shutdown_mode);

/******************************************************************************/
unsigned int __always_inline get_oplus_hw_id(void)
{
	return oplus_hw_id;
}

unsigned int __always_inline get_oplus_board_id(void)
{
	return oplus_board_id;
}

unsigned int __always_inline get_oplus_board_type(void)
{
	return oplus_board_type;
}

unsigned int __always_inline get_oplus_panel_id(void)
{
	return oplus_panel_id;
}

unsigned int __always_inline get_panel_present(void)
{
	return panel_present;
}

unsigned int __always_inline get_dsi_relock_en(void)
{
	return dsi_relock_en;
}

unsigned int __always_inline get_user_printk_disable_uart(void)
{
	return user_printk_disable_uart;
}

// unsigned int __always_inline is_user_build(void)
// {
// 	return is_user_build;
// }

// unsigned int __always_inline is_recovery_mode(void)
// {
// 	return is_recovery_mode;
// }

unsigned int __always_inline get_display_vflip(void)
{
	return display_vflip;
}

bool pm_suspend_deepsleep(void)
{
	return PM_SUSPEND_MEM == mem_sleep_current;
}
EXPORT_SYMBOL_GPL(pm_suspend_deepsleep);
/******************************************************************************/
static int __init set_hw_id(char *str)
{
	char *end;
	long new = simple_strtol(str, &end, 0);
	if (end == str || new > INT_MAX || new < INT_MIN)
		return -EINVAL;
	oplus_hw_id = new;
	return 1;
}
__setup("oplus.hw_id=", set_hw_id);

static int __init set_board_id(char *str)
{
	char *end;
	long new = simple_strtol(str, &end, 0);
	if (end == str || new > INT_MAX || new < INT_MIN)
		return -EINVAL;
	oplus_board_id = new;
	return 1;
}
__setup("oplus.board_id=", set_board_id);

static int __init set_board_type(char *str)
{
	char *end;
	long new = simple_strtol(str, &end, 0);
	if (end == str || new > INT_MAX || new < INT_MIN)
		return -EINVAL;
	oplus_board_type = new;
	return 1;
}
__setup("oplus.board_type=", set_board_type);

static int __init set_panel_id(char *str)
{
	char *end;
	long new = simple_strtol(str, &end, 16);
	if (end == str || (new & 0xFF000000))
			return -EINVAL;
	oplus_panel_id = new;
	return 1;
}
__setup("oplus.panel_id=", set_panel_id);

static int __init set_is_panel_present(char *str)
{
	panel_present = 0;
	return 1;
}
__setup("panel.null", set_is_panel_present);

static int __init set_is_dsi_relock(char *str)
{
	dsi_relock_en = 0;
	return 1;
}
__setup("dsi.relock", set_is_dsi_relock);

static int __init set_is_display_vflip(char *str)
{
	display_vflip = 1;
	return 1;
}
__setup("display.vflip", set_is_display_vflip);


static int __init set_printk_disable_uart(char *str)
{
	char *end;
	long new = simple_strtol(str, &end, 0);
	if (end == str || new > INT_MAX || new < INT_MIN)
		return -EINVAL;
	user_printk_disable_uart = new;
	return 1;
}
__setup("printk.disable_uart=", set_printk_disable_uart);


static int __init set_shutdown_soc_100(char *str)
{
	char *end;
	long new = simple_strtol(str, &end, 0);
	if (end == str || new > INT_MAX || new < INT_MIN)
		return -EINVAL;
	shutdown_mode = new;
	return 1;
}
__setup("clikshutdown=", set_shutdown_soc_100);

static int __init set_serial_buildvariant(char *line)
{
	const char typeuser[]  = "user";
	char oplus_buildvariant[20];

		strlcpy(oplus_buildvariant, line, sizeof(oplus_buildvariant));

		is_user_build = !strncmp(oplus_buildvariant, typeuser, sizeof(typeuser));
	return 1;
}
__setup("buildvariant=", set_serial_buildvariant);

static int __init set_boomode(char *line)
{
	const char typemode[]  = "recovery";
	char oplus_bootmode[20];

	strlcpy(oplus_bootmode, line, sizeof(oplus_bootmode));

	is_recovery_mode = !strncmp(oplus_bootmode, typemode, sizeof(typemode));
	return 1;
}
__setup("androidboot.mode=", set_boomode);

