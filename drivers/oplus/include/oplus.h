#ifndef _LINUX_OPLUS_H
#define _LINUX_OPLUS_H

extern unsigned int oplus_hw_id;
extern unsigned int oplus_board_id;
extern unsigned int oplus_board_type;
extern unsigned int oplus_panel_id;
extern unsigned int panel_present;
extern unsigned int shutdown_mode;

unsigned int get_oplus_hw_id(void);
unsigned int get_oplus_board_id(void);
unsigned int get_oplus_board_type(void);
unsigned int get_oplus_panel_id(void);
unsigned int get_panel_present(void);
unsigned int get_dsi_relock_en(void);
unsigned int get_user_printk_disable_uart(void);
// unsigned int is_user_build(void);
// unsigned int is_recovery_mode(void);
unsigned int get_display_vflip(void);
#endif /* _LINUX_OPLUS_H */
