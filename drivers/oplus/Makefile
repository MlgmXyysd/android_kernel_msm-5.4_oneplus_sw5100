#
# Makefile for oplus's custom devices
#
#obj-$(CONFIG_OPLUS) += oplus/osetup.o //Placed in the upper layer makefile.
obj-$(CONFIG_OPLUS_FEATURE_PROJECTINFO) += project_info/
obj-$(CONFIG_OPLUS_SWD) += oplus_sensorhub/
obj-$(CONFIG_OPLUS_SWD) += dap_swd/
obj-$(CONFIG_INPUT_TOUCHSCREEN) += touchscreen/
obj-y += startup_monitor/
obj-y += oplus_hci_bcsp_hook.o
obj-y += comm_master.o
obj-y += comm_master_plus.o

obj-$(CONFIG_OPLUS_COMMON_SOFT) += oplus_project.o
obj-$(CONFIG_OPLUS_SWITCH) += oplus_switch.o
obj-y += oplus_shutdown_detect.o
obj-y += oplus_shutdown.o

obj-y += oplus_ddr_freq.o
obj-$(CONFIG_OPLUS_FEATURE_FEEDBACK) += feedback/

