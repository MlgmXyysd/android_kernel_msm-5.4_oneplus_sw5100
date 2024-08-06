/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 */

#ifndef __UAPI_LINUX_MSM_GENI_SERIAL_H
#define __UAPI_LINUX_MSM_GENI_SERIAL_H

/* IOCTLS used by BT clients to control UART power state */
#define MSM_GENI_SERIAL_TIOCFAULT	0x54EC	/* Uart fault */
#define MSM_GENI_SERIAL_TIOCPMGET	0x54ED	/* PM get */
#define MSM_GENI_SERIAL_TIOCPMPUT	0x54EE	/* PM put */
#define MSM_GENI_SERIAL_TIOCPMACT	0x54EF	/* PM is active */

#endif /* __UAPI_LINUX_MSM_GENI_SERIAL_H */

//Add by ouyangxun
//export for BT driver
unsigned int msm_geni_serial_tx_empty(struct uart_port *uport);
int vote_clock_off(struct uart_port *uport);
int vote_clock_on(struct uart_port *uport);
struct uart_port *msm_geni_get_uart_port(int port_index);
void msm_geni_serial_set_mctrl(struct uart_port *uport,unsigned int mctrl);
