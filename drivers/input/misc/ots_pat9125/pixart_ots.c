/* drivers/input/misc/ots_pat9125/pixart_ots.c
 *
 * Copyright (c) 2016 ~ 2022, The Linux Foundation. All rights reserved.
 *
 */

#include "pixart_platform.h"
#include "pixart_ots.h"


static unsigned char cpi_resolution_x = PIXART_PAT9125_CPI_RESOLUTION_X;

static int __init setup_cpi_resolution_x(char *str)
{
	char *end;
	long new = simple_strtol(str, &end, 0);
	if (end == str || new > 255)
		return -EINVAL;
	cpi_resolution_x = new;
	return 1;
}
__setup("cpi_x=", setup_cpi_resolution_x);

unsigned char get_cpi_resolution_x()
{
	return cpi_resolution_x;
}

void set_cpi_resolution_x(unsigned char new)
{
	pr_err("%s(0x%02x)", __func__, new);
	cpi_resolution_x = new;
}


static bool ots_write_read(struct i2c_client *client, u8 address, u8 wdata)
{
	u8 read_value, try_cnt = 3;

	do {
		write_data(client, address, wdata);
		read_value = read_data(client, address);
		try_cnt--;
	} while ((read_value != wdata) && (try_cnt));
	if (read_value != wdata) {
		pr_err("%s(): A:0x%02x W:0x%02x R:0x%02x", __func__,
			address, wdata, read_value);
		return FALSE;
	}
	return TRUE;
}

extern uint static_xcpi;
static inline bool ots_sensor_set_register(struct i2c_client *client)
{
	bool ret = TRUE;
	/* disable write protect */
	ret = ots_write_read(client, PIXART_PAT9125_WRITE_PROTECT_REG,
			PIXART_PAT9125_DISABLE_WRITE_PROTECT);
	if (!ret)
		return FALSE;

	/* set X-axis resolution (depends on application) */
	ret = ots_write_read(client, PIXART_PAT9125_SET_CPI_RES_X_REG, static_xcpi);
			//PIXART_PAT9125_CPI_RESOLUTION_X_STATIC);
	if (!ret)
		return FALSE;

	/* set Y-axis resolution (depends on application) */
	ret = ots_write_read(client, PIXART_PAT9125_SET_CPI_RES_Y_REG,
			PIXART_PAT9125_CPI_RESOLUTION_Y);
	if (!ret)
		return FALSE;

	/* set 12-bit X/Y data format (depends on application) */
	ret = ots_write_read(client, PIXART_PAT9125_ORIENTATION_REG,
			PIXART_PAT9125_MOTION_DATA_LENGTH);
	if (!ret)
		return FALSE;

	/* ONLY for VDD=VDDA=1.7~1.9V: for power saving */
	ret = ots_write_read(client, PIXART_PAT9125_VOLTAGE_SEGMENT_SEL_REG,
			PIXART_PAT9125_LOW_VOLTAGE_SEGMENT);

	/* ONLY for V: for power saving */
	ret = ots_write_read(client, PIXART_PAT9125_7C_REG,
			PIXART_PAT9125_7C_INIT_VALUE);
	if (!ret)
		return FALSE;

	ret = ots_write_read(client, PIXART_PAT9125_2B_REG,
			PIXART_PAT9125_2B_INIT_VALUE);
	if (!ret)
		return FALSE;

	ret = ots_write_read(client, PIXART_PAT9125_2D_REG,
			PIXART_PAT9125_2D_INIT_VALUE);
	if (!ret)
		return FALSE;

	//if (read_data(client, PIXART_PAT9125_MISC2_REG) == 0x04) {
	//	ots_write_read(client, PIXART_PAT9125_MISC2_REG, 0x08);
	//	if (read_data(client, PIXART_PAT9125_MISC1_REG) == 0x10)
	//		ots_write_read(client, PIXART_PAT9125_MISC1_REG,
	//				0x19);
	//}
	/* enable write protect */
	ret = ots_write_read(client, PIXART_PAT9125_WRITE_PROTECT_REG,
			PIXART_PAT9125_ENABLE_WRITE_PROTECT);
	if (!ret)
		return FALSE;

	return TRUE;
}


bool ots_sensor_init(struct i2c_client *client)
{
	u8 sensor_pid = 0;
	bool read_id_ok = true;
	int try_cnt = 0;

	for (try_cnt = 0; try_cnt < RESET_RETRY_CNT_MAX; try_cnt++) {
		/*
		* Read sensor_pid in address 0x00 to check if the
		* serial link is valid, read value should be 0x31.
		*/
		sensor_pid = read_data(client, PIXART_PAT9125_PRODUCT_ID1_REG);
		if (sensor_pid == PIXART_PAT9125_SENSOR_ID) {
			u8 read_value = 0;

			/*
			* PAT9125 sensor recommended settings:
			* switch to bank0, not allowed to perform ots_write_read
			*/
			write_data(client, PIXART_PAT9125_SELECT_BANK_REG,
					PIXART_PAT9125_BANK0);

			/*
			* software reset (i.e. set bit7 to 1).
			* It will reset to 0 automatically
			* so perform OTS_RegWriteRead is not allowed.
			* This action device will return nak,
			* and the supplier confirms that it does not care.
			*/
			write_data(client, PIXART_PAT9125_CONFIG_REG,
					PIXART_PAT9125_RESET);

			/* delay 15ms */
			usleep_range(RESET_DELAY_US, RESET_DELAY_US + 100);

			read_value = read_data(client, PIXART_PAT9125_CONFIG_REG);
			if ((PIXART_PAT9125_RESET & 0x7f) == read_value) {
				dev_info(&client->dev, "try %d reset success\n", try_cnt);
				ots_sensor_set_register(client);
				/*set sleep2 sleep1 all enable, and force sleep2*/
				write_data(client, PIXART_PAT9125_OP_MODE_REG,
					PIXART_PAT9125_FORCE_SLEEP2);
				read_id_ok = true;
				break;
			}
			dev_err(&client->dev, "try %d reset failed\n", try_cnt);
		}
		/* delay 20ms */
		usleep_range(DELAY_BETWEEN_REG_US, DELAY_BETWEEN_REG_US + 100);
	}

	return read_id_ok;
}
