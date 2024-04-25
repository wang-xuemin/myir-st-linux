/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright(c) 2023 Semidrive
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _TS_SERDES_AGENT_H
#define _TS_SERDES_AGENT_H

#include <linux/i2c.h>

struct ts_serdes {
	u32 serdes_enable;
	const char *serdes_name;
	const char *ser_name;
	const char *des_name;
	u32 ser_addr;
	u32 des_addr;
	u32 serdes_irq_type;
	u32 ser_port_type;
	int ser_irq_gpio;
	int des_irq_gpio;
	int ser_rst_gpio;
	int des_rst_gpio;
};

extern int ts_serdes_support_check(struct i2c_client *client, struct ts_serdes *cfg);
extern int ts_serdes_link_check(struct i2c_client *client, struct ts_serdes *cfg);
extern int ts_serdes_link_stat(struct i2c_client *client, struct ts_serdes *cfg);
extern int ts_serdes_config(struct i2c_client *client, struct ts_serdes *cfg);
extern int ts_ser_gpio_output(struct i2c_client *client, struct ts_serdes *cfg, int gpio, int val);
extern int ts_ser_gpio_input(struct i2c_client *client, struct ts_serdes *cfg, int gpio);
extern int ts_des_gpio_output(struct i2c_client *client, struct ts_serdes *cfg, int gpio, int val);
extern int ts_des_gpio_input(struct i2c_client *client, struct ts_serdes *cfg, int gpio);
#endif

