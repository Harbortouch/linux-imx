/*
 * Copyright (C) 2016 SATO Corporation
 * License terms:GNU General Public License (GPL) version 2
 */

#ifndef __LINUX_I2C_BU26154_TS_H
#define __LINUX_I2C_BU26154_TS_H

/**
 * struct bu26154_platform_device - Handle the platform data
 * @x_min: touch x min
 * @y_min: touch y min
 * @x_max: touch x max
 * @y_max: touch y max
 * @pressure_min: pressure min
 * @pressure_max: pressure max
 * @x_plate_ohms: xplate terminal resistance
 * @touch_pin: touch gpio pin
 * @ext_clk: external clock flag
 * @x_flip: x flip flag
 * @y_flip: y flip flag
 * @wakeup: wakeup flag
 *
 * This is used to handle the platform data
 */
struct bu26154_platform_device {
	int x_min;
	int y_min;
	int x_max;
	int y_max;
	int pressure_min;
	int pressure_max;
	int absfuzz;
	int absflat;
	int x_plate_ohms;
	int period_time;
	int penup_timeout;
	unsigned int touch_pin;
	bool ext_clk;
	bool x_flip;
	bool y_flip;
	bool wakeup;
};
#endif
