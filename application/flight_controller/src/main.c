/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/usb/usb_device.h>
#include <stdio.h>
#include <math.h>

#include "fjalar.h"
#include "sensors.h"
#include "flight_state.h"
#include "communication.h"
#include "actuation.h"
#include "hello.h"

LOG_MODULE_REGISTER(main, CONFIG_APP_MAIN_LOG_LEVEL);

fjalar_t fjalar_god;

int main(void) {
	#ifdef CONFIG_DELAYED_START
	const int delay = 5;
	for (int i = 0; i < delay; i++) {
		printk("%d\n", delay - i);
		k_msleep(1000);
	}
	#endif

	//usb_enable(NULL);          /* start USB-CDC */
    //printk("USB-CDC ready\n");

	printk("Started\n");
	fjalar_god.sudo = false;
	uint8_t cpp_buf[64];
	hello_from_cpp(cpp_buf, sizeof(cpp_buf));
	printk("%s", cpp_buf);
	init_sensors(&fjalar_god);
	init_flight_state(&fjalar_god);
	init_communication(&fjalar_god);
	init_actuation(&fjalar_god);
	return 0;
}