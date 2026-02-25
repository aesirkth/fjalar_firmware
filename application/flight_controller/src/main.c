/*
This is the main script, its purpose is to:
1) Declare the structures which will be used to store information throughout the script.
2) Initiate the different scripts by starting their threads and distributing their structure pointers through the god struct (fjalar).
*/

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <math.h>

#include "fjalar.h"
#include "sensors.h"
#include "flight_state.h"
#include "com_master.h"
#include "actuation.h"
#include "init.h"
#include "aerodynamics.h"
#include "flight_state.h"
#include "control.h"
#include "com_can.h"
#include "com_lora.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static init_t            init_obj;
static control_t		 control_obj;
static can_t			 can_obj;
static lora_t			 lora_obj;

fjalar_t fjalar_god = {
	.ptr_init         = &init_obj,
	.ptr_control	  = &control_obj,
	.ptr_can	      = &can_obj,
	.ptr_lora		  = &lora_obj,
};

int main(void) {
	#ifdef CONFIG_DELAYED_START
	const int delay = 5;
	for (int i = 0; i < delay; i++) {
		printk("%d\n", delay - i);
		k_msleep(1000);
	}
	#endif
	printk("Started\n");

	fjalar_god.sudo = false;
	init_flight_state(&fjalar_god);
	init_communication(&fjalar_god);
	init_actuation(&fjalar_god);
	init_can(&fjalar_god);
	return 0;
}