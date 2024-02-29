#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <math.h>

#include "../src/fjalar.h"
#include "../src/sensors.h"
#include "../src/flight_state.h"
#include "../src/communication.h"
#include "../src/actuation.h"

LOG_MODULE_REGISTER(test, 5);

ZTEST_SUITE(runtime_tests, NULL, NULL, NULL, NULL, NULL);

ZTEST(runtime_tests, test_assert)
{
    int64_t start = k_uptime_get();
    int64_t delta = 0;
    while (fjalar_god.flight_state != STATE_FREE_FALL && delta < 40 * 1000) {
        k_msleep(100);
        delta = k_uptime_get() - start;
    }
    LOG_INF("Apogee detected at %lld", delta);
    zassert_between_inclusive(delta, 25 * 1000, 35 * 1000);
}


fjalar_t fjalar_god;

void test_main(void) {
	fjalar_god.sudo = false;
	printk("Started test\n");
	init_sensors(&fjalar_god);
	init_flight_state(&fjalar_god);
	init_communication(&fjalar_god);
	init_actuation(&fjalar_god);

    ztest_run_all(NULL, false, 1, 1);
}