/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aesir_hilsensor

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "hilsensor.h"

LOG_MODULE_REGISTER(hilsensor, CONFIG_SENSOR_LOG_LEVEL);

struct hilsensor_data {
	hil_data_t hil_data;
};

struct hilsensor_config {
};

// fetch updates the drivers internal state (driver collects IMU data for example and saves it)
// static int hilsensor_sample_fetch(const struct device *dev, enum sensor_channel chan) {return 0;}

// gets the saved data and gives it to the application
//static int hilsensor_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val){
//	struct hilsensor_data *data = dev->data;
//	switch (chan) {
//		case SENSOR_CHAN_ACCEL_X: return sensor_value_from_float(val, data->hil_data.ax);
//		case SENSOR_CHAN_ACCEL_Y: return sensor_value_from_float(val, data->hil_data.ay);
//		case SENSOR_CHAN_ACCEL_Z: return sensor_value_from_float(val, data->hil_data.az);

//		case SENSOR_CHAN_GYRO_X:  return sensor_value_from_float(val, data->hil_data.gx);
//		case SENSOR_CHAN_GYRO_Y:  return sensor_value_from_float(val, data->hil_data.gy);
//		case SENSOR_CHAN_GYRO_Z:  return sensor_value_from_float(val, data->hil_data.gz);

//		case SENSOR_CHAN_PRESS:   return sensor_value_from_float(val, data->hil_data.p);

//		default:
//			return -ENOTSUP;
//	}
//	return 0;
//}

int hilsensor_feed(const struct device *dev, hil_data_t *hil){
	struct hilsensor_data *data = dev->data;
	data->hil_data = *hil;
	return 0;
// takes in sensordata from the struct and pretends to be sensor data.
// take inspiration from dummy sensor, small difference
};

//static const struct sensor_driver_api hilsensor_api = {
//	.sample_fetch = &hilsensor_sample_fetch,
//	.channel_get = &hilsensor_channel_get,
//};

// static int hilsensor_init(const struct device *dev){return 0;}

#define hilsensor_INIT(i)						       \
	static struct hilsensor_data hilsensor_data_##i = {};	       \
									       \
	static const struct hilsensor_config hilsensor_config_##i; \
									       \
	DEVICE_DT_INST_DEFINE(i, hilsensor_init, NULL,		       \
			      &hilsensor_data_##i,			       \
			      &hilsensor_config_##i, POST_KERNEL,	       \
			      CONFIG_SENSOR_INIT_PRIORITY, &hilsensor_api);

DT_INST_FOREACH_STATUS_OKAY(hilsensor_INIT)