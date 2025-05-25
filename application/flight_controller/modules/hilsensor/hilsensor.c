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

LOG_MODULE_REGISTER(hilsensor, CONFIG_SENSOR_LOG_LEVEL);

struct hilsensor_data {

};

struct hilsensor_config {
};

// fetch updates the drivers internal state (driver collects IMU data for example and saves it)
static int hilsensor_sample_fetch(const struct device *dev,
				      enum sensor_channel chan)
{
}

// gets the saved data and gives it to the application
static int hilsensor_channel_get(const struct device *dev,
				     enum sensor_channel chan,
				     struct sensor_value *val)
{
	struct hilsensor_data *data = dev->data;
	switch (chan) {
		case SENSOR_CHAN_ACCEL_X:
			return sensor_value_from_float(val, data->ax);
		case SENSOR_CHAN_ACCEL_Y:
			return sensor_value_from_float(val, data->ay);
		case SENSOR_CHAN_ACCEL_Z:
			return sensor_value_from_float(val, data->az);
		case SENSOR_CHAN_GYRO_X:
			return sensor_value_from_float(val, data->gx);
		case SENSOR_CHAN_GYRO_Y:
			return sensor_value_from_float(val, data->gy);
		case SENSOR_CHAN_GYRO_Z:
			return sensor_value_from_float(val, data->gz);
		case SENSOR_CHAN_PRESS:
			return sensor_value_from_float(val, data->p);
		case SENSOR_CHAN_PRIV_START:
			data->offset = k_uptime_get();
			data->baro_index = 0;
			data->imu_index = 0;
			break;
		default:
			return -ENOTSUP;
	}
	return 0;
}

static int hilsensor_feed(const struct device *dev, hil_data_t *hil){

// takes in sensordata from the struct and pretends to be sensor data.
// take inspiration from dummy sensor, small difference
};

static const struct sensor_driver_api hilsensor_api = {
	.sample_fetch = &hilsensor_sample_fetch,
	.channel_get = &hilsensor_channel_get,
};

static int hilsensor_init(const struct device *dev)
{
	struct hilsensor_data *data = dev->data;
	data->ax = imu_data[0][1];
	data->ay = imu_data[0][2];
	data->az = imu_data[0][3];
	data->gx = imu_data[0][4];
	data->gy = imu_data[0][5];
	data->gz = imu_data[0][6];
	data->p = baro_data[0][1];
	data->imu_index = 0;
	data->baro_index = 0;
	return 0;
}

#define hilsensor_INIT(i)						       \
	static struct hilsensor_data hilsensor_data_##i = {.baro_index = 0, .imu_index = 0, .offset = 0};	       \
									       \
	static const struct hilsensor_config hilsensor_config_##i; \
									       \
	DEVICE_DT_INST_DEFINE(i, hilsensor_init, NULL,		       \
			      &hilsensor_data_##i,			       \
			      &hilsensor_config_##i, POST_KERNEL,	       \
			      CONFIG_SENSOR_INIT_PRIORITY, &hilsensor_api);

DT_INST_FOREACH_STATUS_OKAY(hilsensor_INIT)