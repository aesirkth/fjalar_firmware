/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aesir_dummysensor

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_DUMMYSENSOR_SKYWARD
#include "data/skyward/skyward.h"
#endif

#ifdef CONFIG_DUMMYSENSOR_ICLR
#include "data/iclr/iclr.h"
#endif

#ifdef CONFIG_DUMMYSENSOR_PICCARD
#include "data/piccard/piccard.h"
#endif
LOG_MODULE_REGISTER(dummysensor, CONFIG_SENSOR_LOG_LEVEL);

struct dummysensor_data {
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;
	float p;

	int64_t offset;

	uint32_t baro_index;
	uint32_t imu_index;
};

struct dummysensor_config {
};

static int dummysensor_sample_fetch(const struct device *dev,
				      enum sensor_channel chan)
{
	struct dummysensor_data *data = (struct dummysensor_data *) dev->data;
	float current_time = k_uptime_get() / 1000.0 - DATA_DELAY - data->offset;

	while (true) {
		if (data->baro_index >= baro_length) {
			break;
		}
		if (baro_data[data->baro_index][0] > current_time) {
			break;
		}
		data->p = baro_data[data->baro_index][1];
		data->baro_index++;
	}

	while (true) {
		if (data->imu_index >= imu_length) {
			break;
		}
		if (imu_data[data->imu_index][0] > current_time) {
			break;
		}
		data->ax = imu_data[data->imu_index][1];
		data->ay = imu_data[data->imu_index][2];
		data->az = imu_data[data->imu_index][3];

		data->gx = imu_data[data->imu_index][4];
		data->gy = imu_data[data->imu_index][5];
		data->gz = imu_data[data->imu_index][6];
		data->imu_index++;
	}
	return 0;
}

static int dummysensor_channel_get(const struct device *dev,
				     enum sensor_channel chan,
				     struct sensor_value *val)
{
	struct dummysensor_data *data = dev->data;
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

static const struct sensor_driver_api dummysensor_api = {
	.sample_fetch = &dummysensor_sample_fetch,
	.channel_get = &dummysensor_channel_get,
};

static int dummysensor_init(const struct device *dev)
{
	struct dummysensor_data *data = dev->data;
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

#define dummysensor_INIT(i)						       \
	static struct dummysensor_data dummysensor_data_##i = {.baro_index = 0, .imu_index = 0, .offset = 0};	       \
									       \
	static const struct dummysensor_config dummysensor_config_##i; \
									       \
	DEVICE_DT_INST_DEFINE(i, dummysensor_init, NULL,		       \
			      &dummysensor_data_##i,			       \
			      &dummysensor_config_##i, POST_KERNEL,	       \
			      CONFIG_SENSOR_INIT_PRIORITY, &dummysensor_api);

DT_INST_FOREACH_STATUS_OKAY(dummysensor_INIT)