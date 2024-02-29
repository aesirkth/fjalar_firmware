#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/zbus/zbus.h>

#include <minmea.h>

#include "fjalar.h"
#include "sensors.h"
#include "communication.h"

LOG_MODULE_REGISTER(sensors, CONFIG_APP_SENSORS_LOG_LEVEL);

#define IMU_THREAD_PRIORITY 7
#define IMU_THREAD_STACK_SIZE 2048

#define BAROMETER_THREAD_PRIORITY 7
#define BAROMETER_THREAD_STACK_SIZE 2048

#define VBAT_THREAD_PRIORITY 7
#define VBAT_THREAD_STACK_SIZE 2048

#define GPS_THREAD_PRIORITY 7
#define GPS_THREAD_STACK_SIZE 2048

void barometer_thread(fjalar_t *fjalar, void *p2, void *p3);
void imu_thread(fjalar_t *fjalar, void *p2, void *p3);
void gps_thread(fjalar_t *fjalar, void *p2, void *p3);
void vbat_thread(fjalar_t *fjalar, void *p2, void *p3);

ZBUS_CHAN_DEFINE(pressure_zchan, /* Name */
		struct pressure_queue_entry, /* Message type */
		NULL, /* Validator */
		NULL, /* User Data */
		ZBUS_OBSERVERS_EMPTY, /* observers */
		ZBUS_MSG_INIT(.t = 0, .pressure = 0) /* Initial value */
);

ZBUS_CHAN_DEFINE(imu_zchan, /* Name */
		struct imu_queue_entry, /* Message type */
		NULL, /* Validator */
		NULL, /* User Data */
		ZBUS_OBSERVERS_EMPTY, /* observers */
		ZBUS_MSG_INIT() /* Initial value */
);

K_MSGQ_DEFINE(pressure_msgq, sizeof(struct pressure_queue_entry), 3, 4);
K_MSGQ_DEFINE(imu_msgq, sizeof(struct imu_queue_entry), 3, 4);

K_THREAD_STACK_DEFINE(barometer_thread_stack, BAROMETER_THREAD_STACK_SIZE);
struct k_thread barometer_thread_data;
k_tid_t barometer_thread_id;

K_THREAD_STACK_DEFINE(imu_thread_stack, IMU_THREAD_STACK_SIZE);
struct k_thread imu_thread_data;
k_tid_t imu_thread_id;

K_THREAD_STACK_DEFINE(vbat_thread_stack, VBAT_THREAD_STACK_SIZE);
struct k_thread vbat_thread_data;
k_tid_t vbat_thread_id;

#if DT_ALIAS_EXISTS(gps_uart)
K_THREAD_STACK_DEFINE(gps_thread_stack, GPS_THREAD_STACK_SIZE);
struct k_thread gps_thread_data;
k_tid_t gps_thread_id;
#endif

void init_sensors(fjalar_t *fjalar) {
	barometer_thread_id = k_thread_create(
		&barometer_thread_data,
		barometer_thread_stack,
		K_THREAD_STACK_SIZEOF(barometer_thread_stack),
		(k_thread_entry_t) barometer_thread,
		fjalar, NULL, NULL,
		BAROMETER_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(barometer_thread_id, "barometer");

	imu_thread_id = k_thread_create(
		&imu_thread_data,
		imu_thread_stack,
		K_THREAD_STACK_SIZEOF(imu_thread_stack),
		(k_thread_entry_t) imu_thread,
		fjalar, NULL, NULL,
		IMU_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(imu_thread_id, "imu");

	#if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
	vbat_thread_id = k_thread_create(
		&vbat_thread_data,
		vbat_thread_stack,
		K_THREAD_STACK_SIZEOF(vbat_thread_stack),
		(k_thread_entry_t) vbat_thread,
		fjalar, NULL, NULL,
		VBAT_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(vbat_thread_id, "bat_adc");
	#endif

	#if DT_ALIAS_EXISTS(gps_uart)
	gps_thread_id = k_thread_create(
		&gps_thread_data,
		gps_thread_stack,
		K_THREAD_STACK_SIZEOF(gps_thread_stack),
		(k_thread_entry_t) gps_thread,
		fjalar, NULL, NULL,
		GPS_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(gps_thread_id, "gps");
	#endif
}

void imu_thread(fjalar_t *fjalar, void *p2, void *p3) {
    const struct device *const imu_dev = DEVICE_DT_GET(DT_ALIAS(imu));
    if (!device_is_ready(imu_dev)) {
		LOG_ERR("imu is not ready");
		return;
	}
	while (true) {
		int ret = 0;
		k_msleep(10);
		ret = sensor_sample_fetch(imu_dev);
		if (ret != 0) {
			LOG_ERR("Could not fetch imu sample");
			continue;
		}

		struct sensor_value ax;
		struct sensor_value ay;
		struct sensor_value az;

		struct sensor_value gx;
		struct sensor_value gy;
		struct sensor_value gz;

		ret = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &ax);
		ret = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &ay);
		ret = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &az);

		ret = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gx);
		ret = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gy);
		ret = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gz);

		if (ret != 0) {
			LOG_ERR("Could get imu values");
			continue;
		}

		LOG_DBG("read imu: %f %f %f %f %f %f",
			sensor_value_to_float(&ax), sensor_value_to_float(&ay), sensor_value_to_float(&az),
			sensor_value_to_float(&gx), sensor_value_to_float(&gy), sensor_value_to_float(&gz)
		);

		struct imu_queue_entry q_entry = {
			.t = k_uptime_get_32(),
			.ax = sensor_value_to_float(&ax),
			.ay = sensor_value_to_float(&ay),
			.az = sensor_value_to_float(&az),
			.gx = sensor_value_to_float(&gx),
			.gy = sensor_value_to_float(&gy),
			.gz = sensor_value_to_float(&gz)
		};
		ret = k_msgq_put(&imu_msgq, &q_entry, K_NO_WAIT);
		if (ret != 0) {
			LOG_ERR("Could not write to imu msgq");
			continue;
		}

		fjalar_message_t msg;
		msg.time = k_uptime_get_32();
		msg.has_data = true;
		msg.data.which_data = FJALAR_DATA_IMU_READING_TAG;
		msg.data.data.imu_reading.ax = sensor_value_to_float(&ax);
		msg.data.data.imu_reading.ay = sensor_value_to_float(&ay);
		msg.data.data.imu_reading.az = sensor_value_to_float(&az);
		msg.data.data.imu_reading.gx = sensor_value_to_float(&gx);
		msg.data.data.imu_reading.gy = sensor_value_to_float(&gy);
		msg.data.data.imu_reading.gz = sensor_value_to_float(&gz);
		send_message(fjalar, &msg, MSG_PRIO_LOW);
	}
}

void barometer_thread(fjalar_t *fjalar, void *p2, void *p3) {
    const struct device *const baro_dev = DEVICE_DT_GET(DT_ALIAS(barometer));
    if (!device_is_ready(baro_dev)) {
		LOG_ERR("barometer is not ready");
		return;
	}
	int ret;
	struct sensor_value osr;
	osr.val1 = 2048;
	osr.val2 = 0;
	ret = sensor_attr_set(baro_dev, SENSOR_CHAN_ALL, SENSOR_ATTR_OVERSAMPLING, &osr);
	if (ret != 0) {
		LOG_ERR("Could not set barometer oversample");
	}
	while (true) {
		int ret = 0;
		k_msleep(10);
		ret = sensor_sample_fetch(baro_dev);
		if (ret != 0) {
			LOG_ERR("Could not fetch barometer sample");
			continue;
		}
		struct sensor_value pressure;
		ret = sensor_channel_get(baro_dev, SENSOR_CHAN_PRESS, &pressure);
		if (ret != 0) {
			LOG_ERR("Could not get barometer pressure");
			continue;
		}
		LOG_DBG("read pressure: %f", sensor_value_to_float(&pressure));
		struct pressure_queue_entry q_entry;
		q_entry.t = k_uptime_get_32();
		q_entry.pressure = sensor_value_to_float(&pressure);
		ret = k_msgq_put(&pressure_msgq, &q_entry, K_NO_WAIT);
		if (ret != 0) {
			LOG_ERR("Could not write to pressure msgq");
		}
		// ret = zbus_chan_pub(&pressure_zchan, &q_entry, K_MSEC(100));
		if (ret != 0) {
			LOG_ERR("Could not publish pressure to zbus");
		}
		fjalar_message_t msg;
		msg.time = k_uptime_get_32();
		msg.has_data = true;
		msg.data.which_data = FJALAR_DATA_PRESSURE_READING_TAG;
		msg.data.data.pressure_reading.pressure = sensor_value_to_float(&pressure);;
		send_message(fjalar, &msg, MSG_PRIO_LOW);
	}
}

#if DT_NODE_EXISTS(DT_ALIAS(gps_uart))
void handle_nmea(fjalar_t *fjalar, char *buf, int len) {
	LOG_DBG("Handling nmea message");
	switch (minmea_sentence_id(buf, true)) {
		case MINMEA_INVALID:
			LOG_ERR("got invalid nmea message \"%.*s\"", len, buf);
			break;
		case MINMEA_UNKNOWN:
			LOG_DBG("got unknown nmea message (quectel probably) \"%.*s\"", len, buf);
			break;

		case MINMEA_SENTENCE_GSA:
			LOG_DBG("got GSA nmea message");
			struct minmea_sentence_gsa gsa;
			minmea_parse_gsa(&gsa, buf);
			break;
		case MINMEA_SENTENCE_GSV:
			LOG_DBG("got GSV nmea message");
			struct minmea_sentence_gsv gsv;
			minmea_parse_gsv(&gsv, buf);
			break;

		case MINMEA_SENTENCE_GGA:
			LOG_DBG("got GGA nmea message");
			struct minmea_sentence_gga gga;
			minmea_parse_gga(&gga, buf);
			fjalar->longitude = minmea_tocoord(&gga.longitude);
			fjalar->latitude = minmea_tocoord(&gga.latitude);
			break;

		case MINMEA_SENTENCE_RMC:
			LOG_DBG("got RMC nmea message");
			struct minmea_sentence_rmc rmc;
			minmea_parse_rmc(&rmc, buf);
			fjalar->longitude = minmea_tocoord(&rmc.longitude);
			fjalar->latitude = minmea_tocoord(&rmc.latitude);
			break;

		default:
			LOG_DBG("got unhandled nmea message \"%.*s\"", len, buf);
			break;
	}
}

void gps_uart_cb(const struct device *dev, void *user_data) {
	static uint8_t nmea_buf[255];
	static int nmea_index = 0;

	if (!uart_irq_update(dev)) {
		return;
	}

	if (!uart_irq_rx_ready(dev)) {
		return;
	}

	while (true) {
		uint8_t fifo_buf[255];
		int fifo_len = uart_fifo_read(dev, fifo_buf, sizeof(fifo_buf));
		if (fifo_len <= 0) {
			break;
		}

		for (int i = 0; i < fifo_len; i++) {
			char byte = fifo_buf[i];
			if (byte == '$') {
				nmea_index = 0;
			}
			if (byte == '\n' || byte == '\r') {
				if (nmea_index != 0) {
					nmea_buf[nmea_index] = '\0';
					handle_nmea(&fjalar_god, nmea_buf, nmea_index); // TODO: don't use fjalar god
				}
				nmea_index = 0;
				continue;
			}
			if (nmea_index < sizeof(nmea_buf) - 1) {
				nmea_buf[nmea_index] = byte;
				nmea_index++;
				continue;
			}
			continue;
		}
	}
}

void gps_thread(fjalar_t *fjalar, void *p2, void *p3) {
    const struct device *const gps_dev = DEVICE_DT_GET(DT_ALIAS(gps_uart));
    if (!device_is_ready(gps_dev)) {
		LOG_ERR("gps is not ready");
		return;
	}
    int ret;
    struct uart_config uart_config = {
		.baudrate = 9600,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1
	};
	ret = uart_configure(gps_dev, &uart_config);
	uart_irq_callback_set(gps_dev, gps_uart_cb);
	uart_irq_rx_enable(gps_dev);

	const char pulse_msg[] = "$PMTK285,2,100*3E\r\n"; // blink
	for (int i = 0; i < strlen(pulse_msg); i++) {
		uart_poll_out(gps_dev, pulse_msg[i]);
	}

	const char mode_msg[] = "$PMTK886,3*2B\r\n"; // balloon mode
	for (int i = 0; i < strlen(mode_msg); i++) {
		uart_poll_out(gps_dev, mode_msg[i]);
	}
}
#endif

//WTF IS THIS I JUST WANT TO READ AN ADC CHANNEL
#if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
void vbat_thread(fjalar_t *fjalar, void *p2, void *p3) {
	#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),
	static const struct adc_dt_spec adc_channels[] = {
		DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
					DT_SPEC_AND_COMMA)
	};
	int err;

	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!device_is_ready(adc_channels[i].dev)) {
			LOG_ERR("ADC controller device %s not ready\n", adc_channels[i].dev->name);
			return;
		}
		err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0) {
			LOG_ERR("Could not setup channel #%d (%d)\n", i, err);
			return;
		}
	}

	while (true) {
		k_msleep(100);
		uint16_t sample;
		struct adc_sequence sequence;
		struct adc_sequence_options options;
		options.interval_us = 0;
		options.extra_samplings = 0;
		options.callback = NULL;
		sequence.buffer = &sample;
		sequence.buffer_size = sizeof(sample);
		sequence.options = &options;
		err = adc_sequence_init_dt(&adc_channels[0], &sequence);
		if (err) {
			LOG_ERR("Could not init adc channel");
			continue;
		}
		err = adc_read(adc_channels[0].dev, &sequence);
		if (err) {
			LOG_ERR("Could not read adc channel");
			continue;
		}
		uint32_t mv = sample;
		float volt;
		const float numerator = DT_PROP(DT_PATH(zephyr_user), battery_numerator);
		const float denominator = DT_PROP(DT_PATH(zephyr_user), battery_denominator);
		err = adc_raw_to_millivolts_dt(&adc_channels[0],
						       &mv);
		if (err) {
			LOG_ERR("Could not convert adc raw");
		}
		volt = (mv / 1000.0) * numerator / denominator;
		fjalar->battery_voltage = volt;
		LOG_DBG("battery voltage raw: %f scaled: %f", mv / 1000.0, volt);
	}
}
#endif