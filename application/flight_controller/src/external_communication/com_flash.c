#include <zephyr/kernel.h>
#include <protocol.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/zbus/zbus.h>

#include "init.h"
#include "filter.h"
#include "aerodynamics.h"
#include "flight_state.h"
#include "control.h"
#include "com_can.h"
#include "com_master.h"
#include "com_flash.h"



#define FLASH_THREAD_PRIORITY 7
#define FLASH_THREAD_STACK_SIZE 2048
#define FLASH_MSG_ENQUEUE_THREAD_PRIORITY 7
#define FLASH_MSG_ENQUEUE_THREAD_STACK_SIZE 2048

LOG_MODULE_REGISTER(com_flash, LOG_LEVEL_INF);

K_MUTEX_DEFINE(flash_mutex);

K_THREAD_STACK_DEFINE(flash_thread_stack, FLASH_THREAD_STACK_SIZE);
struct k_thread flash_thread_data;
k_tid_t flash_thread_id;
K_THREAD_STACK_DEFINE(flash_msg_enqueue_thread_stack, FLASH_MSG_ENQUEUE_THREAD_STACK_SIZE);
struct k_thread flash_msg_enqueue_thread_data;
k_tid_t flash_msg_enqueue_thread_id;

void flash_thread(fjalar_t *fjalar, void *p2, void *p3);
void flash_msg_enqueue_thread(fjalar_t *fjalar, void *p2, void *p3);

void init_com_flash(fjalar_t *fjalar){
    flash_thread_id = k_thread_create(
        &flash_thread_data,
        flash_thread_stack,
        K_THREAD_STACK_SIZEOF(flash_thread_stack),
        (k_thread_entry_t) flash_thread,
        fjalar, NULL, NULL,
        FLASH_THREAD_PRIORITY, 0, K_NO_WAIT
    );
    k_thread_name_set(flash_thread_id, "flash");

	flash_msg_enqueue_thread_id = k_thread_create(
		&flash_msg_enqueue_thread_data,
		flash_msg_enqueue_thread_stack,
		K_THREAD_STACK_SIZEOF(flash_msg_enqueue_thread_stack),
		(k_thread_entry_t) flash_msg_enqueue_thread,
		fjalar, NULL, NULL,
		FLASH_MSG_ENQUEUE_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(flash_thread_id, "flash enqueue");
}

K_MSGQ_DEFINE(flash_msgq, sizeof(struct padded_buf), 32, 4);

void flash_thread(fjalar_t *fjalar, void *p2, void *p3) {
	const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(data_flash));
	if (!device_is_ready(flash_dev)) {
		LOG_ERR("Flash not ready");
		return;
	}
	const uint8_t flash_reset_value = 0xff;
	fjalar->flash_address = 0;
	fjalar->flash_size = DT_PROP(DT_ALIAS(data_flash), size) / 8;
	int chunk_size = 1024;
	int num_cleared_values = 0;
	int chunk_start = 0;
	int ret;
	uint8_t buf[chunk_size];
	k_mutex_lock(&flash_mutex, K_FOREVER);
	while (true) {
		chunk_size = MIN(chunk_size, fjalar->flash_size - fjalar->flash_address);
		if (chunk_size < 1) {
			goto end;
		}
		ret = flash_read(flash_dev, fjalar->flash_address, buf, chunk_size);
		if (ret != 0) {
			LOG_ERR("Flash startup read fail");
			return;
		}
		k_usleep(10);
		chunk_start = fjalar->flash_address;
		fjalar->flash_address = fjalar->flash_address + chunk_size;
		for (int i = 0; i < chunk_size; i++) {
			if (buf[i] == flash_reset_value) {
				num_cleared_values += 1;
			} else {
				num_cleared_values = 0;
			}
			if (num_cleared_values == chunk_size) {
				fjalar->flash_address = chunk_start + i - num_cleared_values + 1;
				LOG_INF("found good address at %d", fjalar->flash_address);
				goto end;
			}
		}

	}
	end:
	k_mutex_unlock(&flash_mutex);
	LOG_WRN("initialized flash index to %d", fjalar->flash_address);
	k_msleep(10);
	while (true) {
		int ret;
		struct padded_buf pbuf;
		ret = k_msgq_get(&flash_msgq, &pbuf, K_FOREVER);
		if (ret == 0) {
			int size = get_encoded_message_length(pbuf.buf);
			k_mutex_lock(&flash_mutex, K_FOREVER);
			if (fjalar->flash_address + size >= fjalar->flash_size) {
				k_mutex_unlock(&flash_mutex);
				continue;
			}
			//LOG_INF("wrote to flash address %d", fjalar->flash_address);
			int ret = flash_write(flash_dev, fjalar->flash_address, pbuf.buf, size);
			if (ret) {
				LOG_ERR("Could not write to flash");
			} else {
				fjalar->flash_address += size;
			}
			k_mutex_unlock(&flash_mutex);
		} else {
			LOG_ERR("flash msgq error");
		}
	}
}

void flash_msg_enqueue(fjalar_message_t *msg){
	struct padded_buf pbuf;
	int size = encode_fjalar_message(msg, pbuf.buf);
	if (size < 0) {
		LOG_ERR("encode_fjalar_message failed");
		return;
	}
	LOG_DBG("sending message w/ size %d", size);
	if (k_msgq_put(&flash_msgq, &pbuf, K_NO_WAIT)){
		//LOG_ERR("could not insert into flash msgq");
	}
}

void flash_msg_enqueue_thread(fjalar_t *fjalar, void *p2, void *p3){
    can_t             *can = fjalar->ptr_can;
	
	while (true){
		struct filter_output_msg filter_msg;
		int rer = zbus_chan_read(&filter_output_zchan, &filter_msg, K_NO_WAIT);
		if (rer == 0) {
			// imu
			fjalar_message_t msg_imu = {
				.time = k_uptime_get_32(),
				.has_data = true,
				.data = {
					.which_data = FJALAR_DATA_IMU_READING_TAG,
					.data.imu_reading = {
						.ax = filter_msg.raw_imu[0],
						.ay = filter_msg.raw_imu[1],
						.az = filter_msg.raw_imu[2],
						.gx = filter_msg.raw_imu[3],
						.gy = filter_msg.raw_imu[4],
						.gz = filter_msg.raw_imu[5],
					},
				},
			};
			flash_msg_enqueue(&msg_imu);

			// baro
			fjalar_message_t msg_baro = {
				.time = k_uptime_get_32(),
				.has_data = true,
				.data = {
					.which_data = FJALAR_DATA_PRESSURE_READING_TAG,
					.data.pressure_reading = {
						.pressure = filter_msg.raw_baro_p
					},
				},
			};
			flash_msg_enqueue(&msg_baro);

			// gps
			fjalar_message_t msg_gps = {
				.time = k_uptime_get_32(),
				.has_data = true,
				.data = {
					.which_data = FJALAR_DATA_GNSS_POSITION_TAG,
					.data.gnss_position = {
						.latitude = filter_msg.raw_gps[0],
						.longitude = filter_msg.raw_gps[1],
						.altitude = filter_msg.raw_gps[2],
					},
				},
			};
			flash_msg_enqueue(&msg_gps);

			// StateEstimate
			fjalar_message_t msg_state_estimate = {
				.time = k_uptime_get_32(),
				.has_data = true,
				.data = {
					.which_data = FJALAR_DATA_STATE_ESTIMATE_TAG,
					.data.state_estimate = {
						.x     = filter_msg.position[0],
						.y     = filter_msg.position[1],
						.z     = filter_msg.position[2],
						.vx    = filter_msg.velocity[0],
						.vy    = filter_msg.velocity[1],
						.vz    = filter_msg.velocity[2],
						.ax    = filter_msg.acceleration[0],
						.ay    = filter_msg.acceleration[1],
						.az    = filter_msg.acceleration[2],

						.roll  = filter_msg.attitude[0],
						.pitch = filter_msg.attitude[1],
						.yaw   = filter_msg.attitude[2],
					},
				},
			};
			flash_msg_enqueue(&msg_state_estimate);
		}
		struct flight_state_output_msg fs_msg;
		int ret = zbus_chan_read(&flight_state_output_zchan, &fs_msg, K_NO_WAIT);

		if (ret == 0) {
			// Flight State
			fjalar_message_t msg_flight_state = {
				.time = fs_msg.timestamp,
				.has_data = true,
				.data = {
					.which_data = FJALAR_DATA_FLIGHT_STATE_TAG,
					.data.flight_state = fs_msg.flight_state
				},
			};
			flash_msg_enqueue(&msg_flight_state);
			// Flight Event
			fjalar_message_t msg_flight_event = {
				.time = fs_msg.timestamp,
				.has_data = true,
				.data = {
					.which_data = FJALAR_DATA_FLIGHT_EVENT_TAG,
					.data.flight_event = {
						.launch              = fs_msg.event_launch,
						.burnout             = fs_msg.event_burnout,
						.apogee              = fs_msg.event_apogee,
						.drogue_deploy       = fs_msg.event_drogue_deployed,
						.main_deploy         = fs_msg.event_main_deployed,
						.landed              = fs_msg.event_landed,
						.above_acs_threshold = fs_msg.event_above_acs_threshold,
					},
				},
			};
			flash_msg_enqueue(&msg_flight_event);
		}

		// CANBus
		fjalar_message_t msg_can_bus = {
			.time = k_uptime_get_32(),
			.has_data = true,
			.data = {
				.which_data = FJALAR_DATA_CAN_BUS_TAG,
				.data.can_bus = {
					.can_started           = can->can_started,
					.loki_latest_rx_time   = can->loki_latest_rx_time,
					.loki_latest_tx_time   = can->loki_latest_tx_time,
					//.sigurd_latest_rx_time = sigurd_context->sigurd_latest_rx_time,
					//.sigurd_latest_tx_time = can->loki_latest_tx_time,
				},
			},
		};
		flash_msg_enqueue(&msg_can_bus);
		
		// PyroStatus
		fjalar_message_t msg_pyro_status = {
			.time = k_uptime_get_32(),
			.has_data = true,
			.data = {
				.which_data = FJALAR_DATA_PYRO_STATUS_TAG,
				.data.pyro_status = {
					.pyro1_connected = fjalar->pyro1_sense,
					.pyro2_connected = fjalar->pyro2_sense,
					.pyro3_connected = fjalar->pyro3_sense,
				},
			},
		};
		flash_msg_enqueue(&msg_pyro_status);

		// LokiToFjalar
		fjalar_message_t msg_loki = {
			.time = k_uptime_get_32(),
			.has_data = true,
			.data = {
				.which_data         = FJALAR_DATA_CAN_LOKI_TO_FJALAR_TAG,
				.data.can_loki_to_fjalar = {
					.loki_state           = can->loki_state,
					.loki_substate        = can->loki_sub_state,
					.loki_current_angle   = can->loki_angle,
					.loki_battery_voltage = can->loki_battery_voltage,
				},
			},
		};
		flash_msg_enqueue(&msg_loki);
		

		k_msleep(1000);
	}
}

