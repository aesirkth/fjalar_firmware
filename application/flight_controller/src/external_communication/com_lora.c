#include <zephyr/kernel.h>
#include <protocol.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/zbus/zbus.h>

#include "init.h"
#include "filter.h"
#include "aerodynamics.h"
#include "flight_state.h"
#include "control.h"
#include "com_can.h"
#include "com_master.h"
#include "com_lora.h"

K_MSGQ_DEFINE(lora_msgq, sizeof(struct padded_buf), 5, 4);

#define LORA_THREAD_PRIORITY 7
#define LORA_THREAD_STACK_SIZE 2048
#define LORA_MSG_ENQUEUE_THREAD_PRIORITY 7
#define LORA_MSG_ENQUEUE_THREAD_STACK_SIZE 2048

#define LORA_TRANSMIT true
#define LORA_RECEIVE false

LOG_MODULE_REGISTER(com_lora, LOG_LEVEL_INF);

K_THREAD_STACK_DEFINE(lora_thread_stack, LORA_THREAD_STACK_SIZE);
struct k_thread lora_thread_data;
k_tid_t lora_thread_id;
K_THREAD_STACK_DEFINE(lora_msg_enqueue_thread_stack, LORA_MSG_ENQUEUE_THREAD_STACK_SIZE);
struct k_thread lora_msg_enqueue_thread_data;
k_tid_t lora_msg_enqueue_thread_id;

void lora_thread(fjalar_t *fjalar, void *p2, void *p3);
void lora_msg_enqueue_thread(fjalar_t *fjalar, void *p2, void *p3);

void init_com_lora(fjalar_t *fjalar){
    lora_thread_id = k_thread_create(
        &lora_thread_data,
        lora_thread_stack,
        K_THREAD_STACK_SIZEOF(lora_thread_stack),
        (k_thread_entry_t) lora_thread,
        fjalar, NULL, NULL,
        LORA_THREAD_PRIORITY, 0, K_NO_WAIT
    );
    k_thread_name_set(lora_thread_id, "lora");

    lora_msg_enqueue_thread_id = k_thread_create(
        &lora_msg_enqueue_thread_data,
        lora_msg_enqueue_thread_stack,
        K_THREAD_STACK_SIZEOF(lora_msg_enqueue_thread_stack),
        (k_thread_entry_t) lora_msg_enqueue_thread,
        fjalar, NULL, NULL,
        LORA_MSG_ENQUEUE_THREAD_PRIORITY, 0, K_NO_WAIT
    );
    k_thread_name_set(lora_msg_enqueue_thread_id, "lora enqueue");
} 

int lora_configure(const struct device *dev, bool transmit) {
	static int8_t current_mode = -1;
	if (current_mode == transmit) {
		LOG_DBG("Mode already configured");
		return 0;
	}
	struct lora_modem_config config = PROTOCOL_ZEPHYR_LORA_CONFIG;
	config.tx = transmit;
	config.tx_power = 22;
	int ret = lora_config(dev, &config);
	if (ret < 0) {
		LOG_ERR("Could not configure lora %d", ret);
		return ret;
	}
	if (transmit) {
		LOG_DBG("LoRa configured to tx");
	} else {
		LOG_DBG("LoRa configured to rx");
	}
	current_mode = transmit;
	return ret;
}

struct lora_rx {
	uint8_t buf[PROTOCOL_BUFFER_LENGTH];
	uint16_t size;
	int16_t rssi;
	int8_t snr;
} __attribute__((aligned(4)));

K_MSGQ_DEFINE(lora_rx_msgq, sizeof(struct lora_rx), 5, 4);

void lora_cb(const struct device *dev, uint8_t *buf, uint16_t size, int16_t rssi, int8_t snr, void *user) {
	struct lora_rx rx;
	rx.size = MIN(size, PROTOCOL_BUFFER_LENGTH);
	memcpy(rx.buf, buf, rx.size);
	rx.rssi = rssi;
	rx.snr = snr;
	k_msgq_put(&lora_rx_msgq, &rx, K_NO_WAIT);
}

void lora_thread(fjalar_t *fjalar, void* p2, void* p3) {
	const struct device *lora_dev = DEVICE_DT_GET(DT_ALIAS(lora));
	if (!device_is_ready(lora_dev)) {
		LOG_ERR("LoRa is not ready");
		return;
	}
	struct lora_rx rx;
	while (true) {
		int ret;
		struct padded_buf pbuf;
		struct k_poll_event events[2] = {
			K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
											K_POLL_MODE_NOTIFY_ONLY,
											&lora_msgq),
			K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
											K_POLL_MODE_NOTIFY_ONLY,
											&lora_rx_msgq),
		};

		ret = lora_configure(lora_dev, LORA_RECEIVE);
		if (ret) {
			LOG_ERR("LoRa rx configure failed");
		} else {
			LOG_DBG("LoRa rxing");
		}
		// TODO: The void user data was recently added, use this to pass data instead of the global variable
		lora_recv_async(lora_dev, lora_cb, NULL);
		// k_poll(&events[1], 2, K_MSEC(10000)); //poll only rx first to not interrupt messages
		k_poll(events, 2, K_FOREVER);

		ret = k_msgq_get(&lora_rx_msgq, &rx, K_NO_WAIT);
		if (ret == 0) {
			events[1].state = K_POLL_STATE_NOT_READY;
			LOG_INF("received LoRa msg");
			struct protocol_state ps;
			reset_protocol_state(&ps);
			handle_fjalar_buf(&ps, fjalar, rx.buf, rx.size, COM_CHAN_LORA);
		}

		ret = k_msgq_get(&lora_msgq, &pbuf, K_NO_WAIT);
		if (ret == 0) {
			events[0].state = K_POLL_STATE_NOT_READY;
			lora_recv_async(lora_dev, NULL, NULL);
			ret = lora_configure(lora_dev, LORA_TRANSMIT);
			if (ret) {
				LOG_ERR("LORA tx configure failed");
			}else {
				LOG_DBG("LoRa txing");
			}
			int size = get_encoded_message_length(pbuf.buf);
			ret = lora_send(lora_dev, pbuf.buf, size);
			if (ret) {
				LOG_ERR("Could not send LoRa message");
			} else {
				LOG_DBG("Sent lora packet with size %d", size);
			}
		}
	}
}

void lora_msg_enqueue(fjalar_message_t *msg){
	struct padded_buf pbuf;
	int size = encode_fjalar_message(msg, pbuf.buf);
	if (size < 0) {
		LOG_ERR("encode_fjalar_message failed");
		return;
	}
	LOG_DBG("sending message w/ size %d", size);
	if (k_msgq_put(&lora_msgq, &pbuf, K_NO_WAIT)){LOG_ERR("could not insert into lora msgq");}
}

void lora_msg_enqueue_thread(fjalar_t *fjalar, void *p2, void *p3){
	//lora->LORA_READY_INITIATE_FJALAR = true; // for testing without tracker DO NOT FORGET TO REMOVE
	//lora->LORA_READY_LAUNCH_FJALAR = true;

	while (true){
		struct filter_output_msg filter_msg;
		struct flight_state_output_msg fs_msg;

		int ret1 = zbus_chan_read(&filter_output_zchan, &filter_msg, K_NO_WAIT);
		int ret2 = zbus_chan_read(&flight_state_output_zchan, &fs_msg, K_NO_WAIT);
		if (ret1 == 0 && ret2 == 0) {
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
			lora_msg_enqueue(&msg_gps);

			// FlightState
			fjalar_message_t msg_flight_state = {
				.time = k_uptime_get_32(),
				.has_data = true,
				.data = {
					.which_data = FJALAR_DATA_FLIGHT_STATE_TAG,
					.data.flight_state = fs_msg.flight_state
				},
			};
			lora_msg_enqueue(&msg_flight_state);
		}
		k_msleep(5000);
	}
}