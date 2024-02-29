#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/console/tty.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/gpio.h>

#include <protocol.h>

#include "fjalar.h"
#include "commands.h"

LOG_MODULE_REGISTER(communication, CONFIG_APP_COMMUNICATION_LOG_LEVEL);

#define LORA_THREAD_PRIORITY 7
#define LORA_THREAD_STACK_SIZE 2048

#define FLASH_THREAD_PRIORITY 7
#define FLASH_THREAD_STACK_SIZE 2048

#define UART_THREAD_PRIORITY 7
#define UART_THREAD_STACK_SIZE 2048

#define USB_THREAD_PRIORITY 7
#define USB_THREAD_STACK_SIZE 2048

#define SAMPLER_THREAD_PRIORITY 7
#define SAMPLER_THREAD_STACK_SIZE 2048

#define LORA_TRANSMIT true
#define LORA_RECEIVE false

volatile bool terminate_communication = false;

K_THREAD_STACK_DEFINE(sampler_thread_stack, SAMPLER_THREAD_STACK_SIZE);
struct k_thread sampler_thread_data;
k_tid_t sampler_thread_id;
void sampler_thread(fjalar_t *fjalar, void *p2, void *p3);

#if DT_ALIAS_EXISTS(lora)
K_THREAD_STACK_DEFINE(lora_thread_stack, LORA_THREAD_STACK_SIZE);
struct k_thread lora_thread_data;
k_tid_t lora_thread_id;
void lora_thread(fjalar_t *fjalar, void *p2, void *p3);
#endif

#if DT_ALIAS_EXISTS(external_uart)
K_THREAD_STACK_DEFINE(uart_thread_stack, UART_THREAD_STACK_SIZE);
struct k_thread uart_thread_data;
k_tid_t uart_thread_id;
void uart_thread(fjalar_t *fjalar, void *p2, void *p3);
#endif

#if DT_ALIAS_EXISTS(data_usb)
K_THREAD_STACK_DEFINE(usb_thread_stack, USB_THREAD_STACK_SIZE);
struct k_thread usb_thread_data;
k_tid_t usb_thread_id;
void usb_thread(fjalar_t *fjalar, void *p2, void *p3);
#endif

K_MUTEX_DEFINE(flash_mutex);
#if DT_ALIAS_EXISTS(data_flash)
K_THREAD_STACK_DEFINE(flash_thread_stack, FLASH_THREAD_STACK_SIZE);
struct k_thread flash_thread_data;
k_tid_t flash_thread_id;
void flash_thread(fjalar_t *fjalar, void *p2, void *p3);
#endif

struct padded_buf {
	uint8_t buf[PROTOCOL_BUFFER_LENGTH];
} __attribute__((aligned(4)));

_Static_assert(sizeof(struct padded_buf) % 4 == 0, "padded buffer length is not aligned");

K_MSGQ_DEFINE(uart_msgq, sizeof(struct padded_buf), 32, 4);
K_MSGQ_DEFINE(flash_msgq, sizeof(struct padded_buf), 32, 4);
K_MSGQ_DEFINE(usb_msgq, sizeof(struct padded_buf), 32, 4);
K_MSGQ_DEFINE(lora_msgq, sizeof(struct padded_buf), 5, 4);


enum message_priority {
	MSG_PRIO_LOW,
	MSG_PRIO_HIGH,
};

void init_communication(fjalar_t *fjalar) {
	terminate_communication = false;
	#if DT_ALIAS_EXISTS(lora)
	lora_thread_id = k_thread_create(
		&lora_thread_data,
		lora_thread_stack,
		K_THREAD_STACK_SIZEOF(lora_thread_stack),
		(k_thread_entry_t) lora_thread,
		fjalar, NULL, NULL,
		LORA_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(lora_thread_id, "lora");
	#endif

	#if DT_ALIAS_EXISTS(data_usb)
	usb_thread_id = k_thread_create(
		&usb_thread_data,
		usb_thread_stack,
		K_THREAD_STACK_SIZEOF(usb_thread_stack),
		(k_thread_entry_t) usb_thread,
		fjalar, NULL, NULL,
		USB_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(usb_thread_id, "data usb");
	#endif

	#if DT_ALIAS_EXISTS(data_flash)
	flash_thread_id = k_thread_create(
		&flash_thread_data,
		flash_thread_stack,
		K_THREAD_STACK_SIZEOF(flash_thread_stack),
		(k_thread_entry_t) flash_thread,
		fjalar, NULL, NULL,
		FLASH_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(flash_thread_id, "data flash");
	#endif

	#if DT_ALIAS_EXISTS(external_uart)
	uart_thread_id = k_thread_create(
		&uart_thread_data,
		uart_thread_stack,
		K_THREAD_STACK_SIZEOF(uart_thread_stack),
		(k_thread_entry_t) uart_thread,
		fjalar, NULL, NULL,
		UART_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(uart_thread_id, "external uart");
	#endif

	sampler_thread_id = k_thread_create(
		&sampler_thread_data,
		sampler_thread_stack,
		K_THREAD_STACK_SIZEOF(sampler_thread_stack),
		(k_thread_entry_t) sampler_thread,
		fjalar, NULL, NULL,
		SAMPLER_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(sampler_thread_id, "sampler");
}

int deinit_communication() {
	terminate_communication = true;
	int e = 0;
	#if DT_ALIAS_EXISTS(lora)
	e |= k_thread_join(&lora_thread_data, K_MSEC(1000));
	#endif

	#if DT_ALIAS_EXISTS(data_usb)
	e |= k_thread_join(&usb_thread_data, K_MSEC(1000));
	#endif

	#if DT_ALIAS_EXISTS(data_flash)
	e |= k_thread_join(&flash_thread_data, K_MSEC(1000));
	#endif

	#if DT_ALIAS_EXISTS(external_uart)
	e |= k_thread_join(&uart_thread_data, K_MSEC(1000));
	#endif

	e |= k_thread_join(&sampler_thread_data, K_MSEC(1000));
	return e;
}

void send_message(fjalar_t *fjalar, fjalar_message_t *msg, enum message_priority prio) {
	struct padded_buf pbuf;
	int size = encode_fjalar_message(msg, pbuf.buf);
	if (size < 0) {
		LOG_ERR("encode_fjalar_message failed");
		return;
	}
	LOG_DBG("sending message w/ size %d", size);
	switch(prio) {
		case MSG_PRIO_LOW:
			if (fjalar->flight_state == STATE_IDLE || fjalar->flight_state == STATE_LANDED) {
				break;
			}
			#if DT_ALIAS_EXISTS(data_flash)
			if (k_msgq_put(&flash_msgq, &pbuf, K_NO_WAIT)) {
				LOG_INF("could not insert into flash msgq");
			}
			#endif
			#if DT_ALIAS_EXISTS(external_uart)
			if (k_msgq_put(&uart_msgq, &pbuf, K_NO_WAIT)) {
				LOG_WRN("could not insert into uart msgq");
			}
			#endif
			// #if DT_ALIAS_EXISTS(data_usb)
			// if (k_msgq_put(&usb_msgq, &pbuf, K_NO_WAIT)) {
			// 	LOG_INF("could not insert data usb msgq");
			// }
			// #endif
			break;

		case MSG_PRIO_HIGH:
			#if DT_ALIAS_EXISTS(data_flash)
			if (k_msgq_put(&flash_msgq, &pbuf, K_NO_WAIT)) {
				LOG_INF("could not insert into flash msgq");
			}
			#endif
			#if DT_ALIAS_EXISTS(lora)
			if (k_msgq_put(&lora_msgq, &pbuf, K_NO_WAIT)) {
				LOG_ERR("could not insert into lora msgq");
			}
			#endif
			#if DT_ALIAS_EXISTS(external_uart)
			if (k_msgq_put(&uart_msgq, &pbuf, K_NO_WAIT)) {
				LOG_WRN("could not insert into uart msgq");
			}
			#endif
			#if DT_ALIAS_EXISTS(data_usb)
			if (k_msgq_put(&usb_msgq, &pbuf, K_NO_WAIT)) {
				LOG_INF("could not insert data usb msgq");
			}
			#endif
			break;
	}
}

void send_response(fjalar_t *fjalar, fjalar_message_t *msg, enum com_channels channel) {
	struct padded_buf pbuf;
	if (msg->has_data == false) {
		msg->has_data = true;
		LOG_WRN("msg had has_data false");
	}
	int size = encode_fjalar_message(msg, pbuf.buf);
	LOG_DBG("encoded size %d", size);
	if (size < 0) {
		LOG_ERR("encode_fjalar_message failed");
		return;
	}
	switch (channel) {
		case COM_CHAN_FLASH:
			#if DT_ALIAS_EXISTS(data_flash)
			if (k_msgq_put(&flash_msgq, &pbuf, K_NO_WAIT)) {
				LOG_INF("could not insert into flash msgq");
			}
			#endif
			break;

		case COM_CHAN_LORA:
			#if DT_ALIAS_EXISTS(lora)
			if (k_msgq_put(&lora_msgq, &pbuf, K_NO_WAIT)) {
				LOG_ERR("could not insert into lora msgq");
			}
			#endif
			break;

		case COM_CHAN_EXT_UART:
			#if DT_ALIAS_EXISTS(external_uart)
			if (k_msgq_put(&uart_msgq, &pbuf, K_NO_WAIT)) {
				LOG_WRN("could not insert into uart msgq");
			}
			#endif
			break;

		case COM_CHAN_USB:
			#if DT_ALIAS_EXISTS(data_usb)
			if (k_msgq_put(&usb_msgq, &pbuf, K_NO_WAIT)) {
				LOG_INF("could not insert data usb msgq");
			}
			#endif
			break;
	}
}

void store_message(fjalar_t *fjalar, fjalar_message_t *msg) {
	struct padded_buf pbuf;
	int size = encode_fjalar_message(msg, pbuf.buf);
	if (size < 0) {
		LOG_ERR("encode_fjalar_message failed");
		return;
	}
	LOG_DBG("storing message w/ size %d", size);
	if (k_msgq_put(&flash_msgq, &pbuf, K_NO_WAIT)) {
		LOG_INF("Could not store message");
	}
}

void sampler_thread(fjalar_t *fjalar, void *p2, void *p3) {
	while (true) {
		k_msleep(1000);
		fjalar_message_t msg;
		msg.time = k_uptime_get_32();
		msg.has_data = true;
		msg.data.which_data = FJALAR_DATA_TELEMETRY_PACKET_TAG;
		msg.data.data.telemetry_packet.altitude = fjalar->altitude - fjalar->ground_level;
		msg.data.data.telemetry_packet.az = fjalar->az;
		msg.data.data.telemetry_packet.flight_state = fjalar->flight_state;
		msg.data.data.telemetry_packet.velocity = fjalar->velocity;
		msg.data.data.telemetry_packet.battery = fjalar->battery_voltage;
		msg.data.data.telemetry_packet.latitude = fjalar->latitude;
		msg.data.data.telemetry_packet.longitude = fjalar->longitude;
		msg.data.data.telemetry_packet.flash_address = fjalar->flash_address;
		msg.data.data.telemetry_packet.pyro1_connected = fjalar->pyro1_sense;
		msg.data.data.telemetry_packet.pyro2_connected = fjalar->pyro2_sense;
		msg.data.data.telemetry_packet.pyro3_connected = fjalar->pyro3_sense;
		msg.data.data.telemetry_packet.sudo = fjalar->sudo;
		send_message(fjalar, &msg, MSG_PRIO_HIGH);
	}
}

void read_flash(fjalar_t *fjalar, uint8_t *buf, uint32_t index, uint8_t len) {
	#if DT_ALIAS_EXISTS(data_flash)
	LOG_DBG("Reading flash");
	const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(data_flash));
	k_mutex_lock(&flash_mutex, K_FOREVER);
	flash_read(flash_dev, index, buf, len);
	k_mutex_unlock(&flash_mutex);
	#endif
}

void clear_flash(fjalar_t *fjalar) {
	LOG_WRN("Clearing flash");
	#if DT_ALIAS_EXISTS(data_flash)
	const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(data_flash));
	k_mutex_lock(&flash_mutex, K_FOREVER);
	flash_erase(flash_dev, 0, fjalar->flash_size);
	fjalar->flash_address = 0;
	k_mutex_unlock(&flash_mutex);
	#endif
}

#if DT_ALIAS_EXISTS(data_flash)
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
			LOG_INF("wrote to flash address %d", fjalar->flash_address);
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
#endif

#if DT_ALIAS_EXISTS(lora)
int lora_configure(const struct device *dev, bool transmit) {
	static uint8_t current_mode = -1;
	if (current_mode == transmit) {
		LOG_DBG("Mode already configured");
		return 0;
	}
	struct lora_modem_config config = PROTOCOL_ZEPHYR_LORA_CONFIG;
	config.tx = transmit;
	config.tx_power = 10;
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

void lora_cb(const struct device *dev, uint8_t *buf, uint16_t size, int16_t rssi, int8_t snr) {
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
		lora_recv_async(lora_dev, lora_cb);
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
			lora_recv_async(lora_dev, NULL);
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
#endif

//posix doens't support async omega lul
// struct generic_async_uart {
// 	uint8_t buf[3][128];
// 	int index;
// 	int used;
// };

// void generic_uart_callback(const struct device *dev, struct uart_event *evt, void *user_data) {
// 	struct generic_async_uart *stuff = (struct generic_async_uart *) user_data;

// 	switch (evt->type) {
// 		case UART_RX_BUF_REQUEST:
// 			if (stuff->used == 3) {
// 				LOG_ERR("UART driver buffer overflow");
// 				return;
// 			}
// 			uart_rx_buf_rsp(dev, stuff->buf[stuff->index], sizeof(stuff->buf[0]));
// 			stuff->index = (stuff->index + 1) % (sizeof(stuff->buf) / sizeof(stuff->buf[0]));
// 			stuff->used++;
// 			break;

// 		case UART_RX_BUF_RELEASED:
// 			stuff->used--;
// 			break;
// 		default:
// 			break;
// 	}
// }

#if DT_ALIAS_EXISTS(external_uart)
void uart_thread(fjalar_t *fjalar, void *p2, void *p3) {
	const struct device *uart_dev = DEVICE_DT_GET(DT_ALIAS(external_uart));
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("External uart not ready");
		return;
	}
	struct protocol_state ps;
	reset_protocol_state(&ps);
	while(1) {
		// rx
		int ret;
		uint8_t byte;
		ret = uart_poll_in(uart_dev, &byte);
		if (ret == 0) {
			handle_fjalar_buf(&ps, fjalar, &byte, 1, COM_CHAN_EXT_UART);
			continue;
		} else
		if (ret != -1) {
			LOG_ERR("UART read error");
		}

		// tx
		struct padded_buf pbuf;
		ret = k_msgq_get(&uart_msgq, &pbuf, K_NO_WAIT);
		if (ret == 0) {
			int size = get_encoded_message_length(pbuf.buf);
			for (int i = 0; i < size; i++) {
				uart_poll_out(uart_dev, pbuf.buf[i]);
			}
		}
		k_msleep(1);
	}
}
#endif

#if DT_ALIAS_EXISTS(data_usb)
void usb_thread(fjalar_t *fjalar, void *p2, void *p3) {
	const struct device *usb_dev = DEVICE_DT_GET(DT_ALIAS(data_usb));
	if (!device_is_ready(usb_dev)) {
		LOG_ERR("data usb not ready");
		return;
	}
	struct protocol_state ps;
	reset_protocol_state(&ps);
	while(1) {
		// rx
		int ret;
		uint8_t byte;
		ret = uart_poll_in(usb_dev, &byte);
		if (ret == 0) {
			handle_fjalar_buf(&ps, fjalar, &byte, 1, COM_CHAN_USB);
			continue;
		} else
		if (ret != -1) {
			LOG_ERR("USB read error");
		}

		// tx
		struct padded_buf pbuf;
		ret = k_msgq_get(&usb_msgq, &pbuf, K_NO_WAIT);
		if (ret == 0) {
			int size = get_encoded_message_length(pbuf.buf);
			for (int i = 0; i < size; i++) {
				uart_poll_out(usb_dev, pbuf.buf[i]);
			}
		}
		k_msleep(1);
	}
}
#endif