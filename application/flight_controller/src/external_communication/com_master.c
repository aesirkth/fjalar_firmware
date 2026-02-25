/* 
This is the communication script, its purpose is to:
1) Send and recieve information to and from a telemetry modem (commonly used with our own modem, Brage), 
it uses a protocol specified in schema.proto.
2) Communicate with uart and usb, this is very important since it makes debugging possible and we can feed it simulated 
sensor information via usb/uart.
*/


#include <zephyr/console/tty.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <protocol.h>
#include <zephyr/drivers/flash.h>


#include "fjalar.h"
#include "com_master.h"
#include "com_flash.h"
#include "com_lora.h"
#include "com_uart.h"
#include "com_usb.h"
#include "flight_state.h"
#include "hilsensor.h"

LOG_MODULE_REGISTER(com_master, LOG_LEVEL_INF);


#define SAMPLER_THREAD_PRIORITY 7
#define SAMPLER_THREAD_STACK_SIZE 2048

volatile bool terminate_communication = false;



K_THREAD_STACK_DEFINE(sampler_thread_stack, SAMPLER_THREAD_STACK_SIZE);
struct k_thread sampler_thread_data;
k_tid_t sampler_thread_id;
void sampler_thread(fjalar_t *fjalar, state_t *state, void *p2, void *p3);

_Static_assert(sizeof(struct padded_buf) % 4 == 0, "padded buffer length is not aligned");


void init_communication(fjalar_t *fjalar) {
	terminate_communication = false;

	sampler_thread_id = k_thread_create(
		&sampler_thread_data,
		sampler_thread_stack,
		K_THREAD_STACK_SIZEOF(sampler_thread_stack),
		(k_thread_entry_t) sampler_thread,
		fjalar, fjalar->ptr_state, NULL,
		SAMPLER_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(sampler_thread_id, "sampler");

	
	init_com_flash(fjalar);
	init_com_lora(fjalar);
	init_com_uart(fjalar);
	init_com_usb(fjalar);
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

void handle_hil_in(hil_in_t *msg, fjalar_t *fjalar, enum com_channels channel){
    #if DT_NODE_EXISTS(DT_ALIAS(hilsensor))
    const struct device *const hilsensor_dev = DEVICE_DT_GET(DT_ALIAS(hilsensor));

    hil_data_t hil_data = {
        .ax = msg->ax,
        .ay = msg->ay,
        .az = msg->az,
        .gx = msg->gx,
        .gy = msg->gy,
        .gz = msg->gz,
        .p = msg->p,
        .lon = msg->lon,
        .lat = msg->lat,
        .alt = msg->alt,
        .time = k_uptime_get_32(),
    };
	
	fjalar->ptr_lora->LORA_READY_INITIATE_FJALAR = msg->awaiting_init;
	fjalar->ptr_lora->LORA_READY_LAUNCH_FJALAR = msg->awaiting_launch;
	
    hilsensor_feed(hilsensor_dev, &hil_data);
    #endif
}

void handle_gcb_in(lora_gcb_to_fjalar_t *msg, fjalar_t *fjalar, enum com_channels channel){
	fjalar->ptr_lora->LORA_READY_INITIATE_FJALAR = msg->ready_initiate_fjalar;
	fjalar->ptr_lora->LORA_READY_LAUNCH_FJALAR = msg->ready_launch_fjalar;
}

void handle_set_sudo(set_sudo_t *msg, fjalar_t *fjalar, enum com_channels channel) {
    fjalar->sudo = msg->enabled;
    LOG_WRN("set sudo %d", msg->enabled);
}

void handle_trigger_pyro(trigger_pyro_t *msg, fjalar_t *fjalar, enum com_channels channel) {
    if (fjalar->sudo != true) {
        LOG_WRN("Tried to enable pyro %d when not in sudo mode", msg->pyro);
        return;
    }
        set_pyro(fjalar, msg->pyro, 1);
        k_msleep(500);
        set_pyro(fjalar, msg->pyro, 0);
}

void read_flash(fjalar_t *fjalar, uint8_t *buf, uint32_t index, uint8_t len) {
	LOG_DBG("Reading flash");
	const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(data_flash));
	k_mutex_lock(&flash_mutex, K_FOREVER);
	flash_read(flash_dev, index, buf, len);
	k_mutex_unlock(&flash_mutex);
}


void clear_flash(fjalar_t *fjalar) {
	LOG_WRN("Clearing flash");
	const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(data_flash));
	k_mutex_lock(&flash_mutex, K_FOREVER);
	flash_erase(flash_dev, 0, fjalar->flash_size);
	fjalar->flash_address = 0;
	k_mutex_unlock(&flash_mutex);
}


void handle_clear_flash(clear_flash_t *msg, fjalar_t *fjalar, enum com_channels channel) {
    //if (fjalar->sudo != true) {
    //    LOG_WRN("Tried to clear flash when not in sudo mode");
    //    return;
    //}
    clear_flash(fjalar); // link this to com_flash
}

void handle_read_flash(read_flash_t *msg, fjalar_t *fjalar, enum com_channels channel) {
    struct fjalar_message resp;
    if (msg->length > sizeof(resp.data.data.flash_data.data.bytes)) {
        LOG_ERR("Requested flash read can't fit in buffer");
        return;
    }
    resp.has_data = true;
    resp.data.which_data = FJALAR_DATA_FLASH_DATA_TAG;
    resp.data.data.flash_data.data.size = msg->length;
    resp.data.data.flash_data.start_index = msg->start_index;
    resp.time = k_uptime_get_32();
    read_flash(fjalar, resp.data.data.flash_data.data.bytes, msg->start_index, msg->length);
    send_response(fjalar, &resp, channel);
}

void handle_fjalar_buf(struct protocol_state *ps, fjalar_t *fjalar, uint8_t *buf, int len, enum com_channels channel) {
    fjalar_message_t msg;
    for (int i = 0; i < len; i++) {
        int ret;
        ret = parse_fjalar_message(ps, buf[i], &msg);
        if (ret == 1) {
            handle_fjalar_message(&msg, fjalar, fjalar->ptr_state, channel);
        } else if (ret == -1) {
            reset_protocol_state(ps);
        }
    }
}

void handle_fjalar_message(fjalar_message_t *msg, fjalar_t *fjalar, state_t *state, enum com_channels channel) {
    //LOG_INF("handling msg with id %d", msg->data.which_data);
    switch (msg->data.which_data) {
        case FJALAR_DATA_SET_SUDO_TAG:
            handle_set_sudo(&msg->data.data.set_sudo, fjalar, channel);
            break;
        case FJALAR_DATA_TRIGGER_PYRO_TAG:
            handle_trigger_pyro(&msg->data.data.trigger_pyro, fjalar, channel);
            break;
        case FJALAR_DATA_CLEAR_FLASH_TAG:
            handle_clear_flash(&msg->data.data.clear_flash, fjalar, channel);
            break;
        case FJALAR_DATA_READ_FLASH_TAG:
            handle_read_flash(&msg->data.data.read_flash, fjalar, channel);
            break;
        case FJALAR_DATA_HIL_IN_TAG:
            handle_hil_in(&msg->data.data.hil_in, fjalar, channel);
            break;
		case FJALAR_DATA_LORA_GCB_TO_FJALAR_TAG:
			handle_gcb_in(&msg->data.data.lora_gcb_to_fjalar, fjalar, channel);
			break;
        default:
            LOG_ERR("Unsupported message: %d", msg->data.which_data);
    }
}

void send_message(fjalar_t *fjalar, state_t *state, fjalar_message_t *msg, enum message_priority prio) {
	struct padded_buf pbuf;
	int size = encode_fjalar_message(msg, pbuf.buf);
	if (size < 0) {
		LOG_ERR("encode_fjalar_message failed");
		return;
	}
	LOG_DBG("sending message w/ size %d", size);
	switch(prio) {
		case MSG_PRIO_LOW:
			struct flight_state_output_msg fs_msg;
			int ret = zbus_chan_read(&flight_state_output_zchan, &fs_msg, K_NO_WAIT);
			if (ret != 0) {
				LOG_DBG("No flight state message available, skipping low-prio send");
				return;
			}

			if (fs_msg.flight_state == STATE_IDLE || fs_msg.flight_state == STATE_LANDED) {
				return;
			}
			#if DT_ALIAS_EXISTS(data_flash)
			if (k_msgq_put(&flash_msgq, &pbuf, K_NO_WAIT)) {
				//LOG_INF("could not insert into flash msgq");
			}
			#endif
			#if DT_ALIAS_EXISTS(external_uart)
			if (k_msgq_put(&uart_msgq, &pbuf, K_NO_WAIT)) {
				LOG_WRN("could not insert into uart msgq");
			}
			#endif
			#if DT_ALIAS_EXISTS(data_usb)
			if (k_msgq_put(&usb_msgq, &pbuf, K_NO_WAIT)) {
				//LOG_INF("could not insert data usb msgq");
			}
			#endif
			break;

		case MSG_PRIO_HIGH:
			#if DT_ALIAS_EXISTS(data_flash)
			if (k_msgq_put(&flash_msgq, &pbuf, K_NO_WAIT)) {
				//LOG_INF("could not insert into flash msgq");
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
				//LOG_INF("could not insert data usb msgq");
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
				//LOG_INF("could not insert into flash msgq");
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
				LOG_WRN("could not insert data usb msgq");
			}
			#endif
			break;
		case COM_CHAN_MAX: // ignore
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

void sampler_thread(fjalar_t *fjalar, state_t *state, void *p2, void *p3) { // remove state ptr?
	while (true) {
		/*
		fjalar_message_t msg;
		msg.time = k_uptime_get_32();
		msg.has_data = true;
		msg.data.which_data = FJALAR_DATA_TELEMETRY_PACKET_TAG;
		msg.data.data.telemetry_packet.altitude = fjalar->altitude - fjalar->ground_level;
		msg.data.data.telemetry_packet.az = fjalar->az;
		msg.data.data.telemetry_packet.flight_state = state->flight_state;
		msg.data.data.telemetry_packet.velocity = fjalar->velocity;
		msg.data.data.telemetry_packet.battery = fjalar->battery_voltage;
		msg.data.data.telemetry_packet.latitude = fjalar->latitude;
		msg.data.data.telemetry_packet.longitude = fjalar->longitude;
		msg.data.data.telemetry_packet.flash_address = fjalar->flash_address;
		msg.data.data.telemetry_packet.pyro1_connected = fjalar->pyro1_sense;
		msg.data.data.telemetry_packet.pyro2_connected = fjalar->pyro2_sense;
		msg.data.data.telemetry_packet.pyro3_connected = fjalar->pyro3_sense;
		msg.data.data.telemetry_packet.sudo = fjalar->sudo;

		msg.data.data.hil_out.airbrake_percentage = 0.0f;
		msg.data.data.hil_out.main_deployed = fjalar->main_deployed;
		msg.data.data.hil_out.drogue_deployed = fjalar->drogue_deployed;
		send_message(fjalar, fjalar->ptr_state, &msg, MSG_PRIO_HIGH);
		*/
		k_msleep(1000);
	}
}