#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/zbus/zbus.h>
#include <math.h>
#include <pla.h>
#include <zephyr/init.h>
#include <zephyr/drivers/can.h> 

#include "fjalar.h"
#include "sensors.h"
#include "init.h"
#include "filter.h"
#include "aerodynamics.h"
#include "flight_state.h"
#include "actuation.h"
#include "com_can.h"
#include "control.h"

LOG_MODULE_REGISTER(can_com, LOG_LEVEL_INF);

#define CAN_THREAD_PRIORITY 7
#define CAN_THREAD_STACK_SIZE 4096

#define MATCH_ALL_STD_ID  0x7FFu  // binary 0b11111111111 (must match all 11 bits of identifier)

void can_thread(fjalar_t *fjalar, void *p2, void *p1);

K_THREAD_STACK_DEFINE(can_thread_stack, CAN_THREAD_STACK_SIZE);
struct k_thread can_thread_data;
k_tid_t can_thread_id;

void init_can(fjalar_t *fjalar) {
    can_thread_id = k_thread_create(
		&can_thread_data,
		can_thread_stack,
		K_THREAD_STACK_SIZEOF(can_thread_stack),
		(k_thread_entry_t) can_thread,
		fjalar, NULL, NULL,
		CAN_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(can_thread_id, "can");
}

#if DT_ALIAS_EXISTS(canbus)

struct can_timing timing;

const struct can_filter filter_rx_loki = {
    .flags = 0,
    .id = 0x6FF,
    .mask = MATCH_ALL_STD_ID
};

/*
const struct can_filter filter_rx_sigurd = {
    .flags = 0, 
    .id = 0x000, // change
    .mask = MATCH_ALL_STD_ID
};
*/

const struct device *const can_dev = DEVICE_DT_GET(DT_ALIAS(canbus));

void can_tx_loki(const struct device *can_dev, struct flight_state_output_msg *fs_msg, control_t *control, can_t *can)
{
    const size_t DLC = 4;
    uint8_t data[DLC];
    int ret;

    // byte 0: high nibble=state, low nibble=substate 
    uint8_t st = (uint8_t)fs_msg->flight_state;
    if (st > 0x0F) {
        LOG_ERR("state out of range");
    }

    uint8_t ev = (fs_msg->event_above_acs_threshold) ? 0xA : 0x5; // choose 0xA (1010) or 0x5 (0101)
    
    data[0] = (ev << 4) | (st & 0x0F);

    // byte 1: event marker 
    data[1] = (fs_msg->event_above_acs_threshold && fs_msg->flight_state == STATE_COAST) ? 0xAA : 0x55;

    // bytes 2–3: angle ×100 
    float  airbrake_angle = control->airbrakes_angle; // from control script (PID algo)
    if (airbrake_angle < 0.0f || airbrake_angle > 360.0f) {LOG_ERR("angle invalid");}
    uint16_t raw_angle = (uint16_t)roundf(airbrake_angle * 100.0f);
    data[2] = (raw_angle >> 8) & 0xFF;
    data[3] = raw_angle & 0xFF;

    struct can_frame frame = {
        .flags = 0,
        .id    = 0x67F, // 0x67F = Fjalar 1, 0x57F = Fjalar 2. Put this as #define in header
        .dlc   = DLC,
    };
    memcpy(frame.data, data, DLC);

    ret = can_send(can_dev, &frame, K_MSEC(100), NULL, NULL);
    if (ret) {LOG_ERR("CAN TX Loki failed [%d]", ret);}

    can->loki_latest_tx_time = k_uptime_get_32();
}

void can_tx_sigurd(state_t *state, const struct device *const can_dev){
    /*
    struct can_frame can_tx_sigurd_frame = {
        .flags = 0,
        .id = 0x123, // change
        .dlc = 8, // change
        .data = {} // change
    };
    
    int ret;
    ret = can_send(can_dev, &can_tx_sigurd_frame, K_MSEC(100), NULL, NULL);
    if (ret != 0) {LOG_ERR("can tx sigurd failed [%d]", ret);}
        
    */

}

void can_rx_loki(const struct device *const can_dev, struct can_frame *frame, void *user_data){
    can_t *can = user_data;
    const uint8_t *data = frame->data;

    if (frame->dlc != 4) {LOG_ERR("Wrong dlc can rx loki");}

    // byte 0: low nibble = state, high nibble = substate
    can->loki_state     =  data[0] & 0x0F;
    can->loki_sub_state = (data[0] >> 4) & 0x0F;

    // bytes 1–2: 16-bit angle, gain=100 
    uint16_t raw_angle = (data[1] << 8) | data[2];
    can->loki_angle    = raw_angle / 100.0f;

    // byte 3: battery, gain=10 
    can->loki_battery_voltage = data[3] / 10.0f;

    can->loki_latest_rx_time = k_uptime_get_32();
}

void can_rx_sigurd(const struct device *const can_dev, struct can_frame *frame, void *user_data){
    // stuff
}



// Give privilage to RX callbacks
static int can_cb_priv_init(can_t *can){
    int err;
    err = can_add_rx_filter(can_dev, can_rx_loki, can, &filter_rx_loki);
    if (err) {LOG_ERR("adding loki filter failed: %d", err);}else{LOG_WRN("adding loki filter success");}

    //err = can_add_rx_filter(can_dev, can_rx_sigurd, can, &filter_rx_sigurd);
    //if (err) {LOG_ERR("adding sigurd filter failed: %d", err);}
    return 0;
}

#endif

void can_thread(fjalar_t *fjalar, void *p2, void *p1) {
    can_t             *can = fjalar->ptr_can;
    control_t         *control = fjalar->ptr_control;
    int ret = 0;

    #if DT_ALIAS_EXISTS(canbus)
    static uint8_t init_flag =0;

    if (init_flag==0)
    {
        init_flag=1;
        ret = can_cb_priv_init(can);
    }

    ret = can_set_bitrate(can_dev, 500000); // 500 kb/s
    if (ret) {LOG_ERR("Failed to set CAN nominal bitrate: [%d]", ret);}
    else{LOG_INF("CAN nominal bitrate successfully set to 500kb/s");}

    ret = can_set_bitrate_data(can_dev, 500000); // 500 kb/s
    if (ret) {LOG_ERR("Failed to set CAN bitrate: [%d]", ret);}
    else{LOG_INF("CAN bitrate successfully set to 500kb/s");}

    ret = can_start(can_dev);
    if (ret) {LOG_ERR("Failed to start CAN: %d", ret);}
    else{
        LOG_INF("CAN started");
        can->can_started = true;
    }
    
    #endif
    struct flight_state_output_msg fs_msg;

    while (true) {
        ret = zbus_chan_read(&flight_state_output_zchan, &fs_msg, K_NO_WAIT);
        #if DT_ALIAS_EXISTS(canbus)
        if (ret == 0) {
            // Use fs_msg to populate CAN messages
            can_tx_loki(can_dev, &fs_msg, control, can);
        }
        //can_tx_sigurd(state, can_dev);
        #endif

        /*
        LOG_INF("loki_state          : %d", can->loki_state);
        LOG_INF("loki_sub_state      : %d", can->loki_sub_state);
        LOG_INF("loki_angle          : %f", can->loki_angle);
        LOG_INF("loki_battery_voltage: %f", can->loki_battery_voltage);
        */

        k_msleep(10); // 100 Hz
    }
}