/*
The flight state script serves the following purpose:
1) Given some information about the rocket, determine its flight state
*/

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <math.h>
#include <pla.h>

#include "fjalar.h"
#include "sensors.h"
#include "init.h"
#include "filter.h"
#include "aerodynamics.h"
#include "flight_state.h"
#include "actuation.h"
#include "com_lora.h"
#include "com_can.h"

LOG_MODULE_REGISTER(flight, LOG_LEVEL_INF);

#define FLIGHT_THREAD_PRIORITY 7
#define FLIGHT_THREAD_STACK_SIZE 4096

#define ALTITUDE_PRIMARY_WINDOW_SIZE 5
#define ALTITUDE_SECONDARY_WINDOW_SIZE 40

#define IMU_WINDOW_SIZE 11


#define DROGUE_DEPLOYMENT_FAILURE_DELAY 8000






void flight_state_thread(fjalar_t *fjalar, void *p2, void *p1);

K_THREAD_STACK_DEFINE(flight_thread_stack, FLIGHT_THREAD_STACK_SIZE);
struct k_thread flight_thread_data;
k_tid_t flight_thread_id;

ZBUS_LISTENER_DEFINE(pressure_zlis, NULL);
ZBUS_LISTENER_DEFINE(imu_zlis, NULL);
// ZBUS_CHAN_ADD_OBS(pressure_zchan, pressure_zobs, 1);
// ZBUS_CHAN_ADD_OBS(imu_zchan, imu_zobs, 1);

// ZBUS_SUBSCRIBER_DEFINE(imu_zchan);

void init_flight_state(fjalar_t *fjalar) {
    flight_thread_id = k_thread_create(
		&flight_thread_data,
		flight_thread_stack,
		K_THREAD_STACK_SIZEOF(flight_thread_stack),
		(k_thread_entry_t) flight_state_thread,
		fjalar, NULL, NULL,
		FLIGHT_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(flight_thread_id, "flight state");
}

// fix the information logic
static void deploy_drogue(fjalar_t *fjalar, init_t *init, state_t *state, position_filter_t *pos_kf) { 
    set_pyro(fjalar, 1, true); // Blowing pyro charge, [1 = Drouge]
    LOG_WRN("drogue deployed at %.2f m %.2f s", pos_kf->X_data[2], (state->apogee_time - state->liftoff_time) / 1000.0f);
}

// fix the information logic
static void deploy_main(fjalar_t *fjalar, init_t *init, state_t *state, position_filter_t *pos_kf) {
    set_pyro(fjalar, 2, true); // Blowing pyro charge, [2 = Main]
    LOG_WRN("Main deployed at %.2f m", pos_kf->X_data[2]);
}

/*
static void start_pyro_camera(fjalar_t *fjalar){
    set_pyro(fjalar, 3, true); // starts camera [3 = Camera]
    LOG_WRN("Camera Started");
}
*/

static void evaluate_state(fjalar_t *fjalar, init_t *init, state_t *state, position_filter_t *pos_kf, aerodynamics_t *aerodynamics, lora_t *lora, can_t *can) {
    float az = pos_kf->X_data[8];
    float vz = pos_kf->X_data[5];
    float z  = pos_kf->X_data[2];

    float a_norm = pos_kf->a_norm;
    float v_norm = pos_kf->v_norm;

    switch (state->flight_state) {
    case STATE_IDLE:
        // Check both LoRa and CAN for ready/arm command (redundancy)
        if (lora->LORA_READY_INITIATE_FJALAR || (can != NULL && can->CAN_GCS_READY_INITIATE_FJALAR)){
            state->flight_state = STATE_AWAITING_INIT;
            init_init(&fjalar_god); // see mural documentation
            init_sensors(&fjalar_god); // see mural documentation
            if (lora->LORA_READY_INITIATE_FJALAR) {
                LOG_WRN("State transitioned from STATE_IDLE to STATE_AWAITING_INIT due to LoRa command");
            } else {
                LOG_WRN("State transitioned from STATE_IDLE to STATE_AWAITING_INIT due to CAN command");
            }
        }
        break;
    case STATE_AWAITING_INIT:
        if (init->init_completed){
            state->flight_state = STATE_INITIATED;
            LOG_WRN("State transitioned from STATE_AWAITING_INIT to STATE_INITIATED due to completed initialization");
        } // change for lora struct
        break;
    case STATE_INITIATED:
        // Check both LoRa and CAN for launch command (redundancy)
        if (lora->LORA_READY_LAUNCH_FJALAR || (can != NULL && can->CAN_GCS_READY_LAUNCH_FJALAR)){
            state->flight_state = STATE_AWAITING_LAUNCH;
            if (lora->LORA_READY_LAUNCH_FJALAR) {
                LOG_WRN("State transitioned from STATE_INITIATED to STATE_AWAITING_LAUNCH due to LoRa command");
            } else {
                LOG_WRN("State transitioned from STATE_INITIATED to STATE_AWAITING_LAUNCH due to CAN command");
            }
        }
        break;
    case STATE_AWAITING_LAUNCH:
        if (a_norm > BOOST_ACCEL_THRESHOLD && z > 3){
            state->flight_state = STATE_BOOST;
            state->event_launch = true;
            state->liftoff_time = k_uptime_get_32();
            LOG_WRN("State transitioned from STATE_AWAITING_LAUNCH to STATE_BOOST due to acceleration");
        }

        // adding velocity as a requirement here will not work due to the velocity zeroing logic i filter.c
        if (v_norm > BOOST_SPEED_THRESHOLD){ 
            state->flight_state = STATE_BOOST;
            state->event_launch = true;
            state->liftoff_time = k_uptime_get_32();
            LOG_WRN("State transitioned from STATE_AWAITING_LAUNCH to STATE_BOOST due to speed");
        }
        break;
    case STATE_BOOST:
        if (az < COAST_ACCEL_THRESHOLD && !aerodynamics->thrust_bool) {
            state->flight_state = STATE_COAST;
            state->event_burnout = true;
            LOG_WRN("State transitioned from STATE_BOOST to STATE_COAST due to acceleration");
        }
        break;
    case STATE_COAST:
        
        if (vz < 0) {
            state->apogee_time = k_uptime_get_32();
            deploy_drogue(fjalar, init, state, pos_kf);
            state->flight_state = STATE_DROGUE_DESCENT;
            state->event_apogee = true;
            LOG_WRN("State transitioned from STATE_COAST to STATE_DROGUE_DESCENT due to velocity");
        }
        break;
    case STATE_DROGUE_DESCENT:
        if (z < 200) {
            deploy_main(fjalar, init, state, pos_kf);
            state->flight_state = STATE_MAIN_DESCENT;
            LOG_WRN("State transitioned from STATE_DROGUE_DESCENT to STATE_MAIN_DESCENT due to altitude");
        }
        break;
    case STATE_MAIN_DESCENT:
        if (a_norm > init->g_accelerometer-2 && a_norm<init->g_accelerometer+2){
            state->flight_state = STATE_LANDED;
            state->event_landed = true;
            LOG_WRN("State transitioned from STATE_MAIN_DESCENT to STATE_LANDED due to acceleration");
        }
        break;
    case STATE_LANDED:
        // yay we landed (right?)
        LOG_INF("flight state: STATE_LANDED");
        break;
    }
}

static void evaluate_event(fjalar_t *fjalar, state_t *state, position_filter_t *pos_kf) {
    float z  = pos_kf->X_data[2];

    state->event_above_acs_threshold = (z > 1500);
    state->event_drogue_deployed = (fjalar->pyro1_sense);
    state->event_main_deployed = (fjalar->pyro2_sense);
}

static void evaluate_velocity(aerodynamics_t *aerodynamics, state_t *state) {
    float M = aerodynamics->mach_number;
    switch(state->velocity_class){
    case VELOCITY_SUBSONIC:
        if (M > 0.8){
            state->velocity_class = VELOCITY_TRANSONIC;
            LOG_WRN("velocity class changed to transsonic");
        }
        break;

    case VELOCITY_TRANSONIC:
        if (M > 1.2){
            state->velocity_class = VELOCITY_SUPERSONIC;
            LOG_WRN("velocity class changed to supersonic");
        }
        if (M < 0.8){
            state->velocity_class = VELOCITY_SUBSONIC;
            LOG_WRN("velocity class changed to subsonic");
        }
        break;

    case VELOCITY_SUPERSONIC:
        if (M < 1.2){
            state->velocity_class = VELOCITY_TRANSONIC;
            LOG_WRN("velocity class changed to transsonic");
        }
        break;

    }
}

void flight_state_thread(fjalar_t *fjalar, void *p2, void *p1) {
    init_t            *init  = fjalar->ptr_init;
    position_filter_t *pos_kf = fjalar->ptr_pos_kf;
    attitude_filter_t *att_kf = fjalar->ptr_att_kf;
    aerodynamics_t    *aerodynamics = fjalar->ptr_aerodynamics;
    state_t           *state = fjalar->ptr_state;
    lora_t            *lora = fjalar->ptr_lora;
    can_t             *can = fjalar->ptr_can;

    state->flight_state = STATE_IDLE;
    state->velocity_class = VELOCITY_SUBSONIC;

    while (true) {
        evaluate_state(fjalar, init, state, pos_kf, aerodynamics, lora, can);
        evaluate_event(fjalar, state, pos_kf);
        evaluate_velocity(aerodynamics, state);
        k_msleep(10); 
    }
}
