/*
The flight state script serves the following purpose:
1) Given some information about the rocket, determine its flight state
*/

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/zbus/zbus.h>
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

LOG_MODULE_REGISTER(flight, LOG_LEVEL_INF);

#define FLIGHT_THREAD_PRIORITY 7
#define FLIGHT_THREAD_STACK_SIZE 4096
#define ALTITUDE_PRIMARY_WINDOW_SIZE 5
#define ALTITUDE_SECONDARY_WINDOW_SIZE 40
#define IMU_WINDOW_SIZE 11
#define DROGUE_DEPLOYMENT_FAILURE_DELAY 8000


void flight_state_thread(fjalar_t *fjalar, void *p2, void *p1);
ZBUS_CHAN_DEFINE(flight_state_output_zchan, /* Name */
		struct flight_state_output_msg, /* Message type */
		NULL, /* Validator */
		NULL, /* User Data */
		ZBUS_OBSERVERS_EMPTY, /* observers */
		ZBUS_MSG_INIT(.timestamp = 0, .flight_state = STATE_IDLE, .velocity_class = VELOCITY_SUBSONIC) /* Initial value */
);
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
static void deploy_drogue(fjalar_t *fjalar, init_t *init, struct flight_state_output_msg *state_data, struct filter_output_msg *filter_data) {
    set_pyro(fjalar, 1, true); // Blowing pyro charge, [1 = Drouge]
    LOG_WRN("drogue deployed at %.2f m %.2f s", filter_data->position[2], (state_data->apogee_time - state_data->liftoff_time) / 1000.0f);
}

// fix the information logic
static void deploy_main(fjalar_t *fjalar, init_t *init, struct filter_output_msg *filter_data) {
    set_pyro(fjalar, 2, true); // Blowing pyro charge, [2 = Main]
    LOG_WRN("Main deployed at %.2f m", filter_data->position[2]);
}

/*
static void start_pyro_camera(fjalar_t *fjalar){
    set_pyro(fjalar, 3, true); // starts camera [3 = Camera]
    LOG_WRN("Camera Started");
}
*/

static void evaluate_state(fjalar_t *fjalar, init_t *init,  struct flight_state_output_msg *state_data, struct filter_output_msg *filter_data, struct aerodynamics_output_msg *aerodynamics, lora_t *lora) {
    float az = filter_data->acceleration[2];
    float vz = filter_data->velocity[2];
    float z  = filter_data->position[2];

    float a_norm = filter_data->a_norm;
    float v_norm = filter_data->v_norm;

    switch (state_data->flight_state) {
    case STATE_IDLE:
        if (lora->LORA_READY_INITIATE_FJALAR){
            state_data->flight_state = STATE_AWAITING_INIT;
            init_init(&fjalar_god); // see mural documentation
            init_sensors(&fjalar_god); // see mural documentation
            LOG_WRN("State transitioned from STATE_IDLE to STATE_AWAITING_INIT due to LoRa command");
        }
        break;
    case STATE_AWAITING_INIT:
        if (k_sem_take(&init_complete_sem, K_NO_WAIT) == 0){
            state_data->flight_state = STATE_INITIATED;
            LOG_WRN("State transitioned from STATE_AWAITING_INIT to STATE_INITIATED due to completed initialization");
        } // change for lora struct
        break;
    case STATE_INITIATED:
        if (lora->LORA_READY_LAUNCH_FJALAR){
            state_data->flight_state = STATE_AWAITING_LAUNCH;
            LOG_WRN("State transitioned from STATE_INITIATED to STATE_AWAITING_LAUNCH due to LoRa command");
        }
        break;
    case STATE_AWAITING_LAUNCH:
        if (a_norm > BOOST_ACCEL_THRESHOLD && z > 3){
            state_data->flight_state = STATE_BOOST;
            state_data->event_launch = true;
            state_data->liftoff_time = k_uptime_get_32();
            LOG_WRN("State transitioned from STATE_AWAITING_LAUNCH to STATE_BOOST due to acceleration");
        }

        // adding velocity as a requirement here will not work due to the velocity zeroing logic i filter.c
        if (v_norm > BOOST_SPEED_THRESHOLD){ 
            state_data->flight_state = STATE_BOOST;
            state_data->event_launch = true;
            state_data->liftoff_time = k_uptime_get_32();
            LOG_WRN("State transitioned from STATE_AWAITING_LAUNCH to STATE_BOOST due to speed");
        }
        break;
    case STATE_BOOST:
        if (az < COAST_ACCEL_THRESHOLD && !aerodynamics->thrust_bool) {
            state_data->flight_state = STATE_COAST;
            state_data->event_burnout = true;
            LOG_WRN("State transitioned from STATE_BOOST to STATE_COAST due to acceleration");
        }
        break;
    case STATE_COAST:
        
        if (vz < 0) {
            state_data->apogee_time = k_uptime_get_32();
            deploy_drogue(fjalar, init, state_data, filter_data);
            state_data->flight_state = STATE_DROGUE_DESCENT;
            state_data->event_apogee = true;
            LOG_WRN("State transitioned from STATE_COAST to STATE_DROGUE_DESCENT due to velocity");
        }
        break;
    case STATE_DROGUE_DESCENT:
        if (z < 200) {
            deploy_main(fjalar, init, filter_data);
            state_data->flight_state = STATE_MAIN_DESCENT;
            LOG_WRN("State transitioned from STATE_DROGUE_DESCENT to STATE_MAIN_DESCENT due to altitude");
        }
        break;
    case STATE_MAIN_DESCENT:
        if (a_norm > init->g_accelerometer-2 && a_norm<init->g_accelerometer+2){
            state_data->flight_state = STATE_LANDED;
            state_data->event_landed = true;
            LOG_WRN("State transitioned from STATE_MAIN_DESCENT to STATE_LANDED due to acceleration");
        }
        break;
    case STATE_LANDED:
        // yay we landed (right?)
        LOG_INF("flight state: STATE_LANDED");
        break;
    }
}

static void evaluate_event(fjalar_t *fjalar,  struct flight_state_output_msg *state_data, struct filter_output_msg *filter_data) {
    float z  = filter_data->position[2];

    state_data->event_above_acs_threshold = (z > 1500);
    state_data->event_drogue_deployed = (fjalar->pyro1_sense);
    state_data->event_main_deployed = (fjalar->pyro2_sense);
}

static void evaluate_velocity(struct aerodynamics_output_msg *aerodynamics, struct flight_state_output_msg *state_data) {
    float M = aerodynamics->mach_number;
    switch(state_data->velocity_class){
    case VELOCITY_SUBSONIC:
        if (M > 0.8){
            state_data->velocity_class = VELOCITY_TRANSONIC;
            LOG_WRN("velocity class changed to transonic");
        }
        break;

    case VELOCITY_TRANSONIC:
        if (M > 1.2){
            state_data->velocity_class = VELOCITY_SUPERSONIC;
            LOG_WRN("velocity class changed to supersonic");
        }
        if (M < 0.8){
            state_data->velocity_class = VELOCITY_SUBSONIC;
            LOG_WRN("velocity class changed to subsonic");
        }
        break;

    case VELOCITY_SUPERSONIC:
        if (M < 1.2){
            state_data->velocity_class = VELOCITY_TRANSONIC;
            LOG_WRN("velocity class changed to transonic");
        }
        break;

    }
}

void flight_state_thread(fjalar_t *fjalar, void *p2, void *p1) {
    init_t            *init  = fjalar->ptr_init;
    lora_t            *lora = fjalar->ptr_lora;
	struct filter_output_msg filter_data;
    struct aerodynamics_output_msg aero_data = {0};

    // Initialize state message
    struct flight_state_output_msg state_msg = {
        .timestamp = 0,
        .flight_state = STATE_IDLE,
        .velocity_class = VELOCITY_SUBSONIC,
        .liftoff_time = 0,
        .apogee_time = 0,
        .event_launch = false,
        .event_burnout = false,
        .event_above_acs_threshold = false,
        .event_apogee = false,
        .event_drogue_deployed = false,
        .event_main_deployed = false,
        .event_landed = false
    };


    while (true) {
		// get latest filter data
		if (zbus_chan_read(&filter_output_zchan, &filter_data, K_NO_WAIT) == 0) {
        	zbus_chan_read(&aero_chan, &aero_data, K_NO_WAIT);
        	// Update timestamp
        	state_msg.timestamp = k_uptime_get_32();
        	// update flight state
        	evaluate_state(fjalar, init, &state_msg, &filter_data, &aero_data, lora);
        	evaluate_event(fjalar, &state_msg, &filter_data);
        	evaluate_velocity(&aero_data, &state_msg);

        	// Publish state to zbus
        	if (zbus_chan_pub(&flight_state_output_zchan, &state_msg, K_NO_WAIT) != 0) {
        	    LOG_WRN("Flight state output zbus channel busy, dropping message");
        	}
        }
    }
}
