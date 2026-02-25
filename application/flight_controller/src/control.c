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
#include "control.h"

#define CONTROL_THREAD_PRIORITY 7
#define CONTROL_THREAD_STACK_SIZE 4096


LOG_MODULE_REGISTER(control, LOG_LEVEL_INF);

void control_thread(fjalar_t *fjalar, void *p2, void *p1);

K_THREAD_STACK_DEFINE(control_thread_stack, CONTROL_THREAD_STACK_SIZE);
struct k_thread control_thread_data;
k_tid_t control_thread_id;

void init_control(fjalar_t *fjalar) {
    control_thread_id = k_thread_create(
		&control_thread_data,
		control_thread_stack,
		K_THREAD_STACK_SIZEOF(control_thread_stack),
		(k_thread_entry_t) control_thread,
		fjalar, NULL, NULL,
		CONTROL_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(control_thread_id, "control");
}

void control_thread(fjalar_t *fjalar, void *p2, void *p1) {
	struct filter_output_msg filter_data;
	struct aerodynamics_output_msg aero_data;
	struct flight_state_output_msg fs_data;
	enum fjalar_flight_state flight_state = STATE_INITIATED;
	enum fjalar_velocity_class velocity_class = VELOCITY_SUBSONIC;
	static float last_predicted_apogee = 0.0f;
	static float altitude_AGL = 0.0f; // Remember last valid altitude

    control_t         *control = fjalar->ptr_control;

    static float integral = 0.0f;
    static float last_error = 0.0f;
    while (true){
        // Try to get latest filter data (non-blocking)
		if (zbus_chan_read(&filter_output_zchan, &filter_data, K_NO_WAIT) == 0) {
    		altitude_AGL = filter_data.position[2];
		}
    	// Update flight state if new messages exist
    	while (zbus_chan_read(&flight_state_output_zchan, &fs_data, K_NO_WAIT) == 0) {
    		flight_state = fs_data.flight_state;
    		velocity_class = fs_data.velocity_class;
    	}
    	// Try to get latest aerodynamics data (non-blocking)
    	if (zbus_chan_read(&aero_chan, &aero_data, K_NO_WAIT) == 0) {
    		last_predicted_apogee = aero_data.expected_apogee;
    	}

        if (flight_state == STATE_COAST && altitude_AGL > 1500.0f) {
            float predicted_apogee = last_predicted_apogee;
            if (isnan(predicted_apogee)){
                LOG_WRN("control run with Nan apogee value, loop blocked");
                k_msleep(10);
                continue;
            }
            float error = predicted_apogee - TARGET_APOGEE_AGL;
            integral += error * SAMPLING_TIME_S;

            if (integral > PID_INTEGRAL_MAX) {
                    integral = PID_INTEGRAL_MAX;
            } else if (integral < -PID_INTEGRAL_MAX) {
                    integral = -PID_INTEGRAL_MAX;
                }
            float derivative = (error - last_error)/SAMPLING_TIME_S;
            float output = (PID_P_GAIN * error) + (PID_I_GAIN * integral) + (PID_D_GAIN * derivative);
            last_error = error;

             // Clamp the final output to range 0 to 1
            if (output > PID_OUTPUT_MAX) {
                    output = PID_OUTPUT_MAX;
            } else if (output < PID_OUTPUT_MIN) {
                    output = PID_OUTPUT_MIN;
                }

            // Block of code converting linear movement of airbrakes on the rails to degrees for the motor    
            float desired_deployment_mm = output * x_max;
            float A = desired_deployment_mm + x_ret;
            float num = (A * A) - (link_L * link_L - crank_R * crank_R);
            float den = 2 * crank_R * A;
            float arg = num / den;

            if (arg > 1.0f) arg = 1.0f;
            if (arg < -1.0f) arg = 1.0f; // TODO: ask about this

            float theta = asinf(arg) * (180.0f / 3.14159f);
            control->airbrakes_angle = theta;

        } else {
            //to prevent integral windup
            integral = 0.0f;
            last_error = 0.0f;
            control->airbrakes_angle = 0.0f;
            }
        
        k_msleep(10); // 100 Hz
    }
}