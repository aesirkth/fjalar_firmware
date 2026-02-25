/*
This is the aerodynamics script, its purpose is to:
1) Given the state estimate and the flight state, perform analysis related to aerodynamics such as finding out expected apogee,
then save this in its struct to be used by other scripts.
The aerodynamic constants are rocket specific, it is critical that these are looked over carefully before each launch.
*/

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <pla.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <math.h>

#include "fjalar.h"
#include "sensors.h"
#include "init.h"
#include "filter.h"
#include "aerodynamics.h"
#include "flight_state.h"

LOG_MODULE_REGISTER(aerodynamics, LOG_LEVEL_INF);

#define AERODYNAMICS_THREAD_PRIORITY 7
#define AERODYNAMICS_THREAD_STACK_SIZE 4096

void aerodynamics_thread(fjalar_t *fjalar, void *p2, void *p1);
ZBUS_CHAN_DEFINE(aero_chan,                          /* Name */
                 struct aerodynamics_output_msg,     /* Message type */
                 NULL,                               /* Validator */
                 NULL,                               /* User Data */
                 ZBUS_OBS_DECLARE(),                 /* Observers (can be empty if using read) */
                 ZBUS_MSG_INIT(0)                    /* Initial Value */
);
K_THREAD_STACK_DEFINE(aerodynamics_thread_stack, AERODYNAMICS_THREAD_STACK_SIZE);
struct k_thread aerodynamics_thread_data;
k_tid_t aerodynamics_thread_id;


void init_aerodynamics(fjalar_t *fjalar) {
    aerodynamics_thread_id = k_thread_create(
		&aerodynamics_thread_data,
		aerodynamics_thread_stack,
		K_THREAD_STACK_SIZEOF(aerodynamics_thread_stack),
		(k_thread_entry_t) aerodynamics_thread,
		fjalar, NULL, NULL,
		AERODYNAMICS_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(aerodynamics_thread_id, "aerodynamics");
}


// might come in handy in future
/*
float pressure_to_AGL(position_filter_t *pos_kf, aerodynamics_t *aerodynamics, float pressure){
    const float p0 = init->pressure_ground;
    const float T0 = 288.15f;
    const float L  = 0.0065f;
    const float g = 9.81f;
    const float R = 287.05f;

    float AGL = (T0/L) * (1-powf(pressure / p0, R * L / g));
    return AGL;
}
*/

void drag_init(struct aerodynamics_output_msg *aerodynamics){
    float drag_init[3] = {0,0,0};
    // First initialize drag_data to zeros
    memcpy(aerodynamics->drag_data, drag_init, sizeof(drag_init));
    // Then copy from drag_data to drag.data (both are arrays)
    memcpy(aerodynamics->drag.data, aerodynamics->drag_data, sizeof(aerodynamics->drag_data));
    // Set the matrix dimensions
    aerodynamics->drag.sz_rows = 3;
    aerodynamics->drag.sz_cols = 1;
}

void drag_update(struct filter_output_msg *filter_data, struct aerodynamics_output_msg *aerodynamics){
    // velocity norm
    ZSL_MATRIX_DEF(v, 3, 1);
    v.data[0] = filter_data->velocity[0];
	v.data[1] = filter_data->velocity[1];
	v.data[2] = filter_data->velocity[2];
    float v_norm = filter_data->v_norm;

    if (v_norm > 1e-6f){

        // velocity unit vector
        ZSL_MATRIX_DEF(v_unit, 3, 1);
        zsl_mtx_copy(&v_unit, &v);
        zsl_mtx_scalar_mult_d(&v_unit, 1.0f/v_norm);

        // air density
        float z = filter_data->position[2];
        float rho = air_density_at(z);

        // drag coefficient
        float c_d;
        if (v_norm>5){c_d = cd_at(v_norm);} // TODO: implement angle dependent drag coeff!
        else{c_d = 0.44;}

        // rotational matrix
        float phi   = filter_data->attitude[0];  // roll
		float theta = filter_data->attitude[1];  // pitch
		float psi   = filter_data->attitude[2];  // yaw


        float sp = sinf(phi), cp = cosf(phi);
        float st = sinf(theta), ct = cosf(theta);
        float ss = sinf(psi),  cs = cosf(psi);

        float rotation_data[9] = {
            ct*cs, sp*st*cs - cp*ss, cp*st*cs + sp*ss,
            ct*ss, sp*st*ss + cp*cs, cp*st*ss - sp*cs,
            -st, sp*ct, cp*ct
        };
        struct zsl_mtx rotation = {
            .sz_rows = 3,
            .sz_cols = 3,
            .data = rotation_data
        };

        // scalar part
        float drag_norm = -(0.5/MASS_DRY)*rho*AREA*v_norm*v_norm*c_d;
        aerodynamics->drag_norm = drag_norm;

        // vector part
        ZSL_MATRIX_DEF(drag_vec, 3, 1);
        zsl_mtx_mult(&rotation, &v_unit, &drag_vec);

        // drag vector
        zsl_mtx_scalar_mult_d(&drag_vec, drag_norm);

		memcpy(aerodynamics->drag_data, drag_vec.data, sizeof(aerodynamics->drag_data));
        aerodynamics->drag.sz_rows = 3;
        aerodynamics->drag.sz_cols = 1;
    } else{
    	aerodynamics->drag_norm = 0.0f;
    	aerodynamics->drag_data[0] = aerodynamics->drag_data[1] = aerodynamics->drag_data[2] = 0.0f;
	}
}

void update_thrust(struct filter_output_msg *filter_data, struct aerodynamics_output_msg *aerodynamics, struct flight_state_output_msg state_data){ // will only be correct if run during flight, not before launch

    float N; // normal force induced acceleration
    if (state_data.flight_state == STATE_INITIATED){
        N = aerodynamics->g_physics;
    } else{N = 0;}
    float g = -(aerodynamics->g_physics); // gravitational acceleration
    float D = aerodynamics->drag_norm; // drag acceleration
    float T = 0.0f; // Thrust
    float total_acceleration = filter_data->a_norm;

    if (state_data.flight_state == STATE_INITIATED){
        T = total_acceleration - N;
    }
    if (state_data.flight_state == STATE_BOOST || state_data.flight_state == STATE_COAST){
        T = total_acceleration - g - D;
    }

    if (fabsf(T)>3){
        aerodynamics->thrust_bool = 1; // thrust exists
    } else{
        aerodynamics->thrust_bool = 0; // thrust does not exist
    }
}

void update_apogee_estimate(struct filter_output_msg *filter_data, struct aerodynamics_output_msg *aerodynamics){
    // get data 
    float dt   = 0.01;
    // float a_z0  = filter_data->acceleration[2]; unused

    float x    = filter_data->position[0];
    float y    = filter_data->position[1];
    float z    = filter_data->position[2];
    float xy   = sqrtf(x*x + y*y);

    float v_x  = filter_data->velocity[0];
    float v_y  = filter_data->velocity[1];
    float v_z  = filter_data->velocity[2];
    float v_xy = sqrtf(v_x*v_x + v_y*v_y);

    float a_xy;
    float a_z;

    while (v_z>0){
        // get info
        float v_total = sqrtf((v_xy*v_xy)+(v_z*v_z));
        float c_d;
        float a_drag;

        if (v_total>5){c_d = cd_at(v_total);} 
        else{c_d = 0.44;}

        a_drag = - (air_density_at(z)*c_d*AREA*v_total*v_total) / (2*MASS_DRY); 
        
        float c_b = air_density_at(z)*v_total*v_total / (2*fabsf(a_drag));


        a_xy  = 0.0f;
        a_z   = -GRAVITY-(air_density_at(z)*(v_z*v_z))/(2*c_b);

        v_xy += dt * a_xy;
        v_z  += dt * a_z;

        xy   += dt * v_xy;
        z    += dt * v_z;
    }
    aerodynamics->expected_apogee = z;

}

void update_mach_number(struct filter_output_msg *filter_data, struct aerodynamics_output_msg *aerodynamics){
    float v_sound = sqrtf(aerodynamics->specific_gas_constant_air * aerodynamics->heat_capacity_ratio_air * aerodynamics->temperature_kelvin);
    aerodynamics->v_sound = v_sound;

    float mach_number = (filter_data->v_norm)/v_sound;
    aerodynamics->mach_number = mach_number;
}


void aerodynamics_thread(fjalar_t *fjalar, void *p2, void *p1) {
	struct filter_output_msg filter_data;
	struct flight_state_output_msg state_data = { .flight_state = STATE_IDLE }; // Initialize with default

    // Initialize aerodynamics message with constants
    struct aerodynamics_output_msg aero_msg = {
        .timestamp = 0,
        .drag_norm = 0.0f,
        .expected_apogee = 0.0f,
        .thrust_bool = 0,
        .g_physics = 9.81f,
        .specific_gas_constant_air = 287.0f, // J/(kg*K)
        .heat_capacity_ratio_air = 1.4f,
        .temperature_kelvin = 273.15f + 15.0f,
        .mach_number = 0.0f,
        .v_sound = 0.0f
    };

    drag_init(&aero_msg);
    while (true){

		if (zbus_chan_read(&filter_output_zchan, &filter_data, K_FOREVER) == 0) {
			zbus_chan_read(&flight_state_output_zchan, &state_data, K_NO_WAIT);
    		// got new data
			aero_msg.timestamp = k_uptime_get_32();

        	update_mach_number(&filter_data, &aero_msg);

        	drag_update(&filter_data, &aero_msg); // needs to be updated at all states due to logic in update_thrust

			if (state_data.flight_state != STATE_IDLE && state_data.flight_state != STATE_AWAITING_INIT){
            	update_thrust(&filter_data, &aero_msg, state_data);
        	}
        	if (state_data.flight_state == STATE_COAST){ // only run when thrust is not active
            	update_apogee_estimate(&filter_data, &aero_msg);
        	} else{aero_msg.expected_apogee = 0.0f;}
        	// update_mach_number(&filter_data, &aero_msg);
			// After all calculations (drag_update, update_thrust, etc.)
			zbus_chan_pub(&aero_chan, &aero_msg, K_MSEC(10));
		}
        //zbus_chan_pub(&aero_chan, &aero_msg, K_MSEC(10));

        //k_msleep(10); // 100 Hz
    }
}