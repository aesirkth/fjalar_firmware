#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <math.h>
#include <pla.h>

#include "fjalar.h"
#include "sensors.h"
#include "filter.h"

LOG_MODULE_REGISTER(flight, CONFIG_APP_FLIGHT_LOG_LEVEL);

#define FLIGHT_THREAD_PRIORITY 7
#define FLIGHT_THREAD_STACK_SIZE 4096

#define ALTITUDE_PRIMARY_WINDOW_SIZE 5
#define ALTITUDE_SECONDARY_WINDOW_SIZE 40

#define IMU_WINDOW_SIZE 11

#define BOOST_ACCEL_THRESHOLD 15.0
#define BOOST_SPEED_THRESHOLD 15.0

#define COAST_ACCEL_THRESHOLD 5.0
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

static float pressure_to_altitude(float pressure) {
    // ISA constants
    const double P0 = 101.3250;     // Sea-level pressure in Pa
    const double T0 = 288.15;       // Sea-level temperature in K
    const double L = 0.0065;        // Temperature lapse rate in K/m
    const double g = 9.81;          // Gravitational acceleration constant in m/s^2
    const double R = 287.05;        // Specific gas constant for dry air in J/(kg·K)

    // Calculate altitude
    double altitude = (T0 / L) * (1.0 - pow(pressure / P0, R * L / g));

    return altitude;
}

static void deploy_drogue(fjalar_t *fjalar) {
    fjalar->apogee_at = k_uptime_get_32();
    LOG_WRN("drogue deployed at %fm %fs", fjalar->altitude - fjalar->ground_level, (fjalar->apogee_at - fjalar->liftoff_at) / 1000.0);
}

static void deploy_main(fjalar_t *fjalar) {
    LOG_WRN("Main deployed at %f", fjalar->altitude - fjalar->ground_level);
}

static void evaluate_state(fjalar_t *fjalar) {
    switch (fjalar->flight_state) {
    case STATE_IDLE:
        //we chillin'
        break;
    case STATE_LAUNCHPAD:
        if (fjalar->az > BOOST_ACCEL_THRESHOLD) {
            fjalar->flight_state = STATE_BOOST;
            fjalar->liftoff_at = k_uptime_get_32();
            LOG_WRN("Changing state to BOOST due to acceleration");
        }
        if (fjalar->velocity > BOOST_SPEED_THRESHOLD) {
            fjalar->flight_state = STATE_BOOST;
            fjalar->liftoff_at = k_uptime_get_32();
            LOG_WRN("Changing state to BOOST due to speed");
        }
        break;
    case STATE_BOOST:
        if (fjalar->az < COAST_ACCEL_THRESHOLD) {
            fjalar->flight_state = STATE_COAST;
            LOG_WRN("Changing state to COAST due to acceleration");
        }
        if (fjalar->velocity < 0) {
            LOG_WRN("Fake pressure increase due to sonic shock wave");
        }
        break;
    case STATE_COAST:
        if (fjalar->velocity < 0) {
            deploy_drogue(fjalar);
            fjalar->flight_state = STATE_FREE_FALL;
            LOG_WRN("Changing state to FREE_FALL due to speed");
        }
        break;
    case STATE_FREE_FALL:
        if (k_uptime_get_32() - fjalar->apogee_at > DROGUE_DEPLOYMENT_FAILURE_DELAY) {
            // vec3 acc = {fjalar->ax, fjalar->ay, fjalar->az};
        }
        break;
    case STATE_DROGUE_DESCENT:
        deploy_main(fjalar);
        break;
    case STATE_MAIN_DESCENT:
        break;
    case STATE_LANDED:
        break;
    }
}

void flight_state_thread(fjalar_t *fjalar, void *p2, void *p1) {
    struct k_poll_event events[2] = {
        K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                                        K_POLL_MODE_NOTIFY_ONLY,
                                        &pressure_msgq),
        K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                                        K_POLL_MODE_NOTIFY_ONLY,
                                        &imu_msgq),
    };

    fjalar->ground_level = 0;
    fjalar->ax = 0;
    fjalar->ay = 0;
    fjalar->az = 9.8;
    fjalar->altitude = 0;
    fjalar->velocity = 0;
    altitude_filter_t altitude_filter;
    altitude_filter_init(&altitude_filter);

    window_t ax_filter;
    window_t ay_filter;
    window_t az_filter;
    float ax_window_data[IMU_WINDOW_SIZE];
    float ay_window_data[IMU_WINDOW_SIZE];
    float az_window_data[IMU_WINDOW_SIZE];
    window_init(&ax_filter, ax_window_data, IMU_WINDOW_SIZE);
    window_init(&ay_filter, ay_window_data, IMU_WINDOW_SIZE);
    window_init(&az_filter, az_window_data, IMU_WINDOW_SIZE);
    struct pressure_queue_entry pressure;
    struct imu_queue_entry imu;

    // k_poll(&events[0], 1, K_FOREVER);
    // k_poll(&events[1], 1, K_FOREVER);
    events[0].state = K_POLL_STATE_NOT_READY;
    events[1].state = K_POLL_STATE_NOT_READY;
    quat acc_correction_quat = {0, 0, 0, 1};

    #ifdef CONFIG_BOOT_STATE_LAUNCHPAD
    fjalar->flight_state = STATE_LAUNCHPAD;
    #endif
    #ifdef CONFIG_BOOT_STATE_IDLE
    fjalar->flight_state = STATE_IDLE;
    #endif
    while (true) {
        if (k_poll(events, 2, K_MSEC(1000))) {
            LOG_ERR("Stopped receiving measurements");
            continue;
        }

        if (k_msgq_get(&pressure_msgq, &pressure, K_NO_WAIT) == 0) {
            events[0].state = K_POLL_STATE_NOT_READY;
            float raw_altitude = pressure_to_altitude(pressure.pressure);
            if (fjalar->flight_state == STATE_LAUNCHPAD || fjalar->flight_state == STATE_BOOST
            || fjalar->flight_state == STATE_COAST) {
                altitude_filter_update_accel(&altitude_filter, raw_altitude, fjalar->az - SENSOR_G / 1000000.0, pressure.t);
            } else {
                altitude_filter_update(&altitude_filter, raw_altitude, pressure.t);
            }
            fjalar->altitude = altitude_filter_get_altitude(&altitude_filter);
            fjalar->velocity = altitude_filter_get_velocity(&altitude_filter);
            if (fjalar->flight_state == STATE_LAUNCHPAD) {
                fjalar->ground_level = fjalar->altitude;
            }
            LOG_DBG("Altitude relative %f raw %f", fjalar->altitude - fjalar->ground_level, raw_altitude);
            LOG_DBG("velocity %f", fjalar->velocity);
        }
        if (k_msgq_get(&imu_msgq, &imu, K_NO_WAIT) == 0) {
            events[1].state = K_POLL_STATE_NOT_READY;
            window_insert(&ax_filter, imu.ax);
            window_insert(&ay_filter, imu.ay);
            window_insert(&az_filter, imu.az);
            float ax = window_get_median(&ax_filter);
            float ay = window_get_median(&ay_filter);
            float az = window_get_median(&az_filter);

            if (fjalar->flight_state == STATE_LAUNCHPAD
            || fjalar->flight_state == STATE_IDLE
            ) {
                vec3 measured_vector = {ax, ay, az};
                vec3_normalize(measured_vector);
                vec3 tmp_vec;
                float x_rot_angle = atan2f(measured_vector[1], measured_vector[2]);
                quat x_rot_quat = {
                    sinf(x_rot_angle / 2),
                    0,
                    0,
                    cosf(x_rot_angle / 2),
                };
                tmp_vec[0] = measured_vector[0];
                tmp_vec[1] = measured_vector[1];
                tmp_vec[2] = measured_vector[2];
                quat_mul_vec3(measured_vector, x_rot_quat, tmp_vec);
                vec3_normalize(measured_vector);

                float y_rot_angle = -atan2f(measured_vector[0], measured_vector[2]); // why is this negative? only god knows
                quat y_rot_quat = {
                    0,
                    sinf(y_rot_angle / 2),
                    0,
                    cosf(y_rot_angle / 2),
                };
                tmp_vec[0] = measured_vector[0];
                tmp_vec[1] = measured_vector[1];
                tmp_vec[2] = measured_vector[2];
                LOG_DBG("x rot %f y rot %f", x_rot_angle, y_rot_angle);

                acc_correction_quat[0] = 0;
                acc_correction_quat[1] = 0;
                acc_correction_quat[2] = 0;
                acc_correction_quat[3] = 1;
                quat tmp;
                quat_mul(tmp, x_rot_quat, acc_correction_quat);
                quat_normalize(tmp);
                quat_mul(acc_correction_quat, y_rot_quat, tmp);
                quat_normalize(acc_correction_quat);
            }
            vec3 acceleration = {ax, ay, az};
            vec3 acceleration_corrected = {ax, ay, az};
            quat_mul_vec3(acceleration_corrected, acc_correction_quat, acceleration);
            // vec3_scale(acceleration_corrected, acc_correction_scale);
            fjalar->ax = acceleration_corrected[0];
            fjalar->ay = acceleration_corrected[1];
            fjalar->az = acceleration_corrected[2];
            if (fjalar->az < 0
            && (fjalar->flight_state == STATE_LAUNCHPAD)) {
                LOG_ERR("Acceleration is negative on the launchpad");
            }

            LOG_DBG("Acceleration: %f %f %f", fjalar->ax, fjalar->ay, fjalar->az);
        }
        evaluate_state(fjalar);
        LOG_DBG("state %d", fjalar->flight_state);
        // k_msleep(1);
    }
}