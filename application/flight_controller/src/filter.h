#pragma once

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>
#include <zsl/matrices.h>
#include "fjalar.h"
#include "flight_state.h"

typedef struct fjalar fjalar_t;
typedef struct init init_t;
typedef struct state state_t;

typedef struct filter_output_msg {
    uint32_t timestamp;
    float position[3];    // x, y, z
    float velocity[3];    // vx, vy, vz
    float acceleration[3]; // ax, ay, az
    float attitude[3];     // roll, pitch, yaw
    float v_norm;
    float a_norm;
    float raw_imu[6];     // ax, ay, az, gx, gy, gz
    float raw_baro_p;
    float raw_gps[3];     // lat, lon, alt
}filter_output_msg;

ZBUS_CHAN_DECLARE(filter_output_zchan);

typedef struct position_filter {
    zsl_real_t P_data[81];
    struct zsl_mtx P;
    zsl_real_t X_data[9];
    struct zsl_mtx X;
    zsl_real_t Q_data[81];
    struct zsl_mtx Q;
    zsl_real_t R_data[1];
    struct zsl_mtx R;
    zsl_real_t T_data[9];
    struct zsl_mtx T;
    zsl_real_t Ptwosigma_data[9];
    struct zsl_mtx Ptwosigma;

    float raw_imu_ax;
    float raw_imu_ay;
    float raw_imu_az;

    float raw_baro_p;

    float raw_gps_lat;
    float raw_gps_lon;
    float raw_gps_alt;

    float a_norm;
    float v_norm;

    uint32_t previous_update_accelerometer;
    bool seeded;
} position_filter_t;

typedef struct attitude_filter {
    zsl_real_t P_data[9];
    struct zsl_mtx P;
    zsl_real_t X_data[9];
    struct zsl_mtx X;
    zsl_real_t Q_data[9];
    struct zsl_mtx Q;
    zsl_real_t R_data[9];
    struct zsl_mtx R;

    float phi; // remove? Not used 
    float theta; // remove? Not used
    float psi; // remove? Not used

    float raw_imu_gx;
    float raw_imu_gy;
    float raw_imu_gz;

    uint32_t previous_update_gyroscope;
    bool seeded;
} attitude_filter_t;

void init_filter(fjalar_t *fjalar);

