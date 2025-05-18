#pragma once

#include <zsl/matrices.h>
#include "fjalar.h"

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

    float lat0;
    float lon0;
    float alt0;
    float g;
    float pressure_ground;
    float expected_apogee;

    float pressure;

    zsl_real_t drag_data[3];
    struct zsl_mtx drag;

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

    float phi;
    float theta;
    float psi;

    uint32_t previous_update_gyroscope;
    bool seeded;
} attitude_filter_t;



void position_filter_init(position_filter_t *pos_kf, init_t *init);
void position_filter_accelerometer(position_filter_t *pos_kf, attitude_filter_t *att_kf, float ax, float ay, float az, uint32_t time);
void position_filter_barometer(position_filter_t *pos_kf, float pressure, uint32_t time);
void position_filter_gps(position_filter_t *pos_kf, float lat, float lon, float alt, uint32_t time);

void attitude_filter_init(attitude_filter_t *att_kf, init_t *init);
void attitude_filter_gyroscope(position_filter_t *pos_kf, attitude_filter_t *att_kf, float gx, float gy, float gz, uint32_t time);
void attitude_filter_accelerometer_ground(attitude_filter_t *att_kf, position_filter_t *pos_kf, float ax, float ay, float az, uint32_t time);
void attitude_filter_accelerometer_cruise(attitude_filter_t *att_kf, position_filter_t *pos_kf, float ax, float ay, float az, uint32_t time);

float filter_get_altitude(position_filter_t *pos_kf);
float filter_get_velocity(position_filter_t *pos_kf);
float filter_get_acceleration(position_filter_t *pos_kf);

void Pmtx_analysis(position_filter_t *pos_kf);
