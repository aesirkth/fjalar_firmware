#pragma once

#define IMU_INIT_N 50
#define BARO_INIT_N 50
#define GPS_INIT_N 10

typedef struct fjalar fjalar_t;
typedef struct position_filter position_filter_t;
typedef struct attitude_filter attitude_filter_t;
typedef struct aerodynamics aerodynamics_t;
typedef struct state state_t;
extern struct k_sem init_complete_sem;

typedef struct init{
    // arrays
    float ax[IMU_INIT_N], ay[IMU_INIT_N], az[IMU_INIT_N], gx[IMU_INIT_N], gy[IMU_INIT_N], gz[IMU_INIT_N];
    float p[BARO_INIT_N];
    float lat[GPS_INIT_N], lon[GPS_INIT_N], alt[GPS_INIT_N];
    // mean and variances
    double mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, mean_p, mean_lon, mean_lat, mean_alt;
    double g_accelerometer;
    double var_ax, var_ay, var_az, var_gx, var_gy, var_gz, var_p, var_lon, var_lat, var_alt;

    uint16_t n_imu, n_baro, n_gps;

    float lat0;
    float lon0;
    float alt0;
    float pressure_ground;

    float roll0;
    float pitch0;
    float yaw0;

    int new_x_index;
    int new_y_index;
    int new_z_index;
    int new_x_sign;
    int new_y_sign;
    int new_z_sign;

    bool init_completed;

    float pressure;
    bool seeded;
} init_t;

void init_init(fjalar_t *fjalar);
