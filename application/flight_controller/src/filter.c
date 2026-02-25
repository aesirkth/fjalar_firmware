/*
This is the filter script, its purpose is to:
1) Using state estimation and data given from the sensors, estimate the rockets position, velocity, acceleration, and orientation.
For this we use the linear Kalman filter (KF) and the nonlinear extended Kalman filter (EKF)
*/

#include <string.h>
#include <math.h>
#include <zsl/matrices.h>
#include <pla.h>
#include <stdint.h>
#include <zephyr/logging/log.h>
#include <zsl/statistics.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/zbus/zbus.h>

#include "fjalar.h"
#include "sensors.h"
#include "init.h"
#include "filter.h"
#include "flight_state.h"
// #include "control.h"

LOG_MODULE_REGISTER(filter, LOG_LEVEL_INF);

#define FILTER_THREAD_PRIORITY 7
#define FILTER_THREAD_STACK_SIZE 4096

void filter_thread(fjalar_t *fjalar, void *p2, void *p1);
ZBUS_CHAN_DEFINE(filter_output_zchan, /* Name */
		struct filter_output_msg, /* Message type */
		NULL, /* Validator */
		NULL, /* User Data */
		ZBUS_OBSERVERS_EMPTY, /* observers */
		ZBUS_MSG_INIT(.timestamp = 0) /* Initial value */
);
K_THREAD_STACK_DEFINE(filter_thread_stack, FILTER_THREAD_STACK_SIZE);
struct k_thread filter_thread_data;
k_tid_t filter_thread_id;


void init_filter(fjalar_t *fjalar) {
    filter_thread_id = k_thread_create(
		&filter_thread_data,
		filter_thread_stack,
		K_THREAD_STACK_SIZEOF(filter_thread_stack),
		(k_thread_entry_t) filter_thread,
		fjalar, NULL, NULL,
		FILTER_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(filter_thread_id, "filter");
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
TODO:
- Add theta0 and phi0 instead of placeholders (original coordinates)
*/

void position_filter_init(position_filter_t *pos_kf, init_t *init) {
    float position_process_variance = 0.01;
    float ax_variance = 1.0; //0.01;
    float ay_variance = 1.0; //0.01;
    float az_variance = 1.0; //0.01;
    float pressure_variance = 1000.0;
    float lon_variance = 0.001; // measure these
    float lat_variance = 0.001; // measure these
    float alt_variance = 0.001; // measure these

    /*
    float ax_variance = init->var_ax;
    float ay_variance = init->var_ay;
    float az_variance = init->var_az;
    float pressure_variance = init->var_p;
    float lon_variance = init->var_lon;
    float lat_variance = init->var_lat;
    float alt_variance = init->var_alt;
    */


    float P_init[81] = {
        position_process_variance, 0, 0, 0, 0, 0, 0, 0, 0,
        0, position_process_variance, 0, 0, 0, 0, 0, 0, 0,
        0, 0, position_process_variance, 0, 0, 0, 0, 0, 0,
        0, 0, 0, position_process_variance, 0, 0, 0, 0, 0,
        0, 0, 0, 0, position_process_variance, 0, 0, 0, 0,
        0, 0, 0, 0, 0, position_process_variance, 0, 0, 0,
        0, 0, 0, 0, 0, 0, position_process_variance, 0, 0, 
        0, 0, 0, 0, 0, 0, 0, position_process_variance, 0,
        0, 0, 0, 0, 0, 0, 0, 0, position_process_variance
    };
    pos_kf->P.data = pos_kf->P_data;
    pos_kf->P.sz_rows = 9;
    pos_kf->P.sz_cols = 9;
    memcpy(pos_kf->P_data, P_init, sizeof(P_init));

    float X_init[9] = {
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
    };
    pos_kf->X.data = pos_kf->X_data;
    pos_kf->X.sz_rows = 9;
    pos_kf->X.sz_cols = 1;
    memcpy(pos_kf->X_data, X_init, sizeof(X_init));

    // IMU noise matrix
    float Q_init[81] = {
        ax_variance, 0, 0, 0, 0, 0, 0, 0, 0,
        0, ay_variance, 0, 0, 0, 0, 0, 0, 0,
        0, 0, az_variance, 0, 0, 0, 0, 0, 0,
        0, 0, 0, ax_variance, 0, 0, 0, 0, 0,
        0, 0, 0, 0, ay_variance, 0, 0, 0, 0,
        0, 0, 0, 0, 0, az_variance, 0, 0, 0,
        0, 0, 0, 0, 0, 0, ax_variance, 0, 0,
        0, 0, 0, 0, 0, 0, 0, ay_variance, 0,
        0, 0, 0, 0, 0, 0, 0, 0, az_variance
    };
    pos_kf->Q.data = pos_kf->Q_data;
    pos_kf->Q.sz_rows = 9;
    pos_kf->Q.sz_cols = 9;
    memcpy(pos_kf->Q_data, Q_init, sizeof(Q_init));

    // Barometer noise matrix
    float R_init[1] = {
        pressure_variance
    };
    pos_kf->R.data = pos_kf->R_data;
    pos_kf->R.sz_rows = 1;
    pos_kf->R.sz_cols = 1;
    memcpy(pos_kf->R_data, R_init, sizeof(R_init));

    // GPS noise matrix
    float T_init[9] = {
        lon_variance, 0, 0,
        0, lat_variance, 0,
        0, 0, alt_variance
    };
    pos_kf->T.data = pos_kf->T_data;
    pos_kf->T.sz_rows = 3;
    pos_kf->T.sz_cols = 3;
    memcpy(pos_kf->T_data, T_init, sizeof(T_init));

    pos_kf->seeded = false;

    // 2*standard_deviation (sigma) matrix derived from P matrix.
    float Ptwosigma_init[9] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0
    };
    pos_kf->Ptwosigma.data = pos_kf->Ptwosigma_data;
    pos_kf->Ptwosigma.sz_rows = 9;
    pos_kf->Ptwosigma.sz_cols = 1;
    memcpy(pos_kf->Ptwosigma_data, Ptwosigma_init, sizeof(Ptwosigma_init));
}


void position_filter_accelerometer(init_t *init, position_filter_t *pos_kf, attitude_filter_t *att_kf, float ax, float ay, float az, uint32_t time){
    if (!pos_kf->seeded) {
        pos_kf->previous_update_accelerometer = time;
        pos_kf->seeded = true;
        return;
    }
    float dt = (time - pos_kf->previous_update_accelerometer) / 1000.0;
    pos_kf->previous_update_accelerometer = time;
    
    // A matrix
    zsl_real_t A_data[81] = {
        1, 0, 0, dt, 0, 0, 0, 0, 0,
        0, 1, 0, 0, dt, 0, 0, 0, 0,
        0, 0, 1, 0, 0, dt, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0
    };
    struct zsl_mtx A = {
        .sz_rows = 9,
        .sz_cols = 9,
        .data = A_data
    };

    // A transpose
    float AT_data[81];

    struct zsl_mtx AT = {
        .sz_rows = 9,
        .sz_cols = 9,
        .data = AT_data
    };

    zsl_mtx_trans(&A, &AT);

    // B matrix
    zsl_real_t B_data[27] = {
        0.5*dt*dt, 0, 0,
        0, 0.5*dt*dt, 0,
        0, 0, 0.5*dt*dt,
        dt, 0, 0,
        0, dt, 0,
        0, 0, dt,
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };

    struct zsl_mtx B = {
        .sz_rows = 9,
        .sz_cols = 3,
        .data = B_data
    };

    // u matrix vector
    zsl_real_t u_data[3] = {
        ax,
        ay,
        az
    };
    struct zsl_mtx u = {
        .sz_rows = 3,
        .sz_cols = 1,
        .data = u_data
    };


    // rotational matrix
    float phi = att_kf->X_data[0];
    float theta = att_kf->X_data[1];
    float psi = att_kf->X_data[2];

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

    // prediction calculations
    // rotate u given an attitude
    ZSL_MATRIX_DEF(u_rot, 3, 1);
    zsl_mtx_mult(&rotation, &u, &u_rot);
    u_rot.data[2] = u_rot.data[2] - init->g_accelerometer; // correct for gravity (accelerometers gravity)

    //LOG_WRN("az rotated into global frame: %f", u_rot.data[2]);
    // X
    ZSL_MATRIX_DEF(AX, 9, 1);
    ZSL_MATRIX_DEF(BU, 9, 1);

    zsl_mtx_mult(&A, &pos_kf->X, &AX);
    zsl_mtx_mult(&B, &u_rot, &BU);
    zsl_mtx_add(&AX, &BU, &pos_kf->X);

    // P
    ZSL_MATRIX_DEF(AP, 9, 9);
    ZSL_MATRIX_DEF(APAT, 9, 9);
    zsl_mtx_mult(&A, &pos_kf->P, &AP);
    zsl_mtx_mult(&AP, &AT, &APAT);
    zsl_mtx_add(&APAT, &pos_kf->Q, &pos_kf->P);

};


// Correction with barometer
void position_filter_barometer(init_t *init, position_filter_t *pos_kf, float pressure_kpa, uint32_t time){
    // z matrix
    zsl_real_t z_data[1] = {
        pressure_kpa * 1000 // kPa to Pa
    };
    struct zsl_mtx z = {
        .sz_rows = 1,
        .sz_cols = 1,
        .data = z_data
    };

    // Identity matrix 9x9
    zsl_real_t I9_data[81] = {
        1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1,
    };
    struct zsl_mtx I9 = {
        .sz_rows = 9,
        .sz_cols = 9,
        .data = I9_data,
    };

    /*EKF STEPS*/
    // create h(x)
    const float P0 = init->pressure_ground;
    const float T0 = 288.15f;
    const float L  = 0.0065f;
    const float g = 9.81f;
    const float R = 287.05f;

    float base_h = 1 - (L * pos_kf->X_data[2] / T0);
    float pressure_h = P0 * powf(base_h, g / (R * L));
    ZSL_MATRIX_DEF(hx, 1, 1);
    hx.data[0] = pressure_h;

    // create H (jacobian of h(x))
    float dpdh;

    float base_H = 1 - (L * pos_kf->X_data[2] / T0);
    float exponent_H = (g / (R * L)) - 1.0f;

    dpdh = -P0 * (g / (R * T0)) * powf(base_H, exponent_H);
    ZSL_MATRIX_DEF(H, 1, 9);
    for (int i = 0; i<9; i++){
        H.data[i] = 0;
    }
    H.data[2] = dpdh;

    ZSL_MATRIX_DEF(HT, 9, 1);
    zsl_mtx_trans(&H, &HT);

    // correction calculation
    // y
    ZSL_MATRIX_DEF(y, 1, 1);
    zsl_mtx_sub(&z, &hx, &y);
    //LOG_INF("P altitude variance: %f", pos_kf->P_data[20]);


    // S
    ZSL_MATRIX_DEF(HP, 1, 9);
    ZSL_MATRIX_DEF(HPHT, 1, 1);
    ZSL_MATRIX_DEF(S, 1, 1);
    zsl_mtx_mult(&H, &pos_kf->P, &HP);
    zsl_mtx_mult(&HP, &HT, &HPHT);
    zsl_mtx_add(&HPHT, &pos_kf->R, &S);

    // K
    ZSL_MATRIX_DEF(PHT, 9, 1);
    ZSL_MATRIX_DEF(S_inv, 1, 1);
    ZSL_MATRIX_DEF(K, 9, 1);
    zsl_mtx_mult(&pos_kf->P, &HT, &PHT);
    zsl_mtx_inv(&S, &S_inv);
    zsl_mtx_mult(&PHT, &S_inv, &K);

    // X
    ZSL_MATRIX_DEF(Ky, 9, 1);
    zsl_mtx_mult(&K, &y, &Ky);
    zsl_mtx_add(&pos_kf->X, &Ky, &pos_kf->X);

    // P
    ZSL_MATRIX_DEF(KH, 9, 9);
    ZSL_MATRIX_DEF(I9KH, 9, 9);
    zsl_mtx_mult(&K, &H, &KH);
    zsl_mtx_sub(&I9, &KH, &I9KH);
    zsl_mtx_mult(&I9KH, &pos_kf->P, &pos_kf->P);
};

// Correction with gps
void position_filter_gps(init_t *init, position_filter_t *pos_kf, float lat, float lon, float alt, uint32_t time){
    float phi0 = init->lon0; // longitude of launchpad
    float theta0 = init->lat0; // Latitude of launchpad
    float alt0 = init->alt0;
    float R = 6370000; // Radius earth placeholder

    // H matrix
    float deg_per_rad = 180.0f / M_PI;
    float lon_scale = deg_per_rad / (R * cosf(theta0 * (M_PI / 180.0f)));
    float lat_scale = deg_per_rad / R;
    zsl_real_t H_data[27] = {
        lon_scale, 0, 0, 0, 0, 0, 0, 0, 0,
        0, lat_scale, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0
    };
    struct zsl_mtx H = {
        .sz_rows = 3,
        .sz_cols = 9,
        .data = H_data
    };

    // H offset
    zsl_real_t Hoffset_data[3] = {
        phi0, // lon
        theta0, // lat
        alt0 // conversion from ASL to AGL
    };

    struct zsl_mtx Hoffset = {
        .sz_rows = 3,
        .sz_cols = 1,
        .data = Hoffset_data
    };

    // H matrix transponate
    float HT_data[27];
    struct zsl_mtx HT = {
        .sz_rows = 9,
        .sz_cols = 3,
        .data = HT_data
    };
    zsl_mtx_trans(&H, &HT);

    // z matrix
    zsl_real_t z_data[3] = {
        lon,
        lat,
        alt
    };
    struct zsl_mtx z = {
        .sz_rows = 3,
        .sz_cols = 1,
        .data = z_data
    };

    // Identity matrix 9x9
    zsl_real_t I9_data[81] = {
        1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1
    };
    struct zsl_mtx I9 = {
        .sz_rows = 9,
        .sz_cols = 9,
        .data = I9_data,
    };

    // correction calculation
    // y
    ZSL_MATRIX_DEF(HX, 3, 1);
    ZSL_MATRIX_DEF(HXHo, 3, 1);
    ZSL_MATRIX_DEF(y, 3, 1);
    zsl_mtx_mult(&H, &pos_kf->X, &HX);
    zsl_mtx_add(&HX, &Hoffset, &HXHo);
    zsl_mtx_sub(&z, &HXHo, &y);

    // S
    ZSL_MATRIX_DEF(HP, 3, 9);
    ZSL_MATRIX_DEF(HPHT, 3, 3);
    ZSL_MATRIX_DEF(S, 3, 3);
    zsl_mtx_mult(&H, &pos_kf->P, &HP);
    zsl_mtx_mult(&HP, &HT, &HPHT);
    zsl_mtx_add(&HPHT, &pos_kf->T, &S);

    // K
    ZSL_MATRIX_DEF(PHT, 9, 3);
    ZSL_MATRIX_DEF(S_inv, 3, 3);
    ZSL_MATRIX_DEF(K, 9, 3);
    zsl_mtx_mult(&pos_kf->P, &HT, &PHT);
    zsl_mtx_inv(&S, &S_inv);
    zsl_mtx_mult(&PHT, &S_inv, &K);

    // X
    ZSL_MATRIX_DEF(Ky, 9, 1);
    zsl_mtx_mult(&K, &y, &Ky);
    zsl_mtx_add(&pos_kf->X, &Ky, &pos_kf->X);

    // P
    ZSL_MATRIX_DEF(KH, 9, 9);
    ZSL_MATRIX_DEF(I9KH, 9, 9);
    zsl_mtx_mult(&K, &H, &KH);
    zsl_mtx_sub(&I9, &KH, &I9KH);
    zsl_mtx_mult(&I9KH, &pos_kf->P, &pos_kf->P);
};

void Pmtx_analysis(position_filter_t *pos_kf){
    for (int i = 0; i < 9; i++) {
        pos_kf->Ptwosigma_data[i] = 2*sqrtf(pos_kf->P_data[9*i+i]);
    }
}





// Attitude Filter
void attitude_filter_init(attitude_filter_t *att_kf, init_t *init) {
    float attitude_process_variance = 100;
    float ax_variance = 0.03;
    float ay_variance = 0.03;
    float az_variance = 0.03;
    float gx_variance = 1.03;
    float gy_variance = 1.03;
    float gz_variance = 1.03;
    /*
    float ax_variance = init->var_ax;
    float ay_variance = init->var_ay;
    float az_variance = init->var_az;
    float gx_variance = init->var_gx;
    float gy_variance = init->var_gy;
    float gz_variance = init->var_gz;
    */

    float P_init[9] = {
        attitude_process_variance, 0, 0,
        0, attitude_process_variance, 0,
        0, 0, attitude_process_variance

    };
    att_kf->P.data = att_kf->P_data;
    att_kf->P.sz_rows = 3;
    att_kf->P.sz_cols = 3;
    memcpy(att_kf->P_data, P_init, sizeof(P_init));

    float X_init[3] = {
        0,
        0,
        0
    };
    att_kf->X.data = att_kf->X_data;
    att_kf->X.sz_rows = 3;
    att_kf->X.sz_cols = 1;
    memcpy(att_kf->X_data, X_init, sizeof(X_init));

    //  gyroscope variance matrix
    float Q_init[9] = {
        gx_variance, 0, 0,
        0, gy_variance, 0,
        0, 0, gz_variance
    };
    att_kf->Q.data = att_kf->Q_data;
    att_kf->Q.sz_rows = 3;
    att_kf->Q.sz_cols = 3;
    memcpy(att_kf->Q_data, Q_init, sizeof(Q_init));

    // accelerometer variance matrix
    float R_init[9] = {
        ax_variance, 0, 0,
        0, ay_variance, 0,
        0, 0, az_variance
    };
    att_kf->R.data = att_kf->R_data;
    att_kf->R.sz_rows = 3;
    att_kf->R.sz_cols = 3;
    memcpy(att_kf->R_data, R_init, sizeof(R_init));

    att_kf->seeded = false;
}


// prediction with gyroscope
void attitude_filter_gyroscope(position_filter_t *pos_kf, attitude_filter_t *att_kf, float gx, float gy, float gz, uint32_t time){
    if (!att_kf->seeded) {
        att_kf->previous_update_gyroscope = time;
        att_kf->seeded = true;
        return;
    }

    float dt = (time - att_kf->previous_update_gyroscope) / 1000.0;
    att_kf->previous_update_gyroscope = time;

    // Identity matrix 3x3
    zsl_real_t I3_data[9] = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };
    struct zsl_mtx I3 = {
        .sz_rows = 3,
        .sz_cols = 3,
        .data = I3_data,
    };

    /*EKF STEPS*/
    // create f(x)
    float phi = att_kf->X_data[0];
    float theta = att_kf->X_data[1];
    float sp = sin(phi);
    float cp = cos(phi);
    float ct = cos(theta);
    float tt = tan(theta);
    ZSL_MATRIX_DEF(fx, 3, 1);
    fx.data[0] = (gx + gy*sp*tt + gz*cp*tt)*dt;
    fx.data[1] = (gy*cp - gz*sp)*dt;
    fx.data[2] = ((gy*sp / ct) + (gz*cp / ct))*dt;

    // create F(x)
    float sect  = 1.0 / ct;
    float sec2t = sect*sect;

    ZSL_MATRIX_DEF(Mdt, 3, 3);
    Mdt.data[0] = (gy*cp*tt - gz*sp*tt)*dt;
    Mdt.data[1] = (gy*sp*sec2t + gz*cp*sec2t)*dt;
    Mdt.data[2] = 0;

    Mdt.data[3] = (-gy*sp - gz*cp)*dt;
    Mdt.data[4] = 0;
    Mdt.data[5] = 0;
    Mdt.data[6] = (gy*cp*sect - gz*sp*sect)*dt;
    Mdt.data[7] = (gy*sp*tt*sect + gz*cp*tt*sect)*dt;
    Mdt.data[8] = 0;

    ZSL_MATRIX_DEF(F, 3, 3);
    ZSL_MATRIX_DEF(FT, 3, 3);
    zsl_mtx_add(&I3, &Mdt, &F);
    zsl_mtx_trans(&F, &FT);

    // prediction calculations
    // X
    zsl_mtx_add(&att_kf->X, &fx, &att_kf->X);

    // P
    ZSL_MATRIX_DEF(FP, 3, 3);
    ZSL_MATRIX_DEF(FPFT, 3, 3);
    zsl_mtx_mult(&F, &att_kf->P, &FP);
    zsl_mtx_mult(&FP, &FT, &FPFT);
    zsl_mtx_add(&FPFT, &att_kf->Q, &att_kf->P);

    // Normalize radians to[-pi, pi] range
    for (int i=0; i<3; i++){
        if (att_kf->X_data[i]>M_PI){
            att_kf->X_data[i] = att_kf->X_data[i]-2*M_PI;
        }
        if (att_kf->X_data[i]<-M_PI){
            att_kf->X_data[i] = att_kf->X_data[i]+2*M_PI;
        }
    }
};


void attitude_filter_accelerometer_ground(init_t *init, attitude_filter_t *att_kf, position_filter_t *pos_kf, float ax, float ay, float az, uint32_t time){
    if (az > 12){return;} // Guards against start of boost phase, before state update (!)

    /* 
    There is a risk of this correction step being run while rocket is acceleration but before 
    the state is updated to STATE_BOOST, thus ruining attitude state. Although the guard at the very start of this function 
    should fix that.
    */

    // z matrix
    zsl_real_t z_data[3] = {
        ax,
        ay,
        az
    };
    struct zsl_mtx z = {
        .sz_rows = 3,
        .sz_cols = 1,
        .data = z_data
    };

    // Identity matrix 3x3
    zsl_real_t I3_data[9] = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };
    struct zsl_mtx I3 = {
        .sz_rows = 3,
        .sz_cols = 3,
        .data = I3_data
    };

    // EKF STEPS
    // create h(x)
    float g = init->g_accelerometer;

    float phi = att_kf->X_data[0];
    float theta = att_kf->X_data[1];

    float sp = sin(phi);
    float cp = cos(phi);
    float st = sin(theta);
    float ct = cos(theta);
    
    ZSL_MATRIX_DEF(hx, 3, 1); // Is not correct for accelerating system
    hx.data[0] = -g*st;
    hx.data[1] = g*sp*ct;
    hx.data[2] = g*cp*ct;

    // create H (jacobian of h(x))
    ZSL_MATRIX_DEF(H, 3, 3);
    H.data[0] = 0;
    H.data[1] = -g*ct;
    H.data[2] = 0;

    H.data[3] = g*cp*ct;
    H.data[4] = -g*st*sp;
    H.data[5] = 0;

    H.data[6] = -g*sp*ct;
    H.data[7] = -g*st*cp;
    H.data[8] = 0;

    ZSL_MATRIX_DEF(HT, 3, 3);
    zsl_mtx_trans(&H, &HT);

    // correction calculation
    // y
    ZSL_MATRIX_DEF(y, 3, 1);
    zsl_mtx_sub(&z, &hx, &y);

    // S
    ZSL_MATRIX_DEF(HP, 3, 3);
    ZSL_MATRIX_DEF(HPHT, 3, 3);
    ZSL_MATRIX_DEF(S, 3, 3);
    zsl_mtx_mult(&H, &att_kf->P, &HP);
    zsl_mtx_mult(&HP, &HT, &HPHT);
    zsl_mtx_add(&HPHT, &att_kf->R, &S);

    // K
    ZSL_MATRIX_DEF(PHT, 3, 3);
    ZSL_MATRIX_DEF(S_inv, 3, 3);
    ZSL_MATRIX_DEF(K, 3, 3);
    zsl_mtx_mult(&att_kf->P, &HT, &PHT);
    zsl_mtx_inv(&S, &S_inv);
    zsl_mtx_mult(&PHT, &S_inv, &K);

    // X
    ZSL_MATRIX_DEF(Ky, 3, 1);
    zsl_mtx_mult(&K, &y, &Ky);
    zsl_mtx_add(&att_kf->X, &Ky, &att_kf->X);

    // P
    ZSL_MATRIX_DEF(KH, 3, 3);
    ZSL_MATRIX_DEF(I3KH, 3, 3);
    zsl_mtx_mult(&K, &H, &KH);
    zsl_mtx_sub(&I3, &KH, &I3KH);
    zsl_mtx_mult(&I3KH, &att_kf->P, &att_kf->P);

    //LOG_INF("y 0: %f, y 1: %f, y 2: %f", y.data[0], y.data[1], y.data[2]);
    //LOG_INF("z 0 : %f", z_data[0]);
    //LOG_INF("z 1 : %f", z_data[1]);
    //LOG_INF("z 2 : %f", z_data[2]);

    //LOG_INF("Hx 0: %f", hx.data[0]);
    //LOG_INF("Hx 1: %f", hx.data[1]);
    //LOG_INF("Hx 2: %f", hx.data[2]);
};



void update_velocity_norm(position_filter_t *pos_kf) {
    float vx = pos_kf->X_data[3];
    float vy = pos_kf->X_data[4];
    float vz = pos_kf->X_data[5];
    if (vz>0){
        pos_kf->v_norm = sqrtf(vx*vx + vy*vy + vz*vz);
    } else{ 
        pos_kf->v_norm = -sqrtf(vx*vx + vy*vy + vz*vz);
    }
    
}

void update_acceleration_norm(position_filter_t *pos_kf) {
    float ax = pos_kf->X_data[6];
    float ay = pos_kf->X_data[7];
    float az = pos_kf->X_data[8];
    if (az>0){
        pos_kf->a_norm = sqrtf(ax*ax + ay*ay + az*az);
    } else{
        pos_kf->a_norm = -sqrtf(ax*ax + ay*ay + az*az);
    }
}

void zero_velocity(position_filter_t *pos_kf){ 
    /* since Fjalar do not have a correcting step for the velocity part of the state vector, 
    this is needed to avoid drift on launchpad. Only run up intil boost*/
    pos_kf->X_data[3] = 0.0f;
    pos_kf->X_data[4] = 0.0f;
    pos_kf->X_data[5] = 0.0f;
}
void filter_thread(fjalar_t *fjalar, void *p2, void *p1) {
    init_t            *init  = fjalar->ptr_init;
    // control_t         *control = fjalar->ptr_control; unused
    position_filter_t pos_kf_l;
    attitude_filter_t att_kf_l;
    position_filter_t *pos_kf = &pos_kf_l;
    attitude_filter_t *att_kf = &att_kf_l;
    
    struct imu_queue_entry imu;
    struct pressure_queue_entry pressure;
    // struct gps_queue_entry gps; unused

    position_filter_init(pos_kf, init);
    attitude_filter_init(att_kf, init);

    struct flight_state_output_msg fs_msg;
    enum fjalar_flight_state flight_state = STATE_INITIATED;
    enum fjalar_velocity_class velocity_class = VELOCITY_SUBSONIC;

    while (true) {
        // Update flight state if new messages exist
        while (zbus_chan_read(&flight_state_output_zchan, &fs_msg, K_NO_WAIT) == 0) {
            flight_state = fs_msg.flight_state;
            velocity_class = fs_msg.velocity_class;
        }

        if (zbus_chan_read(&imu_zchan, &imu, K_NO_WAIT) == 0) { // gets IMU data if available
            // for external communication
            pos_kf->raw_imu_ax = imu.ax;
            pos_kf->raw_imu_ay = imu.ay;
            pos_kf->raw_imu_az = imu.az;
            att_kf->raw_imu_gx = imu.gx;
            att_kf->raw_imu_gy = imu.gy;
            att_kf->raw_imu_gz = imu.gz;

            // correct for IMU mounting inside of rocket - works for any number of rotations of 90 degrees.
            float a_array[3] = {imu.ax, imu.ay, imu.az};
            float g_array[3] = {imu.gx, imu.gy, imu.gz};

            float ax = a_array[init->new_x_index] * init->new_x_sign; 
            float ay = a_array[init->new_y_index] * init->new_y_sign; 
            float az = a_array[init->new_z_index] * init->new_z_sign;
            
            float gx = g_array[init->new_x_index] * init->new_x_sign; 
            float gy = g_array[init->new_y_index] * init->new_y_sign; 
            float gz = g_array[init->new_z_index] * init->new_z_sign; 

            //LOG_INF("ax: %f, ay: %f, az: %f", ax, ay, az);

            // call filters
            
            position_filter_accelerometer(init, pos_kf, att_kf, ax, ay, az, imu.t); // needs magnetometer to not get usage of ax and ay
            attitude_filter_gyroscope(pos_kf, att_kf, gx, gy, gz, imu.t);

            // use state machine TODO: remove state idle since filter.c is not actually being run in that state
            if (flight_state == STATE_INITIATED || flight_state == STATE_AWAITING_LAUNCH){
                attitude_filter_accelerometer_ground(init, att_kf, pos_kf, ax, ay, az, imu.t);
            }
        }

        if (zbus_chan_read(&pressure_zchan, &pressure, K_NO_WAIT) == 0) {            
            pos_kf->raw_baro_p = pressure.pressure*1000;
            // use state machine
            if (velocity_class == VELOCITY_SUBSONIC){ // baro bad in native
                position_filter_barometer(init, pos_kf, pressure.pressure, pressure.t); // Ask other team about barometer solution on bad data
            }          
        }


        #if DT_ALIAS_EXISTS(gps_uart)
        {
            struct gps_queue_entry gps;
            if (zbus_chan_read(&gps_zchan, &gps, K_NO_WAIT) == 0 && !isnan(gps.lat) && !isnan(gps.lon) && !isnan(gps.alt)) {
                pos_kf->raw_gps_lat = gps.lat;
                pos_kf->raw_gps_lon = gps.lon;
                pos_kf->raw_gps_alt = gps.alt;
                #if !DT_ALIAS_EXISTS(hilsensor) // remove when gps has been added to hilsensor
                //position_filter_gps(init, pos_kf, gps.lat, gps.lon, gps.alt, gps.t);
                #endif



            } else {
                //LOG_WRN("GPS sample dropped (NaN/Inf)");
            }
        }
        #endif

        if (flight_state == STATE_INITIATED || flight_state == STATE_AWAITING_LAUNCH){
            // filter.c is only run after initialization, which is why these two states above are sufficient
            zero_velocity(pos_kf);
        }

        update_velocity_norm(pos_kf);
        update_acceleration_norm(pos_kf);

        Pmtx_analysis(pos_kf);


        /*
        static int counter = 0;
        if (counter%10 == 0){LOG_INF("vz: %f", pos_kf->X_data[5]);}
        counter++;
        */


        //LOG_WRN("v_norm: %f", pos_kf->v_norm);
        //LOG_WRN("a_norm: %f", pos_kf->a_norm);
        //LOG_WRN("x : %f, y : %f, z : %f", pos_kf->X_data[0], pos_kf->X_data[1], pos_kf->X_data[2]);
        //LOG_WRN("vx: %f, vy: %f, vz: %f", pos_kf->X_data[3], pos_kf->X_data[4], pos_kf->X_data[5]);
        //LOG_WRN("ax: %f, ay: %f, az: %f", pos_kf->X_data[6], pos_kf->X_data[7], pos_kf->X_data[8]);
        //LOG_WRN("roll: %fπ, pitch: %fπ, yaw: %fπ", att_kf->X_data[0]/3.14, att_kf->X_data[1]/3.14, att_kf->X_data[2]/3.14);
        static int counter = 0;
        if (counter % 1 == 0){
            //LOG_WRN("Pvx: %2.f, Pvy: %2.f, Pvz: %2.f", pos_kf->P_data[30], pos_kf->P_data[40], pos_kf->P_data[50]);
            //LOG_WRN("axy: %.2f, vxy: %.2f, xy: %.2f", sqrtf(pos_kf->X_data[6]*pos_kf->X_data[6]+pos_kf->X_data[7]*pos_kf->X_data[7]), sqrtf(pos_kf->X_data[3]*pos_kf->X_data[3]+pos_kf->X_data[4]*pos_kf->X_data[4]), sqrtf(pos_kf->X_data[0]*pos_kf->X_data[0]+pos_kf->X_data[1]*pos_kf->X_data[1]));
        }
        counter++;
        struct filter_output_msg msg = {
            .timestamp = k_uptime_get_32(),
            .position = {pos_kf->X_data[0], pos_kf->X_data[1], pos_kf->X_data[2]},
            .velocity = {pos_kf->X_data[3], pos_kf->X_data[4], pos_kf->X_data[5]},
            .acceleration = {pos_kf->X_data[6], pos_kf->X_data[7], pos_kf->X_data[8]},
            .attitude = {att_kf->X_data[0], att_kf->X_data[1], att_kf->X_data[2]},
            .v_norm = pos_kf->v_norm,
            .a_norm = pos_kf->a_norm,
            .raw_imu = {pos_kf->raw_imu_ax, pos_kf->raw_imu_ay, pos_kf->raw_imu_az, att_kf->raw_imu_gx, att_kf->raw_imu_gy, att_kf->raw_imu_gz},
            .raw_baro_p = pos_kf->raw_baro_p,
            .raw_gps = {pos_kf->raw_gps_lat, pos_kf->raw_gps_lon, pos_kf->raw_gps_alt}
        };

        // Non-blocking publish - don't want to stall filter if channel is busy
        if (zbus_chan_pub(&filter_output_zchan, &msg, K_NO_WAIT) != 0) {
            // Channel busy - this means consumers are too slow
            LOG_WRN("Filter output zbus channel busy, dropping message");
        } else {
            static int pub_counter = 0;
            if (pub_counter % 100 == 0) {  // Log every 1 second (100Hz loop)
                LOG_INF("Published: alt=%.2f, v_norm=%.2f",
                        msg.position[2], msg.v_norm);
            }
            pub_counter++;
        }
        
        k_msleep(10); // 100 Hz
        }
    }
