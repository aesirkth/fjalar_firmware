// #include <alloca.h>
#include <string.h>
#include <math.h>
#include <zsl/matrices.h>
// #include <pla.h>
#include <stdint.h>
#include <zephyr/logging/log.h>
#include "filter.h"
#include <zsl/statistics.h>
#include "aerodynamics.h"

LOG_MODULE_REGISTER(filter, CONFIG_APP_FILTER_LOG_LEVEL);

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
TODO:
- Add theta0 and phi0 instead of placeholders (original coordinates)
*/
void position_filter_init(position_filter_t *pos_kf, init_t *init) {
    float process_variance = 0.01;
    float ax_variance = 0.01;
    float ay_variance = 0.01;
    float az_variance = 0.01;
    float pressure_variance = 92;
    float lon_variance = 0.001;
    float lat_variance = 0.001;
    float alt_variance = 0.001;

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
        process_variance, 0, 0, 0, 0, 0, 0, 0, 0,
        0, process_variance, 0, 0, 0, 0, 0, 0, 0,
        0, 0, process_variance, 0, 0, 0, 0, 0, 0,
        0, 0, 0, process_variance, 0, 0, 0, 0, 0,
        0, 0, 0, 0, process_variance, 0, 0, 0, 0,
        0, 0, 0, 0, 0, process_variance, 0, 0, 0,
        0, 0, 0, 0, 0, 0, process_variance, 0, 0,
        0, 0, 0, 0, 0, 0, 0, process_variance, 0,
        0, 0, 0, 0, 0, 0, 0, 0, process_variance
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

// Correction with accelerometer | TODO: need to rotate accel vector
void position_filter_accelerometer(position_filter_t *pos_kf, attitude_filter_t *att_kf, float ax, float ay, float az, uint32_t time){
    if (!pos_kf->seeded) {
        pos_kf->previous_update_accelerometer = time;
        pos_kf->seeded = true;
        return;
    }
    float dt = (time - pos_kf->previous_update_accelerometer) / 1000.0;
    pos_kf->previous_update_accelerometer = time;

    // A matrix
    zsl_real_t A_data[81] = {
        1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0, 0,
        0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0,
        0, 0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt,
        0, 0, 0, 1, 0, 0, dt, 0, 0,
        0, 0, 0, 0, 1, 0, 0, dt, 0,
        0, 0, 0, 0, 0, 1, 0, 0, dt,
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
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
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
    u_rot.data[2] = u_rot.data[2] - pos_kf->g; // correct for gravity
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
void position_filter_barometer(position_filter_t *pos_kf, float pressure_kpa, uint32_t time){
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
    const float P0 = pos_kf->pressure_ground;
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
void position_filter_gps(position_filter_t *pos_kf, float lat, float lon, float alt, uint32_t time){
    float phi0 = pos_kf->lon0; // longitude of launchpad
    float theta0 = pos_kf->lat0; // Latitude of launchpad
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
        0 // potential ASL to AGL difference here
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
    float process_variance = 0.01;
    float ax_variance = 0.03;
    float ay_variance = 0.03;
    float az_variance = 0.03;
    float gx_variance = 0.03;
    float gy_variance = 0.03;
    float gz_variance = 0.03;
    /*
    float ax_variance = init->var_ax;
    float ay_variance = init->var_ay;
    float az_variance = init->var_az;
    float gx_variance = init->var_gx;
    float gy_variance = init->var_gy;
    float gz_variance = init->var_gz;
    */

    float P_init[9] = {
        process_variance, 0, 0,
        0, process_variance, 0,
        0, 0, process_variance

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

    gx *= (M_PI/180);
    gy *= (M_PI/180);
    gz *= (M_PI/180);

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


void attitude_filter_accelerometer_ground(attitude_filter_t *att_kf, position_filter_t *pos_kf, float ax, float ay, float az, uint32_t time){
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
    float g = pos_kf->g;

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

    y.data[0] = 0; // remove this later (just for troubleshooting)
    y.data[1] = 0;

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
};

void attitude_filter_accelerometer_cruise(attitude_filter_t *att_kf, position_filter_t *pos_kf, float ax, float ay, float az, uint32_t time){
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
    ZSL_MATRIX_DEF(hx, 3, 1); // Is not correct for accelerating system

    // create H (jacobian of h(x))
    ZSL_MATRIX_DEF(H, 3, 3);

    

    ZSL_MATRIX_DEF(HT, 3, 3);
    zsl_mtx_trans(&H, &HT);

    // correction calculation
    // y
    ZSL_MATRIX_DEF(y, 3, 1);
    zsl_mtx_sub(&z, &hx, &y);

    y.data[0] = 0; // remove this later (just for troubleshooting)
    y.data[1] = 0;

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
}






float filter_get_altitude(position_filter_t *pos_kf) {
    return pos_kf->X_data[2];
}

float filter_get_velocity(position_filter_t *pos_kf) {
    float vx = pos_kf->X_data[3];
    float vy = pos_kf->X_data[4];
    float vz = pos_kf->X_data[5];
    float v_norm;
    if (vz>0){v_norm = sqrtf(vx*vx + vy*vy + vz*vz);}
    else{v_norm = -sqrtf(vx*vx + vy*vy + vz*vz);}
    return v_norm;
}

float filter_get_acceleration(position_filter_t *pos_kf) {
    float ax = pos_kf->X_data[6];
    float ay = pos_kf->X_data[7];
    float az = pos_kf->X_data[8];
    float a_norm;
    if (az>0){a_norm = sqrtf(ax*ax + ay*ay + az*az);}
    else{a_norm = -sqrtf(ax*ax + ay*ay + az*az);}
    return a_norm;
}
