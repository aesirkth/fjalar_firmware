#include <zephyr/logging/log.h>
#include <math.h>
#include "aerodynamics.h"
#include "filter.h"

LOG_MODULE_REGISTER(aerodynamics, CONFIG_APP_AERODYNAMICS_LOG_LEVEL);

bool is_thrust_over(position_filter_t *pos_kf, attitude_filter_t *att_kf, aerodynamics_t *aerodynamics){ // will only be correct if run during flight, not before launch
    drag_update(pos_kf, att_kf, aerodynamics);
    float calculated_acceleration = aerodynamics->drag_norm - pos_kf->g;
    float estimated_acceleration  = filter_get_acceleration(pos_kf);

    float a_diff = fabsf(calculated_acceleration - estimated_acceleration); // diff between estimated and predicted acceleration
    LOG_INF("is thrust over?: %u", pos_kf->X_data[2]>10 && a_diff < 3);
    return (pos_kf->X_data[2]>10 && a_diff < 3);
}

float pressure_to_AGL(position_filter_t *pos_kf, aerodynamics_t *aerodynamics, float pressure){
    const float p0 = pos_kf->pressure_ground;
    const float T0 = 288.15f;
    const float L  = 0.0065f;
    const float g = 9.81f;
    const float R = 287.05f;

    float AGL = (T0/L) * (1-powf(pressure / p0, R * L / g));
    return AGL;
}

void drag_init(aerodynamics_t *aerodynamics){
    float drag_init[3] = {
        0,
        0,
        0
    };
    aerodynamics->drag.data = aerodynamics->drag_data;
    aerodynamics->drag.sz_rows = 3;
    aerodynamics->drag.sz_cols = 1;
    memcpy(aerodynamics->drag_data, drag_init, sizeof(drag_init));
}

void drag_update(position_filter_t *pos_kf, attitude_filter_t *att_kf, aerodynamics_t *aerodynamics){
    // velocity norm
    ZSL_MATRIX_DEF(v, 3, 1);
    v.data[0] = pos_kf->X_data[3];
    v.data[1] = pos_kf->X_data[4];
    v.data[2] = pos_kf->X_data[5];

    float v_norm = sqrtf(v.data[0]*v.data[0] + v.data[1]*v.data[1] + v.data[2]*v.data[2]);

    // velocity unit vector
    ZSL_MATRIX_DEF(v_unit, 3, 1);
    zsl_mtx_copy(&v_unit, &v);
    zsl_mtx_scalar_mult_d(&v_unit, 1.0f/v_norm);

    // air density
    float z = pos_kf->X_data[2];
    float rho = air_density_at(z);

    // drag coefficient
    float c_d;
    if (v_norm>5){c_d = cd_at(v_norm);} // TODO: implement angle dependent drag coeff!
    else{c_d = 0.44;}

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

    // scalar part
    float drag_norm = -(0.5/MASS_DRY)*rho*AREA*v_norm*v_norm*c_d;
    aerodynamics->drag_norm = drag_norm;

    // vector part
    ZSL_MATRIX_DEF(drag_vec, 3, 1);
    zsl_mtx_mult(&rotation, &v_unit, &drag_vec);

    // drag vector
    zsl_mtx_scalar_mult_d(&drag_vec, drag_norm);
    aerodynamics->drag = drag_vec;
    pos_kf->drag = drag_vec;
}

void update_apogee_estimate(position_filter_t *pos_kf, aerodynamics_t *aerodynamics){
    // get data 
    float dt   = 0.01;
    float a_z0  = pos_kf->X_data[8];

    float x    = pos_kf->X_data[0];
    float y    = pos_kf->X_data[1];
    float z    = pos_kf->X_data[2];
    float xy   = sqrtf(x*x + y*y);

    float v_x  = pos_kf->X_data[3];
    float v_y  = pos_kf->X_data[4];
    float v_z  = pos_kf->X_data[5];
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
    float apogee = z;
    LOG_INF("apogee estimate: %f", apogee);
    pos_kf->expected_apogee = apogee;

}
