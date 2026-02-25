#pragma once

#include <zsl/interp.h>   /* zsl_interp_lin_y_arr() */
#include <zephyr/kernel.h>
#include "filter.h"
#include "flight_state.h"
#include <zephyr/zbus/zbus.h>

ZBUS_CHAN_DECLARE(aero_chan);

typedef struct fjalar fjalar_t;
//typedef struct init init_t;
//typedef struct state state_t;

typedef struct aerodynamics_msg_matrix {
    float data[3];  // 3x1 vector
    size_t sz_rows;
    size_t sz_cols;
} aerodynamics_msg_matrix;

typedef struct aerodynamics_output_msg {
    uint32_t timestamp;
    float drag_data[3]; // acceleration, not force
    struct aerodynamics_msg_matrix drag; // drag induced acceleration in local frame
    float drag_norm;
    float expected_apogee;
    uint8_t thrust_bool;
    float g_physics;
    float temperature_kelvin;
    float specific_gas_constant_air;
    float heat_capacity_ratio_air;
    float v_sound;
    float mach_number;
} aerodynamics_output_msg;


/*
typedef struct aerodynamics {
    zsl_real_t drag_data[3]; // acceleration, not force!
    struct zsl_mtx drag; // drag induced acceleration in local frame
    float drag_norm;
    float expected_apogee; // predicted apogee based on drag model
    bool thrust_bool; // true if there is thrust, false if no thrust.
    float g_physics;

    float temperature_kelvin;
    float specific_gas_constant_air;
    float heat_capacity_ratio_air;
    float v_sound;
    float mach_number;
} aerodynamics_t; */

void init_aerodynamics(fjalar_t *fjalar);

#ifndef AERODYNAMICS_H
#define AERODYNAMICS_H

// change these values to Freyja values
#define MASS_DRY  9 //38 // Freyja
#define GRAVITY 9.81
#define AREA 0.008 // 0.022 // Freyja

static const struct zsl_interp_xy cd_tbl[] = {
    {3.43, 0.445392077},
    {6.86, 0.443396227},
    {10.29, 0.429917833},
    {13.72, 0.423512584},
    {17.15, 0.420851543},
    {20.58, 0.417786495},
    {24.01, 0.414716498},
    {27.44, 0.411778402},
    {30.87, 0.409014917},
    {34.3, 0.40642762},
    {37.73, 0.404018232},
    {41.16, 0.401769047},
    {44.59, 0.399665926},
    {48.02, 0.397695141},
    {51.45, 0.395843979},
    {54.88, 0.394101055},
    {58.31, 0.392456144},
    {61.74, 0.390900263},
    {65.17, 0.389425391},
    {68.6, 0.388024502},
    {72.03, 0.386691304},
    {75.46, 0.385420273},
    {78.89, 0.38420643},
    {82.32, 0.383045391},
    {85.75, 0.381933173},
    {89.18, 0.381039561},
    {92.61, 0.380188027},
    {96.04, 0.379375709},
    {99.47, 0.378600005},
    {102.9, 0.377858488},
    {106.33, 0.377149034},
    {109.76, 0.376469684},
    {113.19, 0.375818617},
    {116.62, 0.375194204},
    {120.05, 0.374594941},
    {123.48, 0.374019381},
    {126.91, 0.373466277},
    {130.34, 0.372934425},
    {133.77, 0.372422742},
    {137.2, 0.371930191},
    {140.63, 0.371455842},
    {144.06, 0.370998799},
    {147.49, 0.370558256},
    {150.92, 0.37013344},
    {154.35, 0.36972365},
    {157.78, 0.369328181},
    {161.21, 0.368946434},
    {164.64, 0.368577814},
    {168.07, 0.368221762},
    {171.5, 0.367877794},
    {174.93, 0.367545363},
    {178.36, 0.367224045},
    {181.79, 0.366913391},
    {185.22, 0.366612997},
    {188.65, 0.36634992},
    {192.08, 0.366476374},
    {195.51, 0.36660295},
    {198.94, 0.366729602},
    {202.37, 0.366856351},
    {205.8, 0.366983199},
    {209.23, 0.367423765},
    {212.66, 0.367853163},
    {216.09, 0.368282658},
    {219.52, 0.368712249},
    {222.95, 0.369141935},
    {226.38, 0.369571718},
    {229.81, 0.370001595},
    {233.24, 0.370431589},
    {236.67, 0.370861656},
    {240.1, 0.371291839},
    {243.53, 0.371722071},
    {246.96, 0.372152374},
    {250.39, 0.372582704},
    {253.82, 0.373013126},
    {257.25, 0.373443552},
    {260.68, 0.373874026},
    {264.11, 0.374304546},
    {267.54, 0.374735114},
    {270.97, 0.375165728},
    {274.4, 0.375596365},
    {277.83, 0.376027049},
    {281.26, 0.376457756},
    {284.69, 0.376888531},
    {288.12, 0.37731935},
    {291.55, 0.37775017},
    {294.98, 0.378181056},
    {298.41, 0.378611964},
    {301.84, 0.379042915},
    {305.27, 0.379473887},
    {308.7, 0.379904925},
    {312.13, 0.382488278},
    {315.56, 0.390238339},
    {318.99, 0.407904087},
    {322.42, 0.431610494},
    {325.85, 0.4553169},
    {329.28, 0.479023306},
    {332.71, 0.502729713},
    {336.14, 0.526436119},
    {339.57, 0.550142525},
    {343, 0.573848932},
    {346.43, 0.597555338},
    {349.86, 0.621261744},
    {353.29, 0.644968151},
    {356.72, 0.668674557},
    {360.15, 0.692380963},
    {363.58, 0.691060853},
    {367.01, 0.689751285},
    {370.44, 0.68845212},
    {373.87, 0.687163217},
    {377.3, 0.685884435},
    {380.73, 0.684615533},
    {384.16, 0.683356467},
    {387.59, 0.682107195},
    {391.02, 0.680867275},
    {394.45, 0.679636963},
    {397.88, 0.677524215},
    {401.31, 0.675391688},
    {404.74, 0.673272037},
    {408.17, 0.671165119},
    {411.6, 0.669070889},
    {415.03, 0.666989003},
    {418.46, 0.664919516},
    {421.89, 0.662862183},
    {425.32, 0.66081706},
    {428.75, 0.6587839},
    {432.18, 0.656767527},
    {435.61, 0.65477127},
    {439.04, 0.652791393},
    {442.47, 0.650827029}
};

#define CD_N (sizeof(cd_tbl)/sizeof(cd_tbl[0]))

static inline zsl_real_t cd_at(zsl_real_t v)
{
    zsl_real_t cd;
    zsl_interp_lin_y_arr((struct zsl_interp_xy *)cd_tbl, CD_N, v, &cd);
    return cd;
}

// air density at altitude, works up to 10 km
static const struct zsl_interp_xy rho_tbl[] = {
    {    0, 1.22500}, {  500, 1.16730}, { 1000, 1.11160}, { 1500, 1.05810},
    { 2000, 1.00640}, { 2500, 0.95710}, { 3000, 0.90990}, { 3500, 0.86460},
    { 4000, 0.82120}, { 4500, 0.77940}, { 5000, 0.73930}, { 5500, 0.70080},
    { 6000, 0.66380}, { 6500, 0.62830}, { 7000, 0.59410}, { 7500, 0.56120},
    { 8000, 0.52950}, { 8500, 0.49890}, { 9000, 0.46920}, { 9500, 0.44060},
    {10000, 0.41310}
};
#define RHO_N   (sizeof(rho_tbl) / sizeof(rho_tbl[0]))

/* ρ(h)   – linear interpolate in the ISA table. */
static inline zsl_real_t air_density_at(zsl_real_t z)
{
    /* Clamp negative altitudes to sea level. */
    if (z < 0.0f) z = 0.0f;

    zsl_real_t rho;
    zsl_interp_lin_y_arr((struct zsl_interp_xy *)rho_tbl, RHO_N, z, &rho);
    return rho;
}

#endif /* AERODYNAMICS_H */
