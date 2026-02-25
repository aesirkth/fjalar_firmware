#pragma once

#include "fjalar.h"
#include "filter.h"
#include "aerodynamics.h"

#define TARGET_APOGEE_AGL       3000.0f

#define SAMPLING_RATE_HZ        100.0f
#define SAMPLING_TIME_S         (1.0f / SAMPLING_RATE_HZ)

// PID Gains (These need to be tuned) 
#define PID_P_GAIN              2.0f
#define PID_I_GAIN              0.5f
#define PID_D_GAIN              0.1f

#define PID_INTEGRAL_MAX        500.0f //Used for anti windup
#define PID_OUTPUT_MAX          1.0f    // Max airbrake deployment is 1.0
#define PID_OUTPUT_MIN          0.0f    // Min airbrake deployment is 0.0

#define x_max                   26.0f
#define x_ret                   41.6f
#define crank_R                 21.0f
#define link_L                  46.6f

typedef struct fjalar fjalar_t;
typedef struct init init_t;

typedef struct control {
    float airbrakes_angle; // rename, idk what you guys need
} control_t;

void init_control(fjalar_t *fjalar);
