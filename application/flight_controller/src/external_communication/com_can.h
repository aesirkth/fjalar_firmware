#include "fjalar.h"

typedef struct fjalar fjalar_t;
typedef struct init init_t;
typedef struct position_filter position_filter_t;
typedef struct attitude_filter attitude_filter_t;
typedef struct aerodynamics aerodynamics_t;
typedef struct state state_t;

typedef struct can {
    // Loki rx info
    uint8_t loki_state;
    uint8_t loki_sub_state;
    float loki_angle;
    float loki_battery_voltage;
    uint32_t loki_latest_rx_time;

    // GCS commands (redundancy for LoRa)
    bool CAN_GCS_READY_INITIATE_FJALAR;
    bool CAN_GCS_READY_LAUNCH_FJALAR;
    uint32_t gcs_ready_initiate_latest_rx_time;
    uint32_t gcs_ready_launch_latest_rx_time;

    bool can_started;
    uint32_t loki_latest_tx_time;
    //uint32_t sigurd_latest_tx_time;
} can_t;


extern const struct device *const can_dev;

void init_can(fjalar_t *fjalar);
