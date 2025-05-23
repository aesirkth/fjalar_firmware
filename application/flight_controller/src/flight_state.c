#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <math.h>
#include <pla.h>

#include "fjalar.h"
#include "sensors.h"
#include "filter.h"
#include "aerodynamics.h"

LOG_MODULE_REGISTER(flight, CONFIG_APP_FLIGHT_LOG_LEVEL);

#define FLIGHT_THREAD_PRIORITY 7
#define FLIGHT_THREAD_STACK_SIZE 4096

#define PERIODIC_THREAD_PRIORITY 7
#define PERIODIC_THREAD_STACK_SIZE 4096

#define ALTITUDE_PRIMARY_WINDOW_SIZE 5
#define ALTITUDE_SECONDARY_WINDOW_SIZE 40

#define IMU_WINDOW_SIZE 11

#define BOOST_ACCEL_THRESHOLD 15.0
#define BOOST_SPEED_THRESHOLD 15.0

#define COAST_ACCEL_THRESHOLD 5.0
#define DROGUE_DEPLOYMENT_FAILURE_DELAY 8000



position_filter_t pos_kf;
attitude_filter_t att_kf;
aerodynamics_t aerodynamics;
init_t init;

void flight_state_thread(fjalar_t *fjalar, void *p2, void *p1);
void periodic_thread(void *p1, void *p2, void *p3);

K_THREAD_STACK_DEFINE(flight_thread_stack, FLIGHT_THREAD_STACK_SIZE);
struct k_thread flight_thread_data;
k_tid_t flight_thread_id;

K_THREAD_STACK_DEFINE(periodic_thread_stack, PERIODIC_THREAD_STACK_SIZE);
struct k_thread periodic_thread_data;
k_tid_t periodic_thread_id;

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

    periodic_thread_id = k_thread_create(
        &periodic_thread_data,
        periodic_thread_stack,
        K_THREAD_STACK_SIZEOF(periodic_thread_stack),
        periodic_thread,
        NULL, NULL, NULL,
        PERIODIC_THREAD_PRIORITY, 0, K_NO_WAIT
    );
    k_thread_name_set(periodic_thread_id, "periodic thread");

}

//static init_t init = {0};

static inline bool init_ready(void){
    return init.n_imu  >= IMU_INIT_N  &&
           init.n_baro >= BARO_INIT_N &&
           init.n_gps  >= GPS_INIT_N;
}

/* converts sums -> means, writes them into fjalar  */ 
static void init_finish(fjalar_t *fjalar, position_filter_t *pos_kf, attitude_filter_t *att_kf, init_t *init){
    double sum_ax  = 0, sum_ay  = 0, sum_az  = 0;
    double sum_gx  = 0, sum_gy  = 0, sum_gz  = 0;
    double sum_lon = 0, sum_lat = 0, sum_alt = 0;
    double sum_p   = 0;

    double mean_ax, mean_ay, mean_az;
    double mean_gx, mean_gy, mean_gz;
    double mean_lon, mean_lat, mean_alt;
    double mean_p;

    double diff_ax  = 0, diff_ay  = 0, diff_az  = 0;
    double diff_gx  = 0, diff_gy  = 0, diff_gz  = 0;
    double diff_lon = 0, diff_lat = 0, diff_alt = 0;
    double diff_p   = 0;

    double var_ax, var_ay, var_az;
    double var_gx, var_gy, var_gz;
    double var_lon, var_lat, var_alt;
    double var_p;

    // Calculate variance: summarize deviation from mean squared, divide by N -> variance
    // calculate mean
    for (int i = 0; i<IMU_INIT_N; i++){
        sum_ax += init->ax[i];
        sum_ay += init->ay[i];
        sum_az += init->az[i];
        sum_gx += init->gx[i];
        sum_gy += init->gy[i];
        sum_gz += init->gz[i];}
    mean_ax = sum_ax/IMU_INIT_N;
    mean_ay = sum_ay/IMU_INIT_N;
    mean_az = sum_az/IMU_INIT_N;
    mean_gx = sum_gx/IMU_INIT_N;
    mean_gy = sum_gy/IMU_INIT_N;
    mean_gz = sum_gz/IMU_INIT_N;

    for (int i = 0; i<BARO_INIT_N; i++){
        sum_p += init->p[i];}
    mean_p = sum_p/BARO_INIT_N;

    for (int i = 0; i<GPS_INIT_N; i++){
        sum_lon += init->lon[i];
        sum_lat += init->lat[i];
        sum_alt += init->alt[i];}
    mean_lon = sum_lon/GPS_INIT_N;
    mean_lat = sum_lat/GPS_INIT_N;
    mean_alt = sum_alt/GPS_INIT_N;

    // calculate variance
    for (int i = 0; i<IMU_INIT_N; i++){
        diff_ax += (init->ax[i]-mean_ax)*(init->ax[i]-mean_ax);
        diff_ay += (init->ay[i]-mean_ay)*(init->ay[i]-mean_ay);
        diff_az += (init->az[i]-mean_az)*(init->az[i]-mean_az);
        diff_gx += (init->gx[i]-mean_gx)*(init->gx[i]-mean_gx);
        diff_gy += (init->gy[i]-mean_gy)*(init->gy[i]-mean_gy);
        diff_gz += (init->gz[i]-mean_gz)*(init->gz[i]-mean_gz);}
    var_ax = diff_ax/IMU_INIT_N;
    var_ay = diff_ay/IMU_INIT_N;
    var_az = diff_az/IMU_INIT_N;
    var_gx = diff_gx/IMU_INIT_N;
    var_gy = diff_gy/IMU_INIT_N;
    var_gz = diff_gz/IMU_INIT_N;

    for (int i = 0; i<BARO_INIT_N; i++){
        diff_p += (init->p[i]-mean_p)*(init->p[i]-mean_p);}
    var_p = diff_p/BARO_INIT_N;

    for (int i = 0; i<GPS_INIT_N; i++){
        diff_lon += (init->lon[i]-mean_lon)*(init->lon[i]-mean_lon);
        diff_lat += (init->lat[i]-mean_lat)*(init->lat[i]-mean_lat);
        diff_alt += (init->alt[i]-mean_alt)*(init->alt[i]-mean_alt);}  
    var_lon = diff_lon/GPS_INIT_N;
    var_lat = diff_lat/GPS_INIT_N;
    var_alt = diff_alt/GPS_INIT_N;
    
    // update fjalar struct (nice structure am I right?)
    init->mean_ax  =  mean_ax;
    init->mean_ay  =  mean_ay;
    init->mean_az  =  mean_az;
    init->mean_gx  =  mean_gx;
    init->mean_gy  =  mean_gy;
    init->mean_gz  =  mean_gz;
    init->mean_p   =   mean_p;
    init->mean_lon = mean_lon;
    init->mean_lat = mean_lat;
    init->mean_alt = mean_alt;

    init->var_ax   =   var_ax;
    init->var_ay   =   var_ay;
    init->var_az   =   var_az;
    init->var_gx   =   var_gx;
    init->var_gy   =   var_gy;
    init->var_gz   =   var_gz;
    init->var_p    =    var_p;
    init->var_lon  =  var_lon;
    init->var_lat  =  var_lat;
    init->var_alt  =  var_alt;
    LOG_INF("means: \nmean_ax: %f\nmean_ay: %f\nmean_az: %f\nmean_gx: %f\nmean_gy: %f\nmean_gz: %f\nmean_p: %f\nmean_lon: %f\nmean_lat: %f\nmean_alt: %f", mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, mean_p, mean_lon, mean_lat, mean_alt);
    LOG_INF("variances: \nvar_ax: %f\nvar_ay: %f\nvar_az: %f\nvar_gx: %f\nvar_gy: %f\nvar_gz: %f\nvar_p: %f\nvar_lon: %f\nvar_lat: %f\nvar_alt: %f", var_ax, var_ay, var_az, var_gx, var_gy, var_gz, var_p, var_lon, var_lat, var_alt);
    
    pos_kf->pressure_ground = init->mean_p;
    LOG_INF("ground level pressure: %f", pos_kf->pressure_ground);

    pos_kf->lat0  = init->mean_lat;
    pos_kf->lon0 = init->mean_lon ;
    pos_kf->alt0 = init->mean_alt;

    // identify g (to subtract it later)
    pos_kf->g = sqrtf(mean_ax*mean_ax + mean_ay*mean_ay + mean_az*mean_az);
    LOG_INF("g: %f", pos_kf->g);

    pos_kf->seeded = false;

    init->position_init = true;
    LOG_INF("init done: lat0 = %f, lon0 = %f", pos_kf->lat0, pos_kf->lon0);
    
    // flip all IMU readings to correct frame
    float a[3] = {mean_ax, mean_ay, mean_az};

    // find index of the max‐absolute axis
    int iz = 0;
    if ( fabsf(a[1]) > fabsf(a[iz]) ) iz = 1;
    if ( fabsf(a[2]) > fabsf(a[iz]) ) iz = 2;

    // index the other two indices, in cyclic order
    int ix = (iz + 1) % 3;
    int iy = (iz + 2) % 3;

    // extract & flip signs so that z is “up” and (x,y,z) is right‐handed
    int sz = (a[iz] >= 0.0f ? +1 : -1);
    // flipping x by the same amount preserves right‐handedness when z is flipped
    int sx = sz;  
    int sy = 1;    

    fjalar->new_x_index = ix;
    fjalar->new_y_index = iy;
    fjalar->new_z_index = iz;
    fjalar->new_x_sign = sx;
    fjalar->new_y_sign = sy;
    fjalar->new_z_sign = sz;

    // euler angles start
    float new_ax = a[ix] * sx; 
    float new_ay = a[iy] * sy; 
    float new_az = a[iz] * sz;
    
    att_kf->X_data[0] = atan2f(new_ay, new_az);
    att_kf->X_data[1] = atan2f(-new_ax, sqrtf(new_ay*new_ay + new_az*new_az));
    att_kf->X_data[2] = 0.0f;

    // add beep when init finish
}


static void deploy_drogue(fjalar_t *fjalar) {
    fjalar->apogee_at = k_uptime_get_32();
    LOG_WRN("drogue deployed at %fm %fs", fjalar->altitude - fjalar->ground_level, (fjalar->apogee_at - fjalar->liftoff_at) / 1000.0f);
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

// thread for numerical analysis
void periodic_thread(void *p1, void *p2, void *p3) {
    while (true) {
        Pmtx_analysis(&pos_kf);

        if (pos_kf.X_data[2]>10 && is_thrust_over(&pos_kf, &att_kf, &aerodynamics)){
            update_apogee_estimate(&pos_kf, &aerodynamics);
        } else{pos_kf.expected_apogee = 0;}

        uint32_t t_ms  = k_uptime_get_32();     // milliseconds since boot
        double   t_sec = (double)t_ms / 1000.0; // seconds

        k_msleep(10); // 10 ms = 100 Hz
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

    fjalar->ground_level = 0; // remove
    fjalar->ax = 0;
    fjalar->ay = 0;
    fjalar->az = 9.8;
    fjalar->altitude = 0;
    fjalar->velocity = 0;

    position_filter_init(&pos_kf, &init);
    attitude_filter_init(&att_kf, &init);
    drag_init(&aerodynamics);

    struct pressure_queue_entry pressure;
    struct imu_queue_entry imu;
    struct gps_queue_entry gps;


    // k_poll(&events[0], 1, K_FOREVER);
    // k_poll(&events[1], 1, K_FOREVER);
    events[0].state = K_POLL_STATE_NOT_READY;
    events[1].state = K_POLL_STATE_NOT_READY;

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

        if (k_msgq_get(&imu_msgq, &imu, K_NO_WAIT) == 0) {
            events[1].state = K_POLL_STATE_NOT_READY;

            // init mode
            if (!init.position_init && init.n_imu < IMU_INIT_N) {
                if (init.n_imu == 0 || imu.ax != init.ax[init.n_imu-1]){ // in order to work on native simulation
                    init.ax[init.n_imu] = imu.ax;
                    init.ay[init.n_imu] = imu.ay;
                    init.az[init.n_imu] = imu.az;
                    init.gx[init.n_imu] = imu.gx;
                    init.gy[init.n_imu] = imu.gy;
                    init.gz[init.n_imu] = imu.gz;
                    init.n_imu++;
                } 
            } else {
                // correct for IMU mounting inside of rocket - works for any number of rotations of 90 degrees.
                float a_array[3] = {imu.ax, imu.ay, imu.az};
                float g_array[3] = {imu.gx, imu.gy, imu.gz};

                float ax = a_array[fjalar->new_x_index] * fjalar->new_x_sign; 
                float ay = a_array[fjalar->new_y_index] * fjalar->new_y_sign; 
                float az = a_array[fjalar->new_z_index] * fjalar->new_z_sign;
                
                float gx = g_array[fjalar->new_x_index] * fjalar->new_x_sign; 
                float gy = g_array[fjalar->new_y_index] * fjalar->new_y_sign; 
                float gz = g_array[fjalar->new_z_index] * fjalar->new_z_sign; 
                
                // call filters
                position_filter_accelerometer(&pos_kf, &att_kf, ax, ay, az, imu.t); // needs magnetometer
                attitude_filter_gyroscope(&pos_kf, &att_kf, gx, gy, gz, imu.t);

                // prerequisite for running attitude accelerometer
                drag_update(&pos_kf, &att_kf, &aerodynamics); 

                if (pos_kf.X_data[2]<10 && filter_get_acceleration(&pos_kf)<5){
                    attitude_filter_accelerometer_ground(&att_kf, &pos_kf, ax, ay, az, imu.t); //only used pre launch
                }
                if (pos_kf.X_data[2]>10 && is_thrust_over(&pos_kf, &att_kf, &aerodynamics)){
                    //void attitude_filter_accelerometer_cruise(&att_kf, &pos_kf, ax, ay, az, time);
                }
            }

            // Give an error if the acceleration in z at launchpad is not around 9.8
            if (fjalar->flight_state == STATE_LAUNCHPAD
            || fjalar->flight_state == STATE_IDLE
            ) {
            if (fjalar->az < 0
            && (fjalar->flight_state == STATE_LAUNCHPAD)) {
                LOG_ERR("Acceleration is negative on the launchpad");
            }
        }
        }

        if (k_msgq_get(&pressure_msgq, &pressure, K_NO_WAIT) == 0) {
            events[0].state = K_POLL_STATE_NOT_READY;
            pos_kf.pressure = pressure.pressure*1000;
            

            // init
            if (!init.position_init) {
                if (init.n_baro < BARO_INIT_N) {
                    if (init.n_baro == 0 || pressure.pressure*1000 != init.p[init.n_baro-1]){
                    init.p[init.n_baro] = pressure.pressure*1000;
                    init.n_baro++;
                    } 
                }
            } else {
                //position_filter_barometer(&pos_kf, pressure.pressure, pressure.t);
            }

            fjalar->altitude = filter_get_altitude(&pos_kf);
            fjalar->velocity = filter_get_velocity(&pos_kf);

            if (fjalar->flight_state == STATE_LAUNCHPAD) {
                fjalar->ground_level = fjalar->altitude;
            }
        }


        #if DT_ALIAS_EXISTS(gps_uart)
        {
            struct gps_queue_entry gps;
            if (k_msgq_get(&gps_msgq, &gps, K_NO_WAIT) == 0) {

                // Reject frames that contain NaN or Inf in ANY field
                if (isfinite(gps.lat) &&
                    isfinite(gps.lon) &&
                    isfinite(gps.alt)) {
                    // init position
                    if (!init.position_init) {
                        if (init.n_gps < GPS_INIT_N) {
                            init.lat[init.n_gps] = gps.lat;
                            init.lon[init.n_gps] = gps.lon;
                            init.alt[init.n_gps] = gps.alt;
                            init.n_gps++;
                        }
                    } else {
                        position_filter_gps(&pos_kf, gps.lat, gps.lon, gps.alt, gps.t);
                    }

                } else {
                    LOG_WRN("GPS sample dropped (NaN/Inf)");
                }
            }
        }
        #endif

        if (!init.position_init && init_ready()) {
            init_finish(fjalar, &pos_kf, &att_kf, &init);
        }

        evaluate_state(fjalar);
        //LOG_DBG("state %d", fjalar->flight_state);
        // k_msleep(1);
    }
}
