/*
This is the initialization script, its purpose is to:
1) Take some amount of sensor data and perform some numerical analysis of them, determining values such as initial value (mean), and variance.
2) Start the filter thread, this is because the start values are prerequisites for the state estimation filters.
It is important that the rocket remains stationary while the initialization thread is active.
*/

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/zbus/zbus.h>
#include <math.h>
#include <pla.h>

#include "fjalar.h"
#include "sensors.h"
#include "init.h"
#include "filter.h"
#include "aerodynamics.h"
#include "flight_state.h"
#include "com_master.h"
#include "control.h"

LOG_MODULE_REGISTER(init, LOG_LEVEL_INF);
K_SEM_DEFINE(init_done_sem, 0, 1);
#define INIT_THREAD_PRIORITY 7
#define INIT_THREAD_STACK_SIZE 4096

void init_thread(fjalar_t *fjalar, void *p2, void *p1);

K_THREAD_STACK_DEFINE(init_thread_stack, INIT_THREAD_STACK_SIZE);
struct k_thread init_thread_data;
k_tid_t init_thread_id;

void init_init(fjalar_t *fjalar) {
    init_thread_id = k_thread_create(
		&init_thread_data,
		init_thread_stack,
		K_THREAD_STACK_SIZEOF(init_thread_stack),
		(k_thread_entry_t) init_thread,
		fjalar, NULL, NULL,
		INIT_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(init_thread_id, "init");
}

/* converts sums -> means, writes them into fjalar  */
static void init_finish(init_t *init){
    double sum_ax  = 0, sum_ay  = 0, sum_az  = 0;
    double sum_gx  = 0, sum_gy  = 0, sum_gz  = 0;
    double sum_lon = 0, sum_lat = 0, sum_alt = 0;
    double sum_p   = 0;

    double mean_ax, mean_ay, mean_az;
    double mean_gx, mean_gy, mean_gz;
    double mean_lon = 0, mean_lat = 0, mean_alt = 0;
    double mean_p;

    double diff_ax  = 0, diff_ay  = 0, diff_az  = 0;
    double diff_gx  = 0, diff_gy  = 0, diff_gz  = 0;
    double diff_lon = 0, diff_lat = 0, diff_alt = 0;
    double diff_p   = 0;

    double var_ax, var_ay, var_az;
    double var_gx, var_gy, var_gz;
    double var_lon = 0, var_lat = 0, var_alt = 0;
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

    // identify g as sensed by accelerometer (to subtract it later)
    double g_accelerometer = sqrt(mean_ax*mean_ax + mean_ay*mean_ay + mean_az*mean_az);

    for (int i = 0; i<BARO_INIT_N; i++){
        sum_p += init->p[i];}
    mean_p = sum_p/BARO_INIT_N;

    for (int i = 0; i<GPS_INIT_N; i++){
        sum_lon += init->lon[i];
        sum_lat += init->lat[i];
        sum_alt += init->alt[i];}
    if (GPS_INIT_N){
        mean_lon = sum_lon/GPS_INIT_N;
        mean_lat = sum_lat/GPS_INIT_N;
        mean_alt = sum_alt/GPS_INIT_N;
    }
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
    if (GPS_INIT_N){
    var_lon = diff_lon/GPS_INIT_N;
    var_lat = diff_lat/GPS_INIT_N;
    var_alt = diff_alt/GPS_INIT_N;
    }
    
    // update init struct (nice structure am I right?)
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

    init->g_accelerometer = g_accelerometer;
    LOG_INF("gravitational acceleration as sensed by accelerometer: %f", init->g_accelerometer);

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
    
    init->pressure_ground = init->mean_p;
    LOG_INF("ground level pressure: %f", init->pressure_ground);

    init->lat0  = init->mean_lat;
    init->lon0 = init->mean_lon ;
    init->alt0 = init->mean_alt;

    init->seeded = false;
    LOG_INF("init done: lat0 = %f, lon0 = %f", init->lat0, init->lon0);
    
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

    init->new_x_index = ix;
    init->new_y_index = iy;
    init->new_z_index = iz;
    init->new_x_sign = sx;
    init->new_y_sign = sy;
    init->new_z_sign = sz;

    // euler angles start
    float new_ax = a[ix] * sx; 
    float new_ay = a[iy] * sy; 
    float new_az = a[iz] * sz;
    
    init->roll0 = atan2f(new_ay, new_az);
    init->pitch0 = atan2f(-new_ax, sqrtf(new_ay*new_ay + new_az*new_az));
    init->yaw0 = 0.0f;

}

static inline bool init_ready(const init_t *init)
{
    bool imu_ready  = (IMU_INIT_N  == 0) || (init->n_imu  >= IMU_INIT_N);
    bool baro_ready = (BARO_INIT_N == 0) || (init->n_baro >= BARO_INIT_N);
    bool gps_ready  = (GPS_INIT_N  == 0) || (init->n_gps  >= GPS_INIT_N);

    return imu_ready && baro_ready && gps_ready;
}

void init_thread(fjalar_t *fjalar, void *p2, void *p1) {
    init_t            *init  = fjalar->ptr_init;


    struct imu_queue_entry imu;
    struct pressure_queue_entry pressure;
    // struct gps_queue_entry gps; unused

    init->n_imu = 0;
    init->n_baro = 0;
    init->n_gps = 0;
    
    while (!init_ready(init)) {

		if (zbus_chan_read(&imu_zchan, &imu, K_NO_WAIT) == 0) {
            LOG_INF("IMU N: %d", init->n_imu);

            // init mode
            if (init->n_imu < IMU_INIT_N) {
                init->ax[init->n_imu] = imu.ax;
                init->ay[init->n_imu] = imu.ay;
                init->az[init->n_imu] = imu.az;
                init->gx[init->n_imu] = imu.gx;
                init->gy[init->n_imu] = imu.gy;
                init->gz[init->n_imu] = imu.gz;
                init->n_imu++;
            }
        }
        
		if (zbus_chan_read(&pressure_zchan, &pressure, K_NO_WAIT) == 0) {
            LOG_INF("BARO N: %d", init->n_baro);

            // init
            if (init->n_baro < BARO_INIT_N) {
                init->p[init->n_baro] = pressure.pressure*1000;
                init->n_baro++;
			}
        }
        

		 #if DT_ALIAS_EXISTS(gps_uart)
        {
            struct gps_queue_entry gps;
            if (zbus_chan_read(&gps_zchan, &gps, K_NO_WAIT) == 0) {

                // Reject frames that contain NaN or Inf in ANY field
                if (isfinite(gps.lat) &&
                    isfinite(gps.lon) &&
                    isfinite(gps.alt)) {
                    // init position
					if (init->n_gps < GPS_INIT_N) {
						init->lat[init->n_gps] = gps.lat;
						init->lon[init->n_gps] = gps.lon;
						init->alt[init->n_gps] = gps.alt;
						init->n_gps++;
					}

                } else {
                    LOG_WRN("GPS sample dropped (NaN/Inf)");
                }
            }
        }
        #endif
        k_msleep(10); // 100 Hz
    }
    //LOG_INF("WHAM 1");
    //k_msleep(1000);

	init_finish(init);
    LOG_WRN("Init Finished");

    //LOG_INF("WHAM 2");
    //k_msleep(1000);
    init_filter(fjalar);
    init_aerodynamics(&fjalar_god);
    init_control(&fjalar_god);
    LOG_INF("Init phase completed, started Kalman Filters.");
    // add BEEP from buzzer

    // Signal completion
    k_sem_give(&init_complete_sem);
}

