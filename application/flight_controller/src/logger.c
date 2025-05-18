#include "logger.h"
#include <stdio.h>
#include <sys/stat.h>

/* Return true if file exists on disk */
static bool file_exists(const char *path) {
    struct stat st;
    return (stat(path, &st) == 0);
}

bool csv_init(CSVLogger *logger, const char *path) {
    // always truncate (delete old contents)
    logger->fp = fopen(path, "w");
    if (!logger->fp) {
        return false;
    }

    fprintf(logger->fp,
        "time,"
        "distance_x,distance_y,distance_z,"
        "velocity_x,velocity_y,velocity_z,"
        "acceleration_x,acceleration_y,acceleration_z,"
        "rotation_x,rotation_y,rotation_z,"
        "pressure,expected_apogee\n"
    );
    fflush(logger->fp);
    return true;
}

void csv_log(CSVLogger *logger,
             double    t,
             double    dx, double dy, double dz,
             double    vx, double vy, double vz,
             double    ax, double ay, double az,
             double    rx, double ry, double rz,
             double p, double expected_apogee)
{
    fprintf(logger->fp,
        "%.6f,"    /* time */
        "%.6f,%.6f,%.6f,"  /* dist */
        "%.6f,%.6f,%.6f,"  /* vel   */
        "%.6f,%.6f,%.6f,"  /* accel  */
        "%.6f,%.6f,%.6f," /* rot   */
        "%.6f,%.6f\n", // pressure, estimated_apogee
        t,
        dx, dy, dz,
        vx, vy, vz,
        ax, ay, az,
        rx, ry, rz,
        p, expected_apogee
    );
    fflush(logger->fp);
}

void csv_close(CSVLogger *logger) {
    if (logger->fp) {
        fclose(logger->fp);
        logger->fp = NULL;
    }
}
