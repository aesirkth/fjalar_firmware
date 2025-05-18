#ifndef LOGGER_H
#define LOGGER_H

#include <stdio.h>     /* <<— add this */
#include <stdbool.h>

typedef struct CSVLogger {
    FILE *fp;
} CSVLogger;

bool csv_init(CSVLogger *logger, const char *path);
void csv_log(CSVLogger *logger,
             double t,
             double dx, double dy, double dz,
             double vx, double vy, double vz,
             double ax, double ay, double az,
             double rx, double ry, double rz,
             double p,  double expected_apogee);
void csv_close(CSVLogger *logger);

extern CSVLogger logger;

#endif /* LOGGER_H */
