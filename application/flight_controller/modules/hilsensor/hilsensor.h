typedef struct hil_data {
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    float p;

    float lon;
    float lat;
    float alt;

    uint32_t time;
} hil_data_t;

static int hilsensor_feed(const struct device *dev, hil_data_t *hil);
