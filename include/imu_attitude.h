#ifndef IMU_ATTITUDE_H
#define IMU_ATTITUDE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ATTITUDE_RING_SIZE 512

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG(x) ((x) * 180.0 / M_PI)
#define RAD(x) ((x) * M_PI / 180.0)

typedef struct {
    double roll;
    double pitch;
    double yaw;
} euler_t;

typedef struct {
    char accel_path[256];
    char gyro_path[256];
    double gyro_bias_x;
    double gyro_bias_y;
    double gyro_bias_z;
} imu_t;

typedef struct {
    int64_t ts_ns;
    double ax, ay, az;
    double gx, gy, gz;
} imu_sample_t;

typedef struct {
    int64_t ts_ns;
    double roll;
    double pitch;
    double yaw;
    double level_angle;
} attitude_sample_t;

typedef struct {
    euler_t e;
    double level_angle_filt;
    int initialized;
} attitude_filter_t;

typedef struct {
    attitude_sample_t samples[ATTITUDE_RING_SIZE];
    unsigned int head;
    unsigned int count;
} attitude_ring_t;

/*
 * attitude_ring_t is a small single-writer data container. Protect push/query
 * with an external mutex when the IMU and render loops run in different threads.
 */
int64_t imu_now_ns(void);

int imu_init(imu_t *imu);
int imu_init_with_names(imu_t *imu,
                        const char *accel_name,
                        const char *gyro_name,
                        int gyro_calib_samples);
int imu_read_sample(const imu_t *imu, imu_sample_t *sample);

void attitude_filter_init(attitude_filter_t *filter);
void attitude_update(attitude_filter_t *filter,
                     const imu_sample_t *sample,
                     double dt,
                     attitude_sample_t *out);

void attitude_ring_init(attitude_ring_t *ring);
void attitude_ring_push(attitude_ring_t *ring,
                        const attitude_sample_t *sample);
int attitude_ring_query(const attitude_ring_t *ring,
                        int64_t ts_ns,
                        attitude_sample_t *out);

#ifdef __cplusplus
}
#endif

#endif
