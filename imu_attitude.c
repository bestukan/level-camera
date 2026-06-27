// icm42688_attitude.c
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>

#define DEG(x) ((x) * 180.0 / M_PI)
#define RAD(x) ((x) * M_PI / 180.0)
#define ATTITUDE_RING_SIZE 512

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

static int64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static int read_double_file(const char *path, double *val)
{
    FILE *fp = fopen(path, "r");
    if (!fp)
        return -1;

    int ret = fscanf(fp, "%lf", val);
    fclose(fp);

    return ret == 1 ? 0 : -1;
}

static int read_iio_double(const char *dev_path, const char *name, double *val)
{
    char path[512];
    snprintf(path, sizeof(path), "%s/%s", dev_path, name);
    return read_double_file(path, val);
}

static int find_iio_device(const char *target_name, char *out, size_t out_sz)
{
    char path[256];
    char name[128];

    for (int i = 0; i < 32; i++) {
        snprintf(path, sizeof(path), "/sys/bus/iio/devices/iio:device%d/name", i);

        FILE *fp = fopen(path, "r");
        if (!fp)
            continue;

        if (fgets(name, sizeof(name), fp)) {
            name[strcspn(name, "\r\n")] = 0;

            if (strcmp(name, target_name) == 0) {
                snprintf(out, out_sz, "/sys/bus/iio/devices/iio:device%d", i);
                fclose(fp);
                return 0;
            }
        }

        fclose(fp);
    }

    return -1;
}

static int read_accel(const imu_t *imu, double *ax, double *ay, double *az)
{
    double x, y, z, scale;

    if (read_iio_double(imu->accel_path, "in_accel_x_raw", &x) < 0)
        return -1;
    if (read_iio_double(imu->accel_path, "in_accel_y_raw", &y) < 0)
        return -1;
    if (read_iio_double(imu->accel_path, "in_accel_z_raw", &z) < 0)
        return -1;
    if (read_iio_double(imu->accel_path, "in_accel_scale", &scale) < 0)
        return -1;

    *ax = x * scale;
    *ay = y * scale;
    *az = z * scale;

    return 0;
}

static int read_gyro(const imu_t *imu, double *gx, double *gy, double *gz)
{
    double x, y, z, scale;

    if (read_iio_double(imu->gyro_path, "in_anglvel_x_raw", &x) < 0)
        return -1;
    if (read_iio_double(imu->gyro_path, "in_anglvel_y_raw", &y) < 0)
        return -1;
    if (read_iio_double(imu->gyro_path, "in_anglvel_z_raw", &z) < 0)
        return -1;
    if (read_iio_double(imu->gyro_path, "in_anglvel_scale", &scale) < 0)
        return -1;

    // IIO angular velocity scale is normally rad/s per LSB.
    *gx = x * scale - imu->gyro_bias_x;
    *gy = y * scale - imu->gyro_bias_y;
    *gz = z * scale - imu->gyro_bias_z;

    return 0;
}

static int calibrate_gyro(imu_t *imu, int samples)
{
    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;

    printf("Keep IMU still, calibrating gyro...\n");

    for (int i = 0; i < samples; i++) {
        double gx, gy, gz;

        if (read_gyro(imu, &gx, &gy, &gz) < 0)
            return -1;

        sx += gx;
        sy += gy;
        sz += gz;

        usleep(10000);
    }

    imu->gyro_bias_x = sx / samples;
    imu->gyro_bias_y = sy / samples;
    imu->gyro_bias_z = sz / samples;

    return 0;
}

static double wrap_pi(double a)
{
    while (a > M_PI)
        a -= 2.0 * M_PI;
    while (a < -M_PI)
        a += 2.0 * M_PI;
    return a;
}

static double lowpass_angle(double prev, double cur, double alpha)
{
    return wrap_pi(prev + alpha * wrap_pi(cur - prev));
}

static double interp_angle(double a, double b, double ratio)
{
    return wrap_pi(a + wrap_pi(b - a) * ratio);
}

static void attitude_ring_push(attitude_ring_t *ring, const attitude_sample_t *sample)
{
    ring->samples[ring->head] = *sample;
    ring->head = (ring->head + 1) % ATTITUDE_RING_SIZE;

    if (ring->count < ATTITUDE_RING_SIZE)
        ring->count++;
}

static int attitude_ring_get(const attitude_ring_t *ring,
                             unsigned int age,
                             attitude_sample_t *out)
{
    unsigned int idx;

    if (age >= ring->count)
        return -1;

    idx = (ring->head + ATTITUDE_RING_SIZE - 1 - age) % ATTITUDE_RING_SIZE;
    *out = ring->samples[idx];

    return 0;
}

static int attitude_ring_query(const attitude_ring_t *ring,
                               int64_t ts_ns,
                               attitude_sample_t *out)
{
    attitude_sample_t older;
    attitude_sample_t newer;

    if (ring->count == 0)
        return -1;

    if (attitude_ring_get(ring, 0, &newer) < 0)
        return -1;

    if (ts_ns >= newer.ts_ns) {
        *out = newer;
        return 0;
    }

    for (unsigned int age = 1; age < ring->count; age++) {
        if (attitude_ring_get(ring, age, &older) < 0)
            return -1;

        if (ts_ns >= older.ts_ns && ts_ns <= newer.ts_ns) {
            double span = (double)(newer.ts_ns - older.ts_ns);
            double ratio = span > 0.0 ? (double)(ts_ns - older.ts_ns) / span : 0.0;

            out->ts_ns = ts_ns;
            out->roll = interp_angle(older.roll, newer.roll, ratio);
            out->pitch = interp_angle(older.pitch, newer.pitch, ratio);
            out->yaw = interp_angle(older.yaw, newer.yaw, ratio);
            out->level_angle = interp_angle(older.level_angle, newer.level_angle, ratio);
            return 0;
        }

        newer = older;
    }

    *out = newer;
    return 0;
}

static void attitude_update(attitude_filter_t *f,
                            const imu_sample_t *s,
                            double dt,
                            attitude_sample_t *out)
{
    const double tau = 0.5;
    double alpha = tau / (tau + dt);

    double ax = s->ax;
    double ay = s->ay;
    double az = s->az;

    double acc_norm = sqrt(ax * ax + ay * ay + az * az);
    int accel_valid = acc_norm > 0.7 * 9.80665 && acc_norm < 1.3 * 9.80665;

    double acc_roll = atan2(ay, az);
    double acc_pitch = atan2(-ax, sqrt(ay * ay + az * az));

    f->e.roll  = wrap_pi(f->e.roll  + s->gx * dt);
    f->e.pitch = wrap_pi(f->e.pitch + s->gy * dt);
    f->e.yaw   = wrap_pi(f->e.yaw   + s->gz * dt);

    if (accel_valid) {
        f->e.roll = lowpass_angle(acc_roll, f->e.roll, alpha);
        f->e.pitch = lowpass_angle(acc_pitch, f->e.pitch, alpha);
    }

    double level_angle = f->e.roll;   // 初期可先用 roll，后续建议加安装矩阵修正

    if (!f->initialized) {
        f->level_angle_filt = level_angle;
        f->initialized = 1;
    } else {
        double smooth = 0.15;
        f->level_angle_filt = lowpass_angle(f->level_angle_filt, level_angle, smooth);
    }

    out->ts_ns = s->ts_ns;
    out->roll = f->e.roll;
    out->pitch = f->e.pitch;
    out->yaw = f->e.yaw;
    out->level_angle = f->level_angle_filt;
}

int main(void)
{
    imu_t imu;
    memset(&imu, 0, sizeof(imu));

    if (find_iio_device("icm42688-accel", imu.accel_path, sizeof(imu.accel_path)) < 0) {
        fprintf(stderr, "Cannot find IIO device: icm42688-accel\n");
        return 1;
    }

    if (find_iio_device("icm42688-gyro", imu.gyro_path, sizeof(imu.gyro_path)) < 0) {
        fprintf(stderr, "Cannot find IIO device: icm42688-gyro\n");
        return 1;
    }

    printf("accel: %s\n", imu.accel_path);
    printf("gyro : %s\n", imu.gyro_path);

    if (calibrate_gyro(&imu, 200) < 0) {
        fprintf(stderr, "Gyro calibration failed\n");
        return 1;
    }

    attitude_filter_t filter;
    attitude_ring_t ring;
    memset(&filter, 0, sizeof(filter));
    memset(&ring, 0, sizeof(ring));

    int64_t last_ns = now_ns();

    while (1) {
        imu_sample_t sample;
        attitude_sample_t attitude;

        sample.ts_ns = now_ns();

        int64_t dt_ns = sample.ts_ns - last_ns;
        last_ns = sample.ts_ns;

        double dt = dt_ns / 1000000000.0;
        if (dt <= 0.0 || dt > 0.2)
            dt = 0.01;

        if (read_accel(&imu, &sample.ax, &sample.ay, &sample.az) < 0) {
            fprintf(stderr, "accel: %s\n", imu.accel_path);
            return 1;
        }

        if (read_gyro(&imu, &sample.gx, &sample.gy, &sample.gz) < 0) {
            fprintf(stderr, "gyro : %s\n", imu.gyro_path);
            return 1;
        }

        attitude_update(&filter, &sample, dt, &attitude);
        attitude_ring_push(&ring, &attitude);

        if (attitude_ring_query(&ring, attitude.ts_ns, &attitude) < 0)
            continue;

        printf("%lld,%.3f,%.3f,%.3f,%.3f\n",
            (long long)attitude.ts_ns,
            DEG(attitude.roll),
            DEG(attitude.pitch),
            DEG(attitude.yaw),
            DEG(attitude.level_angle));
        fflush(stdout);

        usleep(10000);
    }

    return 0;
}
