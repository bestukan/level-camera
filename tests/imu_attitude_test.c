#define _GNU_SOURCE

#include "imu_attitude.h"

#include <stdio.h>
#include <unistd.h>

int main(void)
{
    imu_t imu;
    attitude_filter_t filter;
    attitude_ring_t ring;
    int64_t last_ns;

    if (imu_init(&imu) < 0)
        return 1;

    attitude_filter_init(&filter);
    attitude_ring_init(&ring);

    last_ns = imu_now_ns();

    while (1) {
        imu_sample_t sample;
        attitude_sample_t attitude;
        int64_t dt_ns;
        double dt;

        if (imu_read_sample(&imu, &sample) < 0)
            return 1;

        dt_ns = sample.ts_ns - last_ns;
        last_ns = sample.ts_ns;
        dt = dt_ns / 1000000000.0;

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
