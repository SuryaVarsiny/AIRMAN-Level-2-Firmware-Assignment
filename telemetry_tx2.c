#include <stdio.h>
#include <time.h>
#include <math.h>
#include "ahrs.h"

#ifdef _WIN32
#include <windows.h>
#define sleep_ms(x) Sleep(x)
#else
#include <unistd.h>
#define sleep_ms(x) usleep(x * 1000)
#endif

long timestamp_ms() {
    return (long)(clock() * 1000 / CLOCKS_PER_SEC);
}

unsigned char checksum(const char *s) {
    unsigned char chk = 0;
    while (*s) chk ^= *s++;
    return chk;
}

int main() {
    FILE *fp = fopen("telemetry_l2.log", "w");
    if (!fp) return -1;

    AHRS_State ahrs;
    ahrs_init(&ahrs);

    float t = 0.0f;
    const float dt = 0.05f;

    while (1) {
        float ax = sinf(t);
        float ay = cosf(t);
        float az = 9.81f;

        float gx = 0.02f;
        float gy = 0.015f;
        float gz = 0.01f;

        float mx = cosf(t);
        float my = sinf(t);
        float mz = 0.0f;

        float altitude = 100.0f + t;
        float temperature = 25.0f + sinf(t);

        ahrs_update(&ahrs, ax, ay, az, gx, gy, gz, mx, my, mz, dt);

        char frame[256];
        snprintf(frame, sizeof(frame),
            "$L2,%ld,%.2f,%.2f,%.2f,%.2f,%.2f",
            timestamp_ms(),
            ahrs.roll,
            ahrs.pitch,
            ahrs.yaw,
            altitude,
            temperature
        );

        unsigned char chk = checksum(frame + 1);
        fprintf(fp, "%s*%02X\n", frame, chk);
        fflush(fp);

        t += dt;
        sleep_ms(50);
    }
}
