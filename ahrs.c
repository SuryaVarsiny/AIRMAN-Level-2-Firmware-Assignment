#include "ahrs.h"
#include <math.h>

#define RAD_TO_DEG 57.2957795f
#define DEG_TO_RAD 0.017453292f
#define ALPHA 0.98f   // Complementary filter weight

void ahrs_init(AHRS_State *state) {
    state->roll = 0.0f;
    state->pitch = 0.0f;
    state->yaw = 0.0f;
}

void ahrs_update(
    AHRS_State *state,
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float mx, float my, float mz,
    float dt
) {
    // Accelerometer angles
    float acc_roll  = atan2f(ay, az) * RAD_TO_DEG;
    float acc_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;

    // Gyro integration
    state->roll  += gx * dt * RAD_TO_DEG;
    state->pitch += gy * dt * RAD_TO_DEG;
    state->yaw   += gz * dt * RAD_TO_DEG;

    // Complementary filter
    state->roll  = ALPHA * state->roll  + (1.0f - ALPHA) * acc_roll;
    state->pitch = ALPHA * state->pitch + (1.0f - ALPHA) * acc_pitch;

    // Magnetometer yaw (simple tilt-uncompensated)
    state->yaw = atan2f(my, mx) * RAD_TO_DEG;
}
