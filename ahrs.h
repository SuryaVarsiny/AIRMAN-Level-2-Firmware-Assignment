#ifndef AHRS_H
#define AHRS_H

typedef struct {
    float roll;
    float pitch;
    float yaw;
} AHRS_State;

void ahrs_init(AHRS_State *state);

void ahrs_update(
    AHRS_State *state,
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float mx, float my, float mz,
    float dt
);

#endif
