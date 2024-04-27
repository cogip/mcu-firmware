#pragma once

#ifndef M_PI
#define M_PI   3.14159265358979323846  /* pi */
#endif

#define square(__x) (__x * __x)

#define RAD2DEG(a) ((a * 180.0) / M_PI)
#define DEG2RAD(a) ((a * M_PI) / 180.0)

#ifdef __cplusplus
extern "C" {
#endif

inline double limit_angle_rad(double O)
{
    // TODO: avoid risk of blocking loop
    while (O > M_PI) {
        O -= 2.0 * M_PI;
    }

    while (O < -M_PI) {
        O += 2.0 * M_PI;
    }

    return O;
}

inline double limit_angle_deg(double O)
{
    while (O > 180) {
        O -= 360;
    }

    while (O < -180) {
        O += 360;
    }

    return O;
}

#ifdef __cplusplus
}
#endif
