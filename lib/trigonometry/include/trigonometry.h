#pragma once

#include "etl/math_constants.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846 /* pi */
#endif

#define square(__x) (__x * __x)

#define RAD2DEG(a) ((a * 180.0) / M_PI)
#define DEG2RAD(a) ((a * M_PI) / 180.0)

#ifdef __cplusplus
extern "C" {
#endif

/// @brief Periodic boundary condition using modulus in order to map periodic
/// values
///
/// @note This can be used in oder to map value inside one interval.
///
/// @param x
/// @param y
/// @return
inline float periodicmod(float x, float y)
{
    return fmod(fmod(x, y) + y, y); // ((x % y) + y) % y
}

/// @brief Map input value to the given range
///
/// @note eg. for x = 360, min = -180 and max = 180
///           - 1. a = 360 - (-180) = 540
///           - 2. b = 180 - (-180) = 360
///           - 3. c = periodicmod(a, b) = 180
///           - 4. d = c + (-180) = 0
///
/// @param x Value to map
/// @param min min interval
/// @param max max interval
/// @return float the mapped value
inline float inrange(float x, float min, float max)
{
    return periodicmod(x - min, max - min) + min;
}

/// @brief limit angle in radians
/// @param O The value to map between -pi and pi
/// @return float the mapped value
inline float limit_angle_rad(float O)
{
    return inrange(O, -etl::math::pi, etl::math::pi);
}

/// @brief limit angle in degrees
/// @param O The value to map between -180 and 180
/// @return float the mapped value
inline float limit_angle_deg(float O)
{
    return inrange(O, -180, 180);
}

#ifdef __cplusplus
}
#endif
