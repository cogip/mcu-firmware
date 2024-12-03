#pragma once

#include <cmath>
#include "etl/math_constants.h"

#ifndef M_PI
#define M_PI   3.14159265358979323846  /* pi */
#endif

#define square(__x) (__x * __x)

#define RAD2DEG(a) ((a * 180.0) / M_PI)
#define DEG2RAD(a) ((a * M_PI) / 180.0)

#ifdef __cplusplus
extern "C" {
#endif

/// @brief Periodic boundary condition using modulus in order to map periodic values
///
/// @note This can be used in oder to map value inside one interval.
///
/// @param x
/// @param y
/// @return
inline double periodicmod(double x, double y)
{
    return fmod(fmod(x, y) + y, y);  // ((x % y) + y) % y
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
/// @return double the mapped value
inline double inrange(double x, double min, double max)
{
    return periodicmod(x - min, max - min) + min;
}

/// @brief limit angle in radians
/// @param O The value to map between -pi and pi
/// @return double the mapped value
inline double limit_angle_rad(double O)
{
    return inrange(O, -etl::math::pi, etl::math::pi);
}

/// @brief limit angle in degrees
/// @param O The value to map between -180 and 180
/// @return double the mapped value
inline double limit_angle_deg(double O)
{
    return inrange(O, -180, 180);
}

#ifdef __cplusplus
}
#endif
