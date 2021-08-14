#pragma once

#include <stdint.h>

#include "cogip_defs.h"

#define SEGMENT 0
#define ARC     1

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \fn odometry_setup
 * \brief odometry pose_t and wheels_distance setup
 * \param p : robot pose
 * \param d : distance between wheels [pulse]
 */
void odometry_setup(double d);

/**
 * \fn odometry_update
 * \brief update new robot pose_t (x, y, O) by ARC or SEGMENT approximation
 * \param polar_t : delta value for distance [pulse]
 * \param approximation : SEGMENT (default) or ARC
 */
void odometry_update(pose_t *p,
                     polar_t *robot_speed,
                     const uint8_t approximation);

#ifdef __cplusplus
}
#endif
