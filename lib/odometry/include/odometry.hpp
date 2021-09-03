#pragma once

#include <cstdint>

#include "cogip_defs/Polar.hpp"
#include "cogip_defs/Pose.hpp"

#define SEGMENT 0
#define ARC     1

/**
 * \fn odometry_setup
 * \brief odometry wheels_distance setup
 * \param d : distance between wheels [pulse]
 */
void odometry_setup(double d);

/**
 * \fn odometry_update
 * \brief update new robot pose (x, y, O) by ARC or SEGMENT approximation
 * \param p : new pose
 * \param robot_speed : delta value for distance [pulse]
 * \param approximation : SEGMENT (default) or ARC
 */
void odometry_update(cogip::cogip_defs::Pose &p,
                     const cogip::cogip_defs::Polar &robot_speed,
                     const uint8_t approximation);
