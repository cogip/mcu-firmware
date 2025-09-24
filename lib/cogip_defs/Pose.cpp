// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @brief       Pose class implementation
/// @author      COGIP Robotics

#include <cmath>

#include "cogip_defs/Pose.hpp"
#include "trigonometry.h"
#include "utils.hpp"

namespace cogip {

namespace cogip_defs {

Polar Pose::operator-(const Pose& p)
{
    float error_x = x_ - p.x();
    float error_y = y_ - p.y();

    float error_O = limit_angle_rad(std::atan2(error_y, error_x) - DEG2RAD(p.O()));

    return Polar(std::sqrt(square(error_x) + square(error_y)), RAD2DEG(error_O));
}

} // namespace cogip_defs

} // namespace cogip
