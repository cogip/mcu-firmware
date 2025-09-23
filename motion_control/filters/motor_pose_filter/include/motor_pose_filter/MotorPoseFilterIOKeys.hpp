// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    motor_pose_filter Motor pose filter IO keys
/// @{
/// @file
/// @brief      Motor Pose filter IO keys
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {
namespace motion_control {

/// @brief Bundle of ControllersIO key names for a MotorPoseFilter.
///        The application must supply the correct literals at runtime.
struct MotorPoseFilterIOKeys
{
    etl::string_view current_pose;  ///< key for current pose input
    etl::string_view target_pose;   ///< key for target pose input
    etl::string_view current_speed; ///< key for current speed input
    etl::string_view target_speed;  ///< key for target speed input
    etl::string_view pose_reached;  ///< key for pose reached status input

    etl::string_view position_error;    ///< key for pose error output
    etl::string_view filtered_speed;    ///< key for filtered target speed output
    etl::string_view speed_filter_flag; ///< key for speed filter flag output
    etl::string_view pose_reached_out;  ///< key for updated pose reached status output
};

} // namespace motion_control
} // namespace cogip

/// @}
