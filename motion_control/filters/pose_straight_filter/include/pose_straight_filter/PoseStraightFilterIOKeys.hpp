// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    pose_straight_filter Pose straight filter IO keys
/// @{
/// @file
/// @brief      Pose straight filter IO keys.
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {
namespace motion_control {

/// @brief Bundle of ControllersIO key names for a PoseStraightFilter.
///        The application must supply the correct literals at runtime.
struct PoseStraightFilterIOKeys
{
    // Input keys
    etl::string_view current_pose_x;        ///< key for first coordinate of current pose
    etl::string_view current_pose_y;        ///< key for second coordinate of current pose
    etl::string_view current_pose_O;        ///< key for orientation of current pose
    etl::string_view target_pose_x;         ///< key for first coordinate of target pose
    etl::string_view target_pose_y;         ///< key for second coordinate of target pose
    etl::string_view target_pose_O;         ///< key for orientation of target pose
    etl::string_view current_linear_speed;  ///< key for linear component of current speed
    etl::string_view current_angular_speed; ///< key for angular component of current speed
    etl::string_view target_linear_speed;   ///< key for linear component of target speed
    etl::string_view target_angular_speed;  ///< key for angular component of target speed
    etl::string_view motion_direction;      ///< key for motion direction mode

    // Output keys
    etl::string_view linear_pose_error;         ///< key for linear distance to target
    etl::string_view linear_current_speed;      ///< key for linear component of
                                                ///< current speed output
    etl::string_view linear_target_speed;       ///< key for filtered linear speed output
    etl::string_view linear_speed_filter_flag;  ///< key for linear speed filter indicator
    etl::string_view angular_pose_error;        ///< key for angular difference to target
    etl::string_view angular_current_speed;     ///< key for angular component of
                                                ///< current speed output
    etl::string_view angular_target_speed;      ///< key for filtered angular speed output
    etl::string_view angular_speed_filter_flag; ///< key for angular speed filter indicator
    etl::string_view pose_reached;              ///< key for updated pose reached status
};

} // namespace motion_control

} // namespace cogip

/// @}
