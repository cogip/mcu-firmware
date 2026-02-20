// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    pose_error_filter
/// @{
/// @file
/// @brief      Pose error filter implementation
/// @author     Gilles DOFFE <g.doffe@gmail.com>

// System includes
#include <cmath>

// Project includes
#include "log.h"
#include "pose_error_filter/PoseErrorFilter.hpp"
#include "trigonometry.h"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

void PoseErrorFilter::execute_linear(ControllersIO& io, float& pose_error)
{
    // Read target position
    float target_x = 0.0f;
    float target_y = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_x)) {
        target_x = *opt;
    }
    if (auto opt = io.get_as<float>(keys_.target_y)) {
        target_y = *opt;
    }

    // Read current position and orientation
    float current_x = 0.0f;
    float current_y = 0.0f;
    float current_O = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_x)) {
        current_x = *opt;
    }
    if (auto opt = io.get_as<float>(keys_.current_y)) {
        current_y = *opt;
    }
    if (auto opt = io.get_as<float>(keys_.current_O)) {
        current_O = *opt;
    }

    // Compute Euclidean distance
    float dx = target_x - current_x;
    float dy = target_y - current_y;
    float distance = std::sqrt(dx * dx + dy * dy);

    // Determine direction (bidirectional: choose optimal direction)
    // Compute angle from robot to target
    float angle_to_target = std::atan2(dy, dx) * 180.0f / static_cast<float>(M_PI);
    // Compute angle difference with current heading
    float angle_diff = limit_angle_deg(angle_to_target - current_O);

    // If target is behind (|angle_diff| > 90°), go backward (negative distance)
    if (std::fabs(angle_diff) > 90.0f) {
        pose_error = -distance;
    } else {
        pose_error = distance;
    }

    DEBUG("PoseErrorFilter LINEAR: target=(%.1f, %.1f) current=(%.1f, %.1f, %.1f) error=%.1f\n",
          static_cast<double>(target_x), static_cast<double>(target_y),
          static_cast<double>(current_x), static_cast<double>(current_y),
          static_cast<double>(current_O), static_cast<double>(pose_error));
}

void PoseErrorFilter::execute_angular(ControllersIO& io, float& pose_error)
{
    // Read target orientation
    float target_O = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_O)) {
        target_O = *opt;
    }

    // Read current orientation
    float current_O = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_O)) {
        current_O = *opt;
    }

    // Compute angle difference (limited to [-180, 180])
    pose_error = limit_angle_deg(target_O - current_O);

    DEBUG("PoseErrorFilter ANGULAR: target_O=%.1f current_O=%.1f error=%.1f\n",
          static_cast<double>(target_O), static_cast<double>(current_O),
          static_cast<double>(pose_error));
}

void PoseErrorFilter::execute(ControllersIO& io)
{
    DEBUG("Execute PoseErrorFilter\n");

    float pose_error = 0.0f;

    if (parameters_.mode() == PoseErrorFilterMode::LINEAR) {
        execute_linear(io, pose_error);
    } else {
        execute_angular(io, pose_error);
    }

    // Write pose error output
    io.set(keys_.pose_error, pose_error);

    // Pass through new_target flag from TargetChangeDetector
    if (!keys_.new_target.empty()) {
        if (auto opt = io.get_as<bool>(keys_.new_target)) {
            DEBUG("PoseErrorFilter: new_target=%d\n", *opt);
        }
    }
}

} // namespace motion_control

} // namespace cogip

/// @}
