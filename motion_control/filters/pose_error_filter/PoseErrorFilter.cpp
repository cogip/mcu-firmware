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

void PoseErrorFilter::execute_linear(ControllersIO& io, float& pose_error, bool& target_changed)
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

    // Detect target change
    if (first_run_ || target_x != prev_target_x_ || target_y != prev_target_y_) {
        target_changed = true;
        prev_target_x_ = target_x;
        prev_target_y_ = target_y;
        first_run_ = false;
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

void PoseErrorFilter::execute_angular(ControllersIO& io, float& pose_error, bool& target_changed)
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

    // Detect target change
    if (first_run_ || target_O != prev_target_O_) {
        target_changed = true;
        prev_target_O_ = target_O;
        first_run_ = false;
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
    bool target_changed = false;

    if (parameters_.mode() == PoseErrorFilterMode::LINEAR) {
        execute_linear(io, pose_error, target_changed);
    } else {
        execute_angular(io, pose_error, target_changed);
    }

    // Write pose error output
    io.set(keys_.pose_error, pose_error);

    // Set recompute flag if target changed
    if (target_changed && !keys_.recompute.empty()) {
        io.set(keys_.recompute, true);
    }
}

} // namespace motion_control

} // namespace cogip

/// @}
