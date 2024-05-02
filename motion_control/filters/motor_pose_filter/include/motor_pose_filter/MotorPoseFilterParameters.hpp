// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    motor_pose_filter Pose straight filter parameters
/// @{
/// @file
/// @brief      Movements switch thresholds
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

namespace cogip {

namespace motion_control {

/// Movements switch thresholds
class MotorPoseFilterParameters {
public:
    /// Constructor
    MotorPoseFilterParameters(
        double threshold = 0.0,   ///< [in]  see linear_threshold_
        double deceleration = 0.0    ///< [in]  see linear_deceleration_threshold_
    ) :
    threshold_(threshold),
    deceleration_(deceleration) {};

    /// Get threshold
    /// return threshold
    double threshold() const { return threshold_; };

    /// Get deceleration
    /// return deceleration
    double deceleration() const { return deceleration_; };

private:
    /// the motor has reach the pose when the error is lower than this threshold
    double threshold_;

    /// deceleration
    double deceleration_;
};

} // namespace motion_control

} // namespace cogip

/// @}
