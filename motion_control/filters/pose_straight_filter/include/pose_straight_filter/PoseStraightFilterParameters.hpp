// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    pose_straight_filter Pose straight filter parameters
/// @{
/// @file
/// @brief      Movements switch thresholds
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

namespace cogip {

namespace motion_control {

/// Movements switch thresholds
class PoseStraightFilterParameters {
public:
    /// Constructor
    PoseStraightFilterParameters(
        double angular_threshold = 0.0,  ///< [in]  see angular_threshold_
        double linear_threshold = 0.0,   ///< [in]  see linear_threshold_
        double linear_deceleration_threshold = 0.0    ///< [in]  see linear_deceleration_threshold_
    ) :
    angular_threshold_(angular_threshold),
    linear_threshold_(linear_threshold),
    linear_deceleration_threshold_(linear_deceleration_threshold) {};

    ///  Get angular threshold
    /// return angular threshold
    double angular_threshold() const { return angular_threshold_; };

    /// Set angular threshold
    void set_angular_threshold(
        double angular_threshold                      ///< [in]   angular threshold
        ) { angular_threshold_ = angular_threshold; };

    ///  Get linear threshold
    /// return Linear threshold
    double linear_threshold() const { return linear_threshold_; };

    ///  Get linear deceleration threshold
    /// return Linear deceleration threshold
    double linear_deceleration_threshold() const { return linear_deceleration_threshold_; };

    /// Set linear threshold
    void set_linear_threshold(
        double linear_threshold                      ///< [in]   linear threshold
        ) { linear_threshold_ = linear_threshold; };

    /// Set linear deceleration threshold
    void set_linear_deceleration_threshold(
        double linear_deceleration_threshold         ///< [in]   linear deceleration threshold
        ) { linear_deceleration_threshold_ = linear_deceleration_threshold; };

private:
    /// the robot turns on itself until the angle error is lower than this threshold
    double angular_threshold_;

    /// the robot has reach the point when the linear error is lower than this threshold
    double linear_threshold_;

    /// the robot start to force its deceleration when the linear distance to the target point is below that value
    double linear_deceleration_threshold_;
};

} // namespace motion_control

} // namespace cogip

/// @}
