// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    pose_straight_filter Pose straight filter parameters
/// @{
/// @file
/// @brief      Movements switch tresholds
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

namespace cogip {

namespace motion_control {

/// Movements switch tresholds
class PoseStraightFilterParameters {
public:
    /// Constructor
    PoseStraightFilterParameters(
        double angular_treshold = 0.0,  ///< [in]  see angular_treshold_
        double linear_treshold = 0.0,   ///< [in]  see linear_treshold_
        double linear_deceleration_treshold = 0.0    ///< [in]  see linear_deceleration_treshold_
    ) :
    angular_treshold_(angular_treshold),
    linear_treshold_(linear_treshold),
    linear_deceleration_treshold_(linear_deceleration_treshold) {};

    ///  Get angular treshold
    /// return angular treshold
    double angular_treshold() const { return angular_treshold_; };

    /// Set angular treshold
    void set_angular_treshold(
        double angular_treshold                      ///< [in]   angular treshold
        ) { angular_treshold_ = angular_treshold; };

    ///  Get linear treshold
    /// return Linear treshold
    double linear_treshold() const { return linear_treshold_; };

    ///  Get linear deceleration treshold
    /// return Linear deceleration treshold
    double linear_deceleration_treshold() const { return linear_deceleration_treshold_; };

    /// Set linear treshold
    void set_linear_treshold(
        double linear_treshold                      ///< [in]   linear treshold
        ) { linear_treshold_ = linear_treshold; };

    /// Set linear deceleration treshold
    void set_linear_deceleration_treshold(
        double linear_deceleration_treshold         ///< [in]   linear deceleration treshold
        ) { linear_deceleration_treshold_ = linear_deceleration_treshold; };

private:
    /// the robot turns on itself until the angle error is lower than this treshold
    double angular_treshold_;

    /// the robot has reach the point when the linear error is lower than this treshold
    double linear_treshold_;

    /// the robot start to force its deceleration when the linear distance to the target point is below that value
    double linear_deceleration_treshold_;
};

} // namespace motion_control

} // namespace cogip

/// @}
