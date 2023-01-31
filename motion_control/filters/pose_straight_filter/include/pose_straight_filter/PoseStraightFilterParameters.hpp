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
        double linear_treshold = 0.0    ///< [in]  see linear_treshold_
    ) :  angular_treshold_(angular_treshold), linear_treshold_(linear_treshold) {};

    ///  Get angular treshold
    /// return angular treshold
    double angular_treshold() const { return angular_treshold_; };

    ///  Get linear treshold
    /// return Linear treshold
    double linear_treshold() const { return linear_treshold_; };

private:
    /// the robot turns on itself until the angle error is lower than this treshold
    double angular_treshold_;

    /// the robot has reach the point when the linear error is lower than this treshold
    double linear_treshold_;
};

} // namespace motion_control

} // namespace cogip

/// @}
