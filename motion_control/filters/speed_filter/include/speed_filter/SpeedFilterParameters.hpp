// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    speed_filter Speed filter parameters
/// @{
/// @file
/// @brief      Speed and acceleration limits
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

namespace cogip {

namespace motion_control {

/// Speed and acceleration limits
class SpeedFilterParameters {
public:
    /// Constructor
    SpeedFilterParameters(
        double max_speed = 0.0,         ///< [in]  see max_speed_
        double max_acceleration = 0.0   ///< [in]  see max_acceleration_
    ) :  max_speed_(max_speed), max_acceleration_(max_acceleration) {};

    /// Get maximum speed
    /// return maximum speed
    double max_speed() const { return max_speed_; };

    /// Get maximum acceleration
    /// return maximum acceleration
    double max_acceleration() const { return max_acceleration_; };

    /// Set maximum speed
    void set_max_speed(
        double max_speed                ///< [in]   maximum speed
        ) { max_speed_ = max_speed; };

    /// Set maximum acceleration
    void set_max_acceleration(
        double max_acceleration         ///< [in]   maximum acceleration
        ) { max_acceleration_ = max_acceleration; };

private:
    /// maximum speed the robot can reach
    double max_speed_;

    /// maximum robot acceleration allowed
    double max_acceleration_;
};

} // namespace motion_control

} // namespace cogip

/// @}