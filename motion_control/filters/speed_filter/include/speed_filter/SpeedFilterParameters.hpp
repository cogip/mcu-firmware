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
class SpeedFilterParameters
{
  public:
    /// Constructor
    explicit SpeedFilterParameters(float min_speed = 0.0,        ///< [in]  see min_speed_
                                   float max_speed = 0.0,        ///< [in]  see max_speed_
                                   float max_acceleration = 0.0, ///< [in]  see max_acceleration_
                                   float max_deceleration = 0.0  ///< [in]  see max_deceleration_
                                   )
        : min_speed_(min_speed), max_speed_(max_speed), max_acceleration_(max_acceleration),
          max_deceleration_(max_deceleration == 0.0 ? max_acceleration : max_deceleration){};

    /// Get minimum speed
    /// return minimum speed
    float min_speed() const
    {
        return min_speed_;
    };

    /// Get maximum speed
    /// return maximum speed
    float max_speed() const
    {
        return max_speed_;
    };

    /// Get maximum acceleration
    /// return maximum acceleration
    float max_acceleration() const
    {
        return max_acceleration_;
    };

    /// Set minimum speed
    void set_min_speed(float min_speed ///< [in]   minimum speed
    )
    {
        min_speed_ = min_speed;
    };

    /// Set maximum speed
    void set_max_speed(float max_speed ///< [in]   maximum speed
    )
    {
        max_speed_ = max_speed;
    };

    /// Set maximum acceleration
    void set_max_acceleration(float max_acceleration ///< [in]   maximum acceleration
    )
    {
        max_acceleration_ = max_acceleration;
    };

    /// Get maximum deceleration
    /// return maximum deceleration
    float max_deceleration() const
    {
        return max_deceleration_;
    };

    /// Set maximum deceleration
    void set_max_deceleration(float max_deceleration ///< [in]   maximum deceleration
    )
    {
        max_deceleration_ = max_deceleration;
    };

  private:
    /// minimum speed the robot should reach
    float min_speed_;

    /// maximum speed the robot can reach
    float max_speed_;

    /// maximum robot acceleration allowed
    float max_acceleration_;

    /// maximum robot deceleration allowed
    float max_deceleration_;
};

} // namespace motion_control

} // namespace cogip

/// @}
