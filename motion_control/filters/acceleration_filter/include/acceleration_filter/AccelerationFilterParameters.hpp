// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    acceleration_filter
/// @{
/// @file
/// @brief      Acceleration filter parameters
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

namespace cogip {

namespace motion_control {

/// @brief Parameters for AccelerationFilter.
class AccelerationFilterParameters
{
  public:
    /// Constructor.
    explicit AccelerationFilterParameters(
        float acceleration,    ///< [in] acceleration rate (units/period²)
        float min_speed = 0.0f ///< [in] minimum guaranteed speed (units/period)
        )
        : acceleration_(acceleration), min_speed_(min_speed)
    {
    }

    /// Get acceleration rate.
    float acceleration() const
    {
        return acceleration_;
    }

    /// Set acceleration rate.
    void set_acceleration(float acceleration)
    {
        acceleration_ = acceleration;
    }

    /// Get minimum guaranteed speed.
    float min_speed() const
    {
        return min_speed_;
    }

    /// Set minimum guaranteed speed.
    void set_min_speed(float min_speed)
    {
        min_speed_ = min_speed;
    }

  private:
    float acceleration_; ///< Acceleration rate (units/period²)
    float min_speed_;    ///< Minimum guaranteed speed (units/period)
};

} // namespace motion_control

} // namespace cogip

/// @}
