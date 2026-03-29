// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    speed_ramp_filter
/// @{
/// @file
/// @brief      Speed ramp filter parameters
/// @author     Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

namespace cogip {

namespace motion_control {

/// @brief Parameters for SpeedRampFilter.
class SpeedRampFilterParameters
{
  public:
    /// Constructor.
    explicit SpeedRampFilterParameters(
        float max_acceleration, ///< [in] max acceleration rate (units/period)
        float max_deceleration  ///< [in] max deceleration rate (units/period)
        )
        : max_acceleration_(max_acceleration), max_deceleration_(max_deceleration)
    {
    }

    /// Get max acceleration rate.
    float max_acceleration() const
    {
        return max_acceleration_;
    }

    /// Set max acceleration rate.
    void set_max_acceleration(float max_acceleration)
    {
        max_acceleration_ = max_acceleration;
    }

    /// Get max deceleration rate.
    float max_deceleration() const
    {
        return max_deceleration_;
    }

    /// Set max deceleration rate.
    void set_max_deceleration(float max_deceleration)
    {
        max_deceleration_ = max_deceleration;
    }

  private:
    float max_acceleration_; ///< Max acceleration rate (units/period)
    float max_deceleration_; ///< Max deceleration rate (units/period)
};

} // namespace motion_control

} // namespace cogip

/// @}
