// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    speed_limit_filter
/// @{
/// @file
/// @brief      Speed limit filter parameters
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

namespace cogip {

namespace motion_control {

/// @brief Parameters for SpeedLimitFilter.
class SpeedLimitFilterParameters
{
  public:
    /// Constructor.
    explicit SpeedLimitFilterParameters(float min_speed, ///< [in] minimum speed (units/period)
                                        float max_speed  ///< [in] maximum speed (units/period)
                                        )
        : min_speed_(min_speed), max_speed_(max_speed)
    {
    }

    /// Get minimum speed.
    float min_speed() const
    {
        return min_speed_;
    }

    /// Set minimum speed.
    void set_min_speed(float min_speed)
    {
        min_speed_ = min_speed;
    }

    /// Get maximum speed.
    float max_speed() const
    {
        return max_speed_;
    }

    /// Set maximum speed.
    void set_max_speed(float max_speed)
    {
        max_speed_ = max_speed;
    }

  private:
    float min_speed_; ///< Minimum speed (units/period)
    float max_speed_; ///< Maximum speed (units/period)
};

} // namespace motion_control

} // namespace cogip

/// @}
