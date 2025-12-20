// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    deceleration_filter
/// @{
/// @file
/// @brief      Deceleration filter parameters
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

namespace cogip {

namespace motion_control {

/// @brief Parameters for DecelerationFilter.
class DecelerationFilterParameters
{
  public:
    /// Constructor.
    explicit DecelerationFilterParameters(
        float deceleration ///< [in] deceleration rate (units/period²)
        )
        : deceleration_(deceleration)
    {
    }

    /// Get deceleration rate.
    float deceleration() const
    {
        return deceleration_;
    }

    /// Set deceleration rate.
    void set_deceleration(float deceleration)
    {
        deceleration_ = deceleration;
    }

  private:
    float deceleration_; ///< Deceleration rate (units/period²)
};

} // namespace motion_control

} // namespace cogip

/// @}
