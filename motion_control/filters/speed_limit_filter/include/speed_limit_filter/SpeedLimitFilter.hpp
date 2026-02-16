// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    speed_limit_filter
/// @{
/// @file
/// @brief      Speed limit filter class declaration
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include "motion_control_common/Controller.hpp"
#include "motion_control_common/ControllersIO.hpp"

#include "SpeedLimitFilterIOKeys.hpp"
#include "SpeedLimitFilterParameters.hpp"

namespace cogip {

namespace motion_control {

/// @brief Speed limit filter that clamps target speed to min/max bounds.
///
/// This filter implements simple speed limiting by clamping the target speed
/// to the configured minimum and maximum values.
///
/// - If |target_speed| > max_speed, it is clamped to max_speed (preserving sign)
/// - If |target_speed| < min_speed and non-zero, it is set to min_speed (preserving sign)
///
/// This ensures the robot operates within safe speed bounds.
class SpeedLimitFilter : public Controller<SpeedLimitFilterIOKeys, SpeedLimitFilterParameters>
{
  public:
    /// @brief Constructor.
    /// @param keys       Reference to IO key names.
    /// @param parameters Reference to filter parameters.
    /// @param name       Optional instance name for identification.
    explicit SpeedLimitFilter(const SpeedLimitFilterIOKeys& keys,
                              SpeedLimitFilterParameters& parameters, etl::string_view name = "")
        : Controller<SpeedLimitFilterIOKeys, SpeedLimitFilterParameters>(keys, parameters, name)
    {
    }

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "SpeedLimitFilter";
    }

    /// Execute the speed limit filter.
    void execute(ControllersIO& io) override;
};

} // namespace motion_control

} // namespace cogip

/// @}
