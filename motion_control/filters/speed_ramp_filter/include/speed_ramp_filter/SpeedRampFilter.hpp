// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    speed_ramp_filter
/// @{
/// @file
/// @brief      Speed ramp filter class declaration
/// @author     Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

#include "motion_control_common/Controller.hpp"
#include "motion_control_common/ControllersIO.hpp"

#include "SpeedRampFilterIOKeys.hpp"
#include "SpeedRampFilterParameters.hpp"

namespace cogip {

namespace motion_control {

/// @brief Speed ramp filter that generates a smooth ramp toward a target speed.
///
/// This filter implements the genRampSetpoint algorithm: it smoothly ramps
/// toward a step setpoint using distinct acceleration and deceleration rates.
/// If the measured speed overshoots the ramp, the ramp resynchronizes to the
/// current speed to avoid discontinuities.
class SpeedRampFilter : public Controller<SpeedRampFilterIOKeys, SpeedRampFilterParameters>
{
  public:
    /// @brief Constructor.
    /// @param keys       Reference to IO key names.
    /// @param parameters Reference to filter parameters.
    /// @param name       Optional instance name for identification.
    explicit SpeedRampFilter(const SpeedRampFilterIOKeys& keys,
                             SpeedRampFilterParameters& parameters, etl::string_view name = "")
        : Controller<SpeedRampFilterIOKeys, SpeedRampFilterParameters>(keys, parameters, name),
          ramp_setpoint_(0.0f)
    {
    }

    /// @brief Get the type name of this controller.
    const char* type_name() const override
    {
        return "SpeedRampFilter";
    }

    /// Execute the speed ramp filter.
    void execute(ControllersIO& io) override;

    /// Reset the filter state (ramp setpoint).
    void reset() override
    {
        ramp_setpoint_ = 0.0f;
    }

  private:
    float ramp_setpoint_; ///< Current ramp value tracking toward target
};

} // namespace motion_control

} // namespace cogip

/// @}
