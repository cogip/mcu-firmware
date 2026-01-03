// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    acceleration_filter
/// @{
/// @file
/// @brief      Acceleration filter class declaration
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include "motion_control_common/Controller.hpp"
#include "motion_control_common/ControllersIO.hpp"

#include "AccelerationFilterIOKeys.hpp"
#include "AccelerationFilterParameters.hpp"

namespace cogip {

namespace motion_control {

/// @brief Acceleration filter that limits target speed increase based on acceleration rate.
///
/// This filter implements kinematic acceleration limiting by comparing target speed
/// against current speed and limiting the increase to the configured acceleration rate.
///
/// This ensures the robot accelerates smoothly without sudden speed changes that could
/// cause wheel slip or mechanical stress.
class AccelerationFilter : public Controller<AccelerationFilterIOKeys, AccelerationFilterParameters>
{
  public:
    /// @brief Constructor.
    /// @param keys       Reference to IO key names.
    /// @param parameters Reference to filter parameters.
    /// @param name       Optional instance name for identification.
    explicit AccelerationFilter(const AccelerationFilterIOKeys& keys,
                                AccelerationFilterParameters& parameters,
                                etl::string_view name = "")
        : Controller<AccelerationFilterIOKeys, AccelerationFilterParameters>(keys, parameters,
                                                                             name),
          previous_speed_order_(0.0f)
    {
    }

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "AccelerationFilter";
    }

    /// Execute the acceleration filter.
    void execute(ControllersIO& io) override;

    /// Reset the filter state (previous output speed).
    void reset()
    {
        previous_speed_order_ = 0.0f;
    }

  private:
    float previous_speed_order_; ///< Previous cycle's speed order for acceleration calculation
};

} // namespace motion_control

} // namespace cogip

/// @}
