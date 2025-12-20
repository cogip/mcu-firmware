// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    deceleration_filter
/// @{
/// @file
/// @brief      Deceleration filter class declaration
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include "motion_control_common/Controller.hpp"
#include "motion_control_common/ControllersIO.hpp"

#include "DecelerationFilterIOKeys.hpp"
#include "DecelerationFilterParameters.hpp"

namespace cogip {

namespace motion_control {

/// @brief Deceleration filter that limits target speed based on remaining distance.
///
/// This filter implements kinematic deceleration by comparing remaining distance
/// against a braking distance threshold. When the error falls below the threshold
/// computed from current speed and deceleration parameters, it recalculates the
/// target speed using the formula: speed = sqrt(2 * deceleration * remaining_error)
///
/// This ensures the robot decelerates smoothly to approach the target without
/// overshooting.
class DecelerationFilter : public Controller<DecelerationFilterIOKeys, DecelerationFilterParameters>
{
  public:
    /// @brief Constructor.
    /// @param keys       Reference to IO key names.
    /// @param parameters Reference to filter parameters.
    /// @param name       Optional instance name for identification.
    explicit DecelerationFilter(const DecelerationFilterIOKeys& keys,
                                DecelerationFilterParameters& parameters,
                                etl::string_view name = "")
        : Controller<DecelerationFilterIOKeys, DecelerationFilterParameters>(keys, parameters, name)
    {
    }

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "DecelerationFilter";
    }

    /// Execute the deceleration filter.
    void execute(ControllersIO& io) override;
};

} // namespace motion_control

} // namespace cogip

/// @}
