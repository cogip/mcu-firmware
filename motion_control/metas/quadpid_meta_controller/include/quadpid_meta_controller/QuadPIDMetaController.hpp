// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    quadpid_meta_controller
/// @{
/// @file
/// @brief      Run 2 (meta-)controllers, one for linear control, one for
/// angular control
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "motion_control_common/MetaController.hpp"

namespace cogip {

namespace motion_control {

/// @brief Meta-controller combining four PID controllers.
///
/// This class manages up to four sub-controllers:
/// - one dedicated to resetting IO values at each cycle,
/// - one dedicated to position filtering if needed,
/// - one dedicated to coordinating dual PID control (position and speed)
///   for both linear and angular motion,
/// - one dedicated to telemetry.
class QuadPIDMetaController : public MetaController<>
{
  public:
    /// Constructor
    /// @param name Optional instance name for identification
    explicit QuadPIDMetaController(etl::string_view name = "") : MetaController<>(name) {}

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "QuadPIDMetaController";
    }
};

} // namespace motion_control

} // namespace cogip

/// @}
