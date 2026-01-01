// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    dualpid_meta_controller
/// @{
/// @file
/// @brief      Meta controller for pose and speed PID corrections
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "motion_control_common/MetaController.hpp"

namespace cogip {
namespace motion_control {

/// @brief Meta controller that chains one PosePIDController and one
/// SpeedPIDController (or SpeedFilter) in sequence.
class DualPIDMetaController : public MetaController<5>
{
  public:
    /// Constructor
    /// @param name Optional instance name for identification
    explicit DualPIDMetaController(etl::string_view name = "") : MetaController<5>(name) {}

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "DualPIDMetaController";
    }
};

} // namespace motion_control

} // namespace cogip

/// @}
