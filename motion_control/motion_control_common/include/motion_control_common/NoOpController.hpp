// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    motion_control_common
/// @{
/// @file
/// @brief      No-operation controller (does nothing)
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include "BaseController.hpp"
#include "ControllersIO.hpp"

namespace cogip {
namespace motion_control {

/// @brief A controller that does nothing.
///
/// This is useful as a placeholder in ConditionalSwitchMetaController
/// when one branch should not execute any controller.
class NoOpController : public BaseController
{
  public:
    /// Constructor
    NoOpController() = default;

    /// Execute does nothing
    void execute([[maybe_unused]] ControllersIO& io) override
    {
        // Intentionally empty - this controller does nothing
    }

    /// Get the type name of this controller
    const char* type_name() const override
    {
        return "NoOpController";
    }
};

} // namespace motion_control
} // namespace cogip

/// @}
