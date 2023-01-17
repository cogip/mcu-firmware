// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    motion_control_common
/// @{
/// @file
/// @brief      Base class for meta controllers
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include "Controller.hpp"

namespace cogip {

namespace motion_control {

/// Base class for meta controllers. Meta controllers are controllers containers to execute them in chain.
class BaseMetaController : virtual public BaseController {
public:
    /// Add a controller to the controllers chain.
    /// @param controller
    virtual void add_controller(BaseController *controller) = 0;

protected:
    /// Number of controllers added to the chain.
    size_t nb_controllers_ = 0;
};

} // namespace motion_control

} // namespace cogip

/// @}
