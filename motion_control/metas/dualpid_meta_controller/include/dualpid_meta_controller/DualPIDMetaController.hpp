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

/// Meta controller for at least one PID controller for position and one PID controller for speed in chain.
/// A filter can be used between the two PID controllers.
/// Input 0:    polar pose error
/// Input 1:    current speed
/// Input 2:    target speed
/// Output 0:   motor command
class DualPIDMetaController: public MetaController <4, 1, 3> {};

} // namespace motion_control

} // namespace cogip

/// @}
