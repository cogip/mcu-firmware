// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    quadpid_meta_controller
/// @{
/// @file
/// @brief      Run 2 (meta-)controllers, one for linear control, one for angular control
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "motion_control_common/MetaController.hpp"

namespace cogip {

namespace motion_control {

/// Run 2 (meta-)controllers, one for linear control, one for angular control.
/// Input 0-2:  current pose
/// Input 3-5:  target pose
/// Input 6-7:  current speed
/// Input 8-9:  target speed
/// Input 10:   allow reverse
/// Output 0:   linear motor command
/// Output 1:   angular motor command
/// Output 2:   pose_reached
class QuadPIDMetaController: public MetaController <11, 3, 2> {};

} // namespace motion_control

} // namespace cogip

/// @}
