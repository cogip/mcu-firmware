// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    motion_control_common
/// @{
/// @file
/// @brief      Meta Controller parameters
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

namespace cogip {

namespace motion_control {

/// Empty Meta Controller parameters.
/// MetaController does not use parameters, but it inherits from controller
/// which requires a parameters class, so declare it empty.
class MetaControllerParameters {
};

} // namespace motion_control

} // namespace cogip

/// @}
