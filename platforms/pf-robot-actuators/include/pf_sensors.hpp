// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pf-robot-sensors
/// @{
/// @file
/// @brief       Generic definitions related to sensors.
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Firmware includes
#include "canpb/CanProtobuf.hpp"
#include "pf_sensors.hpp"

// Standard includes
#include <cstdint>

namespace cogip {
namespace pf {
namespace sensors {

/// Initialize all sensors
void init();

} // namespace sensors
} // namespace pf
} // namespace cogip

/// @}
