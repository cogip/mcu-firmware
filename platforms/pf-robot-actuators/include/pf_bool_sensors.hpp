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
#include "BoolSensor.hpp"

// Standard includes
#include <cstdint>

namespace cogip {
namespace pf {
namespace sensors {
namespace bool_sensors {

// Motors ids
constexpr auto START_LINE = __LINE__;
enum class Enum: uint8_t {
    BOTTOM_GRIP_LEFT = 0,
    BOTTOM_GRIP_RIGHT = 1,
    TOP_GRIP_LEFT = 2,
    TOP_GRIP_RIGHT = 3,
    MAGNET_LEFT = 4,
    MAGNET_RIGHT = 5
};
constexpr auto COUNT = __LINE__ - START_LINE - 3;


/// Initialize all sensors
void init();

/// Check if a boolean sensor identified by id exists.
bool contains(
    cogip::pf::sensors::Enum id  ///< [in] sensor id
);

/// Get a sensor by id.
BoolSensor & get(
    cogip::pf::sensors::Enum id  ///< [in] sensor id
);

/// Send bool sensor state
void send_state(
    cogip::pf::sensors::Enum id  ///< [in] sensor id
);

} // namespace bool_sensors
} // namespace sensors
} // namespace pf
} // namespace cogip

/// @}
