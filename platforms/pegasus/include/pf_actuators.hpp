// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pegasus
/// @{
/// @file
/// @brief       Generic definitions related to actuators.
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

// Firmware includes
#include "uartpb/UartProtobuf.hpp"

// Standard includes
#include <cstdint>

namespace cogip {
namespace pf {
namespace actuators {

/// Actuator state protobuf message id
constexpr cogip::uartpb::uuid_t actuator_state_uuid = 1674079543;
/// Emergency button released protobuf message id
constexpr cogip::uartpb::uuid_t emergency_button_pressed_uuid = 1885006827;
/// Emergency button pressed protobuf message id
constexpr cogip::uartpb::uuid_t emergency_button_released_uuid = 1396723216;

/// LX servomotors IDs
enum LXServoIDs {
    LXID_SWITCHER = 1,
    LXID_RIGHT_ARM = 2,
    LXID_RIGHT_ARM_LIFT = 3,
    LXID_LEFT_ARM = 4,
    LXID_LEFT_ARM_LIFT = 5,
};

/// Enum using to group actuators
enum class GroupEnum: uint8_t {
    NO_GROUP = 0,
};

/// Enable all actuators
void enable_all();

/// Disable all actuators
void disable_all();

/// Initialize all actuators
void init();

} // namespace actuators
} // namespace pf
} // namespace cogip

/// @}
