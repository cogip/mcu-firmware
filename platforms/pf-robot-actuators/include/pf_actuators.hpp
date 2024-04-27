// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pf-robot-actuators
/// @{
/// @file
/// @brief       Generic definitions related to actuators.
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

// Firmware includes
#include "canpb/CanProtobuf.hpp"

// Standard includes
#include <cstdint>

namespace cogip {
namespace pf {
namespace actuators {

// Actuators: 0x2000 - 0x2FFF
constexpr cogip::canpb::uuid_t actuator_state_uuid = 0x2003;
constexpr cogip::canpb::uuid_t thread_start_uuid = 0x2001;
constexpr cogip::canpb::uuid_t thread_stop_uuid = 0x2002;
constexpr cogip::canpb::uuid_t command_uuid = 0x2004;

/// Emergency button released protobuf message id
constexpr cogip::canpb::uuid_t emergency_button_pressed_uuid = 0x1;
/// Emergency button pressed protobuf message id
constexpr cogip::canpb::uuid_t emergency_button_released_uuid = 0x2;

/// LX servomotors IDs
enum LXServoIDs {
    LXSERVO_LEFT_CART = 0,
    LXSERVO_RIGHT_CART = 1,
    LXSERVO_ARM_PANEL = 2,
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
