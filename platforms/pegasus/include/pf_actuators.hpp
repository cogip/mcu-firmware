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

#include <cstdint>

namespace cogip {
namespace pf {
namespace actuators {

/// Enum using to group actuators
enum class GroupEnum: uint8_t {
    NO_GROUP = 0,
    LEFT_ARM = 1,
    RIGHT_ARM = 2,
    CENTRAL_ARM = 3
};

/// Initialize all actuators
void init();

} // namespace actuators
} // namespace pf
} // namespace cogip

/// @}
