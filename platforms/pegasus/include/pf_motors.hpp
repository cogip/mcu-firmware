// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pegasus
/// @{
/// @file
/// @brief       Functions and definitions related to motors.
/// @author      Eric Courtois <eric.courtois@gmail.com>
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include "Motor.hpp"

namespace cogip {
namespace pf {
namespace actuators {
namespace motors {

// Motors ids
constexpr auto START_LINE = __LINE__;
enum class Enum: uint8_t {
    CENTRAL_LIFT_MOTOR = 1,
};
constexpr auto COUNT = __LINE__ - START_LINE - 3;

using PB_Message = EmbeddedProto::RepeatedFieldFixedSize<PB_Motor, COUNT>;

/// Initialize motors.
void init();

/// Get a motor by id.
Motor & get(
    Enum id  ///< [in] motor id
);

/// Copy data to Protobuf message.
void pb_copy(
    PB_Message & pb_message  ///< [out] Protobuf message to fill
);

} // namespace motors
} // namespace actuators
} // namespace pf
} // namespace cogip

/// @}
