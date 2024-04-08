// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pf-robot-actuators
/// @{
/// @file
/// @brief       Functions and definitions related to servos.
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

// RIOT includes
#include "LxServo.hpp"

#include "etl/list.h"

namespace cogip {
namespace pf {
namespace actuators {
namespace servos {

/// Servomotors ids
constexpr auto START_LINE = __LINE__;
enum class Enum: lx_id_t {
    LXSERVO_LEFT_CART = 0,
    LXSERVO_RIGHT_CART = 1,
    LXSERVO_ARM_PANEL = 2,
};
constexpr auto COUNT = __LINE__ - START_LINE - 3;

using PB_Message = EmbeddedProto::RepeatedFieldFixedSize<PB_Servo, COUNT>;

/// Serco command class.
class Command {
public:
    /// Constructor.
    Command(
        Enum id,            ///< [in] servo id
        uint16_t position,  ///< [in] position to reach
        uint16_t time = 0   ///< [in] time to reach the position (in ms)
     ) : id(id), position(position), time(time) {};

    Enum id;            ///< servo id
    uint16_t position;  ///< position to reach
    uint16_t time;      ///< time to reach the position (in ms)
};

/// Initialize LX servomotors.
void init(uart_half_duplex_t *lx_stream);

/// Get a servo by id.
LxServo & get(
    Enum id  ///< servo id
);

/// Move servo according to the given command.
void move(
    const Command & command,  ///< [in] servo command
    uint32_t wait = 0         ///< [in] time to wait after move (in ms)
);

/// Move mutliple servos in parallel according to the given commands
void parallel_move(
    const etl::list<Command, COUNT> & commands,  ///< [in] servo commands
    uint32_t wait = 0                            ///< [in] time to wait after move (in ms)
);

/// Disable all servomotors
void disable_all();

/// Send pump state protobuf message
void send_state(Enum servo);

/// Copy data to Protobuf message.
void pb_copy(
    PB_Message & pb_message  ///< [out] Protobuf message to fill
);

} // namespace servos
} // namespace actuators
} // namespace pf
} // namespace cogip

/// @}
