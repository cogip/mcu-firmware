// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pegasus
/// @{
/// @file
/// @brief       Functions and definitions related to pumps.
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#include "Pump.hpp"

namespace cogip {
namespace pf {
namespace actuators {
namespace pumps {

// Servomotors ids
constexpr auto START_LINE = __LINE__;
enum class Enum: vacuum_pump_t {
    RIGHT_ARM_PUMP = 0,
    LEFT_ARM_PUMP = 1,
};
constexpr auto COUNT = __LINE__ - START_LINE - 3;

using PB_Message = EmbeddedProto::RepeatedFieldFixedSize<PB_Pump, COUNT>;

/// Pump command class.
class Command {
public:
    /// Constructor.
    Command(
        Enum id,        ///< [in] pump id
        bool activated  ///< [in] activate pump if true, deactivate otherwise
    ) : id(id), activated(activated) {};

    Enum id;         ///< pump id
    bool activated;  ///< activate pump if true, deactivate otherwise
};

/// Initialize pumps.
void init();

/// Get a pump by id.
Pump & get(
    Enum id  ///< [in] pump id
);

/// Copy data to Protobuf message.
void pb_copy(
    PB_Message & pb_message  ///< [out] Protobuf message to fill
);

} // namespace pumps
} // namespace actuators
} // namespace pf
} // namespace cogip

/// @}
