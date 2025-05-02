// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pf-robot-actuators
/// @{
/// @file
/// @brief       Functions and definitions related to positional_actuators.
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include "actuator/PositionalActuator.hpp"
#include "actuator/LiftParameters.hpp"

namespace cogip {
namespace pf {
namespace actuators {
namespace positional_actuators {

/// Initialize positional_actuators.
void init();

/// Functional initiliazation sequence
void init_sequence();

/// Check if a positional_actuator identified by id exists.
bool contains(
    cogip::actuators::Enum id  ///< [in] positional_actuator id
);

/// Get a positional_actuator by id.
cogip::actuators::positional_actuators::PositionalActuator & get(
    cogip::actuators::Enum id  ///< [in] positional_actuator id
);

/// Disable all positional actuators
void disable_all();

/// Enable all positional actuators
void enable_all();

/// Send positional actuator state protobuf message
void send_state(cogip::actuators::Enum positional_actuator);

/// Initialize motors at their origin
void pf_init_motors_sequence(void);

/// Instanciate a lift
/// @return 0 if ok, negative otherwise
int create_lift(
    cogip::actuators::Enum id,
    const cogip::actuators::positional_actuators::LiftParameters& lift_params);

} // namespace positional_actuators
} // namespace actuators
} // namespace pf
} // namespace cogip

/// @}
