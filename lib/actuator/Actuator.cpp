// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @brief       Base actuators class implementation
/// @author      COGIP Robotics

#include "actuator/Actuator.hpp"

namespace cogip {
namespace actuators {

void Actuator::send_state(void)
{
    if (send_state_cb_)
        send_state_cb_(id_);
}

} // namespace actuators
} // namespace cogip
