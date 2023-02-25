/*
 * Copyright (C) 2022 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

// RIOT includes
#include <xtimer.h>

// Firmware includes
#include "pf_servos.hpp"
#include "pf_pumps.hpp"

namespace cogip {
namespace app {
namespace arms {

using ServoEnum = pf::actuators::servos::Enum;
using PumpEnum = pf::actuators::pumps::Enum;

static void _wait_timeout(uint32_t timeout) {
    xtimer_msleep(timeout);
}

} // namespace arms
} // namespace app
} // namespace cogip
