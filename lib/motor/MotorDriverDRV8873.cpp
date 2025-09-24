// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @brief       DRV8873 motor driver implementation
/// @author      COGIP Robotics

#include "ztimer.h"

#include "motor/MotorDriverDRV8873.hpp"

namespace cogip {

namespace motor {

int MotorDriverDRV8873::set_speed(float speed, int id)
{
    // WORKAROUND for H-Bridge TI DRV8873HPWPRQ1, need to reset fault in case of
    // undervoltage
    disable(id);
    ztimer_sleep(ZTIMER_USEC, 1);
    enable(id);

    return MotorDriverRIOT::set_speed(speed, id);
}

} // namespace motor

} // namespace cogip
