///
/// Copyright (C) 2022 COGIP Robotics association <cogip35@gmail.com>
///
/// This file is subject to the terms and conditions of the GNU Lesser
/// General Public License v2.1. See the file LICENSE in the top level
/// directory for more details.
///
///
///
/// @defgroup    sys_thread Threading
/// @ingroup     sys
/// @brief       Threads management
///              Thread management wrapper to RIOT API to monitor threads loops
///              and period overshots
///
/// @{
/// @file
/// @brief       Public API for thread module
///
/// @author      Gilles DOFFE <g.doffe@gmail.com>
///

// RIOT includes
#include <ztimer.h>

#pragma once

namespace cogip {

namespace thread {

/// Wrapper to RIOT ztimer_periodic_wakeup()
/// @param  clock           ztimer clock to operate on
/// @param  last_wakeup     base time stamp for the wakeup
/// @param  period          time in ticks that will be added to last_wakeup
void thread_ztimer_periodic_wakeup(ztimer_clock_t* clock, uint32_t* last_wakeup, uint32_t period);

} // namespace thread

} // namespace cogip

/// @}
