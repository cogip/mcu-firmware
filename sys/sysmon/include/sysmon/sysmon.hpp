///
/// Copyright (C) 2022 COGIP Robotics association <cogip35@gmail.com>
///
/// This file is subject to the terms and conditions of the GNU Lesser
/// General Public License v2.1. See the file LICENSE in the top level
/// directory for more details.
///
///
///
/// @defgroup    sys_sysmon System monitoring
/// @ingroup     sys
/// @brief       System monitoring module
///              Module used to monitor memory (thread stacks, heap and overall).
///
/// @{
/// @file
/// @brief       Public API for sysmon module
///
/// @author      Gilles DOFFE <g.doffe@gmail.com>
///

// Project includes
#ifdef MODULE_UARTPB
#include "uartpb/UartProtobuf.hpp"
#endif

#pragma once

namespace cogip {

namespace sysmon {

/// Display heap memory status
void display_heap_status(void);
/// Display each thread status
void display_threads_status(void);
/// Start system monitoring thread
void sysmon_start(void);
/// Update threads scheduling status
/// @param  pid             Thread pid
/// @param  has_overshot    Period overshot
void update_thread_sched_status(kernel_pid_t pid, bool has_overshot);

#ifdef MODULE_UARTPB
/// Register uartpb serial interface for messaging
void register_uartpb(cogip::uartpb::UartProtobuf *);
#endif

} // namespace sysmon

} // namespace cogip

/// @}
