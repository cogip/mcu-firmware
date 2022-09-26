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

// RIOT includes
#include <etl/string.h>

// Project includes
#include "MemoryStatus.hpp"
#include "PB_ThreadStatus.hpp"
#ifdef MODULE_UARTPB
#include "uartpb/UartProtobuf.hpp"
#endif

#pragma once

#ifndef SYSMON_THREADSTATUS_NAME_MAX_LENGTH
#   define SYSMON_THREADSTATUS_NAME_MAX_LENGTH 64
#endif

namespace cogip {

namespace sysmon {

class ThreadStatus: public MemoryStatus {
    /// Protobuf message type. Shortcut for original template type.
    using PB_Message = PB_ThreadStatus<SYSMON_THREADSTATUS_NAME_MAX_LENGTH>;

    public:
        /// Constructor
        ThreadStatus();

        /// Get thread name
        inline etl::string<SYSMON_THREADSTATUS_NAME_MAX_LENGTH> name() const { return name_; };
        /// Get thread pid
        inline uint32_t pid() const { return pid_; };
        /// Set thread name
        inline void set_name(const etl::string<SYSMON_THREADSTATUS_NAME_MAX_LENGTH> &name) { name_ = name; };
        /// Set thread pid
        inline void set_pid(const uint32_t pid) { pid_ = pid; };

        /// Return the Protobuf message.
        const PB_Message &pb_message(void) const { return pb_message_; };
        /// Update Protobuf message
        void update_pb_message(void);

    private:
        /// Thread pid
        uint32_t pid_;
        /// Thread name
        etl::string<SYSMON_THREADSTATUS_NAME_MAX_LENGTH> name_;

        /// Protobug message
        PB_Message pb_message_;
};

/// Display heap memory status
void display_heap_status(void);
/// Display each thread status
void display_threads_status(void);
/// Start system monitoring thread
void sysmon_start(void);

#ifdef MODULE_UARTPB
/// Register uartpb serial interface for messaging
void register_uartpb(cogip::uartpb::UartProtobuf *);
#endif

} // namespace sysmon

} // namespace cogip

/// @}
