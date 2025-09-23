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
///              Module used to monitor memory (thread stacks, heap and
///              overall).
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
#ifdef MODULE_CANPB
#include "canpb/CanProtobuf.hpp"
#endif

#pragma once

#ifndef SYSMON_THREADSTATUS_NAME_MAX_LENGTH
#define SYSMON_THREADSTATUS_NAME_MAX_LENGTH 64
#endif

namespace cogip {

namespace sysmon {

class ThreadStatus : public StatusBase<PB_ThreadStatus<SYSMON_THREADSTATUS_NAME_MAX_LENGTH>>
{

  public:
    /// Constructor
    ThreadStatus() : pid_(0), loops_(0), overshots_(0){};

    /// Increment thread loops number
    void inc_loops()
    {
        loops_++;
    };
    /// Increment thread overshots number
    void inc_overshots()
    {
        overshots_++;
    };
    /// Get thread loops number
    uint32_t loops() const
    {
        return loops_;
    };
    /// Get thread name
    etl::string<SYSMON_THREADSTATUS_NAME_MAX_LENGTH> name() const
    {
        return name_;
    };
    /// Get thread overshots number
    uint32_t overshots() const
    {
        return overshots_;
    };
    /// Get thread pid
    uint32_t pid() const
    {
        return pid_;
    };
    /// Set thread loops number
    void set_loops(const uint32_t loops)
    {
        loops_ = loops;
    };
    /// Set thread name
    void set_name(const etl::string<SYSMON_THREADSTATUS_NAME_MAX_LENGTH>& name)
    {
        name_ = name;
    };
    /// Set thread overshots number
    void set_overshots(const uint32_t overshots)
    {
        overshots_ = overshots;
    };
    /// Set thread pid
    void set_pid(const uint32_t pid)
    {
        pid_ = pid;
    };

    /// Memory status accessors (delegating to stack_status_)
    std::size_t size() const
    {
        return stack_status_.size();
    };
    std::size_t used() const
    {
        return stack_status_.used();
    };
    void set_size(const std::size_t size)
    {
        stack_status_.set_size(size);
    };
    void set_used(const std::size_t used)
    {
        stack_status_.set_used(used);
    };

    /// Update Protobuf message
    void update_pb_message() override;

  private:
    /// Thread pid
    uint32_t pid_;
    /// Thread name
    etl::string<SYSMON_THREADSTATUS_NAME_MAX_LENGTH> name_;
    /// Thread loop number
    uint32_t loops_;
    /// Thread time overshot
    uint32_t overshots_;
    /// Stack memory status
    MemoryStatus stack_status_;
};

/// Display heap memory status
void display_heap_status();
/// Display each thread status
void display_threads_status();
/// Start system monitoring thread
void sysmon_start();

#ifdef MODULE_CANPB
/// Register canpb serial interface for messaging
void register_canpb(cogip::canpb::CanProtobuf*);
#endif

} // namespace sysmon

} // namespace cogip

/// @}
