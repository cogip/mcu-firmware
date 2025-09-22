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

// System includes
#include <cstddef>
#include <string>

// Project includes
#include "PB_MemoryStatus.hpp"
#ifdef MODULE_CANPB
#include "canpb/CanProtobuf.hpp"
#endif

#pragma once

namespace cogip {

namespace sysmon {

class MemoryStatus {
    /// Protobuf message type. Shortcut for original template type.
    using PB_Message = PB_MemoryStatus;

    public:
        MemoryStatus();
        /// Get memory total size in bytes
        std::size_t size() const { return size_; };
        /// Get memory used size in bytes
        std::size_t used() const { return used_; };
        /// Set memory total size in bytes
        void set_size(const std::size_t size) { size_ = size; };
        /// Set memory used size in bytes
        void set_used(const std::size_t used) { used_ = used; };

        /// Return the Protobuf message.
        virtual const PB_Message &pb_message() const { return pb_message_; };
        /// Update Protobuf message
        virtual void update_pb_message();

    private:
        /// Memory total size in bytes
        std::size_t size_;
        /// Memory used size in bytes
        std::size_t used_;

        /// Protobug message
        PB_Message pb_message_;
};

} // namespace sysmon

} // namespace cogip

/// @}
