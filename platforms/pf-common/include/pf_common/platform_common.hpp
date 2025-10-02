// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pf-common
/// @{
/// @file
/// @brief       Common platform base functionality declarations
/// @author      Mathis LECRIVAIN <lecrivain.mathis@gmail.com>

#pragma once

#include "canpb/CanProtobuf.hpp"
#include "canpb/ReadBuffer.hpp"

#include "etl/delegate.h"

namespace cogip {
namespace pf_common {

/// Prototype for copilot connection callbacks
using copilot_callback_t = etl::delegate<void(canpb::ReadBuffer&)>;

/// @brief Get copilot connected status
/// @return true if copilot is connected, false otherwise
bool is_copilot_connected();

/// @brief Returns the platform's CAN Protobuf instance
/// @return Reference to the CanProtobuf instance
canpb::CanProtobuf& get_canpb();

/// @brief Initialize common platform components (CAN, heartbeat)
/// @param[in] on_copilot_connected Optional callback when copilot connects
/// @param[in] on_copilot_disconnected Optional callback when copilot disconnects
/// @return 0 on success, error code otherwise
int pf_init(copilot_callback_t on_copilot_connected = copilot_callback_t(),
            copilot_callback_t on_copilot_disconnected = copilot_callback_t());

/// @brief Start common platform threads (heartbeat, CAN reader)
void pf_init_tasks();

} // namespace pf_common
} // namespace cogip

/// @brief Returns the CAN Protobuf instance (C-style API for compatibility)
/// @return Reference to the CanProtobuf instance
cogip::canpb::CanProtobuf& pf_get_canpb();

/// @}
