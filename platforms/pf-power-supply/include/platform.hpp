///  Copyright (C) 2025 COGIP Robotics association
///
///  This file is subject to the terms and conditions of the GNU Lesser
///  General Public License v2.1. See the file LICENSE in the top level
///  directory for more details.

///
///  @file
///  @defgroup    pf_power_supply Power Supply Platform
///  @ingroup     platforms
///  @brief       COGIP power supply platform definition
///
///  @author      Mathis LECRIVAIN <lecrivain.mathis@gmail.com>
///  @{

#pragma once

// Project includes
#include "canpb/CanProtobuf.hpp"

#include <iostream>

/// Messages Id
/// @{
// Motion Control: 0x1000 - 0x1FFF
// Actuators: 0x2000 - 0x2FFF
// Service: 0x3000 - 0x3FFF
constexpr cogip::canpb::uuid_t copilot_connected_uuid = 0x3002;
constexpr cogip::canpb::uuid_t copilot_disconnected_uuid = 0x3003;
// Game: 0x4000 - 0x4FFF
// Power Supply: 0x5000 - 0x5FFF
constexpr cogip::canpb::uuid_t emergency_stop_status_uuid = 0x5001;
constexpr cogip::canpb::uuid_t power_source_status_uuid = 0x5002;
constexpr cogip::canpb::uuid_t power_rails_status_uuid = 0x5003;
// Board: 0xF000 - 0xFFFF
/// @}

/// @brief Returns uarpb.
/// @return uarpb pointer
cogip::canpb::CanProtobuf& pf_get_canpb();

/// @brief Initialize all platforms threads
void pf_init_tasks(void);

/// @brief Platform initialization.
/// Must be called before any use of platform variables or functions.
void pf_init(void);

/// @}
