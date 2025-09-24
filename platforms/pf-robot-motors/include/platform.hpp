// Copyright (C) 2021 COGIP Robotics association
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @defgroup    platforms_pf-robot-motors Test platform
/// @ingroup     platforms
/// @brief       COGIP test platform definition
/// @author      Gilles DOFFE <g.doffe@gmail.com>
/// @{

#pragma once

// Project includes
#include "canpb/CanProtobuf.hpp"

#include <iostream>

/// @name Messages Id
/// @{
// Motion Control: 0x1000 - 0x1FFF
// Actuators: 0x2000 - 0x2FFF
// Board: 0xF000 - 0xFFFF
// Game: 0x4000 - 0x4FFF
constexpr cogip::canpb::uuid_t game_start_uuid = 0x4001;
constexpr cogip::canpb::uuid_t game_end_uuid = 0x4002;
constexpr cogip::canpb::uuid_t game_reset_uuid = 0x4003;
// Service: 0x3000 - 0x3FFF
constexpr cogip::canpb::uuid_t reset_uuid = 0x3001;
constexpr cogip::canpb::uuid_t copilot_connected_uuid = 0x3002;
constexpr cogip::canpb::uuid_t copilot_disconnected_uuid = 0x3003;
/// @}

/// @brief Set/unset copilot connected
///
/// @param[in] connected  copilot connected if true, not connected otherwise
void pf_set_copilot_connected(bool connected);

/// @brief Returns canpb.
///
/// @return  canpb pointer
cogip::canpb::CanProtobuf& pf_get_canpb();

/// @brief Initialize all platform threads
///
/// @return
void pf_init_tasks(void);

/// @brief Platform initialization.
/// Must be called before any use of platform variables or functions.
///
/// @return
void pf_init(void);

/// @}
