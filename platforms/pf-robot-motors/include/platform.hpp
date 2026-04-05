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
#include "pf_common/platform_common.hpp"
#include "pf_common/uuids.hpp"

/**
 * @name Messages Id
 * @{
 * @note UUID definitions are centralized in platforms/pf-common/include/pf_common/uuids.hpp
 *       Add new UUIDs there to ensure consistency across all platforms.
 */
// Import common UUIDs into global namespace for compatibility
// Game: 0x4000 - 0x4FFF
using cogip::pf_common::game_end_uuid;
using cogip::pf_common::game_reset_uuid;
using cogip::pf_common::game_start_uuid;
/// @}

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
