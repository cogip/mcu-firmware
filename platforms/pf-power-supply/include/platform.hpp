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
#include "pf_common/platform_common.hpp"
#include "pf_common/uuids.hpp"

/**
 * @name Messages Id
 * @{
 * @note UUID definitions are centralized in platforms/pf-common/include/pf_common/uuids.hpp
 *       Add new UUIDs there to ensure consistency across all platforms.
 */
// Import common UUIDs into global namespace for compatibility
// Power Supply: 0x5000 - 0x5FFF
using cogip::pf_common::emergency_stop_status_uuid;
using cogip::pf_common::power_rails_status_uuid;
using cogip::pf_common::power_source_status_uuid;
/// @}

/// @brief Initialize all platform threads
void pf_init_tasks(void);

/// @brief Platform initialization.
/// Must be called before any use of platform variables or functions.
void pf_init(void);

/// @}
