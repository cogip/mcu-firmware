/*
 * Copyright (C) 2021 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    platforms_pf-robot-motion-control Test platform
 * @ingroup     platforms
 * @brief       COGIP motion control platform definition
 * @{
 *
 * @file
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 * @author      Eric Courtois <eric.courtois@gmail.com>
 * @author      Mathis Lécrivain <lecrivain.mathis@gmail.com>
 */

#pragma once

/* Project includes */
#include "pf_common/platform_common.hpp"
#include "pf_common/uuids.hpp"

/**
 * @name Messages Id
 * @{
 * @note UUID definitions are centralized in platforms/pf-common/include/pf_common/uuids.hpp
 *       Add new UUIDs there to ensure consistency across all platforms.
 */
// Import common UUIDs into global namespace for compatibility
// Motion Control: 0x1000 - 0x1FFF
using cogip::pf_common::blocked_uuid;
using cogip::pf_common::brake_uuid;
using cogip::pf_common::controller_uuid;
using cogip::pf_common::intermediate_pose_reached_uuid;
using cogip::pf_common::pid_request_uuid;
using cogip::pf_common::pid_uuid;
using cogip::pf_common::pose_order_uuid;
using cogip::pf_common::pose_reached_uuid;
using cogip::pf_common::pose_start_uuid;
using cogip::pf_common::state_uuid;
// Service: 0x3000 - 0x3FFF
using cogip::pf_common::parameter_get_response_uuid;
using cogip::pf_common::parameter_get_uuid;
using cogip::pf_common::parameter_set_response_uuid;
using cogip::pf_common::parameter_set_uuid;
using cogip::pf_common::reset_uuid;
using cogip::pf_common::telemetry_data_uuid;
using cogip::pf_common::telemetry_disable_uuid;
using cogip::pf_common::telemetry_enable_uuid;
// Game: 0x4000 - 0x4FFF
using cogip::pf_common::game_end_uuid;
using cogip::pf_common::game_reset_uuid;
using cogip::pf_common::game_start_uuid;
/** @} */

/**
 * @brief Get trace mode status
 *
 * @return                      true if trace mode is activated, false otherwise
 */
bool pf_trace_on(void);

/**
 * @brief Initialize all platforms threads
 *
 * @return
 **/
void pf_init_tasks(void);

/**
 * @brief Platform initialization.
 * Must be called before any use of platform variables or functions.
 *
 * @return
 **/
void pf_init(void);

/** @} */
