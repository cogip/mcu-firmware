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
#include "utils.hpp"
#include "canpb/CanProtobuf.hpp"

/**
 * @name Messages Id
 * @{
 */
// Motion Control: 0x1000 - 0x1FFF
constexpr cogip::canpb::uuid_t state_uuid = 0x1001;
constexpr cogip::canpb::uuid_t pose_order_uuid = 0x1002;
constexpr cogip::canpb::uuid_t pose_reached_uuid = 0x1003;
constexpr cogip::canpb::uuid_t pose_start_uuid = 0x1004;
constexpr cogip::canpb::uuid_t pid_request_uuid = 0x1005;
constexpr cogip::canpb::uuid_t pid_uuid = 0x1006;
constexpr cogip::canpb::uuid_t brake_uuid = 0x1007;
constexpr cogip::canpb::uuid_t controller_uuid = 0x1008;
constexpr cogip::canpb::uuid_t blocked_uuid = 0x1009;
constexpr cogip::canpb::uuid_t intermediate_pose_reached_uuid = 0x100A;
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
/** @} */

/**
 * @brief Get trace mode status
 *
 * @return                      true if trace mode is activated, false otherwise
 */
bool pf_trace_on(void);

/**
 * @brief Set/unset copilot connected
 *
 * param[in]    connected             copilot connected or not
 */
void pf_set_copilot_connected(bool connected);

/**
 * @brief Print current robot state in JSON format
 *
 * param[in]    out             tracefd descriptor used to print state
 *
 * @return
 */
void pf_print_state(void);

/**
 * @brief Check if a game is started
 *
 * @return                      Return 1(true) if started, 0(false) otherwise
 */
int pf_is_game_launched(void);

/**
 * @brief Returns canpb.
 *
 * return   canpb pointer
 **/
cogip::canpb::CanProtobuf & pf_get_canpb();

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
