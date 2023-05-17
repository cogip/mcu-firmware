/*
 * Copyright (C) 2021 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    platforms_pegasus Test platform
 * @ingroup     platforms
 * @brief       COGIP test platform definition
 * @{
 *
 * @file
 * @brief       Define hardware properties of test platform.
 *              Units:
 *              * time:         s
 *              * distance:     mm
 *              * speed:        mm/s
 *              * acceleration: mm/s²
 *
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 * @author      Eric Courtois <eric.courtois@gmail.com>
 */

#pragma once

/* Project includes */
#include "odometry.hpp"
#include "utils.hpp"
#include "uartpb/UartProtobuf.hpp"

#define ROBOT_ID                            0       /**< Robot ID for logs */
#define CONTROLLER_SPEED_LOOP_PERIOD_MSEC   20      /**< Motion controller speed loop default period */

/**
 * @name Eurobot general properties
 *
 * @{
 */
#define GAME_DURATION_SEC   100 /**< Timeout before completely stop the robot once started */
#define CAMP_LEFT 1             /**< Camp left selection for path mirroring */
#define CAMP_RIGHT 0            /**< Camp left selection for path mirroring */
/** @} */

/**
 * @name Avoidance borders
 * Borders in which the robot can move (mm)
 * @{
 */
#define AVOIDANCE_BORDER_X_MIN (ROBOT_MARGIN + 10)                /**< Minimal X axis border */
#define AVOIDANCE_BORDER_X_MAX (3000 - ROBOT_MARGIN - 10)         /**< Maximal X axis border */
#define AVOIDANCE_BORDER_Y_MIN (-1000 + ROBOT_MARGIN + 10)        /**< Minimal Y axis border */
#define AVOIDANCE_BORDER_Y_MAX (AVOIDANCE_BORDER_Y_MIN * -1)      /**< Maximal Y axis border */
/** @} */

// Define uartpb uuids
constexpr cogip::uartpb::uuid_t game_end_uuid = 1532296089;
constexpr cogip::uartpb::uuid_t reset_uuid = 3351980141;
constexpr cogip::uartpb::uuid_t copilot_connected_uuid = 1132911482;
constexpr cogip::uartpb::uuid_t copilot_disconnected_uuid = 1412808668;


/**
 * @brief Get trace mode status
 *
 * @return                      true if trace mode is activited, false otherwise
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
 * @brief Returns uarpb.
 *
 * return   uarpb pointer
 **/
cogip::uartpb::UartProtobuf & pf_get_uartpb();

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
