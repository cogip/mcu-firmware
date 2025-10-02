// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @defgroup    platforms_pf-common Common platform definitions
/// @ingroup     platforms
/// @brief       Shared CAN message UUID definitions for all COGIP platforms
/// @{
/// @file
/// @brief       CAN Protobuf message UUID definitions
/// @author      Mathis LECRIVAIN <lecrivain.mathis@gmail.com>

#pragma once

#include "canpb/CanProtobuf.hpp"

namespace cogip {
namespace pf_common {

/**
 * @name CAN Message ID Ranges
 * @brief UUID ranges allocated for different subsystems
 * @{
 */
// Motion Control: 0x1000 - 0x1FFF
// Actuators: 0x2000 - 0x2FFF
// Service: 0x3000 - 0x3FFF
// Game: 0x4000 - 0x4FFF
// Power Supply: 0x5000 - 0x5FFF (reserved)
// Board: 0xF000 - 0xFFFF
/** @} */

/**
 * @name Motion Control Message UUIDs (0x1000 - 0x1FFF)
 * @{
 */
constexpr canpb::uuid_t state_uuid = 0x1001;
constexpr canpb::uuid_t pose_order_uuid = 0x1002;
constexpr canpb::uuid_t pose_reached_uuid = 0x1003;
constexpr canpb::uuid_t pose_start_uuid = 0x1004;
constexpr canpb::uuid_t pid_request_uuid = 0x1005;
constexpr canpb::uuid_t pid_uuid = 0x1006;
constexpr canpb::uuid_t brake_uuid = 0x1007;
constexpr canpb::uuid_t controller_uuid = 0x1008;
constexpr canpb::uuid_t blocked_uuid = 0x1009;
constexpr canpb::uuid_t intermediate_pose_reached_uuid = 0x100A;
/** @} */

/**
 * @name Actuator Message UUIDs (0x2000 - 0x2FFF)
 * @{
 */
constexpr canpb::uuid_t actuator_thread_start_uuid = 0x2001;
constexpr canpb::uuid_t actuator_thread_stop_uuid = 0x2002;
constexpr canpb::uuid_t actuator_state_uuid = 0x2003;
constexpr canpb::uuid_t actuator_command_uuid = 0x2004;
constexpr canpb::uuid_t actuator_init_uuid = 0x2005;
/** @} */

/**
 * @name Service Message UUIDs (0x3000 - 0x3FFF)
 * @{
 */
constexpr canpb::uuid_t reset_uuid = 0x3001;
constexpr canpb::uuid_t copilot_connected_uuid = 0x3002;
constexpr canpb::uuid_t copilot_disconnected_uuid = 0x3003;
/** @} */

/**
 * @name Game Message UUIDs (0x4000 - 0x4FFF)
 * @{
 */
constexpr canpb::uuid_t game_start_uuid = 0x4001;
constexpr canpb::uuid_t game_end_uuid = 0x4002;
constexpr canpb::uuid_t game_reset_uuid = 0x4003;
/** @} */

/**
 * @name Power Supply Message UUIDs (0x5000 - 0x5FFF)
 * @{
 */
constexpr canpb::uuid_t emergency_stop_status_uuid = 0x5001;
constexpr canpb::uuid_t power_source_status_uuid = 0x5002;
constexpr canpb::uuid_t power_rails_status_uuid = 0x5003;
/** @} */

} // namespace pf_common
} // namespace cogip

/// @}
