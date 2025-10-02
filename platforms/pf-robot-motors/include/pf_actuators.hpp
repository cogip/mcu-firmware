// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pf-robot-motors
/// @{
/// @file
/// @brief       Generic definitions related to actuators.
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Firmware includes
#include "actuator/Actuator.hpp"
#include "canpb/CanProtobuf.hpp"
#include "pf_common/uuids.hpp"

// Standard includes
#include <cstdint>

namespace cogip {
namespace pf {
namespace actuators {

/**
 * @name Actuator Message UUIDs (0x2000 - 0x2FFF)
 * @{
 * @note UUID definitions are centralized in platforms/pf-common/include/pf_common/uuids.hpp
 *       Add new UUIDs there to ensure consistency across all platforms.
 */
// Import common actuator UUIDs
using pf_common::actuator_command_uuid;
using pf_common::actuator_init_uuid;
using pf_common::actuator_state_uuid;
constexpr cogip::canpb::uuid_t thread_start_uuid = pf_common::actuator_thread_start_uuid;
constexpr cogip::canpb::uuid_t thread_stop_uuid = pf_common::actuator_thread_stop_uuid;
constexpr cogip::canpb::uuid_t command_uuid = actuator_command_uuid;
constexpr cogip::canpb::uuid_t init_uuid = actuator_init_uuid;
/// @}

/// Enable all actuators
void enable_all();

/// Disable all actuators
void disable_all();

/// Initialize all actuators
void init();

} // namespace actuators
} // namespace pf
} // namespace cogip

/// @}
