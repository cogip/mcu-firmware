// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file pf_parameters.cpp
/// @brief Lift motor PID parameters registry and handlers implementation.

// Project includes
#include "pf_parameters.hpp"
#include "KeyHash.hpp"
#include "lift_common_conf.hpp"
#include "parameter_handler/ParameterHandler.hpp"
#include "pf_common/platform_common.hpp"
#include "pf_common/uuids.hpp"

namespace cogip {
namespace pf {
namespace parameters {

using namespace cogip::parameter;
using cogip::utils::operator"" _key_hash;

/// @brief Parameter key hashes for identification
///
/// @note Each parameter is identified by a unique 32-bit hash computed from its string key.

/// Maximum number of parameters in the registry
constexpr size_t MAX_PARAMETERS_NUMBER = 16;

// Lift pose PID
constexpr uint32_t LIFT_POSE_PID_KP_KEY = "lift_pose_pid_kp"_key_hash;
constexpr uint32_t LIFT_POSE_PID_KI_KEY = "lift_pose_pid_ki"_key_hash;
constexpr uint32_t LIFT_POSE_PID_KD_KEY = "lift_pose_pid_kd"_key_hash;
// Lift speed PID
constexpr uint32_t LIFT_SPEED_PID_KP_KEY = "lift_speed_pid_kp"_key_hash;
constexpr uint32_t LIFT_SPEED_PID_KI_KEY = "lift_speed_pid_ki"_key_hash;
constexpr uint32_t LIFT_SPEED_PID_KD_KEY = "lift_speed_pid_kd"_key_hash;

// Parameter handler type
using ParameterHandlerType = parameter_handler::ParameterHandler<MAX_PARAMETERS_NUMBER>;

/// @brief Registry mapping parameter key hashes to parameters object references
///
/// @warning The registry should only contain parameters available for read/write through canpb
static const ParameterHandlerType::Registry registry = {
    /// Lift PID parameters
    // Pose PID
    {LIFT_POSE_PID_KP_KEY, cogip::app::actuators::motor_lift_pose_pid_kp},
    {LIFT_POSE_PID_KI_KEY, cogip::app::actuators::motor_lift_pose_pid_ki},
    {LIFT_POSE_PID_KD_KEY, cogip::app::actuators::motor_lift_pose_pid_kd},
    // Speed PID
    {LIFT_SPEED_PID_KP_KEY, cogip::app::actuators::motor_lift_speed_pid_kp},
    {LIFT_SPEED_PID_KI_KEY, cogip::app::actuators::motor_lift_speed_pid_ki},
    {LIFT_SPEED_PID_KD_KEY, cogip::app::actuators::motor_lift_speed_pid_kd},
};

static ParameterHandlerType parameter_handler(registry);

void pf_handle_parameter_get(cogip::canpb::ReadBuffer& buffer)
{
    auto response = parameter_handler.handle_get(buffer);
    // Only respond if this board owns the parameter
    if (response.has_value()) {
        pf_get_canpb().send_message(pf_common::parameter_get_response_uuid, &response.value());
    }
}

void pf_handle_parameter_set(cogip::canpb::ReadBuffer& buffer)
{
    auto response = parameter_handler.handle_set(buffer);
    // Only respond if this board owns the parameter
    if (response.has_value()) {
        pf_get_canpb().send_message(pf_common::parameter_set_response_uuid, &response.value());
    }
}

void init()
{
    cogip::canpb::CanProtobuf& canpb = pf_get_canpb();
    canpb.register_message_handler(pf_common::parameter_get_uuid,
                                   canpb::message_handler_t::create<pf_handle_parameter_get>());
    canpb.register_message_handler(pf_common::parameter_set_uuid,
                                   canpb::message_handler_t::create<pf_handle_parameter_set>());
}

} // namespace parameters
} // namespace pf
} // namespace cogip
