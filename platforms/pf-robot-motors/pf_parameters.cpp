// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file pf_parameters.cpp
/// @brief Lift motor PID parameters registry and handlers implementation.

// Project includes
#include "pf_parameters.hpp"
#include "lift_common_conf.hpp"
#include "parameter_handler/ParameterHandler.hpp"
#include "pf_common/platform_common.hpp"
#include "pf_common/uuids.hpp"

namespace cogip {
namespace pf {
namespace parameters {

using parameter_handler::ParameterEntry;

/// @brief Parameters of the board, identified by their name
///
/// @warning The registry should only contain parameters available for read/write through canpb
static const ParameterEntry registry[] = {
    /// Lift PID parameters
    // Pose PID
    {"lift_pose_pid_kp", cogip::app::actuators::motor_lift_pose_pid_kp},
    {"lift_pose_pid_ki", cogip::app::actuators::motor_lift_pose_pid_ki},
    {"lift_pose_pid_kd", cogip::app::actuators::motor_lift_pose_pid_kd},
    // Speed PID
    {"lift_speed_pid_kp", cogip::app::actuators::motor_lift_speed_pid_kp},
    {"lift_speed_pid_ki", cogip::app::actuators::motor_lift_speed_pid_ki},
    {"lift_speed_pid_kd", cogip::app::actuators::motor_lift_speed_pid_kd},
};

static parameter_handler::ParameterHandler parameter_handler(registry);

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
