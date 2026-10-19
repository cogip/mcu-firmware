// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file motion_control_parameters.cpp
/// @brief Motion control parameters registry and handlers implementation.

// RIOT includes
#include "log.h"

// Project includes
#include "motion_control_parameters.hpp"
#include "parameter_handler/ParameterHandler.hpp"
#include "platform.hpp"

namespace cogip {
namespace pf {
namespace motion_control {

using parameter_handler::ParameterEntry;

/// @brief Parameters of the board, identified by their name
///
/// @note Persistent parameters are stored in flash under the hash of their name.
///
/// @warning The registry should only contains parameters available for read/write through canpb
static const ParameterEntry registry[] = {
    /// Odometry parameters
    {"qdec_left_polarity", qdec_left_polarity},
    {"qdec_right_polarity", qdec_right_polarity},
    {"left_wheel_diameter_mm", left_encoder_wheels_diameter_mm, true},
    {"right_wheel_diameter_mm", right_encoder_wheels_diameter_mm, true},
    {"encoder_wheels_distance_mm", encoder_wheels_distance_mm, true},
    {"encoder_wheels_resolution_pulses", encoder_wheels_resolution_pulses},
#ifdef ROBOT_HAS_OTOS
    /// OTOS localization calibration scalars
    {"otos_linear_scalar", otos_linear_scalar, true},
    {"otos_angular_scalar", otos_angular_scalar, true},
#endif
    /// QuadPID chain PID parameters
    // Linear pose PID
    {"linear_pose_pid_kp", linear_pose_pid_kp, true},
    {"linear_pose_pid_ki", linear_pose_pid_ki, true},
    {"linear_pose_pid_kd", linear_pose_pid_kd, true},
    // Angular pose PID
    {"angular_pose_pid_kp", angular_pose_pid_kp, true},
    {"angular_pose_pid_ki", angular_pose_pid_ki, true},
    {"angular_pose_pid_kd", angular_pose_pid_kd, true},
    // Linear speed PID
    {"linear_speed_pid_kp", linear_speed_pid_kp, true},
    {"linear_speed_pid_ki", linear_speed_pid_ki, true},
    {"linear_speed_pid_kd", linear_speed_pid_kd, true},
    // Angular speed PID
    {"angular_speed_pid_kp", angular_speed_pid_kp, true},
    {"angular_speed_pid_ki", angular_speed_pid_ki, true},
    {"angular_speed_pid_kd", angular_speed_pid_kd, true},
    /// Tracker chain PID parameters
    // Tracker linear pose PID
    {"tracker_linear_pose_pid_kp", tracker_linear_pose_pid_kp, true},
    {"tracker_linear_pose_pid_ki", tracker_linear_pose_pid_ki, true},
    {"tracker_linear_pose_pid_kd", tracker_linear_pose_pid_kd, true},
    // Tracker angular pose PID
    {"tracker_angular_pose_pid_kp", tracker_angular_pose_pid_kp, true},
    {"tracker_angular_pose_pid_ki", tracker_angular_pose_pid_ki, true},
    {"tracker_angular_pose_pid_kd", tracker_angular_pose_pid_kd, true},
    // Tracker linear speed PID
    {"tracker_linear_speed_pid_kp", tracker_linear_speed_pid_kp, true},
    {"tracker_linear_speed_pid_ki", tracker_linear_speed_pid_ki, true},
    {"tracker_linear_speed_pid_kd", tracker_linear_speed_pid_kd, true},
    // Tracker angular speed PID
    {"tracker_angular_speed_pid_kp", tracker_angular_speed_pid_kp, true},
    {"tracker_angular_speed_pid_ki", tracker_angular_speed_pid_ki, true},
    {"tracker_angular_speed_pid_kd", tracker_angular_speed_pid_kd, true},
    // Brake linear speed PID
    {"brake_linear_speed_pid_kp", brake_linear_speed_pid_kp, true},
    {"brake_linear_speed_pid_ki", brake_linear_speed_pid_ki, true},
    {"brake_linear_speed_pid_kd", brake_linear_speed_pid_kd, true},
    // Brake angular speed PID
    {"brake_angular_speed_pid_kp", brake_angular_speed_pid_kp, true},
    {"brake_angular_speed_pid_ki", brake_angular_speed_pid_ki, true},
    {"brake_angular_speed_pid_kd", brake_angular_speed_pid_kd, true},
    /// Pose straight filter thresholds
    {"linear_threshold", param_linear_threshold, true},
    {"angular_threshold", param_angular_threshold, true},
    {"angular_intermediate_threshold", param_angular_intermediate_threshold, true},
    /// Speed and acceleration limits
    {"min_speed_linear", param_min_speed_linear},
    {"max_speed_linear", param_max_speed_linear},
    {"max_acc_linear", param_max_acc_linear},
    {"max_dec_linear", param_max_dec_linear},
    {"min_speed_angular", param_min_speed_angular},
    {"max_speed_angular", param_max_speed_angular},
    {"max_acc_angular", param_max_acc_angular},
    {"max_dec_angular", param_max_dec_angular},
};

static parameter_handler::ParameterHandler parameter_handler(registry);

void pf_load_parameters()
{
    parameter_handler.load();
}

void pf_handle_parameter_get(cogip::canpb::ReadBuffer& buffer)
{
    auto response = parameter_handler.handle_get(buffer);
    // Only respond if this board owns the parameter
    if (response.has_value()) {
        pf_get_canpb().send_message(parameter_get_response_uuid, &response.value());
    }
}

void pf_handle_parameter_set(cogip::canpb::ReadBuffer& buffer)
{
    auto response = parameter_handler.handle_set(buffer);
    // Only respond if this board owns the parameter
    if (response.has_value()) {
        pf_get_canpb().send_message(parameter_set_response_uuid, &response.value());
    }
}

void pf_handle_parameter_reset(cogip::canpb::ReadBuffer& buffer)
{
    auto response = parameter_handler.handle_reset(buffer);
    // Only respond if this board owns the parameter
    if (response.has_value()) {
        pf_get_canpb().send_message(parameter_reset_response_uuid, &response.value());
    }
}

} // namespace motion_control
} // namespace pf
} // namespace cogip
