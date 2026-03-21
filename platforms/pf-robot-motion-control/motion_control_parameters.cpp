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

/// Maximum number of parameters in the registry
constexpr size_t MAX_PARAMETERS_NUMBER = 64;

// Parameter handler type
using ParameterHandlerType = parameter_handler::ParameterHandler<MAX_PARAMETERS_NUMBER>;

/// @brief Registry mapping parameter key hashes to parameters object references
///
/// @warning The registry should only contains parameters available for read/write through canpb
static const ParameterHandlerType::Registry registry = {
    /// Odometry parameters
    {QDEC_LEFT_POLARITY_KEY, qdec_left_polarity},
    {QDEC_RIGHT_POLARITY_KEY, qdec_right_polarity},
    {LEFT_WHEEL_DIAMETER_KEY, left_encoder_wheels_diameter_mm},
    {RIGHT_WHEEL_DIAMETER_KEY, right_encoder_wheels_diameter_mm},
    {ENCODER_WHEELS_DISTANCE_KEY, encoder_wheels_distance_mm},
    {ENCODER_WHEELS_RESOLUTION_KEY, encoder_wheels_resolution_pulses},
    /// QuadPID chain PID parameters
    // Linear pose PID
    {LINEAR_POSE_PID_KP_KEY, linear_pose_pid_kp},
    {LINEAR_POSE_PID_KI_KEY, linear_pose_pid_ki},
    {LINEAR_POSE_PID_KD_KEY, linear_pose_pid_kd},
    // Angular pose PID
    {ANGULAR_POSE_PID_KP_KEY, angular_pose_pid_kp},
    {ANGULAR_POSE_PID_KI_KEY, angular_pose_pid_ki},
    {ANGULAR_POSE_PID_KD_KEY, angular_pose_pid_kd},
    // Linear speed PID
    {LINEAR_SPEED_PID_KP_KEY, linear_speed_pid_kp},
    {LINEAR_SPEED_PID_KI_KEY, linear_speed_pid_ki},
    {LINEAR_SPEED_PID_KD_KEY, linear_speed_pid_kd},
    // Angular speed PID
    {ANGULAR_SPEED_PID_KP_KEY, angular_speed_pid_kp},
    {ANGULAR_SPEED_PID_KI_KEY, angular_speed_pid_ki},
    {ANGULAR_SPEED_PID_KD_KEY, angular_speed_pid_kd},
    /// Tracker chain PID parameters
    // Tracker linear pose PID
    {TRACKER_LINEAR_POSE_PID_KP_KEY, tracker_linear_pose_pid_kp},
    {TRACKER_LINEAR_POSE_PID_KI_KEY, tracker_linear_pose_pid_ki},
    {TRACKER_LINEAR_POSE_PID_KD_KEY, tracker_linear_pose_pid_kd},
    // Tracker angular pose PID
    {TRACKER_ANGULAR_POSE_PID_KP_KEY, tracker_angular_pose_pid_kp},
    {TRACKER_ANGULAR_POSE_PID_KI_KEY, tracker_angular_pose_pid_ki},
    {TRACKER_ANGULAR_POSE_PID_KD_KEY, tracker_angular_pose_pid_kd},
    // Tracker linear speed PID
    {TRACKER_LINEAR_SPEED_PID_KP_KEY, tracker_linear_speed_pid_kp},
    {TRACKER_LINEAR_SPEED_PID_KI_KEY, tracker_linear_speed_pid_ki},
    {TRACKER_LINEAR_SPEED_PID_KD_KEY, tracker_linear_speed_pid_kd},
    // Tracker angular speed PID
    {TRACKER_ANGULAR_SPEED_PID_KP_KEY, tracker_angular_speed_pid_kp},
    {TRACKER_ANGULAR_SPEED_PID_KI_KEY, tracker_angular_speed_pid_ki},
    {TRACKER_ANGULAR_SPEED_PID_KD_KEY, tracker_angular_speed_pid_kd},
};

static ParameterHandlerType parameter_handler(registry);

void pf_load_parameters()
{
    for (auto& entry : registry) {
        if (!entry.second.load()) {
            LOG_WARNING("Parameter 0x%08" PRIx32 ": flash load failed, using default\n",
                        entry.first);
        }
    }
    LOG_INFO("All parameters loaded (%u entries)\n", static_cast<unsigned>(registry.size()));
}

void pf_handle_parameter_get(cogip::canpb::ReadBuffer& buffer)
{
    PB_ParameterGetResponse response = parameter_handler.handle_get(buffer);
    pf_get_canpb().send_message(parameter_get_response_uuid, &response);
}

void pf_handle_parameter_set(cogip::canpb::ReadBuffer& buffer)
{
    PB_ParameterSetResponse response = parameter_handler.handle_set(buffer);
    pf_get_canpb().send_message(parameter_set_response_uuid, &response);
}

} // namespace motion_control
} // namespace pf
} // namespace cogip
