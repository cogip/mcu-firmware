//// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
//// This file is subject to the terms and conditions of the GNU Lesser
//// General Public License v2.1. See the file LICENSE in the top level
//// directory for more details.

/// @file motion_control_parameters.hpp
/// @brief Motion control parameter definitions, key hashes, and registry
///
/// @note Parameter<> objects are declared here with their policies and flash
///       storage keys. Default values come from the robot-specific app_conf.hpp.

#pragma once

#include <cstdint>

#include "KeyHash.hpp"
#include "app_conf.hpp"
#include "canpb/ReadBuffer.hpp"
#include "parameter/Parameter.hpp"

using namespace cogip::parameter;
using cogip::utils::operator"" _key_hash;

// ============================================================================
// Parameter key hashes
// ============================================================================

/// Odometry parameters keys
constexpr uint32_t QDEC_LEFT_POLARITY_KEY = "qdec_left_polarity"_key_hash;
constexpr uint32_t QDEC_RIGHT_POLARITY_KEY = "qdec_right_polarity"_key_hash;
constexpr uint32_t LEFT_WHEEL_DIAMETER_KEY = "left_wheel_diameter_mm"_key_hash;
constexpr uint32_t RIGHT_WHEEL_DIAMETER_KEY = "right_wheel_diameter_mm"_key_hash;
constexpr uint32_t ENCODER_WHEELS_DISTANCE_KEY = "encoder_wheels_distance_mm"_key_hash;
constexpr uint32_t ENCODER_WHEELS_RESOLUTION_KEY = "encoder_wheels_resolution_pulses"_key_hash;

// Linear pose PID
constexpr uint32_t LINEAR_POSE_PID_KP_KEY = "linear_pose_pid_kp"_key_hash;
constexpr uint32_t LINEAR_POSE_PID_KI_KEY = "linear_pose_pid_ki"_key_hash;
constexpr uint32_t LINEAR_POSE_PID_KD_KEY = "linear_pose_pid_kd"_key_hash;
// Angular pose PID
constexpr uint32_t ANGULAR_POSE_PID_KP_KEY = "angular_pose_pid_kp"_key_hash;
constexpr uint32_t ANGULAR_POSE_PID_KI_KEY = "angular_pose_pid_ki"_key_hash;
constexpr uint32_t ANGULAR_POSE_PID_KD_KEY = "angular_pose_pid_kd"_key_hash;
// Linear speed PID
constexpr uint32_t LINEAR_SPEED_PID_KP_KEY = "linear_speed_pid_kp"_key_hash;
constexpr uint32_t LINEAR_SPEED_PID_KI_KEY = "linear_speed_pid_ki"_key_hash;
constexpr uint32_t LINEAR_SPEED_PID_KD_KEY = "linear_speed_pid_kd"_key_hash;
// Angular speed PID
constexpr uint32_t ANGULAR_SPEED_PID_KP_KEY = "angular_speed_pid_kp"_key_hash;
constexpr uint32_t ANGULAR_SPEED_PID_KI_KEY = "angular_speed_pid_ki"_key_hash;
constexpr uint32_t ANGULAR_SPEED_PID_KD_KEY = "angular_speed_pid_kd"_key_hash;

// Tracker linear pose PID
constexpr uint32_t TRACKER_LINEAR_POSE_PID_KP_KEY = "tracker_linear_pose_pid_kp"_key_hash;
constexpr uint32_t TRACKER_LINEAR_POSE_PID_KI_KEY = "tracker_linear_pose_pid_ki"_key_hash;
constexpr uint32_t TRACKER_LINEAR_POSE_PID_KD_KEY = "tracker_linear_pose_pid_kd"_key_hash;
// Tracker angular pose PID
constexpr uint32_t TRACKER_ANGULAR_POSE_PID_KP_KEY = "tracker_angular_pose_pid_kp"_key_hash;
constexpr uint32_t TRACKER_ANGULAR_POSE_PID_KI_KEY = "tracker_angular_pose_pid_ki"_key_hash;
constexpr uint32_t TRACKER_ANGULAR_POSE_PID_KD_KEY = "tracker_angular_pose_pid_kd"_key_hash;
// Tracker linear speed PID
constexpr uint32_t TRACKER_LINEAR_SPEED_PID_KP_KEY = "tracker_linear_speed_pid_kp"_key_hash;
constexpr uint32_t TRACKER_LINEAR_SPEED_PID_KI_KEY = "tracker_linear_speed_pid_ki"_key_hash;
constexpr uint32_t TRACKER_LINEAR_SPEED_PID_KD_KEY = "tracker_linear_speed_pid_kd"_key_hash;
// Tracker angular speed PID
constexpr uint32_t TRACKER_ANGULAR_SPEED_PID_KP_KEY = "tracker_angular_speed_pid_kp"_key_hash;
constexpr uint32_t TRACKER_ANGULAR_SPEED_PID_KI_KEY = "tracker_angular_speed_pid_ki"_key_hash;
constexpr uint32_t TRACKER_ANGULAR_SPEED_PID_KD_KEY = "tracker_angular_speed_pid_kd"_key_hash;

// ============================================================================
// Parameter objects (default values from robot-specific app_conf.hpp)
// ============================================================================
// clang-format off
/// Quadrature decoding polarity
inline Parameter<float, ReadOnly> qdec_left_polarity{default_qdec_left_polarity};
inline Parameter<float, ReadOnly> qdec_right_polarity{default_qdec_right_polarity};

/// Encoder parameters
inline Parameter<float> left_encoder_wheels_diameter_mm{default_left_encoder_wheels_diameter_mm};
inline Parameter<float> right_encoder_wheels_diameter_mm{default_right_encoder_wheels_diameter_mm};
inline Parameter<float> encoder_wheels_distance_mm{default_encoder_wheels_distance_mm};
inline Parameter<float, ReadOnly> encoder_wheels_resolution_pulses{default_encoder_wheels_resolution_pulses};

// Linear pose PID (QUADPID chain)
inline Parameter<float, NonNegative> linear_pose_pid_kp{default_linear_pose_pid_kp};
inline Parameter<float, NonNegative> linear_pose_pid_ki{default_linear_pose_pid_ki};
inline Parameter<float, NonNegative> linear_pose_pid_kd{default_linear_pose_pid_kd};
// Angular pose PID (QUADPID chain)
inline Parameter<float, NonNegative> angular_pose_pid_kp{default_angular_pose_pid_kp};
inline Parameter<float, NonNegative> angular_pose_pid_ki{default_angular_pose_pid_ki};
inline Parameter<float, NonNegative> angular_pose_pid_kd{default_angular_pose_pid_kd};
// Linear speed PID (QUADPID chain)
inline Parameter<float, NonNegative> linear_speed_pid_kp{default_linear_speed_pid_kp};
inline Parameter<float, NonNegative> linear_speed_pid_ki{default_linear_speed_pid_ki};
inline Parameter<float, NonNegative> linear_speed_pid_kd{default_linear_speed_pid_kd};
// Angular speed PID (QUADPID chain)
inline Parameter<float, NonNegative> angular_speed_pid_kp{default_angular_speed_pid_kp};
inline Parameter<float, NonNegative> angular_speed_pid_ki{default_angular_speed_pid_ki};
inline Parameter<float, NonNegative> angular_speed_pid_kd{default_angular_speed_pid_kd};

// Tracker linear pose PID
inline Parameter<float, NonNegative> tracker_linear_pose_pid_kp{default_tracker_linear_pose_pid_kp};
inline Parameter<float, NonNegative> tracker_linear_pose_pid_ki{default_tracker_linear_pose_pid_ki};
inline Parameter<float, NonNegative> tracker_linear_pose_pid_kd{default_tracker_linear_pose_pid_kd};
// Tracker angular pose PID
inline Parameter<float, NonNegative> tracker_angular_pose_pid_kp{default_tracker_angular_pose_pid_kp};
inline Parameter<float, NonNegative> tracker_angular_pose_pid_ki{default_tracker_angular_pose_pid_ki};
inline Parameter<float, NonNegative> tracker_angular_pose_pid_kd{default_tracker_angular_pose_pid_kd};
// Tracker linear speed PID
inline Parameter<float, NonNegative> tracker_linear_speed_pid_kp{default_tracker_linear_speed_pid_kp};
inline Parameter<float, NonNegative> tracker_linear_speed_pid_ki{default_tracker_linear_speed_pid_ki};
inline Parameter<float, NonNegative> tracker_linear_speed_pid_kd{default_tracker_linear_speed_pid_kd};
// Tracker angular speed PID
inline Parameter<float, NonNegative> tracker_angular_speed_pid_kp{default_tracker_angular_speed_pid_kp};
inline Parameter<float, NonNegative> tracker_angular_speed_pid_ki{default_tracker_angular_speed_pid_ki};
inline Parameter<float, NonNegative> tracker_angular_speed_pid_kd{default_tracker_angular_speed_pid_kd};

// PID integral limits
inline Parameter<float, NonNegative> linear_pose_pid_integral_limit{default_linear_pose_pid_integral_limit};
inline Parameter<float, NonNegative> angular_pose_pid_integral_limit{default_angular_pose_pid_integral_limit};
inline Parameter<float, NonNegative> linear_speed_pid_integral_limit{default_linear_speed_pid_integral_limit};
inline Parameter<float, NonNegative> angular_speed_pid_integral_limit{default_angular_speed_pid_integral_limit};

// Tracker PID integral limits
inline Parameter<float, NonNegative> tracker_linear_pose_pid_integral_limit{default_tracker_linear_pose_pid_integral_limit};
inline Parameter<float, NonNegative> tracker_angular_pose_pid_integral_limit{default_tracker_angular_pose_pid_integral_limit};
inline Parameter<float, NonNegative> tracker_linear_speed_pid_integral_limit{default_tracker_linear_speed_pid_integral_limit};
inline Parameter<float, NonNegative> tracker_angular_speed_pid_integral_limit{default_tracker_angular_speed_pid_integral_limit};
// clang-format on
// ============================================================================
// Parameter registry handlers (canpb)
// ============================================================================

namespace cogip {
namespace pf {
namespace motion_control {

/// @brief Handle parameter get request from canpb
void pf_handle_parameter_get(cogip::canpb::ReadBuffer& buffer);

/// @brief Handle parameter set request from CAN bus
void pf_handle_parameter_set(cogip::canpb::ReadBuffer& buffer);

} // namespace motion_control
} // namespace pf
} // namespace cogip
