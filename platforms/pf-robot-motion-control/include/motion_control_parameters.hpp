//// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
//// This file is subject to the terms and conditions of the GNU Lesser
//// General Public License v2.1. See the file LICENSE in the top level
//// directory for more details.

/// @file motion_control_parameters.hpp
/// @brief Motion control parameter definitions
///
/// @note Parameter<> objects are declared here with their ranges and policies.
///       Default values come from the robot-specific app_conf.hpp. Names and flash
///       persistence are declared in the registry (motion_control_parameters.cpp).

#pragma once

#include <cstdint>

#include "app_conf.hpp"
#include "canpb/ReadBuffer.hpp"
#include "parameter/ConversionPolicies.hpp"
#include "parameter/Parameter.hpp"

// ============================================================================
// Unit conversion macros and control loop period
// ============================================================================

/// Conversion macro from speed in x/s to x/period (eg. mm/s or rad/s)
#define X_SEC_TO_X_PERIOD(speed, period) (((speed) * (period)) / 1000.0)
/// Conversion macro from acceleration in x/s² to x/period² (eg. mm/s² or rad/s²)
#define X_SEC2_TO_X_PERIOD2(acc, period) ((acc) * (((period) * (period)) / (1000.0 * 1000.0)))

namespace cogip {
namespace pf {
namespace motion_control {

constexpr uint16_t motion_control_thread_period_ms = 20; ///< controller thread loop period

/// @name Platform speed/acceleration constexpr (per-period units, derived from app_conf.hpp)
/// @{
constexpr float platform_max_acc_linear_mm_per_period2 =
    X_SEC2_TO_X_PERIOD2(max_acc_mm_per_s2, motion_control_thread_period_ms);
constexpr float platform_max_dec_linear_mm_per_period2 =
    X_SEC2_TO_X_PERIOD2(max_dec_mm_per_s2, motion_control_thread_period_ms);
constexpr float platform_min_speed_linear_mm_per_period =
    X_SEC_TO_X_PERIOD(min_speed_mm_per_s, motion_control_thread_period_ms);
constexpr float platform_max_speed_linear_mm_per_period =
    X_SEC_TO_X_PERIOD(max_speed_mm_per_s, motion_control_thread_period_ms);
constexpr float platform_low_speed_linear_mm_per_period =
    (platform_max_speed_linear_mm_per_period / 4);
constexpr float platform_normal_speed_linear_mm_per_period =
    (platform_max_speed_linear_mm_per_period / 2);

constexpr float platform_max_acc_angular_deg_per_period2 =
    X_SEC2_TO_X_PERIOD2(max_acc_deg_per_s2, motion_control_thread_period_ms);
constexpr float platform_max_dec_angular_deg_per_period2 =
    X_SEC2_TO_X_PERIOD2(max_dec_deg_per_s2, motion_control_thread_period_ms);
constexpr float platform_min_speed_angular_deg_per_period =
    X_SEC_TO_X_PERIOD(min_speed_deg_per_s, motion_control_thread_period_ms);
constexpr float platform_max_speed_angular_deg_per_period =
    X_SEC_TO_X_PERIOD(max_speed_deg_per_s, motion_control_thread_period_ms);
constexpr float platform_low_speed_angular_deg_per_period =
    (platform_max_speed_angular_deg_per_period / 4);
constexpr float platform_normal_speed_angular_deg_per_period =
    (platform_max_speed_angular_deg_per_period / 2);

constexpr double platform_linear_anti_blocking_speed_threshold_mm_per_period =
    (motion_control_thread_period_ms * platform_linear_anti_blocking_speed_threshold_mm_per_s) /
    1000;
constexpr double platform_linear_anti_blocking_error_threshold_mm_per_period =
    (motion_control_thread_period_ms * platform_linear_anti_blocking_error_threshold_mm_per_s) /
    1000;
/// @}

} // namespace motion_control
} // namespace pf
} // namespace cogip

using namespace cogip::pf::motion_control;

// ============================================================================
// Parameter objects (default values from robot-specific app_conf.hpp)
// ============================================================================
// clang-format off
/// Quadrature decoding polarity
inline cogip::parameter::Parameter<float, cogip::parameter::ReadOnly> qdec_left_polarity{default_qdec_left_polarity};
inline cogip::parameter::Parameter<float, cogip::parameter::ReadOnly> qdec_right_polarity{default_qdec_right_polarity};

/// Encoder parameters
inline cogip::parameter::Parameter<float> left_encoder_wheels_diameter_mm{default_left_encoder_wheels_diameter_mm};
inline cogip::parameter::Parameter<float> right_encoder_wheels_diameter_mm{default_right_encoder_wheels_diameter_mm};
inline cogip::parameter::Parameter<float> encoder_wheels_distance_mm{default_encoder_wheels_distance_mm};
inline cogip::parameter::Parameter<float, cogip::parameter::ReadOnly> encoder_wheels_resolution_pulses{default_encoder_wheels_resolution_pulses};

// OTOS localization calibration scalars
// Range [0.872, 1.127] is the valid domain of the OTOS linear/angular
// calibration scalars. The range rejects out-of-range flash entries
// or bus writes instead of silently normalizing them, so calibration
// mistakes surface as an invalid parameter.
#ifdef ROBOT_HAS_OTOS
inline cogip::parameter::Parameter<float> otos_linear_scalar{default_otos_linear_scalar, {0.872f, 1.127f}};
inline cogip::parameter::Parameter<float> otos_angular_scalar{default_otos_angular_scalar, {0.872f, 1.127f}};
#endif

// Linear pose PID (QUADPID chain)
inline cogip::parameter::Parameter<float> linear_pose_pid_kp{default_linear_pose_pid_kp, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> linear_pose_pid_ki{default_linear_pose_pid_ki, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> linear_pose_pid_kd{default_linear_pose_pid_kd, cogip::parameter::non_negative<float>};
// Angular pose PID (QUADPID chain)
inline cogip::parameter::Parameter<float> angular_pose_pid_kp{default_angular_pose_pid_kp, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> angular_pose_pid_ki{default_angular_pose_pid_ki, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> angular_pose_pid_kd{default_angular_pose_pid_kd, cogip::parameter::non_negative<float>};
// Linear speed PID (QUADPID chain)
inline cogip::parameter::Parameter<float> linear_speed_pid_kp{default_linear_speed_pid_kp, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> linear_speed_pid_ki{default_linear_speed_pid_ki, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> linear_speed_pid_kd{default_linear_speed_pid_kd, cogip::parameter::non_negative<float>};
// Angular speed PID (QUADPID chain)
inline cogip::parameter::Parameter<float> angular_speed_pid_kp{default_angular_speed_pid_kp, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> angular_speed_pid_ki{default_angular_speed_pid_ki, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> angular_speed_pid_kd{default_angular_speed_pid_kd, cogip::parameter::non_negative<float>};

// Tracker linear pose PID
inline cogip::parameter::Parameter<float> tracker_linear_pose_pid_kp{default_tracker_linear_pose_pid_kp, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> tracker_linear_pose_pid_ki{default_tracker_linear_pose_pid_ki, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> tracker_linear_pose_pid_kd{default_tracker_linear_pose_pid_kd, cogip::parameter::non_negative<float>};
// Tracker angular pose PID
inline cogip::parameter::Parameter<float> tracker_angular_pose_pid_kp{default_tracker_angular_pose_pid_kp, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> tracker_angular_pose_pid_ki{default_tracker_angular_pose_pid_ki, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> tracker_angular_pose_pid_kd{default_tracker_angular_pose_pid_kd, cogip::parameter::non_negative<float>};
// Tracker linear speed PID
inline cogip::parameter::Parameter<float> tracker_linear_speed_pid_kp{default_tracker_linear_speed_pid_kp, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> tracker_linear_speed_pid_ki{default_tracker_linear_speed_pid_ki, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> tracker_linear_speed_pid_kd{default_tracker_linear_speed_pid_kd, cogip::parameter::non_negative<float>};
// Tracker angular speed PID
inline cogip::parameter::Parameter<float> tracker_angular_speed_pid_kp{default_tracker_angular_speed_pid_kp, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> tracker_angular_speed_pid_ki{default_tracker_angular_speed_pid_ki, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> tracker_angular_speed_pid_kd{default_tracker_angular_speed_pid_kd, cogip::parameter::non_negative<float>};

// Brake linear speed PID
inline cogip::parameter::Parameter<float> brake_linear_speed_pid_kp{default_brake_linear_speed_pid_kp, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> brake_linear_speed_pid_ki{default_brake_linear_speed_pid_ki, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> brake_linear_speed_pid_kd{default_brake_linear_speed_pid_kd, cogip::parameter::non_negative<float>};
// Brake angular speed PID
inline cogip::parameter::Parameter<float> brake_angular_speed_pid_kp{default_brake_angular_speed_pid_kp, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> brake_angular_speed_pid_ki{default_brake_angular_speed_pid_ki, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> brake_angular_speed_pid_kd{default_brake_angular_speed_pid_kd, cogip::parameter::non_negative<float>};

// PID integral limits
inline cogip::parameter::Parameter<float> linear_pose_pid_integral_limit{default_linear_pose_pid_integral_limit, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> angular_pose_pid_integral_limit{default_angular_pose_pid_integral_limit, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> linear_speed_pid_integral_limit{default_linear_speed_pid_integral_limit, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> angular_speed_pid_integral_limit{default_angular_speed_pid_integral_limit, cogip::parameter::non_negative<float>};

// Tracker PID integral limits
inline cogip::parameter::Parameter<float> tracker_linear_pose_pid_integral_limit{default_tracker_linear_pose_pid_integral_limit, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> tracker_angular_pose_pid_integral_limit{default_tracker_angular_pose_pid_integral_limit, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> tracker_linear_speed_pid_integral_limit{default_tracker_linear_speed_pid_integral_limit, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> tracker_angular_speed_pid_integral_limit{default_tracker_angular_speed_pid_integral_limit, cogip::parameter::non_negative<float>};

// Brake speed PID integral limits
inline cogip::parameter::Parameter<float> brake_linear_speed_pid_integral_limit{default_brake_linear_speed_pid_integral_limit, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float> brake_angular_speed_pid_integral_limit{default_brake_angular_speed_pid_integral_limit, cogip::parameter::non_negative<float>};

// Pose straight filter thresholds (absolute units: mm, deg)
inline cogip::parameter::Parameter<float> param_linear_threshold{linear_threshold, {1.0f, 10.0f}};
inline cogip::parameter::Parameter<float> param_angular_threshold{angular_threshold, {1.0f, 5.0f}};
inline cogip::parameter::Parameter<float> param_angular_intermediate_threshold{angular_intermediate_threshold, {1.0f, 5.0f}};

// Speed limits (internal: /period, protobuf: /s)
inline cogip::parameter::Parameter<float, cogip::parameter::SpeedConversion<motion_control_thread_period_ms>> param_min_speed_linear{platform_min_speed_linear_mm_per_period, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float, cogip::parameter::SpeedConversion<motion_control_thread_period_ms>> param_max_speed_linear{platform_max_speed_linear_mm_per_period, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float, cogip::parameter::SpeedConversion<motion_control_thread_period_ms>> param_min_speed_angular{platform_min_speed_angular_deg_per_period, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float, cogip::parameter::SpeedConversion<motion_control_thread_period_ms>> param_max_speed_angular{platform_max_speed_angular_deg_per_period, cogip::parameter::non_negative<float>};

// Acceleration/deceleration limits (internal: /period², protobuf: /s²)
inline cogip::parameter::Parameter<float, cogip::parameter::AccelerationConversion<motion_control_thread_period_ms>> param_max_acc_linear{platform_max_acc_linear_mm_per_period2, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float, cogip::parameter::AccelerationConversion<motion_control_thread_period_ms>> param_max_dec_linear{platform_max_dec_linear_mm_per_period2, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float, cogip::parameter::AccelerationConversion<motion_control_thread_period_ms>> param_max_acc_angular{platform_max_acc_angular_deg_per_period2, cogip::parameter::non_negative<float>};
inline cogip::parameter::Parameter<float, cogip::parameter::AccelerationConversion<motion_control_thread_period_ms>> param_max_dec_angular{platform_max_dec_angular_deg_per_period2, cogip::parameter::non_negative<float>};
// clang-format on
// ============================================================================
// Parameter registry handlers (canpb)
// ============================================================================

namespace cogip {
namespace pf {
namespace motion_control {

/// @brief Load all parameters from flash persistent storage
void pf_load_parameters();

/// @brief Handle parameter get request from canpb
void pf_handle_parameter_get(cogip::canpb::ReadBuffer& buffer);

/// @brief Handle parameter set request from CAN bus
void pf_handle_parameter_set(cogip::canpb::ReadBuffer& buffer);

/// @brief Handle parameter reset request from CAN bus
/// @note Erases the persisted value and restores the compile-time default
void pf_handle_parameter_reset(cogip::canpb::ReadBuffer& buffer);

} // namespace motion_control
} // namespace pf
} // namespace cogip
