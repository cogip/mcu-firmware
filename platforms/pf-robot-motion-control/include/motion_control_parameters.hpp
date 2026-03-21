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
#include "parameter/ConversionPolicies.hpp"
#include "parameter/Parameter.hpp"
#include "parameter/StoragePolicies.hpp"

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

#ifdef ROBOT_HAS_OTOS
// OTOS localization calibration scalars
// ROBOT_HAS_OTOS is defined by the OTOS robot configuration and gates the registration of the
// scalars in the CAN parameter registry so encoder-based robots are not polluted with keys they do
// not own.
constexpr uint32_t OTOS_LINEAR_SCALAR_KEY = "otos_linear_scalar"_key_hash;
constexpr uint32_t OTOS_ANGULAR_SCALAR_KEY = "otos_angular_scalar"_key_hash;
#endif

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

// Pose straight filter thresholds
constexpr uint32_t LINEAR_THRESHOLD_KEY = "linear_threshold"_key_hash;
constexpr uint32_t ANGULAR_THRESHOLD_KEY = "angular_threshold"_key_hash;
constexpr uint32_t ANGULAR_INTERMEDIATE_THRESHOLD_KEY = "angular_intermediate_threshold"_key_hash;

// Speed and acceleration limits
constexpr uint32_t MIN_SPEED_LINEAR_KEY = "min_speed_linear"_key_hash;
constexpr uint32_t MAX_SPEED_LINEAR_KEY = "max_speed_linear"_key_hash;
constexpr uint32_t MAX_ACC_LINEAR_KEY = "max_acc_linear"_key_hash;
constexpr uint32_t MAX_DEC_LINEAR_KEY = "max_dec_linear"_key_hash;
constexpr uint32_t MIN_SPEED_ANGULAR_KEY = "min_speed_angular"_key_hash;
constexpr uint32_t MAX_SPEED_ANGULAR_KEY = "max_speed_angular"_key_hash;
constexpr uint32_t MAX_ACC_ANGULAR_KEY = "max_acc_angular"_key_hash;
constexpr uint32_t MAX_DEC_ANGULAR_KEY = "max_dec_angular"_key_hash;

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
inline cogip::parameter::Parameter<float, cogip::parameter::WithFlashStorage<LEFT_WHEEL_DIAMETER_KEY>> left_encoder_wheels_diameter_mm{default_left_encoder_wheels_diameter_mm};
inline cogip::parameter::Parameter<float, cogip::parameter::WithFlashStorage<RIGHT_WHEEL_DIAMETER_KEY>> right_encoder_wheels_diameter_mm{default_right_encoder_wheels_diameter_mm};
inline cogip::parameter::Parameter<float, cogip::parameter::WithFlashStorage<ENCODER_WHEELS_DISTANCE_KEY>> encoder_wheels_distance_mm{default_encoder_wheels_distance_mm};
inline cogip::parameter::Parameter<float, cogip::parameter::ReadOnly> encoder_wheels_resolution_pulses{default_encoder_wheels_resolution_pulses};

// OTOS localization calibration scalars
// Range [0.872, 1.127] is the valid domain of the OTOS linear/angular
// calibration scalars. WithBounds rejects out-of-range flash entries
// or bus writes instead of silently normalizing them, so calibration
// mistakes surface as an invalid parameter.
#ifdef ROBOT_HAS_OTOS
inline cogip::parameter::Parameter<float, cogip::parameter::WithBounds<0.872f, 1.127f>, cogip::parameter::WithFlashStorage<OTOS_LINEAR_SCALAR_KEY>> otos_linear_scalar{default_otos_linear_scalar};
inline cogip::parameter::Parameter<float, cogip::parameter::WithBounds<0.872f, 1.127f>, cogip::parameter::WithFlashStorage<OTOS_ANGULAR_SCALAR_KEY>> otos_angular_scalar{default_otos_angular_scalar};
#endif

// Linear pose PID (QUADPID chain)
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<LINEAR_POSE_PID_KP_KEY>> linear_pose_pid_kp{default_linear_pose_pid_kp};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<LINEAR_POSE_PID_KI_KEY>> linear_pose_pid_ki{default_linear_pose_pid_ki};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<LINEAR_POSE_PID_KD_KEY>> linear_pose_pid_kd{default_linear_pose_pid_kd};
// Angular pose PID (QUADPID chain)
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<ANGULAR_POSE_PID_KP_KEY>> angular_pose_pid_kp{default_angular_pose_pid_kp};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<ANGULAR_POSE_PID_KI_KEY>> angular_pose_pid_ki{default_angular_pose_pid_ki};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<ANGULAR_POSE_PID_KD_KEY>> angular_pose_pid_kd{default_angular_pose_pid_kd};
// Linear speed PID (QUADPID chain)
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<LINEAR_SPEED_PID_KP_KEY>> linear_speed_pid_kp{default_linear_speed_pid_kp};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<LINEAR_SPEED_PID_KI_KEY>> linear_speed_pid_ki{default_linear_speed_pid_ki};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<LINEAR_SPEED_PID_KD_KEY>> linear_speed_pid_kd{default_linear_speed_pid_kd};
// Angular speed PID (QUADPID chain)
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<ANGULAR_SPEED_PID_KP_KEY>> angular_speed_pid_kp{default_angular_speed_pid_kp};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<ANGULAR_SPEED_PID_KI_KEY>> angular_speed_pid_ki{default_angular_speed_pid_ki};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<ANGULAR_SPEED_PID_KD_KEY>> angular_speed_pid_kd{default_angular_speed_pid_kd};

// Tracker linear pose PID
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<TRACKER_LINEAR_POSE_PID_KP_KEY>> tracker_linear_pose_pid_kp{default_tracker_linear_pose_pid_kp};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<TRACKER_LINEAR_POSE_PID_KI_KEY>> tracker_linear_pose_pid_ki{default_tracker_linear_pose_pid_ki};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<TRACKER_LINEAR_POSE_PID_KD_KEY>> tracker_linear_pose_pid_kd{default_tracker_linear_pose_pid_kd};
// Tracker angular pose PID
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<TRACKER_ANGULAR_POSE_PID_KP_KEY>> tracker_angular_pose_pid_kp{default_tracker_angular_pose_pid_kp};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<TRACKER_ANGULAR_POSE_PID_KI_KEY>> tracker_angular_pose_pid_ki{default_tracker_angular_pose_pid_ki};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<TRACKER_ANGULAR_POSE_PID_KD_KEY>> tracker_angular_pose_pid_kd{default_tracker_angular_pose_pid_kd};
// Tracker linear speed PID
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<TRACKER_LINEAR_SPEED_PID_KP_KEY>> tracker_linear_speed_pid_kp{default_tracker_linear_speed_pid_kp};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<TRACKER_LINEAR_SPEED_PID_KI_KEY>> tracker_linear_speed_pid_ki{default_tracker_linear_speed_pid_ki};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<TRACKER_LINEAR_SPEED_PID_KD_KEY>> tracker_linear_speed_pid_kd{default_tracker_linear_speed_pid_kd};
// Tracker angular speed PID
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<TRACKER_ANGULAR_SPEED_PID_KP_KEY>> tracker_angular_speed_pid_kp{default_tracker_angular_speed_pid_kp};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<TRACKER_ANGULAR_SPEED_PID_KI_KEY>> tracker_angular_speed_pid_ki{default_tracker_angular_speed_pid_ki};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::WithFlashStorage<TRACKER_ANGULAR_SPEED_PID_KD_KEY>> tracker_angular_speed_pid_kd{default_tracker_angular_speed_pid_kd};

// PID integral limits
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> linear_pose_pid_integral_limit{default_linear_pose_pid_integral_limit};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> angular_pose_pid_integral_limit{default_angular_pose_pid_integral_limit};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> linear_speed_pid_integral_limit{default_linear_speed_pid_integral_limit};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> angular_speed_pid_integral_limit{default_angular_speed_pid_integral_limit};

// Tracker PID integral limits
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> tracker_linear_pose_pid_integral_limit{default_tracker_linear_pose_pid_integral_limit};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> tracker_angular_pose_pid_integral_limit{default_tracker_angular_pose_pid_integral_limit};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> tracker_linear_speed_pid_integral_limit{default_tracker_linear_speed_pid_integral_limit};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> tracker_angular_speed_pid_integral_limit{default_tracker_angular_speed_pid_integral_limit};

// Pose straight filter thresholds (absolute units: mm, deg)
inline cogip::parameter::Parameter<float, cogip::parameter::Clamp<1, 10>, cogip::parameter::WithFlashStorage<LINEAR_THRESHOLD_KEY>> param_linear_threshold{linear_threshold};
inline cogip::parameter::Parameter<float, cogip::parameter::Clamp<1, 5>, cogip::parameter::WithFlashStorage<ANGULAR_THRESHOLD_KEY>> param_angular_threshold{angular_threshold};
inline cogip::parameter::Parameter<float, cogip::parameter::Clamp<1, 5>, cogip::parameter::WithFlashStorage<ANGULAR_INTERMEDIATE_THRESHOLD_KEY>> param_angular_intermediate_threshold{angular_intermediate_threshold};

// Speed limits (internal: /period, protobuf: /s)
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::SpeedConversion<motion_control_thread_period_ms>> param_min_speed_linear{platform_min_speed_linear_mm_per_period};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::SpeedConversion<motion_control_thread_period_ms>> param_max_speed_linear{platform_max_speed_linear_mm_per_period};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::SpeedConversion<motion_control_thread_period_ms>> param_min_speed_angular{platform_min_speed_angular_deg_per_period};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::SpeedConversion<motion_control_thread_period_ms>> param_max_speed_angular{platform_max_speed_angular_deg_per_period};

// Acceleration/deceleration limits (internal: /period², protobuf: /s²)
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::AccelerationConversion<motion_control_thread_period_ms>> param_max_acc_linear{platform_max_acc_linear_mm_per_period2};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::AccelerationConversion<motion_control_thread_period_ms>> param_max_dec_linear{platform_max_dec_linear_mm_per_period2};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::AccelerationConversion<motion_control_thread_period_ms>> param_max_acc_angular{platform_max_acc_angular_deg_per_period2};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative, cogip::parameter::AccelerationConversion<motion_control_thread_period_ms>> param_max_dec_angular{platform_max_dec_angular_deg_per_period2};
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

} // namespace motion_control
} // namespace pf
} // namespace cogip
