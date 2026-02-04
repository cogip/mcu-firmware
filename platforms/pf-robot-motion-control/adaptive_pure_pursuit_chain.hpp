// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief Adaptive Pure Pursuit chain controller instances
/// @details Controllers specific to the Adaptive Pure Pursuit chain.

#pragma once

#include "app_conf.hpp"
#include "motion_control.hpp"

#include "acceleration_filter/AccelerationFilter.hpp"
#include "acceleration_filter/AccelerationFilterIOKeys.hpp"
#include "acceleration_filter/AccelerationFilterParameters.hpp"
#include "adaptive_pure_pursuit_controller/AdaptivePurePursuitController.hpp"
#include "adaptive_pure_pursuit_controller/AdaptivePurePursuitControllerIOKeys.hpp"
#include "adaptive_pure_pursuit_controller/AdaptivePurePursuitControllerParameters.hpp"
#include "anti_blocking_controller/AntiBlockingController.hpp"
#include "anti_blocking_controller/AntiBlockingControllerParameters.hpp"
#include "conditional_switch_meta_controller/ConditionalSwitchMetaController.hpp"
#include "motion_control_common/MetaController.hpp"
#include "motion_control_common/NoOpController.hpp"
#include "pid/PID.hpp"
#include "polar_parallel_meta_controller/PolarParallelMetaController.hpp"
#include "pose_pid_controller/PosePIDController.hpp"
#include "pose_pid_controller/PosePIDControllerIOKeys.hpp"
#include "pose_pid_controller/PosePIDControllerParameters.hpp"
#include "profile_tracker_controller/ProfileTrackerController.hpp"
#include "profile_tracker_controller/ProfileTrackerControllerIOKeys.hpp"
#include "profile_tracker_controller/ProfileTrackerControllerParameters.hpp"
#include "quadpid_meta_controller/QuadPIDMetaController.hpp"
#include "speed_limit_filter/SpeedLimitFilter.hpp"
#include "speed_limit_filter/SpeedLimitFilterIOKeys.hpp"
#include "speed_limit_filter/SpeedLimitFilterParameters.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "speed_pid_controller/SpeedPIDControllerIOKeysDefault.hpp"
#include "speed_pid_controller/SpeedPIDControllerParameters.hpp"
#include "telemetry_controller/TelemetryController.hpp"
#include "telemetry_controller/TelemetryControllerIOKeysDefault.hpp"
#include "telemetry_controller/TelemetryControllerParameters.hpp"
#include "tracker_combiner_controller/TrackerCombinerController.hpp"
#include "tracker_combiner_controller/TrackerCombinerControllerIOKeys.hpp"
#include "tracker_combiner_controller/TrackerCombinerControllerParameters.hpp"

namespace cogip {
namespace pf {
namespace motion_control {
namespace adaptive_pure_pursuit_chain {

// ============================================================================
// Local PIDs (independent from other chains)
// ============================================================================

inline cogip::pid::PIDParameters
    linear_speed_pid_parameters(tracker_linear_speed_pid_kp, tracker_linear_speed_pid_ki,
                                tracker_linear_speed_pid_kd,
                                tracker_linear_speed_pid_integral_limit);

inline cogip::pid::PIDParameters
    angular_speed_pid_parameters(tracker_angular_speed_pid_kp, tracker_angular_speed_pid_ki,
                                 tracker_angular_speed_pid_kd,
                                 tracker_angular_speed_pid_integral_limit);

inline cogip::pid::PID linear_speed_pid(linear_speed_pid_parameters);
inline cogip::pid::PID angular_speed_pid(angular_speed_pid_parameters);

// ============================================================================
// AdaptivePurePursuitController
// ============================================================================

inline constexpr cogip::motion_control::AdaptivePurePursuitControllerIOKeys pure_pursuit_io_keys = {
    // Input keys
    .current_pose_x = "current_pose_x",
    .current_pose_y = "current_pose_y",
    .current_pose_O = "current_pose_O",
    .linear_current_speed = "linear_current_speed",
    .angular_current_speed = "angular_current_speed",
    .motion_direction = "motion_direction",
    .bypass_final_orientation = "bypass_final_orientation",
    // Output keys
    .linear_speed_order = "linear_speed_order",
    .angular_speed_order = "angular_speed_order",
    .pose_reached = "pose_reached",
    .path_complete = "path_complete",
    .is_intermediate = "is_intermediate",
    // Output keys for ROTATING_TO_FINAL (pose loop)
    .angular_pose_error = "angular_pose_error",
    .recompute_angular_profile = "recompute_angular_profile",
    .rotating_in_place = "rotating_in_place"};

inline cogip::motion_control::AdaptivePurePursuitControllerParameters pure_pursuit_parameters(
    platform_pure_pursuit_min_lookahead_mm,      // min_lookahead_distance
    platform_pure_pursuit_max_lookahead_mm,      // max_lookahead_distance
    platform_pure_pursuit_lookahead_speed_ratio, // lookahead_speed_ratio
    platform_max_speed_linear_mm_per_period,     // max_linear_speed
    platform_max_speed_angular_deg_per_period,   // max_angular_speed
    linear_threshold,                            // linear_threshold (same as PoseStraightFilter)
    angular_threshold,                           // angular_threshold (same as PoseStraightFilter)
    pure_pursuit_initial_rotation_threshold_deg, // initial_rotation_threshold
    platform_max_acc_linear_mm_per_period2,      // linear_acceleration
    platform_max_dec_linear_mm_per_period2,      // linear_deceleration
    platform_max_dec_angular_deg_per_period2     // angular_deceleration
);

inline cogip::motion_control::AdaptivePurePursuitController
    pure_pursuit_controller(pure_pursuit_io_keys, pure_pursuit_parameters);

// ============================================================================
// SpeedPIDController instances
// ============================================================================

inline cogip::motion_control::SpeedPIDControllerParameters
    linear_speed_controller_parameters(&linear_speed_pid);

inline cogip::motion_control::SpeedPIDController
    linear_speed_controller(cogip::motion_control::linear_speed_pid_controller_io_keys_default,
                            linear_speed_controller_parameters);

inline cogip::motion_control::SpeedPIDControllerParameters
    angular_speed_controller_parameters(&angular_speed_pid);

inline cogip::motion_control::SpeedPIDController
    angular_speed_controller(cogip::motion_control::angular_speed_pid_controller_io_keys_default,
                             angular_speed_controller_parameters);

// ============================================================================
// Angular pose loop for ROTATING_TO_FINAL state
// (ProfileTracker → PosePID → Combiner)
// ============================================================================

// Angular pose PID (for tracking error correction during rotation)
inline cogip::pid::PIDParameters
    angular_pose_pid_parameters(tracker_angular_pose_pid_kp, tracker_angular_pose_pid_ki,
                                tracker_angular_pose_pid_kd,
                                tracker_angular_pose_pid_integral_limit);

inline cogip::pid::PID angular_pose_pid(angular_pose_pid_parameters);

// Angular ProfileTrackerController IO keys
inline cogip::motion_control::ProfileTrackerControllerIOKeys angular_profile_tracker_io_keys = {
    .pose_error = "angular_pose_error",
    .current_speed = "angular_current_speed",
    .recompute_profile = "recompute_angular_profile",
    .tracker_velocity = "angular_tracker_velocity",
    .tracking_error = "angular_tracking_error",
    .profile_complete = ""}; // Not used

// Angular ProfileTrackerController parameters
inline cogip::motion_control::ProfileTrackerControllerParameters
    angular_profile_tracker_parameters(platform_max_speed_angular_deg_per_period, // max_speed
                                       platform_max_acc_angular_deg_per_period2,  // acceleration
                                       platform_max_dec_angular_deg_per_period2,  // deceleration
                                       true, // must_stop_at_end
                                       1     // period_increment
    );

// Angular ProfileTrackerController
inline cogip::motion_control::ProfileTrackerController
    angular_profile_tracker_controller(angular_profile_tracker_io_keys,
                                       angular_profile_tracker_parameters);

// Angular PosePIDController IO keys (tracking error correction)
inline constexpr cogip::motion_control::PosePIDControllerIOKeys
    angular_pose_pid_controller_io_keys = {.position_error = "angular_tracking_error",
                                           .current_speed = "angular_current_speed",
                                           .target_speed = "dummy_target_speed",
                                           .disable_filter = "dummy_disable",
                                           .pose_reached = "dummy_pose_reached",
                                           .speed_order = "angular_feedback_correction"};

inline cogip::motion_control::PosePIDControllerParameters angular_pose_pid_controller_parameters{
    &angular_pose_pid};

inline cogip::motion_control::PosePIDController
    angular_pose_pid_controller(angular_pose_pid_controller_io_keys,
                                angular_pose_pid_controller_parameters);

// Angular TrackerCombinerController IO keys
inline cogip::motion_control::TrackerCombinerControllerIOKeys angular_tracker_combiner_io_keys = {
    .tracker_velocity = "angular_tracker_velocity",
    .feedback_correction = "angular_feedback_correction",
    .speed_order = "angular_speed_order",
    .speed_command = ""}; // Speed PID will handle this

// Angular TrackerCombinerController parameters
inline cogip::motion_control::TrackerCombinerControllerParameters
    angular_tracker_combiner_parameters;

// Angular TrackerCombinerController
inline cogip::motion_control::TrackerCombinerController
    angular_tracker_combiner_controller(angular_tracker_combiner_io_keys,
                                        angular_tracker_combiner_parameters);

// Angular pose loop meta controller (ProfileTracker → PosePID → Combiner)
inline cogip::motion_control::MetaController<> angular_pose_loop_meta_controller;

// No-op controller for when rotating_in_place is false (FOLLOWING_PATH state)
inline cogip::motion_control::NoOpController noop_controller;

// ConditionalSwitch: executes angular pose loop only when rotating_in_place is true
inline cogip::motion_control::ConditionalSwitchMetaController
    rotating_in_place_conditional_switch("rotating_in_place", &angular_pose_loop_meta_controller,
                                         &noop_controller);

// ============================================================================
// Anti-blocking controllers
// ============================================================================

inline cogip::motion_control::AntiBlockingControllerIOKeys linear_anti_blocking_io_keys = {
    .speed_order = "linear_speed_order",
    .current_speed = "linear_current_speed",
    .speed_error = "linear_speed_error",
    .pose_reached = "pose_reached"};

inline cogip::motion_control::AntiBlockingControllerParameters
    linear_anti_blocking_parameters(true, // enabled
                                    platform_linear_anti_blocking_speed_threshold_mm_per_period,
                                    platform_linear_anti_blocking_error_threshold_mm_per_period,
                                    platform_linear_anti_blocking_blocked_cycles_nb_threshold);

inline cogip::motion_control::AntiBlockingController
    linear_anti_blocking_controller(linear_anti_blocking_io_keys, linear_anti_blocking_parameters);

inline cogip::motion_control::AntiBlockingControllerIOKeys angular_anti_blocking_io_keys = {
    .speed_order = "angular_speed_order",
    .current_speed = "angular_current_speed",
    .speed_error = "angular_speed_error",
    .pose_reached = "pose_reached"};

inline cogip::motion_control::AntiBlockingControllerParameters
    angular_anti_blocking_parameters(false, // disabled by default
                                     platform_linear_anti_blocking_speed_threshold_mm_per_period,
                                     platform_linear_anti_blocking_error_threshold_mm_per_period,
                                     platform_linear_anti_blocking_blocked_cycles_nb_threshold);

inline cogip::motion_control::AntiBlockingController
    angular_anti_blocking_controller(angular_anti_blocking_io_keys,
                                     angular_anti_blocking_parameters);

// ============================================================================
// Speed limit filters (safety clamp at ratio × max)
// ============================================================================

inline cogip::motion_control::SpeedLimitFilterIOKeys linear_speed_limit_io_keys = {
    .target_speed = "linear_speed_order", .output_speed = ""};

inline cogip::motion_control::SpeedLimitFilterParameters
linear_speed_limit_parameters(platform_min_speed_linear_mm_per_period,
                              platform_max_speed_linear_mm_per_period* speed_clamp_ratio);

inline cogip::motion_control::SpeedLimitFilter
    linear_speed_limit_filter(linear_speed_limit_io_keys, linear_speed_limit_parameters);

inline cogip::motion_control::SpeedLimitFilterIOKeys angular_speed_limit_io_keys = {
    .target_speed = "angular_speed_order", .output_speed = ""};

inline cogip::motion_control::SpeedLimitFilterParameters
angular_speed_limit_parameters(platform_min_speed_angular_deg_per_period,
                               platform_max_speed_angular_deg_per_period* speed_clamp_ratio);

inline cogip::motion_control::SpeedLimitFilter
    angular_speed_limit_filter(angular_speed_limit_io_keys, angular_speed_limit_parameters);

// ============================================================================
// Acceleration filters (safety clamp at ratio × max)
// ============================================================================

inline cogip::motion_control::AccelerationFilterIOKeys linear_acceleration_io_keys = {
    .target_speed = "linear_speed_order"};

inline cogip::motion_control::AccelerationFilterParameters
linear_acceleration_parameters(platform_max_acc_linear_mm_per_period2* acceleration_clamp_ratio,
                               platform_min_speed_linear_mm_per_period);

inline cogip::motion_control::AccelerationFilter
    linear_acceleration_filter(linear_acceleration_io_keys, linear_acceleration_parameters);

inline cogip::motion_control::AccelerationFilterIOKeys angular_acceleration_io_keys = {
    .target_speed = "angular_speed_order"};

inline cogip::motion_control::AccelerationFilterParameters
angular_acceleration_parameters(platform_max_acc_angular_deg_per_period2* acceleration_clamp_ratio,
                                platform_min_speed_angular_deg_per_period);

inline cogip::motion_control::AccelerationFilter
    angular_acceleration_filter(angular_acceleration_io_keys, angular_acceleration_parameters);

// ============================================================================
// Telemetry controller
// ============================================================================

inline cogip::motion_control::TelemetryControllerParameters telemetry_controller_parameters;

inline cogip::motion_control::TelemetryController
    telemetry_controller(cogip::motion_control::linear_telemetry_controller_io_keys_default,
                         telemetry_controller_parameters);

// ============================================================================
// MetaControllers
// ============================================================================

// Speed loop: linear + angular in parallel
inline cogip::motion_control::MetaController<> linear_speed_loop_meta_controller;
inline cogip::motion_control::MetaController<> angular_speed_loop_meta_controller;
inline cogip::motion_control::PolarParallelMetaController speed_loop_polar_parallel_meta_controller;

// Anti-blocking loop: linear + angular in parallel
inline cogip::motion_control::PolarParallelMetaController
    anti_blocking_polar_parallel_meta_controller;

// Main meta controller
inline cogip::motion_control::QuadPIDMetaController meta_controller;

// ============================================================================
// Chain initialization function
// ============================================================================

/// Initialize adaptive pure pursuit chain meta controller
cogip::motion_control::QuadPIDMetaController* init();

// ============================================================================
// Chain reset function
// ============================================================================

/// Reset adaptive pure pursuit chain state (all controllers via cascade)
inline void reset()
{
    meta_controller.reset();
}

} // namespace adaptive_pure_pursuit_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
