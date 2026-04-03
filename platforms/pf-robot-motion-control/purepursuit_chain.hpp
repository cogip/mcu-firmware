// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief
/// @details
#pragma once

#include "motion_control.hpp"
#include "motion_control_common/MetaController.hpp"
#include "parameter/Parameter.hpp"
#include "polar_parallel_meta_controller/PolarParallelMetaController.hpp"
#include "profile_tracker_controller/ProfileTrackerController.hpp"
#include "purepursuit_controller/PurePursuitController.hpp"
#include "purepursuit_controller/PurePursuitControllerIOKeys.hpp"
#include "purepursuit_controller/PurePursuitControllerParameters.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "speed_pid_controller/SpeedPIDControllerIOKeys.hpp"
#include "speed_ramp_filter/SpeedRampFilter.hpp"
#include "speed_ramp_filter/SpeedRampFilterIOKeys.hpp"
#include "speed_ramp_filter/SpeedRampFilterParameters.hpp"

namespace cogip {
namespace pf {
namespace motion_control {
namespace purepursuit_chain {

// ============================================================================
// LINEAR PID
// ============================================================================
inline cogip::pid::PIDParameters
    linear_speed_pid_parameters(tracker_linear_speed_pid_kp, tracker_linear_speed_pid_ki,
                                tracker_linear_speed_pid_kd,
                                tracker_linear_speed_pid_integral_limit);

inline cogip::pid::PID linear_speed_pid(linear_speed_pid_parameters);

// ============================================================================
// ANGULAR PID
// ============================================================================
inline cogip::pid::PIDParameters
    angular_speed_pid_parameters(tracker_angular_speed_pid_kp, tracker_angular_speed_pid_ki,
                                 tracker_angular_speed_pid_kd,
                                 tracker_angular_speed_pid_integral_limit);

inline cogip::pid::PID angular_speed_pid(angular_speed_pid_parameters);

// ============================================================================
// Linear Speed Controller
// ============================================================================
inline cogip::motion_control::SpeedPIDControllerIOKeys linear_speed_pid_io_keys = {
    .speed_order = "linear_speed_order",
    .current_speed = "linear_current_speed",
    .speed_command = "linear_speed_command",
    .reset = "linear_pid_reset",
};

inline cogip::motion_control::SpeedPIDControllerParameters
    linear_speed_controller_parameters(&linear_speed_pid);

inline cogip::motion_control::SpeedPIDController
    linear_speed_controller(linear_speed_pid_io_keys, linear_speed_controller_parameters);

// ============================================================================
// Angular Speed Controller
// ============================================================================
inline cogip::motion_control::SpeedPIDControllerIOKeys angular_speed_pid_io_keys = {
    .speed_order = "angular_speed_order",
    .current_speed = "angular_current_speed",
    .speed_command = "angular_speed_command",
    .reset = "angular_pid_reset",
};

inline cogip::motion_control::SpeedPIDControllerParameters
    angular_speed_controller_parameters(&angular_speed_pid);

inline cogip::motion_control::SpeedPIDController
    angular_speed_controller(angular_speed_pid_io_keys, angular_speed_controller_parameters);

// ============================================================================
// Speed Ramp Filters (acceleration + distance-based braking)
// ============================================================================

inline cogip::motion_control::SpeedRampFilterIOKeys linear_speed_ramp_io_keys = {
    .target_speed = "linear_speed_order",
    .current_speed = "linear_current_speed",
    .pose_error = "linear_pose_error",
    .reset = "linear_ramp_reset",
};

inline cogip::motion_control::SpeedRampFilterParameters
    linear_speed_ramp_parameters(platform_max_acc_linear_mm_per_period2,
                                 platform_max_dec_linear_mm_per_period2);

inline cogip::motion_control::SpeedRampFilter
    linear_speed_ramp_filter(linear_speed_ramp_io_keys, linear_speed_ramp_parameters);

inline cogip::motion_control::SpeedRampFilterIOKeys angular_speed_ramp_io_keys = {
    .target_speed = "angular_speed_order",
    .current_speed = "angular_current_speed",
    .pose_error = "angular_pose_error",
    .reset = "angular_ramp_reset",
};

inline cogip::motion_control::SpeedRampFilterParameters
    angular_speed_ramp_parameters(platform_max_acc_angular_deg_per_period2,
                                  platform_max_dec_angular_deg_per_period2);

inline cogip::motion_control::SpeedRampFilter
    angular_speed_ramp_filter(angular_speed_ramp_io_keys, angular_speed_ramp_parameters);

// ============================================================================
// Purepursuit Controller
// ============================================================================
inline cogip::motion_control::PurePursuitControllerIOKeys purepursuit_io_keys = {
    .current_pose_x = "current_pose_x",
    .current_pose_y = "current_pose_y",
    .current_pose_O = "current_pose_O",
    .linear_speed_order = "linear_speed_order",
    .angular_speed_order = "angular_speed_order",
    .linear_pose_error = "linear_pose_error",
    .angular_pose_error = "angular_pose_error",
    .pose_reached = "pose_reached",
    .linear_pid_reset = "linear_pid_reset",
    .angular_pid_reset = "angular_pid_reset",
    .linear_ramp_reset = "linear_ramp_reset",
    .angular_ramp_reset = "angular_ramp_reset",
};

inline Parameter<float, NonNegative> purepursuit_linear_kp{2.0f};
inline Parameter<float, NonNegative> purepursuit_angular_kp{2.0f};
inline Parameter<float, NonNegative> purepursuit_lookahead_distance{150};
inline Parameter<float, NonNegative> purepursuit_final_lookahead_distance{200};
inline Parameter<float, NonNegative> purepursuit_max_linear_speed{
    platform_max_speed_linear_mm_per_period};
inline Parameter<float, NonNegative> purepursuit_max_angular_speed{
    platform_max_speed_angular_deg_per_period};
inline Parameter<float, NonNegative> purepursuit_linear_threshold{linear_threshold};
inline Parameter<float, NonNegative> purepursuit_angular_threshold{angular_threshold};

inline cogip::motion_control::PurePursuitControllerParameters
    purepursuit_parameters(purepursuit_linear_kp, purepursuit_angular_kp,
                           purepursuit_lookahead_distance, purepursuit_final_lookahead_distance,
                           purepursuit_max_linear_speed, purepursuit_max_angular_speed,
                           purepursuit_linear_threshold, purepursuit_angular_threshold);

inline cogip::motion_control::PurePursuitController
    purepursuit_controller(purepursuit_io_keys, purepursuit_parameters, motion_control_path);

// ============================================================================
// Meta controllers
// ============================================================================

inline cogip::motion_control::MetaController<2> linear_meta_controller;
inline cogip::motion_control::MetaController<2> angular_meta_controller;
inline cogip::motion_control::PolarParallelMetaController polar_parallel_meta_controller;
inline cogip::motion_control::MetaController<> pose_loop_meta_controller;

// ============================================================================
// Chain initialization function
// ============================================================================

/// Initialize tracker speed tuning chain meta controller
cogip::motion_control::MetaController<>* init();

} // namespace purepursuit_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
