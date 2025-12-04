// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief QuadPID chain controller instances
/// @details Controllers specific to the QuadPID chain (historical cascaded PID control).

#pragma once

#include "app_conf.hpp"
#include "motion_control_common/MetaController.hpp"
#include "motion_control_common/ThrottledController.hpp"
#include "passthrough_pose_pid_controller/PassthroughPosePIDController.hpp"
#include "passthrough_pose_pid_controller/PassthroughPosePIDControllerIOKeysDefault.hpp"
#include "passthrough_pose_pid_controller/PassthroughPosePIDControllerParameters.hpp"
#include "pid/PID.hpp"
#include "polar_parallel_meta_controller/PolarParallelMetaController.hpp"
#include "pose_pid_controller/PosePIDController.hpp"
#include "pose_pid_controller/PosePIDControllerIOKeysDefault.hpp"
#include "pose_pid_controller/PosePIDControllerParameters.hpp"
#include "pose_straight_filter/PoseStraightFilter.hpp"
#include "pose_straight_filter/PoseStraightFilterIOKeysDefault.hpp"
#include "pose_straight_filter/PoseStraightFilterParameters.hpp"
#include "quadpid_meta_controller/QuadPIDMetaController.hpp"
#include "speed_filter/SpeedFilter.hpp"
#include "speed_filter/SpeedFilterIOKeysDefault.hpp"
#include "speed_filter/SpeedFilterParameters.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "speed_pid_controller/SpeedPIDControllerIOKeysDefault.hpp"
#include "speed_pid_controller/SpeedPIDControllerParameters.hpp"

namespace cogip {
namespace pf {
namespace motion_control {

// PIDs are declared extern here and defined in motion_control.cpp
extern cogip::pid::PID linear_pose_pid;
extern cogip::pid::PID linear_speed_pid;
extern cogip::pid::PID angular_pose_pid;
extern cogip::pid::PID angular_speed_pid;

namespace quadpid_chain {

// ============================================================================
// PoseStraightFilter
// ============================================================================

inline cogip::motion_control::PoseStraightFilterParameters pose_straight_filter_parameters(
    angular_threshold, linear_threshold, angular_intermediate_threshold,
    platform_max_dec_angular_deg_per_period2, platform_max_dec_linear_mm_per_period2);

inline cogip::motion_control::PoseStraightFilter
    pose_straight_filter(cogip::motion_control::pose_straight_filter_io_keys_default,
                         pose_straight_filter_parameters);

// ============================================================================
// MetaControllers
// ============================================================================

inline cogip::motion_control::MetaController<2> pose_loop_meta_controller;
inline cogip::motion_control::PolarParallelMetaController pose_loop_polar_parallel_meta_controller;
inline cogip::motion_control::PolarParallelMetaController speed_loop_polar_parallel_meta_controller;

// ============================================================================
// Linear chain
// ============================================================================

inline cogip::motion_control::MetaController<1> linear_pose_loop_meta_controller;
inline cogip::motion_control::MetaController<2> linear_speed_loop_meta_controller;

inline cogip::motion_control::PosePIDControllerParameters
    linear_pose_controller_parameters(&cogip::pf::motion_control::linear_pose_pid);

inline cogip::motion_control::PosePIDController
    linear_pose_controller(cogip::motion_control::linear_pose_pid_controller_io_keys_default,
                           linear_pose_controller_parameters);

inline cogip::motion_control::SpeedFilterParameters
    linear_speed_filter_parameters(platform_min_speed_linear_mm_per_period,
                                   platform_max_speed_linear_mm_per_period,
                                   platform_max_acc_linear_mm_per_period2);

inline cogip::motion_control::SpeedFilter
    linear_speed_filter(cogip::motion_control::linear_speed_filter_io_keys_default,
                        linear_speed_filter_parameters);

inline cogip::motion_control::SpeedPIDControllerParameters
    linear_speed_controller_parameters(&cogip::pf::motion_control::linear_speed_pid);

inline cogip::motion_control::SpeedPIDController
    linear_speed_controller(cogip::motion_control::linear_speed_pid_controller_io_keys_default,
                            linear_speed_controller_parameters);

// ============================================================================
// Angular chain
// ============================================================================

inline cogip::motion_control::MetaController<1> angular_pose_loop_meta_controller;
inline cogip::motion_control::MetaController<2> angular_speed_loop_meta_controller;

inline cogip::motion_control::PosePIDControllerParameters
    angular_pose_controller_parameters(&cogip::pf::motion_control::angular_pose_pid);

inline cogip::motion_control::PosePIDController
    angular_pose_controller(cogip::motion_control::angular_pose_pid_controller_io_keys_default,
                            angular_pose_controller_parameters);

inline cogip::motion_control::SpeedFilterParameters
    angular_speed_filter_parameters(platform_min_speed_angular_deg_per_period,
                                    platform_max_speed_angular_deg_per_period,
                                    platform_max_acc_angular_deg_per_period2);

inline cogip::motion_control::SpeedFilter
    angular_speed_filter(cogip::motion_control::angular_speed_filter_io_keys_default,
                         angular_speed_filter_parameters);

inline cogip::motion_control::SpeedPIDControllerParameters
    angular_speed_controller_parameters(&cogip::pf::motion_control::angular_speed_pid);

inline cogip::motion_control::SpeedPIDController
    angular_speed_controller(cogip::motion_control::angular_speed_pid_controller_io_keys_default,
                             angular_speed_controller_parameters);

// ============================================================================
// Throttled controller
// ============================================================================

inline cogip::motion_control::ThrottledController
    throttled_pose_loop_controllers(&pose_loop_meta_controller, pose_controllers_throttle_divider);

// ============================================================================
// Passthrough controllers (for test modes)
// ============================================================================

inline cogip::motion_control::PassthroughPosePIDControllerParameters
    passthrough_linear_pose_controller_parameters(platform_max_speed_linear_mm_per_period, true);

inline cogip::motion_control::PassthroughPosePIDController passthrough_linear_pose_controller(
    cogip::motion_control::linear_passthrough_pose_pid_controller_io_keys_default,
    passthrough_linear_pose_controller_parameters);

inline cogip::motion_control::PassthroughPosePIDControllerParameters
    passthrough_angular_pose_controller_parameters(platform_max_speed_angular_deg_per_period, true);

inline cogip::motion_control::PassthroughPosePIDController passthrough_angular_pose_controller(
    cogip::motion_control::angular_passthrough_pose_pid_controller_io_keys_default,
    passthrough_angular_pose_controller_parameters);

// ============================================================================
// QuadPID meta controller
// ============================================================================

inline cogip::motion_control::QuadPIDMetaController quadpid_meta_controller;

} // namespace quadpid_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
