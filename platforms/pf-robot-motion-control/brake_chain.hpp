// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief Brake chain controller instances for the platform engine
/// @details Minimal controller chain executed while the engine brake is
/// latched. Forces linear and angular speed orders to zero and runs the speed
/// loop, so process_outputs drives the motors toward zero speed via
/// closed-loop control. Used for controlled active stop, not for emergency
/// stop (EMS still disables motors).

#pragma once

#include "motion_control_common/MetaController.hpp"
#include "motion_control_parameters.hpp"
#include "pid/PID.hpp"
#include "polar_parallel_meta_controller/PolarParallelMetaController.hpp"
#include "reset_controller/ResetController.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "speed_pid_controller/SpeedPIDControllerIOKeysDefault.hpp"
#include "speed_pid_controller/SpeedPIDControllerParameters.hpp"

namespace cogip {
namespace pf {
namespace motion_control {

namespace brake_chain {

// Dedicated PIDParameters for the brake speed loop. Gains live in their own
// Parameter<float> instances (declared in motion_control_parameters.hpp) so
// the brake can be tuned independently from the tracking PIDs; same for the
// PID state (integrator, previous error) thanks to the dedicated PID
// instances below. The brake chain typically wants higher Ki than the
// tracking loop to absorb static disturbances on hold (slope, friction,
// push from another robot) which the cascaded Ki=0 pose loop cannot.
inline cogip::pid::PIDParameters
    brake_linear_speed_pid_parameters(brake_linear_speed_pid_kp, brake_linear_speed_pid_ki,
                                      brake_linear_speed_pid_kd,
                                      brake_linear_speed_pid_integral_limit);
inline cogip::pid::PIDParameters
    brake_angular_speed_pid_parameters(brake_angular_speed_pid_kp, brake_angular_speed_pid_ki,
                                       brake_angular_speed_pid_kd,
                                       brake_angular_speed_pid_integral_limit);

// Dedicated PID instances. Their integrator / previous-error state stays
// isolated from the tracking PIDs so latching the brake does not pollute
// (or get polluted by) the running chain.
inline cogip::pid::PID brake_linear_speed_pid(brake_linear_speed_pid_parameters);
inline cogip::pid::PID brake_angular_speed_pid(brake_angular_speed_pid_parameters);

inline cogip::motion_control::SpeedPIDControllerParameters
    brake_linear_speed_controller_parameters(&brake_linear_speed_pid);
inline cogip::motion_control::SpeedPIDControllerParameters
    brake_angular_speed_controller_parameters(&brake_angular_speed_pid);

// Dedicated SpeedPIDController instances. IO keys are identical to the
// normal chain because only one chain runs per cycle; the ownership rule
// (a controller can only belong to one meta-controller) forces duplication.
inline cogip::motion_control::SpeedPIDController brake_linear_speed_controller(
    cogip::motion_control::linear_speed_pid_controller_io_keys_default,
    brake_linear_speed_controller_parameters);
inline cogip::motion_control::SpeedPIDController brake_angular_speed_controller(
    cogip::motion_control::angular_speed_pid_controller_io_keys_default,
    brake_angular_speed_controller_parameters);

// Parallel execution of linear and angular speed loops.
inline cogip::motion_control::PolarParallelMetaController brake_speed_loop_polar_parallel;

// Forces both speed orders to zero at the head of the brake chain.
inline cogip::motion_control::ResetController brake_zero_speed_order_controller(
    {{"linear_speed_order", 0.0f}, {"angular_speed_order", 0.0f}});

// Top-level brake chain: zero speed orders, then run the speed loop.
inline cogip::motion_control::MetaController<> brake_meta_controller;

/// Wire the brake chain sub-controllers and register it on the platform
/// engine as its brake controller.
void init();

} // namespace brake_chain

} // namespace motion_control
} // namespace pf
} // namespace cogip
