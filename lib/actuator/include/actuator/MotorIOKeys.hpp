// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     actuator
/// @{
/// @file
/// @brief       Aligned IO keys for Motor actuator controller chains
/// @details     These keys ensure proper data flow between controllers.
///
///              Classic DualPID chain:
///              MotorEngine → MotorPoseFilter → PosePIDController → SpeedFilter →
///              SpeedPIDController
///
///              Tracker chain (feedforward + feedback):
///              MotorEngine → MotorPoseFilter → ProfileTracker → PosePID → Combiner → SpeedPID
///
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include "motor_pose_filter/MotorPoseFilterIOKeys.hpp"
#include "pose_pid_controller/PosePIDControllerIOKeys.hpp"
#include "profile_tracker_controller/ProfileTrackerControllerIOKeys.hpp"
#include "speed_filter/SpeedFilterIOKeys.hpp"
#include "speed_pid_controller/SpeedPIDControllerIOKeys.hpp"
#include "tracker_combiner_controller/TrackerCombinerControllerIOKeys.hpp"

namespace cogip {
namespace actuators {

/// @brief IO keys for MotorPoseFilter aligned with MotorEngine and PosePIDController
/// @details
///   - Reads: current_pose, target_pose, current_speed, target_speed, pose_reached (from
///   MotorEngine)
///   - Writes: pose_error (for PosePIDController), target_speed (filtered), bypass_filter,
///   pose_reached
static const motion_control::MotorPoseFilterIOKeys motor_pose_filter_io_keys = {
    .current_pose = "current_pose",
    .target_pose = "target_pose",
    .current_speed = "current_speed",
    .target_speed = "target_speed",
    .pose_reached = "pose_reached",

    .position_error = "pose_error",   ///< Aligned with PosePIDController input
    .filtered_speed = "target_speed", ///< Write filtered speed back to target_speed for SpeedFilter
    .speed_filter_flag = "bypass_filter", ///< Aligned with SpeedFilter bypass flag
    .pose_reached_out = "pose_reached"    ///< Write back to same key for MotorEngine
};

/// @brief IO keys for PosePIDController aligned with MotorPoseFilter and SpeedFilter
/// @details
///   - Reads: pose_error (from MotorPoseFilter)
///   - Writes: speed_order (for SpeedFilter)
static const motion_control::PosePIDControllerIOKeys motor_pose_pid_io_keys = {
    .position_error = "pose_error", ///< Aligned with MotorPoseFilter output
    .current_speed = "current_speed",
    .target_speed = "target_speed",
    .disable_filter = "bypass_filter",
    .pose_reached = "pose_reached",
    .speed_order = "speed_order",
    .reset = "" ///< No reset key for motor pose PID
};

/// @brief IO keys for SpeedFilter aligned with PosePIDController and SpeedPIDController
/// @details
///   - Reads: speed_order (from PosePIDController), current_speed, target_speed
///   - Writes: speed_order (filtered), speed_error
static const motion_control::SpeedFilterIOKeys motor_speed_filter_io_keys = {
    .speed_order = "speed_order", ///< Aligned with PosePIDController output
    .current_speed = "current_speed",
    .target_speed = "target_speed",
    .speed_error = "speed_error",
    .bypass_filter = "bypass_filter" ///< Aligned with MotorPoseFilter output
};

/// @brief IO keys for SpeedPIDController aligned with SpeedFilter and MotorEngine
/// @details
///   - Reads: speed_order (from SpeedFilter), current_speed
///   - Writes: speed_command (for MotorEngine)
static const motion_control::SpeedPIDControllerIOKeys motor_speed_pid_io_keys = {
    .speed_order = "speed_order", ///< Aligned with SpeedFilter output
    .current_speed = "current_speed",
    .speed_command = "speed_command", ///< Aligned with MotorEngine input
    .reset = ""                       ///< No reset key for motor speed PID
};

// ============================================================================
// Tracker Chain IO Keys (feedforward + feedback)
// ============================================================================

/// @brief IO keys for MotorPoseFilter in tracker chain
/// @details Same as classic chain, but outputs new_target flag for profile recomputation
static const motion_control::MotorPoseFilterIOKeys motor_tracker_pose_filter_io_keys = {
    .current_pose = "current_pose",
    .target_pose = "target_pose",
    .current_speed = "current_speed",
    .target_speed = "target_speed",
    .pose_reached = "pose_reached",

    .position_error = "pose_error",
    .filtered_speed = "target_speed",
    .speed_filter_flag = "bypass_filter",
    .pose_reached_out = "pose_reached"};

/// @brief IO keys for ProfileTrackerController in motor tracker chain
/// @details
///   - Reads: pose_error (from MotorPoseFilter), current_speed
///   - Writes: tracker_velocity, tracking_error
static const motion_control::ProfileTrackerControllerIOKeys motor_profile_tracker_io_keys = {
    .pose_error = "pose_error",
    .current_speed = "speed_order",    ///< Use speed_order for profile continuity
    .recompute_profile = "new_target", ///< Flag to regenerate profile
    .tracker_velocity = "tracker_velocity",
    .tracking_error = "tracking_error",
    .profile_complete = "",
    .target_speed = "target_speed"};

/// @brief IO keys for PosePIDController in tracker chain (tracking error correction)
/// @details
///   - Reads: tracking_error (from ProfileTrackerController)
///   - Writes: feedback_correction
static const motion_control::PosePIDControllerIOKeys motor_tracker_pose_pid_io_keys = {
    .position_error = "tracking_error",
    .current_speed = "current_speed",
    .target_speed = "target_speed",
    .disable_filter = "",
    .pose_reached = "",
    .speed_order = "feedback_correction",
    .reset = ""};

/// @brief IO keys for TrackerCombinerController in motor tracker chain
/// @details
///   - Reads: tracker_velocity, feedback_correction
///   - Writes: speed_order
static const motion_control::TrackerCombinerControllerIOKeys motor_tracker_combiner_io_keys = {
    .tracker_velocity = "tracker_velocity",
    .feedback_correction = "feedback_correction",
    .speed_order = "speed_order",
    .speed_command = "" ///< SpeedPID will write speed_command
};

/// @brief IO keys for SpeedPIDController in tracker chain
/// @details
///   - Reads: speed_order (from TrackerCombiner), current_speed
///   - Writes: speed_command
static const motion_control::SpeedPIDControllerIOKeys motor_tracker_speed_pid_io_keys = {
    .speed_order = "speed_order",
    .current_speed = "current_speed",
    .speed_command = "speed_command",
    .reset = ""};

} // namespace actuators
} // namespace cogip

/// @}
