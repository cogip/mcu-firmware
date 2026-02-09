// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief DualPID Tracker chain controller instances for Motor/Lift actuators
/// @details Controllers for feedforward + feedback control chain:
///          ProfileTrackerController → PosePIDController → TrackerCombinerController →
///          SpeedPIDController This provides smoother motion than the classic DualPID chain by
///          using trapezoidal velocity profiles with tracking error correction.

#pragma once

#include "motion_control_common/MetaController.hpp"
#include "pid/PID.hpp"
#include "pid/PIDParameters.hpp"
#include "pose_pid_controller/PosePIDController.hpp"
#include "pose_pid_controller/PosePIDControllerIOKeys.hpp"
#include "pose_pid_controller/PosePIDControllerParameters.hpp"
#include "profile_tracker_controller/ProfileTrackerController.hpp"
#include "profile_tracker_controller/ProfileTrackerControllerIOKeys.hpp"
#include "profile_tracker_controller/ProfileTrackerControllerParameters.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "speed_pid_controller/SpeedPIDControllerIOKeys.hpp"
#include "speed_pid_controller/SpeedPIDControllerParameters.hpp"
#include "tracker_combiner_controller/TrackerCombinerController.hpp"
#include "tracker_combiner_controller/TrackerCombinerControllerIOKeys.hpp"
#include "tracker_combiner_controller/TrackerCombinerControllerParameters.hpp"

namespace cogip {
namespace pf {
namespace motion_control {
namespace dualpid_tracker_chain {

// ============================================================================
// IO Keys for Motor Tracker Chain
// ============================================================================

/// @brief IO keys for ProfileTrackerController
/// @details
///   - Reads: pose_error (from MotorEngine via recompute), current_speed
///   - Writes: tracker_velocity, tracking_error
inline constexpr cogip::motion_control::ProfileTrackerControllerIOKeys profile_tracker_io_keys = {
    .pose_error = "pose_error",        ///< Distance remaining (from pose filter)
    .current_speed = "speed_order",    ///< Use speed_order for profile continuity
    .recompute_profile = "new_target", ///< Flag to regenerate profile
    .tracker_velocity = "tracker_velocity",
    .tracking_error = "tracking_error",
    .profile_complete = "",        ///< Not used, pose filter handles pose_reached
    .target_speed = "target_speed" ///< Optional speed limit from path
};

/// @brief IO keys for PosePIDController (tracking error correction)
/// @details
///   - Reads: tracking_error (from ProfileTrackerController)
///   - Writes: feedback_correction
inline constexpr cogip::motion_control::PosePIDControllerIOKeys tracker_pose_pid_io_keys = {
    .position_error = "tracking_error",
    .current_speed = "current_speed",
    .target_speed = "target_speed",
    .disable_filter = "", ///< Not used
    .pose_reached = "",   ///< Not used
    .speed_order = "feedback_correction",
    .reset = "" ///< No reset key
};

/// @brief IO keys for TrackerCombinerController
/// @details
///   - Reads: tracker_velocity, feedback_correction
///   - Writes: speed_order
inline constexpr cogip::motion_control::TrackerCombinerControllerIOKeys tracker_combiner_io_keys = {
    .tracker_velocity = "tracker_velocity",
    .feedback_correction = "feedback_correction",
    .speed_order = "speed_order",
    .speed_command = "" ///< SpeedPID will write speed_command
};

/// @brief IO keys for SpeedPIDController
/// @details
///   - Reads: speed_order, current_speed
///   - Writes: speed_command
inline constexpr cogip::motion_control::SpeedPIDControllerIOKeys tracker_speed_pid_io_keys = {
    .speed_order = "speed_order",
    .current_speed = "current_speed",
    .speed_command = "speed_command",
    .reset = "" ///< No reset key
};

// ============================================================================
// Controller Parameters Structure
// ============================================================================

/// @brief Parameters for initializing a DualPID Tracker Chain
struct DualPIDTrackerChainParameters
{
    /// @brief PID for pose (tracking error) correction
    cogip::pid::PID* pose_pid;

    /// @brief PID for speed control
    cogip::pid::PID* speed_pid;

    /// @brief Maximum speed (mm/period)
    float max_speed_mm_per_period;

    /// @brief Acceleration (mm/period²)
    float acceleration_mm_per_period2;

    /// @brief Deceleration (mm/period²)
    float deceleration_mm_per_period2;
};

// ============================================================================
// Chain Instance Structure
// ============================================================================

/// @brief A complete DualPID Tracker Chain instance
/// @details Contains all controllers needed for the tracker chain.
///          Each lift actuator should have its own instance of this structure.
struct DualPIDTrackerChain
{
    /// @brief Profile tracker controller - generates trapezoidal velocity profile
    cogip::motion_control::ProfileTrackerController profile_tracker;

    /// @brief Pose PID controller - corrects tracking error
    cogip::motion_control::PosePIDController pose_pid;

    /// @brief Tracker combiner - adds feedforward + feedback
    cogip::motion_control::TrackerCombinerController combiner;

    /// @brief Speed PID controller - velocity control loop
    cogip::motion_control::SpeedPIDController speed_pid;

    /// @brief Meta controller chaining all controllers
    cogip::motion_control::MetaController<> meta_controller;

    /// @brief Initialize the chain by connecting controllers
    void init()
    {
        meta_controller.add_controller(&profile_tracker);
        meta_controller.add_controller(&pose_pid);
        meta_controller.add_controller(&combiner);
        meta_controller.add_controller(&speed_pid);
    }

    /// @brief Reset all controllers in the chain
    void reset()
    {
        meta_controller.reset();
    }

    /// @brief Get the meta controller for use with MotorEngine
    cogip::motion_control::MetaController<>* get_controller()
    {
        return &meta_controller;
    }
};

/// @brief Create a DualPIDTrackerChain with the given parameters
/// @param params Configuration parameters for the chain
/// @return A fully configured DualPIDTrackerChain instance
inline DualPIDTrackerChain create_chain(const DualPIDTrackerChainParameters& params)
{
    // Create profile tracker parameters
    cogip::motion_control::ProfileTrackerControllerParameters profile_params(
        params.max_speed_mm_per_period, params.acceleration_mm_per_period2,
        params.deceleration_mm_per_period2,
        true, // must_stop_at_end
        1     // period_increment
    );

    // Create pose PID parameters
    cogip::motion_control::PosePIDControllerParameters pose_pid_params(params.pose_pid);

    // Create combiner parameters (empty)
    cogip::motion_control::TrackerCombinerControllerParameters combiner_params;

    // Create speed PID parameters
    cogip::motion_control::SpeedPIDControllerParameters speed_pid_params(params.speed_pid);

    // Create the chain
    DualPIDTrackerChain chain{
        .profile_tracker = cogip::motion_control::ProfileTrackerController(profile_tracker_io_keys,
                                                                           profile_params),
        .pose_pid =
            cogip::motion_control::PosePIDController(tracker_pose_pid_io_keys, pose_pid_params),
        .combiner = cogip::motion_control::TrackerCombinerController(tracker_combiner_io_keys,
                                                                     combiner_params),
        .speed_pid =
            cogip::motion_control::SpeedPIDController(tracker_speed_pid_io_keys, speed_pid_params),
        .meta_controller = {}};

    // Initialize the chain
    chain.init();

    return chain;
}

} // namespace dualpid_tracker_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
