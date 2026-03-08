// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief Angular pose tuning chain implementation

#include "angular_pose_tuning_chain.hpp"
#include "motion_control.hpp"
#include "motion_control_common/MetaController.hpp"
#include "pose_pid_controller/PosePIDController.hpp"
#include "pose_pid_controller/PosePIDControllerParameters.hpp"
#include "quadpid_tracker_chain.hpp"
#include "telemetry_controller/TelemetryController.hpp"
#include "telemetry_controller/TelemetryControllerIOKeysDefault.hpp"
#include "telemetry_controller/TelemetryControllerParameters.hpp"

namespace cogip {
namespace pf {
namespace motion_control {
namespace angular_pose_tuning_chain {

// ============================================================================
// Meta controller, telemetry, and pose controller
// ============================================================================

static cogip::motion_control::TelemetryControllerParameters telemetry_controller_parameters;

static cogip::motion_control::TelemetryController
    telemetry_controller(cogip::motion_control::angular_telemetry_controller_io_keys_default,
                         telemetry_controller_parameters);

static cogip::motion_control::PosePIDControllerParameters
    pose_controller_parameters(&quadpid_tracker_chain::tracker_angular_pose_pid);

static cogip::motion_control::PosePIDController pose_controller(pose_pid_io_keys,
                                                                pose_controller_parameters);

// ============================================================================
// Initialization function
// ============================================================================

cogip::motion_control::MetaController<>* init()
{
    // Chain: TargetChangeDetector -> PoseErrorFilter -> ProfileTrackerController ->
    // PosePIDController -> TrackerCombinerController -> SpeedPIDController ->
    // TuningPoseReachedFilter -> TelemetryController
    meta_controller.add_controller(&target_change_detector);
    meta_controller.add_controller(&pose_error_filter);
    meta_controller.add_controller(&profile_tracker_controller);
    meta_controller.add_controller(&pose_controller);
    meta_controller.add_controller(&tracker_combiner_controller);
    meta_controller.add_controller(&speed_controller);
    meta_controller.add_controller(&tuning_pose_reached_filter);
    meta_controller.add_controller(&telemetry_controller);

    return &meta_controller;
}

} // namespace angular_pose_tuning_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
