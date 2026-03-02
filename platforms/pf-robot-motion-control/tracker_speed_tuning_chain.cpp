// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief Tracker speed tuning chain implementation

#include "tracker_speed_tuning_chain.hpp"
#include "motion_control.hpp"
#include "motion_control_common/MetaController.hpp"
#include "telemetry_controller/TelemetryController.hpp"
#include "telemetry_controller/TelemetryControllerIOKeysDefault.hpp"
#include "telemetry_controller/TelemetryControllerParameters.hpp"

namespace cogip {
namespace pf {
namespace motion_control {
namespace tracker_speed_tuning_chain {

// ============================================================================
// Telemetry
// ============================================================================

static cogip::motion_control::TelemetryControllerParameters telemetry_controller_parameters{
    .loop_period_ms = motion_control_thread_period_ms};

static cogip::motion_control::TelemetryController
    linear_telemetry_controller(cogip::motion_control::linear_telemetry_controller_io_keys_default,
                                telemetry_controller_parameters);

static cogip::motion_control::TelemetryController angular_telemetry_controller(
    cogip::motion_control::angular_telemetry_controller_io_keys_default,
    telemetry_controller_parameters);

// ============================================================================
// Initialization function
// ============================================================================

cogip::motion_control::MetaController<>* init()
{
    // Linear speed loop: TargetChangeDetector -> ProfileTracker -> SpeedPID -> TrackerCombiner
    linear_meta_controller.add_controller(&linear_target_change_detector);
    linear_meta_controller.add_controller(&linear_profile_tracker_controller);
    linear_meta_controller.add_controller(&linear_speed_controller);
    linear_meta_controller.add_controller(&linear_tracker_combiner_controller);

    // Angular speed loop: TargetChangeDetector -> ProfileTracker -> SpeedPID -> TrackerCombiner
    angular_meta_controller.add_controller(&angular_target_change_detector);
    angular_meta_controller.add_controller(&angular_profile_tracker_controller);
    angular_meta_controller.add_controller(&angular_speed_controller);
    angular_meta_controller.add_controller(&angular_tracker_combiner_controller);

    // Run linear + angular in parallel
    polar_parallel_meta_controller.add_controller(&linear_meta_controller);
    polar_parallel_meta_controller.add_controller(&angular_meta_controller);

    // Main chain: parallel speed loops + telemetry
    meta_controller.add_controller(&polar_parallel_meta_controller);
    meta_controller.add_controller(&linear_telemetry_controller);
    meta_controller.add_controller(&angular_telemetry_controller);

    return &meta_controller;
}

} // namespace tracker_speed_tuning_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
