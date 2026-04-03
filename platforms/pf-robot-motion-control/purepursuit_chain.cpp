// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief Tracker speed tuning chain implementation

#include "purepursuit_chain.hpp"
#include "motion_control.hpp"
#include "motion_control_common/MetaController.hpp"
#include "telemetry_controller/TelemetryController.hpp"
#include "telemetry_controller/TelemetryControllerIOKeys.hpp"
#include "telemetry_controller/TelemetryControllerParameters.hpp"

namespace cogip {
namespace pf {
namespace motion_control {
namespace purepursuit_chain {

// ============================================================================
// Telemetry
// ============================================================================

static const cogip::motion_control::TelemetryControllerIOKeys linear_telemetry_io_keys = {
    .speed_order = "linear_speed_order",
    .current_speed = "linear_current_speed",
    .tracker_velocity = "",
    .pose_error = "",
};

static const cogip::motion_control::TelemetryControllerIOKeys angular_telemetry_io_keys = {
    .speed_order = "angular_speed_order",
    .current_speed = "angular_current_speed",
    .tracker_velocity = "",
    .pose_error = "",
};

static cogip::motion_control::TelemetryControllerParameters telemetry_controller_parameters{
    .loop_period_ms = motion_control_thread_period_ms};

static cogip::motion_control::TelemetryController
    linear_telemetry_controller(linear_telemetry_io_keys, telemetry_controller_parameters);

static cogip::motion_control::TelemetryController
    angular_telemetry_controller(angular_telemetry_io_keys, telemetry_controller_parameters);

// ============================================================================
// Initialization function
// ============================================================================

cogip::motion_control::MetaController<>* init()
{
    // Linear loop: SpeedRampFilter -> SpeedPID
    linear_meta_controller.add_controller(&linear_speed_ramp_filter);
    linear_meta_controller.add_controller(&linear_speed_controller);

    // Angular loop: SpeedRampFilter -> SpeedPID
    angular_meta_controller.add_controller(&angular_speed_ramp_filter);
    angular_meta_controller.add_controller(&angular_speed_controller);

    // Run linear + angular in parallel
    polar_parallel_meta_controller.add_controller(&linear_meta_controller);
    polar_parallel_meta_controller.add_controller(&angular_meta_controller);

    // Main chain: PurePursuit -> parallel loops + telemetry
    pose_loop_meta_controller.add_controller(&purepursuit_controller);
    pose_loop_meta_controller.add_controller(&polar_parallel_meta_controller);
    pose_loop_meta_controller.add_controller(&linear_telemetry_controller);
    pose_loop_meta_controller.add_controller(&angular_telemetry_controller);

    return &pose_loop_meta_controller;
}

} // namespace purepursuit_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
