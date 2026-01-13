// RIOT includes
#include <ztimer.h>

// System includes
#include <cstdio>

// Project includes
#include "etl/list.h"
#include "etl/vector.h"
#include "thread/thread.hpp"

#include "log.h"
#include "platform_engine/PlatformEngine.hpp"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

PlatformEngine::PlatformEngine(localization::LocalizationInterface& localization,
                               drive_controller::DriveControllerInterface& drive_contoller,
                               pose_reached_cb_t pose_reached_cb, uint32_t engine_thread_period_ms)
    : BaseControllerEngine(engine_thread_period_ms), localization_(localization),
      drive_contoller_(drive_contoller), pose_reached_cb_(pose_reached_cb)
{
}

void PlatformEngine::prepare_inputs()
{
    // Update current pose and speed
    localization_.update();

    // Reset read-only markers to allow engine updates
    io_.reset_readonly_markers();

    // Current pose
    io_.set("current_pose_x", localization_.pose().x());
    io_.set("current_pose_y", localization_.pose().y());
    io_.set("current_pose_O", localization_.pose().O());

    // Target pose
    io_.set("target_pose_x", target_pose_.x());
    io_.set("target_pose_y", target_pose_.y());
    io_.set("target_pose_O", target_pose_.O());

    // Current speed
    io_.set("linear_current_speed", localization_.delta_polar_pose().distance());
    io_.set("angular_current_speed", localization_.delta_polar_pose().angle());

    // Target speed
    io_.set("linear_target_speed", target_speed_.distance());
    io_.set("angular_target_speed", target_speed_.angle());

    // Motion direction
    io_.set("motion_direction", target_pose_.get_motion_direction());

    // Initialize pose_reached to moving (will be updated by PoseStraightFilter)
    io_.set("pose_reached", pose_reached_);

    // Initialize speed commands to 0 (will be updated by SpeedPIDController)
    io_.set("linear_speed_command", 0.0f);
    io_.set("angular_speed_command", 0.0f);

    // Initialize speed orders to 0 (will be updated by PosePIDController or FeedforwardCombiner)
    io_.set("linear_speed_order", 0.0f);
    io_.set("angular_speed_order", 0.0f);

    // Mark measured values read‑only:
    io_.mark_readonly("motion_direction");
    io_.mark_readonly("current_pose_x");
    io_.mark_readonly("current_pose_y");
    io_.mark_readonly("current_pose_O");
    io_.mark_readonly("linear_current_speed");
    io_.mark_readonly("angular_current_speed");
};

void PlatformEngine::process_outputs()
{
    // Check pose_reached from IO (set by controllers like PoseErrorFilter or PoseStraightFilter)
    auto io_pose_reached = io_.get_as<target_pose_status_t>("pose_reached");
    if (io_pose_reached && *io_pose_reached == target_pose_status_t::reached) {
        // Target reached - use this regardless of timeout mode
        pose_reached_ = target_pose_status_t::reached;
    } else if (!timeout_enable_) {
        // No timeout mode: use IO value directly
        pose_reached_ = io_pose_reached.value_or(target_pose_status_t::moving);
    }
    // If timeout is enabled and not reached: pose_reached_ keeps its current value
    // (either 'moving' or 'timeout' set by the engine)

    cogip_defs::Polar command(0, 0);

    // if (pose_reached_ == target_pose_status_t::moving) {
    DEBUG("Start process_outputs()\n");
    command.set_distance(io_.get_as<float>("linear_speed_command").value());
    command.set_angle(io_.get_as<float>("angular_speed_command").value());
    DEBUG("End process_outputs()\n");
    //}

    // Set robot polar velocity order
    drive_contoller_.set_polar_velocity(command);

    // Dispatch pose reached state
    pose_reached_cb_(pose_reached_);
};

} // namespace motion_control

} // namespace cogip
