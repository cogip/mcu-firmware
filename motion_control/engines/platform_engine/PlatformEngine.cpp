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
    io_.set("motion_direction", static_cast<int>(target_pose_.get_motion_direction()));

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
    // If timeout is enabled, pose_reached_ has been set by the engine itself, do
    // not override it.
    if (!timeout_enable_) {
        pose_reached_ = io_.get_as<target_pose_status_t>("pose_reached").value();
    }

    cogip_defs::Polar command(0, 0);

    if (pose_reached_ == target_pose_status_t::moving) {
        DEBUG("Start process_outputs()\n");
        command.set_distance(io_.get_as<float>("linear_speed_command").value());
        command.set_angle(io_.get_as<float>("angular_speed_command").value());
        DEBUG("End process_outputs()\n");
    }

    // Set robot polar velocity order
    drive_contoller_.set_polar_velocity(command);

    // Dispatch pose reached state
    pose_reached_cb_(pose_reached_);
};

} // namespace motion_control

} // namespace cogip
