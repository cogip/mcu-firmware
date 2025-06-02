// RIOT includes
#include <ztimer.h>

// System includes
#include <cstdio>
#include <iostream>

// Project includes
#include "etl/list.h"
#include "etl/vector.h"
#include "thread/thread.hpp"

#include "platform_engine/PlatformEngine.hpp"

namespace cogip {

namespace motion_control {

void PlatformEngine::prepare_inputs() {
    // Update current pose and speed
    localization_.update();

    // Current pose
    io_.set("current_pose_x",  localization_.pose().x());
    io_.set("current_pose_y",  localization_.pose().y());
    io_.set("current_pose_o",  localization_.pose().O());

    // Target pose
    io_.set("target_pose_x",   target_pose_.x());
    io_.set("target_pose_y",   target_pose_.y());
    io_.set("target_pose_o",   target_pose_.O());

    // Current speed
    io_.set("current_speed_distance", localization_.delta_polar_pose().distance());
    io_.set("current_speed_angle", localization_.delta_polar_pose().angle());

    // Target speed
    io_.set("target_speed_distance",  target_speed_.distance());
    io_.set("target_speed_angle",  target_speed_.angle());

    // Allow reverse
    io_.set("allow_reverse",   target_pose_.allow_reverse());

    // Mark measured values read‑only:
    io_.mark_readonly("current_pose_x");
    io_.mark_readonly("current_pose_y");
    io_.mark_readonly("current_pose_o");
    io_.mark_readonly("current_speed_distance");
    io_.mark_readonly("current_speed_angle");
};

void PlatformEngine::process_outputs() {
    // If timeout is enabled, pose_reached_ has been set by the engine itself, do not override it.
    if (!timeout_enable_)
        pose_reached_ = io_.get_as<target_pose_status_t>("pose_reached").value();

    cogip_defs::Polar command(0, 0);

    if (pose_reached_ == target_pose_status_t::moving) {
        command.set_distance(io_.get_as<float>("linear_speed_command").value());
        command.set_angle(io_.get_as<float>("angular_speed_command").value());
    }

    // Set robot polar velocity order
    drive_contoller_.set_polar_velocity(command);

    // Dispatch pose reached state
    pose_reached_cb_(pose_reached_);
};

} // namespace motion_control

} // namespace cogip
