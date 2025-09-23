// RIOT includes
#include <ztimer.h>

// System includes
#include <cstdio>

// Project includes
#include "etl/list.h"
#include "etl/vector.h"
#include "thread/thread.hpp"

#include "motor_engine/MotorEngine.hpp"
#include "log.h"

namespace cogip {

namespace motion_control {

void MotorEngine::prepare_inputs() {
    // Update current distance and speed
    odometer_.update();
    float current_distance = odometer_.distance_mm();
    float current_speed = odometer_.delta_distance_mm();

    if (controller_) {
        // Current distance
        io_.set("current_pose", current_distance);

        // Target distance
        io_.set("target_pose", target_distance_);

        // Current speed
        io_.set("current_speed", current_speed);

        // Target speed
        io_.set("target_speed", target_speed_);

        // Position reached flag
        io_.set("pose_reached", target_pose_status_t::moving);
    }
};

void MotorEngine::process_outputs() {
    // If timeout is enabled, pose_reached_ has been set by the engine itself, do not override it.
    if (pose_reached_ != target_pose_status_t::timeout) {
        pose_reached_ = io_.get_as<target_pose_status_t>("pose_reached").value();
    }
    else {
        LOG_ERROR("MotorEngine timed out, disable.\n");

        // Disable engine
        enable_ = false;

        // Disable motor
        motor_.disable();

        return;
    }

    if (pose_reached_ == target_pose_status_t::blocked) {
        LOG_ERROR("MotorEngine blocked, disable.\n");

        // Disable engine
        enable_ = false;

        // Disable motor
        motor_.disable();

        return;
    }

    // Disable the timeout as we want to hold the position
    if ((pose_reached_ == target_pose_status_t::reached)
        && (timeout_enable_ == true)) {
        // Reset timeout cycles counter
        timeout_cycle_counter_ = timeout_ms_ / engine_thread_period_ms_;
    }

    float command = io_.get_as<float>("speed_command").value();

    motor_.set_speed(command);
};

} // namespace motion_control

} // namespace cogip
