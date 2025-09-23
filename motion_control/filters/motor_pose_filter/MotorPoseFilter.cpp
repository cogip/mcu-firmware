// RIOT includes
#include <ztimer.h>

// System includes
#include <cstdio>

// Project includes
#include "etl/absolute.h"
#include "etl/list.h"
#include "etl/vector.h"

#include "log.h"
#include "motor_pose_filter/MotorPoseFilter.hpp"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

void MotorPoseFilter::execute(ControllersIO& io)
{
    DEBUG("Execute MotorPoseFilter");

    // Read current pose (default to zero if missing)
    float current_pose = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_pose)) {
        current_pose = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f",
                    keys_.current_pose.data(), current_pose);
    }

    // Read target pose (default to zero if missing)
    float target_pose = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_pose)) {
        target_pose = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f",
                    keys_.target_pose.data(), target_pose);
    }

    // Read current speed (default to zero if missing)
    float current_speed = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_speed)) {
        current_speed = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f",
                    keys_.current_speed.data(), current_speed);
    }

    // Read target speed (default to zero if missing)
    float target_speed = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_speed)) {
        target_speed = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f",
                    keys_.target_speed.data(), target_speed);
    }

    // Read pose reached status (default to moving if missing)
    target_pose_status_t pose_reached = target_pose_status_t::moving;
    if (auto opt = io.get_as<float>(keys_.pose_reached)) {
        pose_reached = static_cast<target_pose_status_t>(static_cast<int>(*opt));
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %d",
                    keys_.pose_reached.data(), static_cast<int>(pose_reached));
    }

    // Compute pose difference
    float position_error = target_pose - current_pose;

    // Decide whether to stop or decelerate based on thresholds
    bool no_speed_limit = false;
    float abs_error = etl::absolute(position_error);
    float threshold = parameters_.threshold();
    float decel = parameters_.deceleration();

    if (abs_error <= threshold) {
        // final pose reached
        pose_reached = target_pose_status_t::reached;
        target_speed = 0.0f;
    } else {
        // compute deceleration if needed
        float stopping_distance = (current_speed * current_speed) / (2.0f * decel);
        if (abs_error <= stopping_distance) {
            target_speed = std::sqrt(2.0f * decel * abs_error);
        }
    }

    // Write pose error
    io.set(keys_.position_error, position_error);

    // Pass through current speed
    io.set(keys_.current_speed, current_speed);

    // Write filtered target speed as absolute value
    io.set(keys_.filtered_speed, etl::absolute(target_speed));

    // Indicate whether speed limitation is disabled
    io.set(keys_.speed_filter_flag, static_cast<float>(no_speed_limit));

    // Write updated pose reached status
    io.set(keys_.pose_reached_out, static_cast<float>(pose_reached));
}

} // namespace motion_control

} // namespace cogip
