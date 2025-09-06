// RIOT includes
#include <ztimer.h>

// System includes
#include <cstdio>
#include <iostream>

// Project includes
#include "etl/list.h"
#include "etl/vector.h"
#include "etl/absolute.h"

#include "motor_pose_filter/MotorPoseFilter.hpp"

namespace cogip {

namespace motion_control {

void MotorPoseFilter::execute(ControllersIO& io)
{
    std::cout << "Execute MotorPoseFilter" << std::endl;

    // Read current pose (default to zero if missing)
    float current_pose = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_pose)) {
        current_pose = *opt;
    }
    else {
        std::cout   << "WARNING: " << keys_.current_pose
                    << " is not available, using default value "
                    << current_pose << std::endl;
    }

    // Read target pose (default to zero if missing)
    float target_pose = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_pose)) {
        target_pose = *opt;
    }
    else {
        std::cout   << "WARNING: " << keys_.target_pose
                    << " is not available, using default value "
                    << target_pose << std::endl;
    }

    // Read current speed (default to zero if missing)
    float current_speed = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_speed)) {
        current_speed = *opt;
    }
    else {
        std::cout   << "WARNING: " << keys_.current_speed
                    << " is not available, using default value "
                    << current_speed << std::endl;
    }

    // Read target speed (default to zero if missing)
    float target_speed = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_speed)) {
        target_speed = *opt;
    }
    else {
        std::cout   << "WARNING: " << keys_.target_speed
                    << " is not available, using default value "
                    << target_speed << std::endl;
    }

    // Read pose reached status (default to moving if missing)
    target_pose_status_t pose_reached = target_pose_status_t::moving;
    if (auto opt = io.get_as<float>(keys_.pose_reached)) {
        pose_reached = static_cast<target_pose_status_t>(static_cast<int>(*opt));
    }
    else {
        std::cout   << "WARNING: " << keys_.pose_reached
                    << " is not available, using default value "
                    << static_cast<int>(pose_reached) << std::endl;
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
    }
    else {
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
