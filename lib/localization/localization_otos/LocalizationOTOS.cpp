// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     localization
/// @{
/// @file
/// @brief       Localization implementation using SparkFun OTOS sensor
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#include <cmath>

#include "localization/LocalizationOTOS.hpp"
#include "trigonometry.h"

namespace cogip {
namespace localization {

LocalizationOTOS::LocalizationOTOS(cogip::otos::OTOS& otos, const Parameters& params)
    : otos_(otos), params_(params), pose_(), prev_pose_(), polar_(), first_update_(true)
{
}

int LocalizationOTOS::init()
{
    int ret = otos_.init();
    if (ret < 0) {
        return ret;
    }
    otos_.set_linear_scalar(params_.linear_scalar);
    otos_.set_angular_scalar(params_.angular_scalar);
    otos_.set_offset(params_.offset_x_mm, params_.offset_y_mm, params_.offset_h_deg);
    return otos_.calibrate_imu();
}

void LocalizationOTOS::reset()
{
    // OTOS tracks absolute position, no counters to reset
}

void LocalizationOTOS::set_pose(float x, float y, float O)
{
    pose_.set_x(x);
    pose_.set_y(y);
    pose_.set_O(O);
    prev_pose_ = pose_;
    otos_.set_position(x, y, O);
}

void LocalizationOTOS::set_pose(const cogip::cogip_defs::Pose& pose)
{
    set_pose(pose.x(), pose.y(), pose.O());
}

const cogip::cogip_defs::Pose& LocalizationOTOS::pose()
{
    return pose_;
}

const cogip::cogip_defs::Polar& LocalizationOTOS::delta_polar_pose()
{
    return polar_;
}

int LocalizationOTOS::update()
{
    int ret = otos_.update();
    if (ret < 0) {
        return ret;
    }

    const auto& otos_pose = otos_.pose();

    // Store previous pose for delta computation
    prev_pose_ = pose_;

    // Update pose from sensor (already in mm and degrees)
    pose_.set_x(otos_pose.x);
    pose_.set_y(otos_pose.y);
    pose_.set_O(otos_pose.h);

    // On first update, no meaningful delta
    if (first_update_) {
        polar_.set_distance(0.0f);
        polar_.set_angle(0.0f);
        first_update_ = false;
        return 0;
    }

    // Compute linear delta by projecting displacement onto heading direction
    // (matches the convention of LocalizationDifferential where positive = forward)
    float dx = pose_.x() - prev_pose_.x();
    float dy = pose_.y() - prev_pose_.y();
    float heading_rad = DEG2RAD(prev_pose_.O());
    float delta_linear = dx * cos(heading_rad) + dy * sin(heading_rad);

    // Compute angular delta, normalized to [-180, 180]
    float delta_angular = limit_angle_deg(pose_.O() - prev_pose_.O());

    polar_.set_distance(delta_linear);
    polar_.set_angle(delta_angular);

    return 0;
}

} // namespace localization
} // namespace cogip

/// @}
