// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include <cmath>
#include <inttypes.h>

#include "localization/LocalizationDifferential.hpp"
#include "log.h"
#ifdef MODULE_TELEMETRY
#include "telemetry/Telemetry.hpp"
#endif
#include "trigonometry.h"
#include "utils.hpp"

namespace cogip {

namespace localization {

int LocalizationDifferential::init()
{
    int error = left_encoder_.init();
    if (error) {
        LOG_ERROR("Left encoder init failed, error=%d\n", error);
    }
    int error_right = right_encoder_.init();
    if (error_right) {
        LOG_ERROR("Right encoder init failed, error=%d\n", error_right);
    }
    return error ? error : error_right;
}

void LocalizationDifferential::reset()
{
    left_encoder_.reset();
    right_encoder_.reset();
}

void LocalizationDifferential::set_pose(float x, float y, float O)
{
    pose_.set_x(x);
    pose_.set_y(y);
    pose_.set_O(O);
}

void LocalizationDifferential::set_pose(const cogip::cogip_defs::Pose& pose)
{
    pose_.set_x(pose.x());
    pose_.set_y(pose.y());
    pose_.set_O(pose.O());
}

int LocalizationDifferential::update()
{
    // Compute encoders wheels left and right linear delta in mm
    const float dL = left_encoder_.get_angle_and_reset() *
                     parameters_.left_wheel_diameter_mm.get() * parameters_.left_polarity.get();
    const float dR = right_encoder_.get_angle_and_reset() *
                     parameters_.right_wheel_diameter_mm.get() * parameters_.right_polarity.get();

    // Compute linear delta for robot in mm
    const float delta_linear_pose = (dL + dR) / 2;

    // Compute angular delta for robot in rad
    const float delta_angular_pose = (dR - dL) / parameters_.track_width_mm.get();

    // Compute angle in rad between -pi and pi
    float O_rad = DEG2RAD(pose_.O());
    O_rad = limit_angle_rad(O_rad + delta_angular_pose);

    // Compute x and y coordinates in mm
    pose_.set_x(pose_.x() + delta_linear_pose * cos(O_rad));
    pose_.set_y(pose_.y() + delta_linear_pose * sin(O_rad));
    pose_.set_O(RAD2DEG(O_rad));

    // Save polar pose delta since last call
    polar_.set_distance(delta_linear_pose);
    polar_.set_angle(RAD2DEG(delta_angular_pose));

    return 0;
}

void LocalizationDifferential::send_telemetry()
{
#ifdef MODULE_TELEMETRY
    using cogip::utils::operator"" _key_hash;
    cogip::telemetry::Telemetry::send<int64_t>("encoder_left"_key_hash, left_encoder_.counter());
    cogip::telemetry::Telemetry::send<int64_t>("encoder_right"_key_hash, right_encoder_.counter());
#endif
}

} // namespace localization

} // namespace cogip
