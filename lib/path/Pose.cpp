#include "path/Path.hpp"

#include <algorithm>

#include "motion_control.hpp"

#include "PB_SpeedEnum.hpp"

namespace cogip {

namespace path {

Pose::Pose(
    double x, double y, double O,
    double max_speed_linear, double max_speed_angular,
    bool allow_reverse, func_cb_t act_
    ) : cogip_defs::Pose(x, y, O), allow_reverse_(allow_reverse), act_(act_)
{
    max_speed_linear_ =  std::min(max_speed_linear, (double)max_speed_linear);
    max_speed_angular_ = std::min(max_speed_angular, (double)max_speed_angular);
}

void Pose::pb_read(const PB_PathPose &path_pose)
{
    auto & pose = path_pose.pose();
    x_ = pose.x();
    y_ = pose.y();
    O_ = pose.O();
    allow_reverse_ = path_pose.allow_reverse();

    double speed_order = 0;
    switch (path_pose.get_max_speed_linear_enum()) {
        case PB_SpeedEnum::LOW:
            speed_order = cogip::pf::motion_control::platform_low_speed_linear_mm_per_period;
            break;
        case PB_SpeedEnum::MAX:
            speed_order = cogip::pf::motion_control::platform_max_speed_linear_mm_per_period;
            break;
        case PB_SpeedEnum::NORMAL:
        default:
            speed_order =  cogip::pf::motion_control::platform_normal_speed_linear_mm_per_period;
    }
    max_speed_linear_ = speed_order;

    switch (path_pose.get_max_speed_angular_enum()) {
        case PB_SpeedEnum::LOW:
            speed_order =  cogip::pf::motion_control::platform_low_speed_angular_deg_per_period;
            break;
        case PB_SpeedEnum::MAX:
            speed_order =  cogip::pf::motion_control::platform_max_speed_angular_deg_per_period;
            break;
        case PB_SpeedEnum::NORMAL:
        default:
            speed_order =  cogip::pf::motion_control::platform_normal_speed_angular_deg_per_period;
    }
    max_speed_angular_ = speed_order;
}

void Pose::pb_copy(PB_PathPose &path_pose) const {
    auto & pose = path_pose.mutable_pose();
    pose.set_x(x_);
    pose.set_y(y_);
    pose.set_O(O_);
    path_pose.set_allow_reverse(allow_reverse_);

    if (max_speed_linear_ == cogip::pf::motion_control::platform_low_speed_linear_mm_per_period) {
        path_pose.set_max_speed_linear_enum(PB_SpeedEnum::LOW);
    }
    else if (max_speed_linear_ == cogip::pf::motion_control::platform_max_speed_linear_mm_per_period) {
        path_pose.set_max_speed_linear_enum(PB_SpeedEnum::MAX);
    }
    else {
        path_pose.set_max_speed_linear_enum(PB_SpeedEnum::NORMAL);
    }

    if (max_speed_angular_ == cogip::pf::motion_control::platform_low_speed_angular_deg_per_period) {
        path_pose.set_max_speed_angular_enum(PB_SpeedEnum::LOW);
    }
    else if (max_speed_angular_ == cogip::pf::motion_control::platform_max_speed_angular_deg_per_period) {
        path_pose.set_max_speed_angular_enum(PB_SpeedEnum::MAX);
    }
    else {
        path_pose.set_max_speed_angular_enum(PB_SpeedEnum::NORMAL);
    }
}

} // namespace path

} // namespace cogip
