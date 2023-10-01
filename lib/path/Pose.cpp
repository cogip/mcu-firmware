#include "path/Pose.hpp"

#include <algorithm>

namespace cogip {

namespace path {

Pose::Pose(
    double x, double y, double O,
    double max_speed_ratio_linear, double max_speed_ratio_angular,
    bool allow_reverse, func_cb_t act_
    ) : cogip_defs::Pose(x, y, O), allow_reverse_(allow_reverse), act_(act_)
{
    // Ratios are betwen 0 and 1
    max_speed_ratio_linear_ =  std::min(max_speed_ratio_linear, 1.);
    max_speed_ratio_angular_ = std::min(max_speed_ratio_angular, 1.);
}

void Pose::pb_read(const PB_PathPose &path_pose)
{
    auto & pose = path_pose.pose();
    x_ = pose.x();
    y_ = pose.y();
    O_ = pose.O();
    allow_reverse_ = path_pose.allow_reverse();
    max_speed_ratio_linear_ = path_pose.max_speed_ratio_linear();
    max_speed_ratio_angular_ = path_pose.max_speed_ratio_angular();
}

void Pose::pb_copy(PB_PathPose &path_pose) const {
    auto & pose = path_pose.mutable_pose();
    pose.set_x(x_);
    pose.set_y(y_);
    pose.set_O(O_);
    path_pose.set_allow_reverse(allow_reverse_);
    path_pose.set_max_speed_ratio_linear(max_speed_ratio_linear_);
    path_pose.set_max_speed_ratio_angular(max_speed_ratio_angular_);
}

} // namespace path

} // namespace cogip
