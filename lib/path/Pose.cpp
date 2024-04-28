#include "path/Pose.hpp"

#include <algorithm>

namespace cogip {

namespace path {

Pose::Pose(
    double x, double y, double O,
    double max_speed_ratio_linear, double max_speed_ratio_angular,
    bool allow_reverse, bool bypass_anti_blocking, uint32_t timeout_ms
    ) : cogip_defs::Pose(x, y, O), allow_reverse_(allow_reverse),
    bypass_anti_blocking_(bypass_anti_blocking), timeout_ms_(timeout_ms)
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
    bypass_anti_blocking_ = path_pose.bypass_anti_blocking();
    timeout_ms_ = path_pose.timeout_ms();
}

void Pose::pb_copy(PB_PathPose &path_pose) const {
    auto & pose = path_pose.mutable_pose();
    pose.set_x(x_);
    pose.set_y(y_);
    pose.set_O(O_);
    path_pose.set_allow_reverse(allow_reverse_);
    path_pose.set_max_speed_ratio_linear(max_speed_ratio_linear_);
    path_pose.set_max_speed_ratio_angular(max_speed_ratio_angular_);
    path_pose.set_bypass_anti_blocking(bypass_anti_blocking_);
    path_pose.set_timeout_ms(timeout_ms_);
}

} // namespace path

} // namespace cogip
