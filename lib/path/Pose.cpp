#include "path/Pose.hpp"

#include <algorithm>

namespace cogip {

namespace path {

Pose::Pose(float x, float y, float O, float max_speed_ratio_linear, float max_speed_ratio_angular,
           bool allow_reverse, bool bypass_anti_blocking, uint32_t timeout_ms,
           bool bypass_final_orientation, bool is_intermediate)
    : cogip_defs::Pose(x, y, O), allow_reverse_(allow_reverse),
      bypass_anti_blocking_(bypass_anti_blocking), timeout_ms_(timeout_ms),
      bypass_final_orientation_(bypass_final_orientation), is_intermediate_(is_intermediate)
{
    // Ratios are between 0 and 1
    max_speed_ratio_linear_ = std::min(max_speed_ratio_linear, 1.0f);
    max_speed_ratio_angular_ = std::min(max_speed_ratio_angular, 1.0f);
}

void Pose::pb_read(const PB_PathPose& path_pose)
{
    auto& pose = path_pose.pose();
    x_ = pose.x();
    y_ = pose.y();
    O_ = pose.O();
    allow_reverse_ = path_pose.allow_reverse();
    max_speed_ratio_linear_ = path_pose.max_speed_ratio_linear();
    max_speed_ratio_angular_ = path_pose.max_speed_ratio_angular();
    bypass_anti_blocking_ = path_pose.bypass_anti_blocking();
    timeout_ms_ = path_pose.timeout_ms();
    bypass_final_orientation_ = path_pose.bypass_final_orientation();
    is_intermediate_ = path_pose.is_intermediate();
}

void Pose::pb_copy(PB_PathPose& path_pose) const
{
    auto& pose = path_pose.mutable_pose();
    pose.set_x(x_);
    pose.set_y(y_);
    pose.set_O(O_);
    path_pose.set_allow_reverse(allow_reverse_);
    path_pose.set_max_speed_ratio_linear(max_speed_ratio_linear_);
    path_pose.set_max_speed_ratio_angular(max_speed_ratio_angular_);
    path_pose.set_bypass_anti_blocking(bypass_anti_blocking_);
    path_pose.set_timeout_ms(timeout_ms_);
    path_pose.set_bypass_final_orientation(bypass_final_orientation_);
}

} // namespace path

} // namespace cogip
