#include "path/Path.hpp"

#include <algorithm>

namespace cogip {

namespace path {

size_t Path::operator++(int)
{
    if (current_pose_index_ < size() - 1) {
        current_pose_index_ += 1;
    }
    else if (play_in_loop_) {
        current_pose_index_ = 0;
    }
    return current_pose_index_;
}

size_t Path::operator--(int)
{
    if (current_pose_index_ > 0) {
        current_pose_index_ -= 1;
    }
    else if (play_in_loop_) {
        current_pose_index_ = size() - 1;
    }
    return current_pose_index_;
}

double Path::current_max_speed_linear()
{
    return at(current_pose_index_).max_speed_linear();
}

double Path::current_max_speed_angular()
{
    return at(current_pose_index_).max_speed_angular();
}

void Path::horizontal_mirror_all_poses()
{
    for (auto &pose: *this) {
        pose.set_x(-pose.x());
        pose.set_O(-pose.O());
    }
}

} // namespace path

} // namespace cogip
