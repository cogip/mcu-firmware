#include "app_path.hpp"

#include <algorithm>
#include <iostream>

#include "platform.hpp"

namespace cogip {

namespace app {

cogip::app::Path *_path = nullptr;

Path::Path(Actions *actions) : path::Path(), actions_(actions)
{
    new_action();
}

const cogip::path::Pose *Path::current_pose()
{
    return current_pose_;
}

void Path::reset_current_pose_index()
{
    current_pose_index_ = 0;
}

void Path::new_action()
{
    current_action_ = actions_->new_action();
    if (current_action_) {
        current_pose_ = current_action_->current_pose();
        std::cout << "app::Path::new_action: current_pose = " << current_pose_ << std::endl;
    }
}

void Path::next()
{
    if (! current_action_) {
        new_action();
    }
    else {
        auto it = current_action_->next_pose();
        if (it == current_action_->end_pose()) {
            new_action();
        }
        else {
            current_pose_ = current_action_->current_pose();
            std::cout << "app::Path::next: current_pose = " << current_pose_ << std::endl;
        }
    }
}

double Path::current_max_speed_linear()
{
    return std::min(current_pose_->max_speed_linear(), (double)MAX_SPEED_LINEAR);
}

double Path::current_max_speed_angular()
{
    return std::min(current_pose_->max_speed_angular(), (double)MAX_SPEED_ANGULAR);
}

void Path::horizontal_mirror_all_poses()
{
    // for (auto &pose: *this) {
    //     pose.set_x(-pose.x());
    //     pose.set_O(180 - pose.O());
    //     pose.set_O(((int)pose.O()) % 360);
    // }
}

void Path::unreachable()
{
    std::cout << "unreachable" << std::endl;
    current_action_ = actions_->new_action(current_action_);
}

Path &app_get_path(void)
{
    if (! _path) {
        _path = new Path(app_actions_get());
    }
    return *_path;
}

void app_reset_path(void)
{
    if (_path) {
        delete _path;
        _path = nullptr;
        app_actions_reset();
    }
    app_get_path();
}

} // namespace app

} // namespace cogip
