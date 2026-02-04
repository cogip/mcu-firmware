// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     lib_path
/// @{
/// @file
/// @brief       Path class implementation
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#include "path/Path.hpp"

namespace cogip {

namespace path {

Path& Path::instance()
{
    static Path instance;
    return instance;
}

Path::Path() : current_index_(0), started_(false) {}

void Path::reset()
{
    waypoints_.clear();
    current_index_ = 0;
    started_ = false;
}

bool Path::add_point(const Pose& pose)
{
    if (waypoints_.full()) {
        return false;
    }
    waypoints_.push_back(pose);
    return true;
}

bool Path::add_point_from_pb(const PB_PathPose& pb_pose)
{
    Pose pose;
    pose.pb_read(pb_pose);
    return add_point(pose);
}

void Path::start()
{
    if (!waypoints_.empty()) {
        started_ = true;
        current_index_ = 0;
    }
}

void Path::stop()
{
    started_ = false;
}

bool Path::advance()
{
    if (!started_ || waypoints_.empty()) {
        return false;
    }

    if (current_index_ < waypoints_.size() - 1) {
        current_index_++;
        return true;
    }

    return false; // Already at last waypoint
}

const Pose* Path::current_pose() const
{
    if (!started_ || waypoints_.empty() || current_index_ >= waypoints_.size()) {
        return nullptr;
    }
    return &waypoints_[current_index_];
}

const Pose* Path::waypoint_at(size_t index) const
{
    if (index >= waypoints_.size()) {
        return nullptr;
    }
    return &waypoints_[index];
}

bool Path::is_started() const
{
    return started_;
}

bool Path::is_complete() const
{
    if (!started_ || waypoints_.empty()) {
        return true;
    }
    return current_index_ >= waypoints_.size() - 1;
}

bool Path::is_last_waypoint() const
{
    if (!started_ || waypoints_.empty()) {
        return false;
    }
    return current_index_ == waypoints_.size() - 1;
}

size_t Path::current_index() const
{
    return current_index_;
}

size_t Path::size() const
{
    return waypoints_.size();
}

bool Path::empty() const
{
    return waypoints_.empty();
}

const Path::PathContainer& Path::waypoints() const
{
    return waypoints_;
}

} // namespace path

} // namespace cogip

/// @}
