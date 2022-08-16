#pragma once

#include "path/Pose.hpp"

#include <iostream>
#include <functional>

#include "etl/vector.h"

namespace cogip {

namespace app {

class Pose : public path::Pose {
public:
    /// Constuctor.
    Pose(
        double x=0.0,                  ///< [in] X coordinate
        double y=0.0,                  ///< [in] Y coodinate
        double O=0.0,                  ///< [in] 0-orientation
        double max_speed_linear=0.0,   ///< [in] max speed
        double max_speed_angular=0.0,  ///< [in] max speed
        bool allow_reverse=true        ///< [in] reverse mode
        ) : path::Pose(x, y, O, max_speed_linear, max_speed_angular, allow_reverse) {};
    virtual ~Pose() {};

    void act() const override { after_pose(); };

    void before_pose() const { before_pose_(); };
    void after_pose() const { after_pose_(); };

    void set_before_pose(std::function<void()> f) { before_pose_ = f; };
    void set_after_pose(std::function<void()> f) { after_pose_ = f; };

private:
    std::function<void()> before_pose_ = []() {};
    std::function<void()> after_pose_ = []() {};
};

using Poses = etl::vector<Pose *, PATH_MAX_POSES>;

std::ostream & operator << (std::ostream &out, const Pose *pose);
std::ostream & operator << (std::ostream &out, const Poses &poses);

} // namespace app

} // namespace cogip
