#include "app_pose.hpp"

#include <iostream>

namespace cogip {

namespace app {

std::ostream & operator << (std::ostream &out, const cogip::app::Pose *pose)
{
    out << "Pose[" << (void *)pose << "](" << pose->coords().x() << "," << pose->coords().y() << "," << pose->O() << ")";
    return out;
}

std::ostream & operator << (std::ostream &out, const Poses &poses)
{
    out << "[";
    for (auto *pose: poses) {
        out << pose << ",";
    }
    out << "]";
    return out;
}

} // namespace app

} // namespace cogip
