
// System includes
#include "cmath"

// Project includes
#include "obstacles/Rectangle.hpp"
#include "trigonometry.h"
#include "utils.hpp"

namespace cogip {

namespace obstacles {

Rectangle::Rectangle(
    const cogip_defs::Coords &center, double angle,
    double length_x, double length_y)
    : angle_(angle), length_x_(length_x), length_y_(length_y)
{
    center_ = center;
    radius_ = sqrt(length_x_ * length_x_ + length_y_ * length_y_) / 2;
    push_back(cogip_defs::Coords(
        center.x() - (length_x_ / 2) * cos(DEG2RAD(angle)) + (length_y_ / 2) * sin(DEG2RAD(angle)),
        center.y() - (length_x_ / 2) * sin(DEG2RAD(angle)) - (length_y_ / 2) * cos(DEG2RAD(angle))
    ));
    push_back(cogip_defs::Coords(
        center.x() + (length_x_ / 2) * cos(DEG2RAD(angle)) + (length_y_ / 2) * sin(DEG2RAD(angle)),
        center.y() + (length_x_ / 2) * sin(DEG2RAD(angle)) - (length_y_ / 2) * cos(DEG2RAD(angle))
    ));
    push_back(cogip_defs::Coords(
        center.x() + (length_x_ / 2) * cos(DEG2RAD(angle)) - (length_y_ / 2) * sin(DEG2RAD(angle)),
        center.y() + (length_x_ / 2) * sin(DEG2RAD(angle)) + (length_y_ / 2) * cos(DEG2RAD(angle))
    ));
    push_back(cogip_defs::Coords(
        center.x() - (length_x_ / 2) * cos(DEG2RAD(angle)) - (length_y_ / 2) * sin(DEG2RAD(angle)),
        center.y() - (length_x_ / 2) * sin(DEG2RAD(angle)) + (length_y_ / 2) * cos(DEG2RAD(angle))
    ));

    for (const auto & point: *this) {
        bounding_box_.push_back(point);
    }
}

} // namespace obstacles

} // namespace cogip
