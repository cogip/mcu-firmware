#include "obstacles/Rectangle.hpp"

// System includes
#include "cmath"

// Project includes
#include "trigonometry.h"

namespace cogip {

namespace obstacles {

Rectangle::Rectangle(
    const cogip_defs::Coords &center, double angle,
    double length_x, double length_y)
    : angle_(angle), length_x_(length_x), length_y_(length_y)
{
    center_ = center;
    radius_ = sqrt(length_x * length_x + length_y * length_y) / 2;
    push_back(cogip_defs::Coords(
        (center.x() - length_x / 2) * cos(DEG2RAD(angle)) -
            (center.y() - length_y / 2) * sin(DEG2RAD(angle)),
        (center.x() - length_x / 2) * sin(DEG2RAD(angle)) +
            (center.y() - length_y / 2) * cos(DEG2RAD(angle))
    ));
    push_back(cogip_defs::Coords(
        (center.x() - length_x / 2) * cos(DEG2RAD(angle)) -
            (center.y() + length_y / 2) * sin(DEG2RAD(angle)),
        (center.x() - length_x / 2) * sin(DEG2RAD(angle)) +
            (center.y() + length_y / 2) * cos(DEG2RAD(angle))
    ));
    push_back(cogip_defs::Coords(
        (center.x() + length_x / 2) * cos(DEG2RAD(angle)) -
            (center.y() + length_y / 2) * sin(DEG2RAD(angle)),
        (center.x() + length_x / 2) * sin(DEG2RAD(angle)) +
            (center.y() + length_y / 2) * cos(DEG2RAD(angle))
    ));
    push_back(cogip_defs::Coords(
        (center.x() + length_x / 2) * cos(DEG2RAD(angle)) -
            (center.y() - length_y / 2) * sin(DEG2RAD(angle)),
        (center.x() + length_x / 2) * sin(DEG2RAD(angle)) +
            (center.y() - length_y / 2) * cos(DEG2RAD(angle))
    ));
}

void Rectangle::print_json(cogip::tracefd::File &out) const
{
    out.printf(
        "{\"x\":%.3lf,\"y\":%.3lf,\"angle\":%.3lf,\"length_x\":%.3lf,\"length_y\":%.3lf}",
        center_.x(),
        center_.y(),
        angle_,
        length_x_,
        length_y_
        );
}

} // namespace obstacles

} // namespace cogip
