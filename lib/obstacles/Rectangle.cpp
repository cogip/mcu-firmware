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

void Rectangle::pb_copy(PB_Message &message) const
{
    Obstacle::pb_copy(message);

    PB_Rectangle &rectangle = message.mutable_rectangle();
    rectangle.set_x(center_.x());
    rectangle.set_y(center_.y());
    rectangle.set_angle(angle_);
    rectangle.set_length_x(length_x_);
    rectangle.set_length_y(length_y_);
}

} // namespace obstacles

} // namespace cogip
