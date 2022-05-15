#include "obstacles/Obstacle.hpp"

// System includes
#include <cmath>

// Project includes
#include "trigonometry.h"

namespace cogip {

namespace obstacles {

Obstacle::Obstacle(
    const cogip_defs::Coords &center, double radius)
    : center_(center), radius_(radius), enabled_(true)
{
}

void Obstacle::update_bounding_box()
{
    if (radius_) {
        double radius = radius_ * (1 + OBSTACLE_BOUNDING_BOX_MARGIN);

        bounding_box_.clear();

        for (uint8_t i = 0; i < OBSTACLE_BOUNDING_BOX_VERTICES; i++) {
            bounding_box_.push_back(cogip_defs::Coords(
                center_.x() + radius * cos(((double)i * 2 * M_PI) / (double)OBSTACLE_BOUNDING_BOX_VERTICES),
                center_.y() + radius * sin(((double)i * 2 * M_PI) / (double)OBSTACLE_BOUNDING_BOX_VERTICES)));
        }
    }
}

void Obstacle::pb_copy(PB_Message &message) const
{
    auto &bb = message.mutable_bounding_box();
    for (const auto &point: bounding_box_) {
        auto &bb_point = bb.get(bb.get_length());
        point.pb_copy(bb_point);
    }
}

} // namespace obstacles

} // namespace cogip
