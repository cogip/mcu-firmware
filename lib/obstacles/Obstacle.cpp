#include "obstacles/Obstacle.hpp"

// System includes
#include <cmath>

// Project includes
#include "trigonometry.h"

namespace cogip {

namespace obstacles {

Obstacle::Obstacle(
    const cogip_defs::Coords &center, double radius)
    : center_(center), radius_(radius)
{
}

cogip_defs::Polygon Obstacle::bounding_box(uint8_t nb_vertices, double radius_margin) const
{
    double radius = radius_ * (1 + radius_margin);

    cogip_defs::Polygon bb;

    uint8_t nb_points = (nb_vertices < 4) ? 4 : nb_vertices;

    for (uint8_t i = 0; i < nb_points; i++) {
        bb.push_back(cogip_defs::Coords(
            center_.x() + radius * cos(((double)i * 2 * M_PI) / (double)nb_points),
            center_.y() + radius * sin(((double)i * 2 * M_PI) / (double)nb_points)));
    }

    return bb;
}

} // namespace obstacles

} // namespace cogip
