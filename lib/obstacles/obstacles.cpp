#include "obstacles/obstacles.hpp"
#include "obstacles/List.hpp"
#include "obstacles_private.hpp"

/* System includes */
#include <cmath>
#include <cstring>
#include <set>

/* Project includes */
#include "trigonometry.h"
#include "cogip_defs/Polygon.hpp"

#define ENABLE_DEBUG (0)
#include "debug.h"

namespace cogip {

namespace obstacles {

std::set<List const *> all_obstacles;

// Obstacle
Obstacle::Obstacle(
    const cogip_defs::Coords &center, double radius, double angle)
    : center_(center), radius_(radius), angle_(angle)
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

// Global functions
void print_all_json(cogip::tracefd::File &out)
{
    size_t nb_obstacles = 0;

    out.printf("[");

    for (auto l: all_obstacles) {
        if (nb_obstacles > 0 && l->size() > 0) {
            out.printf(",");
        }

        l->print_json(out);
        nb_obstacles += l->size();
    }
    out.printf("]");
}

bool is_point_in_obstacles(const cogip_defs::Coords &p, const Obstacle *filter)
{
    for (auto obstacles: all_obstacles) {
        for (auto obstacle: *obstacles) {
            if (filter == obstacle) {
                continue;
            }
            if (obstacle->is_point_inside(p)) {
                return true;
            }
        }
    }
    return false;
}

} // namespace obstacles

} // namespace cogip
