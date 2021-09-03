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

// circle
Circle::Circle(const cogip_defs::Coords &center, double radius, double angle)
    : Obstacle(center, radius, angle)
{
}

bool Circle::is_point_inside(const cogip_defs::Coords &p) const {
    double d = center_.distance(p);

    if (d * d > radius_ * radius_) {
        return false;
    }
    else {
        return true;
    }
}

bool Circle::is_segment_crossing(const cogip_defs::Coords &a, const cogip_defs::Coords &b) const
{
    const cogip_defs::Coords &c = center_;

    if (!is_line_crossing_circle(a, b)) {
        return false;
    }

    if (is_point_inside(a)) {
        return true;
    }
    if (is_point_inside(b)) {
        return true;
    }

    cogip_defs::Coords vect_ab(b.x() - a.x(), b.y() - a.y());
    cogip_defs::Coords vect_ac(c.x() - a.x(), c.y() - a.y());
    cogip_defs::Coords vect_bc(c.x() - b.x(), c.y() - b.y());

    double scal1 = vect_ab.x() * vect_ac.x() + vect_ab.y() * vect_ac.y();
    double scal2 = (-vect_ab.x()) * vect_bc.x() + (-vect_ab.y()) * vect_bc.y();
    if (scal1 >= 0 && scal2 >= 0) {
        return true;
    }

    return false;
}

cogip_defs::Coords Circle::nearest_point(const cogip_defs::Coords &p) const
{
    cogip_defs::Coords vect(
        p.x() - center_.x(),
        p.y() - center_.y()
    );

    double vect_norm = sqrt(vect.x() * vect.x() + vect.y() * vect.y());

    return cogip_defs::Coords(
        center_.x() + (vect.x() / vect_norm) * radius_,
        center_.y() + (vect.y() / vect_norm) * radius_
    );
}

void Circle::print_json(cogip::tracefd::File &out) const
{
    out.printf(
        "{\"x\":%.3lf,\"y\":%.3lf,\"radius\":%.3lf}",
        center_.x(),
        center_.y(),
        radius_
        );
}

bool Circle::is_line_crossing_circle(const cogip_defs::Coords &a, const cogip_defs::Coords &b) const
{
    const cogip_defs::Coords &c = center_;

    cogip_defs::Coords vect_ab(b.x() - a.x(), b.y() - a.y());
    cogip_defs::Coords vect_ac(c.x() - a.x(), c.y() - a.y());

    /* Norm of vector V */
    double numerator = vect_ab.x() * vect_ac.y() - vect_ab.y() * vect_ac.x();
    if (numerator < 0) {
        numerator = -numerator;
    }

    /* Norm of vector U */
    double denominator = sqrt(vect_ab.x() * vect_ab.x() + vect_ab.y() * vect_ab.y());

    /* Norm of vector CI where I is the nearest point of the line */
    double ci = numerator / denominator;

    /* If CI norm is less or equal to the circle radius, point I is inside the
     * circle */
    if (ci < radius_) {
        return true;
    }
    else {
        return false;
    }
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
