#include "obstacles.hpp"

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

static std::set<List const *> all_obstacles;

/**
 * @brief Check if a segment defined by two points A,B is crossing line
 * defined by two other points C,D.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 * @param[in]   c           point C
 * @param[in]   d           point D
 *
 * @return                  true if [AB] crosses (CD), false otherwise
 */
static bool _is_segment_crossing_line(
    const cogip_defs::Coords &a, const cogip_defs::Coords &b,
    const cogip_defs::Coords &c, const cogip_defs::Coords &d)
{
    cogip_defs::Coords ab(b.x() - a.x(), b.y() - a.y());
    cogip_defs::Coords ac(c.x() - a.x(), c.y() - a.y());
    cogip_defs::Coords ad(d.x() - a.x(), d.y() - a.y());

    double det = (ab.x() * ad.y() - ab.y() * ad.x()) * (ab.x() * ac.y() - ab.y() * ac.x());

    return (det < 0);
}

/**
 * @brief Check if a segment defined by two points A,B is crossing segment
 * defined by two other points C,D.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 * @param[in]   c           point C
 * @param[in]   d           point D
 *
 * @return                  true if [AB] crosses [CD], false otherwise
 */
static bool _is_segment_crossing_segment(
    const cogip_defs::Coords &a, const cogip_defs::Coords &b,
    const cogip_defs::Coords &c, const cogip_defs::Coords &d)
{
    if (!_is_segment_crossing_line(a, b, c, d)) {
        return false;
    }
    if (!_is_segment_crossing_line(c, d, a, b)) {
        return false;
    }
    return true;
}

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

// Rectangle
Rectangle::Rectangle(
    const cogip_defs::Coords &center, double angle,
    double length_x, double length_y)
    : length_x_(length_x), length_y_(length_y)
{
    center_ = center;
    radius_ = sqrt(length_x * length_x + length_y * length_y) / 2;
    angle_ = angle;
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

// Polygon
Polygon::Polygon(const std::list<cogip_defs::Coords> *points)
    : Obstacle(cogip_defs::Coords(0.0, 0.0), 0, 0)
{
    if (points) {
        for (const auto &point: *points) {
            push_back(point);
        }
    }
}

bool Polygon::is_point_inside(const cogip_defs::Coords &p) const {
    for (size_t i = 0; i < size(); i++) {
        cogip_defs::Coords a = at(i);
        cogip_defs::Coords b = (i == (size() - 1) ? at(0) : at(i + 1));
        cogip_defs::Coords ab(b.x() - a.x(), b.y() - a.y());
        cogip_defs::Coords ap(p.x() - a.x(), p.y() - a.y());

        if (ab.x() * ap.y() - ab.y() * ap.x() <= 0) {
            return false;
        }
    }
    return true;
}

bool Polygon::is_segment_crossing(const cogip_defs::Coords &a, const cogip_defs::Coords &b) const
{
    /* Check if that segment crosses a polygon */
    for (size_t i = 0; i < size(); i++) {
        const cogip_defs::Coords &p_next = ((i + 1 == size()) ? at(0) : at(i + 1));

        if (_is_segment_crossing_segment(a, b, at(i), p_next)) {
            return true;
        }

        /* If A and B are vertices of the polygon */
        int index = point_index(a);
        int index2 = point_index(b);
        /* Consecutive vertices: no collision */
        if (((index == 0) && (index2 == ((int)size() - 1)))
            || ((index2 == 0) && (index == ((int)size() - 1)))) {
            continue;
        }
        /* If not consecutive vertices: collision */
        if ((index >= 0) && (index2 >= 0) && (abs(index - index2) != 1)) {
            return true;
        }

        /* If polygon vertice is on segment [AB] */
        if (at(i).on_segment(a, b)) {
            return true;
        }
    }

    return false;
}

cogip_defs::Coords Polygon::nearest_point(const cogip_defs::Coords &p) const
{
    double min = UINT32_MAX;
    cogip_defs::Coords tmp = p;

    for (const auto &point: *this) {
        double distance = p.distance(point);
        if (distance < min) {
            min = distance;
            tmp = point;
        }
    }

    return tmp;
}

void Polygon::print_json(cogip::tracefd::File &out) const
{
    out.printf(
        "{\"x\":%.3lf,\"y\":%.3lf,\"angle\":%.3lf,\"points\":[",
        center_.x(),
        center_.y(),
        angle_
        );
    for (auto it = begin(); it != end(); it++) {
        if (it != begin()) {
            out.printf(",");
        }
        out.printf(
            "{\"x\":%.3lf,\"y\":%.3lf}",
            it->x(),
            it->y()
            );
    }
    out.printf("]}");
}

// List
List::List(
    uint32_t default_circle_radius,
    uint32_t default_rectangle_width,
    uint32_t min_distance,
    uint32_t max_distance)
    : default_circle_radius_(default_circle_radius), default_rectangle_width_(default_rectangle_width),
      min_distance_(min_distance), max_distance_(max_distance)
{
    all_obstacles.insert(this);
}

List::~List()
{
    all_obstacles.erase(this);
}

void List::clear()
{
    for (auto obs: *this) {
        delete obs;
    }
    std::list<Obstacle *>::clear();
}

void List::print_json(cogip::tracefd::File &out) const
{
    size_t i = 0;
    for (auto obs: *this) {
        if (i++ > 0) {
            out.printf(",");
        }

        obs->print_json(out);
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
