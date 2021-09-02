#include "obstacles.hpp"

/* System includes */
#include <cmath>
#include <cstring>
#include <set>

/* Project includes */
#include "collisions.hpp"
#include "trigonometry.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

namespace cogip {

namespace obstacles {

static std::set<list const *> all_obstacles;

// obstacle
obstacle::obstacle(
    coords_t center, double radius, double angle)
    : center_(center), radius_(radius), angle_(angle)
{
}

polygon_t obstacle::bounding_box(uint8_t nb_vertices, double radius_margin) const
{
    double radius = radius_ * (1 + radius_margin);

    polygon_t bb;

    bb.count = (nb_vertices < 4) ? 4 : nb_vertices;

    for (uint8_t i = 0; i < bb.count; i++) {
        bb.points[i].x = center_.x +
                         radius * cos(((double)i * 2 * M_PI) / (double)bb.count);
        bb.points[i].y = center_.y +
                         radius * sin(((double)i * 2 * M_PI) / (double)bb.count);
    }

    return bb;
}

// rectangle
rectangle::rectangle(
    coords_t center, double angle,
    double length_x, double length_y)
    : obstacle(center, 0, angle)
{
    radius_ = sqrt(length_x * length_x + length_y * length_y) / 2;
    rectangle_ = {
        .length_x = length_x,
        .length_y = length_y,
        .points = {
            (coords_t){
                .x = (center.x - length_x / 2) * cos(DEG2RAD(angle)) -
                        (center.y - length_y / 2) * sin(DEG2RAD(angle)),
                .y = (center.x - length_x / 2) * sin(DEG2RAD(angle)) +
                        (center.y - length_y / 2) * cos(DEG2RAD(angle)),
            },
            (coords_t){
                .x = (center.x - length_x / 2) * cos(DEG2RAD(angle)) -
                        (center.y + length_y / 2) * sin(DEG2RAD(angle)),
                .y = (center.x - length_x / 2) * sin(DEG2RAD(angle)) +
                        (center.y + length_y / 2) * cos(DEG2RAD(angle)),
            },
            (coords_t){
                .x = (center.x + length_x / 2) * cos(DEG2RAD(angle)) -
                        (center.y + length_y / 2) * sin(DEG2RAD(angle)),
                .y = (center.x + length_x / 2) * sin(DEG2RAD(angle)) +
                        (center.y + length_y / 2) * cos(DEG2RAD(angle)),
            },
            (coords_t){
                .x = (center.x + length_x / 2) * cos(DEG2RAD(angle)) -
                        (center.y - length_y / 2) * sin(DEG2RAD(angle)),
                .y = (center.x + length_x / 2) * sin(DEG2RAD(angle)) +
                        (center.y - length_y / 2) * cos(DEG2RAD(angle)),
            },
        }
    };
}

bool rectangle::is_point_inside(const coords_t &p) const {
    polygon_t polygon_tmp;
    polygon_tmp.count = 4;
    memcpy(polygon_tmp.points, rectangle_.points,
            polygon_tmp.count * sizeof(coords_t));
    return collisions_is_point_in_polygon(&polygon_tmp, &p);
}

bool rectangle::is_segment_crossing(const coords_t &a, const coords_t &b) const
{
    polygon_t polygon_tmp;
    polygon_tmp.count = 4;
    memcpy(polygon_tmp.points, rectangle_.points,
            polygon_tmp.count * sizeof(coords_t));
    return collisions_is_segment_crossing_polygon(&a, &b, &polygon_tmp);
}

coords_t rectangle::nearest_point(const coords_t &p) const
{
    polygon_t polygon_tmp;
    polygon_tmp.count = 4;
    memcpy(polygon_tmp.points, rectangle_.points,
            polygon_tmp.count * sizeof(coords_t));
    return collisions_find_nearest_point_in_polygon(&polygon_tmp, &p);
}

void rectangle::print_json(cogip::tracefd::File &out) const
{
    out.printf(
        "{\"x\":%.3lf,\"y\":%.3lf,\"angle\":%.3lf,\"length_x\":%.3lf,\"length_y\":%.3lf}",
        center_.x,
        center_.y,
        angle_,
        rectangle_.length_x,
        rectangle_.length_y
        );
}

// circle
circle::circle(coords_t center, double radius, double angle)
    : obstacle(center, radius, angle)
{
    circle_.center = center;
    circle_.radius = radius;
}

bool circle::is_point_inside(const coords_t &p) const {
    return collisions_is_point_in_circle(&circle_, &p);
}

bool circle::is_segment_crossing(const coords_t &a, const coords_t &b) const
{
    return collisions_is_segment_crossing_circle(&a, &b, &circle_);
}

coords_t circle::nearest_point(const coords_t &p) const
{
    return collisions_find_nearest_point_in_circle(&circle_, &p);
}

void circle::print_json(cogip::tracefd::File &out) const
{
    out.printf(
        "{\"x\":%.3lf,\"y\":%.3lf,\"radius\":%.3lf}",
        center_.x,
        center_.y,
        circle_.radius
        );
}

// polygon
polygon::polygon(const std::list<coords_t> *points)
    : obstacle({0, 0}, 0, 0)
{
    polygon_.count = 0;
    for (auto point: *points) {
        polygon_.points[polygon_.count++] = point;
    }
}

bool polygon::is_point_inside(const coords_t &p) const {
    return collisions_is_point_in_polygon(&polygon_, &p);
}

bool polygon::is_segment_crossing(const coords_t &a, const coords_t &b) const
{
    return collisions_is_segment_crossing_polygon(&a, &b, &polygon_);
}

coords_t polygon::nearest_point(const coords_t &p) const
{
    return collisions_find_nearest_point_in_polygon(&polygon_, &p);
}

void polygon::print_json(cogip::tracefd::File &out) const
{
    out.printf(
        "{\"x\":%.3lf,\"y\":%.3lf,\"angle\":%.3lf,\"points\":[",
        center_.x,
        center_.y,
        angle_
        );
    for (uint8_t i = 0; i < polygon_.count; i++) {
        if (i > 0) {
            out.printf(",");
        }
        out.printf(
            "{\"x\":%.3lf,\"y\":%.3lf}",
            polygon_.points[i].x,
            polygon_.points[i].y
            );
    }
    out.printf("]}");
}

// list
list::list(
    uint32_t default_circle_radius,
    uint32_t default_rectangle_width,
    uint32_t min_distance,
    uint32_t max_distance)
    : default_circle_radius_(default_circle_radius), default_rectangle_width_(default_rectangle_width),
      min_distance_(min_distance), max_distance_(max_distance)
{
    all_obstacles.insert(this);
}

list::~list()
{
    all_obstacles.erase(this);
}

void list::clear()
{
    for (auto obs: *this) {
        delete obs;
    }
    std::list<obstacle *>::clear();
}

void list::print_json(cogip::tracefd::File &out) const
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

bool is_point_in_obstacles(const coords_t &p, const obstacle *filter)
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
