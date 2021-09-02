#include "obstacles.hpp"

/* System includes */
#include <cmath>
#include <cstring>
#include <set>

/* Project includes */
#include "trigonometry.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

namespace cogip {

namespace obstacles {

static std::set<list const *> all_obstacles;

/**
 * @brief Compute the distance between two points A and B.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 *
 * @return                  distance between A and B
 */
static double _distance_points(const coords_t *a, const coords_t *b)
{
    return sqrt((b->x - a->x) * (b->x - a->x)
                + (b->y - a->y) * (b->y - a->y));
}

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
static bool _is_segment_crossing_line(const coords_t *a, const coords_t *b, const coords_t *c, const coords_t *d)
{
    vector_t ac, ad, ab;
    double det = 0;

    ab.x = b->x - a->x;
    ab.y = b->y - a->y;
    ad.x = d->x - a->x;
    ad.y = d->y - a->y;
    ac.x = c->x - a->x;
    ac.y = c->y - a->y;

    det = (ab.x * ad.y - ab.y * ad.x) * (ab.x * ac.y - ab.y * ac.x);

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
static bool _is_segment_crossing_segment(const coords_t *a, const coords_t *b,
                                            const coords_t *c, const coords_t *d)
{
    if (!_is_segment_crossing_line(a, b, c, d)) {
        return false;
    }
    if (!_is_segment_crossing_line(c, d, a, b)) {
        return false;
    }
    return true;
}

/**
 * @brief Check if a point C is placed on a segment defined by two points A,B.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 * @param[in]   c           point C
 *
 * @return                  true if C is on [AB], false otherwise
 */
static bool _is_point_on_segment(const coords_t *a, const coords_t *b, const coords_t *c)
{
    bool res = false;

    if ((b->x - a->x) / (b->y - a->y) == (b->x - c->x) / (b->y - c->y)) {
        if (a->x < b->x) {
            if ((c->x < b->x) && (c->x > a->x)) {
                res = true;
            }
        }
        else {
            if ((c->x < a->x) && (c->x > b->x)) {
                res = true;
            }
        }
    }
    return res;
}

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
    : length_x_(length_x), length_y_(length_y)
{
    center_ = center;
    radius_ = sqrt(length_x * length_x + length_y * length_y) / 2;
    angle_ = angle;
    polygon_.count = 4;
    polygon_.points[0] = (coords_t){
        .x = (center.x - length_x / 2) * cos(DEG2RAD(angle)) -
                (center.y - length_y / 2) * sin(DEG2RAD(angle)),
        .y = (center.x - length_x / 2) * sin(DEG2RAD(angle)) +
                (center.y - length_y / 2) * cos(DEG2RAD(angle)),
    };
    polygon_.points[1] = (coords_t){
        .x = (center.x - length_x / 2) * cos(DEG2RAD(angle)) -
                (center.y + length_y / 2) * sin(DEG2RAD(angle)),
        .y = (center.x - length_x / 2) * sin(DEG2RAD(angle)) +
                (center.y + length_y / 2) * cos(DEG2RAD(angle)),
    };
    polygon_.points[2] = (coords_t){
        .x = (center.x + length_x / 2) * cos(DEG2RAD(angle)) -
                (center.y + length_y / 2) * sin(DEG2RAD(angle)),
        .y = (center.x + length_x / 2) * sin(DEG2RAD(angle)) +
                (center.y + length_y / 2) * cos(DEG2RAD(angle)),
    };
    polygon_.points[3] = (coords_t){
        .x = (center.x + length_x / 2) * cos(DEG2RAD(angle)) -
                (center.y - length_y / 2) * sin(DEG2RAD(angle)),
        .y = (center.x + length_x / 2) * sin(DEG2RAD(angle)) +
                (center.y - length_y / 2) * cos(DEG2RAD(angle)),
    };
}

void rectangle::print_json(cogip::tracefd::File &out) const
{
    out.printf(
        "{\"x\":%.3lf,\"y\":%.3lf,\"angle\":%.3lf,\"length_x\":%.3lf,\"length_y\":%.3lf}",
        center_.x,
        center_.y,
        angle_,
        length_x_,
        length_y_
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
    double d = _distance_points(&circle_.center, &p);

    if (d * d > circle_.radius * circle_.radius) {
        return false;
    }
    else {
        return true;
    }
}

bool circle::is_segment_crossing(const coords_t &a, const coords_t &b) const
{
    const coords_t &c = circle_.center;

    if (!is_line_crossing_circle(a, b)) {
        return false;
    }

    if (is_point_inside(a)) {
        return true;
    }
    if (is_point_inside(b)) {
        return true;
    }

    vector_t vect_ab, vect_ac, vect_bc;
    vect_ab.x = b.x - a.x;
    vect_ab.y = b.y - a.y;
    vect_ac.x = c.x - a.x;
    vect_ac.y = c.y - a.y;
    vect_bc.x = c.x - b.x;
    vect_bc.y = c.y - b.y;

    double scal1 = vect_ab.x * vect_ac.x + vect_ab.y * vect_ac.y;
    double scal2 = (-vect_ab.x) * vect_bc.x + (-vect_ab.y) * vect_bc.y;
    if (scal1 >= 0 && scal2 >= 0) {
        return true;
    }

    return false;
}

coords_t circle::nearest_point(const coords_t &p) const
{
    vector_t vect = {
        .x = p.x - circle_.center.x,
        .y = p.y - circle_.center.y,
    };

    double vect_norm = sqrt(vect.x * vect.x + vect.y * vect.y);

    return (coords_t) {
               .x = circle_.center.x + (vect.x / vect_norm) * circle_.radius,
               .y = circle_.center.y + (vect.y / vect_norm) * circle_.radius,
    };
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

bool circle::is_line_crossing_circle(const coords_t &a, const coords_t &b) const
{
    const coords_t &c = circle_.center;

    vector_t vect_ab;

    vect_ab.x = b.x - a.x;
    vect_ab.y = b.y - a.y;

    vector_t vect_ac;
    vect_ac.x = c.x - a.x;
    vect_ac.y = c.y - a.y;

    /* Norm of vector V */
    double numerator = vect_ab.x * vect_ac.y - vect_ab.y * vect_ac.x;
    if (numerator < 0) {
        numerator = -numerator;
    }

    /* Norm of vector U */
    double denominator = sqrt(vect_ab.x * vect_ab.x + vect_ab.y * vect_ab.y);

    /* Norm of vector CI where I is the nearest point of the line */
    double ci = numerator / denominator;

    /* If CI norm is less or equal to the circle radius, point I is inside the
     * circle */
    if (ci < circle_.radius) {
        return true;
    }
    else {
        return false;
    }
}

// polygon
polygon::polygon(const std::list<coords_t> *points)
    : obstacle({0, 0}, 0, 0)
{
    polygon_.count = 0;
    if (points) {
        for (auto point: *points) {
            polygon_.points[polygon_.count++] = point;
        }
    }
}

bool polygon::is_point_inside(const coords_t &p) const {
    for (uint8_t i = 0; i < polygon_.count; i++) {
        coords_t a = polygon_.points[i];
        coords_t b = (i == (polygon_.count - 1) ? polygon_.points[0] : polygon_.points[i + 1]);
        vector_t ab, ap;

        ab.x = b.x - a.x;
        ab.y = b.y - a.y;
        ap.x = p.x - a.x;
        ap.y = p.y - a.y;

        if (ab.x * ap.y - ab.y * ap.x <= 0) {
            return false;
        }
    }
    return true;
}

static int8_t _get_point_index_in_polygon(const polygon_t *polygon, const coords_t *p)
{
    for (uint8_t i = 0; i < polygon->count; i++) {
        if ((polygon->points[i].x == p->x) && (polygon->points[i].y == p->y)) {
            return i;
        }
    }
    return -1;
}

bool polygon::is_segment_crossing(const coords_t &a, const coords_t &b) const
{
    /* Check if that segment crosses a polygon */
    for (int i = 0; i < polygon_.count; i++) {
        coords_t p_next = ((i + 1 == polygon_.count) ? polygon_.points[0] : polygon_.points[i + 1]);

        if (_is_segment_crossing_segment(&a, &b, &polygon_.points[i], &p_next)) {
            return true;
        }

        /* If A and B are vertices of the polygon */
        int8_t index = _get_point_index_in_polygon(&polygon_, &a);
        int8_t index2 = _get_point_index_in_polygon(&polygon_, &b);
        /* Consecutive vertices: no collision */
        if (((index == 0) && (index2 == (polygon_.count - 1)))
            || ((index2 == 0) && (index == (polygon_.count - 1)))) {
            continue;
        }
        /* If not consecutive vertices: collision */
        if ((index >= 0) && (index2 >= 0) && (abs(index - index2) != 1)) {
            return true;
        }

        /* If polygon vertice is on segment [AB] */
        if (_is_point_on_segment(&a, &b, &polygon_.points[i])) {
            return true;
        }
    }

    return false;
}

coords_t polygon::nearest_point(const coords_t &p) const
{
    double min = UINT32_MAX;
    coords_t tmp = p;

    for (int j = 0; j < polygon_.count; j++) {
        double distance = _distance_points(&p, &polygon_.points[j]);
        if (distance < min) {
            min = distance;
            tmp = polygon_.points[j];
        }
    }

    return tmp;
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
