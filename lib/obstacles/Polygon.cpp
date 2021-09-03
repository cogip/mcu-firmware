#include "obstacles/Polygon.hpp"

namespace cogip {

namespace obstacles {


/// @brief Check if a segment defined by two points A,B is crossing line
///        defined by two other points C,D.
/// @param[in]   a           point A
/// @param[in]   b           point B
/// @param[in]   c           point C
/// @param[in]   d           point D
/// @return                  true if [AB] crosses (CD), false otherwise
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


/// @brief Check if a segment defined by two points A,B is crossing segment
///        defined by two other points C,D.
/// @param[in]   a           point A
/// @param[in]   b           point B
/// @param[in]   c           point C
/// @param[in]   d           point D
/// @return                  true if [AB] crosses [CD], false otherwise
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
    // Check if that segment crosses a polygon
    for (size_t i = 0; i < size(); i++) {
        const cogip_defs::Coords &p_next = ((i + 1 == size()) ? at(0) : at(i + 1));

        if (_is_segment_crossing_segment(a, b, at(i), p_next)) {
            return true;
        }

        // If A and B are vertices of the polygon
        int index = point_index(a);
        int index2 = point_index(b);
        // Consecutive vertices: no collision
        if (((index == 0) && (index2 == ((int)size() - 1)))
            || ((index2 == 0) && (index == ((int)size() - 1)))) {
            continue;
        }
        // If not consecutive vertices: collision
        if ((index >= 0) && (index2 >= 0) && (abs(index - index2) != 1)) {
            return true;
        }

        // If polygon vertice is on segment [AB]
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

} // namespace obstacles

} // namespace cogip
