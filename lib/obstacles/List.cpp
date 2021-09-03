#include "obstacles/List.hpp"
#include "obstacles_private.hpp"

namespace cogip {

namespace obstacles {

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
    std::vector<Obstacle *>::clear();
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

} // namespace obstacles

} // namespace cogip
