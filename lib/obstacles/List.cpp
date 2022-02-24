#include "obstacles/List.hpp"
#include "obstacles/obstacles.hpp"

#include "PB_Obstacle.hpp"

namespace cogip {

namespace obstacles {

List::List(
    uint32_t min_distance,
    uint32_t max_distance)
    : min_distance_(min_distance), max_distance_(max_distance)
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

void List::pb_copy(PB_Message &message) const {
    for (auto obs: *this) {
        obs->pb_copy(message.get(message.get_length()));
    }
}

} // namespace obstacles

} // namespace cogip
