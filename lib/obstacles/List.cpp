// Project includes
#include "obstacles/List.hpp"
#include "obstacles/obstacles.hpp"
#include "PB_Obstacle.hpp"
#include "utils.hpp"

namespace cogip {

namespace obstacles {

List::List()
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
    etl::vector<Obstacle *, OBSTACLES_MAX_NUMBER>::clear();
}

void List::print_json(void) const
{
    size_t i = 0;
    for (auto obs: *this) {
        if (! obs->enabled()) {
            continue;
        }
        if (i++ > 0) {
            COGIP_DEBUG_COUT(",");
        }

        obs->print_json();
    }
}

void List::pb_copy(PB_Message &message) const {
    for (auto obs: *this) {
        if (! obs->enabled()) {
            continue;
        }
        obs->pb_copy(message.get(message.get_length()));
    }
}

size_t List::enabled_obstacles() const
{
    size_t count = 0;
    for (auto obs: *this) {
        if (obs->enabled()) {
            count++;
        }
    }
    return count;
}

} // namespace obstacles

} // namespace cogip
