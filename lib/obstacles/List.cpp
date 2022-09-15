// Project includes
#include "obstacles/List.hpp"
#include "obstacles/obstacles.hpp"
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
