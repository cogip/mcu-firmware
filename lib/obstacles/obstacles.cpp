#include "obstacles/obstacles.hpp"
#include "obstacles/List.hpp"

/* System includes */
#include <cmath>
#include <cstring>

/* Project includes */
#include "cogip_defs/Polygon.hpp"
#include "trigonometry.h"
#include "utils.hpp"

#define ENABLE_DEBUG (0)
#include "debug.h"

namespace cogip {

namespace obstacles {

etl::set<List const *, OBSTACLE_MAX_LISTS> all_obstacles;

// Global functions
void print_all_json(void)
{
    size_t nb_obstacles = 0;

    COGIP_DEBUG_COUT("[");

    for (auto l: all_obstacles) {
        if (nb_obstacles > 0 && l->enabled_obstacles() > 0) {
            COGIP_DEBUG_COUT(",");
        }

        l->print_json();
        nb_obstacles += l->enabled_obstacles();
    }
    COGIP_DEBUG_COUT("]");
}

void pb_copy(cogip::obstacles::List::PB_Message &message)
{
    for (auto l: all_obstacles) {
        l->pb_copy(message);
    }
}


bool is_point_in_obstacles(const cogip_defs::Coords &p, const Obstacle *filter)
{
    for (auto obstacles: all_obstacles) {
        for (auto obstacle: *obstacles) {
            if (! obstacle->enabled()) {
                continue;
            }
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
