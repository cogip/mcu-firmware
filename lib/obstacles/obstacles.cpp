#include "obstacles/obstacles.hpp"
#include "obstacles/List.hpp"

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

// Global functions
void print_all_json(cogip::tracefd::File &out)
{
    size_t nb_obstacles = 0;

    out.printf("[");

    for (auto l: all_obstacles) {
        if (nb_obstacles > 0 && l->enabled_obstacles() > 0) {
            out.printf(",");
        }

        l->print_json(out);
        nb_obstacles += l->enabled_obstacles();
    }
    out.printf("]");
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
