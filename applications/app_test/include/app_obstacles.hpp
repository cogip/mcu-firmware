#pragma once

#include "app_camp.hpp"
#include "platform.hpp"
#include "obstacles/Rectangle.hpp"

#include <map>

namespace cogip {

namespace app {

class FixedObstacle: public cogip::obstacles::Rectangle {
public:
    FixedObstacle(
        const cogip::cogip_defs::Coords &center, double angle,
        double length_x, double length_y
        ): cogip::obstacles::Rectangle(
            center, angle, length_x + ROBOT_MARGIN * 2, length_y + ROBOT_MARGIN * 2) {};
};

/// Initialize fixed obstacles.
void app_obstacles_init();

/// Return excavation site obstacles.
std::map<CampColor, FixedObstacle *> & app_get_excavation_sites_obstacles(void);

/// Return shed obstacles.
std::map<CampColor, FixedObstacle *> & app_get_shed_obstacles(void);

} // namespace app

} // namespace cogip
