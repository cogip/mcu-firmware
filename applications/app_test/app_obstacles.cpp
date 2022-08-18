#include "app_obstacles.hpp"

#include "obstacles/List.hpp"

#include "etl/pool.h"

namespace cogip {

namespace app {

static etl::pool<FixedObstacle, 16> obstacles_pool;

static etl::map<CampColor, FixedObstacle *, 2> _excavation_site_obstacles;
static etl::map<CampColor, FixedObstacle *, 2> _shed_obstacles;

void _init_border_obstacles(void)
{
    FixedObstacle *obstacle = nullptr;
    static obstacles::List border_obstacles;

    // // Table borders
    // border_obstacles.push_back(obstacles_pool.create(cogip_defs::Coords(0, 0), 0, 3000, 1));
    // border_obstacles.push_back(obstacles_pool.create(cogip_defs::Coords(0, 2000), 0, 3000, 1));
    // border_obstacles.push_back(obstacles_pool.create(cogip_defs::Coords(-1500, 1000), 0, 1, 2000));
    // border_obstacles.push_back(obstacles_pool.create(cogip_defs::Coords(1500, 1000), 0, 1, 2000));

    // Gallery split cleats
    border_obstacles.push_back(obstacles_pool.create(cogip_defs::Coords(0, 150), 0, 11, 150));

    // Statuette supports
    obstacle = obstacles_pool.create(cogip_defs::Coords(-1500, 2000), 45, 721.25, 721.25);
    border_obstacles.push_back(obstacle);
    _shed_obstacles[CampColor::Purple] = obstacle;
    obstacle = obstacles_pool.create(cogip_defs::Coords(1500, 2000), -45, 721.25, 721.25);
    border_obstacles.push_back(obstacle);
    _shed_obstacles[CampColor::Yellow] = obstacle;

    // Sample supports
    border_obstacles.push_back(obstacles_pool.create(cogip_defs::Coords(-150, 51), 0, 150, 102));
    border_obstacles.push_back(obstacles_pool.create(cogip_defs::Coords(150, 51), 0, 150, 102));
    border_obstacles.push_back(obstacles_pool.create(cogip_defs::Coords(-1449, 1250), 0, 102, 150));
    border_obstacles.push_back(obstacles_pool.create(cogip_defs::Coords(1449, 1250), 0, 102, 150));

    // Exhibition galleries
    border_obstacles.push_back(obstacles_pool.create(cogip_defs::Coords(-690, 0), 0, 720, 84.87*2));
    border_obstacles.push_back(obstacles_pool.create(cogip_defs::Coords(690, 0), 0, 720, 84.87*2));
}

static void _init_excavation_sites_obstacles(void)
{
    FixedObstacle *obstacle = nullptr;
    static obstacles::List obstacles;

    obstacle = obstacles_pool.create(cogip_defs::Coords(525, 1375), 0, 350, 350);
    obstacles.push_back(obstacle);
    _excavation_site_obstacles[CampColor::Yellow] = obstacle;

    obstacle = obstacles_pool.create(cogip_defs::Coords(-525, 1375), 0, 350, 350);
    obstacles.push_back(obstacle);
    _excavation_site_obstacles[CampColor::Purple] = obstacle;
}

etl::map<CampColor, FixedObstacle *, 2> & app_get_excavation_sites_obstacles(void) {
    return _excavation_site_obstacles;
}

etl::map<CampColor, FixedObstacle *, 2> & app_get_shed_obstacles(void) {
    return _shed_obstacles;
}

void app_obstacles_init()
{
    _init_border_obstacles();
    _init_excavation_sites_obstacles();
}

} // namespace app

} // namespace cogip
