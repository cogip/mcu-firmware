#include "lidar_obstacles.hpp"

// System includes
#include "etl/pool.h"

// Application includes
#include "obstacles/Circle.hpp"
#include "platform.hpp"

#include "PB_Obstacles.hpp"

static etl::pool<cogip::obstacles::Circle, OBSTACLES_MAX_NUMBER> circles_pool;

// Obstacles list of Lidar detected obstacles
cogip::obstacles::List & lidar_obstacles() {
    static cogip::obstacles::List obstacles;
    return obstacles;
};

static PB_Obstacles<
    OBSTACLES_MAX_NUMBER,
    OBSTACLES_MAX_NUMBER
> pb_obstacles;

void obstacles_handler(cogip::uartpb::ReadBuffer & buffer)
{
    auto & obstacles = lidar_obstacles();
    circles_pool.release_all();
    obstacles.clear();
    pb_obstacles.clear();

    EmbeddedProto::Error error = pb_obstacles.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        std::cout << "Obstacles: Protobuf deserialization error: " << static_cast<int>(error) << std::endl;
        return;
    }

    for (uint32_t i = 0; i < pb_obstacles.get_circles().get_length(); i++) {
        auto & pb_circle = pb_obstacles.circles(i);
        cogip::cogip_defs::Coords center(
            pb_circle.x(),
            pb_circle.y()
        );
        auto * obstacle = circles_pool.create(center, pb_circle.radius());
        obstacles.push_back(obstacle);
    }
}
