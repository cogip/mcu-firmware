#pragma once

// Application includes
#include "cogip_defs/cogip_defs.hpp"

// Project includes
#include "obstacles.hpp"

// Obstacles of Lidar detected obstacles
extern cogip::obstacles::list *lidar_obstacles;

void obstacle_updater_start(const pose_t *robot_state);
