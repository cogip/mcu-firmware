#pragma once

// Application includes
#include "cogip_defs/Pose.hpp"

// Project includes
#include "obstacles.hpp"

// Obstacles of Lidar detected obstacles
extern cogip::obstacles::List *lidar_obstacles;

void obstacle_updater_start(const cogip::cogip_defs::Pose &robot_state);
