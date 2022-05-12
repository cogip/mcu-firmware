#pragma once

// Project includes
#include "cogip_defs/Pose.hpp"
#include "obstacles/List.hpp"

// Obstacles of Lidar detected obstacles
extern cogip::obstacles::List *lidar_obstacles;

void obstacle_updater_start(const cogip::cogip_defs::Pose &robot_state);
