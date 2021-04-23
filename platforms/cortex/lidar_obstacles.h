#pragma once

/* Application includes */
#include "cogip_defs.h"

/* Obstacles id of Lidar detected obstacles*/
extern obstacles_t lidar_obstacles;

void obstacle_updater_start(const pose_t *robot_state);
