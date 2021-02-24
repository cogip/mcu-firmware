/*
 * Copyright (C) 2018 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    planner Simple trajectory planification module
 * @ingroup     planners
 * @brief       Trajectory planification module
 *
 * The planner, asociated to a controller, generate the robot course according
 * to a given path. It also handles avoidance behavior.
 *
 *
 * @{
 * @file
 * @brief       Common planner API and datas
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 * @author      Yannick GICQUEL <yannick.gicquel@gmail.com>
 * @author      Stephen CLYMANS <sclymans@gmail.com>
 */
#pragma once

/* Project includes */
#include "ctrl.h"
#include "odometry.h"
#include "path.h"
#include "utils.h"

/**
 * @brief Start the trajectory planification and the associated controller
 *
 * @param[in]   ctrl            Controller object
 *
 * @return
 */
void pln_start(ctrl_t* ctrl);

/**
 * @brief Stop the trajectory planification and the associated controller
 *
 * @param[in]   ctrl            Controller object
 *
 * @return
 */
void pln_stop(ctrl_t* ctrl);

/**
 * @brief Periodic task function to process a planner
 *
 * @param[in] arg               Should be NULL
 *
 * @return
 */
void *task_planner(void *arg);

/**
 * @brief Allow or not the planner to change automatically the next path pose
 * to reach once current pose is reached
 *
 * @param[in] value             TRUE to allow planner to change path pose,
 *                              FALSE otherwise
 *
 * @return
 */
void pln_set_allow_change_path_pose(uint8_t value);

/** @} */
