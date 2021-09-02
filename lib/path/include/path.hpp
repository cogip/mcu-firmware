/*
 * Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    lib_path Containing Path structure definition and utility functions to work with path
 * @ingroup     lib
 * @brief       Path module
 *
 *
 * @{
 * @file
 * @brief       Public API for path module
 *
 * @author      Stephen Clymans <sclymans@gmail.com>
 */
#pragma once

/* Project includes */
#include "utils.h"
#include "odometry.hpp"

/**
 * @brief Position type as used in path type
 */
typedef struct {
    pose_t pos;
    uint8_t allow_reverse;
    double max_speed;
    func_cb_t act;
} path_pose_t;

/**
 * @brief Path type
 */
typedef struct {
    /* static cfg */
    uint8_t play_in_loop; /* for unit tests */
    uint8_t nb_poses;
    const path_pose_t *poses;

    /* dynamic variables */
    uint8_t current_pose_idx;
} path_t;

/**
 * @brief Global variable containing complete robot path
 */
extern path_t robot_path;

/**
 * @brief Get current position
 * @param[in]    path    robot path
 * @return               current position of the path
 */
const path_pose_t *path_get_current_pose(const path_t *path);

/**
 * @brief Reset current position to index 0
 * @param[out]    path    robot path
 */
void path_reset_current_pose_idx(path_t *path);

/**
 * @brief Increment the current position in the robot path (going forward to the next position)
 * @param[out]    path    robot path
 */
void path_increment_current_pose_idx(path_t *path);

/**
 * @brief Decrement the current position in the robot path (going back to the last position)
 * @param[out]    path    robot path
 */
void path_decrement_current_pose_idx(path_t *path);

/**
 * @brief Get maximum speed to use in order to go to the current position
 * @param[out]    path    robot path
 * @return               maximum speed to use to go to the current position
 */
uint8_t path_get_current_max_speed(const path_t *path);

/**
 * @brief Mirror all points in path to match the two possible sides of the game
 * @param[out]    path    robot path
 */
void path_horizontal_mirror_all_poses(const path_t *path);

/** @} */
