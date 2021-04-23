/*
 * Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    sys_obstacles Obstacles module
 * @ingroup     sys
 * @brief       Obstacles module
 *
 *
 * @{
 * @file
 * @brief       Public API for obstacles module
 *
 * @author      Eric Courtois <eric.courtois@gmail.com>
 */

#pragma once

/* Standard includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* Project includes */
#include "cogip_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef OBSTACLES_NUMOF
#define OBSTACLES_NUMOF 1   /**< number of used obstacles contexts */
#endif

/**
 * @brief   Obstacles parameters
 */
typedef struct {
    uint32_t default_width; /**< Obstacle default width */
    uint32_t min_distance;  /**< Minimun distance from origin to create an obstacle */
    uint32_t max_distance;  /**< Maximum distance from origin to create an obstacle */
} obstacles_params_t;

typedef struct {
    pose_t points[4];       /**< Points defining the obstacle */
    double angle;           /**< Orientation of the obstacle */
} obstacle_t;

/**
 * @brief   Default obstacles identifier definition
 */
typedef unsigned int obstacles_t;

/**
 * @brief Initialize obstacles context
 *
 * @param[in]   obstacles_params  obstacles parameters
 *
 * @return                        id of the initialized obstacles
 */
obstacles_t obstacles_init(const obstacles_params_t *obstacles_params);

/**
 * @brief Return the number of obstacles
 * @param[in]   obstacles_id  obstacles id
 * @return                    number of obstacles
 */
size_t obstacles_size(const obstacles_t obstacles_id);

/**
 * @brief Get an obstacle
 * @param[in]   obstacles_id  obstacles id
 * @param[in]   n             index of the obstacle to get
 * @return                    pointer to the nth obstacle
 */
const obstacle_t *obstacles_get(const obstacles_t obstacles_id, size_t n);

/**
 * @brief Get all obstacles
 * @param[in]   obstacles_id  obstacles id
 * @return                    pointer the list of obstacles
 */
const obstacle_t *obstacles_get_all(const obstacles_t obstacles_id);

/**
 * @brief Add an obstacle
 * @param[in]   obstacles_id  obstacles id
 * @return                    true if the obstacle was successfully added, false otherwise
 */
bool obstacles_add(const obstacles_t obstacles_id, const obstacle_t obstacle);

/**
 * @brief Lock obstacles data
 * @param[in]   obstacles_id  obstacles id to initialize
 */
void obstacles_lock(const obstacles_t obstacles_id);

/**
 * @brief Unlock obstacles data
 * @param[in]   obstacles_id  obstacles id
 */
void obstacles_unlock(const obstacles_t obstacles_id);

/**
 * @brief Update obstacles using data from a Lidar
 * @param[in]   obstacles_id  obstacles id
 * @param[in]   origin        origin of the Lidar data
 * @param[in]   distances     data provided by the Lidar (array of 360 distances, one per angle)
 */
void obstacles_update_from_lidar(const obstacles_t obstacles_id, const pose_t *origin, const uint16_t *distances);

/**
 * @brief Print specified obstacles in JSON format
 * @param[in]   obstacles_id  obstacles id
 * @param[in]   out           file descriptor used to print obstacles
 */
void obstacles_print_json(const obstacles_t obstacles_id, FILE *out);

/**
 * @brief Print all obstacles in JSON format
 * @param[in]   out           file descriptor used to print obstacles
 */
void obstacles_print_all_json(FILE *out);

#ifdef __cplusplus
}
#endif

/** @} */
