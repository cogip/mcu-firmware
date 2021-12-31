/*
 * Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    lib_avoidance Avoidance module
 * @ingroup     lib
 * @brief       Avoidance module
 *
 * Avoidance is split in 2 steps:
 *   1. avoidance_build_graph(): build graph of avoidance path to reach finish
 *      pose, including obstacles avoidance.
 *   2. avoidance_get_pose(): get intermediate pose in avoidance computed path.
 *
 * Direct segment to reach final position can be checked to see if it becomes
 * free of obstacles with avoidance_check_recompute()
 *
 * @{
 * @file
 * @brief       Public API for avoidance module
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#pragma once

/* Standard includes */
#include <cstdint>

/* Project includes */
#include "cogip_defs/Coords.hpp"
#include "tracefd/File.hpp"

/**
 * @name Dijkstra avoidance parameters
 * @{
 */
#define AVOIDANCE_GRAPH_MAX_VERTICES         64          /**< Maximum graph vertice number */
#define AVOIDANCE_DIJKSTRA_MAX_DISTANCE      UINT32_MAX  /**< Maximum Dijkstra distance */
/** @} */

/**
 * @brief Return given index pose in the avoidance path.
 *
 * @param[in]    index  pose index in the avoidance path
 *
 * @return              wanted pose if found, start position otherwise
 */
cogip::cogip_defs::Pose avoidance_get_pose(uint8_t index);

/**
 * @brief Build avoidance graph, listing all path possibilities between start
 * and finish poses.
 *
 * @param[in]   start   start position
 * @param[in]   finish  finish position
 *
 * @return              true if recompute is needed, false otherwise
 */
bool avoidance_build_graph(const cogip::cogip_defs::Coords &s, const cogip::cogip_defs::Coords &f);

/**
 * @brief Check if avoidance should be recomputed by checking if an obstacle is
 * crossing the segment between start and finish position.
 *
 * @param[in]   start   start position
 * @param[in]   finish  finish position
 *
 * @return              true if recompute is needed, false otherwise
 */
bool avoidance_check_recompute(const cogip::cogip_defs::Coords &start,
                               const cogip::cogip_defs::Coords &stop);

/**
 * @brief Print list of intermediate positions to reach wanted path position
 *
 * @param[in]   out     tracefd descriptor used to print avoidance path
 *
 * @return
 */
void avoidance_print_path(cogip::tracefd::File &out);

/** @} */
