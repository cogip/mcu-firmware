#include <stddef.h>
#include <kernel_defines.h>

#include "path.h"
#include "platform.h"

/*
 * TODO:
 * - Add anti-blocking activation for each point,
 * - For each actions: keep a status if its done (to avoid multiple puck capture
 *    if delivery was bypassed for any reason)
 * - Absolute coordinate recalibration on games frames (?)
 */
static path_pose_t poses[] = {
    {
        .pos = {
            .coords.x = -1200,
            .coords.y = 1000,
            .O = 0,
        },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED,
        .act = NULL,
    },
    {
        .pos = {
            .coords.x = 1200,
            .coords.y = 1000,
            .O = 180,
        },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED,
        .act = NULL,
    },
    {
        .pos = {
            .coords.x = -1200,
            .coords.y = 1000,
            .O = 0,
        },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED,
        .act = NULL,
    },
};

path_t robot_path = {
    /* static cfg */
    .current_pose_idx = 0,
    .play_in_loop = TRUE,
    .nb_poses = ARRAY_SIZE(poses),
    .poses = poses,
};
