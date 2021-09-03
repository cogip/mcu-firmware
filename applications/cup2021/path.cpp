#include <cstddef>
#include <kernel_defines.h>

#include "path.hpp"
#include "platform.hpp"

// TODO:
// - Add anti-blocking activation for each point,
// - For each actions: keep a status if its done (to avoid multiple puck capture
//    if delivery was bypassed for any reason)
// - Absolute coordinate recalibration on games frames (?)
static path_pose_t poses[] = {
    {
        .pos = { -1200, 1000, 0 },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED,
        .act = NULL,
    },
    {
        .pos = { 1200, 1000, 180 },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED,
        .act = NULL,
    },
    {
        .pos = { -1200, 1000, 0 },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED,
        .act = NULL,
    },
};

path_t robot_path = {
    // static cfg
    .play_in_loop = TRUE,
    .nb_poses = ARRAY_SIZE(poses),
    .poses = poses,
    .current_pose_idx = 0
};
