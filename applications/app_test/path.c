#include <stddef.h>

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
            .x = -1200,
            .y = 1000,
            .O = 0,
        },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED,
        .act = NULL,
    },
    {
        .pos = {
            .x = 1200,
            .y = 1000,
            .O = 180,
        },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED,
        .act = NULL,
    },
    {
        .pos = {
            .x = -1200,
            .y = 1000,
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
    .nb_pose = sizeof(poses) / sizeof(path_pose_t),
    .poses = poses,
};

inline const path_pose_t *path_get_current_path_pos(const path_t *path)
{
    return &path->poses[path->current_pose_idx];
}

inline const path_pose_t *path_get_pose_at_idx(path_t *path, uint8_t idx)
{
    if ((idx < path->nb_pose) && (idx > 0)) {
        return &path->poses[idx];
    }
    else {
        return NULL;
    }
}

inline uint8_t path_get_current_pose_idx(path_t *path)
{
    return path->current_pose_idx;
}

inline void path_set_current_pose_idx(path_t *path, uint8_t idx)
{
    path->current_pose_idx = idx;
}

inline void path_reset_current_pose_idx(path_t *path)
{
    path->current_pose_idx = 0;
}

inline void path_increment_current_pose_idx(path_t *path)
{
    if (path->current_pose_idx < path->nb_pose - 1) {
        path->current_pose_idx += 1;
    }
    else if (path->play_in_loop) {
        path->current_pose_idx = 0;
    }
}

inline void path_decrement_current_pose_idx(path_t *path)
{
    if (path->current_pose_idx > 0) {
        path->current_pose_idx -= 1;
    }
    else if (path->play_in_loop) {
        path->current_pose_idx = path->nb_pose - 1;
    }
}

inline uint8_t path_get_current_max_speed(const path_t *path)
{
    const path_pose_t *current_path_pos = &path->poses[path->current_pose_idx];

    return MIN(current_path_pos->max_speed, MAX_SPEED);
}

void path_horizontal_mirror_all_pos(const path_t *path)
{
    for (int i = 0; i < path->nb_pose; i++) {
        pose_t *pos = (pose_t *)&path->poses[i].pos;

        pos->x *= -1;
        pos->O = 180 - pos->O;
        pos->O = ((int)pos->O) % 360;
    }
}
