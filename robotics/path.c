#include <stdio.h>

#include "path.h"
#include "platform.h"

#include "utils.h"

static path_pose_t poses[] = {
    /* POSE_INITIAL */
    {
        .pos = {
                   .x = 0,
                   .y = 0,
                   .O = 90,
               },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED / 2,
        .act = NULL,
    },

    /* Travel around a square of 1000mm, reaching 0 angle at each point */
    {
        .pos = {
                   .x = 0,
                   .y = 1000,
                   .O = 0,
               },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED / 2,
        .act = NULL,
    },
    {
        .pos = {
                   .x = 1000,
                   .y = 1000,
                   .O = 0,
               },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED / 2,
        .act = NULL,
    },
    {
        .pos = {
                   .x = 1000,
                   .y = 0,
                   .O = -90,
               },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED / 2,
        .act = NULL,
    },
    {
        .pos = {
                   .x = 0,
                   .y = 0,
                   .O = 135,
               },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED / 2,
        .act = NULL,
    },

    /* Diagonals of the square */
    {
        .pos = {
                   .x = 1000,
                   .y = 1000,
                   .O = 0,
               },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED,
        .act = NULL,
    },
    {
        .pos = {
                   .x = 0,
                   .y = 1000,
                   .O = -135,
               },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED,
        .act = NULL,
    },
    {
        .pos = {
                   .x = 1000,
                   .y = 0,
                   .O = 0,
               },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED,
        .act = NULL,
    },

    /* Return to initial pose */
    {
        .pos = {
                   .x = 0,
                   .y = 0,
                   .O = 90,
               },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED,
        .act = NULL,
    },
};

path_t robot_path = {
    /* static cfg */
    .play_in_loop = FALSE,
    .nb_pose = 9,
    .poses = poses,
};

inline path_pose_t *path_get_current_path_pos(const path_t *path)
{
    return &path->poses[path->current_pose_idx];
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

inline uint8_t path_get_current_max_speed(const path_t *path)
{
    path_pose_t *current_path_pos = &path->poses[path->current_pose_idx];

    return MIN(current_path_pos->max_speed, MAX_SPEED);
}

void path_horizontal_mirror_all_pos(const path_t *path)
{
    for (int i = 0; i < path->nb_pose; i++) {
        pose_t *pos = &path->poses[i].pos;

        pos->x *= -1;
        pos->O = 180 - pos->O;
        pos->O = ((int)pos->O) % 360;
    }
}
