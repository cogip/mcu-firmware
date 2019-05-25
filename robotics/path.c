#include <stdio.h>

#include "path.h"
#include "platform.h"

#include "utils.h"

static path_pose_t poses[] = {
    /* POSE_INITIAL */
    {
        .pos = {
                   .x = 1500 - ROBOT_MARGIN,
                   .y = 300 + ROBOT_MARGIN,
                   .O = 180,
               },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED / 2,
        .act = NULL,
    },
    /* Game path */
    {   /* Sortie zone de départ */
        .pos = {
                   .x = 1200,
                   .y = 300 + ROBOT_MARGIN,
                   .O = 180,
               },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED / 2,
        .act = NULL,
    },


    {
        /* Pré-Point récolte 3 palets proche balance */
        .pos = {
                   .x = 600,
                   .y = 1300,
                   .O = 90,
               },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED / 2,
        .act = NULL, // TODO: Ouvrir (pos 90 deg) les 3 servos ventouse face AV + activer pompes
    },
    {
        /* Point récolte 3 palets proche balance */
        /* TODO /!\ Régulation de vitesse seulement !! */
        .pos = {
                   .x = 600,
                   .y = 1361,
                   .O = 90,
               },
        .allow_reverse = FALSE,
        .max_speed = MAX_SPEED / 4,
        .act = NULL,
    },
    {
        /* Post-Point récolte 3 palets proche balance */
        .pos = {
                   .x = 600,
                   .y = 1300,
                   .O = 90,
               },
        .allow_reverse = TRUE,
        .max_speed = MAX_SPEED / 2,
        .act = pf_front_cup_ramp,
    },
};

path_t robot_path = {
    /* static cfg */
    .current_pose_idx = 0,
    .play_in_loop = FALSE,
    .nb_pose = sizeof(poses)/sizeof(path_pose_t),
    .poses = poses,
};

inline const path_pose_t *path_get_current_path_pos(const path_t *path)
{
    return &path->poses[path->current_pose_idx];
}

inline const path_pose_t* path_get_pose_at_idx(path_t *path, uint8_t idx)
{
    if ((idx < path->nb_pose) && (idx > 0))
        return &path->poses[idx];
    else
        return NULL;
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
