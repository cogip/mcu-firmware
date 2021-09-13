#include "path/path.hpp"

#include "platform.hpp"
#include "utils.h"

const cogip::path::Pose *path_get_current_pose(const path_t *path)
{
    return &path->poses[path->current_pose_idx];
}

void path_reset_current_pose_idx(path_t *path)
{
    path->current_pose_idx = 0;
}

void path_increment_current_pose_idx(path_t *path)
{
    if (path->current_pose_idx < path->nb_poses - 1) {
        path->current_pose_idx += 1;
    }
    else if (path->play_in_loop) {
        path->current_pose_idx = 0;
    }
}

void path_decrement_current_pose_idx(path_t *path)
{
    if (path->current_pose_idx > 0) {
        path->current_pose_idx -= 1;
    }
    else if (path->play_in_loop) {
        path->current_pose_idx = path->nb_poses - 1;
    }
}

uint8_t path_get_current_max_speed(const path_t *path)
{
    const cogip::path::Pose &current_path_pos = path->poses[path->current_pose_idx];

    return MIN(current_path_pos.max_speed(), MAX_SPEED);
}

void path_horizontal_mirror_all_poses(path_t *path)
{
    for (int i = 0; i < path->nb_poses; i++) {
        cogip::path::Pose &pos = path->poses[i];

        pos.set_x(-pos.x());
        pos.set_O(180 - pos.O());
        pos.set_O(((int)pos.O()) % 360);
    }
}
