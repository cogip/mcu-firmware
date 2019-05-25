#ifndef PATH_H_
#define PATH_H_

#include "utils.h"
#include "odometry.h"

typedef struct {
    pose_t pos;
    uint8_t allow_reverse;
    uint8_t max_speed;
    func_cb_t act;
} path_pose_t;

typedef struct {
    /* static cfg */
    uint8_t play_in_loop; /* for unit tests */
    uint8_t nb_pose;
    const path_pose_t *poses;

    /* dynamic variables */
    uint8_t current_pose_idx;
} path_t;


extern path_t robot_path;

const path_pose_t* path_get_current_path_pos(const path_t *path);
const path_pose_t* path_get_pose_at_idx(path_t *path, uint8_t idx);
uint8_t path_get_current_pose_idx(path_t *path);
void path_set_current_pose_idx(path_t *path, uint8_t idx);
void path_reset_current_pose_idx(path_t *path);
void path_increment_current_pose_idx(path_t *path);
void path_decrement_current_pose_idx(path_t *path);
uint8_t path_get_current_max_speed(const path_t *path);
void path_horizontal_mirror_all_pos(const path_t *path);

#endif /* PATH_H_ */
