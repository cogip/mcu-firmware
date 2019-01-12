#ifndef CONTROLLER_H_
#define CONTROLLER_H_

/* Standard includes */
#include <stdint.h>

/* Project includes */
#include "odometry.h"
#include "pid.h"

typedef void (*mode_cb_t)(pose_t*, polar_t*);

typedef enum {
    CTRL_STATE_STOP = 0,
    CTRL_STATE_IDLE,
    CTRL_STATE_BLOCKED,
    CTRL_STATE_INGAME,
    CTRL_STATE_NUMOF,
} ctrl_mode_id_t;

typedef struct {
    ctrl_mode_id_t mode_id;
    char *name;
    mode_cb_t mode_cb;
} ctrl_mode_t;

typedef struct {
    pose_t pose_order;
    pose_t* pose_current;
    polar_t* speed_order;

    uint8_t pose_reached;
    uint8_t pose_intermediate;
    uint8_t allow_reverse;

    ctrl_mode_t *current_mode;
    ctrl_mode_t modes[CTRL_STATE_NUMOF];
} ctrl_common_t;

typedef struct {
    ctrl_common_t common;
} ctrl_t;

void motor_drive(polar_t *command);

void ctrl_set_pose_intermediate(ctrl_t *ctrl, uint8_t intermediate);
void ctrl_set_allow_reverse(ctrl_t *ctrl, uint8_t allow);

uint8_t ctrl_is_pose_reached(ctrl_t* ctrl);
void ctrl_set_pose_to_reach(ctrl_t* ctrl, pose_t* pose_order);
const pose_t* ctrl_get_pose_to_reach(ctrl_t *ctrl);
void ctrl_set_pose_current(ctrl_t* ctrl, pose_t* pose_current);
pose_t* ctrl_get_pose_current(ctrl_t* ctrl);

void ctrl_set_speed_order(ctrl_t* ctrl, polar_t* speed_order);
polar_t* ctrl_get_speed_order(ctrl_t* ctrl);

void ctrl_set_mode(ctrl_t *ctrl, ctrl_mode_id_t new_mode);

void *task_ctrl_update(void *arg);

#endif /* CONTROLLER_H_ */
