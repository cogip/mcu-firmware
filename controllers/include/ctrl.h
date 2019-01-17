#ifndef CONTROLLER_H_
#define CONTROLLER_H_

/* Standard includes */
#include <stdint.h>

/* Project includes */
#include "odometry.h"
#include "pid.h"

typedef void (*ctrl_pre_mode_cb_t)(pose_t*, polar_t*, polar_t*);
typedef void (*ctrl_post_mode_cb_t)(pose_t*, polar_t*, polar_t*);

typedef enum {
    CTRL_STATE_STOP = 0,
    CTRL_STATE_IDLE,
    CTRL_STATE_BLOCKED,
    CTRL_STATE_INGAME,
    CTRL_STATE_NUMOF,
} ctrl_mode_t;

typedef struct {
    pose_t pose_order;
    pose_t pose_current;
    polar_t* speed_order;
    polar_t speed_current;

    uint8_t pose_reached;
    uint8_t pose_intermediate;
    uint8_t allow_reverse;

    ctrl_mode_t current_mode;

    ctrl_pre_mode_cb_t ctrl_pre_mode_cb[CTRL_STATE_NUMOF];
    ctrl_post_mode_cb_t ctrl_post_mode_cb[CTRL_STATE_NUMOF];
} ctrl_common_t;

typedef struct _ctrl_t ctrl_t;
typedef int (*ctrl_mode_cb_t)(ctrl_t* ctrl, polar_t* command);

typedef struct {
    ctrl_mode_cb_t ctrl_mode_cb[CTRL_STATE_NUMOF];
} ctrl_configuration_t;

struct _ctrl_t {
    ctrl_common_t common;
    ctrl_configuration_t conf;
};

void motor_drive(polar_t *command);

void ctrl_set_pose_intermediate(ctrl_t *ctrl, uint8_t intermediate);
void ctrl_set_allow_reverse(ctrl_t *ctrl, uint8_t allow);

void ctrl_set_pose_reached(ctrl_t* ctrl);
uint8_t ctrl_is_pose_reached(ctrl_t* ctrl);

void ctrl_set_pose_to_reach(ctrl_t* ctrl, const pose_t* pose_order);
const pose_t* ctrl_get_pose_to_reach(ctrl_t *ctrl);

void ctrl_set_pose_current(ctrl_t* ctrl, const pose_t* pose_current);
const pose_t* ctrl_get_pose_current(ctrl_t* ctrl);

void ctrl_set_speed_order(ctrl_t* ctrl, polar_t* speed_order);
polar_t* ctrl_get_speed_order(ctrl_t* ctrl);

void ctrl_set_speed_current(ctrl_t* ctrl, const polar_t* speed_current);
const polar_t* ctrl_get_speed_current(ctrl_t* ctrl);

void ctrl_set_mode(ctrl_t *ctrl, ctrl_mode_t new_mode);

void *task_ctrl_update(void *arg);

#endif /* CONTROLLER_H_ */
