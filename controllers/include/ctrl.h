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
    CTRL_MODE_STOP = 0,
    CTRL_MODE_IDLE,
    CTRL_MODE_BLOCKED,
    CTRL_MODE_RUNNING,
    CTRL_MODE_NUMOF,
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
} ctrl_control_t;

typedef struct {
    const ctrl_pre_mode_cb_t ctrl_pre_mode_cb[CTRL_MODE_NUMOF];
    const ctrl_post_mode_cb_t ctrl_post_mode_cb[CTRL_MODE_NUMOF];
} ctrl_platform_configuration_t;

typedef struct _ctrl_t ctrl_t;
typedef int (*ctrl_mode_cb_t)(ctrl_t* ctrl, polar_t* command);

typedef struct {
    ctrl_mode_cb_t ctrl_mode_cb[CTRL_MODE_NUMOF];
} ctrl_configuration_t;

struct _ctrl_t {
    const ctrl_configuration_t conf;
    const ctrl_platform_configuration_t pf_conf;
    ctrl_control_t control;
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
