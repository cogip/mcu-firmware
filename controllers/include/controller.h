#ifndef CONTROLLER_H_
#define CONTROLLER_H_

/* Standard includes */
#include <stdint.h>

/* Project includes */
#include "odometry.h"
#include "pid.h"

typedef void (*state_cb_t)(pose_t*, polar_t*);

typedef struct {
    char *name;
    state_cb_t state_cb;
} ctrl_mode_t;

typedef enum {
    CTRL_STATE_STOP = 0,
    CTRL_STATE_IDLE,
    CTRL_STATE_BLOCKED,
    CTRL_STATE_INGAME,
} ctrl_mode_id_t;

typedef enum {
    CTRL_REGUL_IDLE = 0,
    CTRL_REGUL_POSE_DIST,
    CTRL_REGUL_POSE_ANGL,
    CTRL_REGUL_POSE_PRE_ANGL,
    //CTRL_REGUL_SPEED, /* time for actions */
} ctrl_regul_t;

typedef struct {
    PID_t linear_speed_pid;
    PID_t angular_speed_pid;
    PID_t linear_pose_pid;
    PID_t angular_pose_pid;

    /* Distance approximation to switch to angular correction */
    uint16_t min_distance_for_angular_switch;

    /* Angle approximation to switch to position reached state */
    uint16_t min_angle_for_pose_reached;

    /* Dynamics variables */
    ctrl_mode_t *mode;

    pose_t* pose_order;
    pose_t* pose_current;
    polar_t* speed_order;
    uint8_t allow_reverse;

    ctrl_regul_t regul;
    uint8_t pose_reached;
    uint8_t pose_intermediate;
    uint8_t in_reverse;
} ctrl_t;

void motor_drive(polar_t *command);
polar_t speed_ctrl(ctrl_t *ctrl,
                         polar_t speed_setpoint, polar_t real_speed);

polar_t ctrl_update(ctrl_t *ctrl,
                          const pose_t *current_pose,
                          polar_t current_speed);

void ctrl_set_pose_intermediate(ctrl_t *ctrl, uint8_t intermediate);
uint8_t ctrl_is_in_reverse(ctrl_t *ctrl);
void ctrl_set_allow_reverse(ctrl_t *ctrl, uint8_t allow);

uint8_t ctrl_is_pose_reached(ctrl_t* ctrl);
void ctrl_set_pose_to_reach(ctrl_t* ctrl, pose_t* pose_order);
pose_t* ctrl_get_pose_to_reach(ctrl_t *ctrl);
void ctrl_set_pose_current(ctrl_t* ctrl, pose_t* pose_current);
pose_t* ctrl_get_pose_current(ctrl_t* ctrl);

void ctrl_set_speed_order(ctrl_t* ctrl, polar_t* speed_order);
polar_t* ctrl_get_speed_order(ctrl_t* ctrl);

void ctrl_set_mode(ctrl_t *ctrl, ctrl_mode_id_t new_mode);

void *task_ctrl_update(void *arg);

#endif /* CONTROLLER_H_ */
