
#ifndef QUADPID_H_
#define QUADPID_H_

/* Standard includes */
#include <stdint.h>

/* Project includes */
#include "ctrl.h"
#include "odometry.h"
#include "pid.h"

typedef enum {
    CTRL_REGUL_IDLE = 0,
    CTRL_REGUL_POSE_DIST,
    CTRL_REGUL_POSE_ANGL,
    CTRL_REGUL_POSE_PRE_ANGL,
    //CTRL_REGUL_SPEED, /* time for actions */
} ctrl_regul_t;

typedef struct {
    ctrl_common_t common;

    PID_t linear_speed_pid;
    PID_t angular_speed_pid;
    PID_t linear_pose_pid;
    PID_t angular_pose_pid;

    /* Distance approximation to switch to angular correction */
    uint16_t min_distance_for_angular_switch;

    /* Angle approximation to switch to position reached state */
    uint16_t min_angle_for_pose_reached;

    ctrl_regul_t regul;

} ctrl_quadpid_t;

polar_t ctrl_quadpid_speed(ctrl_quadpid_t* ctrl,
                         polar_t speed_order, polar_t speed_current);

polar_t ctrl_quadpid_pose(ctrl_quadpid_t* ctrl,
                          const pose_t* pose_current,
                          polar_t speed_current);

#endif  /* QUADPID_H_ */
