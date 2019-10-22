#ifndef PLATFORM_COMMON_H_
#define PLATFORM_COMMON_H_

/* Project includes */
#include "ctrl.h"
#include "ctrl/quadpid.h"
#include "odometry.h"
#include "path.h"
#include "utils.h"

/* RIOT includes */
#include "periph/qdec.h"
#include "shell.h"

#define ROBOT_ID            0
#define ROBOT_WIDTH         354                 /* units: mm */
#define ROBOT_MARGIN        (ROBOT_WIDTH / 2)

/* To be computed :
 *  - PULSE_PER_MM		: Number of pulses per mm of coding wheel
 *  - WHEELS_DISTANCE		: Distance between coding wheels in pulses
 *  - PULSE_PER_DEGREE		: Number of pulses per degree of coding wheel
 *
 * Must be known :
 *  - WHEELS_DIAMETER		: Coding wheel diameter
 *  - WHEELS_DISTANCE_MM	: Distance between coding wheels in mm
 *
 * Must be known and defined :
 *  - WHEELS_ENCODER_RESOLUTION	: Number of pulses by turn of coding wheels
 */

#define WHEELS_ENCODER_RESOLUTION   2000
/* WHEELS_PERIMETER = pi*WHEELS_DIAMETER
 * PULSE_PER_MM = WHEELS_ENCODER_RESOLUTION / WHEELS_PERIMETER
 */
#define PULSE_PER_MM        10.624
/* WHEELS_DISTANCE = WHEELS_DISTANCE_MM * PULSE_PER_MM */
#define WHEELS_DISTANCE     2974.72
/* WHEELS_DISTANCE*2*pi pulses for 360 deg. Thus 51.76 pulses per deg */
#define PULSE_PER_DEGREE    51.91

#define PF_START_COUNTDOWN  3

#define NB_SHELL_COMMANDS   10

#define GAME_DURATION_SEC   100

#define CAMP_LEFT 1
#define CAMP_RIGHT 0

#define MAX_ACC     5
#define MAX_SPEED   10

#define LOW_SPEED           (MAX_SPEED / 4)
#define NORMAL_SPEED        (MAX_SPEED / 2)

#define PF_CTRL_BLOCKING_SPEED_TRESHOLD         1
#define PF_CTRL_BLOCKING_SPEED_ERR_TRESHOLD     1.5
#define PF_CTRL_BLOCKING_NB_ITERATIONS          40

#define AVOIDANCE_BORDER_X_MIN  (-1500 + ROBOT_MARGIN + 10)
#define AVOIDANCE_BORDER_X_MAX  (AVOIDANCE_BORDER_X_MIN * -1)
#define AVOIDANCE_BORDER_Y_MIN  0 + (ROBOT_MARGIN + 10)
#define AVOIDANCE_BORDER_Y_MAX  2000 - (ROBOT_MARGIN)

#define OBSTACLE_BORDER_X_MIN   AVOIDANCE_BORDER_X_MIN
#define OBSTACLE_BORDER_X_MAX   AVOIDANCE_BORDER_X_MAX
#define OBSTACLE_BORDER_Y_MIN   AVOIDANCE_BORDER_Y_MIN
#define OBSTACLE_BORDER_Y_MAX   AVOIDANCE_BORDER_Y_MAX

path_t *pf_get_path(void);
int pf_is_game_launched(void);
int pf_is_camp_left(void);

void pf_add_shell_command(shell_command_t *command);
void pf_ctrl_pre_running_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command);
void pf_ctrl_post_running_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command);
void pf_ctrl_post_stop_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command);
ctrl_quadpid_t* pf_get_quadpid_ctrl(void);
ctrl_t* pf_get_ctrl(void);
void pf_init(void);
void pf_init_tasks(void);
int encoder_read(polar_t *robot_speed);
void encoder_reset(void);
int pf_read_sensors(void);
void pf_fixed_obstacles_init(void);

void motor_drive(polar_t *command);

static const ctrl_platform_configuration_t ctrl_pf_quadpid_conf = {
    .ctrl_pre_mode_cb[CTRL_MODE_RUNNING]        = pf_ctrl_pre_running_cb,
    .ctrl_pre_mode_cb[CTRL_MODE_RUNNING_SPEED]  = pf_ctrl_pre_running_cb,
    .ctrl_pre_mode_cb[CTRL_MODE_STOP]           = pf_ctrl_pre_running_cb,
    .ctrl_pre_mode_cb[CTRL_MODE_BLOCKED]        = pf_ctrl_pre_running_cb,
    .ctrl_post_mode_cb[CTRL_MODE_STOP]          = pf_ctrl_post_stop_cb,
    .ctrl_post_mode_cb[CTRL_MODE_BLOCKED]       = pf_ctrl_post_stop_cb,
    .ctrl_post_mode_cb[CTRL_MODE_RUNNING]       = pf_ctrl_post_running_cb,
    .ctrl_post_mode_cb[CTRL_MODE_RUNNING_SPEED] = pf_ctrl_post_running_cb,

    .ctrl_pre_mode_cb[CTRL_MODE_PASSTHROUGH]   = pf_ctrl_pre_running_cb,
    .ctrl_post_mode_cb[CTRL_MODE_PASSTHROUGH]  = pf_ctrl_post_running_cb,

    .blocking_speed_treshold            = PF_CTRL_BLOCKING_SPEED_TRESHOLD,
    .blocking_speed_error_treshold      = PF_CTRL_BLOCKING_SPEED_ERR_TRESHOLD,
    .blocking_cycles_max                = PF_CTRL_BLOCKING_NB_ITERATIONS,
};

#endif  /* PLATFORM_COMMON_H_ */
