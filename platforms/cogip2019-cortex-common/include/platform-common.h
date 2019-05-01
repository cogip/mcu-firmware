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

#define PF_START_COUNTDOWN  5

#define NB_SHELL_COMMANDS   10

path_t *pf_get_path(void);
int pf_is_game_launched(void);
int pf_is_camp_left(void);

void pf_add_shell_command(shell_command_t *command);
void pf_ctrl_pre_running_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command);
void pf_ctrl_post_running_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command);
void pf_ctrl_post_stop_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command);
ctrl_quadpid_t* pf_get_quadpid_ctrl(void);
void pf_init(void);
void pf_init_tasks(void);
int encoder_read(polar_t *robot_speed);
void encoder_reset(void);

void motor_drive(polar_t *command);

static const ctrl_platform_configuration_t ctrl_pf_quadpid_conf = {
    .ctrl_pre_mode_cb[CTRL_MODE_RUNNING]    = pf_ctrl_pre_running_cb,
    .ctrl_post_mode_cb[CTRL_MODE_STOP]      = pf_ctrl_post_stop_cb,
    .ctrl_post_mode_cb[CTRL_MODE_BLOCKED]   = pf_ctrl_post_stop_cb,
    .ctrl_post_mode_cb[CTRL_MODE_RUNNING]   = pf_ctrl_post_running_cb,
};

#endif  /* PLATFORM_COMMON_H_ */
