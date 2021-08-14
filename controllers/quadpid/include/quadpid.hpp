/*
 * Copyright (C) 2019 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    quadpid Quadruple PID actuator motion controller
 * @ingroup     controllers
 * @brief       Actuators Quadruple PID  motion speed and position controller
 *
 * The QuadPID controller aims to control actuators motions according to a
 * quadruple PID corrector:
 *   * Linear pose corrector:   Regulate distance to the pose order.
 *   * Angular pose corrector:  Regulate angle to the pose order.
 *   * Linear speed corrector:  Regulate linear speed to the pose order.
 *   * Angular speed corrector: Regulate rotation speed to the pose order.
 *
 * The structure ctrl_quadpid_t is used to represent the controller and inherit
 * from ctrl_t
 *   * linear_pose_pid:     Linear pose corrector
 *   * angular_pose_pid:    Angular pose corrector
 *   * linear_speed_pid:    Linear speed corrector
 *   * angular_speed_pid:   Angular speed corrector
 *
 * @{
 * @file
 * @brief       QuadPID controllers API and datas
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 * @author      Yannick GICQUEL <yannick.gicquel@gmail.com>
 * @author      Stephen CLYMANS <sclymans@gmail.com>
 */

#pragma once

/* Standard includes */
#include <stdint.h>

/* Project includes */
#include "ctrl.h"
#include "odometry.h"
#include "pid.h"

/**
 * @brief    PID regulation modes.
 */
typedef enum {
    CTRL_REGUL_IDLE = 0,        /**< No regulation */
    CTRL_REGUL_POSE_DIST,       /**< Distance regulation, reach destination */
    CTRL_REGUL_POSE_ANGL,       /**< Final angle correction */
    CTRL_REGUL_POSE_PRE_ANGL,   /**< Pre-angle orientation to reach destination
                                     with straight trajectory */
} ctrl_regul_t;

/**
 * @brief    QuadPID controller specific configuration.
 */
typedef struct {
    PID_t linear_speed_pid;                     /**< Linear speed Kp, Ki, Kd */
    PID_t angular_speed_pid;                    /**< Angular speed Kp, Ki, Kd */
    PID_t linear_pose_pid;                      /**< Linear pose Kp, Ki, Kd */
    PID_t angular_pose_pid;                     /**< Angular pose Kp, Ki, Kd */

    uint16_t min_distance_for_angular_switch;   /**< Distance approximation to
                                                     switch to angular
                                                     correction */

    uint16_t min_angle_for_pose_reached;        /**< Angle approximation to
                                                     switch to position reached
                                                     state */

    ctrl_regul_t regul;                         /**< Current regulation type */
} ctrl_quadpid_parameters_t;

/**
 * @brief    QuadPID controller specific structure based on ctrl_t.
 */
typedef struct {
    const ctrl_configuration_t *conf;                   /**< See ctrl_t */
    const ctrl_platform_configuration_t *pf_conf;       /**< See ctrl_t */
    ctrl_control_t control;                             /**< See ctrl_t */
    ctrl_quadpid_parameters_t quadpid_params;           /**< QuadPID specific
                                                           configuration */
} ctrl_quadpid_t;

/**
 * @brief   Speed regulation function.
 *
 * Perform the linear and angular speeds regulation according to the given
 * speed order.
 *
 * @param[in]       ctrl            QuadPID controller object
 * @param[in,out]   command         In: Linear and angular pose command \n
 *                                  Out: Linear and angular speed command
 * @param[in]       speed_current   Current linear and angular speed
 *
 * @return                      0 on success
 * @return                      not 0 on error
 */
int ctrl_quadpid_speed(ctrl_quadpid_t *ctrl,
                       polar_t *command, const polar_t *speed_current);

/**
 * @brief   QuadPID CTRL_MODE_STOP callback.
 *
 * Callback launched when controller is stopped
 *
 * @param[in]       ctrl        QuadPID controller object
 * @param[out]   command        Null linear and angular speed command
 *
 * @return                      0 on success
 * @return                      not 0 on error
 */
int ctrl_quadpid_stop(ctrl_t *ctrl, polar_t *command);

/**
 * @brief   QuadPID CTRL_MODE_PASSTHROUGH callback.
 *
 * Callback launched when controller is set in no-PID mode.
 *
 * @param[in]       ctrl        QuadPID controller object
 * @param[out]   command        Linear and angular motors commands
 *
 * @return                      0 on success
 * @return                      not 0 on error
 */
int ctrl_quadpid_nopid(ctrl_t *ctrl, polar_t *command);

/**
 * @brief   QuadPID CTRL_MODE_RUNNING_SPEED callback.
 *
 * Callback launched when controller is in running speed only mode.
 *
 * @param[in]       ctrl        QuadPID controller object
 * @param[out]   command        Null linear and angular speed command
 *
 * @return                      0 on success
 * @return                      not 0 on error
 */
int ctrl_quadpid_running_speed(ctrl_t *ctrl, polar_t *command);

/**
 * @brief   QuadPID CTRL_MODE_RUNNING callback.
 *
 * Callback launched when controller is in running mode
 *
 * @param[in]       ctrl        QuadPID controller object
 * @param[out]   command        Null linear and angular speed command
 *
 * @return                      0 on success
 * @return                      not 0 on error
 */
int ctrl_quadpid_ingame(ctrl_t *ctrl, polar_t *command);


/**
 * @brief    QuadPID controller static configuration.
 *           Setup callbacks for each controller mode.
 */
static const ctrl_configuration_t ctrl_quadpid_conf = {
    .ctrl_mode_cb[CTRL_MODE_STOP] = ctrl_quadpid_stop,
    .ctrl_mode_cb[CTRL_MODE_BLOCKED] = ctrl_quadpid_stop,
    .ctrl_mode_cb[CTRL_MODE_RUNNING] = ctrl_quadpid_ingame,
    .ctrl_mode_cb[CTRL_MODE_RUNNING_SPEED] = ctrl_quadpid_running_speed,
    .ctrl_mode_cb[CTRL_MODE_PASSTHROUGH] = ctrl_quadpid_nopid,
};

/** @} */
