/*
 * Copyright (C) 2021 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    platforms_pf_test Test platform
 * @ingroup     platforms
 * @brief       COGIP test platform definition
 * @{
 *
 * @file
 * @brief       Define hardware properties of test platform.
 *              Units:
 *              * time:         s
 *              * distance:     mm
 *              * speed:        mm/s
 *              * acceleration: mm/s²
 *
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 * @author      Eric Courtois <eric.courtois@gmail.com>
 */

#pragma once

/* Project includes */
#include "ctrl.h"
#include "quadpid.h"
#include "odometry.h"
#include "obstacles.h"
#include "path.h"
#include "utils.h"

#define ROBOT_ID            0       /**< Robot ID for logs */

/**
 * @name Robot mechanical properties
 *
 * To be computed :
 *  - PULSE_PER_MM		    : Number of pulses per mm of coding wheel
 *  - WHEELS_DISTANCE		: Distance between coding wheels (pulses)
 *  - PULSE_PER_DEGREE		: Number of pulses per degree of coding wheel
 *
 * Must be known :
 *  - WHEELS_DIAMETER		: Coding wheel diameter (mm)
 *  - WHEELS_DISTANCE_MM	: Distance between coding wheels (mm)
 *  - WHEELS_ENCODER_RESOLUTION: Number of pulses by turn of coding wheels
 *
 * Intermediate calculation:
 *  - WHEELS_PERIMETER = pi*WHEELS_DIAMETER
 *  - PULSE_PER_MM = WHEELS_ENCODER_RESOLUTION / WHEELS_PERIMETER
 *
 *  - WHEELS_DIAMETER = 60 mm
 *  - WHEELS_DISTANCE_MM = 280 mm
 *  - WHEELS_ENCODER_RESOLUTION = 2000
 *
 * @{
 */
#define ROBOT_WIDTH                         354                     /**< Robot width (mm) */
#define ROBOT_MARGIN                        (ROBOT_WIDTH / 2)       /**< Point the most far from robot center (mm) */
#define BEACON_SUPPORT_DIAMETER             70                      /**< Size of the beacon support (a cylinder of 70mm diameter to a cube of 100mm width) */

#define PULSE_PER_MM                        6.299                   /**< WHEELS_ENCODER_RESOLUTION / WHEELS_PERIMETER */
#define WHEELS_DISTANCE                     724.365                 /**< WHEELS_DISTANCE_MM * PULSE_PER_MM */
#define PULSE_PER_DEGREE                    12.64                   /**< WHEELS_DISTANCE * 2 * PI / 360 */
/** @} */

/**
 * @name Calibration
 *
 * @{
 */
#define PF_START_COUNTDOWN  3   /**< Delay to press a key before the robot starts */
/** @} */

/**
 * @name Eurobot general properties
 *
 * @{
 */
#define GAME_DURATION_SEC   100 /**< Timeout before completely stop the robot once started */
#define CAMP_LEFT 1             /**< Camp left selection for path mirroring */
#define CAMP_RIGHT 0            /**< Camp left selection for path mirroring */
/** @} */

/**
 * @name Acceleration and speed profiles
 * @{
 */
#define MAX_ACC         1.5             /**< Maximum acceleration (mm/s²) */
#define MAX_SPEED       3               /**< Maximum speed (mm/s) */
#define LOW_SPEED       (MAX_SPEED / 4) /**< Low speed (mm/s) */
#define NORMAL_SPEED    (MAX_SPEED / 2) /**< Normal speed (mm/s) */
/** @} */

/**
 * @name Anti-blocking
 * Quadpid controller anti-blocking characteristics
 * @{
 */
#define PF_CTRL_BLOCKING_SPEED_TRESHOLD     1   /**< Minimal speed treshold */
#define PF_CTRL_BLOCKING_SPEED_ERR_TRESHOLD 1.5 /**< Speed error treshold */
#define PF_CTRL_BLOCKING_NB_ITERATIONS      40  /**< Number of interation before considering the robot blocked */
/** @} */

/**
 * @name LIDAR characteristics
 * @{
 */
#define LIDAR_MIN_DISTANCE                  (ROBOT_MARGIN + 100)    /**< Minimum Lidar detection distance */
#define LIDAR_MAX_DISTANCE                  1200                    /**< Maximum Lidar detection distance */
#define LIDAR_MINIMUN_INTENSITY             1000                    /**< Minimum intensity required to validate a Lidar distance */
/** @} */

/**
 * @name Avoidance borders
 * Borders in which the robot can move (mm)
 * @{
 */
#define AVOIDANCE_BORDER_X_MIN -3000    /**< Minimal X axis border */
#define AVOIDANCE_BORDER_X_MAX  3000    /**< Maximal X axis border */
#define AVOIDANCE_BORDER_Y_MIN -3000    /**< Minimal Y axis border */
#define AVOIDANCE_BORDER_Y_MAX  3000    /**< Maximal Y axis border */
/** @} */

/**
 * @name Obstacle borders
 * Borders in which an obstacle can be placed
 * @{
 */
#define OBSTACLE_BORDER_X_MIN AVOIDANCE_BORDER_X_MIN    /**< Minimal X axis border */
#define OBSTACLE_BORDER_X_MAX AVOIDANCE_BORDER_X_MAX    /**< Maximal X axis border */
#define OBSTACLE_BORDER_Y_MIN AVOIDANCE_BORDER_Y_MIN    /**< Minimal Y axis border */
#define OBSTACLE_BORDER_Y_MAX AVOIDANCE_BORDER_Y_MAX    /**< Maximal Y axis border */
/** @} */

/**
 * @name Avoidance characteristics
 * @{
 */
#define OBSTACLES_BB_RADIUS_MARGIN          0.2                     /**< Obstacle circle bounding box mark-up */
#define OBSTACLE_DEFAULT_CIRCLE_RADIUS      400                     /**< Obstacle circle radius */
#define OBSTACLE_DETECTION_MINIMUM_TRESHOLD 10                      /**< Minimum obstacle detection treshold */
#define OBSTACLE_DETECTION_MAXIMUM_TRESHOLD 200                     /**< Maximum obstacle detection treshold */
/** @} */


/**
 * @brief    General sensor structure
 */
typedef struct {
    double angle_offset;        /**< angle offset from robot front axis */
    double distance_offset;     /**< distance from robot center */
} pf_sensor_t;

/**
 * @brief Get platform path
 *
 * @return                      Platform path
 */
path_t *pf_get_path(void);

/**
 * @brief Check if a game is started
 *
 * @return                      Return 1(true) if started, 0(false) otherwise
 */
int pf_is_game_launched(void);

/**
 * @brief Check the robot starting camp to mirror or not the path
 *
 * @return                      Return 1(true) if started, 0(false) otherwise
 */
int pf_is_camp_left(void);

/**
 * @brief Callback that prepares motion control CTRL_MODE_RUNNING, mainly
 * reading encoders and compute position and speed
 *
 * param[out]   robot_pose      Robot position computed from encoder data
 * param[out]   robot_speed     Robot speed computed from encoder data
 * param[in]    motor_command   Not used
 *
 * @return
 **/
void pf_ctrl_pre_running_cb(pose_t *robot_pose, polar_t *robot_speed, polar_t *motor_command);

/**
 * @brief Callback that drives actuators after motion control CTRL_MODE_RUNNING,
 * sending motion command to the motors.
 *
 * param[in]    robot_pose      Not used
 * param[in]    robot_speed     Not used
 * param[in]    motor_command   Motion command to sent to the motors
 *
 * @return
 **/
void pf_ctrl_post_running_cb(pose_t *robot_pose, polar_t *robot_speed, polar_t *motor_command);

/**
 * @brief Callback that drives actuators after motion control CTRL_MODE_STOP,
 * stopping the motors.
 *
 * param[in]    robot_pose      Not used
 * param[in]    robot_speed     Not used
 * param[in]    motor_command   Motion command forced to {0, 0}.
 *
 * @return
 **/
void pf_ctrl_post_stop_cb(pose_t *robot_pose, polar_t *robot_speed, polar_t *motor_command);

/**
 * @brief Initialize quadPID controller specific parameters, mainly PID
 * coefficients.
 *
 * param[in]    ctrl_quadpid_params     QuadPID specific parameters.
 *
 * @return
 **/
void pf_init_quadpid_params(ctrl_quadpid_parameters_t ctrl_quadpid_params);

/**
 * @brief Returns platform QuadPID controller.
 *
 * return   QuadPID controller
 **/
ctrl_quadpid_t *pf_get_quadpid_ctrl(void);

/**
 * @brief Returns platform controller.
 *
 * return   Controller
 **/
ctrl_t *pf_get_ctrl(void);

/**
 * @brief Initialize all platforms threads
 *
 * @return
 **/
void pf_init_tasks(void);

/**
 * @brief Platform initialization.
 * Must be called before any use of platform variables or functions.
 *
 * @return
 **/
void pf_init(void);

/**
 * @brief Read quadrature encoders to get linear and angular robot speed
 *
 * param[out]   robot_speed     Linear and angular speed
 *
 * @return                      0 if success, non 0 on error
 **/
int encoder_read(polar_t *robot_speed);

/**
 * @brief Reset quadrature encoders buffers
 *
 * @return
 **/
void encoder_reset(void);

/**
 * @brief Apply the given command to the motors
 *
 * param[in]    command         Linear and angular speeds command
 *
 * @return
 **/
void motor_drive(polar_t *command);

/**
 * @brief Get dynamic obstacles context.
 *
 * @return                      id of dynamic obstacles context
 **/
obstacles_t pf_get_dyn_obstacles_id(void);

static const ctrl_platform_configuration_t ctrl_pf_quadpid_conf = {
    .ctrl_pre_mode_cb[CTRL_MODE_RUNNING] = pf_ctrl_pre_running_cb,
    .ctrl_pre_mode_cb[CTRL_MODE_RUNNING_SPEED] = pf_ctrl_pre_running_cb,
    .ctrl_pre_mode_cb[CTRL_MODE_STOP] = pf_ctrl_pre_running_cb,
    .ctrl_pre_mode_cb[CTRL_MODE_BLOCKED] = pf_ctrl_pre_running_cb,
    .ctrl_post_mode_cb[CTRL_MODE_STOP] = pf_ctrl_post_stop_cb,
    .ctrl_post_mode_cb[CTRL_MODE_BLOCKED] = pf_ctrl_post_stop_cb,
    .ctrl_post_mode_cb[CTRL_MODE_RUNNING] = pf_ctrl_post_running_cb,
    .ctrl_post_mode_cb[CTRL_MODE_RUNNING_SPEED] = pf_ctrl_post_running_cb,

    .ctrl_pre_mode_cb[CTRL_MODE_PASSTHROUGH] = pf_ctrl_pre_running_cb,
    .ctrl_post_mode_cb[CTRL_MODE_PASSTHROUGH] = pf_ctrl_post_running_cb,

    .blocking_speed_treshold = PF_CTRL_BLOCKING_SPEED_TRESHOLD,
    .blocking_speed_error_treshold = PF_CTRL_BLOCKING_SPEED_ERR_TRESHOLD,
    .blocking_cycles_max = PF_CTRL_BLOCKING_NB_ITERATIONS,
};

/** @} */
