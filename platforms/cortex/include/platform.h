/*
 * Copyright (C) 2020 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    platform_cortex Cortex platform
 * @ingroup     platforms
 * @brief       COGIP robotic base platform definition
 * @{
 *
 * @file
 * @brief       Define hardware properties of Cortex platform.
 *              Units:
 *              * time:         s
 *              * distance:     mm
 *              * speed:        mm/s
 *              * acceleration: mm/s²
 *
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#pragma once

/* Project includes */
#include "ctrl.h"
#include "ctrl/quadpid.h"
#include "odometry.h"
#include "path.h"
#include "pca9548.h"
#include "utils.h"
#include "vl53l0x.h"

/* RIOT includes */
#include <periph/qdec.h>
#include <shell.h>

#define ROBOT_ID            0       /**< Robot ID for logs */

/**
 * @name Robot mechanical properties
 *
 * To be computed :
 *  - PULSE_PER_MM          : Number of pulses per mm of coding wheel
 *  - WHEELS_DISTANCE       : Distance between coding wheels (pulses)
 *  - PULSE_PER_DEGREE      : Number of pulses per degree of coding wheel
 *
 * Must be known :
 *  - WHEELS_DIAMETER       : Coding wheel diameter (mm)
 *  - WHEELS_DISTANCE_MM    : Distance between coding wheels (mm)
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
#define ROBOT_WIDTH         354      /**< Robot width (mm) */
#define ROBOT_MARGIN \
    (ROBOT_WIDTH / 2)               /**< Point the most far from
                                      robot center (mm) */

#define PULSE_PER_MM        10.624   /**< WHEELS_ENCODER_RESOLUTION
                                       / WHEELS_PERIMETER */
#define WHEELS_DISTANCE     2974.72  /**< WHEELS_DISTANCE_MM
                                       * PULSE_PER_MM */
#define PULSE_PER_DEGREE    51.91    /**< WHEELS_DISTANCE * 2 * PI
                                       / 360 */
/** @} */

/**
 * @name Calibration
 *
 * @{
 */
#define PF_START_COUNTDOWN  3   /**< Delay to press a key before the robot
                                  starts */
#define NB_SHELL_COMMANDS   17  /**< Shell commands array size */
/** @} */

/**
 * @name Eurobot general properties
 *
 * @{
 */
#define GAME_DURATION_SEC   100 /**< Timeout before completely stop the robot
                                  once started */
#define CAMP_LEFT 1             /**< Camp left selection for path mirroring */
#define CAMP_RIGHT 0            /**< Camp left selection for path mirroring */
/** @} */

/**
 * @name Acceleration and speed profiles
 * @{
 */
#define MAX_ACC         5               /**< Maximum acceleration (mm/s²) */
#define MAX_SPEED       10              /**< Maximum speed (mm/s) */
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
#define PF_CTRL_BLOCKING_NB_ITERATIONS      40  /**< Number of interation
                                                  before considering the
                                                  robot blocked */
/** @} */

/**
 * @name Avoidance borders
 * Borders in which the robot can move (mm)
 * @{
 */
#define AVOIDANCE_BORDER_X_MIN \
    (-1500 + ROBOT_MARGIN + 10)     /**< Minimal X axis border */
#define AVOIDANCE_BORDER_X_MAX  \
    (AVOIDANCE_BORDER_X_MIN * -1)   /**< Maximal X axis border */
#define AVOIDANCE_BORDER_Y_MIN  \
    (0 + (ROBOT_MARGIN + 10))       /**< Minimal Y axis border */
#define AVOIDANCE_BORDER_Y_MAX \
    (2000 - (ROBOT_MARGIN))         /**< Maximal Y axis border */
/** @} */

/**
 * @name Obstacle borders
 * Borders in which an obstacle can be placed
 * @{
 */
#define OBSTACLE_BORDER_X_MIN \
    AVOIDANCE_BORDER_X_MIN          /**< Minimal X axis border */
#define OBSTACLE_BORDER_X_MAX \
    AVOIDANCE_BORDER_X_MAX          /**< Maximal X axis border */
#define OBSTACLE_BORDER_Y_MIN \
    AVOIDANCE_BORDER_Y_MIN          /**< Minimal Y axis border */
#define OBSTACLE_BORDER_Y_MAX \
    AVOIDANCE_BORDER_Y_MAX          /**< Maximal Y axis border */
/** @} */

/**
 * @name Avoidance characteristics
 * @{
 */
#define OBSTACLE_DYN_SIZE                   400 /**< Obstacle size */

#define OBSTACLE_DETECTION_MINIMUM_TRESHOLD 10  /**< Minimum obstacle detection
                                                  treshold */
#define OBSTACLE_DETECTION_MAXIMUM_TRESHOLD 200 /**< Maximum obstacle detection
                                                  treshold */
/** @} */


/**
 * @brief    General sensor structure
 */
typedef struct {
    double angle_offset;        /**< angle offset from robot front axis */
    double distance_offset;     /**< distance from robot center */
} pf_sensor_t;

/**
 * @brief   States of possible actions
 */
typedef struct {
    uint8_t nb_puck_front_ramp;     /**< Number of pucks in the front ramp */
    uint8_t nb_puck_back_ramp;      /**< Number of pucks in the back ramp */
    uint8_t front_ramp_blocked;     /**< Pucks blocked in front ramp */
    uint8_t back_ramp_blocked;      /**< Pucks blocked in back ramp */
    uint8_t front_arms_opened;      /**< Front arms state */
    uint8_t goldenium_opened;       /**< Goldenium lift opened */
    uint8_t goldenium_taken;        /**< Goldenium inside the robot */
    uint8_t red_puck_on_hold_front; /**< Red puck lifted on front */
    uint8_t red_puck_on_hold_back;  /**< Red puck lifted on back */
    uint8_t any_pump_on;            /**< All pumps are turned on */
    uint8_t front_fork_occupied;    /**< Front fork status */
} pf_actions_context_t;

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
void pf_ctrl_pre_running_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command);

/**
 * @brief Callback that drives actuators after motion control CTRL_MODE_RUNNING,
 * sending motion command to the motors.
 *
 * param[in]    robot_pose      Not used
 * param[in]    robot_speed     Not used
 * param[in]    motor_command   Motion command to send to the motors
 *
 * @return
 **/
void pf_ctrl_post_running_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command);

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
void pf_ctrl_post_stop_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command);


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
ctrl_quadpid_t* pf_get_quadpid_ctrl(void);


/**
 * @brief Returns platform controller.
 *
 * return   Controller
 **/
ctrl_t* pf_get_ctrl(void);

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
 * @brief Parse each sensor to read and store its value.
 * If one measure is lower that the avoidance detection treshold,
 * return True.
 *
 * @return                      0 if no avoidance needed, non 0 if needed
 **/
int pf_read_sensors(void);

/**
 * @brief Read all I2C sensors through PCA9548 switch
 *
 * param[in]    dev             PCA9548 device id
 *
 * @return
 **/
void pf_calib_read_sensors(pca9548_t dev);

/**
 * @brief Apply the given command to the motors
 *
 * param[in]    command         Linear and angular speeds command
 *
 * @return
 **/
void motor_drive(polar_t *command);

/**
 * @name Platform parameters for QuadPID controller.
 * @{
 **/
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
/** @} */

/**
 * @name VL53L0X I2C configuration.
 * @brief Address is identical for all devices as a PCA9548 I2C switch is used.
 */
static const vl53l0x_conf_t vl53l0x_config[] = {
    {
        .i2c_dev    = 1,
        .i2c_addr   = 0x29,
    },
    {
        .i2c_dev    = 1,
        .i2c_addr   = 0x29,
    },
    {
        .i2c_dev    = 1,
        .i2c_addr   = 0x29,
    },
    {
        .i2c_dev    = 1,
        .i2c_addr   = 0x29,
    },
    {
        .i2c_dev    = 1,
        .i2c_addr   = 0x29,
    },
    {
        .i2c_dev    = 1,
        .i2c_addr   = 0x29,
    },
};

/**
 * @name VL53L0X number defined by vl53l0x_config size
 * @{
 */
#define VL53L0X_NUMOF (sizeof(vl53l0x_config) / sizeof(vl53l0x_config[0])) \
    /**< VL53L0X number */
/** @} */

/**
 * @name PCA9548 I2C switch configuration
 * @{
 **/
static const pca9548_conf_t pca9548_config[] = {
    {
        .i2c_dev_id         = 1,
        .i2c_address        = 0x70,
        .channel_numof      = PCA9548_CHANNEL_MAX,
    },
};
/** @} */

/**
 * @name Relation between VL53L0X id (index of the array)
 * and its PCA9548 port id (value of the array)
 * @{
 */
static const uint8_t vl53l0x_channel[VL53L0X_NUMOF] = {
    0,
    1,
    2,
    6,
    4,
    5,
};
/** @} */

/**
 * @name Physical placement of each VL53L0X sensor on the robot
 * @{
 */
static const pf_sensor_t pf_sensors[VL53L0X_NUMOF] = {
    {
        .angle_offset = 135,
        .distance_offset = ROBOT_MARGIN,
    },
    {
        .angle_offset = 180,
        .distance_offset = ROBOT_MARGIN,
    },
    {
        .angle_offset = -135,
        .distance_offset = ROBOT_MARGIN,
    },
    {
        .angle_offset = -45,
        .distance_offset = ROBOT_MARGIN,
    },
    {
        .angle_offset = 0,
        .distance_offset = ROBOT_MARGIN,
    },
    {
        .angle_offset = 45,
        .distance_offset = ROBOT_MARGIN,
    },
};
/** @} */

/**
 * @name PCA9548 number defined by pca9548_config size
 * @{
 */
#define PCA9548_NUMOF (sizeof(pca9548_config) / sizeof(pca9548_config[0]))
    /**< PCA9548 number */
/** @} */

/** @} */
