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
#include "ctrl.hpp"
#include "quadpid.hpp"
#include "odometry.hpp"
#include "utils.hpp"
#include "uartpb/UartProtobuf.hpp"

#define ROBOT_ID                            0       /**< Robot ID for logs */
#define CONTROLLER_SPEED_LOOP_PERIOD_MSEC   20      /**< Motion controller speed loop default period */

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
#define ROBOT_WIDTH                         333.54                  /**< Robot width (mm) */
#define ROBOT_MARGIN                        (ROBOT_WIDTH / 2)       /**< Point the most far from robot center (mm) */
#define BEACON_SUPPORT_DIAMETER             70                      /**< Size of the beacon support (a cylinder of 70mm diameter to a cube of 100mm width) */

#define PULSE_PER_MM                        6.299                   /**< WHEELS_ENCODER_RESOLUTION / WHEELS_PERIMETER */
#define WHEELS_DISTANCE                     724.365                 /**< WHEELS_DISTANCE_MM * PULSE_PER_MM */
#define PULSE_PER_DEGREE                    12.64                   /**< WHEELS_DISTANCE * 2 * PI / 360 */
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
#define MAX_ACC_M_PER_S2    0.2                                                 /**< Maximum acceleration (m/s²) */
#define MAX_SPEED_M_PER_S   1.2                                                 /**< Maximum speed (m/s) */
#define MAX_ACC_DEG_PER_S2  60                                                  /**< Maximum acceleration (deg/s²) */
#define MAX_SPEED_DEG_PER_S 270                                                 /**< Maximum speed (deg/s) */

#define MAX_ACC_LINEAR      ((MAX_ACC_M_PER_S2 * 1000) \
                            * ((double)CONTROLLER_SPEED_LOOP_PERIOD_MSEC))      /**< Maximum acceleration (pulses/THREAD_PERIOD_INTERVAL²) */
#define MAX_SPEED_LINEAR    ((MAX_SPEED_M_PER_S * 1000) \
                            * ((double)CONTROLLER_SPEED_LOOP_PERIOD_MSEC))      /**< Maximum speed (pulses/THREAD_PERIOD_INTERVAL) */
#define MAX_ACC_ANGULAR     (MAX_ACC_DEG_PER_S2 \
                            * ((double)CONTROLLER_SPEED_LOOP_PERIOD_MSEC))      /**< Maximum acceleration (pulses/THREAD_PERIOD_INTERVAL²) */
#define MAX_SPEED_ANGULAR   (MAX_SPEED_DEG_PER_S \
                            * ((double)CONTROLLER_SPEED_LOOP_PERIOD_MSEC))      /**< Maximum speed (pulses/THREAD_PERIOD_INTERVAL) */

#define LOW_SPEED_LINEAR        (MAX_SPEED_LINEAR / 4)                          /**< Low speed (mm/THREAD_PERIOD_INTERVAL) */
#define NORMAL_SPEED_LINEAR     (MAX_SPEED_LINEAR / 2)                          /**< Normal speed (mm/THREAD_PERIOD_INTERVAL) */
#define LOW_SPEED_ANGULAR       (MAX_SPEED_ANGULAR / 4)                         /**< Low speed (mm/THREAD_PERIOD_INTERVAL) */
#define NORMAL_SPEED_ANGULAR    (MAX_SPEED_ANGULAR / 2)                         /**< Normal speed (mm/THREAD_PERIOD_INTERVAL) */
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
 * @name Avoidance borders
 * Borders in which the robot can move (mm)
 * @{
 */
#define AVOIDANCE_BORDER_X_MIN (ROBOT_MARGIN + 10)                /**< Minimal X axis border */
#define AVOIDANCE_BORDER_X_MAX (3000 - ROBOT_MARGIN - 10)         /**< Maximal X axis border */
#define AVOIDANCE_BORDER_Y_MIN (-1000 + ROBOT_MARGIN + 10)        /**< Minimal Y axis border */
#define AVOIDANCE_BORDER_Y_MAX (AVOIDANCE_BORDER_Y_MIN * -1)      /**< Maximal Y axis border */
/** @} */

/**
 * @brief Get trace mode status
 *
 * @return                      true if trace mode is activited, false otherwise
 */
bool pf_trace_on(void);

/**
 * @brief Set/unset copilot connected
 *
 * param[in]    connected             copilot connected or not
 */
void pf_set_copilot_connected(bool connected);

/**
 * @brief Print current robot state in JSON format
 *
 * param[in]    out             tracefd descriptor used to print state
 *
 * @return
 */
void pf_print_state(void);

/**
 * @brief Send current robot pose in Protobuf format over UART
 * @return
 */
void pf_send_pb_pose(void);

/**
 * @brief Send current robot state in Protobuf format over UART
 * @return
 */
void pf_send_pb_state(void);

/**
 * @brief Check if a game is started
 *
 * @return                      Return 1(true) if started, 0(false) otherwise
 */
int pf_is_game_launched(void);

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
void pf_ctrl_pre_running_cb(cogip::cogip_defs::Pose &robot_pose,
                            cogip::cogip_defs::Polar &robot_speed,
                            cogip::cogip_defs::Polar &motor_command);

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
void pf_ctrl_post_running_cb(cogip::cogip_defs::Pose &robot_pose,
                             cogip::cogip_defs::Polar &robot_speed,
                             cogip::cogip_defs::Polar &motor_command);

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
void pf_ctrl_post_stop_cb(cogip::cogip_defs::Pose &robot_pose,
                          cogip::cogip_defs::Polar &robot_speed,
                          cogip::cogip_defs::Polar &motor_command);

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
 * @brief Returns uarpb.
 *
 * return   uarpb pointer
 **/
cogip::uartpb::UartProtobuf & pf_get_uartpb();

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
int encoder_read(cogip::cogip_defs::Polar &robot_speed);

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
void motor_drive(const cogip::cogip_defs::Polar &command);

/**
 * @name Platform parameters for QuadPID controller.
 * @{
 **/
static const ctrl_platform_configuration_t ctrl_pf_quadpid_conf = {
    .ctrl_pre_mode_cb = {
        pf_ctrl_pre_running_cb,  // CTRL_MODE_STOP
        nullptr,                 // CTRL_MODE_IDLE
        pf_ctrl_pre_running_cb,  // CTRL_MODE_BLOCKED
        pf_ctrl_pre_running_cb,  // CTRL_MODE_RUNNING
        pf_ctrl_pre_running_cb,  // CTRL_MODE_RUNNING_SPEED
        pf_ctrl_pre_running_cb,  // CTRL_MODE_PASSTHROUGH
    },
    .ctrl_post_mode_cb = {
        pf_ctrl_post_stop_cb,    // CTRL_MODE_STOP
        nullptr,                 // CTRL_MODE_IDLE
        pf_ctrl_post_stop_cb,    // CTRL_MODE_BLOCKED
        pf_ctrl_post_running_cb, // CTRL_MODE_RUNNING
        pf_ctrl_post_running_cb, // CTRL_MODE_RUNNING_SPEED
        pf_ctrl_post_running_cb, // CTRL_MODE_PASSTHROUGH
    },
    .blocking_speed_treshold = PF_CTRL_BLOCKING_SPEED_TRESHOLD,
    .blocking_speed_error_treshold = PF_CTRL_BLOCKING_SPEED_ERR_TRESHOLD,
    .blocking_cycles_max = PF_CTRL_BLOCKING_NB_ITERATIONS
};

/** @} */
