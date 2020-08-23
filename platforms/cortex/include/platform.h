/*
 * Copyright (C) 2020 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    platforms_cortex Cortex platform
 * @ingroup     platforms_cortex
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

#define ROBOT_ID            0
#define ROBOT_WIDTH         354                 /* units: mm */
#define ROBOT_MARGIN        (ROBOT_WIDTH / 2)

/* Motorization bloc mechanical characteristics
 *
 * To be computed :
 *  - PULSE_PER_MM		    : Number of pulses per mm of coding wheel
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
/* Intermediate calculation
 * WHEELS_PERIMETER = pi*WHEELS_DIAMETER
 * PULSE_PER_MM = WHEELS_ENCODER_RESOLUTION / WHEELS_PERIMETER
 */
#define PULSE_PER_MM        10.624
/* WHEELS_DISTANCE = WHEELS_DISTANCE_MM * PULSE_PER_MM */
#define WHEELS_DISTANCE     2974.72
/* WHEELS_DISTANCE*2*pi pulses for 360 deg. Thus 51.76 pulses per deg */
#define PULSE_PER_DEGREE    51.91

/* Delay to press a key before the robot starts */
#define PF_START_COUNTDOWN  3

/* Shell commands array size */
#define NB_SHELL_COMMANDS   17

/* Timeout before completely stop the robot once started */
#define GAME_DURATION_SEC   100

/* Camp selection for path mirroring */
#define CAMP_LEFT 1
#define CAMP_RIGHT 0

/* Acceleration and speed profiles */
#define MAX_ACC     5
#define MAX_SPEED   10
#define LOW_SPEED           (MAX_SPEED / 4)
#define NORMAL_SPEED        (MAX_SPEED / 2)

/* Anti-blocking */
#define PF_CTRL_BLOCKING_SPEED_TRESHOLD         1
#define PF_CTRL_BLOCKING_SPEED_ERR_TRESHOLD     1.5
#define PF_CTRL_BLOCKING_NB_ITERATIONS          40

/* Borders in which the robot can move */
#define AVOIDANCE_BORDER_X_MIN  (-1500 + ROBOT_MARGIN + 10)
#define AVOIDANCE_BORDER_X_MAX  (AVOIDANCE_BORDER_X_MIN * -1)
#define AVOIDANCE_BORDER_Y_MIN  0 + (ROBOT_MARGIN + 10)
#define AVOIDANCE_BORDER_Y_MAX  2000 - (ROBOT_MARGIN)
/* Borders in which an obstacle can be placed */
#define OBSTACLE_BORDER_X_MIN   AVOIDANCE_BORDER_X_MIN
#define OBSTACLE_BORDER_X_MAX   AVOIDANCE_BORDER_X_MAX
#define OBSTACLE_BORDER_Y_MIN   AVOIDANCE_BORDER_Y_MIN
#define OBSTACLE_BORDER_Y_MAX   AVOIDANCE_BORDER_Y_MAX
/* Obstacle size */
#define OBSTACLE_DYN_SIZE                   400
/* Detection thresholds */
#define OBSTACLE_DETECTION_MINIMUM_TRESHOLD 10
#define OBSTACLE_DETECTION_MAXIMUM_TRESHOLD 200

typedef struct {
    double angle_offset;
    double distance_offset;
} pf_sensor_t;

typedef struct {
    uint8_t nb_puck_front_ramp;
    uint8_t nb_puck_back_ramp;
    uint8_t front_ramp_blocked;
    uint8_t back_ramp_blocked;
    uint8_t front_arms_opened;
    uint8_t goldenium_opened;
    uint8_t goldenium_taken;
    uint8_t red_puck_on_hold_front;
    uint8_t red_puck_on_hold_back;
    uint8_t any_pump_on;
    uint8_t front_fork_occupied;
} pf_actions_context_t;

/*
 * VL53L0X I2C configuration.
 * Identical for all as a PCA9548 I2C switch is used.
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

/* VL53L0X number */
#define VL53L0X_NUMOF     (sizeof(vl53l0x_config) / sizeof(vl53l0x_config[0]))

/* PCA9548 I2C switch configuration */
static const pca9548_conf_t pca9548_config[] = {
    {
        .i2c_dev_id         = 1,
        .i2c_address        = 0x70,
        .channel_numof      = PCA9548_CHANNEL_MAX,
    },
};

/* PCA9548 number */
#define PCA9548_NUMOF      (sizeof(motor_driver_config) / sizeof(motor_driver_config[0]))

/*
 * Relation between VL53L0X id (index of the array)
 * and its PCA9548 port id (value of the array)
 */
static const uint8_t vl53l0x_channel[VL53L0X_NUMOF] = {
    0,
    1,
    2,
    6,
    4,
    5,
};

/*
 * Physical placement of each VL53L0X sensor on the robot
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

typedef struct shell_command_linked shell_command_linked_t;
struct shell_command_linked {
    /* Copy of the shell_commands currently used */
    shell_command_t shell_commands[NB_SHELL_COMMANDS];
    /* Pointer to the real current shell_commands */
    shell_command_linked_t *current;
    /* Pointer to the real previous shell_commands */
    shell_command_linked_t *previous;
    /* Menu name */
    const char *name;
};

extern shell_command_linked_t pf_shell_commands;
extern int pf_shm_key;

/* TODO: These functions/structs should be moved to common code */
void pf_push_shell_commands(shell_command_linked_t *shell_commands);
void pf_pop_shell_commands(void);
void pf_init_shell_commands(shell_command_linked_t *shell_commands, const char *name);
void pf_add_shell_command(shell_command_linked_t *shell_commands, shell_command_t *command);
int pf_display_json_help(int argc, char **argv);
int pf_exit_shell(int argc, char **argv);
int pf_print_pose_current_cb(int argc, char **argv);
extern shell_command_t cmd_help_json;
extern shell_command_t cmd_exit_shell;
extern shell_command_t cmd_print_pose_current;
extern shell_command_t cmd_print_dyn_obstacles;
extern shell_command_t cmd_set_shm_key;

path_t *pf_get_path(void);
int pf_is_game_launched(void);
int pf_is_camp_left(void);

void pf_ctrl_pre_running_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command);
void pf_ctrl_post_running_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command);
void pf_ctrl_post_stop_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command);
void pf_init_quadpid_params(ctrl_quadpid_parameters_t ctrl_quadpid_params);
ctrl_quadpid_t* pf_get_quadpid_ctrl(void);
ctrl_t* pf_get_ctrl(void);
void pf_init_tasks(void);
void pf_init(void);
int encoder_read(polar_t *robot_speed);
void encoder_reset(void);
int pf_read_sensors(void);
void pf_calib_read_sensors(pca9548_t dev);
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
