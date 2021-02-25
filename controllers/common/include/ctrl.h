/*
 * Copyright (C) 2018 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    controller Common
 * @ingroup     controllers
 * @brief       Actuators motion speed and position controller
 *
 * The controller aims to control actuators motions according to algorithms
 * like On-Off, PID, fuzzy logical, ...
 *
 * The structure ctrl_t is used to represent a controller.
 * Each specific controller has to inherit from this structure by adding first
 * the same parameters than ctrl_t.
 *
 * The controller structure ctrl_t is divided into 3 sub-structures:
 *  * ctrl_configuration_t defines controller core callbacks for each mode.
 *  * ctrl_platform_configuration_t defines preparation and post treatment
 *    functions to launch before and after each mode callbacks.
 *  * ctrl_control_t defines the various control variables common to all
 *    controllers
 *
 * Three structures are needed because each one is setup by different software
 * entities:
 * * The controller itself which defines its core callbacks
 * * The platform which is the only one to know how to prepare the controller
 *   from hardware informations.
 * * The entire source code which could change controller mode according to
 *   external events.
 * 
 * @verbatim
    --------------------------------------------------------------------
   | Structure/Setup               | Controller | Platform | Everywhere |
   |--------------------------------------------------------------------|
   | ctrl_configuration_t          |     X      |          |            |
   | ctrl_platform_configuration_t |            |    X     |            |
   | ctrl_control_t                |            |          |     X      |
   |_______________________________|____________|__________|____________|
   @endverbatim
 *
 * @{
 * @file
 * @brief       Common controllers API and datas
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 * @author      Yannick GICQUEL <yannick.gicquel@gmail.com>
 * @author      Stephen CLYMANS <sclymans@gmail.com>
 */

#pragma once

/* Standard includes */
#include <stdint.h>

/* Project includes */
#include "odometry.h"
#include "pid.h"

/**
 * @brief   Pre-controller callback. Called before the controller process
 *
 * This callback is used to prepare the call to the controller mode callback
 * ctrl_mode_cb[] according to the current mode.
 */
typedef void (*ctrl_pre_mode_cb_t)(pose_t*, polar_t*, polar_t*);

/**
 * @brief   Post-controller callback. Called after the controller process
 *
 * This callback is used for post treatment of the controller mode callback
 * ctrl_mode_cb[] according to the current mode.
 */
typedef void (*ctrl_post_mode_cb_t)(pose_t*, polar_t*, polar_t*);

/**
 *@brief    Controllers working mode
 */
typedef enum {
    CTRL_MODE_STOP = 0,     /**< Stopped */
    CTRL_MODE_IDLE,         /**< Idled, left free of motion */
    CTRL_MODE_BLOCKED,      /**< Blocked, often meaning something went wrong */
    CTRL_MODE_RUNNING,      /**< Move according to asked position */
    CTRL_MODE_RUNNING_SPEED,/**< Move according to asked speed  */
    CTRL_MODE_PASSTHROUGH,  /**< Direct control of motion, input is output */
    CTRL_MODE_NUMOF,        /**< Number of mode, never use it as an index */
} ctrl_mode_t;

/**
 * @brief   Controller default definition
 */
typedef struct ctrl_t ctrl_t;

/**
 * @brief   Variable speed order callback.
 *
 * When registered, this callback can enhance controller to move from a constant
 * speed_order to a variable speed_order driven by external part of code.
 * Thus, the speed_order can change depending on some factors, such as time or
 * mobile robot position.
 *
 * @param[in]       ctrl        Controler object
 *
 * @return                      New computed speed order
 */
typedef polar_t (*speed_order_cb_t)(ctrl_t* ctrl);

/**
 * @brief    Controller general structure
 */
typedef struct {
    pose_t pose_order;          /**< Position order */
    pose_t pose_current;        /**< Current position */
    polar_t speed_order;        /**< Speed order to reach the position */
    polar_t speed_current;      /**< Current speed reaching the position */
    speed_order_cb_t speed_order_cb; /**< Optional @ref speed_order_cb_t. */

    uint8_t pose_reached;       /**< Boolean set when pose_order is reached */
    uint8_t pose_intermediate;  /**< Boolean set when current pose_order is
                                     not the final destination */
    uint8_t allow_reverse;      /**< Boolean to allow going backward to reach
                                     the pose_order */
    uint8_t anti_blocking_on;   /**< Continuous cycles number the controller is
                                     blocked */
    uint16_t blocking_cycles;   /**< Continuous cycles number the controller is
                                     blocked */

    ctrl_mode_t current_mode;   /**< Current controller mode */

    uint32_t current_cycle;     /**< Count each control loop turn.
                                     Reset when a new controller mode is set,
                                     incremented on each loop turn otherwise */
} ctrl_control_t;

/**
 * @brief   Controllers pre and post callbacks for each mode
 */
typedef struct {
    const ctrl_pre_mode_cb_t \
        ctrl_pre_mode_cb[CTRL_MODE_NUMOF];  /**< Modes pre callbacks */
    const ctrl_post_mode_cb_t \
        ctrl_post_mode_cb[CTRL_MODE_NUMOF]; /**< Modes post callbacks */

    const uint16_t  blocking_speed_treshold;        /**< Blocking speed
                                                         treshold */
    const uint16_t  blocking_speed_error_treshold;  /**< Blocking speed error
                                                         treshold */
    const uint16_t  blocking_cycles_max;            /**< Blocking cycles
                                                         maximum number */
} ctrl_platform_configuration_t;

/**
 * @brief   Modes controller callbacks.
 *
 * This callback is the core action of the controller. There should be one
 * callback by mode
 *
 * @param[in]   ctrl            Controller object
 * @param[out]  command         Final command
 *
 * @return                      0 on success
 * @return                      not 0 on error
 */
typedef int (*ctrl_mode_cb_t)(ctrl_t* ctrl, polar_t* command);

/**
 * @brief   Controller modes callbacks definitions
 */
typedef struct {
    ctrl_mode_cb_t ctrl_mode_cb[CTRL_MODE_NUMOF];   /**< Modes callbacks */
} ctrl_configuration_t;

/**
 * @brief   Controller default definition
 */
struct ctrl_t {
    const ctrl_configuration_t* conf;               /**< Modes callbacks */
    const ctrl_platform_configuration_t* pf_conf;   /**< Pre and post
                                                         callbacks */
    ctrl_control_t control;                         /**< Control variables */
};

/**
 * @brief Get current elapsed cycle
 *
 * @param[in] ctrl              Controller object
 *
 * @return                      Current elapsed cycle
 */
uint32_t ctrl_get_current_cycle(ctrl_t* ctrl);

/**
 * @brief Set the pose order as an intermediate position
 *
 * @param[in] ctrl              Controller object
 * @param[in] intermediate      normal position when 0, intermediate otherwise
 *
 * @return
 */
void ctrl_set_pose_intermediate(ctrl_t *ctrl, uint8_t intermediate);

/**
 * @brief Get if the pose order is an intermediate position
 *
 * @param[in] ctrl              Controller object
 *
 * @return                      non-zero on intermediate position, 0 otherwise
 */
uint8_t ctrl_is_pose_intermediate(ctrl_t *ctrl);

/**
 * @brief Set the ability to go backward to reach the pose order
 *
 * @param[in] ctrl              Controller object
 * @param[in] allow             forbidden when 0, allowed otherwise
 *
 * @return
 */
void ctrl_set_allow_reverse(ctrl_t *ctrl, uint8_t allow);

/**
 * @brief Enable/disable robot blocked detection
 *
 * @param[in] ctrl              Controller object
 * @param[in] value             Disabled when 0, enabled otherwise
 *
 * @return
 */
void ctrl_set_anti_blocking_on(ctrl_t* ctrl, uint8_t value);

/**
 * @brief Get robot blocked detection status
 *
 * @param[in] ctrl              Controller object
 *
 * @return                      0 if disabled
 *                              >0 if enabled
 */
uint8_t ctrl_get_anti_blocking_on(ctrl_t* ctrl);

/**
 * @brief Up pose reached flag
 *
 * @param[in] ctrl              Controller object
 *
 * @return
 */
void ctrl_set_pose_reached(ctrl_t* ctrl);

/**
 * @brief Up pose reached flag
 *
 * @param[in] ctrl              Controller object
 *
 * @return                      0 if pose is not reached
 * @return                      >0 otherwise
 */
uint8_t ctrl_is_pose_reached(ctrl_t* ctrl);

/**
 * @brief Set pose order
 *
 * @param[in] ctrl              Controller object
 * @param[in] pose_order        Pose to reach
 *
 * @return
 */
void ctrl_set_pose_to_reach(ctrl_t* ctrl, const pose_t pose_order);

/**
 * @brief Get pose order
 *
 * @param[in] ctrl              Controller object
 *
 * @return                      Pose to reach
 */
pose_t ctrl_get_pose_to_reach(ctrl_t *ctrl);

/**
 * @brief Set current pose
 *
 * @param[in] ctrl              Controller object
 * @param[in] pose_current      Current pose
 *
 * @return
 */
void ctrl_set_pose_current(ctrl_t* const ctrl, const pose_t* pose_current);

/**
 * @brief Get current pose
 *
 * @param[in] ctrl              Controller object
 *
 * @return                      Current pose
 */
const pose_t* ctrl_get_pose_current(ctrl_t* ctrl);

/**
 * @brief Set speed order
 *
 * @param[in] ctrl              Controller object
 * @param[in] speed_order       Speed goal to reach pose order
 *
 * @note This set point can be overriden and ignored if user request a variable
 * speed_order. Please check @ref ctrl_register_speed_order_cb for more
 * informations.
 *
 * @return                      Nothing
 */
void ctrl_set_speed_order(ctrl_t* ctrl, polar_t speed_order);

/**
 * @brief Get speed order
 *
 * @param[in] ctrl              Controller object
 *
 * @return                      Speed order
 */
const polar_t* ctrl_get_speed_order(ctrl_t* ctrl);

/**
 * @brief Register a speed_order callback.
 *
 * When this callback function is called, the controller no more uses a
 * constant speed order to reach its regulations points, but instead rely on an
 * external function to compute its speed order. This function is called on each
 * controller cycle if @ref ctrl_compute_speed_order is called.
 * It allows the speed order to vary over time or other factor depending on the
 * callback implementation.
 *
 * This function overrides any other constant speed order the user could set
 * through @ref ctrl_set_speed_order. Thus any previous or future call to
 * @ref ctrl_set_speed_order could have no effect if a speed_order callback
 * function is registered and if @ref ctrl_compute_speed_order is called at
 * each cycle.
 *
 * To move back to a constant speed_order, user can unregister the callback
 * by calling this function and register a NULL speed_order callback.
 *
 * @param[in] ctrl              Controller object
 * @param[in] speed_order_cb    Function callback. (See @ref speed_order_cb_t )
 *
 * @return                      Nothing
 */
void ctrl_register_speed_order_cb(ctrl_t *ctrl, speed_order_cb_t speed_order_cb);

/**
 * @brief compute speed order
 *
 * This function computes speed order by calling callback registered with
 * @ref ctrl_register_speed_order_cb and set the new speed order accordingly.
 *
 * @param[in] ctrl              Controller object
 */
void ctrl_compute_speed_order(ctrl_t* ctrl);

/**
 * @brief Get current speed
 *
 * @param[in] ctrl              Controller object
 *
 * @return                      Current speed
 */
const polar_t* ctrl_get_speed_current(ctrl_t* ctrl);

/**
 * @brief Set current mode
 *
 * @param[in] ctrl              Controller object
 * @param[in] new_mode          New mode
 *
 * @return
 */
void ctrl_set_mode(ctrl_t *ctrl, ctrl_mode_t new_mode);

/**
 * @brief Get current mode
 *
 * @param[in] ctrl              Controller object
 *
 * @return                      Current mode
 */
ctrl_mode_t ctrl_get_mode(ctrl_t* ctrl);

/**
 * @brief Periodic task function to process a controller
 *
 * @param[in] arg               Controller object
 *
 * @return
 */
void *task_ctrl_update(void *arg);

/** @} */
