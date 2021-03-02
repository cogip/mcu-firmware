/*
 * Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    platforms_calibration Common platform calibration
 * @ingroup     platforms
 * @brief       Common platform calibration with shell management
 *
 * The common platform calibration code defines shell commands to retrieve
 * informations from the platform component which has a central role.
 * For now the shell has the following commands:
 * * _dyn_obstacles : Print dynamic obstacles
 * * _help_json     : Display available commands in JSON format
 * * _pose          : Print current pose
 * * _set_shm_key   : Set shared memory key to communicate with simulator
 *
 * @{
 * @file
 * @brief       Common platform calibration headers
 *
 * @author      Eric Courtois <eric.courtois@gmail.com>
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */
/* Project includes */
#include "ctrl.h"
#include "platform.h"

void pf_init_calib_tasks(ctrl_t* pf_ctrl);
void pf_calib_init(void);

/**
 * @brief Chained list to store commands through modules declaration
 */
typedef struct shell_command_linked shell_command_linked_t;

/**
 * @brief Chained list to store commands through modules
 */
struct shell_command_linked {
    shell_command_t shell_commands[NB_SHELL_COMMANDS];
                            /**< Copy of the shell_commands currently used */
    shell_command_linked_t *current;
                            /**< Pointer to the real current shell_commands */
    shell_command_linked_t *previous;
                            /**< Pointer to the real previous shell_commands */
    const char *name;
                            /**< Menu name */
};

/* TODO: These functions/structs should be moved to common code */
void pf_push_shell_commands(shell_command_linked_t *shell_commands);
void pf_pop_shell_commands(void);
void pf_init_shell_commands(shell_command_linked_t *shell_commands, const char *name);
void pf_add_shell_command(shell_command_linked_t *shell_commands, const shell_command_t *command);
int pf_display_json_help(int argc, char **argv);
int pf_exit_shell(int argc, char **argv);
int pf_print_state_cb(int argc, char **argv);

extern shell_command_t cmd_help_json;
extern shell_command_t cmd_exit_shell;
extern shell_command_t cmd_print_state;
extern shell_command_t cmd_motors_test;
extern shell_command_t cmd_print_dyn_obstacles;
extern shell_command_linked_t pf_shell_commands;

/** @} */
