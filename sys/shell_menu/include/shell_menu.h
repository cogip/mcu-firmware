/*
 * Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    sys_shell_menu Shell menu
 * @ingroup     sys
 * @brief       Menus for shell
 *
 * @{
 *
 * @file
 * @brief       Shell menu interface definition
 *
 * @author      Eric Courtois <eric.courtois@gmail.com>
 */

#pragma once

/* RIOT includes */
#include "shell.h"

/* Project includes */
#include "utils.h"

/**
 * @brief       Max shell commands by menu
 */
#ifndef NB_SHELL_COMMANDS
#  define NB_SHELL_COMMANDS 20
#endif

/**
 * @brief       Max shell menus
 */
#ifndef NB_SHELL_MENUS
#  define NB_SHELL_MENUS 10
#endif

/**
 * @brief       Null command used to terminate arrays of commands
 */
#define MENU_NULL_CMD { NULL, NULL, NULL }

/**
 * @brief       Typedef for shell_menu identifier.
 */
typedef unsigned int shell_menu_t;

/**
 * @brief       Root menu.
 */
extern const shell_menu_t menu_root;

/**
 * @brief       Typedef for shell_menu structure.
 */
typedef struct shell_menu shell_menu_data_t;

/**
 * @brief       Shell menu linked list element.
 */
struct shell_menu {
    shell_command_t shell_commands[NB_SHELL_COMMANDS];  /**< Copy of the menu currently used */
    shell_menu_data_t *current;                         /**< Pointer to the real current shell_commands */
    shell_menu_data_t *previous;                        /**< Pointer to the real previous shell_commands */
    const char *name;                                   /**< Menu name */
    const char *cmd;                                    /**< Command to enter this menu */
    func_cb_t enter_cb;                                 /**< Function to execute at menu entry */
};

/**
 * @brief        Add a a list of global commands to all menus. Should be called before any menu_init().
 *
 * @param[in]    menu      menu
 * @param[in]    command   command to add
 */
void menu_set_global_commands(const shell_command_t command[]);

/**
 * @brief        Initialize a menu.
 *
 * @param[in]    name      name of the menu
 * @param[in]    cmd       shell command to enter the menu
 * @param[in]    parent    parent menu
 * @param[in]    enter_cb  callback function executed at menu entry (can be NULL)
 *
 * @return                 menu identifer
 */
shell_menu_t menu_init(const char *name, const char *cmd, const shell_menu_t parent, func_cb_t enter_cb);

/**
 * @brief        Add a command to a menu.
 *
 * @param[in]    menu      menu identifier
 * @param[in]    command   command to add
 */
void menu_add_one(const shell_menu_t menu, const shell_command_t *command);

/**
 * @brief        Add a list of command to a menu.
 *
 * @param[in]    menu       menu identifier
 * @param[in]    commands   list of commands to add
 */
void menu_add_list(const shell_menu_t menu, const shell_command_t commands[]);

/**
 * @brief        Enter a new menu.
 *
 * @param[in]    menu       new menu identifier
 */
void menu_enter(const shell_menu_t menu);

/**
 * @brief        Exit current menu and go back to previous menu.
 */
void menu_exit(void);

/**
 * @brief        Start the main menu.
 */
void menu_start(void);
