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

/**
 * @brief       Max shell commands
 */
#ifndef NB_SHELL_COMMANDS
#  define NB_SHELL_COMMANDS 20
#endif

/**
 * @brief       Typedef for shell_menu structure.
 */
typedef struct shell_menu shell_menu_t;

/**
 * @brief       Shell menu linked list element.
 */
struct shell_menu {
    shell_command_t shell_commands[NB_SHELL_COMMANDS]; /**< Copy of the menu currently used */
    shell_menu_t *current;  /**< Pointer to the real current shell_commands */
    shell_menu_t *previous; /**< Pointer to the real previous shell_commands */
    const char *name;       /**< Menu name */
};

/**
 * @brief        Null command used to terminate a command list.
 */
extern const shell_command_t menu_cmd_null;

/**
 * @brief        Initialize main root menu.
 *
 * @param[in]    name      name of the menu
 */
void menu_init(const char *name);

/**
 * @brief        Initialize a menu.
 *
 * @param[in]    menu      menu to initialize
 * @param[in]    name      name of the menu
 */
void menu_init_menu(shell_menu_t *menu, const char *name);

/**
 * @brief        Add a command to a menu.
 *
 * @param[in]    menu      menu
 * @param[in]    command   command to add
 */
void menu_add_one(shell_menu_t *menu, const shell_command_t *command);

/**
 * @brief        Add a list of command to a menu.
 *
 * @param[in]    menu       menu
 * @param[in]    commands   list of commands to add
 */
void menu_add_list(shell_menu_t *menu, const shell_command_t commands[]);

/**
 * @brief        Enter a new menu.
 *
 * @param[in]    menu       new menu
 */
void menu_enter(shell_menu_t *menu);

/**
 * @brief        Exit current menu and go back to previous menu.
 */
void menu_exit(void);

/**
 * @brief        Start the main menu.
 */
void menu_start(void);

/**
 * @brief        Return main root menu
 *
 * @return       pointer on main_menu
 */
shell_menu_t *menu_get_main_menu(void);
