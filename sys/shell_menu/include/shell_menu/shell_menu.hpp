// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @defgroup    sys_shell_menu Shell menu
/// @ingroup     sys
/// @brief       Menus for shell
/// @{
/// @file
/// @brief       Public functions from shell_menu namespace
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#include "shell_menu/Command.hpp"
#include "shell_menu/Menu.hpp"

/// @def NB_SHELL_COMMANDS
/// @brief Max shell commands by menu
#ifndef NB_SHELL_COMMANDS
#  define NB_SHELL_COMMANDS 20
#endif

namespace cogip {

namespace shell {

/// Root menu
extern Menu root_menu;

/// @brief        Start the main menu.
void start(void);

/// @brief        Add a global command that will be available in all menus.
/// @param[in]    command   global command to add
void add_global_command(Command * global_command);

/// @brief        Rename a command.
/// @param[in]    old_name  old command name
/// @param[in]    new_name  new command name
void rename_command(const std::string &old_name, const std::string &new_name);

} // namespace shell

} // namespace cogip

/// @}
