// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @defgroup    sys_shell_menu Shell menu
/// @ingroup     sys
/// @brief       Menus for shell
/// @{
/// @file
/// @brief       Shell menu interface definition
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

// System includes
#include <string>
#include <list>

// RIOT includes
#include "shell.h"

// Project includes
#include "utils.h"

/// @def NB_SHELL_COMMANDS
/// @brief Max shell commands by menu
#ifndef NB_SHELL_COMMANDS
#  define NB_SHELL_COMMANDS 20
#endif

namespace cogip {

namespace shell {

class command {
public:
    /// @brief        Constructor.
    /// @param[in]    name      command name
    /// @param[in]    desc      description to print in the "help" command.
    /// @param[in]    handler   the callback function
    command(const std::string &name, const std::string &desc, shell_command_handler_t handler);

    /// @brief        Destructor.
    ~command();

    /// @brief        Return the name of this command.
    const std::string & name(void) const { return name_; };

    /// @brief        Return the name of this menu.
    /// @param[in]    name      new command name
    void set_name(const std::string & name) { name_ = name; };

    /// @brief        Return the description of this command.
    const std::string & desc(void) const { return desc_; };

    /// @brief        Return the name of this command.
    shell_command_handler_t handler(void) const { return handler_; };

private:
    std::string name_;                 /// name of the command
    std::string desc_;                 /// description to print in the "help" command
    shell_command_handler_t handler_;  /// the callback function
};

class menu : public std::list<command *> {
public:
    /// @brief        Constructor.
    /// @param[in]    name      name of the menu
    /// @param[in]    cmd       command name to enter the menu
    /// @param[in]    parent    parent menu (optional)
    /// @param[in]    enter_cb  callback function executed at menu entry (optional)
    menu(const std::string &name, const std::string &cmd,
         menu *parent = nullptr, func_cb_t enter_cb = nullptr);

    /// @brief        Enter this menu.
    void enter(void) const;

    /// @brief        Return the name of this menu.
    const std::string & name(void) const { return name_; };

    /// @brief        Return the parent of this menu.
    const menu * parent(void) const { return parent_; };

private:
    std::string name_;                 /// menu name
    std::string cmd_;                  /// command to enter this menu
    menu *parent_;                     /// pointer to the parent menu
    func_cb_t enter_cb_;               /// function to execute at menu entry
};

/// @brief        Start the main menu.
void start(void);

/// @brief        Add a global command that will be available in all menus.
/// @param[in]    command   global command to add
void add_global_command(command * global_command);

/// @brief        Rename a command.
/// @param[in]    old_name  old command name
/// @param[in]    new_name  new command name
void rename_command(const std::string &old_name, const std::string &new_name);

extern menu root_menu;  /// Root menu

} // namespace shell

} // namespace cogip
