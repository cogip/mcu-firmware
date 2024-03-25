// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     sys_shell_menu
/// @{
/// @file
/// @brief       Menu class declaration
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#include "shell_menu/Command.hpp"

// System includes
#include <etl/list.h>
#include <etl/string.h>

// Project includes
#include "utils.hpp"

#ifndef NB_SHELL_COMMANDS
#  define NB_SHELL_COMMANDS 20    ///< max shell commands per menu
#endif

#ifndef NB_SHELL_MENUS
#  define NB_SHELL_MENUS 10       ///< max shell menus
#endif

namespace cogip {

namespace shell {

/// Menu class.
/// A menu contains a list of Command class.
/// It has a name and a command to enter it.
/// A callback can be specified to be executed at menu entry.
/// It is linked to its parent menu.
class Menu : public etl::list<Command *, NB_SHELL_COMMANDS> {
public:
    /// Constructor.
    Menu(
        const etl::string<COMMAND_NAME_MAX_LENGTH> &name, ///< [in] name of the menu
        const etl::string<COMMAND_NAME_MAX_LENGTH> &cmd,  ///< [in] command name to enter the menu
        Menu *parent = nullptr,                           ///< [in] parent menu (optional)
        func_cb_t enter_cb = nullptr                      ///< [in] callback function executed at menu entry (optional)
        );

    /// Enter this menu.
    void enter(void);

    /// Return the name of this menu.
    const etl::string<COMMAND_NAME_MAX_LENGTH> & name(void) const { return name_; };

    /// Return the parent of this menu.
    Menu * parent(void) const { return parent_; };

    /// Update the Protobuf message describing this menu.
    void update_pb_message(void);

#ifdef MODULE_CANPB
    /// Send the Protobuf message describing this menu if canpb is registered.
    void send_pb_message(void);
#endif

private:
    etl::string<COMMAND_NAME_MAX_LENGTH> name_;  ///< menu name
    etl::string<COMMAND_NAME_MAX_LENGTH> cmd_;   ///< command to enter this menu
    Menu *parent_;                               ///< pointer to the parent menu
    func_cb_t enter_cb_;                         ///< function to execute at menu entry
    Command enter_cmd_;                          ///< command used to enter this menu
};

} // namespace shell

} // namespace cogip

/// @}
