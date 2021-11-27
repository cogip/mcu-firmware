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

namespace cogip {

namespace shell {

extern Menu root_menu;            ///< root menu

/// Start the main menu.
void start(void);

/// Add a global command that will be available in all menus.
void add_global_command(
    Command * global_command      ///< [in] global command to add
    );

/// Rename a command.
void rename_command(
    const std::string &old_name,  ///< [in] old command name
    const std::string &new_name   ///< [in] new command name
    );

/// Register an UartProtobuf instance that will be used to send entered menus
/// in Protobuf format over UART.
void register_uartpb(
    cogip::uartpb::UartProtobuf *uartpb_ptr
    );

} // namespace shell

} // namespace cogip

/// @}
