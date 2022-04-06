// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     sys_shell_menu
/// @{
/// @file
/// @brief       Shell menu private variables
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#include "shell_menu/shell_menu.hpp"

#ifdef MODULE_UARTPB
#include "uartpb/UartProtobuf.hpp"
#endif

#include <map>
#include <set>

namespace cogip {

namespace shell {

extern std::map<std::string, Menu *> all_menus;     ///< map containing all menus indexed by cmd
extern std::set<Command *> all_commands;            ///< all commands

#ifdef MODULE_UARTPB
extern cogip::uartpb::UartProtobuf *uart_protobuf;  ///< UartProtocol instance used to send new menu over UART
#endif

/// Shell commands used by RIOT shell module.
/// It is updated each time a menu is entered or exited.
extern shell_command_t current_commands[NB_SHELL_COMMANDS];

} // namespace shell

} // namespace cogip

/// @}
