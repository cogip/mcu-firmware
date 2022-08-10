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

#include <etl/map.h>
#include <etl/set.h>
#include <array>

namespace cogip {

namespace shell {

etl::map<etl::string<COMMAND_NAME_MAX_LENGTH>, Menu *, NB_SHELL_MENUS> & all_menus();  ///< map containing all menus indexed by cmd
etl::set<Command *, NB_SHELL_COMMANDS * NB_SHELL_MENUS> & all_commands();              ///< all commands

#ifdef MODULE_UARTPB
extern cogip::uartpb::UartProtobuf *uart_protobuf;  ///< UartProtocol instance used to send new menu over UART
#endif

/// Shell commands used by RIOT shell module.
/// It is updated each time a menu is entered or exited.
extern shell_command_t current_commands[NB_SHELL_COMMANDS];

} // namespace shell

} // namespace cogip

/// @}
