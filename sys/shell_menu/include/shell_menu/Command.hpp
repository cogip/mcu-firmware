// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     sys_shell_menu
/// @{
/// @file
/// @brief       Command class declaration
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

// System includes
#include <string>

// RIOT includes
#include "shell.h"

namespace cogip {

namespace shell {

class Command {
public:
    /// @brief        Constructor.
    /// @param[in]    name      command name
    /// @param[in]    desc      description to print in the "help" command.
    /// @param[in]    handler   the callback function
    Command(const std::string &name, const std::string &desc, shell_command_handler_t handler);

    /// @brief        Destructor.
    ~Command();

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

} // namespace shell

} // namespace cogip

/// @}
