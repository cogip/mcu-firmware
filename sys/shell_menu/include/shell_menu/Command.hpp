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

#include "PB_Command.hpp"

#ifndef COMMAND_NAME_MAX_LENGTH
#  define COMMAND_NAME_MAX_LENGTH 16
#endif

#ifndef COMMAND_DESC_MAX_LENGTH
#  define COMMAND_DESC_MAX_LENGTH 128
#endif

namespace cogip {

namespace shell {

/// Command class.
/// This class is a C++ wrapper around RIOT shell_command_t struct.
class Command {
public:
    /// Protobuf message type. Shortcut for original template type.
    using PB_Message = PB_Command<COMMAND_NAME_MAX_LENGTH, COMMAND_DESC_MAX_LENGTH>;

    /// Constructor.
    Command(
        const std::string &name,         ///< [in] command name
        const std::string &desc,         ///< [in] description to print in the "help" command
        shell_command_handler_t handler  ///< [in] the callback function
        );

    ///Destructor.
    ~Command();

    /// Return the name of this command.
    const std::string & name(void) const { return name_; };

    /// Return the name of this menu.
    void set_name(
        const std::string & name         ///< [in] new command name
        ) { name_ = name; };

    /// Return the description of this command.
    const std::string & desc(void) const { return desc_; };

    /// Return the name of this command.
    shell_command_handler_t handler(void) const { return handler_; };

    /// Update the Protobuf message describing this menu.
    void update_pb_message(void);

    /// Return the Protobuf message.
    const PB_Message &pb_message(void) const { return pb_message_; };

private:
    std::string name_;                   ///< name of the command
    std::string desc_;                   ///< description to print in the "help" command
    shell_command_handler_t handler_;    ///< the callback function
    PB_Message pb_message_;
};

} // namespace shell

} // namespace cogip

/// @}
