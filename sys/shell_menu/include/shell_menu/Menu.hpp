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
#include <string>
#include <list>

// Project includes
#include "utils.h"

namespace cogip {

namespace shell {

/// Menu class.
/// A menu contains a list of Command class.
/// It has a name and a command to enter it.
/// A callback can be specified to be executed at menu entry.
/// It is linked to its parent menu.
class Menu : public std::list<Command *> {
public:
    /// Constructor.
    Menu(
        const std::string &name,      ///< [in] name of the menu
        const std::string &cmd,       ///< [in] command name to enter the menu
        Menu *parent = nullptr,       ///< [in] parent menu (optional)
        func_cb_t enter_cb = nullptr  ///< [in] callback function executed at menu entry (optional)
        );

    /// Enter this menu.
    void enter(void) const;

    /// Return the name of this menu.
    const std::string & name(void) const { return name_; };

    /// Return the parent of this menu.
    const Menu * parent(void) const { return parent_; };

private:
    std::string name_;                ///< menu name
    std::string cmd_;                 ///< command to enter this menu
    Menu *parent_;                    ///< pointer to the parent menu
    func_cb_t enter_cb_;              ///< function to execute at menu entry
};

} // namespace shell

} // namespace cogip

/// @}
