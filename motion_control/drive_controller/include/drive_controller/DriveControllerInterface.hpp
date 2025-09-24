// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @brief       Drive controller interface
/// @author      Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

#include "cogip_defs/Polar.hpp"

namespace cogip {

namespace drive_controller {

class DriveControllerInterface
{
  public:
    /// @brief Virtual destructor
    ~DriveControllerInterface() {}

    /// @brief Set polar velocity command
    /// @param command polar velocity command reference
    /// @return  0 on success, negative on error
    virtual int set_polar_velocity(cogip_defs::Polar& command) = 0;
};

} // namespace drive_controller

} // namespace cogip

// @}
