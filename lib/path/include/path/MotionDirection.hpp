// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     lib_path
/// @{
/// @file
/// @brief       Motion direction enum declaration
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <cstdint>

namespace cogip {
namespace path {

/// @brief Motion direction mode for path navigation
enum class motion_direction : uint8_t {
    bidirectional = 0, ///< Robot can move forward or backward (choose optimal)
    forward_only = 1,  ///< Force forward motion only
    backward_only = 2  ///< Force backward motion only
};

} // namespace path
} // namespace cogip

/// @}
