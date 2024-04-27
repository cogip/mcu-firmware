// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pami
/// @{
/// @file
/// @brief       Base actuators class definition.
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

namespace cogip {
namespace pf {
namespace actuators {

class Actuator {
public:
    /// Constructor
    explicit Actuator() : blocked_(0) {};

    /// Get blocked state
    bool blocked() { return blocked_; }

    /// Set blocked state
    void set_blocked(bool blocked) { blocked_ = blocked; }

protected:
    bool blocked_;      /// blocked state
};

} // namespace actuators
} // namespace pf
} // namespace cogip

/// @}
