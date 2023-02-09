// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    motion_control_common
/// @{
/// @file
/// @brief      Base class for engines
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include "Controller.hpp"

namespace cogip {

namespace motion_control {

/// Base class for controllers engine. The engine is responsible of launching the controllers chain.
class BaseControllerEngine {
public:
    /// Constructor
    BaseControllerEngine() : controller_(nullptr), current_cycle_(0) {};

    /// Set the controller to launch.
    /// @param controller
    void set_controller(BaseController *controller);

    /// Start controller main thread, launching the thread loop.
    void start_thread();

    /// Engine thread loop with period internally defined.
    virtual void thread_loop();

    /// Return motion control current cycle
    uint32_t current_cycle() const { return current_cycle_; };

    /// Get pose reached flag
    /// return     true if pose reached
    target_pose_status_t pose_reached() const { return pose_reached_; };

protected:
    /// Prepare controller inputs from platform functions.
    virtual void prepare_inputs() = 0;

    /// Process controller output for platform restitution.
    virtual void process_outputs() = 0;

    /// Controller to be launched by this engine. This could be a meta controller leading to the execution of a chain of controllers.
    BaseController *controller_;

    /// Current motion control cycle
    uint32_t current_cycle_;
};

} // namespace motion_control

} // namespace cogip

/// @}
