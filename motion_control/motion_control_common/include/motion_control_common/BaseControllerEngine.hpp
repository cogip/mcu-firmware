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
#include "thread/thread.hpp"

// RIOT includes
#include <mutex.h>

namespace cogip {

namespace motion_control {

/// Base class for controllers engine. The engine is responsible of launching the controllers chain.
class BaseControllerEngine {
public:
    /// Constructor
    BaseControllerEngine() :
        enable_(true),
        controller_(nullptr),
        current_cycle_(0),
        pose_reached_(moving),
        timeout_cycle_number_(0),
        timeout_enable_(false)
        {
            memset(controller_thread_stack_, 0, sizeof(controller_thread_stack_));
            mutex_init(&mutex);
        };

    /// Set the controller to launch.
    void set_controller(
        BaseController *controller  ///< [in]   controller
        );

    /// Start controller main thread, launching the thread loop.
    void start_thread();

    /// Engine thread loop with period internally defined.
    virtual void thread_loop();

    /// Enable thread loop
    void enable() { mutex_lock(&mutex); enable_ = true; mutex_unlock(&mutex); };

    /// Disable thread loop
    void disable() { mutex_lock(&mutex); enable_ = false; mutex_unlock(&mutex); };

    /// Get controller
    BaseController* controller() const { return controller_; };

    /// Get pose reached flag
    /// return     true if pose reached
    target_pose_status_t pose_reached() const { return pose_reached_; };

    /// Return motion control current cycle
    uint32_t current_cycle() const { return current_cycle_; };

    /// Return motion control timeout cycle number
    uint32_t timeout_cycle_number() const { return timeout_cycle_number_; };

    /// Return motion control timeout enable flag
    bool timeout_enable() const { return timeout_enable_; };

    /// Set pose reached flag
    void set_pose_reached(
        target_pose_status_t pose_reached       ///< [in]   pose reached flag
        ) { pose_reached_ = pose_reached; };

    /// Set current cycle
    void set_current_cycle(
        uint32_t current_cycle                  ///< [in]   new current cycle
        ) { current_cycle_ = current_cycle; };

    /// Set timeout cycle number
    void set_timeout_cycle_number(
        uint32_t timeout_cycle_number           ///< [in]   timeout in cycles
        ) { timeout_cycle_number_ = timeout_cycle_number; };

    /// Set timeout enable
    void set_timeout_enable(
        bool timeout_enable                     ///< [in]   timeout enable flag
        ) { timeout_enable_ = timeout_enable; };

protected:
    /// Prepare controller inputs from platform functions.
    virtual void prepare_inputs() = 0;

    /// Process controller output for platform restitution.
    virtual void process_outputs() = 0;

    /// Enable thread loop flag
    bool enable_;

    /// Controller to be launched by this engine. This could be a meta controller leading to the execution of a chain of controllers.
    BaseController *controller_;

    /// Current motion control cycle
    uint32_t current_cycle_;

    /// Pose reached flag
    target_pose_status_t pose_reached_;

    /// Timeout before the engine considers it has reached the position, useful for speed only controllers.
    uint32_t timeout_cycle_number_;

    /// Timeout enable flag
    bool timeout_enable_;

    /// Controller thread stack
    char controller_thread_stack_[THREAD_STACKSIZE_LARGE];

    /// Mutex protecting engine loop
    mutex_t mutex;
};

} // namespace motion_control

} // namespace cogip

/// @}
