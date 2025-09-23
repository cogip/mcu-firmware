// Copyright (C) CONFIG_ACTUATOR_NUMBER25 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     actuator
/// @{
/// @file
/// @brief       Singleton managing GPIO-based limit switches for motors.
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// RIOT includes
#include <event.h>
#include <periph/gpio.h>

// Project includes
#include "actuator/Lift.hpp"

// Packages includes
#include "etl/map.h"
#include "etl/delegate.h"
#include "etl/pool.h"


namespace cogip {
namespace actuators {
namespace positional_actuators {

/// @brief Singleton managing GPIO-based limit switches for motors.
/// @details This class handles GPIO interrupts via a dedicated thread,
///          allowing motor control to react to events such as limit switches
///          in a safe, non-blocking, and asynchronous way.
class LiftsLimitSwitchesManager {
public:
    /// @brief Get the singleton instance of the manager.
    /// @return Reference to the LiftsLimitSwitchesManager singleton.
    static LiftsLimitSwitchesManager& instance();

    /// @brief Initialize the manager by creating the event thread and queue.
    void init();

    /// @brief Register a GPIO pin for falling-edge interrupts with a Lift handler.
    /// @details Configures the pin with pulldown and sets up the ISR to enqueue
    ///          events which are later dispatched in thread context.
    /// @param pin GPIO pin to register.
    /// @param lift Pointer to the Lift actuator instance to notify on trigger.
    /// @return 0 on success,
    ///         -ENODEV if GPIO is undefined,
    ///         -EIO on gpio init failed
    ///         -ENOMEM on event creation failed
    int register_gpio(gpio_t pin, Lift* lift);

    /// @brief Unregister a previously registered GPIO pin.
    /// @details Disables the interrupt, releases the associated event, and removes
    ///          all internal references to the pin and its callback.
    /// @param pin GPIO pin to unregister.
    /// @return 0 on success,
    ///         -ENODEV if GPIO is undefined,
    ///         -ENOENT if GPIO was not previously registered.
    int unregister_gpio(gpio_t pin);

    /// @brief Event handler executed in the event thread context.
    /// @param evt Pointer to the event to handle.
    static void event_handler(event_t* evt);

private:
    /// @brief Private constructor to enforce singleton pattern.
    LiftsLimitSwitchesManager() : mutex_(MUTEX_INIT), event_stack_{}, event_thread_pid_(KERNEL_PID_UNDEF) {}

    /// @brief GPIO ISR callback invoked in interrupt context.
    /// @details Enqueues the corresponding event for later handling.
    /// @param arg Pointer to the GPIO pin wrapped as void*.
    static void isr_callback(void* arg);

    /// @brief Entry function for the event processing thread.
    /// @param arg Unused thread argument.
    /// @return nullptr on exit.
    static void* event_thread_entry(void* arg);

    /// @brief Dispatches a GPIO event to the registered Lift callback.
    /// @param pin GPIO pin whose event occurred.
    void handle_gpio_event(gpio_t pin);

    /// @brief Mutex protecting access to callback and event maps.
    mutex_t mutex_;

    /// @brief Map from GPIO pin to associated Lift actuator pointer.
    /// @note Consider 2 GPIOs per lift.
    etl::map<gpio_t, Lift*, CONFIG_ACTUATOR_LIFT_NUMBER * 2> callbacks_;

    /// @brief Map from GPIO pin to allocated event pointer.
    /// @note Consider 2 GPIOs per lift.
    etl::map<gpio_t, event_t*, CONFIG_ACTUATOR_LIFT_NUMBER * 2> gpio_to_event_;

    /// @brief Map from event pointer back to its GPIO pin.
    /// @note Consider 2 GPIOs per lift.
    etl::map<event_t*, gpio_t, CONFIG_ACTUATOR_LIFT_NUMBER * 2> event_to_gpio_;

    /// @brief Pool of pre-allocated event objects.
    /// @note Consider 2 GPIOs per lift.
    etl::pool<event_t, CONFIG_ACTUATOR_LIFT_NUMBER * 2> event_pool_;

    /// @brief Internal event queue for dispatching GPIO events.
    event_queue_t event_queue_;

    /// @brief Stack memory for the event thread.
    char event_stack_[THREAD_STACKSIZE_MAIN];

    /// @brief PID of the event thread.
    kernel_pid_t event_thread_pid_;
};

} // namespace positional_actuators
} // namespace actuators
} // namespace cogip

/// @}
