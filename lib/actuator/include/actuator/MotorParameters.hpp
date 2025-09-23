/// @ingroup     actuator
/// @{
/// @file
/// @brief       Configuration structure for initializing a Motor object.
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// System includes
#include <cstdint>

// RIOT includes
#include <motor_driver.h>
#include <periph/gpio.h>

// Project includes
#include "Actuator.hpp"
#include "motor/MotorInterface.hpp"
#include "odometer/OdometerInterface.hpp"
// Motion control
#include "motor_pose_filter/MotorPoseFilterParameters.hpp"
#include "pose_pid_controller/PosePIDControllerParameters.hpp"
#include "speed_filter/SpeedFilterParameters.hpp"
#include "speed_pid_controller/SpeedPIDControllerParameters.hpp"

namespace cogip {
namespace actuators {
namespace positional_actuators {

/// @struct MotorParameters
/// @brief Configuration structure for initializing a Motor object.
/// @details
///   Gathers all necessary parameters to configure and instantiate a Motor,
///   including control, filtering, timeout, and hardware interface options.
struct MotorParameters
{
    /// @brief Motor identifier (enum used internally to differentiate motors).
    Enum id;

    /// @brief Default timeout period, in milliseconds.
    /// @details Used to determine if the motor should be disabled due to
    /// inactivity.
    const uint32_t default_timeout_ms = 0;

    /// @brief Callback used to send the actuator state.
    send_state_cb_t send_state_cb;

    /// @brief GPIO pin used to clear the motor's overload state.
    gpio_t clear_overload_pin = GPIO_UNDEF;

    /// @brief Parameters for the PosePIDController (position control).
    motion_control::PosePIDControllerParameters& pose_controller_parameters;

    /// @brief Parameters for the SpeedPIDController (speed control).
    motion_control::SpeedPIDControllerParameters& speed_controller_parameters;

    /// @brief Parameters for the MotorPoseFilter (used to smooth and validate
    /// position).
    motion_control::MotorPoseFilterParameters& motor_pose_filter_parameters;

    /// @brief Parameters for the SpeedFilter (used to smooth and limit speed).
    motion_control::SpeedFilterParameters& speed_filter_parameters;

    /// @brief Period of the motor engine thread, in milliseconds.
    uint32_t engine_thread_period_ms;

    /// @brief Reference to the motor driver interface.
    motor::MotorInterface& motor;

    /// @brief Reference to the odometer interface.
    localization::OdometerInterface& odometer;
};

} // namespace positional_actuators
} // namespace actuators
} // namespace cogip

/// @}
