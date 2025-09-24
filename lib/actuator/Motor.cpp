// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "actuator/Motor.hpp"
#include "actuator/PositionalActuator.hpp"
#include "board.h"
#include "log.h"
#include "motor_pose_filter/MotorPoseFilterIOKeysDefault.hpp"
#include "pose_pid_controller/PosePIDControllerIOKeysDefault.hpp"
#include "speed_filter/SpeedFilterIOKeysDefault.hpp"
#include "speed_pid_controller/SpeedPIDControllerIOKeysDefault.hpp"
#include <inttypes.h>

namespace cogip {
namespace actuators {
namespace positional_actuators {

/// @brief Construct a Motor by unpacking the static parameters and starting the
/// control thread.
/// @param motor_parameters  Reference to a `MotorParameters` in static storage.
Motor::Motor(const MotorParameters& motor_parameters)
    : PositionalActuator(motor_parameters.id, motor_parameters.default_timeout_ms,
                         motor_parameters.send_state_cb),
      params_(motor_parameters),
      distance_controller_(motion_control::linear_pose_pid_controller_io_keys_default,
                           motor_parameters.pose_controller_parameters),
      speed_controller_(motion_control::linear_speed_pid_controller_io_keys_default,
                        motor_parameters.speed_controller_parameters),
      motor_distance_filter_(motion_control::motor_pose_filter_io_keys_default,
                             motor_parameters.motor_pose_filter_parameters),
      speed_filter_(motion_control::linear_speed_filter_io_keys_default,
                    motor_parameters.speed_filter_parameters),
      motor_engine_(motor_parameters.motor, motor_parameters.odometer,
                    motor_parameters.engine_thread_period_ms)
{
    // Build the cascade: MotorPoseFilter → PosePID → SpeedFilter → SpeedPID
    dualpid_meta_controller_.add_controller(&motor_distance_filter_);
    dualpid_meta_controller_.add_controller(&distance_controller_);
    dualpid_meta_controller_.add_controller(&speed_filter_);
    dualpid_meta_controller_.add_controller(&speed_controller_);

    motor_engine_.set_controller(&dualpid_meta_controller_);

    // Init motor
    int ret = params_.motor.init();
    if (ret) {
        LOG_ERROR("Motor init failed\n");
    }
    // Init encoder
    ret = params_.odometer.init();
    if (ret) {
        LOG_ERROR("Odometer init failed\n");
    }

    // Ensure overload flag is cleared at startup
    gpio_init(params_.clear_overload_pin, GPIO_OUT);
    gpio_clear(params_.clear_overload_pin);

    // Start disabled and spin up the control thread
    disable();
    motor_engine_.start_thread();
}

void Motor::init()
{
    params_.motor.enable();
    motor_engine_.enable();
}

void Motor::enable()
{
    motor_engine_.enable();
}

void Motor::disable()
{
    motor_engine_.disable();
    params_.motor.brake();
}

void Motor::actuate(int32_t command)
{
    LOG_INFO("Move motor to command %" PRIi32 " from distance %.2f\n", command,
             static_cast<double>(motor_engine_.get_current_distance_from_odometer()));

    // Reset filters/PIDs on a new command
    distance_controller_.parameters().pid()->reset();
    speed_controller_.parameters().pid()->reset();
    speed_filter_.reset_previous_speed_order();
    speed_filter_.reset_anti_blocking_blocked_cycles_nb();

    // Apply new target distance
    command_ = command;

    // If no timeout set, use default
    if (!timeout_ms_) {
        timeout_ms_ = params_.default_timeout_ms;
    }

    LOG_INFO("Timeout is set to %" PRIu32 "ms\n", timeout_ms_);

    // Apply timeout
    motor_engine_.set_timeout_ms(timeout_ms_);
    // Reset timeout once used to avoid using it twice
    timeout_ms_ = 0;
    // Enable timeout
    motor_engine_.set_timeout_enable(true);

    // Timeout security is setup, set target distance to reach
    motor_engine_.set_target_distance(command_);

    // Enable engine in case it has been previously disabled by timeout
    motor_engine_.enable();

    // Enable motor
    params_.motor.enable();
}

float Motor::get_target_speed_percentage() const
{
    return params_.speed_filter_parameters.max_speed() == 0.0f
               ? 0.0f
               : (motor_engine_.target_speed() / params_.speed_filter_parameters.max_speed()) *
                     100.0f;
}

void Motor::set_target_speed_percent(float percentage)
{
    motor_engine_.set_target_speed((percentage / 100.0f) *
                                   params_.speed_filter_parameters.max_speed());
}

float Motor::get_current_distance() const
{
    return motor_engine_.get_current_distance_from_odometer();
}

void Motor::set_current_distance(float distance)
{
    motor_engine_.set_current_distance_to_odometer(distance);
}

} // namespace positional_actuators
} // namespace actuators
} // namespace cogip
