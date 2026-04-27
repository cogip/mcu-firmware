// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "actuator/Motor.hpp"
#include "actuator/MotorIOKeys.hpp"
#include "actuator/PositionalActuator.hpp"
#include "board.h"
#include "log.h"
#include <inttypes.h>

namespace cogip {
namespace actuators {
namespace positional_actuators {

// Default parameters for tracker chain (used if not provided in MotorParameters)
static motion_control::ProfileTrackerControllerParameters
    default_profile_tracker_params(10.0f, // max_speed (mm/period)
                                   1.0f,  // acceleration (mm/period²)
                                   1.0f,  // deceleration (mm/period²)
                                   true,  // must_stop_at_end
                                   1      // period_increment
    );

static motion_control::TrackerCombinerControllerParameters default_tracker_combiner_params;

/// @brief Construct a Motor by unpacking the static parameters and starting the
/// control thread.
/// @param motor_parameters  Reference to a `MotorParameters` in static storage.
/// @param mode Control mode (DUALPID or DUALPID_TRACKER)
Motor::Motor(const MotorParameters& motor_parameters, MotorControlMode mode)
    : PositionalActuator(motor_parameters.id, motor_parameters.default_timeout_ms,
                         motor_parameters.send_state_cb),
      params_(motor_parameters), control_mode_(mode),
      // Classic DualPID chain controllers
      distance_controller_(actuators::motor_pose_pid_io_keys,
                           motor_parameters.pose_controller_parameters),
      speed_controller_(actuators::motor_speed_pid_io_keys,
                        motor_parameters.speed_controller_parameters),
      motor_distance_filter_(actuators::motor_pose_filter_io_keys,
                             motor_parameters.motor_pose_filter_parameters),
      speed_filter_(actuators::motor_speed_filter_io_keys,
                    motor_parameters.speed_filter_parameters),
      // Tracker chain controllers
      profile_tracker_controller_(actuators::motor_profile_tracker_io_keys,
                                  motor_parameters.profile_tracker_parameters
                                      ? *motor_parameters.profile_tracker_parameters
                                      : default_profile_tracker_params),
      tracker_pose_controller_(actuators::motor_tracker_pose_pid_io_keys,
                               motor_parameters.pose_controller_parameters),
      tracker_combiner_controller_(actuators::motor_tracker_combiner_io_keys,
                                   motor_parameters.tracker_combiner_parameters
                                       ? *motor_parameters.tracker_combiner_parameters
                                       : default_tracker_combiner_params),
      tracker_speed_controller_(actuators::motor_tracker_speed_pid_io_keys,
                                motor_parameters.speed_controller_parameters),
      tracker_motor_distance_filter_(actuators::motor_tracker_pose_filter_io_keys,
                                     motor_parameters.motor_pose_filter_parameters),
      // Brake chain: zero speed_order + dedicated speed PID. Separate PID
      // instance from the tracking loop so the static-hold gains (typically a
      // higher Ki to counter constant disturbances like gravity) do not couple
      // their integrator state with trajectory tracking.
      brake_zero_speed_order_controller_({{"speed_order", 0.0f}}),
      brake_speed_controller_(actuators::motor_speed_pid_io_keys,
                              motor_parameters.brake_speed_controller_parameters),
      // Motor engine
      motor_engine_(motor_parameters.motor, motor_parameters.odometer,
                    motor_parameters.engine_thread_period_ms)
{
    if (control_mode_ == MotorControlMode::DUALPID_TRACKER) {
        // Build tracker chain:
        // MotorPoseFilter → ProfileTracker → PosePID → Combiner →
        //   [SpeedLimitFilter] → [AccelerationFilter] → SpeedPID
        tracker_meta_controller_.add_controller(&tracker_motor_distance_filter_);
        tracker_meta_controller_.add_controller(&profile_tracker_controller_);
        tracker_meta_controller_.add_controller(&tracker_pose_controller_);
        tracker_meta_controller_.add_controller(&tracker_combiner_controller_);
        // Safety filters (if configured) - applied to speed_order before SpeedPID
        if (motor_parameters.deceleration_filter_parameters) {
            tracker_deceleration_filter_.emplace(actuators::motor_tracker_deceleration_io_keys,
                                                 *motor_parameters.deceleration_filter_parameters);
            tracker_meta_controller_.add_controller(&tracker_deceleration_filter_.value());
        }
        if (motor_parameters.speed_limit_filter_parameters) {
            tracker_speed_limit_filter_.emplace(actuators::motor_tracker_speed_limit_io_keys,
                                                *motor_parameters.speed_limit_filter_parameters);
            tracker_meta_controller_.add_controller(&tracker_speed_limit_filter_.value());
        }
        if (motor_parameters.acceleration_filter_parameters) {
            tracker_acceleration_filter_.emplace(actuators::motor_tracker_acceleration_io_keys,
                                                 *motor_parameters.acceleration_filter_parameters);
            tracker_meta_controller_.add_controller(&tracker_acceleration_filter_.value());
        }
        tracker_meta_controller_.add_controller(&tracker_speed_controller_);
        if (motor_parameters.anti_blocking_parameters) {
            anti_blocking_controller_.emplace(actuators::motor_tracker_anti_blocking_io_keys,
                                              *motor_parameters.anti_blocking_parameters);
            tracker_meta_controller_.add_controller(&anti_blocking_controller_.value());
        }

        motor_engine_.set_controller(&tracker_meta_controller_);

        LOG_INFO("Motor using DUALPID_TRACKER control mode\n");
    } else {
        // Build classic cascade: MotorPoseFilter → PosePID → SpeedFilter → SpeedPID
        dualpid_meta_controller_.add_controller(&motor_distance_filter_);
        dualpid_meta_controller_.add_controller(&distance_controller_);
        dualpid_meta_controller_.add_controller(&speed_filter_);
        dualpid_meta_controller_.add_controller(&speed_controller_);

        motor_engine_.set_controller(&dualpid_meta_controller_);

        LOG_INFO("Motor using DUALPID control mode\n");
    }

    // Brake chain: ZeroSpeedOrder -> SpeedPID. Runs only when engine.brake_
    // is set; writes to the same speed_order/speed_command IO keys as the
    // normal chain, which is fine because only one chain runs per cycle.
    brake_meta_controller_.add_controller(&brake_zero_speed_order_controller_);
    brake_meta_controller_.add_controller(&brake_speed_controller_);
    motor_engine_.set_brake_controller(&brake_meta_controller_);

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

    // Configure overload pin and pass it to the engine for periodic clearing
    gpio_init(params_.clear_overload_pin, GPIO_OUT);
    gpio_clear(params_.clear_overload_pin);
    motor_engine_.set_clear_overload_pin(params_.clear_overload_pin);

    // Register state change callback for pose status reporting
    motor_engine_.set_pose_reached_cb(
        etl::delegate<void(
            motion_control::target_pose_status_t)>::create<Motor, &Motor::on_state_change>(*this));

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
    if (control_mode_ == MotorControlMode::DUALPID_TRACKER) {
        tracker_pose_controller_.parameters().pid()->reset();
        tracker_speed_controller_.parameters().pid()->reset();
        profile_tracker_controller_.reset();
        if (tracker_acceleration_filter_) {
            tracker_acceleration_filter_->reset();
        }
    } else {
        distance_controller_.parameters().pid()->reset();
        speed_controller_.parameters().pid()->reset();
        speed_filter_.reset();
    }

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

    // An explicit actuate() is a deliberate motion request: release any
    // latched brake. EMS gates actuator commands upstream (handler-level),
    // so this cannot silently revert an emergency stop.
    motor_engine_.set_brake(false);

    // Enable engine in case it has been previously disabled by timeout
    motor_engine_.enable();

    // Enable motor
    params_.motor.enable();
}

float Motor::get_target_speed_percentage() const
{
    if (control_mode_ == MotorControlMode::DUALPID_TRACKER) {
        // For tracker mode, use profile tracker max speed
        float max_speed = profile_tracker_controller_.parameters().max_speed();
        return max_speed == 0.0f ? 0.0f : (motor_engine_.target_speed() / max_speed) * 100.0f;
    } else {
        return params_.speed_filter_parameters.max_speed() == 0.0f
                   ? 0.0f
                   : (motor_engine_.target_speed() / params_.speed_filter_parameters.max_speed()) *
                         100.0f;
    }
}

void Motor::set_target_speed_percent(float percentage)
{
    if (control_mode_ == MotorControlMode::DUALPID_TRACKER) {
        // For tracker mode, use profile tracker max speed
        float max_speed = profile_tracker_controller_.parameters().max_speed();
        motor_engine_.set_target_speed((percentage / 100.0f) * max_speed);
    } else {
        motor_engine_.set_target_speed((percentage / 100.0f) *
                                       params_.speed_filter_parameters.max_speed());
    }
}

float Motor::get_current_distance() const
{
    return motor_engine_.get_current_distance_from_odometer();
}

void Motor::set_current_distance(float distance)
{
    motor_engine_.set_current_distance_to_odometer(distance);
}

void Motor::on_state_change(motion_control::target_pose_status_t state)
{
    switch (state) {
    case motion_control::target_pose_status_t::reached:
        set_state(PB_PositionalActuatorStateEnum::REACHED);
        break;
    case motion_control::target_pose_status_t::blocked:
        set_state(PB_PositionalActuatorStateEnum::BLOCKED);
        break;
    case motion_control::target_pose_status_t::timeout:
        set_state(PB_PositionalActuatorStateEnum::TIMEOUT);
        break;
    default:
        return;
    }
    send_state();
}

} // namespace positional_actuators
} // namespace actuators
} // namespace cogip
