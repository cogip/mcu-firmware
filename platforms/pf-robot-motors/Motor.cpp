// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "board.h"
#include "Motor.hpp"

#include <iostream>

namespace cogip {
namespace pf {
namespace actuators {
namespace positional_actuators {

/// Constructor
Motor::Motor(
        cogip::pf::actuators::Enum id,
        uint32_t default_timeout_period,
        send_state_cb_t send_state_cb,
        motor_driver_t *motor_driver,
        uint8_t motor_id,
        gpio_t clear_overload_pin,
        double target_speed,
        cogip::motion_control::PosePIDControllerParameters *pose_controller_parameters,
        cogip::motion_control::SpeedPIDControllerParameters *speed_controller_parameters,
        cogip::motion_control::MotorPoseFilterParameters *motor_pose_filter_parameters,
        cogip::motion_control::SpeedFilterParameters *speed_filter_parameters,
        cogip::motion_control::motor_get_speed_and_pose_cb_t motor_get_speed_and_pose_cb,
        cogip::motion_control::motor_process_commands_cb_t motor_process_commands_cb
    ) : PositionalActuator(id, default_timeout_period, send_state_cb),
        motor_driver_(motor_driver),
        motor_id_(motor_id),
        clear_overload_pin_(clear_overload_pin),
        pose_controller_(pose_controller_parameters),
        speed_controller_(speed_controller_parameters),
        motor_pose_filter_(motor_pose_filter_parameters),
        speed_filter_(speed_filter_parameters),
        motor_engine_(motor_get_speed_and_pose_cb, motor_process_commands_cb) {

    // DualPIDMetaController:
    // MotorPoseFilter -> PosePIDController -> SpeedFilter -> SpeedPIDController
    dualpid_meta_controller_.add_controller(&motor_pose_filter_);
    dualpid_meta_controller_.add_controller(&pose_controller_);
    dualpid_meta_controller_.add_controller(&speed_filter_);
    dualpid_meta_controller_.add_controller(&speed_controller_);

    motor_engine_.set_controller(&dualpid_meta_controller_);

    motor_engine_.set_target_speed(target_speed);

    motor_engine_.start_thread();

    gpio_init(clear_overload_pin, GPIO_OUT);

    motor_enable(motor_driver_, motor_id_);

    disable();
}

void Motor::disable() {
    if (!motor_driver_) {
        std::cerr << __func__ << ": motor " << id_ << ": motor driver is null pointer" << std::endl;
        return;
    }

    if (motor_id_ >= motor_driver_->params->nb_motors) {
        std::cerr << __func__ << ": motor " << id_ << ": wrong motor id" << std::endl;
        return;
    }

    motor_brake(motor_driver_, motor_id_);
    motor_engine_.disable();
}

void Motor::actuate(int32_t command) {
    command_ = command;
    // reset PIDs on new command
    this->pose_controller_.parameters()->pid()->reset();
    this->speed_controller_.parameters()->pid()->reset();
    this->speed_filter_.reset_previous_speed_order();
    this->speed_filter_.reset_anti_blocking_blocked_cycles_nb();

    if (!motor_driver_) {
        std::cerr << __func__ << ": motor " << id_ << ": motor driver is null pointer" << std::endl;
        return;
    }

    if (motor_id_ >= motor_driver_->params->nb_motors) {
        std::cerr << __func__ << ": motor " << id_ << ": wrong motor id" << std::endl;
        return;
    }

    motor_engine_.set_target_pose(command_);
    motor_engine_.enable();

    if (!timeout_period_)
        timeout_period_ = default_timeout_period_;

    // Reset overload in case motor would be disabled
    gpio_clear(clear_overload_pin_);
    ztimer_sleep(ZTIMER_USEC, 1);
    gpio_set(clear_overload_pin_);
}

} // namespace positional_actuators
} // namespace actuators
} // namespace pf
} // namespace cogip
