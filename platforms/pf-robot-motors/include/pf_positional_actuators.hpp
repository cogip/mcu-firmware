// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pf-robot-actuators
/// @{
/// @file
/// @brief       Functions and definitions related to positional_actuators.
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "board.h"
#include "PositionalActuator.hpp"
#include "trigonometry.h"

// RIOT includes
#include <periph/gpio.h>

namespace cogip {
namespace pf {
namespace actuators {
namespace positional_actuators {

/// Quadrature decoding
#ifndef QDEC_MODE
#define QDEC_MODE                   QDEC_X4
#endif
#define QDEC_BOTTOM_LIFT_POLARITY   -1
#define QDEC_TOP_LIFT_POLARITY      1

/// Motor minimal PWM
constexpr int pwm_minimal = 70;

/// @name Lift motor application properties
///
/// To be computed:
///  - pulse_per_mm             : number of pulses per mm of coding wheel
///
/// must be known:
///  - wheels_diameter          : coding wheel diameter (mm)
///  - wheels_encoder_resolution: number of pulses by turn of coding wheels
///
/// Intermediate calculation:
///  - wheels_perimeter = pi*wheels_diameter
///  - pulse_per_mm = wheels_encoder_resolution / wheels_perimeter
///
/// @{
constexpr double wheels_diameter_mm = 12.03;
constexpr double wheels_encoder_resolution = 37.35 * 11 * 4;
constexpr double wheels_perimeter_mm = M_PI * wheels_diameter_mm;
constexpr double pulse_per_mm = wheels_encoder_resolution / wheels_perimeter_mm;///< WHEELS_ENCODER_RESOLUTION / WHEELS_PERIMETER
/// @}

///< controller thread loop period
constexpr uint16_t motor_lift_control_thread_period_ms = 20;

/// Lift motors speed filter parameters
constexpr double motor_lift_anti_blocking_speed_threshold_per_period = 0.3;
constexpr double motor_lift_anti_blocking_error_threshold_per_period = 0.02;
constexpr double motor_lift_anti_blocking_blocked_cycles_nb_threshold = 10;
constexpr double motor_lift_min_speed_motor_lift_m_per_s = 0;
constexpr double motor_lift_max_init_speed_motor_lift_m_per_s = 0.02;
constexpr double motor_lift_max_speed_motor_lift_m_per_s = 0.17;
constexpr double motor_lift_max_acc_motor_lift_m_per_s2 = motor_lift_max_speed_motor_lift_m_per_s * 10;
constexpr double motor_lift_max_dec_motor_lift_m_per_s2 = motor_lift_max_speed_motor_lift_m_per_s * 2;
constexpr double motor_lift_max_acc_motor_lift_mm_per_period2 = (
    (1000 * motor_lift_max_acc_motor_lift_m_per_s2 * motor_lift_control_thread_period_ms * motor_lift_control_thread_period_ms) \
    / (1000 * 1000)
    );          ///< Maximum motor_lift acceleration (mm/<motor_lift_control_thread_period_ms>²)
constexpr double motor_lift_max_dec_motor_lift_mm_per_period2 = (
    (1000 * motor_lift_max_dec_motor_lift_m_per_s2 * motor_lift_control_thread_period_ms * motor_lift_control_thread_period_ms) \
    / (1000 * 1000)
    );          ///< Maximum motor_lift deceleration (mm/<motor_lift_control_thread_period_ms>²)
constexpr double motor_lift_min_speed_motor_lift_mm_per_period = (
    (1000 * motor_lift_min_speed_motor_lift_m_per_s * motor_lift_control_thread_period_ms) \
    / 1000);    ///< Minimum motor_lift speed (mm/<motor_lift_control_thread_period_ms>)
constexpr double motor_lift_max_speed_motor_lift_mm_per_period = (
    (1000 * motor_lift_max_speed_motor_lift_m_per_s * motor_lift_control_thread_period_ms) \
    / 1000);    ///< Maximum motor_lift speed (mm/<motor_lift_control_thread_period_ms>)
constexpr double motor_lift_max_init_speed_motor_lift_mm_per_period = (
    (1000 * motor_lift_max_init_speed_motor_lift_m_per_s * motor_lift_control_thread_period_ms) \
    / 1000);    ///< Maximum motor_lift init speed (mm/<motor_lift_control_thread_period_ms>)

// Motors ids
constexpr auto START_LINE = __LINE__;
enum class Enum: uint8_t {
    MOTOR_BOTTOM_LIFT = 0,
    MOTOR_TOP_LIFT = 1,
};
constexpr auto COUNT = __LINE__ - START_LINE - 3;

/// Actuators timeouts
/// @{
constexpr uint32_t default_timeout_period_motor_bottom_lift = 30;
constexpr uint32_t default_timeout_period_motor_top_lift = 40;
/// @}

/// Motors initial pose
/// @{
constexpr int32_t motor_bottom_lift_initial_pose = 5;
constexpr int32_t motor_top_lift_initial_pose = 20;
/// @}

/// Actuators DC motors IDs
/// @{
/// @}

/// Initialize positional_actuators.
void init();

/// Check if a positional_actuator identified by id exists.
bool contains(
    cogip::pf::actuators::Enum id  ///< [in] positional_actuator id
);

/// Get a positional_actuator by id.
PositionalActuator & get(
    cogip::pf::actuators::Enum id  ///< [in] positional_actuator id
);

/// Disable all positional actuators
void disable_all();

/// Send positional actuator state protobuf message
void send_state(cogip::pf::actuators::Enum positional_actuator);

/// Initialize motors at their origin
void pf_init_motors_sequence(void);

} // namespace positional_actuators
} // namespace actuators
} // namespace pf
} // namespace cogip

/// @}
