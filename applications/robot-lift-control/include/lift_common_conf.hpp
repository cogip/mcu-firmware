#pragma once

// RIOT includes
#include <periph/gpio.h>

// Project includes
#include "acceleration_filter/AccelerationFilterParameters.hpp"
#include "actuator/LiftParameters.hpp"
#include "actuator/LiftsLimitSwitchesManager.hpp"
#include "actuator/MotorParameters.hpp"
#include "actuator/PositionalActuator.hpp"
#include "actuators_motors_params.hpp"
#include "anti_blocking_controller/AntiBlockingControllerParameters.hpp"
#include "deceleration_filter/DecelerationFilterParameters.hpp"
#include "encoder/EncoderQDEC.hpp"
#include "motor/MotorDriverDRV8873.hpp"
#include "motor/MotorRIOT.hpp"
#include "odometer/OdometerEncoder.hpp"
#include "parameter/Parameter.hpp"
#include "pf_positional_actuators.hpp"
#include "pid/PIDParameters.hpp"
#include "profile_tracker_controller/ProfileTrackerControllerParameters.hpp"
#include "speed_limit_filter/SpeedLimitFilterParameters.hpp"
#include "trigonometry.h"

// Package includes
#include "etl/numeric.h"

namespace cogip {
namespace app {
namespace actuators {

/// @name Lift motor application properties
/// @{

/// @brief Control loop timing.
namespace lift_control {
constexpr uint16_t control_period_ms = 20u;
constexpr uint32_t default_timeout_lift_ms = 5000u;
} // namespace lift_control

/// @brief Conversion factors to controller units.
namespace lift_conversion {
constexpr float m_per_s_to_mm_per_period = lift_control::control_period_ms;
constexpr float period2_div_1000 =
    (lift_control::control_period_ms * lift_control::control_period_ms) / 1000.0f;
} // namespace lift_conversion

/// @brief Limit switch GPIO pins.
constexpr gpio_t lower_limit_switch_pin = GPIO_PIN(PORT_A, 6);
constexpr gpio_t upper_limit_switch_pin = GPIO_PIN(PORT_A, 4);

enum { MOTOR_LIFT_ID = 0 };

/// @}

} // namespace actuators
} // namespace app
} // namespace cogip
