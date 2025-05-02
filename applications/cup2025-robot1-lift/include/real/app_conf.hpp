#pragma once

// RIOT includes
#include <periph/gpio.h>

// Project includes
#include "trigonometry.h"
#include "actuators_motors_params.hpp"
#include "actuator/LiftParameters.hpp"
#include "actuator/LiftsLimitSwitchesManager.hpp"
#include "actuator/MotorParameters.hpp"
#include "actuator/PositionalActuator.hpp"
#include "encoder/EncoderQDEC.hpp"
#include "motor/MotorDriverDRV8873.hpp"
#include "motor/MotorRIOT.hpp"
#include "odometer/OdometerEncoder.hpp"
#include "pf_positional_actuators.hpp"

// Package includes
#include "etl/numeric.h"

namespace cogip {

/// @brief Lift motor identifier enum.
enum class cogip::actuators::Enum: uint8_t {
    MOTOR_LIFT = 0,
};

namespace app {
namespace actuators {

/// Quadrature decoding
#ifndef QDEC_MODE
#define QDEC_MODE                   QDEC_X4
#endif
#define QDEC_LIFT_POLARITY   -1

/// Motor minimal PWM
constexpr int pwm_minimal = 0;

// Motor lift pose PID
constexpr float motor_lift_pose_pid_kp = .2;
constexpr float motor_lift_pose_pid_ki = 0;
constexpr float motor_lift_pose_pid_kd = 0;
// Motor lift speed PID
constexpr float motor_lift_speed_pid_kp = 15;
constexpr float motor_lift_speed_pid_ki = 7.5;
constexpr float motor_lift_speed_pid_kd = 0;

// Motor lift pose PID integral limit
constexpr float motor_lift_pose_pid_integral_limit = etl::numeric_limits<int16_t>::max();
// Motor lift speed PID integral limit
//constexpr float motor_lift_speed_pid_integral_limit = etl::numeric_limits<int16_t>::max();
constexpr float motor_lift_speed_pid_integral_limit = 500 / motor_lift_speed_pid_ki;

// Motor lift threshold
constexpr float motor_lift_threshold = 0.1;

/// @name Lift motor application properties
/// @{

/// @brief Wheel geometry parameters for lift encoder conversion.
/// @details
///   - wheels_perimeter_mm = π × wheels_diameter_mm
///   - pulse_per_mm = wheels_encoder_resolution / wheels_perimeter_mm
namespace lift_motor_configuration {
    constexpr float wheels_diameter_mm            = 25.3f;                      ///< Coding wheel diameter (mm)
    constexpr float wheels_encoder_resolution     = 19 * 16 * 4;                ///< Ratio × encoder ticks × QDEC
    constexpr float wheels_perimeter_mm           = M_PI * wheels_diameter_mm;  ///< Wheel circumference (mm)
    constexpr float pulse_per_mm                  = wheels_encoder_resolution
                                                     / wheels_perimeter_mm;     ///< Encoder pulses per mm
}

/// @brief Control loop timing.
/// @details
///   - control_period_ms: loop period in milliseconds
///   - period_s: same period in seconds
namespace lift_control {
    constexpr uint16_t control_period_ms = 20u;                                 ///< Thread loop period (ms)
}

/// @brief Conversion factors to controller units.
/// @details
///   - m_per_s_to_mm_per_period = mm/s → mm/period
///   - period2_div_1000          = (ms² per period²) / 1000 (for m/s² → mm/period²)
namespace lift_conversion {
    constexpr float m_per_s_to_mm_per_period = lift_control::control_period_ms;
    constexpr float period2_div_1000 =
        (lift_control::control_period_ms * lift_control::control_period_ms) / 1000.0f;
}

/// @brief Speed & acceleration limits in both SI and controller units.
/// @details
///   Combines physical constraints (m/s, m/s²) with conversion factors
///   to produce controller-unit limits (mm/period, mm/period²).
namespace lift_limits {
    // Physical limits (SI)
    constexpr float min_speed_m_s      = 0.0f;
    constexpr float max_init_speed_m_s = 0.1f;
    constexpr float max_speed_m_s      = 0.5f;
    constexpr float max_acceleration_m_s2   = 1.5f;
    constexpr float max_deceleration_m_s2   = 1.f;

    // Controller-unit limits
    constexpr float min_speed_mm_per_period      = min_speed_m_s
                                                   * lift_conversion::m_per_s_to_mm_per_period;
    constexpr float max_init_speed_mm_per_period = max_init_speed_m_s
                                                   * lift_conversion::m_per_s_to_mm_per_period;
    constexpr float max_speed_mm_per_period      = max_speed_m_s
                                                   * lift_conversion::m_per_s_to_mm_per_period;
    constexpr float max_acceleration_mm_per_period2       = max_acceleration_m_s2
                                                   * lift_conversion::period2_div_1000;
    constexpr float max_deceleration_mm_per_period2       = max_deceleration_m_s2
                                                   * lift_conversion::period2_div_1000;
}

/// @brief Anti-blocking filter thresholds (in controller units).
namespace lift_anti_blocking {
    constexpr float speed_threshold            = 0.3f;      ///< Speed below which blockage is suspected (mm/period)
    constexpr float error_threshold            = 0.02f;     ///< Position error threshold (mm)
    constexpr float blocked_cycles_threshold   = 10.0f;     ///< Max consecutive cycles below speed_threshold
}

/// @brief Mechanical travel limits (mm).
constexpr float motor_lift_min_pose_mm = 0.0f;              ///< Lower travel limit (mm)
constexpr float motor_lift_max_pose_mm = 200.0f;            ///< Upper travel limit (mm)

/// @brief Limit switch GPIO pins.
constexpr gpio_t lower_limit_switch_pin = GPIO_PIN(PORT_A, 6);  ///< Lower end-stop pin
constexpr gpio_t upper_limit_switch_pin = GPIO_PIN(PORT_A, 4);  ///< Upper end-stop pin

/// @brief Motor Lift pose PID controller
static cogip::pid::PID motor_lift_pose_pid(
    motor_lift_pose_pid_kp,
    motor_lift_pose_pid_ki,
    motor_lift_pose_pid_kd,
    motor_lift_pose_pid_integral_limit
    );

/// @brief Motor Lift speed PID controller
static cogip::pid::PID motor_lift_speed_pid(
    motor_lift_speed_pid_kp,
    motor_lift_speed_pid_ki,
    motor_lift_speed_pid_kd,
    motor_lift_speed_pid_integral_limit
    );

/// @brief Motor Lift MotorPoseFilterParameters
static cogip::motion_control::MotorPoseFilterParameters motor_lift_pose_filter_parameters(
    motor_lift_threshold,
    lift_limits::max_deceleration_mm_per_period2
);

/// @brief Motor Lift PosePIDControllerParameters.
static cogip::motion_control::PosePIDControllerParameters motor_lift_pose_pid_parameters(&motor_lift_pose_pid);

/// @brief Motor Lift SpeedFilterParameters.
static cogip::motion_control::SpeedFilterParameters motor_lift_speed_filter_parameters(
    lift_limits::min_speed_mm_per_period,
    lift_limits::max_speed_mm_per_period,
    lift_limits::max_acceleration_mm_per_period2,
    false,
    lift_anti_blocking::speed_threshold,
    lift_anti_blocking::error_threshold,
    lift_anti_blocking::blocked_cycles_threshold
    );

/// @brief Motor Lift SpeedPIDControllerParameters.
static cogip::motion_control::SpeedPIDControllerParameters motor_lift_speed_pid_parameters(&motor_lift_speed_pid);

enum {
    MOTOR_LIFT_ID = static_cast<uint8_t>(cogip::actuators::Enum::MOTOR_LIFT)
};

/// @brief Lifts Motor driver
static cogip::motor::MotorDriverDRV8873 lifts_motor_driver(actuators_motors_params);

/// @brief Motors
static cogip::motor::MotorRIOT lift_motor(lifts_motor_driver, MOTOR_LIFT_ID);

/// @brief Lift motor encoder
static cogip::encoder::EncoderQDEC lift_motor_encoder(MOTOR_LIFT_ID,
    cogip::encoder::EncoderMode::ENCODER_MODE_X4,
    lift_motor_configuration::wheels_encoder_resolution);

static cogip::localization::OdometerEncoderParameters lift_motor_odometer_params{
    /* pulse_per_mm     */ lift_motor_configuration::pulse_per_mm,
    /* reverse_polarity */ true
};

/// @brief Lift motor odometer
static cogip::localization::OdometerEncoder lift_motor_odometer(lift_motor_odometer_params, lift_motor_encoder);

// 1) Prepare generic motor parameters for the front lift actuator
static const cogip::actuators::positional_actuators::MotorParameters lift_motor_params{
    /* id                           */ cogip::actuators::Enum::MOTOR_LIFT,
    /* default_timeout_ms           */ 2000u,
    /* send_state_cb                */ cogip::pf::actuators::positional_actuators::send_state,
    /* clear_overload_pin           */ CLEAR_OVERLOAD_PIN,
    /* pose_controller_params       */ &motor_lift_pose_pid_parameters,
    /* speed_controller_params      */ &motor_lift_speed_pid_parameters,
    /* pose_filter_params           */ &motor_lift_pose_filter_parameters,
    /* speed_filter_params          */ &motor_lift_speed_filter_parameters,
    /* engine_thread_timeout_ms_    */ lift_control::control_period_ms,
    /* motor                        */ lift_motor,
    /* odometer                     */ lift_motor_odometer
};

// 2) Specify front-lift-specific speeds (up/down)
static const cogip::actuators::positional_actuators::LiftParameters lift_params{
    /* motor_params             */ lift_motor_params,       ///< Base motor parameters
    /* init_speed_percentage    */ 100
                                   * lift_limits::max_init_speed_mm_per_period
                                   / lift_limits::max_speed_mm_per_period,
                                                            ///< Init sequence speed
    /* lower_limit              */ 0,                       ///< Lower lift limit
    /* upper_limit              */ 160,                     ///< Upper lift limit
    /* lower_limit_switch_pin   */ lower_limit_switch_pin,  ///< Lower end-stop input
    /* upper_limit_switch_pin   */ upper_limit_switch_pin,  ///< Upper end-stop input
};

/// @}

/// Actuators timeouts
/// @{
constexpr uint32_t default_timeout_period_motor_lift = 30;
/// @}

/// Motors initial pose
/// @{
constexpr int32_t motor_lift_initial_pose = 0;
/// @}

} // namespace actuators
} // namespace app
} // namespace cogip
