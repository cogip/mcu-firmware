#pragma once

// RIOT includes
#include <periph/gpio.h>

// Project includes
#include "actuator/LiftParameters.hpp"
#include "actuator/LiftsLimitSwitchesManager.hpp"
#include "actuator/MotorParameters.hpp"
#include "actuator/PositionalActuator.hpp"
#include "actuators_motors_params.hpp"
#include "encoder/EncoderQDEC.hpp"
#include "motor/MotorDriverDRV8873.hpp"
#include "motor/MotorRIOT.hpp"
#include "odometer/OdometerEncoder.hpp"
#include "parameter/Parameter.hpp"
#include "pf_positional_actuators.hpp"
#include "pid/PIDParameters.hpp"
#include "trigonometry.h"

// Package includes
#include "etl/numeric.h"

namespace cogip {

/// @brief Lift motor identifier enum.
enum class cogip::actuators::Enum : uint8_t {
    MOTOR_LIFT = 0,
};

namespace app {
namespace actuators {

/// Quadrature decoding
#ifndef QDEC_MODE
#define QDEC_MODE QDEC_X4
#endif
#define QDEC_LIFT_POLARITY -1

// Motor lift pose PID
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> motor_lift_pose_pid_kp{
    .2f};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> motor_lift_pose_pid_ki{
    0.f};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> motor_lift_pose_pid_kd{
    0.f};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative>
    motor_lift_pose_pid_integral_limit{static_cast<float>(etl::numeric_limits<int16_t>::max())};

// Motor lift speed PID
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> motor_lift_speed_pid_kp{
    15.f};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> motor_lift_speed_pid_ki{
    7.5f};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> motor_lift_speed_pid_kd{
    0.f};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative>
    motor_lift_speed_pid_integral_limit{500.f / 7.5f};

// Motor lift threshold
constexpr float motor_lift_threshold = 0.1;

/// @name Lift motor application properties
/// @{

/// @brief Wheel geometry parameters for lift encoder conversion.
/// @details
///   - wheels_perimeter_mm = π × wheels_diameter_mm
///   - pulse_per_mm = wheels_encoder_resolution / wheels_perimeter_mm
namespace lift_motor_configuration {
constexpr float wheels_diameter_mm = 25.3f;                      ///< Coding wheel diameter (mm)
constexpr float wheels_encoder_resolution = 19 * 16 * 4;         ///< Ratio × encoder ticks × QDEC
constexpr float wheels_perimeter_mm = M_PI * wheels_diameter_mm; ///< Wheel circumference (mm)
constexpr float pulse_per_mm =
    wheels_encoder_resolution / wheels_perimeter_mm; ///< Encoder pulses per mm
} // namespace lift_motor_configuration

/// @brief Control loop timing.
/// @details
///   - control_period_ms: loop period in milliseconds
///   - period_s: same period in seconds
namespace lift_control {
constexpr uint16_t control_period_ms = 20u;         ///< Thread loop period (ms)
constexpr uint32_t default_timeout_lift_ms = 2000u; ///< Default lift timeout before stop (ms)
} // namespace lift_control

/// @brief Conversion factors to controller units.
/// @details
///   - m_per_s_to_mm_per_period = mm/s → mm/period
///   - period2_div_1000          = (ms² per period²) / 1000 (for m/s² →
///   mm/period²)
namespace lift_conversion {
constexpr float m_per_s_to_mm_per_period = lift_control::control_period_ms;
constexpr float period2_div_1000 =
    (lift_control::control_period_ms * lift_control::control_period_ms) / 1000.0f;
} // namespace lift_conversion

/// @brief Speed & acceleration limits in both SI and controller units.
/// @details
///   Combines physical constraints (m/s, m/s²) with conversion factors
///   to produce controller-unit limits (mm/period, mm/period²).
namespace lift_limits {
// Physical limits (SI)
constexpr float min_speed_m_s = 0.0f;
constexpr float max_init_speed_m_s = 0.1f;
constexpr float max_speed_m_s = 0.5f;
constexpr float max_acceleration_m_s2 = 1.5f;
constexpr float max_deceleration_m_s2 = 1.f;

// Controller-unit limits
constexpr float min_speed_mm_per_period = min_speed_m_s * lift_conversion::m_per_s_to_mm_per_period;
constexpr float max_init_speed_mm_per_period =
    max_init_speed_m_s * lift_conversion::m_per_s_to_mm_per_period;
constexpr float max_speed_mm_per_period = max_speed_m_s * lift_conversion::m_per_s_to_mm_per_period;
constexpr float max_acceleration_mm_per_period2 =
    max_acceleration_m_s2 * lift_conversion::period2_div_1000;
constexpr float max_deceleration_mm_per_period2 =
    max_deceleration_m_s2 * lift_conversion::period2_div_1000;

// Mechanical travel limits (mm).
constexpr int32_t lift_lower_limit_mm = 0;
constexpr int32_t lift_upper_limit_mm = 160.0;

} // namespace lift_limits

/// @brief Anti-blocking filter thresholds (in controller units).
namespace lift_anti_blocking {
constexpr float speed_threshold_mm_per_s =
    200.0; ///< Speed below which blockage is suspected (mm/s)
constexpr float error_threshold_mm_per_s = 15.0f; ///< Speed error threshold (mm/s)
constexpr float speed_threshold_mm_per_period =
    speed_threshold_mm_per_s * lift_control::control_period_ms /
    1000.0f; ///< Speed below which blockage is suspected (mm/period)
constexpr float error_threshold_mm_per_period = error_threshold_mm_per_s *
                                                lift_control::control_period_ms /
                                                1000.0f; ///< Speed error threshold (mm/period)

constexpr uint32_t blocked_cycles_threshold = 7; ///< Max consecutive cycles below speed_threshold
} // namespace lift_anti_blocking

/// @brief Limit switch GPIO pins.
constexpr gpio_t lower_limit_switch_pin = GPIO_PIN(PORT_A, 6); ///< Lower end-stop pin
constexpr gpio_t upper_limit_switch_pin = GPIO_PIN(PORT_A, 4); ///< Upper end-stop pin

/// @brief Motor Lift pose PID parameters
inline cogip::pid::PIDParameters motor_lift_pose_pid_params(motor_lift_pose_pid_kp,
                                                            motor_lift_pose_pid_ki,
                                                            motor_lift_pose_pid_kd,
                                                            motor_lift_pose_pid_integral_limit);

/// @brief Motor Lift pose PID controller
inline cogip::pid::PID motor_lift_pose_pid(motor_lift_pose_pid_params);

/// @brief Motor Lift speed PID parameters
inline cogip::pid::PIDParameters motor_lift_speed_pid_params(motor_lift_speed_pid_kp,
                                                             motor_lift_speed_pid_ki,
                                                             motor_lift_speed_pid_kd,
                                                             motor_lift_speed_pid_integral_limit);

/// @brief Motor Lift speed PID controller
inline cogip::pid::PID motor_lift_speed_pid(motor_lift_speed_pid_params);

/// @brief Motor Lift MotorPoseFilterParameters
static cogip::motion_control::MotorPoseFilterParameters
    motor_lift_pose_filter_parameters(motor_lift_threshold,
                                      lift_limits::max_deceleration_mm_per_period2);

/// @brief Motor Lift PosePIDControllerParameters.
static cogip::motion_control::PosePIDControllerParameters
    motor_lift_pose_pid_parameters(&motor_lift_pose_pid);

/// @brief Motor Lift SpeedFilterParameters.
static cogip::motion_control::SpeedFilterParameters motor_lift_speed_filter_parameters(
    lift_limits::min_speed_mm_per_period, lift_limits::max_speed_mm_per_period,
    lift_limits::max_acceleration_mm_per_period2, lift_limits::max_deceleration_mm_per_period2);

/// @brief Motor Lift SpeedPIDControllerParameters.
static cogip::motion_control::SpeedPIDControllerParameters
    motor_lift_speed_pid_parameters(&motor_lift_speed_pid);

enum { MOTOR_LIFT_ID = static_cast<uint8_t>(cogip::actuators::Enum::MOTOR_LIFT) };

/// @brief Lifts Motor driver
static cogip::motor::MotorDriverDRV8873 lifts_motor_driver(actuators_motors_params);

/// @brief Motors
static cogip::motor::MotorRIOT lift_motor(lifts_motor_driver, MOTOR_LIFT_ID);

/// @brief Lift motor encoder
static cogip::encoder::EncoderQDEC
    lift_motor_encoder(MOTOR_LIFT_ID, cogip::encoder::EncoderMode::ENCODER_MODE_X4,
                       lift_motor_configuration::wheels_encoder_resolution);

static cogip::localization::OdometerEncoderParameters lift_motor_odometer_params{
    /* pulse_per_mm     */ lift_motor_configuration::pulse_per_mm,
    /* reverse_polarity */ true};

/// @brief Lift motor odometer
static cogip::localization::OdometerEncoder lift_motor_odometer(lift_motor_odometer_params,
                                                                lift_motor_encoder);

// 1) Prepare generic motor parameters for the front lift actuator
static const cogip::actuators::positional_actuators::MotorParameters lift_motor_params{
    /* id                           */ cogip::actuators::Enum::MOTOR_LIFT,
    /* default_timeout_ms           */
    lift_control::default_timeout_lift_ms,
    /* send_state_cb                */
    cogip::pf::actuators::positional_actuators::send_state,
    /* clear_overload_pin           */ CLEAR_OVERLOAD_PIN,
    /* pose_controller_params       */ motor_lift_pose_pid_parameters,
    /* speed_controller_params      */ motor_lift_speed_pid_parameters,
    /* pose_filter_params           */ motor_lift_pose_filter_parameters,
    /* speed_filter_params          */ motor_lift_speed_filter_parameters,
    /* engine_thread_timeout_ms_    */ lift_control::control_period_ms,
    /* motor                        */ lift_motor,
    /* odometer                     */ lift_motor_odometer};

// 2) Specify front-lift-specific speeds (up/down)
static const cogip::actuators::positional_actuators::LiftParameters lift_params{
    /* motor_params             */ lift_motor_params, ///< Base motor parameters
    /* init_speed_percentage    */ 100 * lift_limits::max_init_speed_mm_per_period /
        lift_limits::max_speed_mm_per_period,
    ///< Init sequence speed
    /* lower_limit_mm           */ lift_limits::lift_lower_limit_mm, ///< Lower
                                                                     ///< lift
                                                                     ///< limit
    /* upper_limit_mm           */ lift_limits::lift_upper_limit_mm, ///< Upper
                                                                     ///< lift
                                                                     ///< limit
    /* lower_limit_switch_pin   */ lower_limit_switch_pin,           ///< Lower end-stop
                                                                     ///< input
    /* upper_limit_switch_pin   */ upper_limit_switch_pin,           ///< Upper end-stop
                                                                     ///< input
};

/// @}

} // namespace actuators
} // namespace app
} // namespace cogip
