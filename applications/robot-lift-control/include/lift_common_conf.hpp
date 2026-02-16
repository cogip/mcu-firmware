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
#include "profile_tracker_controller/ProfileTrackerControllerParameters.hpp"
#include "trigonometry.h"

// Package includes
#include "etl/numeric.h"

namespace cogip {
namespace app {
namespace actuators {

// Motor lift pose PID
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> motor_lift_pose_pid_kp{
    0.2f};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> motor_lift_pose_pid_ki{
    0.f};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> motor_lift_pose_pid_kd{
    0.f};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative>
    motor_lift_pose_pid_integral_limit{static_cast<float>(etl::numeric_limits<int16_t>::max())};

// Motor lift speed PID
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> motor_lift_speed_pid_kp{
    10.0f};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> motor_lift_speed_pid_ki{
    0.4f};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> motor_lift_speed_pid_kd{
    0.f};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative>
    motor_lift_speed_pid_integral_limit{500.f / 7.5f};

// Motor lift threshold
constexpr float motor_lift_threshold = 1.0;

/// @name Lift motor application properties
/// @{

/// @brief Wheel geometry parameters for lift encoder conversion.
namespace lift_motor_configuration {
constexpr float wheels_diameter_mm = 50.6f;
constexpr float wheels_encoder_resolution = 19 * 16 * 4;
constexpr float wheels_perimeter_mm = M_PI * wheels_diameter_mm;
constexpr float pulse_per_mm = wheels_encoder_resolution / wheels_perimeter_mm;
} // namespace lift_motor_configuration

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

/// @brief Speed & acceleration limits.
namespace lift_limits {
constexpr float min_speed_m_s = 0.0f;
constexpr float max_init_speed_m_s = 0.1f;
constexpr float max_speed_m_s = 0.25f;
constexpr float max_acceleration_m_s2 = 1.f;
constexpr float max_deceleration_m_s2 = 2.f;

constexpr float min_speed_mm_per_period = min_speed_m_s * lift_conversion::m_per_s_to_mm_per_period;
constexpr float max_init_speed_mm_per_period =
    max_init_speed_m_s * lift_conversion::m_per_s_to_mm_per_period;
constexpr float max_speed_mm_per_period = max_speed_m_s * lift_conversion::m_per_s_to_mm_per_period;
constexpr float max_acceleration_mm_per_period2 =
    max_acceleration_m_s2 * lift_conversion::period2_div_1000;
constexpr float max_deceleration_mm_per_period2 =
    max_deceleration_m_s2 * lift_conversion::period2_div_1000;

constexpr int32_t lift_lower_limit_mm = 0;
constexpr int32_t lift_upper_limit_mm = 130;
} // namespace lift_limits

/// @brief Anti-blocking filter thresholds.
namespace lift_anti_blocking {
constexpr float speed_threshold_mm_per_s = 200.0;
constexpr float error_threshold_mm_per_s = 15.0f;
constexpr float speed_threshold_mm_per_period =
    speed_threshold_mm_per_s * lift_control::control_period_ms / 1000.0f;
constexpr float error_threshold_mm_per_period =
    error_threshold_mm_per_s * lift_control::control_period_ms / 1000.0f;
constexpr uint32_t blocked_cycles_threshold = 7;
} // namespace lift_anti_blocking

/// @brief Limit switch GPIO pins.
constexpr gpio_t lower_limit_switch_pin = GPIO_PIN(PORT_A, 6);
constexpr gpio_t upper_limit_switch_pin = GPIO_PIN(PORT_A, 4);

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

/// @brief Motor Lift ProfileTrackerControllerParameters (for DUALPID_TRACKER mode).
/// @details Defines the trapezoidal velocity profile limits for smooth motion control.
static cogip::motion_control::ProfileTrackerControllerParameters
    motor_lift_profile_tracker_parameters(
        lift_limits::max_speed_mm_per_period,         // max_speed
        lift_limits::max_acceleration_mm_per_period2, // acceleration
        lift_limits::max_deceleration_mm_per_period2, // deceleration
        true,                                         // must_stop_at_end
        1                                             // period_increment
    );

enum { MOTOR_LIFT_ID = 0 };

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
    /* reverse_polarity */ false};

/// @brief Lift motor odometer
static cogip::localization::OdometerEncoder lift_motor_odometer(lift_motor_odometer_params,
                                                                lift_motor_encoder);

/// @brief Create lift motor parameters with the given actuator ID
/// @param actuator_id The actuator ID for CAN protobuf messages
inline cogip::actuators::positional_actuators::MotorParameters
make_lift_motor_params(cogip::actuators::Enum actuator_id)
{
    return {
        /* id                           */ actuator_id,
        /* default_timeout_ms           */ lift_control::default_timeout_lift_ms,
        /* send_state_cb                */ cogip::pf::actuators::positional_actuators::send_state,
        /* clear_overload_pin           */ CLEAR_OVERLOAD_PIN,
        /* pose_controller_params       */ motor_lift_pose_pid_parameters,
        /* speed_controller_params      */ motor_lift_speed_pid_parameters,
        /* pose_filter_params           */ motor_lift_pose_filter_parameters,
        /* speed_filter_params          */ motor_lift_speed_filter_parameters,
        /* engine_thread_timeout_ms_    */ lift_control::control_period_ms,
        /* motor                        */ lift_motor,
        /* odometer                     */ lift_motor_odometer,
        /* profile_tracker_params       */ &motor_lift_profile_tracker_parameters,
        /* tracker_combiner_params      */ nullptr // Use default parameters
    };
}

/// @brief Create lift parameters with the given actuator ID
/// @param actuator_id The actuator ID for CAN protobuf messages
inline cogip::actuators::positional_actuators::LiftParameters
make_lift_params(cogip::actuators::Enum actuator_id)
{
    static const auto motor_params = make_lift_motor_params(actuator_id);
    return {
        /* motor_params             */ motor_params,
        /* init_speed_percentage    */ 100 * lift_limits::max_init_speed_mm_per_period /
            lift_limits::max_speed_mm_per_period,
        /* lower_limit_mm           */ lift_limits::lift_lower_limit_mm,
        /* upper_limit_mm           */ lift_limits::lift_upper_limit_mm,
        /* lower_limit_switch_pin   */ lower_limit_switch_pin,
        /* upper_limit_switch_pin   */ upper_limit_switch_pin,
    };
}

/// @}

} // namespace actuators
} // namespace app
} // namespace cogip
