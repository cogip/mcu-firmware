#pragma once

#include "lift_common_conf.hpp"

namespace cogip {
namespace app {
namespace actuators {

// Motor lift pose PID
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> motor_lift_pose_pid_kp{
    0.5f};
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
    motor_lift_speed_pid_integral_limit{static_cast<float>(etl::numeric_limits<int16_t>::max())};

// Motor lift brake speed PID (used only by the brake chain to actively hold
// the lift against gravity once a limit is reached). Higher Ki than the
// tracking PID so the integrator builds up fast enough to absorb the
// constant gravity load and keep the residual drop in the few-mm range
// on brake engagement (bench-tuned).
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative>
    motor_lift_brake_speed_pid_kp{10.0f};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative>
    motor_lift_brake_speed_pid_ki{3.0f};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative>
    motor_lift_brake_speed_pid_kd{0.f};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative>
    motor_lift_brake_speed_pid_integral_limit{
        static_cast<float>(etl::numeric_limits<int16_t>::max())};

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

/// @brief Speed & acceleration limits.
namespace lift_limits {
constexpr float min_speed_m_s = 0.0f;
constexpr float max_init_speed_m_s = 0.1f;
constexpr float max_speed_m_s = 0.4f;
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

/// Safety clamp ratio (20% above nominal max to catch runaway values)
constexpr float speed_clamp_ratio = 1.2f;
constexpr float acceleration_clamp_ratio = 1.2f;
constexpr float deceleration_clamp_ratio = 1.2f;
} // namespace lift_limits

/// @brief Anti-blocking controller thresholds.
/// @details Ratios aligned with motion control platform:
///   speed_threshold ≈ 5% of max_speed, error_threshold ≈ 20% of max_speed
namespace lift_anti_blocking {
constexpr float speed_threshold_mm_per_s = 12.5f;
constexpr float error_threshold_mm_per_s = 50.0f;
constexpr float speed_threshold_mm_per_period =
    speed_threshold_mm_per_s * lift_control::control_period_ms / 1000.0f;
constexpr float error_threshold_mm_per_period =
    error_threshold_mm_per_s * lift_control::control_period_ms / 1000.0f;
constexpr uint16_t blocked_cycles_threshold = 50;
} // namespace lift_anti_blocking

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

/// @brief Motor Lift brake speed PID parameters (dedicated to the brake chain)
inline cogip::pid::PIDParameters
    motor_lift_brake_speed_pid_params(motor_lift_brake_speed_pid_kp, motor_lift_brake_speed_pid_ki,
                                      motor_lift_brake_speed_pid_kd,
                                      motor_lift_brake_speed_pid_integral_limit);

/// @brief Motor Lift brake speed PID controller (separate instance so its
/// integrator state does not interfere with the tracking PID).
inline cogip::pid::PID motor_lift_brake_speed_pid(motor_lift_brake_speed_pid_params);

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

/// @brief Motor Lift brake SpeedPIDControllerParameters (dedicated to brake chain).
static cogip::motion_control::SpeedPIDControllerParameters
    motor_lift_brake_speed_pid_parameters(&motor_lift_brake_speed_pid);

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

/// @brief Motor Lift AntiBlockingControllerParameters.
static cogip::motion_control::AntiBlockingControllerParameters
    motor_lift_anti_blocking_parameters(true, // enabled
                                        lift_anti_blocking::speed_threshold_mm_per_period,
                                        lift_anti_blocking::error_threshold_mm_per_period,
                                        lift_anti_blocking::blocked_cycles_threshold);

/// @brief Motor Lift SpeedLimitFilter parameters (safety clamp at ratio × max).
static cogip::motion_control::SpeedLimitFilterParameters motor_lift_speed_limit_parameters(
    lift_limits::min_speed_mm_per_period,
    lift_limits::max_speed_mm_per_period* lift_limits::speed_clamp_ratio);

/// @brief Motor Lift AccelerationFilter parameters (safety clamp at ratio × max).
static cogip::motion_control::AccelerationFilterParameters
motor_lift_acceleration_filter_parameters(
    lift_limits::max_acceleration_mm_per_period2* lift_limits::acceleration_clamp_ratio,
    lift_limits::min_speed_mm_per_period);

/// @brief Motor Lift DecelerationFilter parameters (safety clamp at ratio × max).
/// @details Uses stronger deceleration than ProfileTracker to brake harder near target.
static cogip::motion_control::DecelerationFilterParameters
motor_lift_deceleration_filter_parameters(
    lift_limits::max_deceleration_mm_per_period2* lift_limits::deceleration_clamp_ratio);

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
/// @param use_tracker_chain If true, use DUALPID_TRACKER mode with profile tracker
/// @note When use_tracker_chain is true, the profile_tracker_parameters pointer is set,
///       enabling the Motor to use the tracker chain instead of the classic DualPID chain.
inline cogip::actuators::positional_actuators::MotorParameters
make_lift_motor_params(cogip::actuators::Enum actuator_id, bool use_tracker_chain = true)
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
        /* profile_tracker_params       */
        use_tracker_chain ? &motor_lift_profile_tracker_parameters : nullptr,
        /* tracker_combiner_params      */ nullptr, // Use default parameters
        /* acceleration_filter_params   */ &motor_lift_acceleration_filter_parameters,
        /* speed_limit_filter_params    */ &motor_lift_speed_limit_parameters,
        /* deceleration_filter_params   */ &motor_lift_deceleration_filter_parameters,
        /* anti_blocking_params         */ &motor_lift_anti_blocking_parameters,
        /* brake_speed_controller_params*/ motor_lift_brake_speed_pid_parameters,
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

/// @brief Lift 2 actuator ID (for CAN protobuf messages)
inline constexpr cogip::actuators::Enum LIFT_ACTUATOR_ID =
    cogip::actuators::Enum{1}; // MOTOR_LIFT_2

/// @brief Lift parameters for lift 2
static const auto lift_params = make_lift_params(LIFT_ACTUATOR_ID);

} // namespace actuators
} // namespace app
} // namespace cogip
