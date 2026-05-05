#pragma once

// Macro helpers for LIFT_ID → liftX_conf.hpp include
#define STRINGIFY(x) #x
#define EXPAND_AND_STRINGIFY(x) STRINGIFY(x)
#define CONCATENATE3(a, b, c) a##b##c
#define LIFT_CONF_FILE(id) CONCATENATE3(lift, id, _conf.hpp)

#ifndef LIFT_ID
#error "Build with LIFT_ID=<1-2>. Ex: make LIFT_ID=1"
#endif

#if LIFT_ID < 1 || LIFT_ID > 2
#error "LIFT_ID must be 1 or 2"
#endif

// Include the appropriate lift configuration based on LIFT_ID
#include EXPAND_AND_STRINGIFY(LIFT_CONF_FILE(LIFT_ID))

namespace cogip {
namespace app {
namespace actuators {

/// @name Lift motor parameter definitions
/// @{

// Motor lift pose PID
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> motor_lift_pose_pid_kp{
    lift_pid_defaults::motor_lift_pose_pid_kp};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> motor_lift_pose_pid_ki{
    lift_pid_defaults::motor_lift_pose_pid_ki};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> motor_lift_pose_pid_kd{
    lift_pid_defaults::motor_lift_pose_pid_kd};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative>
    motor_lift_pose_pid_integral_limit{lift_pid_defaults::motor_lift_pose_pid_integral_limit};

// Motor lift speed PID
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> motor_lift_speed_pid_kp{
    lift_pid_defaults::motor_lift_speed_pid_kp};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> motor_lift_speed_pid_ki{
    lift_pid_defaults::motor_lift_speed_pid_ki};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative> motor_lift_speed_pid_kd{
    lift_pid_defaults::motor_lift_speed_pid_kd};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative>
    motor_lift_speed_pid_integral_limit{lift_pid_defaults::motor_lift_speed_pid_integral_limit};

// Motor lift brake speed PID
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative>
    motor_lift_brake_speed_pid_kp{lift_pid_defaults::motor_lift_brake_speed_pid_kp};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative>
    motor_lift_brake_speed_pid_ki{lift_pid_defaults::motor_lift_brake_speed_pid_ki};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative>
    motor_lift_brake_speed_pid_kd{lift_pid_defaults::motor_lift_brake_speed_pid_kd};
inline cogip::parameter::Parameter<float, cogip::parameter::NonNegative>
    motor_lift_brake_speed_pid_integral_limit{
        lift_pid_defaults::motor_lift_brake_speed_pid_integral_limit};

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

/// @brief Lift parameters for the configured lift
static const auto lift_params = make_lift_params(LIFT_ACTUATOR_ID);

/// @}

} // namespace actuators
} // namespace app
} // namespace cogip
