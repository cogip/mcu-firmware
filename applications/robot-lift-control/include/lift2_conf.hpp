#pragma once

#include "lift_common_conf.hpp"

namespace cogip {
namespace app {
namespace actuators {

/// @name Lift 2 default values
/// @{

// clang-format off

/// @brief Default PID gains.
namespace lift_pid_defaults {
// Motor lift pose PID
constexpr float motor_lift_pose_pid_kp = 0.5f;
constexpr float motor_lift_pose_pid_ki = 0.f;
constexpr float motor_lift_pose_pid_kd = 0.f;
constexpr float motor_lift_pose_pid_integral_limit = static_cast<float>(etl::numeric_limits<int16_t>::max());

// Motor lift speed PID
constexpr float motor_lift_speed_pid_kp = 10.0f;
constexpr float motor_lift_speed_pid_ki = 0.4f;
constexpr float motor_lift_speed_pid_kd = 0.f;
constexpr float motor_lift_speed_pid_integral_limit = static_cast<float>(etl::numeric_limits<int16_t>::max());

// Motor lift brake speed PID (used only by the brake chain to actively hold
// the lift against gravity once a limit is reached). Higher Ki than the
// tracking PID so the integrator builds up fast enough to absorb the
// constant gravity load and keep the residual drop in the few-mm range
// on brake engagement (bench-tuned).
constexpr float motor_lift_brake_speed_pid_kp = 10.0f;
constexpr float motor_lift_brake_speed_pid_ki = 3.0f;
constexpr float motor_lift_brake_speed_pid_kd = 0.f;
constexpr float motor_lift_brake_speed_pid_integral_limit = static_cast<float>(etl::numeric_limits<int16_t>::max());
} // namespace lift_pid_defaults

/// @brief Motor lift threshold
constexpr float motor_lift_threshold = 1.0f;

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
constexpr float max_init_speed_mm_per_period = max_init_speed_m_s * lift_conversion::m_per_s_to_mm_per_period;
constexpr float max_speed_mm_per_period = max_speed_m_s * lift_conversion::m_per_s_to_mm_per_period;
constexpr float max_acceleration_mm_per_period2 = max_acceleration_m_s2 * lift_conversion::period2_div_1000;
constexpr float max_deceleration_mm_per_period2 = max_deceleration_m_s2 * lift_conversion::period2_div_1000;

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
constexpr float speed_threshold_mm_per_period = speed_threshold_mm_per_s * lift_control::control_period_ms / 1000.0f;
constexpr float error_threshold_mm_per_period = error_threshold_mm_per_s * lift_control::control_period_ms / 1000.0f;
constexpr uint16_t blocked_cycles_threshold = 50;
} // namespace lift_anti_blocking

/// @brief Lift 2 actuator ID (for CAN protobuf messages)
inline constexpr cogip::actuators::Enum LIFT_ACTUATOR_ID = cogip::actuators::Enum{1}; // MOTOR_LIFT_2

// clang-format on

/// @}

} // namespace actuators
} // namespace app
} // namespace cogip
