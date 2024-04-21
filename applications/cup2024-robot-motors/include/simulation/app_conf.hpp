#pragma once

// Project includes
#include "etl/numeric.h"

// Motor lift pose PID
constexpr double motor_lift_pose_pid_kp = 1;
constexpr double motor_lift_pose_pid_ki = 0;
constexpr double motor_lift_pose_pid_kd = 0;
// Motor lift speed PID
constexpr double motor_lift_speed_pid_kp = 200;
constexpr double motor_lift_speed_pid_ki = 75;
constexpr double motor_lift_speed_pid_kd = 0;

// Motor lift pose PID integral limit
constexpr double motor_lift_pose_pid_integral_limit = etl::numeric_limits<int16_t>::max();
// Motor lift speed PID integral limit
constexpr double motor_lift_speed_pid_integral_limit = 500 / 75; // (PWM resolution / Ki)

// Motor lift threshold
constexpr double motor_lift_threshold = 0.1;
