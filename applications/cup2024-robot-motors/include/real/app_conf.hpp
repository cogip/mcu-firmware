#pragma once

// Project includes
#include "etl/numeric.h"

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
