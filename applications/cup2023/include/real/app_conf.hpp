#pragma once

// Project includes
#include "etl/numeric.h"

// Linear pose PID
constexpr double linear_pose_pid_kp = 0.035;
constexpr double linear_pose_pid_ki = 0;
constexpr double linear_pose_pid_kd = 0;
// Angular pose PID
constexpr double angular_pose_pid_kp = 0.07;
constexpr double angular_pose_pid_ki = 0;
constexpr double angular_pose_pid_kd = 0;
// Linear speed PID
constexpr double linear_speed_pid_kp = 70;
constexpr double linear_speed_pid_ki = 2;
constexpr double linear_speed_pid_kd = 0;
// Angular speed PID
constexpr double angular_speed_pid_kp = 300;
constexpr double angular_speed_pid_ki = 1;
constexpr double angular_speed_pid_kd = 0;

// Linear pose PID integral limit
constexpr double linear_pose_pid_integral_limit = etl::numeric_limits<int16_t>::max();
// Angular pose PID integral limit
constexpr double angular_pose_pid_integral_limit = etl::numeric_limits<int16_t>::max();
// Linear speed PID integral limit
constexpr double linear_speed_pid_integral_limit = etl::numeric_limits<int16_t>::max() / 4;
// Angular speed PID integral limit
constexpr double angular_speed_pid_integral_limit = etl::numeric_limits<int16_t>::max() / 4;

// Linear treshold
constexpr double linear_treshold = 10;
// Angular treshold
constexpr double angular_treshold = 4;
// Linear deceleration treshold
constexpr double linear_deceleration_treshold = 400;

