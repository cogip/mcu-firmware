#pragma once

// Project includes
#include "etl/numeric.h"

// Linear pose PID
constexpr double linear_pose_pid_kp = 0.05;
constexpr double linear_pose_pid_ki = 0;
constexpr double linear_pose_pid_kd = 0;
// Angular pose PID
constexpr double angular_pose_pid_kp = 0.065;
constexpr double angular_pose_pid_ki = 0;
constexpr double angular_pose_pid_kd = 0;
// Linear speed PID
constexpr double linear_speed_pid_kp = 125;
constexpr double linear_speed_pid_ki = 40;
constexpr double linear_speed_pid_kd = 0;
// Angular speed PID
constexpr double angular_speed_pid_kp = 250;
constexpr double angular_speed_pid_ki = 75;
constexpr double angular_speed_pid_kd = 0;

// Linear pose PID integral limit
constexpr double linear_pose_pid_integral_limit = etl::numeric_limits<uint16_t>::max();
// Angular pose PID integral limit
constexpr double angular_pose_pid_integral_limit = etl::numeric_limits<uint16_t>::max();
// Linear speed PID integral limit
constexpr double linear_speed_pid_integral_limit = etl::numeric_limits<uint16_t>::max();
//constexpr double linear_speed_pid_integral_limit = 2000;
// Angular speed PID integral limit
constexpr double angular_speed_pid_integral_limit = etl::numeric_limits<uint16_t>::max();
//constexpr double angular_speed_pid_integral_limit = 2000;

// Linear treshold
constexpr double linear_treshold = 5;
// Angular treshold
constexpr double angular_treshold = 5;
