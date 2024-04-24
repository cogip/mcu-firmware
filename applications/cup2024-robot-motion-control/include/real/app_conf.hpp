#pragma once

// Project includes
#include "etl/numeric.h"

// Linear pose PID
constexpr double linear_pose_pid_kp = 0.1;
constexpr double linear_pose_pid_ki = 0;
constexpr double linear_pose_pid_kd = 0;
// Angular pose PID
constexpr double angular_pose_pid_kp = 0.1;
constexpr double angular_pose_pid_ki = 0;
constexpr double angular_pose_pid_kd = 0;
// Linear speed PID
constexpr double linear_speed_pid_kp = 25;
constexpr double linear_speed_pid_ki = 5;
constexpr double linear_speed_pid_kd = 0;
// Angular speed PID
constexpr double angular_speed_pid_kp = 75;
constexpr double angular_speed_pid_ki = 3;
constexpr double angular_speed_pid_kd = 0;

// Linear pose PID integral limit
constexpr double linear_pose_pid_integral_limit = etl::numeric_limits<int16_t>::max();
// Angular pose PID integral limit
constexpr double angular_pose_pid_integral_limit = etl::numeric_limits<int16_t>::max();
// Linear speed PID integral limit
constexpr double linear_speed_pid_integral_limit = etl::numeric_limits<int16_t>::max();
// Angular speed PID integral limit
constexpr double angular_speed_pid_integral_limit = etl::numeric_limits<int16_t>::max();

// Linear threshold
constexpr double linear_threshold = 1;
// Angular threshold
constexpr double angular_threshold = 1;
// Angular intermediate threshold (when the robot turns on itself to go straight to its destination)
constexpr double angular_intermediate_threshold = 5;

