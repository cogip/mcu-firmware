#pragma once

// Project includes
#include "etl/numeric.h"

/// @name Robot mechanical properties
///
/// To be computed:
///  - pulse_per_mm             : number of pulses per mm of coding wheel
///  - wheels_distance          : distance between coding wheels (pulses)
///  - pulse_per_degree         : number of pulses per degree of coding wheel
///
/// must be known:
///  - wheels_diameter          : coding wheel diameter (mm)
///  - wheels_distance_mm       : distance between coding wheels (mm)
///  - wheels_encoder_resolution: number of pulses by turn of coding wheels
///
/// Intermediate calculation:
///  - wheels_perimeter = pi*wheels_diameter
///  - pulse_per_mm = wheels_encoder_resolution / wheels_perimeter
///
/// @{

/// Motors properties
constexpr double motor_wheels_diameter_mm = 60.0;
constexpr double motor_wheels_distance_mm = 89.8;
constexpr double left_motor_constant = 7.079;
constexpr double right_motor_constant = 7.079;

constexpr double max_motor_speed_percent = 100;

/// Encoders properties
constexpr double left_encoder_wheels_diameter_mm = 60.0;
constexpr double right_encoder_wheels_diameter_mm = 60.0;
constexpr double encoder_wheels_distance_mm = 89.8;
constexpr double encoder_wheels_resolution_pulses = 16 * 4 * 8;

// Linear pose PID
constexpr double linear_pose_pid_kp = 2.0;
constexpr double linear_pose_pid_ki = 0;
constexpr double linear_pose_pid_kd = 0;
// Angular pose PID
constexpr double angular_pose_pid_kp = 2.0;
constexpr double angular_pose_pid_ki = 0;
constexpr double angular_pose_pid_kd = 0;
// Linear speed PID
constexpr double linear_speed_pid_kp = 8;
constexpr double linear_speed_pid_ki = 0.3;
constexpr double linear_speed_pid_kd = 0;
// Angular speed PID
constexpr double angular_speed_pid_kp = 1.1;
constexpr double angular_speed_pid_ki = 0.1;
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
constexpr double linear_threshold = 10;
// Angular threshold
constexpr double angular_threshold = 3;
// Angular intermediate threshold (when the robot turns on itself to go straight to its destination)
constexpr double angular_intermediate_threshold = 5;
