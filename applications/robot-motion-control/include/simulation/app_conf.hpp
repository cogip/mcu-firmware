#pragma once

// Project includes
#include "etl/numeric.h"

/* Motion motors */
#define MOTOR_LEFT  1
#define MOTOR_RIGHT 0

/* Quadrature decoding polarity */
#define QDEC_LEFT_POLARITY  1
#define QDEC_RIGHT_POLARITY -1

/// Motors properties
constexpr double motor_wheels_diameter_mm = 60.0;
constexpr double motor_wheels_distance_mm = 88.0;
constexpr double left_motor_constant = 7.079;
constexpr double right_motor_constant = 7.079;

constexpr double min_motor_speed_percent = 0;
constexpr double max_motor_speed_percent = 100;

/// Encoders properties
constexpr double left_encoder_wheels_diameter_mm = motor_wheels_diameter_mm;
constexpr double right_encoder_wheels_diameter_mm = motor_wheels_diameter_mm;
constexpr double encoder_wheels_distance_mm = motor_wheels_distance_mm;
constexpr double encoder_wheels_resolution_pulses = 16 * 8 * 4;

/// @name Simulation mechanical properties
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
constexpr double wheels_perimeter = M_PI * left_encoder_wheels_diameter_mm;
constexpr double pulse_per_mm = encoder_wheels_resolution_pulses / wheels_perimeter;
constexpr double wheels_distance_pulse = encoder_wheels_distance_mm * pulse_per_mm;
constexpr double pulse_per_degree = (wheels_distance_pulse * 2 * M_PI) / 360;
/// @}


// Linear pose PID
constexpr double linear_pose_pid_kp = 0.0325;
constexpr double linear_pose_pid_ki = 0;
constexpr double linear_pose_pid_kd = 0;
// Angular pose PID
constexpr double angular_pose_pid_kp = 0.06;
constexpr double angular_pose_pid_ki = 0;
constexpr double angular_pose_pid_kd = 0;
// Linear speed PID
constexpr double linear_speed_pid_kp = 0;
constexpr double linear_speed_pid_ki = 0;
constexpr double linear_speed_pid_kd = 0;
// Angular speed PID
constexpr double angular_speed_pid_kp = 0;
constexpr double angular_speed_pid_ki = 0;
constexpr double angular_speed_pid_kd = 0;

// Linear pose PID integral limit
constexpr double linear_pose_pid_integral_limit = etl::numeric_limits<uint16_t>::max();
// Angular pose PID integral limit
constexpr double angular_pose_pid_integral_limit = etl::numeric_limits<uint16_t>::max();
// Linear speed PID integral limit
constexpr double linear_speed_pid_integral_limit = etl::numeric_limits<uint16_t>::max();
// Angular speed PID integral limit
constexpr double angular_speed_pid_integral_limit = etl::numeric_limits<uint16_t>::max();

// Linear threshold
constexpr double linear_threshold = 1;
// Angular threshold
constexpr double angular_threshold = 2;
// Angular intermediate threshold (when the robot turns on itself to go straight to its destination)
constexpr double angular_intermediate_threshold = 1;

// TODO:
constexpr double platform_linear_anti_blocking_speed_threshold_per_period = 0.25;
constexpr double platform_linear_anti_blocking_error_threshold_per_period = 1;
constexpr double platform_linear_anti_blocking_blocked_cycles_nb_threshold = 15;

constexpr double min_speed_mm_per_s = 0;    ///< Minimum speed (mm/s)
constexpr double max_speed_mm_per_s = 500;  ///< Maximum speed (mm/s)
constexpr double max_acc_mm_per_s2 = 500.0; ///< Maximum acceleration (mm/s²)
constexpr double max_dec_mm_per_s2 = 500.0; ///< Maximum deceleration (mm/s²)

constexpr double min_speed_deg_per_s = 0;  ///< Maximum speed (deg/s)
constexpr double max_speed_deg_per_s = 60; ///< Maximum speed (deg/s)
constexpr double max_acc_deg_per_s2 = 30;  ///< Maximum acceleration (deg/s²)
constexpr double max_dec_deg_per_s2 = 30;  ///< Maximum deceleration (deg/s²)

// Linear antiblocking
constexpr bool platform_linear_antiblocking = true;
// Angular antiblocking
constexpr bool angular_antiblocking = false;