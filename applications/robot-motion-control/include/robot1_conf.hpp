#pragma once

// Project includes
#include "etl/numeric.h"

/* Motion motors */
#define MOTOR_LEFT 1
#define MOTOR_RIGHT 0

/* Quadrature decoding polarity */
#define QDEC_LEFT_POLARITY 1
#define QDEC_RIGHT_POLARITY -1

/// Motors properties
constexpr float motor_wheels_diameter_mm = 50.8;
constexpr float motor_wheels_distance_mm = 172;
constexpr float left_motor_constant = 3.703;
constexpr float right_motor_constant = 3.703;

constexpr float min_motor_speed_percent = 10;
constexpr float max_motor_speed_percent = 100;

/// Encoders properties
constexpr float left_encoder_wheels_diameter_mm = 47.64768795921133;
constexpr float right_encoder_wheels_diameter_mm = 47.792104995747586;
constexpr float encoder_wheels_distance_mm = 275.7117596881151;
constexpr float encoder_wheels_resolution_pulses = 4096 * 4;

// Linear pose PID
constexpr float linear_pose_pid_kp = 0.2;
constexpr float linear_pose_pid_ki = 0;
constexpr float linear_pose_pid_kd = 0;
// Angular pose PID
constexpr float angular_pose_pid_kp = 0.3;
constexpr float angular_pose_pid_ki = 0;
constexpr float angular_pose_pid_kd = 0;
// Linear speed PID
constexpr float linear_speed_pid_kp = 3.;
constexpr float linear_speed_pid_ki = 0.8;
constexpr float linear_speed_pid_kd = 0;
// Angular speed PID
constexpr float angular_speed_pid_kp = 5.5;
constexpr float angular_speed_pid_ki = 0.6;
constexpr float angular_speed_pid_kd = 0;

// Linear threshold
constexpr float linear_threshold = 1;
// Angular threshold
constexpr float angular_threshold = 1;
// Angular intermediate threshold (when the robot turns on itself to go straight
// to its destination)
constexpr float angular_intermediate_threshold = 20;

// Linear anti-blocking
constexpr double platform_linear_anti_blocking_speed_threshold_mm_per_s = 12.5;
constexpr double platform_linear_anti_blocking_error_threshold_mm_per_s = 50;
constexpr double platform_linear_anti_blocking_blocked_cycles_nb_threshold = 10;

// Speeds and accelerations/decelerations limits
constexpr float min_speed_mm_per_s = 0;    ///< Minimum speed (mm/s)
constexpr float max_speed_mm_per_s = 2000; ///< Maximum speed (mm/s)
constexpr float max_acc_mm_per_s2 = 500.0; ///< Maximum acceleration (mm/s²)
constexpr float max_dec_mm_per_s2 = 500.0; ///< Maximum deceleration (mm/s²)

constexpr float min_speed_deg_per_s = 0;   ///< Maximum speed (deg/s)
constexpr float max_speed_deg_per_s = 720; ///< Maximum speed (deg/s)
constexpr float max_acc_deg_per_s2 = 360;  ///< Maximum acceleration (deg/s²)
constexpr float max_dec_deg_per_s2 = 360;  ///< Maximum deceleration (deg/s²)

// Linear antiblocking
constexpr bool platform_linear_antiblocking = true;
// Angular antiblocking
constexpr bool angular_antiblocking = false;

// Linear pose PID integral limit
constexpr float linear_pose_pid_integral_limit = etl::numeric_limits<uint16_t>::max();
// Angular pose PID integral limit
constexpr float angular_pose_pid_integral_limit = etl::numeric_limits<uint16_t>::max();
// Linear speed PID integral limit
constexpr float linear_speed_pid_integral_limit = max_speed_mm_per_s / linear_speed_pid_ki;
// Angular speed PID integral limit
constexpr float angular_speed_pid_integral_limit = max_speed_deg_per_s / angular_speed_pid_ki;
