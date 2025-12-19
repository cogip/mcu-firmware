#pragma once

// Project includes
#include "etl/numeric.h"
#include "parameter/Parameter.hpp"

using namespace cogip::parameter;

/* Motion motors */
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

/* Quadrature decoding polarity */
inline Parameter<float, ReadOnly> qdec_left_polarity{-1.0};
inline Parameter<float, ReadOnly> qdec_right_polarity{1.0};

/// Motors properties
constexpr float motor_wheels_diameter_mm = 60.0;
constexpr float motor_wheels_distance_mm = 89.5;
constexpr float left_motor_constant = 7.079;
constexpr float right_motor_constant = 7.079;

constexpr float min_motor_speed_percent = 0;
constexpr float max_motor_speed_percent = 100;

/// Encoders properties
inline Parameter<float> left_encoder_wheels_diameter_mm{motor_wheels_diameter_mm};
inline Parameter<float> right_encoder_wheels_diameter_mm{motor_wheels_diameter_mm};
inline Parameter<float> encoder_wheels_distance_mm{motor_wheels_distance_mm};
inline Parameter<float, ReadOnly> encoder_wheels_resolution_pulses{16 * 4 * 8};

// Linear pose PID
inline Parameter<float, NonNegative> linear_pose_pid_kp{1};
inline Parameter<float, NonNegative> linear_pose_pid_ki{0.0};
inline Parameter<float, NonNegative> linear_pose_pid_kd{0.0};
// Angular pose PID
inline Parameter<float, NonNegative> angular_pose_pid_kp{1};
inline Parameter<float, NonNegative> angular_pose_pid_ki{0.0};
inline Parameter<float, NonNegative> angular_pose_pid_kd{0.0};
// Linear speed PID
inline Parameter<float, NonNegative> linear_speed_pid_kp{10};
inline Parameter<float, NonNegative> linear_speed_pid_ki{1};
inline Parameter<float, NonNegative> linear_speed_pid_kd{0.0};
// Angular speed PID
inline Parameter<float, NonNegative> angular_speed_pid_kp{10};
inline Parameter<float, NonNegative> angular_speed_pid_ki{1};
inline Parameter<float, NonNegative> angular_speed_pid_kd{0.0};

// Linear threshold
constexpr float linear_threshold = 5;
// Angular threshold
constexpr float angular_threshold = 5;
// Angular intermediate threshold (when the robot turns on itself to go straight
// to its destination)
constexpr float angular_intermediate_threshold = 5;

// TODO:
constexpr double platform_linear_anti_blocking_speed_threshold_mm_per_s = 12.5;
constexpr double platform_linear_anti_blocking_error_threshold_mm_per_s = 50;
constexpr double platform_linear_anti_blocking_blocked_cycles_nb_threshold = 10;

constexpr float min_speed_mm_per_s = 0;      ///< Minimum speed (mm/s)
constexpr float max_speed_mm_per_s = 1000.0; ///< Maximum speed (mm/s)
constexpr float max_acc_mm_per_s2 = 750.0;   ///< Maximum acceleration (mm/s²)
constexpr float max_dec_mm_per_s2 = 1000.0;  ///< Maximum deceleration (mm/s²)

constexpr float min_speed_deg_per_s = 20;  ///< Maximum speed (deg/s)
constexpr float max_speed_deg_per_s = 360; ///< Maximum speed (deg/s)
constexpr float max_acc_deg_per_s2 = 180;  ///< Maximum acceleration (deg/s²)
constexpr float max_dec_deg_per_s2 = 360;  ///< Maximum deceleration (deg/s²)

// Linear pose PID integral limit
inline Parameter<float, NonNegative> linear_pose_pid_integral_limit{
    etl::numeric_limits<uint16_t>::max()};
// Angular pose PID integral limit
inline Parameter<float, NonNegative> angular_pose_pid_integral_limit{
    etl::numeric_limits<uint16_t>::max()};
// Linear speed PID integral limit
inline Parameter<float, NonNegative> linear_speed_pid_integral_limit{max_speed_mm_per_s /
                                                                     linear_speed_pid_ki.get()};
// Angular speed PID integral limit
inline Parameter<float, NonNegative> angular_speed_pid_integral_limit{max_speed_deg_per_s /
                                                                      angular_speed_pid_ki.get()};

// Linear antiblocking
constexpr bool platform_linear_antiblocking = false;
// Angular antiblocking
constexpr bool angular_antiblocking = false;
