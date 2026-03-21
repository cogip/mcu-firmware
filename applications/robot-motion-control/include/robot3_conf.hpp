#pragma once

#include "etl/numeric.h"

/* Motion motors */
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

/* Quadrature decoding polarity */
constexpr float default_qdec_left_polarity = -1.0;
constexpr float default_qdec_right_polarity = 1.0;

/// Motors properties
constexpr float motor_wheels_diameter_mm = 60.0;
constexpr float motor_wheels_distance_mm = 89.5;
constexpr float left_motor_constant = 7.079;
constexpr float right_motor_constant = 7.079;

constexpr float min_motor_speed_percent = 0;
constexpr float max_motor_speed_percent = 100;

/// Encoders properties
constexpr float default_left_encoder_wheels_diameter_mm = motor_wheels_diameter_mm;
constexpr float default_right_encoder_wheels_diameter_mm = motor_wheels_diameter_mm;
constexpr float default_encoder_wheels_distance_mm = motor_wheels_distance_mm;
constexpr float default_encoder_wheels_resolution_pulses = 16 * 4 * 8;

// Linear pose PID
constexpr float default_linear_pose_pid_kp = 1;
constexpr float default_linear_pose_pid_ki = 0.0;
constexpr float default_linear_pose_pid_kd = 0.0;
// Angular pose PID
constexpr float default_angular_pose_pid_kp = 1;
constexpr float default_angular_pose_pid_ki = 0.0;
constexpr float default_angular_pose_pid_kd = 0.0;
// Linear speed PID
constexpr float default_linear_speed_pid_kp = 10;
constexpr float default_linear_speed_pid_ki = 1;
constexpr float default_linear_speed_pid_kd = 0.0;
// Angular speed PID
constexpr float default_angular_speed_pid_kp = 10;
constexpr float default_angular_speed_pid_ki = 1;
constexpr float default_angular_speed_pid_kd = 0.0;

// ============================================================================
// Tracker chain PID gains (QUADPID_TRACKER)
// ============================================================================

// Tracker linear pose PID (tracker during MOVE_TO_POSITION)
constexpr float default_tracker_linear_pose_pid_kp = 0.1;
constexpr float default_tracker_linear_pose_pid_ki = 0.0;
constexpr float default_tracker_linear_pose_pid_kd = 0;
// Tracker angular pose PID (tracker during ROTATE states)
constexpr float default_tracker_angular_pose_pid_kp = 0.2;
constexpr float default_tracker_angular_pose_pid_ki = 0;
constexpr float default_tracker_angular_pose_pid_kd = 0;
// Tracker linear speed PID
constexpr float default_tracker_linear_speed_pid_kp = 10;
constexpr float default_tracker_linear_speed_pid_ki = 1;
constexpr float default_tracker_linear_speed_pid_kd = 0;
// Tracker angular speed PID
constexpr float default_tracker_angular_speed_pid_kp = 10;
constexpr float default_tracker_angular_speed_pid_ki = 1;
constexpr float default_tracker_angular_speed_pid_kd = 0;

// Linear threshold
constexpr float linear_threshold = 5;
// Angular threshold
constexpr float angular_threshold = 5;
// Angular intermediate threshold (when the robot turns on itself to go straight
// to its destination)
constexpr float angular_intermediate_threshold = 5;

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

/// Safety clamp ratio for speed/acceleration filters
/// The filters clamp at ratio × nominal max to catch runaway values
/// while allowing normal operation with some margin
constexpr float speed_clamp_ratio = 1.2f;
constexpr float acceleration_clamp_ratio = 1.2f;

// Linear antiblocking
constexpr bool platform_linear_antiblocking = false;
// Angular antiblocking
constexpr bool angular_antiblocking = false;

// PID integral limits (use float max when ki == 0 to avoid constexpr div-by-zero)
constexpr float default_linear_pose_pid_integral_limit = etl::numeric_limits<uint16_t>::max();
constexpr float default_angular_pose_pid_integral_limit = etl::numeric_limits<uint16_t>::max();
constexpr float default_linear_speed_pid_integral_limit =
    (default_linear_speed_pid_ki != 0) ? (max_speed_mm_per_s / default_linear_speed_pid_ki)
                                       : (etl::numeric_limits<float>::max());
constexpr float default_angular_speed_pid_integral_limit =
    (default_angular_speed_pid_ki != 0) ? (max_speed_deg_per_s / default_angular_speed_pid_ki)
                                        : (etl::numeric_limits<float>::max());

// Tracker PID integral limits
constexpr float default_tracker_linear_pose_pid_integral_limit =
    etl::numeric_limits<uint16_t>::max();
constexpr float default_tracker_angular_pose_pid_integral_limit =
    etl::numeric_limits<uint16_t>::max();
constexpr float default_tracker_linear_speed_pid_integral_limit =
    (default_tracker_linear_speed_pid_ki != 0)
        ? (max_speed_mm_per_s / default_tracker_linear_speed_pid_ki)
        : (etl::numeric_limits<float>::max());
constexpr float default_tracker_angular_speed_pid_integral_limit =
    (default_tracker_angular_speed_pid_ki != 0)
        ? (max_speed_deg_per_s / default_tracker_angular_speed_pid_ki)
        : (etl::numeric_limits<float>::max());
