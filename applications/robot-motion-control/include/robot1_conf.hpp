#pragma once

// Project includes
#include "etl/numeric.h"
#include "parameter/Parameter.hpp"

using namespace cogip::parameter;

/* Motion motors */
#define MOTOR_LEFT 1
#define MOTOR_RIGHT 0

/* Quadrature decoding polarity */
inline Parameter<float, ReadOnly> qdec_left_polarity{1.0};
inline Parameter<float, ReadOnly> qdec_right_polarity{-1.0};

/// Motors properties
constexpr float motor_wheels_diameter_mm = 50.8;
constexpr float motor_wheels_distance_mm = 172;
constexpr float left_motor_constant = 3.703;
constexpr float right_motor_constant = 3.703;

constexpr float min_motor_speed_percent = 10;
constexpr float max_motor_speed_percent = 100;

/// Encoders Parameters
inline Parameter<float> left_encoder_wheels_diameter_mm{47.64768795921133};
inline Parameter<float> right_encoder_wheels_diameter_mm{47.792104995747586};
inline Parameter<float> encoder_wheels_distance_mm{275.7117596881151};
inline Parameter<float, ReadOnly> encoder_wheels_resolution_pulses{4096 * 4};

// Linear pose PID (QUADPID chain)
inline Parameter<float, NonNegative> linear_pose_pid_kp{0.2};
inline Parameter<float, NonNegative> linear_pose_pid_ki{0};
inline Parameter<float, NonNegative> linear_pose_pid_kd{0};
// Angular pose PID (QUADPID chain)
inline Parameter<float, NonNegative> angular_pose_pid_kp{0.1};
inline Parameter<float, NonNegative> angular_pose_pid_ki{0};
inline Parameter<float, NonNegative> angular_pose_pid_kd{0};
// Linear speed PID (QUADPID chain)
inline Parameter<float, NonNegative> linear_speed_pid_kp{3.};
inline Parameter<float, NonNegative> linear_speed_pid_ki{0.8};
inline Parameter<float, NonNegative> linear_speed_pid_kd{0};
// Angular speed PID (QUADPID chain)
inline Parameter<float, NonNegative> angular_speed_pid_kp{5.5};
inline Parameter<float, NonNegative> angular_speed_pid_ki{0.6};
inline Parameter<float, NonNegative> angular_speed_pid_kd{0};

// ============================================================================
// Tracker chain PID gains (QUADPID_TRACKER)
// ============================================================================

// Tracker linear pose PID (tracker during MOVE_TO_POSITION)
// Ki helps eliminate steady-state error when tracker profile ends
inline Parameter<float, NonNegative> tracker_linear_pose_pid_kp{0.1};
inline Parameter<float, NonNegative> tracker_linear_pose_pid_ki{0.0};
inline Parameter<float, NonNegative> tracker_linear_pose_pid_kd{0};
// Tracker angular pose PID (tracker during ROTATE states)
inline Parameter<float, NonNegative> tracker_angular_pose_pid_kp{0.3};
inline Parameter<float, NonNegative> tracker_angular_pose_pid_ki{0};
inline Parameter<float, NonNegative> tracker_angular_pose_pid_kd{0};
// Tracker linear speed PID
inline Parameter<float, NonNegative> tracker_linear_speed_pid_kp{3};
inline Parameter<float, NonNegative> tracker_linear_speed_pid_ki{0.8};
inline Parameter<float, NonNegative> tracker_linear_speed_pid_kd{0};
// Tracker angular speed PID
inline Parameter<float, NonNegative> tracker_angular_speed_pid_kp{5.5};
inline Parameter<float, NonNegative> tracker_angular_speed_pid_ki{0.6};
inline Parameter<float, NonNegative> tracker_angular_speed_pid_kd{0};

// ============================================================================
// Corrector pose PID gains (QUADPID_TRACKER - direct branch)
// Used when tracker profile is invalidated to reach target position
// ============================================================================

// Corrector linear pose PID (used when linear profile is invalidated)
inline Parameter<float, NonNegative> corrector_linear_pose_pid_kp{0.2};
inline Parameter<float, NonNegative> corrector_linear_pose_pid_ki{0};
inline Parameter<float, NonNegative> corrector_linear_pose_pid_kd{0};
// Corrector angular pose PID (used when angular profile is invalidated)
// Also used for heading maintenance during MOVE_TO_POSITION
inline Parameter<float, NonNegative> corrector_angular_pose_pid_kp{0.1};
inline Parameter<float, NonNegative> corrector_angular_pose_pid_ki{0};
inline Parameter<float, NonNegative> corrector_angular_pose_pid_kd{0};

// ============================================================================
// Corrector speed PID gains (QUADPID_TRACKER - direct branch)
// Used in speed loop when profile is invalidated (separate from tracker PIDs)
// ============================================================================

// Corrector linear speed PID (used in direct mode speed loop)
inline Parameter<float, NonNegative> corrector_linear_speed_pid_kp{3};
inline Parameter<float, NonNegative> corrector_linear_speed_pid_ki{0.8};
inline Parameter<float, NonNegative> corrector_linear_speed_pid_kd{0};

// Corrector angular speed PID (used in direct mode speed loop)
inline Parameter<float, NonNegative> corrector_angular_speed_pid_kp{5};
inline Parameter<float, NonNegative> corrector_angular_speed_pid_ki{0.6};
inline Parameter<float, NonNegative> corrector_angular_speed_pid_kd{0};

// Linear threshold
constexpr float linear_threshold = 2;
// Angular threshold
constexpr float angular_threshold = 1;
// Angular intermediate threshold (when the robot turns on itself to go straight
// to its destination)
constexpr float angular_intermediate_threshold = 1;

// Linear anti-blocking
constexpr double platform_linear_anti_blocking_speed_threshold_mm_per_s = 12.5;
constexpr double platform_linear_anti_blocking_error_threshold_mm_per_s = 50;
constexpr double platform_linear_anti_blocking_blocked_cycles_nb_threshold = 10;

// Speeds and accelerations/decelerations limits
constexpr float min_speed_mm_per_s = 0;     ///< Minimum speed (mm/s)
constexpr float max_speed_mm_per_s = 2000;  ///< Maximum speed (mm/s)
constexpr float max_acc_mm_per_s2 = 1200.0; ///< Maximum acceleration (mm/s²)
constexpr float max_dec_mm_per_s2 = 500.0;  ///< Maximum deceleration (mm/s²)

constexpr float min_speed_deg_per_s = 0;   ///< Minimum speed (deg/s)
constexpr float max_speed_deg_per_s = 720; ///< Maximum speed (deg/s)
constexpr float max_acc_deg_per_s2 = 720;  ///< Maximum acceleration (deg/s²)
constexpr float max_dec_deg_per_s2 = 360;  ///< Maximum deceleration (deg/s²)

/// Safety clamp ratio for speed/acceleration filters
/// The filters clamp at ratio × nominal max to catch runaway values
/// while allowing normal operation with some margin
constexpr float speed_clamp_ratio = 1.2f;
constexpr float acceleration_clamp_ratio = 1.2f;

/// @name Pure Pursuit lookahead parameters
/// @{
/// The lookahead distance determines how far ahead the robot looks on the path.
/// It adapts to speed using the formula:
///
///     lookahead = min_lookahead + current_speed × ratio
///
/// Dimensioning trade-off (wheelbase = 275.7mm):
/// - Smaller lookahead → follows path closely but may oscillate
/// - Larger lookahead → smooth trajectory but cuts corners
///
/// Current tuning (1.5× wheelbase max) for tighter corner following:
/// - min_lookahead = 220mm (0.8× wheelbase)
/// - max_lookahead = 420mm (1.5× wheelbase)
/// - ratio = (420 - 220) / 40 = 5.0
///
/// With max_speed = 2000mm/s, period = 20ms:
/// - max_speed_per_period = 2000 × 0.020 = 40 mm/period
constexpr float pure_pursuit_min_lookahead_mm = 220.0f;
constexpr float pure_pursuit_max_lookahead_mm = 420.0f;
constexpr float pure_pursuit_lookahead_speed_ratio = 5.0f;
/// Initial rotation threshold (deg) - tolerance for ROTATING_TO_DIRECTION state
/// This can be larger than angular_threshold since the robot will correct during path following
constexpr float pure_pursuit_initial_rotation_threshold_deg = 5.0f;
/// @}

// Linear antiblocking
constexpr bool platform_linear_antiblocking = true;
// Angular antiblocking
constexpr bool angular_antiblocking = false;

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

// Tracker linear pose PID integral limit
// Limit = max_speed / ki to prevent integral windup
inline Parameter<float, NonNegative> tracker_linear_pose_pid_integral_limit{
    max_speed_mm_per_s / tracker_linear_pose_pid_ki.get()};
// Tracker angular pose PID integral limit
inline Parameter<float, NonNegative> tracker_angular_pose_pid_integral_limit{
    etl::numeric_limits<uint16_t>::max()};
// Tracker linear speed PID integral limit
inline Parameter<float, NonNegative> tracker_linear_speed_pid_integral_limit{
    max_speed_mm_per_s / tracker_linear_speed_pid_ki.get()};
// Tracker angular speed PID integral limit
inline Parameter<float, NonNegative> tracker_angular_speed_pid_integral_limit{
    max_speed_deg_per_s / tracker_angular_speed_pid_ki.get()};

// Corrector linear pose PID integral limit
inline Parameter<float, NonNegative> corrector_linear_pose_pid_integral_limit{
    etl::numeric_limits<uint16_t>::max()};
// Corrector angular pose PID integral limit
inline Parameter<float, NonNegative> corrector_angular_pose_pid_integral_limit{
    etl::numeric_limits<uint16_t>::max()};

// Corrector linear speed PID integral limit
inline Parameter<float, NonNegative> corrector_linear_speed_pid_integral_limit{
    max_speed_mm_per_s / corrector_linear_speed_pid_ki.get()};
// Corrector angular speed PID integral limit
inline Parameter<float, NonNegative> corrector_angular_speed_pid_integral_limit{
    max_speed_deg_per_s / corrector_angular_speed_pid_ki.get()};
