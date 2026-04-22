#pragma once

// Project includes
#include "etl/numeric.h"
#include "localization/LocalizationOTOS.hpp"
#include "otos/OTOS.hpp"
#include "parameter/Parameter.hpp"

using namespace cogip::parameter;

/* Motion motors */
#define MOTOR_LEFT 1
#define MOTOR_RIGHT 0

/* Quadrature decoding polarity */
inline Parameter<float, ReadOnly> qdec_left_polarity{1.0};
inline Parameter<float, ReadOnly> qdec_right_polarity{-1.0};

/// Motors properties
constexpr float motor_wheels_diameter_mm = 60.0;
constexpr float motor_wheels_distance_mm = 131;
// GA32Y-31ZY: 8000 RPM no-load motor, 5.18:1 reduction, so 8000/5.18 ≈ 1544
// RPM at the output shaft at nominal voltage.
// motor_constant = 6000 / output_RPM (see DifferentialDriveController.cpp).
constexpr float left_motor_constant = 3.886;
constexpr float right_motor_constant = 3.886;

constexpr float min_motor_speed_percent = 7.5;
constexpr float max_motor_speed_percent = 100;

/// Encoders Parameters
inline Parameter<float> left_encoder_wheels_diameter_mm{47.64768795921133};
inline Parameter<float> right_encoder_wheels_diameter_mm{47.792104995747586};
inline Parameter<float> encoder_wheels_distance_mm{275.7117596881151};
inline Parameter<float, ReadOnly> encoder_wheels_resolution_pulses{4096 * 4};

// Linear pose PID (QUADPID chain)
inline Parameter<float, NonNegative> linear_pose_pid_kp{0.1};
inline Parameter<float, NonNegative> linear_pose_pid_ki{0};
inline Parameter<float, NonNegative> linear_pose_pid_kd{0};
// Angular pose PID (QUADPID chain)
inline Parameter<float, NonNegative> angular_pose_pid_kp{0.2};
inline Parameter<float, NonNegative> angular_pose_pid_ki{0};
inline Parameter<float, NonNegative> angular_pose_pid_kd{0};
// Linear speed PID (QUADPID chain)
inline Parameter<float, NonNegative> linear_speed_pid_kp{7};
inline Parameter<float, NonNegative> linear_speed_pid_ki{0.4};
inline Parameter<float, NonNegative> linear_speed_pid_kd{0};
// Angular speed PID (QUADPID chain)
inline Parameter<float, NonNegative> angular_speed_pid_kp{4.5};
inline Parameter<float, NonNegative> angular_speed_pid_ki{0.125};
inline Parameter<float, NonNegative> angular_speed_pid_kd{0};

// ============================================================================
// Tracker chain PID gains (QUADPID_TRACKER)
// ============================================================================

// Tracker linear pose PID (tracker during MOVE_TO_POSITION)
// Ki helps eliminate steady-state error when tracker profile ends
inline Parameter<float, NonNegative> tracker_linear_pose_pid_kp{0.15};
inline Parameter<float, NonNegative> tracker_linear_pose_pid_ki{0.0};
inline Parameter<float, NonNegative> tracker_linear_pose_pid_kd{0};
// Tracker angular pose PID (tracker during ROTATE states)
inline Parameter<float, NonNegative> tracker_angular_pose_pid_kp{0.125};
inline Parameter<float, NonNegative> tracker_angular_pose_pid_ki{0};
inline Parameter<float, NonNegative> tracker_angular_pose_pid_kd{0};
// Tracker linear speed PID
inline Parameter<float, NonNegative> tracker_linear_speed_pid_kp{7};
inline Parameter<float, NonNegative> tracker_linear_speed_pid_ki{0.4};
inline Parameter<float, NonNegative> tracker_linear_speed_pid_kd{0};
// Tracker angular speed PID
inline Parameter<float, NonNegative> tracker_angular_speed_pid_kp{4.5};
inline Parameter<float, NonNegative> tracker_angular_speed_pid_ki{0.125};
inline Parameter<float, NonNegative> tracker_angular_speed_pid_kd{0};

// Linear threshold
constexpr float linear_threshold = 3;
// Angular threshold
constexpr float angular_threshold = 1;
// Angular intermediate threshold (when the robot turns on itself to go straight
// to its destination)
constexpr float angular_intermediate_threshold = 5;

// Linear anti-blocking
constexpr double platform_linear_anti_blocking_speed_threshold_mm_per_s = 0;
constexpr double platform_linear_anti_blocking_error_threshold_mm_per_s = 50;
constexpr double platform_linear_anti_blocking_blocked_cycles_nb_threshold = 65535;

// Speeds and accelerations/decelerations limits
constexpr float min_speed_mm_per_s = 0;     ///< Minimum speed (mm/s)
constexpr float max_speed_mm_per_s = 2000;  ///< Maximum speed (mm/s)
constexpr float max_acc_mm_per_s2 = 1000.0; ///< Maximum acceleration (mm/s²)
constexpr float max_dec_mm_per_s2 = 1000.0; ///< Maximum deceleration (mm/s²)

constexpr float min_speed_deg_per_s = 0;   ///< Minimum speed (deg/s)
constexpr float max_speed_deg_per_s = 720; ///< Maximum speed (deg/s)
constexpr float max_acc_deg_per_s2 = 360;  ///< Maximum acceleration (deg/s²)
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

// Linear pose PID integral limit
inline Parameter<float, NonNegative> linear_pose_pid_integral_limit{
    etl::numeric_limits<uint16_t>::max()};
// Angular pose PID integral limit
inline Parameter<float, NonNegative> angular_pose_pid_integral_limit{
    etl::numeric_limits<uint16_t>::max()};
// Linear speed PID integral limit
inline Parameter<float, NonNegative> linear_speed_pid_integral_limit{
    static_cast<float>(etl::numeric_limits<int16_t>::max())};
// Angular speed PID integral limit
inline Parameter<float, NonNegative> angular_speed_pid_integral_limit{
    static_cast<float>(etl::numeric_limits<int16_t>::max())};

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

// ============================================================================
// Localization (OTOS optical tracking sensor)
// ============================================================================

/// @brief Marker macro used by the parameter handler to decide whether to
/// register the OTOS calibration scalars in the CAN parameter registry.
#define ROBOT_HAS_OTOS 1

// OTOS I2C address
constexpr uint8_t otos_i2c_addr = 0x17;

// OTOS calibration scalars (range 0.872 to 1.127). Exposed as live
// Parameter<float> so they can be calibrated and updated at runtime via
// the CAN parameter handler.
inline Parameter<float> otos_linear_scalar{0.969f};
inline Parameter<float> otos_angular_scalar{0.994f};

// OTOS mounting offset relative to robot center (mm, degrees). Physical
// mounting geometry, calibrated at assembly, no runtime tuning.
constexpr float otos_offset_x_mm = 0.0f;
constexpr float otos_offset_y_mm = 0.0f;
constexpr float otos_offset_h_deg = 0.0f;

static cogip::localization::LocalizationOTOS::Parameters
    otos_params(otos_linear_scalar, otos_angular_scalar, otos_offset_x_mm, otos_offset_y_mm,
                otos_offset_h_deg);

static cogip::otos::OTOS otos_sensor(SOFT_I2C_DEV(0), otos_i2c_addr);
static cogip::localization::LocalizationOTOS robot_localization(otos_sensor, otos_params);
