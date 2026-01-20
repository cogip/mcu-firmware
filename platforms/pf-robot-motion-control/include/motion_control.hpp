#pragma once

// Project includes
#include "app_conf.hpp"
#include "platform.hpp"

namespace cogip {

namespace pf {

namespace motion_control {

/// Conversion macro from speed in x/s to x/period (eg. mm/s or rad/s)
#define X_SEC_TO_X_PERIOD(speed, period) (((speed) * (period)) / 1000.0)
/// Conversion macro from acceleration in x/s² to x/period² (eg. mm/s² or
/// rad/s²)
#define X_SEC2_TO_X_PERIOD2(acc, period) ((acc) * (((period) * (period)) / (1000.0 * 1000.0)))

typedef uint8_t pid_id_t;

/// PID ids
constexpr auto START_LINE = __LINE__;
enum class PidEnum : pid_id_t {
    LINEAR_POSE_PID = 1,
    ANGULAR_POSE_PID = 2,
    LINEAR_SPEED_PID = 3,
    ANGULAR_SPEED_PID = 4
};
constexpr auto PID_COUNT = __LINE__ - START_LINE - 3;

constexpr uint16_t motion_control_thread_period_ms = 20; ///< controller thread loop period

/// Throttle divider for QUADPID pose loop controllers (execute every N cycles)
constexpr uint16_t quadpid_pose_controllers_throttle_divider = 1;

/// @name Acceleration and speed profiles
/// @{
/// Maximum linear acceleration/deceleration
/// (mm/<motion_control_thread_period_ms>²)
constexpr float platform_max_acc_linear_mm_per_period2 =
    X_SEC2_TO_X_PERIOD2(max_acc_mm_per_s2, motion_control_thread_period_ms);
constexpr float platform_max_dec_linear_mm_per_period2 =
    X_SEC2_TO_X_PERIOD2(max_dec_mm_per_s2, motion_control_thread_period_ms);

/// Minimum/Maximum linear speed (mm/<motion_control_thread_period_ms>)
constexpr float platform_min_speed_linear_mm_per_period =
    X_SEC_TO_X_PERIOD(min_speed_mm_per_s, motion_control_thread_period_ms);
constexpr float platform_max_speed_linear_mm_per_period =
    X_SEC_TO_X_PERIOD(max_speed_mm_per_s, motion_control_thread_period_ms);

/// Low angular speed (mm/<motion_control_thread_period_ms>)
constexpr float platform_low_speed_linear_mm_per_period =
    (platform_max_speed_linear_mm_per_period / 4);
/// Normal angular speed (mm/<motion_control_thread_period_ms>)
constexpr float platform_normal_speed_linear_mm_per_period =
    (platform_max_speed_linear_mm_per_period / 2);

/// Maximum angular acceleration/deceleration
/// (rad/<motion_control_thread_period_ms>²)
constexpr float platform_max_acc_angular_deg_per_period2 =
    X_SEC2_TO_X_PERIOD2(max_acc_deg_per_s2, motion_control_thread_period_ms);
constexpr float platform_max_dec_angular_deg_per_period2 =
    X_SEC2_TO_X_PERIOD2(max_dec_deg_per_s2, motion_control_thread_period_ms);

/// Minimum/Maximum angular speed (rad/<motion_control_thread_period_ms>)
constexpr float platform_min_speed_angular_deg_per_period =
    X_SEC_TO_X_PERIOD(min_speed_deg_per_s, motion_control_thread_period_ms);
constexpr float platform_max_speed_angular_deg_per_period =
    X_SEC_TO_X_PERIOD(max_speed_deg_per_s, motion_control_thread_period_ms);

/// Low angular speed (rad/<motion_control_thread_period_ms>)
constexpr float platform_low_speed_angular_deg_per_period =
    (platform_max_speed_angular_deg_per_period / 4);
/// Normal angular speed (deg/<motion_control_thread_period_ms>)
constexpr float platform_normal_speed_angular_deg_per_period =
    (platform_max_speed_angular_deg_per_period / 2);

/// Linear anti-cloking setup
constexpr double platform_linear_anti_blocking_speed_threshold_mm_per_period =
    (motion_control_thread_period_ms * platform_linear_anti_blocking_speed_threshold_mm_per_s) /
    1000;
constexpr double platform_linear_anti_blocking_error_threshold_mm_per_period =
    (motion_control_thread_period_ms * platform_linear_anti_blocking_error_threshold_mm_per_s) /
    1000;
/// @}

/// Handle brake signal to stop the robot
void pf_handle_brake(cogip::canpb::ReadBuffer& buffer);

/// Handle game end signal to stop the robot
void pf_handle_game_end(cogip::canpb::ReadBuffer& buffer);

/// Get pose to reach from protobuf message
void pf_handle_target_pose(cogip::canpb::ReadBuffer& buffer);

/// Get start pose from protobuf message
void pf_handle_start_pose(cogip::canpb::ReadBuffer& buffer);

/// Initialize motion control
void pf_init_motion_control(void);

/// Start motion control
void pf_start_motion_control(void);

/// Reset all motion control components
void pf_motion_control_reset_controllers(void);

/// Make motion control engine thread loop disabled
void pf_disable_motion_control();

/// Make motion control engine thread loop enabled
void pf_enable_motion_control();

/// Send current robot pose in Protobuf format over CAN
void pf_send_pb_pose(void);

/// Send current robot state in Protobuf format over CAN
void pf_send_pb_state(void);

/// Send encoder telemetry data
void pf_send_encoder_telemetry(void);

} // namespace motion_control

} // namespace pf

} // namespace cogip
