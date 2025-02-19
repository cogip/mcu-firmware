// Project includes
#include "app_conf.hpp"
#include "pid/PID.hpp"
#include "platform.hpp"
#include "platform_engine/PlatformEngine.hpp"
#include "quadpid_meta_controller/QuadPIDMetaController.hpp"

namespace cogip {

namespace pf {

namespace motion_control {

typedef uint8_t pid_id_t;

/// PID ids
constexpr auto START_LINE = __LINE__;
enum class PidEnum: pid_id_t {
    LINEAR_POSE_PID = 1,
    ANGULAR_POSE_PID = 2,
    LINEAR_SPEED_PID = 3,
    ANGULAR_SPEED_PID = 4
};
constexpr auto PID_COUNT = __LINE__ - START_LINE - 3;

/// Motion Control: 0x1000 - 0x1FFF
constexpr cogip::canpb::uuid_t state_uuid = 0x1001;
constexpr cogip::canpb::uuid_t pose_order_uuid = 0x1002;
constexpr cogip::canpb::uuid_t pose_reached_uuid = 0x1003;
constexpr cogip::canpb::uuid_t pose_start_uuid = 0x1004;
constexpr cogip::canpb::uuid_t pid_request_uuid = 0x1005;
constexpr cogip::canpb::uuid_t pid_uuid = 0x1006;
constexpr cogip::canpb::uuid_t brake_uuid = 0x1007;
constexpr cogip::canpb::uuid_t controller_uuid = 0x1008;

constexpr uint16_t motion_control_thread_period_ms = 20;    ///< controller thread loop period

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
constexpr double wheels_diameter_mm = 48.027;
constexpr double wheels_distance_mm = 274;
constexpr double wheels_encoder_resolution = 4096 * 4;
constexpr double wheels_perimeter = M_PI * wheels_diameter_mm;
constexpr double pulse_per_mm = wheels_encoder_resolution / wheels_perimeter;   ///< WHEELS_ENCODER_RESOLUTION / WHEELS_PERIMETER
constexpr double wheels_distance_pulse = wheels_distance_mm * pulse_per_mm;     ///< WHEELS_DISTANCE_MM * PULSE_PER_MM
constexpr double pulse_per_degree = (wheels_distance_pulse * 2 * M_PI) / 360;   ///< WHEELS_DISTANCE_PULSE * 2 * PI / 360
/// @}

/// Minimal PWM value
constexpr int pwm_minimal = 70;

/// @name Acceleration and speed profiles
/// @{
// Linear maximum speed and acceleration
constexpr double platform_linear_anti_blocking_speed_threshold_per_period = 0.25;
constexpr double platform_linear_anti_blocking_error_threshold_per_period = 1;
constexpr double platform_linear_anti_blocking_blocked_cycles_nb_threshold = 15;
constexpr double platform_min_speed_m_per_s = 0;  ///< Minimum speed (m/s)
constexpr double platform_max_speed_m_per_s = 2 ;  ///< Maximum speed (m/s)
constexpr double platform_max_acc_m_per_s2 = 3;   ///< Maximum acceleration (m/s²)
constexpr double platform_max_dec_m_per_s2 = 1.25;   ///< Maximum deceleration (m/s²)
constexpr double platform_max_acc_linear_mm_per_period2 = (
    (1000 * platform_max_acc_m_per_s2 * motion_control_thread_period_ms * motion_control_thread_period_ms) \
    / (1000 * 1000)
    );          ///< Maximum linear acceleration (mm/<motion_control_thread_period_ms>²)
constexpr double platform_max_dec_linear_mm_per_period2 = (
    (1000 * platform_max_dec_m_per_s2 * motion_control_thread_period_ms * motion_control_thread_period_ms) \
    / (1000 * 1000)
    );          ///< Maximum linear deceleration (mm/<motion_control_thread_period_ms>²)
constexpr double platform_min_speed_linear_mm_per_period = (
    (1000 * platform_min_speed_m_per_s * motion_control_thread_period_ms) \
    / 1000);    ///< Minimum linear speed (mm/<motion_control_thread_period_ms>)
constexpr double platform_max_speed_linear_mm_per_period = (
    (1000 * platform_max_speed_m_per_s * motion_control_thread_period_ms) \
    / 1000);    ///< Maximum linear speed (mm/<motion_control_thread_period_ms>)
constexpr double platform_low_speed_linear_mm_per_period = (platform_max_speed_linear_mm_per_period / 4);     ///< Low angular speed (deg/<motion_control_thread_period_ms>)
constexpr double platform_normal_speed_linear_mm_per_period = (platform_max_speed_linear_mm_per_period / 2);  ///< Normal angular speed (deg/<motion_control_thread_period_ms>)
// Linear antiblocking
constexpr bool platform_linear_antiblocking = true;

// Angular maximum speed and acceleration
constexpr double platform_min_speed_deg_per_s = 0; ///< Maximum speed (deg/s)
constexpr double platform_max_speed_deg_per_s = 360*4; ///< Maximum speed (deg/s)
constexpr double platform_max_acc_deg_per_s2 =  360*10;  ///< Maximum acceleration (deg/s²)
constexpr double platform_max_dec_deg_per_s2 =  360*4;  ///< Maximum deceleration (deg/s²)
constexpr double platform_max_acc_angular_deg_per_period2 = (
    (platform_max_acc_deg_per_s2 * motion_control_thread_period_ms * motion_control_thread_period_ms) \
    / (1000 * 1000)
    );  ///< Maximum angular acceleration (deg/<motion_control_thread_period_ms>)
constexpr double platform_max_dec_angular_deg_per_period2 = (
    (platform_max_dec_deg_per_s2 * motion_control_thread_period_ms * motion_control_thread_period_ms) \
    / (1000 * 1000)
    );  ///< Maximum angular deceleration (deg/<motion_control_thread_period_ms>)
constexpr double platform_min_speed_angular_deg_per_period = (
    (platform_min_speed_deg_per_s * motion_control_thread_period_ms) \
    / 1000);    ///< Minimum angular speed (deg/<motion_control_thread_period_ms>)
constexpr double platform_max_speed_angular_deg_per_period = (
    (platform_max_speed_deg_per_s * motion_control_thread_period_ms) \
    / 1000);    ///< Maximum angular speed (deg/<motion_control_thread_period_ms>)
constexpr double platform_low_speed_angular_deg_per_period = (platform_max_speed_angular_deg_per_period / 4);       ///< Low angular speed (deg/<motion_control_thread_period_ms>)
constexpr double platform_normal_speed_angular_deg_per_period = (platform_max_speed_angular_deg_per_period / 2);    ///< Normal angular speed (deg/<motion_control_thread_period_ms>)
// Angular antiblocking
constexpr bool platform_angular_antiblocking = false;
/// @}

/// Handle brake signal to stop the robot
void pf_handle_brake(cogip::canpb::ReadBuffer &buffer);

/// Get pose to reach from protobuf message
void pf_handle_target_pose(cogip::canpb::ReadBuffer &buffer);

/// Get start pose from protobuf message
void pf_handle_start_pose(cogip::canpb::ReadBuffer &buffer);

/// Initialize motion control
void pf_init_motion_control(void);

/// Start motion control
void pf_start_motion_control(void);

/// Reset all motion control components
void pf_motion_control_reset(void);

/// Make motion control engine thread loop disabled
void pf_disable_motion_control();

/// Make motion control engine thread loop enabled
void pf_enable_motion_control();

/// Send current robot pose in Protobuf format over UART
void pf_send_pb_pose(void);

/// Send current robot state in Protobuf format over UART
void pf_send_pb_state(void);

/// Print robot state
void pf_print_state(void);

/// Compute current speed and pose
void compute_current_speed_and_pose(
    cogip::cogip_defs::Polar &current_speed,    ///< [out]  robot current speed
    cogip::cogip_defs::Pose &current_pose       ///< [out]  robot current pose
    );

/// Apply the given command to the motors
void pf_motor_drive(
    const cogip::cogip_defs::Polar &command     ///< [in]   linear and angular speeds command
    );

} // namespace actuators

} // namespace pf

} // namespace cogip
