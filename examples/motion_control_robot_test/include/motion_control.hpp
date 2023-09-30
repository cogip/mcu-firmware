// Project includes
#include "pid_ng/PID.hpp"
#include "platform_engine/PlatformEngine.hpp"
#include "quadpid_meta_controller/QuadPIDMetaController.hpp"

namespace cogip {

namespace pf {

namespace motion_control {

constexpr uint16_t motion_control_thread_period_ms = 20;    ///< controller thread loop period

/// @name Acceleration and speed profiles
/// @{
// Linear maximum speed and acceleration
constexpr double platform_max_speed_m_per_s = 1;  ///< Maximum speed (m/s)
constexpr double platform_max_speed_linear_mm_per_period = (
    (1000 * platform_max_speed_m_per_s * motion_control_thread_period_ms) \
    / 1000);    ///< Maximum linear speed (mm/<motion_control_thread_period_ms>)
constexpr double platform_normal_speed_linear_mm_per_period = (2 * platform_max_speed_linear_mm_per_period) / 3;
constexpr double platform_low_speed_linear_mm_per_period = platform_max_speed_linear_mm_per_period / 3;

// Angular maximum speed and acceleration
constexpr double platform_max_speed_deg_per_s = 720; ///< Maximum speed (deg/s)
constexpr double platform_max_speed_angular_deg_per_period = (
    (platform_max_speed_deg_per_s * motion_control_thread_period_ms) \
    / 1000);    ///< Maximum angular speed (deg/<motion_control_thread_period_ms>)
constexpr double platform_normal_speed_angular_deg_per_period = (2 * platform_max_speed_angular_deg_per_period) / 3;
constexpr double platform_low_speed_angular_deg_per_period = platform_max_speed_angular_deg_per_period / 3;
/// @}

} // namespace actuators

} // namespace pf

} // namespace cogip
