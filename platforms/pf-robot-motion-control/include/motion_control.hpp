#pragma once

// Project includes
#include "motion_control_parameters.hpp"
#include "path/Path.hpp"
#include "platform.hpp"

namespace cogip {

namespace pf {

namespace motion_control {

/// Path instance for waypoint navigation
inline cogip::path::Path motion_control_path;

/// Throttle divider for QUADPID pose loop controllers (execute every N cycles)
constexpr uint16_t quadpid_pose_controllers_throttle_divider = 1;

/// Handle brake signal to stop the robot
void pf_handle_brake(const cogip::canpb::ReadBuffer& buffer);

/// Handle game end signal to stop the robot
void pf_handle_game_end(const cogip::canpb::ReadBuffer& buffer);

/// Get pose to reach from protobuf message
void pf_handle_target_pose(cogip::canpb::ReadBuffer& buffer);

/// Handle speed order for speed PID tuning (requires active speed tuning chain)
void pf_handle_speed_order(cogip::canpb::ReadBuffer& buffer);

/// Get start pose from protobuf message
void pf_handle_start_pose(cogip::canpb::ReadBuffer& buffer);

/// Reset the path (clear all waypoints)
void pf_handle_path_reset(const cogip::canpb::ReadBuffer& buffer);

/// Add a waypoint to the path
void pf_handle_path_add_point(cogip::canpb::ReadBuffer& buffer);

/// Start path execution
void pf_handle_path_start(const cogip::canpb::ReadBuffer& buffer);

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
