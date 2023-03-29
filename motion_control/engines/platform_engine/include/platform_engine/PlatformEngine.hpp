// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    platform_engine Platform engine
/// @{
/// @file
/// @brief      Engine getting inputs from platform and setting outputs for the platform
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "path/Pose.hpp"
#include "cogip_defs/Polar.hpp"
#include "etl/delegate.h"
#include "motion_control_common/BaseControllerEngine.hpp"

namespace cogip {

namespace motion_control {

/// Get current speed and pose from platform
using platform_get_speed_and_pose_cb_t = etl::delegate<void(cogip_defs::Polar&, cogip_defs::Pose&)>;

/// Process motion control commands
using platform_process_commands_cb_t = etl::delegate<void(const cogip_defs::Polar&)>;

/// Engine getting inputs from platform and setting outputs for the platform
class PlatformEngine : public BaseControllerEngine {
public:
    /// Constructor
    PlatformEngine(
        platform_get_speed_and_pose_cb_t platform_get_speed_and_pose_cb,    ///< [in]  Platform callback to get robot current speed and pose
        platform_process_commands_cb_t platform_process_commands_cb         ///< [in]  Platform callback to process commands output from last controller
    ) : BaseControllerEngine(),
        platform_get_speed_and_pose_cb_(platform_get_speed_and_pose_cb),
        platform_process_commands_cb_(platform_process_commands_cb) {};

    /// Get current speed
    /// return     current speed
    const cogip_defs::Polar& current_speed() const { return current_speed_; };

    /// Get target speed
    /// return     target speed
    const cogip_defs::Polar& target_speed() const { return target_speed_; };

    /// Get current pose
    /// return     current pose
    const cogip_defs::Pose& current_pose() const { return current_pose_; };

    /// Get target pose
    /// return     target pose
    const path::Pose& target_pose() const { return target_pose_; };

    /// Set target speed
    void set_target_speed(
        const cogip_defs::Polar& target_speed   ///< [in]   new target speed
        ) { target_speed_ = target_speed; };

    /// Set current pose
    void set_current_pose(
        const cogip_defs::Pose& current_pose    ///< [in]   new current pose
        ) { current_pose_ = current_pose; };

    /// Set target pose
    void set_target_pose(
        const path::Pose& target_pose     ///< [in]   new target pose
        ) { target_pose_ = target_pose; };

private:
    /// Prepare controller inputs from platform functions.
    void prepare_inputs();

    /// Process controller output for platform restitution.
    void process_outputs();

    /// Robot polar current speed
    cogip_defs::Polar current_speed_;

    /// Robot polar target speed
    cogip_defs::Polar target_speed_;

    /// Robot current pose
    cogip_defs::Pose current_pose_;

    /// Robot target pose
    path::Pose target_pose_;

    /// Platform callback to get target and current poses from platforms
    platform_get_speed_and_pose_cb_t platform_get_speed_and_pose_cb_;

    /// Platform calback to drive motors according to commands
    platform_process_commands_cb_t platform_process_commands_cb_;
};

} // namespace motion_control

} // namespace cogip

/// @}
