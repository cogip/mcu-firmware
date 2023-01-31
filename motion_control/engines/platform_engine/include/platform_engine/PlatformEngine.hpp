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
#include "cogip_defs/Pose.hpp"
#include "cogip_defs/Polar.hpp"
#include "etl/delegate.h"
#include "motion_control_common/BaseControllerEngine.hpp"

namespace cogip {

namespace motion_control {

/// Get current and target poses from platform
using platform_get_poses_cb_t = etl::delegate<void(cogip_defs::Pose&, cogip_defs::Pose&)>;

/// Get current and target speeds from platform
using platform_get_speeds_cb_t = etl::delegate<void(cogip_defs::Polar&, cogip_defs::Polar&)>;

/// Engine getting inputs from platform and setting outputs for the platform
class PlatformEngine : public BaseControllerEngine {
public:
    /// Constructor
    PlatformEngine(
        platform_get_poses_cb_t platform_get_poses_cb,      ///< [in]  Platform callback to get robot current and target poses
        platform_get_speeds_cb_t platform_get_speeds_cb     ///< [in]  Platform callback to get robot current and target speeds
    ) : BaseControllerEngine(),
            platform_get_poses_cb_(platform_get_poses_cb),
            platform_get_speeds_cb_(platform_get_speeds_cb) {};

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
    cogip_defs::Pose target_pose_;

    /// Platform to get target and current poses from platforms
    platform_get_poses_cb_t platform_get_poses_cb_;

    /// Platform to get target and current speeds from platforms
    platform_get_speeds_cb_t platform_get_speeds_cb_;
};

} // namespace motion_control

} // namespace cogip

/// @}
