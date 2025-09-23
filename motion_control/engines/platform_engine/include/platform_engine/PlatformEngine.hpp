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
/// @author     Mathis LECRIVAIN <lecrivain.mathis@gmail.com>

#pragma once

// Project includes
#include "path/Pose.hpp"
#include "cogip_defs/Polar.hpp"
#include "etl/delegate.h"
#include "localization/LocalizationInterface.hpp"
#include "drive_controller/DriveControllerInterface.hpp"
#include "motion_control_common/BaseControllerEngine.hpp"

namespace cogip {

namespace motion_control {

/// Pose reached state callback
using pose_reached_cb_t = etl::delegate<void(const target_pose_status_t)>;

/// Engine getting inputs from platform and setting outputs for the platform
class PlatformEngine : public BaseControllerEngine {
public:
    /// Constructor
    PlatformEngine(
        localization::LocalizationInterface &localization,          ///< [in] Robot localization reference
        drive_controller::DriveControllerInterface &drive_contoller,///< [in] Robot drive controller
        pose_reached_cb_t pose_reached_cb,                          ///< [in] Callback to send pose reached state from last controller
        uint32_t engine_thread_period_ms                            ///< [in] Motion control enginethread period
    );

    /// Get current speed
    /// return     current speed
    const cogip_defs::Polar& current_speed() const { return localization_.delta_polar_pose(); };

    /// Get target speed
    /// return     target speed
    const cogip_defs::Polar& target_speed() const { return target_speed_; };

    /// Get current pose
    /// return     current pose
    const cogip_defs::Pose& current_pose() const { return localization_.pose(); };

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
        ) { localization_.set_pose(current_pose); };

    /// Set target pose
    void set_target_pose(
        const path::Pose& target_pose     ///< [in]   new target pose
        ) { target_pose_ = target_pose; };

private:
    /// Prepare controller inputs from platform functions.
    void prepare_inputs();

    /// Process controller output for platform restitution.
    void process_outputs();

    /// Robot polar target speed
    cogip_defs::Polar target_speed_;

    /// Robot target pose
    path::Pose target_pose_;

    /// Robot localization
    localization::LocalizationInterface & localization_;

    /// Robot drive controller
    drive_controller::DriveControllerInterface &drive_contoller_;

    /// Pose reached callback
    pose_reached_cb_t pose_reached_cb_;

};

} // namespace motion_control

} // namespace cogip

/// @}
