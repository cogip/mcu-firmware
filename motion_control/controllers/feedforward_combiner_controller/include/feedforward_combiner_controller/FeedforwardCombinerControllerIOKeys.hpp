// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    feedforward_combiner_controller Feedforward Combiner controller IO keys
/// @{
/// @file
/// @brief      Feedforward Combiner controller IO keys
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for FeedforwardCombinerController.
///
/// This controller simply adds feedforward velocity and feedback correction.
/// Optional state gating: if current_state is set, the controller only writes
/// output when current_state matches active_state. Otherwise it returns early
/// without writing (preserving any previous value in speed_order).
struct FeedforwardCombinerControllerIOKeys
{
    etl::string_view feedforward_velocity; ///< e.g. "linear_feedforward_velocity"
    etl::string_view feedback_correction;  ///< e.g. "linear_feedback_correction"
    etl::string_view speed_order;          ///< e.g. "linear_speed_order" (output)
    etl::string_view current_state;        ///< e.g. "pose_straight_filter_state" (optional gating)
    int active_state = -1; ///< State value when controller should be active (-1 = always active)
};

} // namespace motion_control

} // namespace cogip

/// @}
