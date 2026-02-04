// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    path_manager_filter
/// @{
/// @file
/// @brief      Path manager filter implementation
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#include "path_manager_filter/PathManagerFilter.hpp"
#include "motion_control_common/BaseMetaController.hpp"
#include "path/Path.hpp"

#define ENABLE_DEBUG 0
#include <debug.h>
#include <log.h>

namespace cogip {

namespace motion_control {

void PathManagerFilter::execute(ControllersIO& io)
{
    DEBUG("PathManagerFilter::execute\n");

    // Get path singleton
    path::Path& path = path::Path::instance();

    // If not started or path is empty, do nothing
    if (!path.is_started() || path.empty()) {
        DEBUG("PathManagerFilter: not started or empty path\n");
        return;
    }

    // Get current target pose
    const path::Pose* current = path.current_pose();
    if (!current) {
        DEBUG("PathManagerFilter: no current pose available\n");
        return;
    }

    // Check pose_reached from PoseStraightFilter
    bool pose_reached = false;
    if (!keys_.pose_reached.empty()) {
        if (auto opt = io.get_as<target_pose_status_t>(keys_.pose_reached)) {
            pose_reached = (*opt == target_pose_status_t::reached);
        }
    }

    DEBUG("PathManagerFilter: waypoint %u/%u, is_intermediate=%d, pose_reached=%d\n",
          static_cast<unsigned>(path.current_index() + 1), static_cast<unsigned>(path.size()),
          current->is_intermediate(), pose_reached);

    // If current pose was reached
    if (pose_reached) {
        DEBUG("PathManagerFilter: pose reached! is_intermediate=%d\n", current->is_intermediate());
        if (current->is_intermediate() && path.advance()) {
            // Intermediate waypoint reached and advanced to next
            DEBUG("PathManagerFilter: intermediate pose reached, advancing\n");

            // Reset all downstream controllers (state machines, profiles, etc.)
            if (meta_) {
                meta_->reset();
            } else {
                LOG_WARNING("PathManagerFilter: meta_ is null, cannot reset!\n");
            }

            // Signal new target for profile recomputation
            if (!keys_.new_target.empty()) {
                io.set(keys_.new_target, true);
            }

            // Update current pose pointer after advance
            current = path.current_pose();
            if (!current) {
                DEBUG("PathManagerFilter: no current pose after advance\n");
                return;
            }
        } else {
            // Final waypoint reached or no more waypoints - path complete
            if (!keys_.path_complete.empty()) {
                io.set(keys_.path_complete, true);
            }
            path.stop();
            DEBUG("PathManagerFilter: path complete\n");
            return;
        }
    }

    // Emit current target pose
    if (!keys_.target_pose_x.empty()) {
        io.set(keys_.target_pose_x, current->x());
    }
    if (!keys_.target_pose_y.empty()) {
        io.set(keys_.target_pose_y, current->y());
    }
    if (!keys_.target_pose_O.empty()) {
        io.set(keys_.target_pose_O, current->O());
    }

    // Emit bypass_final_orientation from current waypoint
    if (!keys_.bypass_final_orientation.empty()) {
        io.set(keys_.bypass_final_orientation, current->bypass_final_orientation());
    }

    // Emit motion_direction from current waypoint
    if (!keys_.motion_direction.empty()) {
        io.set(keys_.motion_direction, static_cast<int>(current->get_motion_direction()));
    }

    // Emit is_intermediate from current waypoint
    if (!keys_.is_intermediate.empty()) {
        io.set(keys_.is_intermediate, current->is_intermediate());
    }

    // Emit current index for debug
    if (!keys_.path_index.empty()) {
        io.set(keys_.path_index, static_cast<float>(path.current_index()));
    }

    DEBUG("PathManagerFilter: emitting waypoint %u/%u (x=%.1f, y=%.1f, O=%.1f, intermediate=%d)\n",
          static_cast<unsigned>(path.current_index() + 1), static_cast<unsigned>(path.size()),
          current->x(), current->y(), current->O(), current->is_intermediate());
}

} // namespace motion_control

} // namespace cogip

/// @}
