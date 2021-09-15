// Copyright (C) 2018-2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     planners
/// @{
/// @file
/// @brief       Astar planner
/// @author      Gilles DOFFE <g.doffe@gmail.com>
/// @author      Yannick GICQUEL <yannick.gicquel@gmail.com>
/// @author      Stephen CLYMANS <sclymans@gmail.com>
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

// Project includes
#include "planners/Planner.hpp"

namespace cogip {

namespace planners {

/// Astar planner class.
class AstarPlanner : public Planner {
public:
    /// Constructor.
    AstarPlanner(ctrl_t *ctrl, path::Path &path) : Planner(ctrl, path) {};

    void *task_planner() override;

private:
    int trajectory_get_route_update(
        const cogip::cogip_defs::Pose &robot_pose   ///< current robot pose
        );
};

} // namespace planners

} // namespace cogip

/// @}
