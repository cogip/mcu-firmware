// Copyright (C) 2018-2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     planners
/// @{
/// @file
/// @brief       Virtual class defining common API for all planner implementations.
/// @author      Gilles DOFFE <g.doffe@gmail.com>
/// @author      Yannick GICQUEL <yannick.gicquel@gmail.com>
/// @author      Stephen CLYMANS <sclymans@gmail.com>
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

// Project includes
#include "ctrl.hpp"
#include "path/Path.hpp"

namespace cogip {

namespace planners {

/// Virtual Planner class.
class Planner {
public:
    /// Constructor.
    Planner(
        ctrl_t *ctrl,                               ///< [in] controller object
        path::Path &path                            ///< [in] application path
        );

    /// Start the trajectory planification and the associated controller.
    void start();

    /// Stop the trajectory planification and the associated controller.
    void stop();

    /// Periodic task function to process a planner.
    virtual void *task_planner() = 0;

    /// Start planner thread
    void start_thread();

    /// Allow or not the planner to change automatically the next path pose
    /// to reach once current pose is reached
    void set_allow_change_path_pose(
        bool allow                                  ///< [in] true to allow planner to change path pose,
                                                    ///       false otherwise
        ) { allow_change_path_pose_ = allow; };

protected:
    ctrl_t *ctrl_;                                  ///< controller object
    path::Path &path_;                              ///< applcation path
    bool started_;                                  ///< planner tarted
    bool allow_change_path_pose_;                   ///< Planner can automatically change next path pose
                                                    ///  to reach when current pose is reached
};

} // namespace planners

} // namespace cogip

/// @}
