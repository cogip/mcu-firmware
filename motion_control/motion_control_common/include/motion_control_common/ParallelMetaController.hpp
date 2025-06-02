// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    motion_control_common
/// @{
/// @file
/// @brief      Meta-controller to execute some controllers in pseudo parallel
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "ControllersIO.hpp"
#include "etl/deque.h"
#include "BaseMetaController.hpp"
#include <iostream>

namespace cogip {
namespace motion_control {

/// @brief Runs multiple sub-controllers “in parallel” using the same ControllersIO.
/// @details If any ParamKey gets written by ≥2 top‐level controllers, prints a warning.
/// @tparam NB_CONTROLLERS Maximum number of sub-controllers.
template <size_t NB_CONTROLLERS>
class ParallelMetaController : public BaseMetaController
{
public:
    /// @brief Execute all sub-controllers on the same `io`.
    ///        Detects and warns if the same key is modified by ≥2 controllers.
    /// @param io Shared ControllersIO instance.
    void execute(ControllersIO& io) override
    {
        if (controllers_.empty()) {
            COGIP_DEBUG_COUT("Error: no controller added.");
            return;
        }

        COGIP_DEBUG_COUT("Execute ParallelMetaController");

        // Cumulative set of keys already written by previous controllers.
        etl::set<ParamKey, MAX_PARAMS> cumulative_written;

        // For each top-level controller:
        for (auto ctrl : controllers_) {
            auto before_io = io.snapshot_modified();

            // Execute controller
            ctrl->execute(io);

            auto after_io = io.snapshot_modified();

            // Compute difference after/before on inputs/outputs via static helper
            auto just_written = ControllersIO::difference(after_io, before_io);

            // Detect collisions, parallel controllers should not modify the same entry
            // in the inputs/outputs map
            auto collisions = ControllersIO::find_collisions(just_written, cumulative_written);
            for (auto h : collisions) {
                std::cerr << "Warning: key hash " << h
                          << " was already written by another parallel controller.\n";
            }

            // Merge
            for (auto h_new : just_written) {
                cumulative_written.insert(h_new);
            }
        }
    }

    /// @brief Add a controller to the end of the chain.
    void add_controller(BaseController* ctrl) override
    {
        if (!ctrl->set_meta(this)) {
            return;
        }
        controllers_.push_back(ctrl);
        ++nb_controllers_;
    }

    /// @brief Add a controller to the front of the chain.
    void prepend_controller(BaseController* ctrl) override
    {
        if (!ctrl->set_meta(this)) {
            return;
        }
        controllers_.push_front(ctrl);
        ++nb_controllers_;
    }

    /// @brief Replace the controller at index.
    void replace_controller(uint32_t index, BaseController* new_ctrl) override
    {
        if (index >= nb_controllers_) {
            return;
        }

        BaseController* old_ctrl = controllers_[index];

        if (old_ctrl == new_ctrl) {
            return;
        }

        if (!new_ctrl->set_meta(this) || !old_ctrl->set_meta(nullptr)) {
            return;
        }

        controllers_[index] = new_ctrl;
    }

private:
    etl::deque<BaseController*, NB_CONTROLLERS> controllers_;
    size_t nb_controllers_ = 0;
};

} // namespace motion_control

} // namespace cogip

/// @}
