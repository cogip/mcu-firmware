// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    motion_control_common
/// @{
/// @file
/// @brief      Simple meta-controller to execute controllers in chain
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// System includes
#include <iostream>

// Project includes
#include "ControllersIO.hpp"
#include "etl/deque.h"
#include "BaseMetaController.hpp"

namespace cogip {
namespace motion_control {

/// @brief Container of sub-controllers executed in sequence, sharing a single ControllersIO.
///
/// Each sub-controller’s execute(ControllersIO& io) is called in order, using the same
/// ControllersIO instance. No checks on input/output sizes are performed—it's up to the
/// sub-controllers (and the engine) to agree on which keys to read/write.
///
/// @tparam NB_CONTROLLERS Maximum number of controllers in the chain.
template <size_t NB_CONTROLLERS>
class MetaController : public BaseMetaController
{
public:
    /// @brief Run every controller in the chain, passing along the same ControllersIO.
    /// @param io Shared IO object containing inputs/outputs for all controllers.
    void execute(ControllersIO& io) override
    {
        if (controllers_.empty()) {
            std::cerr << "Error: no controller in MetaController.\n";
            return;
        }

        COGIP_DEBUG_COUT("Execute MetaController");

        for (auto ctrl : controllers_) {
            ctrl->execute(io);
        }
    }

    /// @brief Add a controller to the end of the chain.
    /// @param ctrl Pointer to a controller; its set_meta(this) must succeed.
    void add_controller(BaseController* ctrl) override
    {
        if (!ctrl->set_meta(this)) {
            return;
        }
        controllers_.push_back(ctrl);
        ++nb_controllers_;
    }

    /// @brief Add a controller to the front of the chain.
    /// @param ctrl Pointer to a controller; its set_meta(this) must succeed.
    void prepend_controller(BaseController* ctrl) override
    {
        if (!ctrl->set_meta(this)) {
            return;
        }
        controllers_.push_front(ctrl);
        ++nb_controllers_;
    }

    /// @brief Replace the controller at index with a new one.
    /// @param index    Position in the chain to replace.
    /// @param new_ctrl New controller pointer; its set_meta(this) must succeed.
    void replace_controller(
        uint32_t index,             ///< [in] Index of controller to replace
        BaseController* new_ctrl    ///< [in] Replacement controller
    ) override
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
};

} // namespace motion_control
} // namespace cogip
