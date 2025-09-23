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

// RIOT includes
#include "log.h"
#include <inttypes.h>

#include <debug.h>

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
            LOG_ERROR("Error: no controller in MetaController");
            return;
        }

        DEBUG("Execute MetaController of %" PRIu32 " controllers", static_cast<uint32_t>(controllers_.size()));

        size_t index = 0;
        for (auto* controller : controllers_) {
            if (!controller) {
                LOG_ERROR("controllers_[%" PRIu32 "] is nullptr!", static_cast<uint32_t>(index));
            }
            else {
                controller->execute(io);
            }
            index++;
        }
    }

    /// @brief Add a controller to the end of the chain.
    /// @param controller Controller to add
    /// @return 0 on success, negative code otherwise
    int add_controller(BaseController* controller) override
    {
        if (controller == nullptr) {
            return -EINVAL;
        }

        if (!controller->set_meta(this)) {
            return -EPERM;
        }

        if (controllers_.size() >= NB_CONTROLLERS) {
            controller->set_meta(nullptr);
            return -ENOMEM;
        }

        controllers_.push_back(controller);

        return 0;
    }

    /// @brief Add a controller to the front of the chain.
    /// @param controller Controller to add
    /// @return 0 on success, negative code otherwise
    int prepend_controller(BaseController* controller) override
    {
        if (controller == nullptr) {
            return -EINVAL;
        }

        if (!controller->set_meta(this)) {
            return -EPERM;
        }

        if (controllers_.size() >= NB_CONTROLLERS) {
            controller->set_meta(nullptr);
            return -ENOMEM;
        }

        controllers_.push_front(controller);

        return 0;
    }

    /// @brief Replace the controller at index with a new one.
    /// @param index    Position in the chain to replace.
    /// @param controller New Controller
    /// @return 0 on success, negative code otherwise
    int replace_controller(
        uint32_t index,             ///< [in] Index of controller to replace
        BaseController* new_controller    ///< [in] Replacement controller
    ) override
    {
        if (index >= controllers_.size()) {
            return -ERANGE;
        }

        BaseController* old_controller = controllers_[index];
        if (old_controller == new_controller) {
            return 0;
        }

        if (!new_controller->set_meta(this)) {
            return -EPERM;
        }

        if (!old_controller->set_meta(nullptr)) {
            new_controller->set_meta(nullptr);
            return -EPERM;
        }

        controllers_[index] = new_controller;

        return 0;
    }

protected:
    etl::deque<BaseController*, NB_CONTROLLERS> controllers_;
};

} // namespace motion_control
} // namespace cogip
