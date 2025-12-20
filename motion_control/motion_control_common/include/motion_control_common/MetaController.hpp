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
#include "BaseMetaController.hpp"
#include "ControllersIO.hpp"
#include "etl/deque.h"
#include "etl/string.h"

namespace cogip {
namespace motion_control {

/// @brief Container of sub-controllers executed in sequence, sharing a single
/// ControllersIO.
///
/// Each sub-controller’s execute(ControllersIO& io) is called in order, using
/// the same ControllersIO instance. No checks on input/output sizes are
/// performed—it's up to the sub-controllers (and the engine) to agree on which
/// keys to read/write.
///
/// @tparam NB_CONTROLLERS Maximum number of controllers in the chain.
template <size_t NB_CONTROLLERS> class MetaController : public BaseMetaController
{
  public:
    /// Constructor
    /// @param name Optional instance name for identification
    explicit MetaController(etl::string_view name = "") : BaseMetaController(name) {}

    /// @brief Get the type name of this controller
    /// @return Type name string
    const char* type_name() const override
    {
        return "MetaController";
    }

    /// @brief Dump the controller hierarchy to stdout as ASCII tree
    /// @param indent Current indentation level
    /// @param is_last Whether this is the last child at current level
    /// @param prefix Prefix string for tree drawing
    void dump(int indent = 0, bool is_last = true, const char* prefix = "") const override
    {
        // Print tree branch for this meta controller
        if (indent > 0) {
            printf("%s%s", prefix, is_last ? "└── " : "├── ");
        }

        // Print type and name
        if (name_.empty()) {
            printf("%s\n", type_name());
        } else {
            printf("%s: %.*s\n", type_name(), static_cast<int>(name_.size()), name_.data());
        }

        // Build new prefix for children
        etl::string<128> new_prefix;
        new_prefix.append(prefix);
        new_prefix.append(is_last ? "    " : "│   ");

        // Dump children
        size_t count = controllers_.size();
        size_t index = 0;
        for (const auto* controller : controllers_) {
            if (controller) {
                bool child_is_last = (index == count - 1);
                controller->dump(indent + 1, child_is_last, new_prefix.c_str());
            }
            index++;
        }
    }

    /// @brief Run every controller in the chain, passing along the same
    /// ControllersIO.
    /// @param io Shared IO object containing inputs/outputs for all controllers.
    void execute(ControllersIO& io) override
    {
        if (controllers_.empty()) {
            LOG_ERROR("Error: no controller in MetaController\n");
            return;
        }

        DEBUG("Execute MetaController of %" PRIu32 " controllers\n",
              static_cast<uint32_t>(controllers_.size()));

        size_t index = 0;
        for (auto* controller : controllers_) {
            if (!controller) {
                LOG_ERROR("controllers_[%" PRIu32 "] is nullptr!\n", static_cast<uint32_t>(index));
            } else {
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
    int replace_controller(uint32_t index,                ///< [in] Index of controller to replace
                           BaseController* new_controller ///< [in] Replacement controller
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
