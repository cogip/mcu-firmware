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
#include "etl/list.h"
#include "BaseMetaController.hpp"
#include "MetaControllerParameters.hpp"
#include "Controller.hpp"

namespace cogip {

namespace motion_control {

/// Parallel meta controllers are controllers containers to execute them in parallel.
/// @tparam INPUT_SIZE      number of inputs
/// @tparam OUTPUT_SIZE     number of outputs
/// @tparam NB_CONTROLLERS  maximum number of controllers
template <
    size_t INPUT_SIZE,
    size_t OUTPUT_SIZE,
    size_t NB_CONTROLLERS
>
class ParallelMetaController :
    public BaseMetaController,
    public Controller<INPUT_SIZE, OUTPUT_SIZE, MetaControllerParameters>,
    protected etl::vector<BaseController *, NB_CONTROLLERS> {
public:
    /// Controller core method. Meta controller executes all sub-controllers in parallel.
    void execute() override {
        if (this->empty()) {
            COGIP_DEBUG_COUT("Error: no controller added.");
            return;
        }

        COGIP_DEBUG_COUT("Execute ParallelMetaController");

        this->set_inputs();

        // Execute each controller.
        for (auto ctrl: *this) {
            // Execute controller.
            ctrl->execute();
        }

        this->sort_outputs();
    };

    /// Add a controller to the controllers list.
    void add_controller(
        BaseController *ctrl    ///< [in]  new controller to add
        ) override {
        if (! ctrl->set_meta(this)) {
            return;
        }
        this->push_back(ctrl);

        nb_controllers_++;
    };

    /// Add a controller to the beginning of the controllers chain.
    void insert_controller(
        BaseController *ctrl    ///< [in]  new controller to add
        ) override {
        if (! ctrl->set_meta(this)) {
            return;
        }
        this->insert(this->end(), ctrl);

        nb_controllers_++;
    };

    /// Replace a controller at the given position in the controllers chain.
    void replace_controller(
        uint32_t index,         ///< [in]  index
        BaseController *ctrl    ///< [in]  position of the controller to replace
        ) override {
        if (! ctrl->set_meta(this)) {
            return;
        }
        if (index >= nb_controllers_) {
            return;
        }

        auto ctrl_iterator = this->end();

        for (uint32_t i = 0; i < index; i++) {
            ctrl_iterator++;
        }

        BaseController *ctrl_to_replace = *ctrl_iterator;

        if ((ctrl_to_replace->nb_inputs() != ctrl->nb_inputs()) ||
            (ctrl_to_replace->nb_outputs() != ctrl->nb_outputs())) {
            return;
        }

        ctrl_iterator = this->erase(ctrl_iterator);
        this->insert(--ctrl_iterator, ctrl);
    };

    protected:
        /// Set inputs for all parallel controllers.
        virtual void set_inputs() = 0;

        /// Sort outputs from all parallel controllers.
        virtual void sort_outputs() = 0;
};

} // namespace motion_control

} // namespace cogip

/// @}
