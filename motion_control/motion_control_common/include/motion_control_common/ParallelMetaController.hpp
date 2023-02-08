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
    protected etl::list<BaseController *, NB_CONTROLLERS> {
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

    protected:
        /// Set inputs for all parallel controllers.
        virtual void set_inputs() = 0;

        /// Sort outputs from all parallel controllers.
        virtual void sort_outputs() = 0;
};

} // namespace motion_control

} // namespace cogip

/// @}
