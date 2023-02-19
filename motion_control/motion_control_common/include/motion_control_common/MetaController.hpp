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

// Project includes
#include "etl/list.h"
#include "BaseMetaController.hpp"
#include "Controller.hpp"
#include "MetaControllerParameters.hpp"

namespace cogip {

namespace motion_control {

/// Meta controllers are controllers containers to execute them in chain.
/// @tparam INPUT_SIZE      number of inputs
/// @tparam OUTPUT_SIZE     number of outputs
/// @tparam NB_CONTROLLERS  maximum number of controllers
template <
    size_t INPUT_SIZE,
    size_t OUTPUT_SIZE,
    size_t NB_CONTROLLERS
>
class MetaController :
    public BaseMetaController,
    public Controller<INPUT_SIZE, OUTPUT_SIZE, MetaControllerParameters>,
    private etl::vector<BaseController *, NB_CONTROLLERS> {
public:
    /// Controller core method. Meta controller executes all sub-controllers in chain.
    void execute() override {
        if (this->empty()) {
            COGIP_DEBUG_CERR("Error: no controller added.");
            return;
        }
        if (this->back()->nb_outputs() != OUTPUT_SIZE) {
            COGIP_DEBUG_CERR("Error: First controller must have the same number of outputs (" << this->back()->nb_inputs() << ") "
                      << "as the last controller (" << this->back()->nb_inputs() << ").");
            return;
        }

        COGIP_DEBUG_COUT("Execute MetaController");

        // Execute each controller in chain.
        BaseController *previous = nullptr;
        for (auto ctrl: *this) {
            // Set inputs.
            for (size_t i = 0; i < ctrl->nb_inputs(); i++) {
                if (previous) {
                    ctrl->set_input(i, previous->output(i));
                }
                else {
                    ctrl->set_input(i, this->input(i));
                }
            }

            // Execute controller.
            ctrl->execute();

            previous = ctrl;
        }

        // Set outputs.
        for (size_t i = 0; i < this->back()->nb_outputs(); i++) {
            this->set_output(i, this->back()->output(i));
        }
    };

    /// Add a controller to the end of the controllers chain.
    void add_controller(
        BaseController *ctrl    ///< [in]  new controller to add
        ) override {
        if (! ctrl->set_meta(this)) {
            return;
        }
        if (this->empty() && INPUT_SIZE != ctrl->nb_inputs()) {
            COGIP_DEBUG_CERR("Error: First controller must have the same number of inputs (" << ctrl->nb_inputs() << ") as the meta controller (" << INPUT_SIZE << ").");
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
        if (INPUT_SIZE != ctrl->nb_inputs()) {
            COGIP_DEBUG_CERR("Error: First controller must have the same number of inputs (" << ctrl->nb_inputs() << ") as the meta controller (" << INPUT_SIZE << ").");
            return;
        }
        this->insert(this->end(), ctrl);

        nb_controllers_++;
    };

    /// Replace a controller at the given position in the controllers chain.
    void replace_controller(
        uint32_t index,         ///< [in]  index
        BaseController *new_ctrl    ///< [in]  position of the controller to replace
        ) override {
        if (index >= nb_controllers_) {
            return;
        }

        BaseController *ctrl_to_replace = (*this)[index];

        if (ctrl_to_replace == new_ctrl) {
            return;
        }

        if ((ctrl_to_replace->nb_inputs() != new_ctrl->nb_inputs())
            || (ctrl_to_replace->nb_outputs() != new_ctrl->nb_outputs())
            || (! new_ctrl->set_meta(this))
            || (! ctrl_to_replace->set_meta(nullptr))
            ) {
            return;
        }

        (*this)[index] = new_ctrl;
    };
};

} // namespace motion_control

} // namespace cogip

/// @}
