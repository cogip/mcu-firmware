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
    private etl::list<BaseController *, NB_CONTROLLERS> {
public:
    /// Controller core method. Meta controller executes all sub-controllers in chain.
    void execute() override {
        if (this->empty()) {
            std::cerr << "Error: no controller added." << std::endl;
            return;
        }
        if (this->back()->nb_outputs() != OUTPUT_SIZE) {
            std::cerr << "Error: First controller must have the same number of inputs (" << this->back()->nb_inputs() << ") "
                      << "as the last controller (" << this->back()->nb_inputs() << ")." << std::endl;
            return;
        }

        std::cout << "Execute MetaController" << std::endl;

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

    /// Add a controller to the controllers chain.
    void add_controller(
        BaseController *ctrl    ///< [in]  new controller to add
        ) override {
        if (! ctrl->set_meta(this)) {
            return;
        }
        if (this->empty() && INPUT_SIZE != ctrl->nb_inputs()) {
            std::cout << "Error: First controller must have the same number of inputs (" << ctrl->nb_inputs() << ") "
                         "as the meta controller (" << INPUT_SIZE << ")." << std::endl;
            return;
        }
        this->push_back(ctrl);

        nb_controllers_++;
    };
};

} // namespace motion_control

} // namespace cogip

/// @}
