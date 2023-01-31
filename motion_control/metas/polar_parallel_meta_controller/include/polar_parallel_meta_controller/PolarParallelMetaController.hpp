// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    polar_parallel_meta_controller
/// @{
/// @file
/// @brief      Run 2 (meta-)controllers, one for linear control, one for angular control
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "motion_control_common/ParallelMetaController.hpp"

namespace cogip {

namespace motion_control {

/// Number of controllers to be ran in parallel.
inline constexpr size_t polar_parallel_meta_controller_nb_controllers = 2;

/// Run 2 (meta-)controllers, one for linear control, one for angular control
/// Input 0:    linear pose error
/// Input 1:    linear current speed
/// Input 2:    linear target speed
/// Input 3:    angular pose error
/// Input 4:    angular current speed1
/// Input 5:    angular target speed
/// Output 0:   linear motor command
/// Output 0:   angular motor command
class PolarParallelMetaController : public ParallelMetaController<6, 2, polar_parallel_meta_controller_nb_controllers> {
protected:

    /// Set inputs for each parallel controller
    void set_inputs() override {
        auto ctrl_iterator = this->begin();

        BaseController *linear_ctrl = *ctrl_iterator++;
        BaseController *angular_ctrl = *ctrl_iterator++;

        // Linear pose error
        linear_ctrl->set_input(0, this->inputs_[0]);
        // Linear current speed
        linear_ctrl->set_input(1, this->inputs_[1]);
        // Linear target speed
        linear_ctrl->set_input(2, this->inputs_[2]);

        // Angular pose error
        angular_ctrl->set_input(0, this->inputs_[3]);
        // Angular current speed
        angular_ctrl->set_input(1, this->inputs_[4]);
        // Angular target speed
        angular_ctrl->set_input(2, this->inputs_[5]);
    };

    /// Sort outputs from each parallel controller
    void sort_outputs() override {
        auto ctrl_iterator = this->begin();

        BaseController *linear_ctrl = *ctrl_iterator++;
        BaseController *angular_ctrl = *ctrl_iterator++;

        // Linear command
        this->outputs_[0] = linear_ctrl->output(0);
        // Angular command
        this->outputs_[1] = angular_ctrl->output(0);
    };
};

} // namespace motion_control

} // namespace cogip

/// @}
