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
/// Input 6:    pose reached
/// Output 0:   linear motor command
/// Output 1:   angular motor command
class PolarParallelMetaController : public ParallelMetaController<7, 3, polar_parallel_meta_controller_nb_controllers> {
protected:

    /// Set inputs for each parallel controller
    void set_inputs() override {
        BaseController *linear_ctrl = this->front();
        BaseController *angular_ctrl = this->back();

        // Linear pose error
        linear_ctrl->set_input(0, inputs_[0]);
        // Linear current speed
        linear_ctrl->set_input(1, inputs_[1]);
        // Linear target speed
        linear_ctrl->set_input(2, inputs_[2]);

        // Angular pose error
        angular_ctrl->set_input(0, inputs_[3]);
        // Angular current speed
        angular_ctrl->set_input(1, inputs_[4]);
        // Angular target speed
        angular_ctrl->set_input(2, inputs_[5]);
    };

    /// Sort outputs from each parallel controller
    void sort_outputs() override {
        BaseController *linear_ctrl = this->front();
        BaseController *angular_ctrl = this->back();

        // Linear command
        outputs_[0] = linear_ctrl->output(0);
        // Angular command
        outputs_[1] = angular_ctrl->output(0);
        // Pose reached has been set by previous filter
        outputs_[2] = inputs_[6];
    };
};

} // namespace motion_control

} // namespace cogip

/// @}
