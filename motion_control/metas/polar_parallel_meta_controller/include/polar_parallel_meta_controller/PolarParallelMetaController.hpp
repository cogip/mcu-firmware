// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    polar_parallel_meta_controller
/// @{
/// @file
/// @brief      Run 2 (meta-)controllers, one for linear control, one for
/// angular control
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "motion_control_common/ParallelMetaController.hpp"

namespace cogip {

namespace motion_control {

/// @brief Runs two dual PID controllers in parallel.
///
/// This class executes two dual PID controllers (position and speed)
/// pseudo simultaneously for controlling both linear and angular motion.
class PolarParallelMetaController : public ParallelMetaController<2>
{
  public:
    /// Constructor
    /// @param name Optional instance name for identification
    explicit PolarParallelMetaController(etl::string_view name = "")
        : ParallelMetaController<2>(name)
    {
    }

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "PolarParallelMetaController";
    }
};

} // namespace motion_control

} // namespace cogip

/// @}
