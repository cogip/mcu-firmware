// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    speed_pid_controller Speed PID controller
/// @{
/// @file
/// @brief      Speed PID controller
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

// Project includes
#include "motion_control_common/Controller.hpp"
#include "motion_control_common/ControllersIO.hpp"
#include "SpeedPIDControllerParameters.hpp"
#include "SpeedPIDControllerIOKeys.hpp"

namespace cogip {

namespace motion_control {

/// @brief PID speed controller.
///        Reads speed error and computes a speed command via PID.
class SpeedPIDController
    : public Controller<SpeedPIDControllerIOKeys, SpeedPIDControllerParameters>
{
public:
    /// @brief Constructor.
    /// @param keys       Reference to IO key names.
    /// @param parameters Reference to PID parameters.
    explicit SpeedPIDController(
        const SpeedPIDControllerIOKeys&         keys,
        const SpeedPIDControllerParameters&     parameters
    )
        : Controller<SpeedPIDControllerIOKeys, SpeedPIDControllerParameters>(keys, parameters)
    {
    }

    /// @brief Read speed error, compute speed command.
    void execute(ControllersIO& io) override;
};

}  // namespace motion_control

}  // namespace cogip

/// @}
