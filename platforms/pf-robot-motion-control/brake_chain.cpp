// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief Brake chain wiring for the platform engine

#include "brake_chain.hpp"

#include "motion_control.hpp"

namespace cogip {
namespace pf {
namespace motion_control {

namespace brake_chain {

void init()
{
    // Speed loop: parallel linear + angular PIDs.
    brake_speed_loop_polar_parallel.add_controller(&brake_linear_speed_controller);
    brake_speed_loop_polar_parallel.add_controller(&brake_angular_speed_controller);

    // Chain: force speed orders to 0, then run the speed loop.
    brake_meta_controller.add_controller(&brake_zero_speed_order_controller);
    brake_meta_controller.add_controller(&brake_speed_loop_polar_parallel);

    // Registration with the platform engine is done by the motion_control
    // module, which owns the engine instance.
}

} // namespace brake_chain

} // namespace motion_control
} // namespace pf
} // namespace cogip
