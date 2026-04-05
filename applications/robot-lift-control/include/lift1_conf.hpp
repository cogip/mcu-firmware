#pragma once

#include "lift_common_conf.hpp"

namespace cogip {
namespace app {
namespace actuators {

/// @brief Lift 1 actuator ID (for CAN protobuf messages)
inline constexpr cogip::actuators::Enum LIFT_ACTUATOR_ID =
    cogip::actuators::Enum{0}; // MOTOR_LIFT_1

/// @brief Lift parameters for lift 1
static const auto lift_params = make_lift_params(LIFT_ACTUATOR_ID);

} // namespace actuators
} // namespace app
} // namespace cogip
