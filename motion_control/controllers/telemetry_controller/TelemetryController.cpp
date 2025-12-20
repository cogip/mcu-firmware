#include "telemetry_controller/TelemetryController.hpp"
#include "log.h"
#include "telemetry/Telemetry.hpp"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

using cogip::utils::operator"" _key_hash;

void TelemetryController::execute(ControllersIO& io)
{
    DEBUG("Start TelemetryController\n");

    // Linear telemetry
    if (auto opt = io.get_as<float>("linear_speed_order")) {
        telemetry::Telemetry::send<int64_t>("linear_speed_order"_key_hash,
                                            static_cast<int64_t>(*opt));
    }
    if (auto opt = io.get_as<float>("linear_feedforward_velocity")) {
        telemetry::Telemetry::send<int64_t>("linear_feedforward_velocity"_key_hash,
                                            static_cast<int64_t>(*opt));
    }
    if (auto opt = io.get_as<float>("linear_current_speed")) {
        telemetry::Telemetry::send<int64_t>("linear_current_speed"_key_hash,
                                            static_cast<int64_t>(*opt));
    }

    // Angular telemetry
    if (auto opt = io.get_as<float>("angular_speed_order")) {
        telemetry::Telemetry::send<int64_t>("angular_speed_order"_key_hash,
                                            static_cast<int64_t>(*opt));
    }
    if (auto opt = io.get_as<float>("angular_feedforward_velocity")) {
        telemetry::Telemetry::send<int64_t>("angular_feedforward_velocity"_key_hash,
                                            static_cast<int64_t>(*opt));
    }
    if (auto opt = io.get_as<float>("angular_current_speed")) {
        telemetry::Telemetry::send<int64_t>("angular_current_speed"_key_hash,
                                            static_cast<int64_t>(*opt));
    }

    DEBUG("End TelemetryController\n");
}

} // namespace motion_control

} // namespace cogip
