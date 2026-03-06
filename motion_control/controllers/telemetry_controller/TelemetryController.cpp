#include "telemetry_controller/TelemetryController.hpp"
#include "log.h"
#include "telemetry/Telemetry.hpp"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

void TelemetryController::execute(ControllersIO& io)
{
    DEBUG("Start TelemetryController\n");

    if (auto opt = io.get_as<float>(keys_.speed_order)) {
        telemetry::Telemetry::send<float>(cogip::utils::hash_key(keys_.speed_order), *opt);
    }
    if (auto opt = io.get_as<float>(keys_.tracker_velocity)) {
        telemetry::Telemetry::send<float>(cogip::utils::hash_key(keys_.tracker_velocity), *opt);
    }
    if (auto opt = io.get_as<float>(keys_.current_speed)) {
        telemetry::Telemetry::send<float>(cogip::utils::hash_key(keys_.current_speed), *opt);
    }
    if (auto opt = io.get_as<float>(keys_.speed_command)) {
        telemetry::Telemetry::send<float>(cogip::utils::hash_key(keys_.speed_command), *opt);
    }

    DEBUG("End TelemetryController\n");
}

} // namespace motion_control

} // namespace cogip
