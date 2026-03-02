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

    const float period_to_sec = 1000.0f / parameters_.loop_period_ms;

    if (auto opt = io.get_as<float>(keys_.speed_order)) {
        telemetry::Telemetry::send<float>(cogip::utils::hash_key(keys_.speed_order),
                                          *opt * period_to_sec);
    }
    if (auto opt = io.get_as<float>(keys_.tracker_velocity)) {
        telemetry::Telemetry::send<float>(cogip::utils::hash_key(keys_.tracker_velocity),
                                          *opt * period_to_sec);
    }
    if (auto opt = io.get_as<float>(keys_.current_speed)) {
        telemetry::Telemetry::send<float>(cogip::utils::hash_key(keys_.current_speed),
                                          *opt * period_to_sec);
    }
    if (auto opt = io.get_as<float>(keys_.pose_error)) {
        telemetry::Telemetry::send<float>(cogip::utils::hash_key(keys_.pose_error), *opt);
    }

    DEBUG("End TelemetryController\n");
}

} // namespace motion_control

} // namespace cogip
