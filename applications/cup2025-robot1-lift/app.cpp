/* Project includes */
#include "app.hpp"
#include "app_conf.hpp"
#include "pf_positional_actuators.hpp"

namespace cogip {
namespace app {

void app_init(void)
{
    cogip::pf::actuators::positional_actuators::create_lift(cogip::actuators::Enum::MOTOR_LIFT,
                                                            actuators::lift_params);
}

} // namespace app
} // namespace cogip
