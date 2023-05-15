// RIOT includes
#include <ztimer.h>

// Firmware includes
#include "app_actuators_shell.hpp"
#include "pf_positional_actuators.hpp"

#include "shell_menu/shell_menu.hpp"

namespace cogip {
namespace app {
namespace actuators {

static void _wait_timeout(uint32_t timeout) {
    ztimer_sleep(ZTIMER_MSEC, timeout);
}

// CENTRAL LIFT
int _cmd_central_lift_test(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    pf::actuators::positional_actuators::get(pf::actuators::positional_actuators::Enum::MOTOR_CENTRAL_LIFT).actuate_timeout(-100, 15);
    _wait_timeout(3000);
    pf::actuators::positional_actuators::get(pf::actuators::positional_actuators::Enum::MOTOR_CENTRAL_LIFT).actuate_timeout(100, 2);
    _wait_timeout(3000);
    pf::actuators::positional_actuators::get(pf::actuators::positional_actuators::Enum::MOTOR_CENTRAL_LIFT).actuate_timeout(100, 15);

    return EXIT_SUCCESS;
}

static void _shell_enter_callback(void)
{
}

static cogip::shell::Menu actuators_menu = { "Actuators actions", "actuators", &cogip::shell::root_menu(), _shell_enter_callback };
static cogip::shell::Command cmd_0 = { "lift", "Test central lift", _cmd_central_lift_test };

void shell_init()
{
    actuators_menu.push_back(&cmd_0);
}

} // namespace actuators
} // namespace app
} // namespace cogip
