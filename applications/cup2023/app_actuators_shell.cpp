// RIOT includes
#include <ztimer.h>

// Firmware includes
#include "app_actuators_shell.hpp"
#include "pf_positional_actuators.hpp"
#include "pf_pumps.hpp"

#include "shell_menu/shell_menu.hpp"

namespace cogip {
namespace app {
namespace actuators {

constexpr int off = 0;
constexpr int on = 1;

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

// PUMPS
int _cmd_arm_pump_right_test(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    pf::actuators::positional_actuators::get(pf::actuators::positional_actuators::Enum::LXMOTOR_RIGHT_ARM_LIFT).actuate_timeout(100, 50);
    _wait_timeout(5000);
    pf::actuators::pumps::get(pf::actuators::pumps::Enum::RIGHT_ARM_PUMP).activate();
    pf::actuators::positional_actuators::get(pf::actuators::positional_actuators::Enum::LXMOTOR_RIGHT_ARM_LIFT).actuate_timeout(-100, 55);
    _wait_timeout(8000);
    pf::actuators::pumps::get(pf::actuators::pumps::Enum::RIGHT_ARM_PUMP).deactivate();

    return EXIT_SUCCESS;
}

// ESC ARM
int _cmd_esc_arm_open_test(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    pf::actuators::positional_actuators::get(pf::actuators::positional_actuators::Enum::ANALOGSERVO_CHERRY_ARM).actuate(on);

    return EXIT_SUCCESS;
}

int _cmd_esc_arm_close_test(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    pf::actuators::positional_actuators::get(pf::actuators::positional_actuators::Enum::ANALOGSERVO_CHERRY_ARM).actuate(off);

    return EXIT_SUCCESS;
}

// ESC
int _cmd_esc_test(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    constexpr int low = 1;
    constexpr int middle = 2;
    constexpr int high = 3;
    constexpr int max = 4;

    pf::actuators::positional_actuators::get(pf::actuators::positional_actuators::Enum::ANALOGSERVO_CHERRY_ESC).actuate(off);
    _wait_timeout(100);
    pf::actuators::positional_actuators::get(pf::actuators::positional_actuators::Enum::ANALOGSERVO_CHERRY_ESC).actuate(low);
    _wait_timeout(100);
    pf::actuators::positional_actuators::get(pf::actuators::positional_actuators::Enum::ANALOGSERVO_CHERRY_ESC).actuate(middle);
    _wait_timeout(100);
    pf::actuators::positional_actuators::get(pf::actuators::positional_actuators::Enum::ANALOGSERVO_CHERRY_ESC).actuate(high);
    _wait_timeout(100);
    pf::actuators::positional_actuators::get(pf::actuators::positional_actuators::Enum::ANALOGSERVO_CHERRY_ESC).actuate(max);
    _wait_timeout(5000);
    pf::actuators::positional_actuators::get(pf::actuators::positional_actuators::Enum::ANALOGSERVO_CHERRY_ESC).actuate(off);

    return EXIT_SUCCESS;
}

int _cmd_arm_pump_left_test(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    pf::actuators::positional_actuators::get(pf::actuators::positional_actuators::Enum::LXMOTOR_LEFT_ARM_LIFT).actuate_timeout(100, 50);
    _wait_timeout(5000);
    pf::actuators::pumps::get(pf::actuators::pumps::Enum::LEFT_ARM_PUMP).activate();
    pf::actuators::positional_actuators::get(pf::actuators::positional_actuators::Enum::LXMOTOR_LEFT_ARM_LIFT).actuate_timeout(-100, 55);
    _wait_timeout(8000);
    pf::actuators::pumps::get(pf::actuators::pumps::Enum::LEFT_ARM_PUMP).deactivate();

    return EXIT_SUCCESS;
}

static void _shell_enter_callback(void)
{
}

static cogip::shell::Menu actuators_menu = { "Actuators actions", "actuators", &cogip::shell::root_menu(), _shell_enter_callback };
static cogip::shell::Command cmd_0 = { "lift", "Test central lift", _cmd_central_lift_test };
static cogip::shell::Command cmd_1 = { "rpump", "Test right arm & pump", _cmd_arm_pump_right_test };
static cogip::shell::Command cmd_2 = { "lpump", "Test left arm & pump", _cmd_arm_pump_left_test };
static cogip::shell::Command cmd_3 = { "esc", "Test ESC", _cmd_esc_test };
static cogip::shell::Command cmd_4 = { "esc_arm_open", "Test ESC open arm", _cmd_esc_arm_open_test };
static cogip::shell::Command cmd_5 = { "esc_arm_close", "Test ESC close arm", _cmd_esc_arm_close_test };

void shell_init()
{
    actuators_menu.push_back(&cmd_0);
    actuators_menu.push_back(&cmd_1);
    actuators_menu.push_back(&cmd_2);
    actuators_menu.push_back(&cmd_3);
    actuators_menu.push_back(&cmd_4);
    actuators_menu.push_back(&cmd_5);
}

} // namespace actuators
} // namespace app
} // namespace cogip
