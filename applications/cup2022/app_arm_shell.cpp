#include "app_arm.hpp"

#include "shell_menu/shell_menu.hpp"

namespace cogip {

namespace app {

static int _cmd_gripping_statuette(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_central_arm_gripping_statuette();
    return EXIT_SUCCESS;
}

static int _cmd_gripping_statuette_up(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_central_arm_gripping_statuette_up();
    return EXIT_SUCCESS;
}

static int _cmd_releasing_statuette(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_central_arm_releasing_statuette();
    return EXIT_SUCCESS;
}

static int _cmd_gripping_replica_yellow(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_central_arm_gripping_replica_yellow();
    return EXIT_SUCCESS;
}

static int _cmd_gripping_replica_purple(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_central_arm_gripping_replica_purple();
    return EXIT_SUCCESS;
}

static int _cmd_gripping_replica_up(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_central_arm_gripping_replica_up();
    return EXIT_SUCCESS;
}

static int _cmd_releasing_replica(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_central_arm_releasing_replica();
    return EXIT_SUCCESS;
}

static int _cmd_folded(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_central_arm_folded();
    return EXIT_SUCCESS;
}

void arm_shell_enter_callback(void)
{
}

void app_arm_shell_init()
{
    /*cogip::shell::Menu *menu = new cogip::shell::Menu(
        "Arm actions", "arm", &cogip::shell::root_menu, arm_shell_enter_callback);

    menu->push_back(
        new cogip::shell::Command("arm1", "Gripping statuette", _cmd_gripping_statuette));

    menu->push_back(
        new cogip::shell::Command("arm2", "Gripping statuette up", _cmd_gripping_statuette_up));

    menu->push_back(
        new cogip::shell::Command("arm3", "Releasing statuette", _cmd_releasing_statuette));

    menu->push_back(
        new cogip::shell::Command("arm4", "Gripping replica yellow", _cmd_gripping_replica_yellow));

    menu->push_back(
        new cogip::shell::Command("arm5", "Gripping replica purple", _cmd_gripping_replica_purple));

    menu->push_back(
        new cogip::shell::Command("arm6", "Gripping replica up", _cmd_gripping_replica_up));

    menu->push_back(
        new cogip::shell::Command("arm7", "Releasing replica", _cmd_releasing_replica));

    menu->push_back(
        new cogip::shell::Command("arm8", "Folded", _cmd_folded));*/
}

} // namespace app

} // namespace cogip
