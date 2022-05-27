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

static int _cmd_gripping_replica(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_central_arm_gripping_replica();
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

static int _cmd_central_arm_gripping_prepare(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_central_arm_gripping_prepare();
    return EXIT_SUCCESS;
}

static int _cmd_central_arm_gripping(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_central_arm_gripping();
    return EXIT_SUCCESS;
}

static int _cmd_central_arm_gripping_full(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_central_arm_gripping_prepare();
    app_central_arm_gripping();
    return EXIT_SUCCESS;
}

static int _cmd_left_grip_to_central(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_left_arm_gripping();
    app_left_arm_giving();
    app_central_arm_taking_left();
    app_left_arm_folded();
    app_central_arm_gripping_statuette_up();
    return EXIT_SUCCESS;
}

static int _cmd_central_drop_gallery_low_drop(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_central_drop_gallery_low_prepare();
    return EXIT_SUCCESS;
}

static int _cmd_central_drop_gallery_low_release(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_central_drop_gallery_low_release();
    return EXIT_SUCCESS;
}

static int _cmd_central_drop_gallery_high_drop(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_central_drop_gallery_high_prepare();
    return EXIT_SUCCESS;
}

static int _cmd_central_drop_gallery_high_release(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_central_drop_gallery_high_release();
    return EXIT_SUCCESS;
}

void arm_shell_enter_callback(void)
{
}

void app_arm_shell_init()
{
    cogip::shell::Menu *menu = new cogip::shell::Menu(
        "Arm actions", "arm", &cogip::shell::root_menu, arm_shell_enter_callback);

    menu->push_back(
        new cogip::shell::Command("arm8", "Folded", _cmd_folded));

    menu->push_back(
        new cogip::shell::Command("arm9", "Central arm gripping prepare", _cmd_central_arm_gripping_prepare));

    menu->push_back(
        new cogip::shell::Command("arm10", "Central arm gripping", _cmd_central_arm_gripping));

    menu->push_back(
        new cogip::shell::Command("arm11", "Central arm gripping full", _cmd_central_arm_gripping_full));

    menu->push_back(
        new cogip::shell::Command("arm12", "Left grip to central", _cmd_left_grip_to_central));

    menu->push_back(
        new cogip::shell::Command("arm13", "Central drop gallery low drop", _cmd_central_drop_gallery_low_drop));

    menu->push_back(
        new cogip::shell::Command("arm14", "Central drop gallery low release", _cmd_central_drop_gallery_low_release));

    menu->push_back(
        new cogip::shell::Command("arm15", "Central drop gallery high drop", _cmd_central_drop_gallery_high_drop));

    menu->push_back(
        new cogip::shell::Command("arm16", "Central drop gallery high release", _cmd_central_drop_gallery_high_release));

    menu->push_back(
        new cogip::shell::Command("arm1", "Gripping statuette", _cmd_gripping_statuette));

    menu->push_back(
        new cogip::shell::Command("arm2", "Gripping statuette up", _cmd_gripping_statuette_up));

    menu->push_back(
        new cogip::shell::Command("arm3", "Releasing statuette", _cmd_releasing_statuette));

    menu->push_back(
        new cogip::shell::Command("arm4", "Gripping replica", _cmd_gripping_replica));

    menu->push_back(
        new cogip::shell::Command("arm6", "Gripping replica up", _cmd_gripping_replica_up));

    menu->push_back(
        new cogip::shell::Command("arm7", "Releasing replica", _cmd_releasing_replica));

    menu->push_back(
        new cogip::shell::Command("arm8", "Folded", _cmd_folded));
}

} // namespace app

} // namespace cogip
