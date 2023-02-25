#include "app_arm.hpp"

#include "shell_menu/shell_menu.hpp"

namespace cogip {
namespace app {
namespace arms {

static int _cmd_gripping_statuette(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    central_arm_gripping_statuette();
    return EXIT_SUCCESS;
}

static int _cmd_gripping_statuette_up(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    central_arm_gripping_statuette_up();
    return EXIT_SUCCESS;
}

static int _cmd_releasing_statuette(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    central_arm_releasing_statuette();
    return EXIT_SUCCESS;
}

static int _cmd_gripping_replica(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    central_arm_gripping_replica();
    return EXIT_SUCCESS;
}

static int _cmd_gripping_replica_up(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    central_arm_gripping_replica_up();
    return EXIT_SUCCESS;
}

static int _cmd_releasing_replica(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    central_arm_releasing_replica();
    return EXIT_SUCCESS;
}

static int _cmd_folded(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    central_arm_folded();
    return EXIT_SUCCESS;
}

static int _cmd_central_arm_gripping_prepare(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    central_arm_gripping_prepare();
    return EXIT_SUCCESS;
}

static int _cmd_central_arm_gripping(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    central_arm_gripping();
    return EXIT_SUCCESS;
}

static int _cmd_central_arm_gripping_full(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    central_arm_gripping_prepare();
    central_arm_gripping();
    return EXIT_SUCCESS;
}

static int _cmd_left_grip_to_central(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    left_arm_gripping();
    left_arm_giving();
    central_arm_taking_left();
    left_arm_folded();
    central_arm_gripping_statuette_up();
    return EXIT_SUCCESS;
}

static int _cmd_central_drop_gallery_low_drop(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    central_drop_gallery_low_prepare();
    return EXIT_SUCCESS;
}

static int _cmd_central_drop_gallery_low_release(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    central_drop_gallery_low_release();
    return EXIT_SUCCESS;
}

static int _cmd_central_drop_gallery_high_drop(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    central_drop_gallery_high_prepare();
    return EXIT_SUCCESS;
}

static int _cmd_central_drop_gallery_high_release(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    central_drop_gallery_high_release();
    return EXIT_SUCCESS;
}

static void _shell_enter_callback(void)
{
}

static cogip::shell::Menu arm_menu = { "Arm actions", "arm", &cogip::shell::root_menu(), _shell_enter_callback };
static cogip::shell::Command cmd_1 = { "arm1", "Gripping statuette", _cmd_gripping_statuette };
static cogip::shell::Command cmd_2 = { "arm2", "Gripping statuette up", _cmd_gripping_statuette_up };
static cogip::shell::Command cmd_3 = { "arm3", "Releasing statuette", _cmd_releasing_statuette };
static cogip::shell::Command cmd_4 = { "arm4", "Gripping replica", _cmd_gripping_replica };
static cogip::shell::Command cmd_6 = { "arm6", "Gripping replica up", _cmd_gripping_replica_up };
static cogip::shell::Command cmd_7 = { "arm7", "Releasing replica", _cmd_releasing_replica };
static cogip::shell::Command cmd_8 = { "arm8", "Folded", _cmd_folded };
static cogip::shell::Command cmd_9 = { "arm9", "Central arm gripping prepare", _cmd_central_arm_gripping_prepare };
static cogip::shell::Command cmd_10 = { "arm10", "Central arm gripping", _cmd_central_arm_gripping };
static cogip::shell::Command cmd_11 = { "arm11", "Central arm gripping full", _cmd_central_arm_gripping_full };
static cogip::shell::Command cmd_12 = { "arm12", "Left grip to central", _cmd_left_grip_to_central };
static cogip::shell::Command cmd_13 = { "arm13", "Central drop gallery low drop", _cmd_central_drop_gallery_low_drop };
static cogip::shell::Command cmd_14 = { "arm14", "Central drop gallery low release", _cmd_central_drop_gallery_low_release };
static cogip::shell::Command cmd_15 = { "arm15", "Central drop gallery high drop", _cmd_central_drop_gallery_high_drop };
static cogip::shell::Command cmd_16 = { "arm16", "Central drop gallery high release", _cmd_central_drop_gallery_high_release };

void shell_init()
{
    arm_menu.push_back(&cmd_1);
    arm_menu.push_back(&cmd_2);
    arm_menu.push_back(&cmd_3);
    arm_menu.push_back(&cmd_4);
    arm_menu.push_back(&cmd_6);
    arm_menu.push_back(&cmd_7);
    arm_menu.push_back(&cmd_8);
    arm_menu.push_back(&cmd_9);
    arm_menu.push_back(&cmd_10);
    arm_menu.push_back(&cmd_11);
    arm_menu.push_back(&cmd_12);
    arm_menu.push_back(&cmd_13);
    arm_menu.push_back(&cmd_14);
    arm_menu.push_back(&cmd_15);
    arm_menu.push_back(&cmd_16);
}

} // namespace arms
} // namespace app
} // namespace cogip
