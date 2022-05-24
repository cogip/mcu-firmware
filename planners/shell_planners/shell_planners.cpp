#include "shell_planners.hpp"

/* Standard includes */
#include <cstdlib>
#include <cstdio>

/* RIOT includes */
#include "fmt.h"
#include "xtimer.h"
#include "riot/chrono.hpp"
#include "riot/thread.hpp"

/* Project includes */
#include "shell_menu/shell_menu.hpp"
#include "app_path.hpp"
#include "app_planner.hpp"
#include "platform.hpp"

static cogip::shell::Menu *planner_menu = nullptr;

/* Shell command array */
static uint8_t shell_path_index;

static int pln_cmd_go_next_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    cogip::app::app_get_path().next();

    return EXIT_SUCCESS;
}

static int pln_cmd_go_previous_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    cogip::app::app_get_path()--;

    return EXIT_SUCCESS;
}

static int pln_cmd_go_start_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    cogip::app::app_get_path().reset_current_pose_index();

    return EXIT_SUCCESS;
}

static int pln_cmd_launch_action_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    const cogip::path::Pose *current_path_pos = cogip::app::app_get_path().current_pose();

    puts("Launch callback!");
    current_path_pos->act();

    return EXIT_SUCCESS;
}

static int pln_cmd_play_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    cogip::app::app_planner_get()->set_allow_change_path_pose(true);

    return EXIT_SUCCESS;
}

static int pln_cmd_stop_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    cogip::app::app_planner_get()->set_allow_change_path_pose(false);

    return EXIT_SUCCESS;
}

void pln_menu_enter(void)
{
    shell_path_index = 0;

    cogip::app::app_planner_reset();
    // Wait for old planner thread to end if any
    riot::this_thread::sleep_for(std::chrono::milliseconds(500));
    cogip::app::app_planner_get()->start_thread();
    cogip::app::app_planner_get()->set_allow_change_path_pose(false);
    cogip::app::app_planner_get()->start();
}

/* Init shell commands */
void pln_shell_init()
{
    /* Planners menu and commands */
    if (! planner_menu) {
        planner_menu = new cogip::shell::Menu(
            "Planners menu", "pln_menu", &cogip::shell::root_menu, pln_menu_enter);

        planner_menu->push_back(new cogip::shell::Command(
            "n", "Go to next position", pln_cmd_go_next_cb));
        planner_menu->push_back(new cogip::shell::Command(
            "p", "Go to previous position", pln_cmd_go_previous_cb));
        planner_menu->push_back(new cogip::shell::Command(
            "s", "Go back to start position", pln_cmd_go_start_cb));
        planner_menu->push_back(new cogip::shell::Command(
            "a", "Launch action", pln_cmd_launch_action_cb));
        planner_menu->push_back(new cogip::shell::Command(
            "P", "Play", pln_cmd_play_cb));
        planner_menu->push_back(new cogip::shell::Command(
            "S", "Stop", pln_cmd_stop_cb));
    }
}
