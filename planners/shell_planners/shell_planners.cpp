#include "shell_planners.hpp"

/* Standard includes */
#include <cstdlib>
#include <cstdio>

/* RIOT includes */
#include "fmt.h"
#include "xtimer.h"

/* Project includes */
#include "shell_menu/shell_menu.hpp"
#include "app.hpp"
#include "platform.hpp"

static cogip::planners::Planner *planner = nullptr;

/* Shell command array */
static uint8_t shell_path_index;

static int pln_cmd_go_next_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    app_get_path().next();

    return EXIT_SUCCESS;
}

static int pln_cmd_go_previous_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    app_get_path()--;

    return EXIT_SUCCESS;
}

static int pln_cmd_go_start_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    app_get_path().reset_current_pose_index();

    return EXIT_SUCCESS;
}

static int pln_cmd_launch_action_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    const cogip::path::Pose *current_path_pos = app_get_path().current_pose();

    puts("Launch callback!");
    current_path_pos->act();

    return EXIT_SUCCESS;
}

static int pln_cmd_play_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    planner->set_allow_change_path_pose(true);

    return EXIT_SUCCESS;
}

static int pln_cmd_stop_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    planner->set_allow_change_path_pose(false);

    return EXIT_SUCCESS;
}

void pln_menu_enter(void)
{
    shell_path_index = 0;

    planner->set_allow_change_path_pose(false);

    planner->start();
}

/* Init shell commands */
void pln_shell_init(cogip::planners::Planner *pln)
{
    planner = pln;

    /* Planners menu and commands */
    cogip::shell::Menu *menu = new cogip::shell::Menu(
        "Planners menu", "pln_menu", &cogip::shell::root_menu, pln_menu_enter);

    menu->push_back(new cogip::shell::Command(
        "n", "Go to next position", pln_cmd_go_next_cb));
    menu->push_back(new cogip::shell::Command(
        "p", "Go to previous position", pln_cmd_go_previous_cb));
    menu->push_back(new cogip::shell::Command(
        "s", "Go back to start position", pln_cmd_go_start_cb));
    menu->push_back(new cogip::shell::Command(
        "a", "Launch action", pln_cmd_launch_action_cb));
    menu->push_back(new cogip::shell::Command(
        "P", "Play", pln_cmd_play_cb));
    menu->push_back(new cogip::shell::Command(
        "S", "Stop", pln_cmd_stop_cb));
}
