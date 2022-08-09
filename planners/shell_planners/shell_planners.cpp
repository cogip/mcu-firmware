#include "shell_planners.hpp"

/* Standard includes */
#include <cstdlib>
#include <cstdio>

/* RIOT includes */
#include "fmt.h"

/* Project includes */
#include "shell_menu/shell_menu.hpp"
#include "app.hpp"
#include "platform.hpp"

static cogip::planners::Planner *planner = nullptr;
static cogip::shell::Menu *planner_menu = nullptr;

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

static cogip::shell::Menu _menu_planner = { "Planners menu", "pln_menu", &cogip::shell::root_menu(), pln_menu_enter };
static cogip::shell::Command _cmd_n = { "n", "Go to next position", pln_cmd_go_next_cb };
static cogip::shell::Command _cmd_p = { "p", "Go to previous position", pln_cmd_go_previous_cb };
static cogip::shell::Command _cmd_s = { "s", "Go back to start position", pln_cmd_go_start_cb };
static cogip::shell::Command _cmd_a = { "a", "Launch action", pln_cmd_launch_action_cb };
static cogip::shell::Command _cmd_P = { "P", "Play", pln_cmd_play_cb };
static cogip::shell::Command _cmd_S = { "S", "Stop", pln_cmd_stop_cb };

/* Init shell commands */
void pln_shell_init(cogip::planners::Planner *pln)
{
    planner = pln;

    /* Planners menu and commands */
    if (! planner_menu) {
        _menu_planner.push_back(&_cmd_n);
        _menu_planner.push_back(&_cmd_p);
        _menu_planner.push_back(&_cmd_s);
        _menu_planner.push_back(&_cmd_a);
        _menu_planner.push_back(&_cmd_P);
        _menu_planner.push_back(&_cmd_S);
    }
}
