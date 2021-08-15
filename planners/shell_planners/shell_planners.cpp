#include "shell_planners.hpp"
#include "planner.hpp"

/* Standard includes */
#include <cstdlib>
#include <cstdio>

/* RIOT includes */
#include "fmt.h"
#include "xtimer.h"

/* Project includes */
#include "shell_menu/shell_menu.hpp"
#include "platform.hpp"

/* Shell command array */
static uint8_t shell_path_index;

static int pln_cmd_go_next_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    path_t *path = pf_get_path();

    path_increment_current_pose_idx(path);

    return EXIT_SUCCESS;
}

static int pln_cmd_go_previous_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    path_t *path = pf_get_path();

    path_decrement_current_pose_idx(path);

    return EXIT_SUCCESS;
}

static int pln_cmd_go_start_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    path_t *path = pf_get_path();

    path_reset_current_pose_idx(path);

    return EXIT_SUCCESS;
}

static int pln_cmd_launch_action_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    path_t *path = pf_get_path();
    const path_pose_t *current_path_pos = path_get_current_pose(path);

    if (current_path_pos->act) {
        puts("Launch callback!");
        (*(current_path_pos->act))();
    }

    return EXIT_SUCCESS;
}

static int pln_cmd_play_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    pln_set_allow_change_path_pose(TRUE);

    return EXIT_SUCCESS;
}

static int pln_cmd_stop_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    pln_set_allow_change_path_pose(FALSE);

    return EXIT_SUCCESS;
}

void pln_menu_enter(void)
{
    ctrl_t *ctrl = pf_get_ctrl();

    shell_path_index = 0;

    pln_set_allow_change_path_pose(FALSE);

    pln_start(ctrl);
}

/* Init shell commands */
void pln_shell_init(void)
{
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
