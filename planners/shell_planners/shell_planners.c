#include <stdio.h>

/* Standard includes */
#include <stdlib.h>

/* RIOT includes */
#include "fmt.h"
#include "xtimer.h"

/* Project includes */
#include "planner.h"
#include "platform.h"
#include "shell_planners.h"

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
    const path_pose_t *current_path_pos = path_get_current_path_pos(path);

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
    shell_menu_t menu = menu_init("Planners menu", "pln_menu", menu_root, pln_menu_enter);

    const shell_command_t planners_menu_commands[] = {
        { "n", "Go to next position", pln_cmd_go_next_cb },
        { "p", "Go to previous position", pln_cmd_go_previous_cb },
        { "s", "Go back to start position", pln_cmd_go_start_cb },
        { "a", "Launch action", pln_cmd_launch_action_cb },
        { "P", "Play", pln_cmd_play_cb },
        { "S", "Stop", pln_cmd_stop_cb },
        MENU_NULL_CMD,
    };

    menu_add_list(menu, planners_menu_commands);
}
