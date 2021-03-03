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

    if (shell_path_index < path->nb_pose - 1)
        shell_path_index++;
    path_set_current_pose_idx(path, shell_path_index);

    return EXIT_SUCCESS;
}

static int pln_cmd_go_previous_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    path_t *path = pf_get_path();

    if (shell_path_index > 0)
        shell_path_index--;
    path_set_current_pose_idx(path, shell_path_index);

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

static int pln_cmd_select_next_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    path_t *path = pf_get_path();

    if (shell_path_index < path->nb_pose - 1)
        shell_path_index++;

    return EXIT_SUCCESS;
}

static int pln_cmd_select_previous_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if (shell_path_index > 0)
        shell_path_index--;

    return EXIT_SUCCESS;
}

static int pln_cmd_launch_action_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    path_t *path = pf_get_path();
    func_cb_t cb = path_get_pose_at_idx(path, shell_path_index)->act;;

    if(cb != NULL) {
        puts("Launch callback!");
        (*cb)();
    }

    return EXIT_SUCCESS;
}

/* Init shell commands */
void pln_shell_init(void)
{
    shell_menu_t planners_menu;

    menu_init(&planners_menu, "Planners menu");

    const shell_command_t planners_menu_commands[] = {
        { "n", "Go to next position", pln_cmd_go_next_cb },
        { "p", "Go to previous position", pln_cmd_go_previous_cb },
        { "s", "Go back to start position", pln_cmd_go_start_cb },
        { "N", "Select next position", pln_cmd_select_next_cb },
        { "P", "Select previous position", pln_cmd_select_previous_cb },
        { "a", "Launch action", pln_cmd_launch_action_cb },
        menu_cmd_null,
    };

    menu_add_list(&planners_menu, planners_menu_commands);
}
