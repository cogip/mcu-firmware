#include <stdio.h>

/* Standard includes */
#include <stdlib.h>

/* RIOT includes */
#include "fmt.h"
#include "xtimer.h"

/* Project includes */
#include "planner.h"
#include "platform.h"
#include "calibration/calib_planner.h"
#include "calibration/calib_platform.h"

/* Shell command array */
static shell_command_linked_t pln_shell_commands;
static const char *pln_name = "planner";

static uint8_t calib_path_index;

static int pln_cmd_go_next_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    path_t *path = pf_get_path();

    if (calib_path_index < path->nb_pose - 1)
        calib_path_index++;
    path_set_current_pose_idx(path, calib_path_index);

    return EXIT_SUCCESS;
}

static int pln_cmd_go_previous_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    path_t *path = pf_get_path();

    if (calib_path_index > 0)
        calib_path_index--;
    path_set_current_pose_idx(path, calib_path_index);

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

    if (calib_path_index < path->nb_pose - 1)
        calib_path_index++;

    return EXIT_SUCCESS;
}

static int pln_cmd_select_previous_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if (calib_path_index > 0)
        calib_path_index--;

    return EXIT_SUCCESS;
}

static int pln_cmd_launch_action_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    path_t *path = pf_get_path();
    func_cb_t cb = path_get_pose_at_idx(path, calib_path_index)->act;;

    if(cb != NULL) {
        puts("Launch callback!");
        (*cb)();
    }

    return EXIT_SUCCESS;
}

/* Speed calibration command */
static int pln_calib_cmd(int argc, char **argv)
{
    (void)argv;
    int ret = 0;

    calib_path_index = 0;

    ctrl_t* ctrl = pf_get_ctrl();

    /* Check arguments */
    if (argc > 1) {
        puts("Bad number of arguments!");
        ret = -1;
        goto pln_calib_cmd_err;
    }

    pln_set_allow_change_path_pose(FALSE);

    pln_start(ctrl);

    pf_init_shell_commands(&pln_shell_commands, pln_name);

    shell_command_t pln_cmd_go_next = {
        "n", "Go to next position",
        pln_cmd_go_next_cb
    };
    pf_add_shell_command(&pln_shell_commands, &pln_cmd_go_next);

    shell_command_t pln_cmd_go_previous = {
        "p", "Go to previous position",
        pln_cmd_go_previous_cb
    };
    pf_add_shell_command(&pln_shell_commands, &pln_cmd_go_previous);

    shell_command_t pln_cmd_go_start = {
        "s", "Go back to start position",
        pln_cmd_go_start_cb
    };
    pf_add_shell_command(&pln_shell_commands, &pln_cmd_go_start);

    shell_command_t pln_cmd_select_next = {
        "N", "Select next position",
        pln_cmd_select_next_cb
    };
    pf_add_shell_command(&pln_shell_commands, &pln_cmd_select_next);

    shell_command_t pln_cmd_select_previous = {
        "P", "Select previous position",
        pln_cmd_select_previous_cb
    };
    pf_add_shell_command(&pln_shell_commands, &pln_cmd_select_previous);

    shell_command_t pln_cmd_launch_action = {
        "a", "Launch action",
        pln_cmd_launch_action_cb
    };
    pf_add_shell_command(&pln_shell_commands, &pln_cmd_launch_action);

    pf_add_shell_command(&pln_shell_commands, &cmd_exit_shell);

    /* Push new menu */
    pf_push_shell_commands(&pln_shell_commands);

pln_calib_cmd_err:
    return ret;
}

/* Init calibration commands */
void pln_calib_init(void)
{
    /* Add planner calibration command */
    shell_command_t cmd_calib_pln = {
        "pc", "Planner calibration",
        pln_calib_cmd
    };
    pf_add_shell_command(&pf_shell_commands, &cmd_calib_pln);
}
